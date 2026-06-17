#include "mcf/cob_mcf_router.hh"

#include "mcf/mcf_bbox.hh"
#include "mcf/mcf_conflict_graph.hh"
#include "mcf/mcf_gurobi_log_io.hh"
#include "mcf/mcf_gurobi_params.hh"
#include "mcf/mcf_gurobi_thread_budget.hh"
#include "mcf/mcf_resource_usage_io.hh"
#include "mcf/mcf_simple_tree_refine.hh"
#include "precompute/tob_path_precompute.hh"
#include "mcf/mcf_graph.hh"
#include "mcf/mcf_hw_map.hh"

#include "ilp_allocation/gurobi_model_stats.hh"
#include "circuit/basedie.hh"
#include "debug/debug.hh"
#include "hardware/cob/cobunit.hh"
#include "hardware/track/trackcoord.hh"

#include "gurobi_c++.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <format>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/resource.h>
#include <tuple>
#include <utility>
#include <vector>

namespace PR_tool {

auto classify_gurobi_status(const int status) -> McfSolutionClass {
    switch (status) {
        case GRB_OPTIMAL:
            return McfSolutionClass::Optimal;
        case GRB_SUBOPTIMAL:
            return McfSolutionClass::Suboptimal;
        case GRB_TIME_LIMIT:
            return McfSolutionClass::TimeLimit;
        default:
            return McfSolutionClass::Failed;
    }
}

auto solution_class_name(const McfSolutionClass c) -> std::String {
    switch (c) {
        case McfSolutionClass::Optimal:    return "Optimal";
        case McfSolutionClass::Suboptimal: return "Suboptimal";
        case McfSolutionClass::TimeLimit:  return "TimeLimit";
        case McfSolutionClass::Failed:     return "Failed";
        case McfSolutionClass::Skipped:    return "Skipped";
    }
    return "Unknown";
}

auto stage_result_ok(const McfSolutionClass c) -> bool {
    switch (c) {
        case McfSolutionClass::Optimal:
        case McfSolutionClass::Suboptimal:
        case McfSolutionClass::Skipped:
            return true;
        default:
            return false;
    }
}

auto stage_result_usable(const McfSolutionClass c) -> bool {
    return c == McfSolutionClass::Optimal || c == McfSolutionClass::Suboptimal;
}

namespace {

using namespace mcf;

using NodeMeta = McfNodeMeta;
using Arc = McfArc;
using NodeKey = McfNodeKey;
using GlobalGraph = McfGlobalGraph;

constexpr double kGurobiInf = GRB_INFINITY;

struct PreparedCommodity {
    std::String label;
    std::String origin_name;
    std::String origin_uid;
    std::size_t record_index{0};
    std::size_t record_id{0};
    std::Vector<std::size_t> record_indices {};
    std::size_t cob_unit{0};
    std::size_t start_track{0};
    std::size_t end_track{0};
    int src{-1};
    int snk{-1};
    int demand{1};
    bool is_bus{false};
    std::String bus_key;
    bool is_refined_segment{false};
    std::optional<McfCommodityBBox> bbox_override {};
    std::Vector<int> guide_path {};
    std::String synthetic_label {};
    /// Bbox lookup index into bbox_ctx.per_commodity (defaults to prepare order).
    std::size_t bbox_global_commodity_id{std::numeric_limits<std::size_t>::max()};
};

struct McfConstraintMeta {
    std::String kind;
    std::String detail;
    std::String bus_key {};
    std::size_t record_index{std::numeric_limits<std::size_t>::max()};
    int simple_unit{-1};
    std::String simple_origin_key {};
};

struct StageSolveTiming {
    int model_build_ms{0};
    int matrix_diag_ms{0};
    int gurobi_optimize_ms{0};
    int compute_iis_ms{0};
    int extract_path_ms{0};
    int solve_ms{0};
};

struct StageSolveResult {
    bool ok{false};
    McfSolutionClass solution_class{McfSolutionClass::Failed};
    std::String message;
    std::String stage_name;
    double objective{0.0};
    int solve_ms{0};
    StageSolveTiming timing;
    int model_status{0};
    std::map<std::pair<int, int>, int> used_edges;
    std::map<int, int> used_nodes;
    std::array<std::map<std::pair<int, int>, int>, 16> unit_used_edges {};
    std::array<std::map<int, int>, 16> unit_used_nodes {};
    std::Vector<McfPathInfo> paths;
    std::Vector<McfConstraintMeta> infeasibility_hints;
    std::Vector<std::String> failed_bus_keys;
    std::Vector<std::size_t> failed_record_indices;
    std::Vector<McfSimpleOriginGroupKey> failed_simple_origin_groups;
    bool bus_failure_unlocalized{false};
};

auto stage_solve_elapsed_ms(const std::chrono::steady_clock::time_point begin) -> int {
    const auto end = std::chrono::steady_clock::now();
    return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
}

auto elapsed_ms_between(
    const std::chrono::steady_clock::time_point begin,
    const std::chrono::steady_clock::time_point end
) -> int {
    return static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
}

auto merge_gurobi_stage_timing(const int cpp_model_build_ms, const StageSolveTiming& grb) -> StageSolveTiming {
    auto timing = StageSolveTiming {};
    timing.model_build_ms = cpp_model_build_ms + grb.model_build_ms;
    timing.matrix_diag_ms = grb.matrix_diag_ms;
    timing.gurobi_optimize_ms = grb.gurobi_optimize_ms;
    timing.compute_iis_ms = grb.compute_iis_ms;
    return timing;
}

auto gurobi_status_name(int status) -> std::String;

auto apply_stage_solution_class(StageSolveResult& out, const McfSolutionClass cls) -> void {
    out.solution_class = cls;
    out.ok = stage_result_ok(cls);
}

auto finish_stage_solve_result(
    StageSolveResult& out,
    const std::chrono::steady_clock::time_point begin,
    StageSolveTiming timing
) -> StageSolveResult {
    timing.solve_ms = stage_solve_elapsed_ms(begin);
    out.solve_ms = timing.solve_ms;
    out.timing = timing;
    if (!out.stage_name.empty()) {
        debug::info_fmt(
            "{}: model_status={}({}) solution_class={} ok={} objective={:.0f} solve_ms={} "
            "model_build_ms={} matrix_diag_ms={} gurobi_optimize_ms={} compute_iis_ms={} extract_path_ms={}",
            out.stage_name,
            gurobi_status_name(out.model_status),
            out.model_status,
            solution_class_name(out.solution_class),
            out.ok,
            out.objective,
            out.timing.solve_ms,
            out.timing.model_build_ms,
            out.timing.matrix_diag_ms,
            out.timing.gurobi_optimize_ms,
            out.timing.compute_iis_ms,
            out.timing.extract_path_ms);
    }
    return out;
}

auto finish_stage_solve_early(
    StageSolveResult& out,
    const std::chrono::steady_clock::time_point begin
) -> StageSolveResult {
    return finish_stage_solve_result(out, begin, StageSolveTiming {});
}

auto log_skipped_simple_unit_stage(StageSolveResult& out, const std::size_t unit) -> void {
    out.stage_name = std::format("SimpleMCF_unit{}", unit);
    const auto begin = std::chrono::steady_clock::now();
    finish_stage_solve_early(out, begin);
}

struct StageWarmStart {
    std::map<std::size_t, std::Vector<int>> nodes_by_record_id;
};

struct StageIncumbentWarmStart {
    std::map<int, double> warm_values_by_col;
};

struct StageRefineIncumbentSeed {
    std::map<std::pair<int, int>, int> used_edges;
    std::map<int, int> used_nodes;
};

struct ArcVar {
    int k{0};
    int a{0};
};

struct OVar {
    int k{0};
    int node{0};
};

struct OriginEdgeVar {
    int h{0};
    int u{0};
    int v{0};
};

struct OriginOVar {
    int h{0};
    int node{0};
};

struct McfOriginGroup {
    std::String origin_key;
    std::size_t cob_unit{0};
    std::Vector<int> commodity_local_indices;
    int origin_group_id{0};
    bool is_multi_fanout{false};
};

struct GurobiMcfSolveResult {
    bool ok{false};
    McfSolutionClass solution_class{McfSolutionClass::Failed};
    std::String message;
    int model_status{0};
    double objective{0.0};
    std::vector<double> col_value;
    std::Vector<McfConstraintMeta> iis_rows;
    StageSolveTiming gurobi_timing;
};

constexpr int kMaxIisLogPerKind = 20;
constexpr std::size_t kInvalidRecordIndex = std::numeric_limits<std::size_t>::max();
/// SimpleMCF min-Sum-x: cost multiplier for x^H_e on warm-start-used physical edges (ninth-edition symmetry break).
constexpr double kSimpleMcfWarmStartUsedEdgeCost = 1.0;

auto normalized_edge_key(int u, int v) -> std::pair<int, int>;
auto node_text(const GlobalGraph& g, const int node) -> std::String;
auto fmt_join_parts(const std::Vector<std::String>& parts) -> std::String;

template <typename T>
auto append_unique(std::Vector<T>& values, const T& value) -> void {
    if (std::find(values.begin(), values.end(), value) == values.end()) {
        values.push_back(value);
    }
}

auto append_bus_retry_hint(StageSolveResult& out, const std::String& bus_key, const std::size_t record_index) -> void {
    if (!bus_key.empty()) {
        append_unique(out.failed_bus_keys, bus_key);
    }
    if (record_index != kInvalidRecordIndex) {
        append_unique(out.failed_record_indices, record_index);
    }
}

auto append_simple_origin_retry_hint(
    StageSolveResult& out,
    const std::size_t unit,
    const std::String& origin_key
) -> void {
    if (origin_key.empty()) {
        return;
    }
    append_unique(out.failed_simple_origin_groups, McfSimpleOriginGroupKey {unit, origin_key});
}

auto collect_bus_retry_hints_from_iis(StageSolveResult& out, const std::Vector<McfConstraintMeta>& hints) -> void {
    for (const auto& hint : hints) {
        append_bus_retry_hint(out, hint.bus_key, hint.record_index);
    }
    if (out.failed_bus_keys.empty()) {
        out.bus_failure_unlocalized = true;
    }
}

auto gurobi_status_name(const int status) -> std::String {
    switch (status) {
        case GRB_LOADED:       return "LOADED";
        case GRB_OPTIMAL:      return "OPTIMAL";
        case GRB_INFEASIBLE:   return "INFEASIBLE";
        case GRB_INF_OR_UNBD:  return "INF_OR_UNBD";
        case GRB_UNBOUNDED:    return "UNBOUNDED";
        case GRB_CUTOFF:       return "CUTOFF";
        case GRB_ITERATION_LIMIT: return "ITERATION_LIMIT";
        case GRB_NODE_LIMIT:   return "NODE_LIMIT";
        case GRB_TIME_LIMIT:   return "TIME_LIMIT";
        case GRB_SOLUTION_LIMIT: return "SOLUTION_LIMIT";
        case GRB_INTERRUPTED:  return "INTERRUPTED";
        case GRB_NUMERIC:      return "NUMERIC";
        case GRB_SUBOPTIMAL:   return "SUBOPTIMAL";
        default:               return std::format("STATUS_{}", status);
    }
}

auto cob_dir_char(const hardware::COBDirection dir) -> char {
    switch (dir) {
        case hardware::COBDirection::Right: return 'R';
        case hardware::COBDirection::Up:    return 'U';
        case hardware::COBDirection::Down:  return 'D';
        default:                            return 'L';
    }
}

auto build_undirected_arc_index(const GlobalGraph& graph) -> std::map<std::pair<int, int>, std::size_t> {
    auto out = std::map<std::pair<int, int>, std::size_t> {};
    for (std::size_t a = 0; a < graph.arcs.size(); ++a) {
        const auto& arc = graph.arcs[a];
        if (arc.is_virtual) {
            continue;
        }
        const auto key = normalized_edge_key(arc.u, arc.v);
        if (!out.contains(key)) {
            out[key] = a;
        }
    }
    return out;
}

auto arc_cob_row_col(const Arc& arc, const int cols) -> std::pair<int, int> {
    return {arc.cob / cols, arc.cob % cols};
}

auto h_channel_key_from_arc(const Arc& arc, const int cols) -> std::optional<McfHChannelKey> {
    if (arc.is_turn || arc.is_virtual) {
        return std::nullopt;
    }
    const auto [cob_r, cob_c] = arc_cob_row_col(arc, cols);
    const bool lr = (arc.from_dir == hardware::COBDirection::Left && arc.to_dir == hardware::COBDirection::Right)
        || (arc.from_dir == hardware::COBDirection::Right && arc.to_dir == hardware::COBDirection::Left);
    if (!lr) {
        return std::nullopt;
    }
    return McfHChannelKey {cob_r, cob_c};
}

auto v_channel_key_from_arc(const Arc& arc, const int cols) -> std::optional<McfVChannelKey> {
    if (arc.is_turn || arc.is_virtual) {
        return std::nullopt;
    }
    const auto [cob_r, cob_c] = arc_cob_row_col(arc, cols);
    const bool ud = (arc.from_dir == hardware::COBDirection::Up && arc.to_dir == hardware::COBDirection::Down)
        || (arc.from_dir == hardware::COBDirection::Down && arc.to_dir == hardware::COBDirection::Up);
    if (!ud) {
        return std::nullopt;
    }
    return McfVChannelKey {cob_r, cob_c};
}

auto describe_arc_resource(
    const GlobalGraph& graph,
    const Arc& arc,
    const int cols
) -> std::String {
    const auto [cob_r, cob_c] = arc_cob_row_col(arc, cols);
    if (arc.is_turn) {
        return std::format(
            "undir U{} switch COB({},{}) {}->{} track_in={}",
            arc.unit,
            cob_r,
            cob_c,
            cob_dir_char(arc.from_dir),
            cob_dir_char(arc.to_dir),
            arc.track_in);
    }
    if (const auto h = h_channel_key_from_arc(arc, cols)) {
        return std::format(
            "undir U{} channel H COB({},{})-COB({},{}) track={}",
            arc.unit,
            h->r,
            h->c,
            h->r,
            h->c + 1,
            arc.track_in);
    }
    if (const auto v = v_channel_key_from_arc(arc, cols)) {
        return std::format(
            "undir U{} channel V COB({},{})-COB({},{}) track={}",
            arc.unit,
            v->r,
            v->c,
            v->r + 1,
            v->c,
            arc.track_in);
    }
    return std::format(
        "undir U{} COB({},{}) {}-{}",
        arc.unit,
        cob_r,
        cob_c,
        node_text(graph, arc.u),
        node_text(graph, arc.v));
}

auto describe_undirected_edge(
    const GlobalGraph& graph,
    const std::map<std::pair<int, int>, std::size_t>& arc_index,
    const int u,
    const int v,
    const int cols
) -> std::String {
    const auto key = normalized_edge_key(u, v);
    const auto it = arc_index.find(key);
    if (it == arc_index.end()) {
        return std::format("{}-{}", node_text(graph, u), node_text(graph, v));
    }
    return describe_arc_resource(graph, graph.arcs[it->second], cols);
}

auto log_mcf_infeasibility_hints(
    const std::String& stage_name,
    const int model_status,
    const std::Vector<McfConstraintMeta>& hints
) -> void {
    debug::error_fmt(
        "MCF infeasibility diagnosis: stage={} status={}({})",
        stage_name,
        gurobi_status_name(model_status),
        model_status);
    if (model_status != GRB_INFEASIBLE) {
        debug::error_fmt("  (IIS not computed: status is not INFEASIBLE)");
        return;
    }
    if (hints.empty()) {
        debug::error_fmt("  (IIS empty or computeIIS failed)");
        return;
    }

    auto kind_counts = std::map<std::String, int> {};
    auto by_kind = std::map<std::String, std::Vector<std::String>> {};
    for (const auto& hint : hints) {
        ++kind_counts[hint.kind];
        by_kind[hint.kind].push_back(hint.detail);
    }
    {
        auto parts = std::Vector<std::String> {};
        for (const auto& [kind, count] : kind_counts) {
            parts.push_back(std::format("{}={}", kind, count));
        }
        debug::error_fmt("  IIS constraint kinds: {}", fmt_join_parts(parts));
    }
    for (const auto& [kind, details] : by_kind) {
        debug::error_fmt("  {}:", kind);
        const auto show = std::min(details.size(), static_cast<std::size_t>(kMaxIisLogPerKind));
        for (std::size_t i = 0; i < show; ++i) {
            debug::error_fmt("    - {}", details[i]);
        }
        if (details.size() > show) {
            debug::error_fmt("    ... and {} more", details.size() - show);
        }
    }
}

auto fmt_join_parts(const std::Vector<std::String>& parts) -> std::String {
    if (parts.empty()) {
        return std::String {};
    }
    auto out = parts.front();
    for (std::size_t i = 1; i < parts.size(); ++i) {
        out += std::format(", {}", parts[i]);
    }
    return out;
}

auto log_mcf_infeasibility_summary(
    const StageSolveResult& bus_res,
    const std::array<StageSolveResult, 16>& simple_results
) -> void {
    if (!bus_res.ok && !bus_res.infeasibility_hints.empty()) {
        log_mcf_infeasibility_hints(bus_res.stage_name, bus_res.model_status, bus_res.infeasibility_hints);
    }
    else if (!bus_res.ok) {
        debug::error_fmt(
            "MCF failure diagnosis: stage={} status={}({}) solution_class={} message={}",
            bus_res.stage_name.empty() ? std::String("BusMCF") : bus_res.stage_name,
            gurobi_status_name(bus_res.model_status),
            bus_res.model_status,
            solution_class_name(bus_res.solution_class),
            bus_res.message);
    }
    for (std::size_t u = 0; u < 16; ++u) {
        if (simple_results[u].ok) {
            continue;
        }
        if (!simple_results[u].infeasibility_hints.empty()) {
            log_mcf_infeasibility_hints(
                simple_results[u].stage_name,
                simple_results[u].model_status,
                simple_results[u].infeasibility_hints);
        }
        else {
            debug::error_fmt(
                "MCF failure diagnosis: stage={} status={}({}) solution_class={} message={}",
                simple_results[u].stage_name.empty() ? std::format("SimpleMCF_unit{}", u) : simple_results[u].stage_name,
                gurobi_status_name(simple_results[u].model_status),
                simple_results[u].model_status,
                solution_class_name(simple_results[u].solution_class),
                simple_results[u].message);
        }
    }
}

auto solve_binary_columns_with_gurobi(
    const std::String& stage_name,
    const std::vector<double>& col_cost,
    const std::vector<double>& col_lo,
    const std::vector<double>& col_up,
    const std::vector<double>& row_lo,
    const std::vector<double>& row_up,
    const std::Vector<std::Vector<std::pair<int, double>>>& col_entries,
    const std::map<int, double>& warm_values_by_col,
    const std::Vector<McfConstraintMeta>* row_meta,
    const GurobiDiagnosticsOptions& diag,
    McfGurobiLogSink* gurobi_sink,
    const McfGurobiSolveMeta& gurobi_meta,
    const McfGurobiSolveParams& gurobi_params
) -> GurobiMcfSolveResult {
    auto out = GurobiMcfSolveResult {};
    std::optional<McfGurobiSolvePaths> log_paths;
    const auto gurobi_begin = std::chrono::steady_clock::now();
    auto timing = StageSolveTiming {};
    try {
        GRBEnv env {true};
        env.set(GRB_IntParam_OutputFlag, 0);
        env.start();

        GRBModel model {env};
        model.set(GRB_StringAttr_ModelName, stage_name);
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
        model.set(GRB_IntParam_OutputFlag, 0);

        auto vars = std::vector<GRBVar> {};
        vars.reserve(col_cost.size());
        for (std::size_t c = 0; c < col_cost.size(); ++c) {
            vars.push_back(model.addVar(col_lo[c], col_up[c], col_cost[c], GRB_BINARY, std::format("x_{}", c)));
        }
        model.update();

        auto row_expr = std::vector<GRBLinExpr>(row_lo.size());
        for (std::size_t c = 0; c < col_entries.size(); ++c) {
            for (const auto& [row, value] : col_entries[c]) {
                row_expr[static_cast<std::size_t>(row)] += value * vars[c];
            }
        }
        for (std::size_t r = 0; r < row_lo.size(); ++r) {
            const auto lo = row_lo[r];
            const auto up = row_up[r];
            if (std::fabs(lo - up) < 1e-9) {
                model.addConstr(row_expr[r], GRB_EQUAL, lo, std::format("mcf_r_{}", r));
            }
            else {
                if (lo > -kGurobiInf / 2.0) {
                    model.addConstr(row_expr[r], GRB_GREATER_EQUAL, lo, std::format("mcf_r_{}_lo", r));
                }
                if (up < kGurobiInf / 2.0) {
                    model.addConstr(row_expr[r], GRB_LESS_EQUAL, up, std::format("mcf_r_{}", r));
                }
            }
        }
        model.update();

        for (const auto& [col, value] : warm_values_by_col) {
            if (col < 0 || static_cast<std::size_t>(col) >= vars.size()) {
                continue;
            }
            vars[static_cast<std::size_t>(col)].set(GRB_DoubleAttr_Start, value);
        }
        timing.model_build_ms = elapsed_ms_between(gurobi_begin, std::chrono::steady_clock::now());

        auto gurobi_row_meta = std::Vector<GurobiRowMeta> {};
        const std::Vector<GurobiRowMeta>* gurobi_row_meta_ptr = nullptr;
        if (row_meta != nullptr) {
            gurobi_row_meta.reserve(row_meta->size());
            for (const auto& meta : *row_meta) {
                gurobi_row_meta.push_back(GurobiRowMeta {meta.kind, meta.detail});
            }
            gurobi_row_meta_ptr = &gurobi_row_meta;
        }
        const auto diag_begin = std::chrono::steady_clock::now();
        log_gurobi_matrix_diagnostics(model, stage_name, diag, gurobi_row_meta_ptr);
        timing.matrix_diag_ms = elapsed_ms_between(diag_begin, std::chrono::steady_clock::now());

        const auto optimize_begin = std::chrono::steady_clock::now();
        apply_mcf_gurobi_solve_params(model, gurobi_params);
        const auto gurobi_threads = gurobi_params.threads > 0 ? gurobi_params.threads : 1;
        auto& thread_budget = mcf_gurobi_thread_budget_instance();
        thread_budget.acquire(gurobi_threads);
        if (gurobi_sink != nullptr) {
            log_paths = gurobi_sink->begin_solve(gurobi_meta);
            McfGurobiLogSink::configure_model_log(model, *log_paths);
            McfGurobiLogSink::write_settings_prm(model, log_paths->prm_path);
        }

        try {
            model.optimize();
        }
        catch (...) {
            thread_budget.release(gurobi_threads);
            throw;
        }
        thread_budget.release(gurobi_threads);
        out.model_status = model.get(GRB_IntAttr_Status);
        out.solution_class = classify_gurobi_status(out.model_status);
        if (out.solution_class == McfSolutionClass::TimeLimit) {
            debug::warning_fmt(
                "{}: unexpected Gurobi TIME_LIMIT status ({})",
                stage_name,
                out.model_status);
        }
        if (stage_result_usable(out.solution_class)) {
            out.objective = model.get(GRB_DoubleAttr_ObjVal);
            out.col_value.resize(vars.size(), 0.0);
            for (std::size_t c = 0; c < vars.size(); ++c) {
                out.col_value[c] = vars[c].get(GRB_DoubleAttr_X);
            }
        }
        timing.gurobi_optimize_ms = elapsed_ms_between(optimize_begin, std::chrono::steady_clock::now());

        if (!stage_result_usable(out.solution_class)
            && out.model_status == GRB_INFEASIBLE
            && row_meta != nullptr
            && row_meta->size() == row_lo.size()) {
            const auto iis_begin = std::chrono::steady_clock::now();
            model.computeIIS();
            const auto num_constrs = model.get(GRB_IntAttr_NumConstrs);   // get the number of constraints
            const auto constrs = model.getConstrs();                    // get the constraints
            auto seen_rows = std::set<std::size_t> {};
            for (int ci = 0; ci < num_constrs; ++ci) {
                const auto& constr = constrs[ci];
                if (constr.get(GRB_IntAttr_IISConstr) == 0) {                   // if the constraint is not in the IIS, skip it
                    continue;
                }
                const auto name = constr.get(GRB_StringAttr_ConstrName); // get the name of the constraint
                std::size_t row = 0;
                if (name.starts_with("mcf_r_")) {
                    const auto suffix = name.substr(6);
                    const auto under = suffix.find('_');
                    const auto num_str = under == std::String::npos ? suffix : suffix.substr(0, under);
                    row = static_cast<std::size_t>(std::stoul(num_str));
                }
                if (row < row_meta->size() && !seen_rows.contains(row)) {
                    seen_rows.insert(row);
                    out.iis_rows.push_back(row_meta->at(row));
                }
            }
            timing.compute_iis_ms = elapsed_ms_between(iis_begin, std::chrono::steady_clock::now());
        }
        out.ok = stage_result_usable(out.solution_class);
        out.message = out.ok ? std::String("ok")
                             : std::format(
                                   "{}: {}",
                                   stage_name,
                                   solution_class_name(out.solution_class));
        if (gurobi_sink != nullptr && log_paths.has_value()) {
            gurobi_sink->end_solve(gurobi_meta, *log_paths, model, out.solution_class);
        }
        out.gurobi_timing = timing;
        return out;
    }
    catch (const GRBException& e) {
        out.gurobi_timing = timing;
        out.solution_class = McfSolutionClass::Failed;
        out.ok = false;
        out.message = std::format("{}: Gurobi exception {}: {}", stage_name, e.getErrorCode(), e.getMessage());
        if (gurobi_sink != nullptr && log_paths.has_value()) {
            gurobi_sink->end_solve_exception(
                gurobi_meta,
                *log_paths,
                McfSolutionClass::Failed,
                e.getMessage());
        }
        return out;
    }
    catch (const std::exception& e) {
        out.gurobi_timing = timing;
        out.solution_class = McfSolutionClass::Failed;
        out.ok = false;
        out.message = std::format("{}: Gurobi solve failed: {}", stage_name, e.what());
        if (gurobi_sink != nullptr && log_paths.has_value()) {
            gurobi_sink->end_solve_exception(gurobi_meta, *log_paths, McfSolutionClass::Failed, e.what());
        }
        return out;
    }
}

auto get_peak_rss_mb() -> double {
    rusage usage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return 0.0;
    }
#if defined(__APPLE__)
    return static_cast<double>(usage.ru_maxrss) / (1024.0 * 1024.0);
#else
    return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
}

auto unit_bank(const std::size_t unit) -> std::size_t {
    return unit < 8 ? 0 : 1;
}

auto unit_local(const std::size_t unit) -> std::size_t {
    return unit % 8;
}

auto track_from_unit_inner(const std::size_t unit, const std::size_t inner) -> std::size_t {
    return unit_bank(unit) * 64 + inner * 8 + unit_local(unit);
}

auto node_text(const GlobalGraph& g, const int node) -> std::String {
    if (node < 0 || node >= static_cast<int>(g.nodes.size())) {
        return std::format("N{}", node);
    }
    const auto& meta = g.nodes[static_cast<std::size_t>(node)];
    if (meta.is_virtual) {
        if (meta.virtual_kind == 1) {
            return std::format("V_P_U{}", meta.unit);
        }
        if (meta.virtual_kind == 2) {
            return std::format("V_N_U{}", meta.unit);
        }
    }
    const auto dir_str = meta.track_dir == 0 ? "H" : "V";
    return std::format("U{} {}({},{}) T{}", meta.unit, dir_str, meta.track_row, meta.track_col, meta.track);
}

auto add_node(GlobalGraph& g, const NodeMeta& node) -> int {
    const int id = static_cast<int>(g.nodes.size());
    g.nodes.push_back(node);
    if (!node.is_virtual) {
        g.node_id_by_key[NodeKey{node.unit, node.track_dir, node.track_row, node.track_col, node.track}] = id;
    }
    return id;
}

auto get_node_id(
    const GlobalGraph& g,
    const std::size_t unit,
    const int dir,
    const int row,
    const int col,
    const std::size_t track
) -> int {
    const auto it = g.node_id_by_key.find(NodeKey{unit, dir, row, col, track});
    if (it == g.node_id_by_key.end()) {
        return -1;
    }
    return it->second;
}

auto add_arc(
    GlobalGraph& g,
    const int u,
    const int v,
    const bool is_virtual,
    const bool is_turn,
    const std::size_t unit,
    const int cob,
    const std::size_t track_in,
    const std::size_t track_out,
    const hardware::COBDirection from_dir = hardware::COBDirection::Left,
    const hardware::COBDirection to_dir = hardware::COBDirection::Left
) -> void {
    if (u < 0 || v < 0 || u >= static_cast<int>(g.nodes.size()) || v >= static_cast<int>(g.nodes.size())) {
        return;
    }
    if (g.directed_arc_set.contains({u, v})) {
        return;
    }
    g.directed_arc_set.insert({u, v});
    g.arcs.push_back(Arc {u, v, is_virtual, is_turn, unit, cob, track_in, track_out, from_dir, to_dir});
}

auto side_track_pos(
    const hardware::COBDirection side,
    const int cob_r,
    const int cob_c
) -> std::tuple<int, int, int> {
    switch (side) {
        case hardware::COBDirection::Down:  return {1, cob_r, cob_c};
        case hardware::COBDirection::Up:    return {1, cob_r + 1, cob_c};
        case hardware::COBDirection::Left:  return {0, cob_r, cob_c};
        case hardware::COBDirection::Right: return {0, cob_r, cob_c + 1};
    }
    return {0, 0, 0};
}

auto is_straight_through(
    const hardware::COBDirection from,
    const hardware::COBDirection to
) -> bool {
    return (from == hardware::COBDirection::Left && to == hardware::COBDirection::Right)
        || (from == hardware::COBDirection::Right && to == hardware::COBDirection::Left)
        || (from == hardware::COBDirection::Up && to == hardware::COBDirection::Down)
        || (from == hardware::COBDirection::Down && to == hardware::COBDirection::Up);
}

auto char_to_cobdir(const char c) -> hardware::COBDirection {
    switch (c) {
        case 'R': return hardware::COBDirection::Right;
        case 'U': return hardware::COBDirection::Up;
        case 'D': return hardware::COBDirection::Down;
        default:  return hardware::COBDirection::Left;
    }
}

auto build_track_graph(const CobMcfGridDims& grid) -> GlobalGraph {
    GlobalGraph g {};
    g.rows = grid.rows;
    g.cols = grid.cols;
    g.num_cob = grid.rows * grid.cols;

    for (std::size_t unit = 0; unit < 16; ++unit) {
        for (std::size_t inner = 0; inner < 8; ++inner) {
            const auto tr = track_from_unit_inner(unit, inner);
            for (int r = 0; r <= g.rows; ++r) {
                for (int c = 0; c < g.cols; ++c) {
                    add_node(g, NodeMeta {false, 0, unit, 1, r, c, tr});
                }
            }
            for (int r = 0; r < g.rows; ++r) {
                for (int c = 0; c <= g.cols; ++c) {
                    add_node(g, NodeMeta {false, 0, unit, 0, r, c, tr});
                }
            }
        }
    }
    constexpr auto dirs = std::array {
        hardware::COBDirection::Left,
        hardware::COBDirection::Right,
        hardware::COBDirection::Up,
        hardware::COBDirection::Down
    };
    for (int cob_r = 0; cob_r < g.rows; ++cob_r) {
        for (int cob_c = 0; cob_c < g.cols; ++cob_c) {
            const int cob_linear = cob_r * g.cols + cob_c;
            for (std::size_t unit = 0; unit < 16; ++unit) {
                for (std::size_t inner = 0; inner < 8; ++inner) {
                    for (const auto from : dirs) {
                        for (const auto to : dirs) {
                            if (from == to) {
                                continue;
                            }
                            const auto mapped = static_cast<std::size_t>(
                                hardware::COBUnit::index_map(from, inner, to));
                            const auto tr_in = track_from_unit_inner(unit, inner);
                            const auto tr_out = track_from_unit_inner(unit, mapped);
                            const auto [in_dir, in_r, in_c] = side_track_pos(from, cob_r, cob_c);
                            const auto [out_dir, out_r, out_c] = side_track_pos(to, cob_r, cob_c);
                            const int u = get_node_id(g, unit, in_dir, in_r, in_c, tr_in);
                            const int v = get_node_id(g, unit, out_dir, out_r, out_c, tr_out);
                            const bool turn = !is_straight_through(from, to);
                            add_arc(g, u, v, false, turn, unit, cob_linear,
                                    tr_in, tr_out, from, to);
                        }
                    }
                }
            }
        }
    }
    return g;
}

auto tob_from_linear(const std::size_t tob_linear) -> hardware::TOBCoord {
    const auto width = static_cast<std::size_t>(hardware::Interposer::TOB_ARRAY_WIDTH);
    return hardware::TOBCoord {
        static_cast<std::i64>(tob_linear / width),
        static_cast<std::i64>(tob_linear % width)};
}

auto tob_anchor_cob(const std::size_t tob_linear) -> hardware::COBCoord {
    const auto tob = tob_from_linear(tob_linear);
    return hardware::COBCoord {
        static_cast<std::i64>(1 + 2 * tob.row),
        static_cast<std::i64>(3 * tob.col)};
}

auto node_from_bump_track(
    const GlobalGraph& g,
    const std::size_t unit,
    const std::size_t tob_linear,
    const std::size_t track
) -> int {
    const auto tob = tob_from_linear(tob_linear);
    const int r = static_cast<int>(1 + 2 * tob.row);
    const int c = static_cast<int>(3 * tob.col);
    return get_node_id(g, unit, 1, r, c, track);
}

auto node_from_track_coord(
    const GlobalGraph& g,
    const std::size_t unit,
    const hardware::TrackCoord& tc,
    const std::size_t track
) -> int {
    const int dir = tc.dir == hardware::TrackDirection::Horizontal ? 0 : 1;
    return get_node_id(g, unit, dir, static_cast<int>(tc.row), static_cast<int>(tc.col), track);
}

constexpr std::string_view kSyncBusOriginPrefix = "SyncNet in group ";

auto is_sync_bus_origin_key(const std::String& origin_key) -> bool {
    if (!origin_key.starts_with(kSyncBusOriginPrefix)) {
        return false;
    }
    const auto suffix = origin_key.substr(kSyncBusOriginPrefix.size());
    if (suffix.empty()) {
        return false;
    }
    int group_id = 0;
    for (const char ch : suffix) {
        if (!std::isdigit(static_cast<unsigned char>(ch))) {
            return false;
        }
        group_id = group_id * 10 + (ch - '0');
    }
    return group_id > 0;
}

auto simple_origin_group_key(const Net_cost_record& record) -> std::String {
    return record_origin_group_uid(record);
}

auto prepare_commodities(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfGridDims& grid,
    GlobalGraph& graph
) -> std::Vector<PreparedCommodity> {
    auto out = std::Vector<PreparedCommodity> {};
    if (records.size() != ilp_result.record_track_endpoints.size()) {
        throw std::runtime_error("MCF prepare: record_track_endpoints size mismatch");
    }

    // 遍历每一条net，构造PreparedCommodity 
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto& record = records[i];
        const auto& endpoint = ilp_result.record_track_endpoints[i];
        PreparedCommodity c {};
        c.label = std::format("{}#{}", record.net_name, record.record_id);
        c.origin_name = record.origin_key.empty() ? record.net_name : record.origin_key;
        c.origin_uid = record.origin_uid.empty() ? c.origin_name : record.origin_uid;
        c.record_index = i;
        c.record_id = record.record_id;
        c.cob_unit = endpoint.cob_unit;
        c.start_track = endpoint.start_track;
        c.end_track = endpoint.end_track;
        c.demand = 1;
        c.bus_key = c.origin_uid;
        c.is_bus = is_sync_bus_origin_key(c.origin_name);

        if (!endpoint.has_start_track) {
            debug::warning_fmt("MCF prepare: record {} has no start track", record.net_name);
            continue;
        }
        if (map_track(endpoint.start_track) != c.cob_unit) {
            debug::warning_fmt(
                "MCF prepare: start track {} not in assigned cobunit {} for record {}",
                endpoint.start_track,
                c.cob_unit,
                record.net_name);
            continue;
        }

        if (record.start_bumps.empty()) {
            continue;
        }
        c.src = node_from_bump_track(graph, c.cob_unit, record.start_bumps.front().TOB, endpoint.start_track);

        if (record.type == Net_type::PNnet) {
            if (c.cob_unit >= 16) {
                continue;
            }
            if (!endpoint.has_end_track) {
                debug::warning_fmt("MCF prepare: PNnet record {} has no selected end track", record.net_name);
                continue;
            }
            if (map_track(endpoint.end_track) != c.cob_unit) {
                debug::warning_fmt(
                    "MCF prepare: PNnet end track {} not in assigned cobunit {} for record {}",
                    endpoint.end_track,
                    c.cob_unit,
                    record.net_name);
                continue;
            }
            if (!record.pn_end_track_coord_by_index.contains(endpoint.end_track)) {
                debug::warning_fmt(
                    "MCF prepare: PNnet record {} missing coord for selected end track {}",
                    record.net_name,
                    endpoint.end_track);
                continue;
            }
            const auto& tc = record.pn_end_track_coord_by_index.at(endpoint.end_track);
            c.snk = node_from_track_coord(graph, c.cob_unit, tc, endpoint.end_track);
            if (c.snk < 0) {
                debug::warning_fmt(
                    "MCF prepare: PNnet record {} unresolved selected end track node {}",
                    record.net_name,
                    endpoint.end_track);
                continue;
            }
            c.end_track = endpoint.end_track;
        }
        else if (record.type == Net_type::Tnet) {
            if (!endpoint.has_end_track) {
                continue;
            }
            c.snk = node_from_track_coord(graph, c.cob_unit, record.mcf_end_track, endpoint.end_track);
        }
        else {
            if (record.end_bumps.empty() || !endpoint.has_end_track) {
                continue;
            }
            c.snk = node_from_bump_track(graph, c.cob_unit, record.end_bumps.front().TOB, endpoint.end_track);
        }

        if (c.src < 0 || c.snk < 0) {
            debug::warning_fmt(
                "MCF prepare: unresolved endpoint node for record {} (src={}, snk={})",
                record.net_name,
                c.src,
                c.snk);
            continue;
        }

        out.push_back(std::move(c));
    }
    return out;
}

auto arc_usable_for_unit(
    const GlobalGraph& graph,
    const Arc& arc,
    const std::size_t unit
) -> bool {
    if (arc.unit != unit) {
        return false;
    }
    (void)graph;
    return true;
}

auto to_bbox_inputs(const std::Vector<PreparedCommodity>& commodities) -> std::Vector<McfBBoxCommodityInput> {
    auto out = std::Vector<McfBBoxCommodityInput> {};
    out.reserve(commodities.size());
    for (const auto& commodity : commodities) {
        out.push_back(McfBBoxCommodityInput {
            commodity.record_index,
            commodity.is_bus,
            commodity.bus_key});
    }
    return out;
}

auto arc_allowed_for_commodity(
    const GlobalGraph& graph,
    const Arc& arc,
    const PreparedCommodity& commodity,
    const McfBBoxContext& ctx,
    const std::size_t global_commodity_id,
    const McfArcBBoxMode mode,
    const McfCommodityBBox& origin_group_bbox
) -> bool {
    if (!arc_usable_for_unit(graph, arc, commodity.cob_unit)) {
        return false;
    }
    const McfBBoxCommodityInput input {
        commodity.record_index,
        commodity.is_bus,
        commodity.bus_key};
    const auto effective = resolve_mcf_bbox(ctx, global_commodity_id, input, mode, origin_group_bbox);
    return arc_allowed_in_mcf_bbox(
        arc,
        graph.nodes[static_cast<std::size_t>(arc.u)],
        graph.nodes[static_cast<std::size_t>(arc.v)],
        effective.restricted,
        effective.box,
        graph.cols,
        commodity.src,
        commodity.snk);
}

auto commodity_bbox_connected(
    const GlobalGraph& graph,
    const PreparedCommodity& commodity,
    const McfCommodityBBox& effective_bbox
) -> bool {
    return commodity_bbox_connected(
        graph,
        commodity.src,
        commodity.snk,
        commodity.cob_unit,
        effective_bbox);
}

auto effective_bbox_for_simple_commodity(
    const std::size_t global_commodity_id,
    const std::Vector<std::size_t>& simple_ids,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& ctx,
    const McfBBoxExpandState* expand_state,
    const PreparedCommodity* commodity_override = nullptr
) -> McfCommodityBBox {
    if (commodity_override != nullptr && commodity_override->is_refined_segment
        && commodity_override->bbox_override.has_value() && commodity_override->bbox_override->restricted) {
        return *commodity_override->bbox_override;
    }
    const std::size_t lookup_id =
        commodity_override != nullptr && commodity_override->bbox_global_commodity_id != std::numeric_limits<std::size_t>::max()
        ? commodity_override->bbox_global_commodity_id
        : global_commodity_id;
    if (lookup_id >= ctx.per_commodity.size()) {
        return McfCommodityBBox {};
    }
    const auto& commodity = commodity_override != nullptr ? *commodity_override : commodities[global_commodity_id];
    const auto& record = records[commodity.record_index];
    const auto group_key = std::make_pair(commodity.cob_unit, simple_origin_group_key(record));
    if (expand_state != nullptr) {
        if (const auto overlay = lookup_simple_origin_hull(*expand_state, group_key)) {
            return McfCommodityBBox {true, *overlay};
        }
    }
    auto global_ids = std::Vector<std::size_t> {};
    auto record_indices = std::Vector<std::size_t> {};
    for (const auto sid : simple_ids) {
        const auto& sc = commodities[sid];
        const auto& sr = records[sc.record_index];
        if (std::make_pair(sc.cob_unit, simple_origin_group_key(sr)) != group_key) {
            continue;
        }
        const auto bbox_gid =
            sc.bbox_global_commodity_id != std::numeric_limits<std::size_t>::max()
            ? sc.bbox_global_commodity_id
            : sid;
        global_ids.push_back(bbox_gid);
        record_indices.push_back(sc.record_index);
    }
    bool is_multi_fanout = false;
    if (global_ids.size() > 1) {
        for (const auto rec_idx : record_indices) {
            const auto& sr = records[rec_idx];
            if (sr.from_track_to_bumps_split || sr.type == Net_type::PNnet) {
                is_multi_fanout = true;
                break;
            }
        }
    }
    if (!is_multi_fanout) {
        return ctx.per_commodity[lookup_id];
    }
    return compute_origin_group_bbox(global_ids, record_indices, records, ctx);
}

auto base_simple_origin_hull(
    const McfSimpleOriginGroupKey& key,
    const std::Vector<std::size_t>& simple_ids,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& ctx
) -> IlpBoundingBox {
    auto global_ids = std::Vector<std::size_t> {};
    auto record_indices = std::Vector<std::size_t> {};
    for (const auto sid : simple_ids) {
        const auto& sc = commodities[sid];
        const auto& sr = records[sc.record_index];
        if (std::make_pair(sc.cob_unit, simple_origin_group_key(sr)) != key) {
            continue;
        }
        global_ids.push_back(sid);
        record_indices.push_back(sc.record_index);
    }
    const auto bbox = compute_origin_group_bbox(global_ids, record_indices, records, ctx);
    return bbox.box;
}

auto extract_path(
    const int src,
    const int snk,
    std::map<std::pair<int, int>, int>& edge_count,
    const int num_nodes
) -> std::Vector<int> {
    auto prev = std::Vector<int>(static_cast<std::size_t>(num_nodes), -1);
    std::queue<int> q;
    q.push(src);
    prev[static_cast<std::size_t>(src)] = src;
    while (!q.empty() && prev[static_cast<std::size_t>(snk)] < 0) {
        const auto u = q.front();
        q.pop();
        for (const auto& [e, cnt] : edge_count) {
            if (cnt <= 0 || e.first != u) {
                continue;
            }
            const auto v = e.second;
            if (prev[static_cast<std::size_t>(v)] >= 0) {
                continue;
            }
            prev[static_cast<std::size_t>(v)] = u;
            q.push(v);
        }
    }
    if (prev[static_cast<std::size_t>(snk)] < 0) {
        return {};
    }
    auto nodes = std::Vector<int> {};
    auto cur = snk;
    while (cur != src) {
        nodes.push_back(cur);
        cur = prev[static_cast<std::size_t>(cur)];
    }
    nodes.push_back(src);
    std::reverse(nodes.begin(), nodes.end());
    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
        auto key = std::make_pair(nodes[i], nodes[i + 1]);
        if (edge_count.contains(key) && edge_count[key] > 0) {
            edge_count[key] -= 1;
        }
    }
    return nodes;
}

auto normalized_edge_key(int u, int v) -> std::pair<int, int> {
    if (u > v) {
        std::swap(u, v);
    }
    return {u, v};
}

auto build_undirected_incidence(
    const std::map<std::pair<int, int>, int>& edge_keys
) -> std::map<int, std::Vector<std::pair<int, int>>> {
    auto delta = std::map<int, std::Vector<std::pair<int, int>>> {};
    for (const auto& [key, _] : edge_keys) {
        delta[key.first].push_back(key);
        delta[key.second].push_back(key);
    }
    return delta;
}

auto route_one_mcf_warm_path(
    const GlobalGraph& graph,
    const PreparedCommodity& commodity,
    const McfCommodityBBox& effective_bbox,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    const std::map<std::pair<int, int>, int>& used_edges,
    const std::map<int, int>& used_nodes,
    const std::set<std::pair<int, int>>* tree_edges = nullptr,
    const std::set<int>* tree_nodes = nullptr
) -> std::Vector<int> {
    auto prev_node = std::vector<int>(graph.nodes.size(), -1);
    auto q = std::queue<int> {};
    q.push(commodity.src);
    prev_node[static_cast<std::size_t>(commodity.src)] = commodity.src;

    while (!q.empty()) {
        const auto node = q.front();
        q.pop();
        if (node == commodity.snk) {
            break;
        }
        for (const auto arc_id : outgoing_arcs[static_cast<std::size_t>(node)]) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!arc_usable_for_unit(graph, arc, commodity.cob_unit)) {
                continue;
            }
            if (!arc_allowed_in_mcf_bbox(
                    arc,
                    graph.nodes[static_cast<std::size_t>(arc.u)],
                    graph.nodes[static_cast<std::size_t>(arc.v)],
                    effective_bbox.restricted,
                    effective_bbox.box,
                    graph.cols,
                    commodity.src,
                    commodity.snk)) {
                continue;
            }
            if (!arc.is_virtual) {
                const auto edge_key = normalized_edge_key(arc.u, arc.v);
                if (const auto it = used_edges.find(edge_key);
                    it != used_edges.end() && it->second >= 1
                    && (tree_edges == nullptr || !tree_edges->contains(edge_key))) {
                    continue;
                }
            }
            if (!graph.nodes[static_cast<std::size_t>(arc.v)].is_virtual) {
                if (const auto it = used_nodes.find(arc.v);
                    it != used_nodes.end() && it->second >= 1
                    && (tree_nodes == nullptr || !tree_nodes->contains(arc.v))) {
                    continue;
                }
            }
            if (prev_node[static_cast<std::size_t>(arc.v)] != -1) {
                continue;
            }
            prev_node[static_cast<std::size_t>(arc.v)] = node;
            q.push(arc.v);
        }
    }

    if (prev_node[static_cast<std::size_t>(commodity.snk)] == -1) {
        return {};
    }
    auto path = std::Vector<int> {};
    auto cur = commodity.snk;
    while (cur != commodity.src) {
        path.push_back(cur);
        cur = prev_node[static_cast<std::size_t>(cur)];
    }
    path.push_back(commodity.src);
    std::reverse(path.begin(), path.end());
    return path;
}

auto route_mcf_warm_path_to_frontier(
    const GlobalGraph& graph,
    const int src,
    const std::set<int>& frontier,
    const PreparedCommodity& commodity,
    const McfCommodityBBox& effective_bbox,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    const std::map<std::pair<int, int>, int>& used_edges,
    const std::map<int, int>& used_nodes,
    const std::set<std::pair<int, int>>& tree_edges,
    const std::set<int>& tree_nodes
) -> std::Vector<int> {
    if (frontier.contains(src)) {
        return std::Vector<int> {src};
    }
    auto prev_node = std::vector<int>(graph.nodes.size(), -1);
    auto q = std::queue<int> {};
    q.push(src);
    prev_node[static_cast<std::size_t>(src)] = src;
    auto hit = -1;
    while (!q.empty() && hit < 0) {
        const auto node = q.front();
        q.pop();
        if (frontier.contains(node)) {
            hit = node;
            break;
        }
        for (const auto arc_id : outgoing_arcs[static_cast<std::size_t>(node)]) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!arc_usable_for_unit(graph, arc, commodity.cob_unit)) {
                continue;
            }
            if (!arc_allowed_in_mcf_bbox(
                    arc,
                    graph.nodes[static_cast<std::size_t>(arc.u)],
                    graph.nodes[static_cast<std::size_t>(arc.v)],
                    effective_bbox.restricted,
                    effective_bbox.box,
                    graph.cols,
                    commodity.src,
                    commodity.snk)) {
                continue;
            }
            if (!arc.is_virtual) {
                const auto edge_key = normalized_edge_key(arc.u, arc.v);
                if (const auto it = used_edges.find(edge_key);
                    it != used_edges.end() && it->second >= 1 && !tree_edges.contains(edge_key)) {
                    continue;
                }
            }
            if (!graph.nodes[static_cast<std::size_t>(arc.v)].is_virtual) {
                if (const auto it = used_nodes.find(arc.v);
                    it != used_nodes.end() && it->second >= 1 && !tree_nodes.contains(arc.v)) {
                    continue;
                }
            }
            if (prev_node[static_cast<std::size_t>(arc.v)] != -1) {
                continue;
            }
            prev_node[static_cast<std::size_t>(arc.v)] = node;
            q.push(arc.v);
        }
    }
    if (hit < 0) {
        return {};
    }
    auto path = std::Vector<int> {};
    auto cur = hit;
    while (cur != src) {
        path.push_back(cur);
        cur = prev_node[static_cast<std::size_t>(cur)];
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return path;
}

auto add_mcf_warm_path_to_origin_tree(
    const GlobalGraph& graph,
    const std::Vector<int>& path,
    std::set<std::pair<int, int>>& tree_edges,
    std::set<int>& tree_nodes
) -> void {
    for (const auto node : path) {
        if (!graph.nodes[static_cast<std::size_t>(node)].is_virtual) {
            tree_nodes.insert(node);
        }
    }
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const auto u = path[i];
        const auto v = path[i + 1];
        for (const auto& arc : graph.arcs) {
            if (arc.u != u || arc.v != v) {
                continue;
            }
            if (!arc.is_virtual) {
                tree_edges.insert(normalized_edge_key(u, v));
            }
            break;
        }
    }
}

auto expand_frontier_from_path(
    std::set<int>& frontier,
    const std::Vector<int>& path
) -> void {
    for (const auto node : path) {
        frontier.insert(node);
    }
}

auto ttb_bump_sort_key(const Net_cost_record& record) -> std::size_t {
    constexpr auto kPrefix = std::string_view {"__ttb_"};
    const auto pos = record.net_name.rfind(kPrefix);
    if (pos == std::String::npos) {
        return record.record_id;
    }
    const auto suffix = record.net_name.substr(pos + kPrefix.size());
    if (suffix.empty()) {
        return record.record_id;
    }
    std::size_t value = 0;
    for (const char ch : suffix) {
        if (!std::isdigit(static_cast<unsigned char>(ch))) {
            return record.record_id;
        }
        value = value * 10 + static_cast<std::size_t>(ch - '0');
    }
    return value;
}

auto sort_multi_fanout_children(
    const McfOriginGroup& group,
    const std::Vector<PreparedCommodity>& local_com,
    const std::Vector<Net_cost_record>& records
) -> std::Vector<int> {
    auto ordered = group.commodity_local_indices;
    if (ordered.empty()) {
        return ordered;
    }
    const auto& first_rec = records[local_com[static_cast<std::size_t>(ordered.front())].record_index];
    if (first_rec.from_track_to_bumps_split) {
        std::sort(ordered.begin(), ordered.end(), [&](const int a, const int b) {
            const auto& rec_a = records[local_com[static_cast<std::size_t>(a)].record_index];
            const auto& rec_b = records[local_com[static_cast<std::size_t>(b)].record_index];
            return ttb_bump_sort_key(rec_a) < ttb_bump_sort_key(rec_b);
        });
    }
    else if (first_rec.type == Net_type::PNnet) {
        std::sort(ordered.begin(), ordered.end(), [&](const int a, const int b) {
            const auto idx_a = local_com[static_cast<std::size_t>(a)].record_index;
            const auto idx_b = local_com[static_cast<std::size_t>(b)].record_index;
            return idx_a < idx_b;
        });
    }
    return ordered;
}

auto resolve_origin_hub_node(
    const GlobalGraph& graph,
    const McfOriginGroup& group,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& simple_ids,
    const std::Vector<PreparedCommodity>& local_com,
    const std::Vector<Net_cost_record>& records
) -> std::optional<int> {
    if (group.commodity_local_indices.empty()) {
        return std::nullopt;
    }
    const auto& first_rec = records[local_com[static_cast<std::size_t>(group.commodity_local_indices.front())].record_index];
    if (first_rec.from_track_to_bumps_split) {
        const auto first_cid = simple_ids[static_cast<std::size_t>(group.commodity_local_indices.front())];
        const auto ref_snk = commodities[first_cid].snk;
        for (const auto local_k : group.commodity_local_indices) {
            const auto cid = simple_ids[static_cast<std::size_t>(local_k)];
            if (commodities[cid].snk != ref_snk) {
                debug::warning_fmt(
                    "pre-routing SimpleMCF warm start: TTB origin \"{}\" has mismatched snk nodes; skipping maze-style routing",
                    group.origin_key);
                return std::nullopt;
            }
        }
        return ref_snk;
    }
    (void)graph;
    return std::nullopt;
}

auto mark_mcf_warm_path_used(
    const GlobalGraph& graph,
    const std::Vector<int>& path,
    std::map<std::pair<int, int>, int>& used_edges,
    std::map<int, int>& used_nodes
) -> void {
    for (const auto node : path) {
        if (!graph.nodes[static_cast<std::size_t>(node)].is_virtual) {
            used_nodes[node] = 1;
        }
    }
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const auto u = path[i];
        const auto v = path[i + 1];
        for (const auto& arc : graph.arcs) {
            if (arc.u != u || arc.v != v) {
                continue;
            }
            if (!arc.is_virtual) {
                used_edges[normalized_edge_key(u, v)] = 1;
            }
            break;
        }
    }
}

struct WarmStartSymmetryBreakInfo {
    std::map<int, std::set<std::pair<int, int>>> used_edges_by_h;
    std::size_t matched_paths{0};
};

auto collect_warm_start_used_edges_by_origin(
    const GlobalGraph& graph,
    const StageWarmStart& warm_start,
    const std::Vector<PreparedCommodity>& local_com,
    const std::Vector<int>& commodity_origin_h
) -> WarmStartSymmetryBreakInfo {
    WarmStartSymmetryBreakInfo out;
    for (int k = 0; k < static_cast<int>(local_com.size()); ++k) {
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        const auto path_it = warm_start.nodes_by_record_id.find(commodity.record_id);
        if (path_it == warm_start.nodes_by_record_id.end()) {
            continue;
        }
        const auto& path = path_it->second;
        if (path.size() < 2) {
            continue;
        }
        ++out.matched_paths;
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        for (std::size_t i = 0; i + 1 < path.size(); ++i) {
            const auto u = path[i];
            const auto v = path[i + 1];
            for (const auto& arc : graph.arcs) {
                if (arc.u != u || arc.v != v) {
                    continue;
                }
                if (!arc.is_virtual) {
                    out.used_edges_by_h[h].insert(normalized_edge_key(u, v));
                }
                break;
            }
        }
    }
    return out;
}

auto route_mcf_stage_warm_start(
    const std::String& stage_name,
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& commodity_ids,
    const std::function<McfCommodityBBox(std::size_t)>& effective_bbox_for,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    std::map<std::pair<int, int>, int>& used_edges,
    std::map<int, int>& used_nodes
) -> StageWarmStart {
    auto warm = StageWarmStart {};
    std::size_t routed = 0;
    std::size_t failed = 0;
    for (const auto cid : commodity_ids) {
        const auto& commodity = commodities[cid];
        auto path = route_one_mcf_warm_path(
            graph,
            commodity,
            effective_bbox_for(cid),
            outgoing_arcs,
            used_edges,
            used_nodes);
        if (path.empty()) {
            ++failed;
            debug::warning_fmt("pre-routing {} warm start failed for commodity {}", stage_name, commodity.label);
            continue;
        }
        mark_mcf_warm_path_used(graph, path, used_edges, used_nodes);
        warm.nodes_by_record_id.emplace(commodity.record_id, std::move(path));
        ++routed;
    }
    debug::info_fmt(
        "pre-routing {} warm start: routed_commodities={} failed_commodities={}",
        stage_name,
        routed,
        failed);
    return warm;
}

auto build_origin_groups(
    const std::Vector<PreparedCommodity>& local_com,
    const std::Vector<Net_cost_record>& records
) -> std::Vector<McfOriginGroup> {
    // 记录 commodity 信息
    auto key_to_indices = std::map<std::pair<std::size_t, std::String>, std::Vector<int>> {};
    for (int k = 0; k < static_cast<int>(local_com.size()); ++k) {
        const auto& c = local_com[static_cast<std::size_t>(k)];
        const auto& rec = records[c.record_index];
        key_to_indices[{c.cob_unit, simple_origin_group_key(rec)}].push_back(k);
    }

    auto groups = std::Vector<McfOriginGroup> {};
    int gid = 0;
    for (auto& [key, indices] : key_to_indices) {
        bool is_multi_fanout = false;
        if (indices.size() > 1) {
            // 是目前允许的多扇出net
            for (const auto k : indices) {
                const auto& rec = records[local_com[static_cast<std::size_t>(k)].record_index];
                if (rec.from_track_to_bumps_split || rec.type == Net_type::PNnet) {
                    is_multi_fanout = true;
                    break;
                }
            }
        }
        McfOriginGroup group {};
        group.origin_key = key.second;
        group.cob_unit = key.first;
        group.commodity_local_indices = std::move(indices);
        group.origin_group_id = gid++;
        group.is_multi_fanout = is_multi_fanout;
        groups.push_back(std::move(group));
    }
    return groups;
}

auto log_origin_groups(const std::String& stage_name, const std::Vector<McfOriginGroup>& groups) -> void {
    std::size_t multi_count = 0;
    for (const auto& g : groups) {
        if (g.is_multi_fanout) {
            ++multi_count;
        }
    }
    debug::info_fmt(
        "{} origin groups: total={} multi_fanout={}",
        stage_name,
        groups.size(),
        multi_count);
}

auto build_local_commodities(
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& commodity_ids
) -> std::Vector<PreparedCommodity> {
    auto local_com = std::Vector<PreparedCommodity> {};
    local_com.reserve(commodity_ids.size());
    for (const auto cid : commodity_ids) {
        local_com.push_back(commodities[cid]);
    }
    return local_com;
}

auto route_simple_mcf_stage_warm_start_by_origin(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& simple_ids,
    const std::Vector<Net_cost_record>& records,
    const std::function<McfCommodityBBox(std::size_t)>& effective_bbox_for,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    std::map<std::pair<int, int>, int>& used_edges,
    std::map<int, int>& used_nodes
) -> StageWarmStart {
    auto warm = StageWarmStart {};
    auto local_com = build_local_commodities(commodities, simple_ids);
    auto origin_groups = build_origin_groups(local_com, records);

    std::size_t routed = 0;
    std::size_t failed = 0;
    std::size_t routed_groups = 0;
    std::size_t failed_groups = 0;
    for (const auto& group : origin_groups) {
        if (!group.is_multi_fanout) {
            std::size_t group_success = 0;
            for (const auto local_k : group.commodity_local_indices) {
                const auto cid = simple_ids[static_cast<std::size_t>(local_k)];
                const auto& commodity = commodities[cid];
                auto path = route_one_mcf_warm_path(
                    graph,
                    commodity,
                    effective_bbox_for(cid),
                    outgoing_arcs,
                    used_edges,
                    used_nodes);
                if (path.empty()) {
                    ++failed;
                    debug::warning_fmt("pre-routing SimpleMCF warm start failed for commodity {}", commodity.label);
                    continue;
                }
                ++routed;
                ++group_success;
                mark_mcf_warm_path_used(graph, path, used_edges, used_nodes);
                warm.nodes_by_record_id.emplace(commodity.record_id, std::move(path));
            }
            if (group_success > 0) {
                ++routed_groups;
            }
            else if (!group.commodity_local_indices.empty()) {
                ++failed_groups;
            }
            continue;
        }

        const auto hub = resolve_origin_hub_node(graph, group, commodities, simple_ids, local_com, records);
        if (!hub.has_value()) {
            auto tree_edges = std::set<std::pair<int, int>> {};
            auto tree_nodes = std::set<int> {};
            std::size_t group_success = 0;
            for (const auto local_k : group.commodity_local_indices) {
                const auto cid = simple_ids[static_cast<std::size_t>(local_k)];
                const auto& commodity = commodities[cid];
                auto path = route_one_mcf_warm_path(
                    graph,
                    commodity,
                    effective_bbox_for(cid),
                    outgoing_arcs,
                    used_edges,
                    used_nodes,
                    &tree_edges,
                    &tree_nodes);
                if (path.empty()) {
                    ++failed;
                    debug::warning_fmt("pre-routing SimpleMCF warm start failed for commodity {}", commodity.label);
                    continue;
                }
                ++routed;
                ++group_success;
                add_mcf_warm_path_to_origin_tree(graph, path, tree_edges, tree_nodes);
                mark_mcf_warm_path_used(graph, path, used_edges, used_nodes);
                warm.nodes_by_record_id.emplace(commodity.record_id, std::move(path));
            }
            if (group_success > 0) {
                ++routed_groups;
            }
            else {
                ++failed_groups;
            }
            continue;
        }

        auto frontier = std::set<int> {*hub};
        auto tree_edges = std::set<std::pair<int, int>> {};
        auto tree_nodes = std::set<int> {};
        const auto ordered = sort_multi_fanout_children(group, local_com, records);
        auto group_paths = std::Vector<std::pair<std::size_t, std::Vector<int>>> {};
        group_paths.reserve(ordered.size());
        std::size_t group_failed = 0;
        for (const auto local_k : ordered) {
            const auto cid = simple_ids[static_cast<std::size_t>(local_k)];
            const auto& commodity = commodities[cid];
            auto path = route_mcf_warm_path_to_frontier(
                graph,
                commodity.src,
                frontier,
                commodity,
                effective_bbox_for(cid),
                outgoing_arcs,
                used_edges,
                used_nodes,
                tree_edges,
                tree_nodes);
            if (path.empty()) {
                ++failed;
                ++group_failed;
                debug::warning_fmt("pre-routing SimpleMCF warm start failed for commodity {}", commodity.label);
                continue;
            }
            ++routed;
            group_paths.push_back({commodity.record_id, std::move(path)});
            add_mcf_warm_path_to_origin_tree(graph, group_paths.back().second, tree_edges, tree_nodes);
            mark_mcf_warm_path_used(graph, group_paths.back().second, used_edges, used_nodes);
            expand_frontier_from_path(frontier, group_paths.back().second);
        }

        if (group_failed == ordered.size()) {
            ++failed_groups;
            continue;
        }
        ++routed_groups;
        for (auto& [record_id, path] : group_paths) {
            if (path.empty()) {
                continue;
            }
            warm.nodes_by_record_id.emplace(record_id, std::move(path));
        }
    }

    debug::info_fmt(
        "pre-routing SimpleMCF warm start: routed_commodities={} failed_commodities={} routed_origin_groups={} failed_origin_groups={}",
        routed,
        failed,
        routed_groups,
        failed_groups);
    return warm;
}

auto seed_warm_used_from_bus_unit(
    const StageSolveResult& bus_res,
    const std::size_t unit_c,
    std::map<std::pair<int, int>, int>& used_edges,
    std::map<int, int>& used_nodes
) -> void {
    used_edges.clear();
    used_nodes.clear();
    for (const auto& [edge, used] : bus_res.unit_used_edges[unit_c]) {
        if (used >= 1) {
            used_edges[edge] = used;
        }
    }
    for (const auto& [node, used] : bus_res.unit_used_nodes[unit_c]) {
        if (used >= 1) {
            used_nodes[node] = used;
        }
    }
}

auto merge_stage_warm_start(StageWarmStart& target, StageWarmStart&& source) -> void {
    for (auto& [record_id, path] : source.nodes_by_record_id) {
        target.nodes_by_record_id.emplace(record_id, std::move(path));
    }
}

auto run_simple_warm_start_for_unit(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& bbox_ctx,
    const McfBBoxExpandState* expand_state,
    const StageSolveResult& bus_res,
    const std::size_t unit_c,
    const std::Vector<std::Vector<int>>& outgoing_arcs
) -> StageWarmStart {
    auto warm_used_edges = std::map<std::pair<int, int>, int> {};
    auto warm_used_nodes = std::map<int, int> {};
    seed_warm_used_from_bus_unit(bus_res, unit_c, warm_used_edges, warm_used_nodes);
    const auto simple_effective_bbox = [&](const std::size_t cid) -> McfCommodityBBox {
        return effective_bbox_for_simple_commodity(
            cid, simple_ids_for_unit, commodities, records, bbox_ctx, expand_state);
    };
    return route_simple_mcf_stage_warm_start_by_origin(
        graph,
        commodities,
        simple_ids_for_unit,
        records,
        simple_effective_bbox,
        outgoing_arcs,
        warm_used_edges,
        warm_used_nodes);
}

auto append_paths_from_f_solution(
    const std::String& stage_name,
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& local_com,
    const std::Vector<ArcVar>& f_vars,
    const std::Vector<int>& f_values,
    StageSolveResult& out
) -> void {
    const auto K = static_cast<int>(local_com.size());
    const auto N = static_cast<int>(graph.nodes.size());
    auto edge_count_by_k = std::Vector<std::map<std::pair<int, int>, int>>(static_cast<std::size_t>(K));
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto val = f_values[j];
        if (val <= 0) {
            continue;
        }
        const auto k = f_vars[j].k;
        const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[j].a)];
        edge_count_by_k[static_cast<std::size_t>(k)][{arc.u, arc.v}] += val;
    }

    out.paths.reserve(static_cast<std::size_t>(K));
    for (int k = 0; k < K; ++k) {
        const auto& c = local_com[static_cast<std::size_t>(k)];
        McfPathInfo info {};
        info.label = c.label;
        info.origin_name = c.origin_name;
        info.record_id = c.record_id;
        info.src = c.src;
        info.snk = c.snk;
        info.demand = c.demand;
        info.cob_unit = c.cob_unit;
        info.start_track = c.start_track;
        info.end_track = c.end_track;
        if (!c.record_indices.empty()) {
            info.record_indices = c.record_indices;
        }
        else {
            info.record_indices.push_back(c.record_id);
        }

        auto& flow_edges = edge_count_by_k[static_cast<std::size_t>(k)];
        for (int pi = 0; pi < c.demand; ++pi) {
            auto path = extract_path(c.src, c.snk, flow_edges, N);
            if (path.empty()) {
                break;
            }
            info.unit_paths.push_back(path);
            auto track_path = std::Vector<std::size_t> {};
            for (const auto n : path) {
                if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
                    continue;
                }
                const auto track = graph.nodes[static_cast<std::size_t>(n)].track;
                if (track_path.empty() || track_path.back() != track) {
                    track_path.push_back(track);
                }
            }
            if (!track_path.empty()) {
                info.track_paths.push_back(std::move(track_path));
            }
        }
        if (static_cast<int>(info.unit_paths.size()) < c.demand) {
            debug::warning_fmt(
                "MCF post-solve path extraction shortfall: stage={} commodity={} extracted_paths={} demand={}",
                stage_name,
                c.label,
                info.unit_paths.size(),
                c.demand);
        }
        int remaining_flow = 0;
        int remaining_edges = 0;
        for (const auto& [edge, count] : flow_edges) {
            (void)edge;
            if (count <= 0) {
                continue;
            }
            ++remaining_edges;
            remaining_flow += count;
        }
        if (remaining_edges > 0) {
            debug::warning_fmt(
                "MCF post-solve surplus flow: stage={} commodity={} remaining_edges={} remaining_flow={}",
                stage_name,
                c.label,
                remaining_edges,
                remaining_flow);
        }
        out.paths.push_back(std::move(info));
    }
}

auto collect_undirected_physical_edge_keys(
    const GlobalGraph& graph,
    const std::optional<std::size_t> unit_filter
) -> std::set<std::pair<int, int>> {
    auto keys = std::set<std::pair<int, int>> {};
    for (const auto& arc : graph.arcs) {
        if (arc.is_virtual) {
            continue;
        }
        if (unit_filter.has_value() && arc.unit != *unit_filter) {
            continue;
        }
        auto u = arc.u;
        auto v = arc.v;
        if (u > v) {
            std::swap(u, v);
        }
        keys.insert({u, v});
    }
    return keys;
}

struct McfGraphScopeStats {
    std::size_t nodes{0};
    std::size_t arcs{0};
    std::size_t physical_arcs{0};
    std::size_t undirected_physical_edges{0};
};

auto count_mcf_graph_scope_stats(
    const GlobalGraph& graph,
    const std::optional<std::size_t> unit_filter
) -> McfGraphScopeStats {
    McfGraphScopeStats stats {};
    for (const auto& node : graph.nodes) {
        if (node.is_virtual) {
            continue;
        }
        if (unit_filter.has_value() && node.unit != *unit_filter) {
            continue;
        }
        ++stats.nodes;
    }
    for (const auto& arc : graph.arcs) {
        if (unit_filter.has_value() && arc.unit != *unit_filter) {
            continue;
        }
        ++stats.arcs;
        if (!arc.is_virtual) {
            ++stats.physical_arcs;
        }
    }
    stats.undirected_physical_edges = collect_undirected_physical_edge_keys(graph, unit_filter).size();
    return stats;
}

auto log_mcf_model_graph(
    const std::String& stage_name,
    const GlobalGraph& graph,
    const int num_commodities,
    const std::optional<std::size_t> unit_filter = std::nullopt
) -> void {
    const auto stats = count_mcf_graph_scope_stats(graph, unit_filter);
    if (unit_filter.has_value()) {
        debug::info_fmt(
            "{} model graph: scope=COBUnit{} nodes={} arcs={} commodities={}",
            stage_name,
            *unit_filter,
            stats.nodes,
            stats.arcs,
            num_commodities);
    }
    else {
        debug::info_fmt(
            "{} model graph: scope=global nodes={} arcs={} commodities={}",
            stage_name,
            stats.nodes,
            stats.arcs,
            num_commodities);
    }
}

auto log_mcf_constraint_rows(
    const std::String& stage_name,
    const std::map<std::String, int>& rows_by_constraint
) -> void {
    int total = 0;
    for (const auto& [name, count] : rows_by_constraint) {
        debug::info_fmt("{} constraint \"{}\": {} row(s)", stage_name, name, count);
        total += count;
    }
    debug::info_fmt("{} constraint rows total: {}", stage_name, total);
}

auto preview_bus_candidate_resources(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& bus_ids,
    const McfBBoxContext& bbox_ctx
) -> std::pair<std::Vector<McfCandidateResources>, std::Vector<std::String>> {
    const auto local_com = build_local_commodities(commodities, bus_ids);
    const auto K = static_cast<int>(bus_ids.size());
    const auto A = static_cast<int>(graph.arcs.size());
    auto key_to_vertex = std::map<std::String, int> {};
    auto per_vertex = std::Vector<McfCandidateResources> {};
    auto labels = std::Vector<std::String> {};
    for (int k = 0; k < K; ++k) {
        const auto global_id = bus_ids[static_cast<std::size_t>(k)];
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        const auto& key = commodity.bus_key;
        if (!key_to_vertex.contains(key)) {
            key_to_vertex[key] = static_cast<int>(per_vertex.size());
            per_vertex.emplace_back();
            labels.push_back(key);
        }
        const auto vid = key_to_vertex.at(key);
        for (int a = 0; a < A; ++a) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
            if (!arc_allowed_for_commodity(
                    graph,
                    arc,
                    commodity,
                    bbox_ctx,
                    global_id,
                    McfArcBBoxMode::Bus,
                    McfCommodityBBox {})) {
                continue;
            }
            if (arc.is_virtual) {
                continue;
            }
            auto u = arc.u;
            auto v = arc.v;
            if (u > v) {
                std::swap(u, v);
            }
            per_vertex[static_cast<std::size_t>(vid)].physical_edges.insert({u, v});
            per_vertex[static_cast<std::size_t>(vid)].physical_nodes.insert(arc.u);
            per_vertex[static_cast<std::size_t>(vid)].physical_nodes.insert(arc.v);
        }
    }
    return {per_vertex, labels};
}

auto bus_ids_for_conflict_component(
    const std::Vector<std::size_t>& bus_ids,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::String>& vertex_labels,
    const McfConflictComponent& component
) -> std::Vector<std::size_t> {
    auto keys = std::set<std::String> {};
    for (const auto vid : component.vertex_ids) {
        if (vid >= 0 && static_cast<std::size_t>(vid) < vertex_labels.size()) {
            keys.insert(vertex_labels[static_cast<std::size_t>(vid)]);
        }
    }
    auto out = std::Vector<std::size_t> {};
    for (const auto gid : bus_ids) {
        if (gid >= commodities.size()) {
            continue;
        }
        if (keys.contains(commodities[gid].bus_key)) {
            out.push_back(gid);
        }
    }
    return out;
}

auto merge_stage_solve_results(
    const std::String& stage_name,
    std::Vector<StageSolveResult>&& parts,
    const std::chrono::steady_clock::time_point begin
) -> StageSolveResult {
    StageSolveResult out {};
    out.stage_name = stage_name;
    if (parts.empty()) {
        apply_stage_solution_class(out, McfSolutionClass::Failed);
        out.message = std::format("{}: no conflict components", stage_name);
        return finish_stage_solve_early(out, begin);
    }
    out.ok = true;
    out.solution_class = McfSolutionClass::Optimal;
    auto timing = StageSolveTiming {};
    const auto merge_component_timing = parts.size() > 1;
    for (auto& part : parts) {
        out.objective += part.objective;
        out.paths.insert(out.paths.end(), part.paths.begin(), part.paths.end());
        for (const auto& [e, used] : part.used_edges) {
            out.used_edges[e] += used;
        }
        for (const auto& [n, used] : part.used_nodes) {
            out.used_nodes[n] += used;
        }
        for (std::size_t u = 0; u < 16; ++u) {
            for (const auto& [e, used] : part.unit_used_edges[u]) {
                out.unit_used_edges[u][e] += used;
            }
            for (const auto& [n, used] : part.unit_used_nodes[u]) {
                out.unit_used_nodes[u][n] += used;
            }
        }
        if (merge_component_timing) {
            timing.model_build_ms = std::max(timing.model_build_ms, part.timing.model_build_ms);
            timing.matrix_diag_ms = std::max(timing.matrix_diag_ms, part.timing.matrix_diag_ms);
            timing.gurobi_optimize_ms = std::max(timing.gurobi_optimize_ms, part.timing.gurobi_optimize_ms);
            timing.compute_iis_ms = std::max(timing.compute_iis_ms, part.timing.compute_iis_ms);
            timing.extract_path_ms = std::max(timing.extract_path_ms, part.timing.extract_path_ms);
        }
        else {
            timing.model_build_ms += part.timing.model_build_ms;
            timing.matrix_diag_ms += part.timing.matrix_diag_ms;
            timing.gurobi_optimize_ms += part.timing.gurobi_optimize_ms;
            timing.compute_iis_ms += part.timing.compute_iis_ms;
            timing.extract_path_ms += part.timing.extract_path_ms;
        }
        if (!part.ok) {
            out.ok = false;
            if (static_cast<int>(part.solution_class) > static_cast<int>(out.solution_class)) {
                out.solution_class = part.solution_class;
            }
            out.message = part.message;
            out.model_status = part.model_status;
            for (const auto& key : part.failed_bus_keys) {
                append_unique(out.failed_bus_keys, key);
            }
            for (const auto idx : part.failed_record_indices) {
                append_unique(out.failed_record_indices, idx);
            }
            for (const auto& group : part.failed_simple_origin_groups) {
                append_unique(out.failed_simple_origin_groups, group);
            }
            out.infeasibility_hints.insert(
                out.infeasibility_hints.end(),
                part.infeasibility_hints.begin(),
                part.infeasibility_hints.end());
            out.bus_failure_unlocalized = out.bus_failure_unlocalized || part.bus_failure_unlocalized;
        }
    }
    if (out.ok) {
        out.message = "ok";
    }
    if (merge_component_timing) {
        debug::info_fmt(
            "{}: merged {} parallel conflict components (sub-timing fields report max)",
            stage_name,
            parts.size());
    }
    return finish_stage_solve_result(out, begin, timing);
}

auto solve_bus_mcf_component(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& bus_ids,
    const McfBBoxContext& bbox_ctx,
    const StageWarmStart* warm_start,
    const GurobiDiagnosticsOptions& diag,
    McfGurobiLogSink* gurobi_sink,
    McfGurobiSolveMeta gurobi_meta,
    const std::String& stage_name,
    const McfGurobiSolveParams& gurobi_params
) -> StageSolveResult {
    const auto solve_begin = std::chrono::steady_clock::now();
    StageSolveResult out {};
    out.stage_name = stage_name;
    if (bus_ids.empty()) {
        if (gurobi_sink != nullptr) {
            gurobi_sink->write_skipped(McfGurobiLogStage::bus(), "empty_stage");
        }
        apply_stage_solution_class(out, McfSolutionClass::Skipped);
        out.message = "empty stage";
        return finish_stage_solve_early(out, solve_begin);
    }

    const auto K = static_cast<int>(bus_ids.size());
    const auto A = static_cast<int>(graph.arcs.size());
    const auto local_com = build_local_commodities(commodities, bus_ids);

    for (int k = 0; k < K; ++k) {
        const auto global_id = bus_ids[static_cast<std::size_t>(k)];
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        const McfBBoxCommodityInput input {
            commodity.record_index,
            commodity.is_bus,
            commodity.bus_key};
        const auto effective = resolve_mcf_bbox(
            bbox_ctx,
            global_id,
            input,
            McfArcBBoxMode::Bus,
            McfCommodityBBox {});
        if (effective.restricted && !commodity_bbox_connected(graph, commodity, effective)) {
            if (gurobi_sink != nullptr) {
                gurobi_sink->write_stub(
                    gurobi_meta,
                    "bbox_disconnected",
                    std::format("commodity={}", commodity.label));
            }
            apply_stage_solution_class(out, McfSolutionClass::Failed);
            out.message = std::format(
                "{}: bbox disconnected for commodity {}",
                stage_name,
                commodity.label);
            append_bus_retry_hint(out, commodity.bus_key, commodity.record_index);
            return finish_stage_solve_early(out, solve_begin);
        }
    }

    // BusMCF §1: f^{c,n}_{ij} variables
    auto f_vars = std::Vector<ArcVar> {};
    auto f_by_k = std::Vector<std::Vector<int>>(static_cast<std::size_t>(K));
    f_vars.reserve(static_cast<std::size_t>(K * A / 8 + 1));
    for (int k = 0; k < K; ++k) {
        const auto global_id = bus_ids[static_cast<std::size_t>(k)];
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        for (int a = 0; a < A; ++a) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
            if (!arc_allowed_for_commodity(
                    graph,
                    arc,
                    commodity,
                    bbox_ctx,
                    global_id,
                    McfArcBBoxMode::Bus,
                    McfCommodityBBox {})) {
                continue;
            }
            const auto var_id = static_cast<int>(f_vars.size());
            f_vars.push_back(ArcVar {k, a});
            f_by_k[static_cast<std::size_t>(k)].push_back(var_id);
        }
    }
    if (f_vars.empty()) {
        if (gurobi_sink != nullptr) {
            gurobi_sink->write_stub(gurobi_meta, "no_feasible_arc_variable_pairs");
        }
        apply_stage_solution_class(out, McfSolutionClass::Failed);
        out.message = std::format("{}: no feasible arc-variable pairs", stage_name);
        out.bus_failure_unlocalized = true;
        return finish_stage_solve_early(out, solve_begin);
    }

    log_mcf_model_graph(stage_name, graph, K);

    const auto arc_index = build_undirected_arc_index(graph);
    auto row_lo = std::vector<double> {};
    auto row_up = std::vector<double> {};
    auto row_meta = std::Vector<McfConstraintMeta> {};
    auto add_eq = [&](const double rhs, McfConstraintMeta meta) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(rhs);
        row_up.push_back(rhs);
        row_meta.push_back(std::move(meta));
        return id;
    };
    auto add_le = [&](const double rhs, McfConstraintMeta meta) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(-kGurobiInf);
        row_up.push_back(rhs);
        row_meta.push_back(std::move(meta));
        return id;
    };

    // BusMCF §3: node flow conservation
    auto flow_row = std::map<std::pair<int, int>, int> {};
    const auto ensure_flow_row = [&](const int k, const int n) -> int {
        const auto key = std::make_pair(k, n);
        if (flow_row.contains(key)) {
            return flow_row.at(key);
        }
        const auto row = add_eq(
            0.0,
            McfConstraintMeta {
                "flow_conservation",
                std::format(
                    "commodity={} node={} (unset)",
                    local_com[static_cast<std::size_t>(k)].label,
                    node_text(graph, n)),
                local_com[static_cast<std::size_t>(k)].bus_key,
                local_com[static_cast<std::size_t>(k)].record_index});
        flow_row[key] = row;
        return row;
    };

    // BusMCF §2: edge capacity Σ_n (f_ij + f_ji) <= capacity
    auto edge_row = std::map<std::pair<int, int>, int> {};
    for (const auto& key : collect_undirected_physical_edge_keys(graph, std::nullopt)) {
        edge_row[key] = add_le(
            1.0,
            McfConstraintMeta {
                "edge_capacity",
                std::format(
                    "rhs=1 {}",
                    describe_undirected_edge(graph, arc_index, key.first, key.second, graph.cols))});
    }

    auto f_entries = std::Vector<std::Vector<std::pair<int, double>>>(f_vars.size());
    auto incident_f = std::map<std::pair<int, int>, std::Vector<int>> {};
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto k = f_vars[j].k;
        const auto a = f_vars[j].a;
        const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
        f_entries[j].push_back({ensure_flow_row(k, arc.u), 1.0});
        f_entries[j].push_back({ensure_flow_row(k, arc.v), -1.0});
        if (!arc.is_virtual) {
            auto u = arc.u;
            auto v = arc.v;
            if (u > v) {
                std::swap(u, v);
            }
            f_entries[j].push_back({edge_row.at({u, v}), 1.0});
        }
        if (!graph.nodes[static_cast<std::size_t>(arc.u)].is_virtual) {
            incident_f[{k, arc.u}].push_back(static_cast<int>(j));
        }
        if (!graph.nodes[static_cast<std::size_t>(arc.v)].is_virtual) {
            incident_f[{k, arc.v}].push_back(static_cast<int>(j));
        }
    }

    for (int k = 0; k < K; ++k) {
        const auto s = local_com[static_cast<std::size_t>(k)].src;
        const auto t = local_com[static_cast<std::size_t>(k)].snk;
        const auto d = local_com[static_cast<std::size_t>(k)].demand;
        const auto rs = ensure_flow_row(k, s);
        const auto rt = ensure_flow_row(k, t);
        row_lo[static_cast<std::size_t>(rs)] = static_cast<double>(d);
        row_up[static_cast<std::size_t>(rs)] = static_cast<double>(d);
        row_lo[static_cast<std::size_t>(rt)] = static_cast<double>(-d);
        row_up[static_cast<std::size_t>(rt)] = static_cast<double>(-d);
        row_meta[static_cast<std::size_t>(rs)] = McfConstraintMeta {
            "flow_conservation",
            std::format(
                "commodity={} node={} rhs=+{} (source)",
                local_com[static_cast<std::size_t>(k)].label,
                node_text(graph, s),
                d),
            local_com[static_cast<std::size_t>(k)].bus_key,
            local_com[static_cast<std::size_t>(k)].record_index};
        row_meta[static_cast<std::size_t>(rt)] = McfConstraintMeta {
            "flow_conservation",
            std::format(
                "commodity={} node={} rhs=-{} (sink)",
                local_com[static_cast<std::size_t>(k)].label,
                node_text(graph, t),
                d),
            local_com[static_cast<std::size_t>(k)].bus_key,
            local_com[static_cast<std::size_t>(k)].record_index};
    }

    // BusMCF §5: f <= o, o <= Σ incident f, Σ_n o_i <= 1
    auto o_entries = std::Vector<std::Vector<std::pair<int, double>>> {};
    auto o_vars = std::Vector<OVar> {};
    auto node_row = std::map<int, int> {};
    auto physical_nodes_in_use = std::set<int> {};
    for (const auto& [kn, vars] : incident_f) {
        (void)vars;
        physical_nodes_in_use.insert(kn.second);
    }
    for (const auto n : physical_nodes_in_use) {
        if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
            continue;
        }
        node_row[n] = add_le(
            1.0,
            McfConstraintMeta {
                "node_capacity",
                std::format("node={} rhs=1", node_text(graph, n))});
    }
    for (const auto& [kn, vars] : incident_f) {
        const auto k = kn.first;
        const auto n = kn.second;
        if (vars.empty()) {
            continue;
        }
        const auto row_o_le_sum_f = add_le(
            0.0,
            McfConstraintMeta {
                "o_le_sum_f_link",
                std::format(
                    "commodity={} node={}",
                    local_com[static_cast<std::size_t>(k)].label,
                    node_text(graph, n)),
                local_com[static_cast<std::size_t>(k)].bus_key,
                local_com[static_cast<std::size_t>(k)].record_index});
        const auto row_link = add_le(
            0.0,
            McfConstraintMeta {
                "f_le_o_link",
                std::format(
                    "commodity={} node={}",
                    local_com[static_cast<std::size_t>(k)].label,
                    node_text(graph, n)),
                local_com[static_cast<std::size_t>(k)].bus_key,
                local_com[static_cast<std::size_t>(k)].record_index});
        for (const auto j : vars) {
            f_entries[static_cast<std::size_t>(j)].push_back({row_link, 1.0});
            f_entries[static_cast<std::size_t>(j)].push_back({row_o_le_sum_f, -1.0});
        }
        o_vars.push_back(OVar {k, n});
        auto col = std::Vector<std::pair<int, double>> {};
        col.push_back({row_link, -2.0});
        col.push_back({row_o_le_sum_f, 1.0});
        col.push_back({node_row.at(n), 1.0});
        o_entries.push_back(std::move(col));
    }

    // BusMCF §6 (第五版): sync equal length
    // total_flow_n = Σ_{(i,j)∈E^c} f^{c,n}_{ij},  ∀n∈Bus ∧ c = n 所在 COBUnit
    // E^c 由 arc_usable_for_unit(..., cob_unit) 限定；非虚拟弧求和即 total_flow_n
    // total_flow_n = total_flow_m,  ∀n,m ∈ the_same_Bus (bus_key)
    int bus_equal_length_rows = 0;
    auto by_bus = std::map<std::String, std::Vector<int>> {};
    for (int k = 0; k < K; ++k) {
        if (!local_com[static_cast<std::size_t>(k)].is_bus) {
            continue;
        }
        by_bus[local_com[static_cast<std::size_t>(k)].bus_key].push_back(k);
    }
    for (const auto& [key, group] : by_bus) {
        (void)key;
        if (group.size() <= 1) {
            continue;
        }
        const auto ref = group.front();
        for (std::size_t gi = 1; gi < group.size(); ++gi) {
            const auto row = add_eq(
                0.0,
                McfConstraintMeta {
                    "bus_equal_length",
                    std::format(
                        "bus_key={} ref={} cur={}",
                        key,
                        local_com[static_cast<std::size_t>(group.front())].label,
                        local_com[static_cast<std::size_t>(group[gi])].label),
                    key,
                    kInvalidRecordIndex});
            ++bus_equal_length_rows;
            const auto cur = group[gi];
            for (const auto j : f_by_k[static_cast<std::size_t>(cur)]) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[static_cast<std::size_t>(j)].a)];
                if (arc.is_virtual) {
                    continue;
                }
                f_entries[static_cast<std::size_t>(j)].push_back({row, 1.0});
            }
            for (const auto j : f_by_k[static_cast<std::size_t>(ref)]) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[static_cast<std::size_t>(j)].a)];
                if (arc.is_virtual) {
                    continue;
                }
                f_entries[static_cast<std::size_t>(j)].push_back({row, -1.0});
            }
        }
    }

    const auto num_f = static_cast<int>(f_vars.size());
    const auto num_o = static_cast<int>(o_vars.size());
    const auto num_col = num_f + num_o;
    const auto num_row = static_cast<int>(row_lo.size());

    log_mcf_constraint_rows(
        stage_name,
        {
            {"flow_conservation", static_cast<int>(flow_row.size())},
            {"edge_capacity", static_cast<int>(edge_row.size())},
            {"f_le_o_link", static_cast<int>(o_vars.size())},
            {"o_le_sum_f_link", static_cast<int>(o_vars.size())},
            {"node_capacity", static_cast<int>(node_row.size())},
            {"bus_equal_length", bus_equal_length_rows},
        });
    debug::info_fmt(
        "{} variables: f={} o={} cols={} rows={}",
        stage_name,
        num_f,
        num_o,
        num_col,
        num_row);

    // BusMCF objective: min Σ f on non-virtual arcs
    auto col_cost = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_lo = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_up = std::vector<double>(static_cast<std::size_t>(num_col), 1.0);
    auto col_entries = std::Vector<std::Vector<std::pair<int, double>>>(static_cast<std::size_t>(num_col));

    for (int j = 0; j < num_f; ++j) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[static_cast<std::size_t>(j)].a)];
        col_cost[static_cast<std::size_t>(j)] = arc.is_virtual ? 0.0 : 1.0;
        col_entries[static_cast<std::size_t>(j)] = f_entries[static_cast<std::size_t>(j)];
    }
    for (int j = 0; j < num_o; ++j) {
        const auto col = num_f + j;
        col_entries[static_cast<std::size_t>(col)] = o_entries[static_cast<std::size_t>(j)];
    }

    auto warm_values_by_col = std::map<int, double> {};
    if (warm_start != nullptr && !warm_start->nodes_by_record_id.empty()) {
        auto o_col_by_k_node = std::map<std::pair<int, int>, int> {};
        for (std::size_t oi = 0; oi < o_vars.size(); ++oi) {
            const auto& ov = o_vars[oi];
            o_col_by_k_node[{ov.k, ov.node}] = num_f + static_cast<int>(oi);
        }

        std::size_t matched_paths = 0;
        for (int k = 0; k < K; ++k) {
            const auto& commodity = local_com[static_cast<std::size_t>(k)];
            const auto path_it = warm_start->nodes_by_record_id.find(commodity.record_id);
            if (path_it == warm_start->nodes_by_record_id.end()) {
                continue;
            }
            const auto& path = path_it->second;
            if (path.size() < 2) {
                continue;
            }
            ++matched_paths;
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                const auto u = path[i];
                const auto v = path[i + 1];
                for (const auto f_col : f_by_k[static_cast<std::size_t>(k)]) {
                    const auto arc_id = f_vars[static_cast<std::size_t>(f_col)].a;
                    const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                    if (arc.u == u && arc.v == v) {
                        warm_values_by_col[f_col] = 1.0;
                        break;
                    }
                }
            }
            for (const auto node : path) {
                if (graph.nodes[static_cast<std::size_t>(node)].is_virtual) {
                    continue;
                }
                const auto it = o_col_by_k_node.find({k, node});
                if (it != o_col_by_k_node.end()) {
                    warm_values_by_col[it->second] = 1.0;
                }
            }
        }

        if (!warm_values_by_col.empty()) {
            debug::info_fmt(
                "{} warm start loaded for Gurobi: matched_paths={}, values={}",
                stage_name,
                matched_paths,
                warm_values_by_col.size());
        }
    }

    const auto cpp_model_build_ms = stage_solve_elapsed_ms(solve_begin);
    const auto solve_res = solve_binary_columns_with_gurobi(
        stage_name,
        col_cost,
        col_lo,
        col_up,
        row_lo,
        row_up,
        col_entries,
        warm_values_by_col,
        &row_meta,
        diag,
        gurobi_sink,
        gurobi_meta,
        gurobi_params);
    out.model_status = solve_res.model_status;
    auto timing = merge_gurobi_stage_timing(cpp_model_build_ms, solve_res.gurobi_timing);
    if (solve_res.solution_class == McfSolutionClass::Failed
        || solve_res.solution_class == McfSolutionClass::TimeLimit) {
        if (warm_start != nullptr && !warm_values_by_col.empty()) {
            debug::warning_fmt(
                "{} warm start led to {}; retrying without warm start",
                stage_name,
                solution_class_name(solve_res.solution_class));
            auto first_log = out;
            apply_stage_solution_class(first_log, solve_res.solution_class);
            first_log.model_status = solve_res.model_status;
            first_log.message = solve_res.message;
            first_log.infeasibility_hints = solve_res.iis_rows;
            collect_bus_retry_hints_from_iis(first_log, first_log.infeasibility_hints);
            first_log = finish_stage_solve_result(first_log, solve_begin, timing);
            auto retry_meta = gurobi_meta;
            retry_meta.warm_start = false;
            retry_meta.retry_kind = McfGurobiRetryKind::NoWarmStart;
            auto retry = solve_bus_mcf_component(
                graph,
                commodities,
                bus_ids,
                bbox_ctx,
                nullptr,
                diag,
                gurobi_sink,
                retry_meta,
                stage_name,
                gurobi_params);
            retry.solve_ms += first_log.solve_ms;
            return retry;
        }
        apply_stage_solution_class(out, solve_res.solution_class);
        out.message = solve_res.message;
        out.infeasibility_hints = solve_res.iis_rows;
        collect_bus_retry_hints_from_iis(out, out.infeasibility_hints);
        return finish_stage_solve_result(out, solve_begin, timing);
    }

    apply_stage_solution_class(out, solve_res.solution_class);
    out.message = "ok";
    out.objective = solve_res.objective;

    const auto extract_begin = std::chrono::steady_clock::now();
    auto f_values = std::Vector<int>(f_vars.size(), 0);
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        f_values[j] = static_cast<int>(std::lround(solve_res.col_value[j]));
        if (f_values[j] <= 0) {
            continue;
        }
        const auto k = f_vars[j].k;
        const auto unit = local_com[static_cast<std::size_t>(k)].cob_unit;
        const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[j].a)];
        if (!arc.is_virtual) {
            auto u = arc.u;
            auto v = arc.v;
            if (u > v) {
                std::swap(u, v);
            }
            out.used_edges[{u, v}] = 1;
            out.unit_used_edges[unit][{u, v}] = 1;
            out.used_nodes[arc.u] = 1;
            out.used_nodes[arc.v] = 1;
            out.unit_used_nodes[unit][arc.u] = 1;
            out.unit_used_nodes[unit][arc.v] = 1;
        }
    }

    append_paths_from_f_solution(stage_name, graph, local_com, f_vars, f_values, out);
    timing.extract_path_ms = elapsed_ms_between(extract_begin, std::chrono::steady_clock::now());
    return finish_stage_solve_result(out, solve_begin, timing);
}

auto solve_bus_mcf(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& bus_ids,
    const McfBBoxContext& bbox_ctx,
    const StageWarmStart* warm_start,
    const GurobiDiagnosticsOptions& diag,
    McfGurobiLogSink* gurobi_sink,
    const McfGurobiSolveMeta& gurobi_meta
) -> StageSolveResult {
    constexpr auto stage_name = "BusMCF";
    const auto solve_begin = std::chrono::steady_clock::now();
    if (bus_ids.empty()) {
        StageSolveResult out {};
        out.stage_name = stage_name;
        if (gurobi_sink != nullptr) {
            gurobi_sink->write_skipped(McfGurobiLogStage::bus(), "empty_stage");
        }
        apply_stage_solution_class(out, McfSolutionClass::Skipped);
        out.message = "empty stage";
        return finish_stage_solve_early(out, solve_begin);
    }

    const auto [per_vertex, vertex_labels] =
        preview_bus_candidate_resources(graph, commodities, bus_ids, bbox_ctx);
    const auto components = build_edge_node_conflict_components(per_vertex, vertex_labels);
    debug::info_fmt(
        "{} conflict decomposition: vertices={} components={}",
        stage_name,
        per_vertex.size(),
        components.size());

    if (components.size() <= 1) {
        auto params = default_mcf_gurobi_solve_params();
        params.threads = 8;
        auto meta = gurobi_meta;
        meta.component_count = 1;
        meta.component_id = 0;
        if (!vertex_labels.empty()) {
            meta.component_summary = vertex_labels.front();
            if (vertex_labels.size() > 1) {
                for (std::size_t i = 1; i < vertex_labels.size(); ++i) {
                    meta.component_summary += std::format(",{}", vertex_labels[i]);
                }
            }
        }
        return solve_bus_mcf_component(
            graph,
            commodities,
            bus_ids,
            bbox_ctx,
            warm_start,
            diag,
            gurobi_sink,
            meta,
            stage_name,
            params);
    }

    const auto component_count = static_cast<int>(components.size());
    auto params = default_mcf_gurobi_solve_params();
    params.threads = 4;
    auto cancel_flag = std::atomic<bool> {false};
    auto results = std::Vector<StageSolveResult>(components.size());
    auto results_mutex = std::mutex {};
    auto tasks = std::Vector<std::function<void()>> {};
    tasks.reserve(components.size());
    for (int ci = 0; ci < component_count; ++ci) {
        tasks.push_back([&, ci] {
            if (cancel_flag.load()) {
                return;
            }
            const auto& component = components[static_cast<std::size_t>(ci)];
            const auto subset_ids = bus_ids_for_conflict_component(bus_ids, commodities, vertex_labels, component);
            auto meta = gurobi_meta;
            meta.component_id = ci;
            meta.component_count = component_count;
            meta.component_summary = std::format("bus_keys={}", component.summary);
            const auto comp_stage =
                std::format("{}_c{}of{}", stage_name, ci, component_count);
            auto part = solve_bus_mcf_component(
                graph,
                commodities,
                subset_ids,
                bbox_ctx,
                warm_start,
                diag,
                gurobi_sink,
                meta,
                comp_stage,
                params);
            {
                const std::lock_guard lock {results_mutex};
                results[static_cast<std::size_t>(ci)] = std::move(part);
                if (!results[static_cast<std::size_t>(ci)].ok) {
                    cancel_flag.store(true);
                }
            }
        });
    }
    run_mcf_parallel_waves(mcf_gurobi_thread_budget_instance(), params.threads, tasks, &cancel_flag);
    return merge_stage_solve_results(stage_name, std::move(results), solve_begin);
}

auto residual_edge_cap(
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::pair<int, int>& e
) -> int {
    const auto it = edge_capacity_override.find(e);
    return it != edge_capacity_override.end() ? it->second : 1;
}

auto residual_node_cap(const std::map<int, int>& node_capacity_override, const int n) -> int {
    const auto it = node_capacity_override.find(n);
    return it != node_capacity_override.end() ? it->second : 1;
}

auto build_origin_group_physical_endpoints(
    const McfOriginGroup& group,
    const std::Vector<PreparedCommodity>& local_com,
    const GlobalGraph& graph
) -> std::set<int> {
    auto out = std::set<int> {};
    for (const auto k : group.commodity_local_indices) {
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        for (const auto n : {commodity.src, commodity.snk}) {
            if (n < 0 || static_cast<std::size_t>(n) >= graph.nodes.size()) {
                continue;
            }
            if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
                continue;
            }
            out.insert(n);
        }
    }
    return out;
}

auto arc_blocked_by_bus_residual(
    const Arc& arc,
    const GlobalGraph& graph,
    const std::set<int>& allowed_endpoint_nodes,
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::map<int, int>& node_capacity_override
) -> bool {
    if (arc.is_virtual) {
        return false;
    }
    const auto e = normalized_edge_key(arc.u, arc.v);
    if (residual_edge_cap(edge_capacity_override, e) == 0) {
        return true;
    }
    for (const auto n : {arc.u, arc.v}) {
        if (n < 0 || static_cast<std::size_t>(n) >= graph.nodes.size()) {
            continue;
        }
        if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
            continue;
        }
        if (allowed_endpoint_nodes.contains(n)) {
            continue;
        }
        if (residual_node_cap(node_capacity_override, n) == 0) {
            return true;
        }
    }
    return false;
}

auto commodity_residual_connected(
    const GlobalGraph& graph,
    const PreparedCommodity& commodity,
    const McfCommodityBBox& effective_bbox,
    const std::set<int>& allowed_endpoint_nodes,
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::map<int, int>& node_capacity_override
) -> bool {
    auto prev = std::vector<int>(graph.nodes.size(), -1);
    auto q = std::queue<int> {};
    q.push(commodity.src);
    prev[static_cast<std::size_t>(commodity.src)] = commodity.src;
    while (!q.empty()) {
        const auto node = q.front();
        q.pop();
        if (node == commodity.snk) {
            return true;
        }
        for (const auto& arc : graph.arcs) {
            if (arc.u != node) {
                continue;
            }
            if (!arc_usable_for_unit(graph, arc, commodity.cob_unit)) {
                continue;
            }
            if (effective_bbox.restricted
                && !arc_allowed_in_mcf_bbox(
                    arc,
                    graph.nodes[static_cast<std::size_t>(arc.u)],
                    graph.nodes[static_cast<std::size_t>(arc.v)],
                    true,
                    effective_bbox.box,
                    graph.cols,
                    commodity.src,
                    commodity.snk)) {
                continue;
            }
            if (arc_blocked_by_bus_residual(
                    arc, graph, allowed_endpoint_nodes, edge_capacity_override, node_capacity_override)) {
                continue;
            }
            if (prev[static_cast<std::size_t>(arc.v)] != -1) {
                continue;
            }
            prev[static_cast<std::size_t>(arc.v)] = node;
            q.push(arc.v);
        }
    }
    return false;
}

auto fail_simple_mcf_early(
    StageSolveResult& out,
    const std::chrono::steady_clock::time_point begin,
    const std::String& stub_reason,
    const std::String& message,
    const std::size_t record_index,
    const Net_cost_record& record,
    const std::size_t unit_c,
    McfGurobiLogSink* gurobi_sink,
    const McfGurobiSolveMeta& gurobi_meta,
    const std::String& stub_detail = {}
) -> StageSolveResult {
    if (gurobi_sink != nullptr) {
        gurobi_sink->write_stub(
            gurobi_meta,
            stub_reason,
            stub_detail.empty() ? message : stub_detail);
    }
    apply_stage_solution_class(out, McfSolutionClass::Failed);
    out.message = message;
    append_unique(out.failed_record_indices, record_index);
    append_simple_origin_retry_hint(out, unit_c, simple_origin_group_key(record));
    return finish_stage_solve_early(out, begin);
}

auto preview_simple_candidate_resources(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& bbox_ctx,
    const McfBBoxExpandState* expand_state,
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::map<int, int>& node_capacity_override
) -> std::pair<std::Vector<McfCandidateResources>, std::Vector<std::String>> {
    const auto local_com = build_local_commodities(commodities, simple_ids_for_unit);
    const auto K = static_cast<int>(simple_ids_for_unit.size());
    const auto A = static_cast<int>(graph.arcs.size());
    auto origin_groups = build_origin_groups(local_com, records);
    auto commodity_origin_h = std::Vector<int>(static_cast<std::size_t>(K), -1);
    auto origin_bbox_by_gid = std::map<int, McfCommodityBBox> {};
    auto origin_endpoints_by_h = std::map<int, std::set<int>> {};
    auto gid_to_vertex = std::map<int, int> {};
    auto per_vertex = std::Vector<McfCandidateResources> {};
    auto labels = std::Vector<std::String> {};
    for (const auto& group : origin_groups) {
        gid_to_vertex[group.origin_group_id] = static_cast<int>(per_vertex.size());
        per_vertex.emplace_back();
        labels.push_back(group.origin_key);
        for (const auto k : group.commodity_local_indices) {
            commodity_origin_h[static_cast<std::size_t>(k)] = group.origin_group_id;
        }
        origin_endpoints_by_h[group.origin_group_id] =
            build_origin_group_physical_endpoints(group, local_com, graph);
        const auto group_key = std::make_pair(unit_c, group.origin_key);
        if (expand_state != nullptr) {
            if (const auto overlay = lookup_simple_origin_hull(*expand_state, group_key)) {
                origin_bbox_by_gid[group.origin_group_id] = McfCommodityBBox {true, *overlay};
                continue;
            }
        }
        if (group.is_multi_fanout) {
            auto global_ids = std::Vector<std::size_t> {};
            auto record_indices = std::Vector<std::size_t> {};
            for (const auto k : group.commodity_local_indices) {
                global_ids.push_back(simple_ids_for_unit[static_cast<std::size_t>(k)]);
                record_indices.push_back(local_com[static_cast<std::size_t>(k)].record_index);
            }
            origin_bbox_by_gid[group.origin_group_id] =
                compute_origin_group_bbox(global_ids, record_indices, records, bbox_ctx);
        }
    }
    for (int k = 0; k < K; ++k) {
        const auto global_id = simple_ids_for_unit[static_cast<std::size_t>(k)];
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        const auto mode = origin_bbox_by_gid.contains(h) ? McfArcBBoxMode::SimpleOriginGroup
                                                         : McfArcBBoxMode::SimpleCommodity;
        const auto origin_bbox = origin_bbox_by_gid.contains(h) ? origin_bbox_by_gid.at(h) : McfCommodityBBox {};
        const auto& allowed_endpoint_nodes = origin_endpoints_by_h.at(h);
        const auto vid = gid_to_vertex.at(h);
        for (int a = 0; a < A; ++a) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
            if (!arc_allowed_for_commodity(
                    graph,
                    arc,
                    commodity,
                    bbox_ctx,
                    global_id,
                    mode,
                    origin_bbox)) {
                continue;
            }
            if (arc_blocked_by_bus_residual(
                    arc, graph, allowed_endpoint_nodes, edge_capacity_override, node_capacity_override)) {
                continue;
            }
            if (arc.is_virtual) {
                continue;
            }
            auto u = arc.u;
            auto v = arc.v;
            if (u > v) {
                std::swap(u, v);
            }
            per_vertex[static_cast<std::size_t>(vid)].physical_edges.insert({u, v});
            per_vertex[static_cast<std::size_t>(vid)].physical_nodes.insert(arc.u);
            per_vertex[static_cast<std::size_t>(vid)].physical_nodes.insert(arc.v);
        }
    }
    return {per_vertex, labels};
}

auto unit_has_multi_fanout_origin(
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<Net_cost_record>& records
) -> bool {
    if (simple_ids_for_unit.size() < 2) {
        return false;
    }
    auto local_com = build_local_commodities(commodities, simple_ids_for_unit);
    const auto groups = build_origin_groups(local_com, records);
    for (const auto& group : groups) {
        if (group.is_multi_fanout) {
            return true;
        }
    }
    return false;
}

auto preview_simple_refine_candidate_resources(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& bbox_ctx,
    const McfBBoxExpandState* expand_state,
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::map<int, int>& node_capacity_override
) -> std::pair<std::Vector<McfCandidateResources>, std::Vector<std::String>> {
    const auto local_com = build_local_commodities(commodities, simple_ids_for_unit);
    const auto K = static_cast<int>(simple_ids_for_unit.size());
    const auto A = static_cast<int>(graph.arcs.size());
    auto origin_groups = build_origin_groups(local_com, records);
    auto commodity_origin_h = std::Vector<int>(static_cast<std::size_t>(K), -1);
    auto origin_endpoints_by_h = std::map<int, std::set<int>> {};
    auto origin_bbox_by_gid = std::map<int, McfCommodityBBox> {};
    for (const auto& group : origin_groups) {
        for (const auto k : group.commodity_local_indices) {
            commodity_origin_h[static_cast<std::size_t>(k)] = group.origin_group_id;
        }
        origin_endpoints_by_h[group.origin_group_id] =
            build_origin_group_physical_endpoints(group, local_com, graph);
        const auto group_key = std::make_pair(unit_c, group.origin_key);
        if (expand_state != nullptr) {
            if (const auto overlay = lookup_simple_origin_hull(*expand_state, group_key)) {
                origin_bbox_by_gid[group.origin_group_id] = McfCommodityBBox {true, *overlay};
            }
        }
        if (!group.is_multi_fanout || group.commodity_local_indices.empty()) {
            continue;
        }
        bool all_refined = true;
        for (const auto k : group.commodity_local_indices) {
            if (!local_com[static_cast<std::size_t>(k)].is_refined_segment) {
                all_refined = false;
                break;
            }
        }
        if (!all_refined) {
            auto global_ids = std::Vector<std::size_t> {};
            auto record_indices = std::Vector<std::size_t> {};
            for (const auto k : group.commodity_local_indices) {
                const auto& sc = local_com[static_cast<std::size_t>(k)];
                const auto bbox_gid =
                    sc.bbox_global_commodity_id != std::numeric_limits<std::size_t>::max()
                    ? sc.bbox_global_commodity_id
                    : simple_ids_for_unit[static_cast<std::size_t>(k)];
                global_ids.push_back(bbox_gid);
                record_indices.push_back(sc.record_index);
            }
            origin_bbox_by_gid[group.origin_group_id] =
                compute_origin_group_bbox(global_ids, record_indices, records, bbox_ctx);
        }
    }

    auto per_vertex = std::Vector<McfCandidateResources>(static_cast<std::size_t>(K));
    auto labels = std::Vector<std::String> {};
    labels.reserve(static_cast<std::size_t>(K));
    for (int k = 0; k < K; ++k) {
        const auto global_id = simple_ids_for_unit[static_cast<std::size_t>(k)];
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        labels.push_back(commodity.synthetic_label.empty() ? commodity.label : commodity.synthetic_label);
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        McfArcBBoxMode mode = McfArcBBoxMode::SimpleCommodity;
        McfCommodityBBox arc_bbox {};
        if (commodity.is_refined_segment && commodity.bbox_override.has_value() && commodity.bbox_override->restricted) {
            mode = McfArcBBoxMode::SimpleExplicit;
            arc_bbox = *commodity.bbox_override;
        }
        else if (origin_bbox_by_gid.contains(h)) {
            mode = McfArcBBoxMode::SimpleOriginGroup;
            arc_bbox = origin_bbox_by_gid.at(h);
        }
        else {
            mode = McfArcBBoxMode::SimpleExplicit;
            arc_bbox = effective_bbox_for_simple_commodity(
                global_id,
                simple_ids_for_unit,
                commodities,
                records,
                bbox_ctx,
                expand_state,
                &commodity);
        }
        const auto& allowed_endpoint_nodes = origin_endpoints_by_h.at(h);
        for (int a = 0; a < A; ++a) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
            if (!arc_allowed_for_commodity(
                    graph,
                    arc,
                    commodity,
                    bbox_ctx,
                    global_id,
                    mode,
                    arc_bbox)) {
                continue;
            }
            if (arc_blocked_by_bus_residual(
                    arc, graph, allowed_endpoint_nodes, edge_capacity_override, node_capacity_override)) {
                continue;
            }
            if (arc.is_virtual) {
                continue;
            }
            auto u = arc.u;
            auto v = arc.v;
            if (u > v) {
                std::swap(u, v);
            }
            per_vertex[static_cast<std::size_t>(k)].physical_edges.insert({u, v});
            per_vertex[static_cast<std::size_t>(k)].physical_nodes.insert(arc.u);
            per_vertex[static_cast<std::size_t>(k)].physical_nodes.insert(arc.v);
        }
    }
    return {per_vertex, labels};
}

auto build_refined_unit_commodity_list(
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    const std::size_t unit_c,
    const UnitTreeCompressSummary& compress_summary
) -> std::pair<std::Vector<PreparedCommodity>, std::Vector<std::size_t>> {
    auto refined_origins = std::map<std::String, const OriginTreeCompressResult*> {};
    for (const auto& origin : compress_summary.per_origin) {
        if (!origin.fallback) {
            refined_origins[origin.origin_key] = &origin;
        }
    }

    auto local_com = build_local_commodities(commodities, simple_ids_for_unit);
    auto origin_groups = build_origin_groups(local_com, records);
    auto out_com = std::Vector<PreparedCommodity> {};
    auto out_ids = std::Vector<std::size_t> {};
  out_com.reserve(local_com.size());
    std::size_t segment_counter = 0;
    auto handled_origins = std::set<std::String> {};

    for (const auto& group : origin_groups) {
        const auto refine_it = refined_origins.find(group.origin_key);
        if (group.is_multi_fanout && refine_it != refined_origins.end() && refine_it->second != nullptr) {
            if (handled_origins.contains(group.origin_key)) {
                continue;
            }
            handled_origins.insert(group.origin_key);
            const auto& origin_result = *refine_it->second;
            for (const auto& seg : origin_result.segments) {
                const auto& template_com = local_com[static_cast<std::size_t>(group.commodity_local_indices.front())];
                PreparedCommodity c = template_com;
                c.is_refined_segment = true;
                c.src = seg.src;
                c.snk = seg.snk;
                c.guide_path = seg.guide_path;
                c.bbox_override = seg.bbox;
                c.record_id = mcf_synthetic_record_id(unit_c, segment_counter);
                c.record_indices = seg.parent_record_indices;
                c.synthetic_label =
                    std::format("{}_seg{}_{}_{}", c.origin_uid, segment_counter, seg.src, seg.snk);
                c.label = c.synthetic_label;
                ++segment_counter;
                c.start_track = template_com.start_track;
                c.end_track = template_com.end_track;
                c.bbox_global_commodity_id =
                    simple_ids_for_unit[static_cast<std::size_t>(group.commodity_local_indices.front())];
                out_com.push_back(std::move(c));
                out_ids.push_back(out_com.size() - 1);
            }
            continue;
        }
        for (const auto k : group.commodity_local_indices) {
            auto c = local_com[static_cast<std::size_t>(k)];
            c.bbox_global_commodity_id = simple_ids_for_unit[static_cast<std::size_t>(k)];
            out_com.push_back(std::move(c));
            out_ids.push_back(out_com.size() - 1);
        }
    }
    return {out_com, out_ids};
}

auto simple_ids_for_refine_conflict_component(
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::String>& vertex_labels,
    const McfConflictComponent& component
) -> std::Vector<std::size_t> {
    auto keys = std::set<std::String> {};
    for (const auto vid : component.vertex_ids) {
        if (vid >= 0 && static_cast<std::size_t>(vid) < vertex_labels.size()) {
            keys.insert(vertex_labels[static_cast<std::size_t>(vid)]);
        }
    }
    auto out = std::Vector<std::size_t> {};
    for (const auto gid : simple_ids_for_unit) {
        if (gid >= commodities.size()) {
            continue;
        }
        const auto& com = commodities[gid];
        const auto label = com.synthetic_label.empty() ? com.label : com.synthetic_label;
        if (keys.contains(label)) {
            out.push_back(gid);
        }
    }
    return out;
}

auto simple_ids_for_conflict_component(
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<Net_cost_record>& records,
    const std::Vector<std::String>& vertex_labels,
    const McfConflictComponent& component
) -> std::Vector<std::size_t> {
    auto keys = std::set<std::String> {};
    for (const auto vid : component.vertex_ids) {
        if (vid >= 0 && static_cast<std::size_t>(vid) < vertex_labels.size()) {
            keys.insert(vertex_labels[static_cast<std::size_t>(vid)]);
        }
    }
    auto out = std::Vector<std::size_t> {};
    for (const auto gid : simple_ids_for_unit) {
        if (gid >= commodities.size()) {
            continue;
        }
        const auto& rec = records[commodities[gid].record_index];
        if (keys.contains(simple_origin_group_key(rec))) {
            out.push_back(gid);
        }
    }
    return out;
}

auto solve_simple_mcf_component(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& bbox_ctx,
    const McfBBoxExpandState* expand_state,
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::map<int, int>& node_capacity_override,
    const bool enable_mcf_obj,
    const StageWarmStart* warm_start,
    const GurobiDiagnosticsOptions& diag,
    McfGurobiLogSink* gurobi_sink,
    McfGurobiSolveMeta gurobi_meta,
    const std::String& stage_name,
    const McfGurobiSolveParams& gurobi_params,
    const StageRefineIncumbentSeed* refine_incumbent_seed = nullptr,
    const bool refine_pass = false
) -> StageSolveResult {
    const auto solve_begin = std::chrono::steady_clock::now();
    StageSolveResult out {};
    out.stage_name = stage_name;
    if (simple_ids_for_unit.empty()) {
        if (gurobi_sink != nullptr) {
            gurobi_sink->write_skipped(McfGurobiLogStage::simple_unit(unit_c), "empty_stage");
        }
        apply_stage_solution_class(out, McfSolutionClass::Skipped);
        out.message = "empty stage";
        return finish_stage_solve_early(out, solve_begin);
    }

    const auto K = static_cast<int>(simple_ids_for_unit.size());
    const auto A = static_cast<int>(graph.arcs.size());
    const auto local_com = build_local_commodities(commodities, simple_ids_for_unit);

    auto origin_groups = build_origin_groups(local_com, records);
    log_origin_groups(stage_name, origin_groups);
    auto commodity_origin_h = std::Vector<int>(static_cast<std::size_t>(K), -1);
    auto origin_endpoints_by_h = std::map<int, std::set<int>> {};
    auto origin_bbox_by_gid = std::map<int, McfCommodityBBox> {};
    for (const auto& group : origin_groups) {
        for (const auto k : group.commodity_local_indices) {
            commodity_origin_h[static_cast<std::size_t>(k)] = group.origin_group_id;
        }
        origin_endpoints_by_h[group.origin_group_id] =
            build_origin_group_physical_endpoints(group, local_com, graph);
        const auto group_key = std::make_pair(unit_c, group.origin_key);
        if (expand_state != nullptr) {
            if (const auto overlay = lookup_simple_origin_hull(*expand_state, group_key)) {
                origin_bbox_by_gid[group.origin_group_id] = McfCommodityBBox {true, *overlay};
                continue;
            }
        }
        if (group.is_multi_fanout) {
            bool all_refined = true;
            for (const auto k : group.commodity_local_indices) {
                if (!local_com[static_cast<std::size_t>(k)].is_refined_segment) {
                    all_refined = false;
                    break;
                }
            }
            if (all_refined) {
                continue;
            }
            auto global_ids = std::Vector<std::size_t> {};
            auto record_indices = std::Vector<std::size_t> {};
            for (const auto k : group.commodity_local_indices) {
                const auto& sc = local_com[static_cast<std::size_t>(k)];
                const auto bbox_gid =
                    sc.bbox_global_commodity_id != std::numeric_limits<std::size_t>::max()
                    ? sc.bbox_global_commodity_id
                    : simple_ids_for_unit[static_cast<std::size_t>(k)];
                global_ids.push_back(bbox_gid);
                record_indices.push_back(sc.record_index);
            }
            origin_bbox_by_gid[group.origin_group_id] =
                compute_origin_group_bbox(global_ids, record_indices, records, bbox_ctx);
        }
    }

    for (int k = 0; k < K; ++k) {
        const auto global_id = simple_ids_for_unit[static_cast<std::size_t>(k)];
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        const auto effective = effective_bbox_for_simple_commodity(
            global_id, simple_ids_for_unit, commodities, records, bbox_ctx, expand_state, &commodity);
        if (effective.restricted && !commodity_bbox_connected(graph, commodity, effective)) {
            if (gurobi_sink != nullptr) {
                gurobi_sink->write_stub(
                    gurobi_meta,
                    "bbox_disconnected",
                    std::format("commodity={}", commodity.label));
            }
            apply_stage_solution_class(out, McfSolutionClass::Failed);
            out.message = std::format(
                "{}: bbox disconnected for commodity {}",
                stage_name,
                commodity.label);
            append_unique(out.failed_record_indices, commodity.record_index);
            append_simple_origin_retry_hint(
                out,
                unit_c,
                simple_origin_group_key(records[commodity.record_index]));
            return finish_stage_solve_early(out, solve_begin);
        }
    }

    for (int k = 0; k < K; ++k) {
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        const auto& record = records[commodity.record_index];
        for (const auto n : {commodity.src, commodity.snk}) {
            if (n < 0 || static_cast<std::size_t>(n) >= graph.nodes.size()) {
                continue;
            }
            if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
                continue;
            }
            if (residual_node_cap(node_capacity_override, n) != 0) {
                continue;
            }
            return fail_simple_mcf_early(
                out,
                solve_begin,
                "endpoint_residual_zero",
                std::format(
                    "{}: endpoint node bus residual=0 for commodity {} node={} origin={}",
                    stage_name,
                    commodity.label,
                    node_text(graph, n),
                    commodity.origin_name),
                commodity.record_index,
                record,
                unit_c,
                gurobi_sink,
                gurobi_meta,
                std::format(
                    "commodity={} node={} origin={} bus_residual=0",
                    commodity.label,
                    node_text(graph, n),
                    commodity.origin_name));
        }
    }

    // SimpleMCF §1: f^{c,n}_{ij} variables
    auto f_vars = std::Vector<ArcVar> {};
    auto f_by_k = std::Vector<std::Vector<int>>(static_cast<std::size_t>(K));
    f_vars.reserve(static_cast<std::size_t>(K * A / 8 + 1));
    for (int k = 0; k < K; ++k) {
        const auto global_id = simple_ids_for_unit[static_cast<std::size_t>(k)];
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        McfArcBBoxMode mode = McfArcBBoxMode::SimpleCommodity;
        McfCommodityBBox arc_bbox {};
        if (commodity.is_refined_segment && commodity.bbox_override.has_value() && commodity.bbox_override->restricted) {
            mode = McfArcBBoxMode::SimpleExplicit;
            arc_bbox = *commodity.bbox_override;
        }
        else if (origin_bbox_by_gid.contains(h)) {
            mode = McfArcBBoxMode::SimpleOriginGroup;
            arc_bbox = origin_bbox_by_gid.at(h);
        }
        else {
            mode = McfArcBBoxMode::SimpleExplicit;
            arc_bbox = effective_bbox_for_simple_commodity(
                global_id, simple_ids_for_unit, commodities, records, bbox_ctx, expand_state, &commodity);
        }
        const auto& allowed_endpoint_nodes = origin_endpoints_by_h.at(h);
        for (int a = 0; a < A; ++a) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
            if (!arc_allowed_for_commodity(
                    graph,
                    arc,
                    commodity,
                    bbox_ctx,
                    global_id,
                    mode,
                    arc_bbox)) {
                continue;
            }
            if (arc_blocked_by_bus_residual(
                    arc, graph, allowed_endpoint_nodes, edge_capacity_override, node_capacity_override)) {
                continue;
            }
            const auto var_id = static_cast<int>(f_vars.size());
            f_vars.push_back(ArcVar {k, a});
            f_by_k[static_cast<std::size_t>(k)].push_back(var_id);
        }
    }

    for (int k = 0; k < K; ++k) {
        const auto global_id = simple_ids_for_unit[static_cast<std::size_t>(k)];
        const auto& commodity = local_com[static_cast<std::size_t>(k)];
        const auto& record = records[commodity.record_index];
        if (f_by_k[static_cast<std::size_t>(k)].empty()) {
            return fail_simple_mcf_early(
                out,
                solve_begin,
                "no_feasible_arc_variable_pairs",
                std::format(
                    "{}: no feasible arc-variable pairs for commodity {}",
                    stage_name,
                    commodity.label),
                commodity.record_index,
                record,
                unit_c,
                gurobi_sink,
                gurobi_meta,
                std::format("commodity={}", commodity.label));
        }
        const auto effective = effective_bbox_for_simple_commodity(
            global_id, simple_ids_for_unit, commodities, records, bbox_ctx, expand_state, &commodity);
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        const auto& allowed_endpoint_nodes = origin_endpoints_by_h.at(h);
        if (!commodity_residual_connected(
                graph,
                commodity,
                effective,
                allowed_endpoint_nodes,
                edge_capacity_override,
                node_capacity_override)) {
            return fail_simple_mcf_early(
                out,
                solve_begin,
                "residual_disconnected",
                std::format(
                    "{}: bus residual disconnected for commodity {}",
                    stage_name,
                    commodity.label),
                commodity.record_index,
                record,
                unit_c,
                gurobi_sink,
                gurobi_meta,
                std::format("commodity={}", commodity.label));
        }
    }

    if (f_vars.empty()) {
        if (gurobi_sink != nullptr) {
            gurobi_sink->write_stub(gurobi_meta, "no_feasible_arc_variable_pairs");
        }
        apply_stage_solution_class(out, McfSolutionClass::Failed);
        out.message = std::format("{}: no feasible arc-variable pairs", stage_name);
        return finish_stage_solve_early(out, solve_begin);
    }

    log_mcf_model_graph(stage_name, graph, K, unit_c);

    const auto arc_index = build_undirected_arc_index(graph);
    auto origin_label_by_h = std::map<int, std::String> {};
    for (const auto& group : origin_groups) {
        origin_label_by_h[group.origin_group_id] = group.origin_key;
    }
    const auto origin_label = [&](const int h) -> std::String {
        const auto it = origin_label_by_h.find(h);
        return it != origin_label_by_h.end() ? it->second : std::format("origin_h{}", h);
    };
    const auto simple_meta = [&](
                                 std::String kind,
                                 std::String detail,
                                 const int h,
                                 const std::size_t record_index = kInvalidRecordIndex
                             ) -> McfConstraintMeta {
        return McfConstraintMeta {
            std::move(kind),
            std::move(detail),
            {},
            record_index,
            static_cast<int>(unit_c),
            origin_label(h)};
    };

    auto row_lo = std::vector<double> {};
    auto row_up = std::vector<double> {};
    auto row_meta = std::Vector<McfConstraintMeta> {};
    auto add_eq = [&](const double rhs, McfConstraintMeta meta) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(rhs);
        row_up.push_back(rhs);
        row_meta.push_back(std::move(meta));
        return id;
    };
    auto add_le = [&](const double rhs, McfConstraintMeta meta) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(-kGurobiInf);
        row_up.push_back(rhs);
        row_meta.push_back(std::move(meta));
        return id;
    };

    // SimpleMCF §3: node flow conservation on f
    auto flow_row = std::map<std::pair<int, int>, int> {};
    const auto ensure_flow_row = [&](const int k, const int n) -> int {
        const auto key = std::make_pair(k, n);
        if (flow_row.contains(key)) {
            return flow_row.at(key);
        }
        const auto row = add_eq(
            0.0,
            simple_meta(
                "flow_conservation",
                std::format(
                    "commodity={} node={} (unset)",
                    local_com[static_cast<std::size_t>(k)].label,
                    node_text(graph, n)),
                commodity_origin_h[static_cast<std::size_t>(k)],
                local_com[static_cast<std::size_t>(k)].record_index));
        flow_row[key] = row;
        return row;
    };

    // SimpleMCF v5 §5: Σ_H x^H_e <= capacity^c_e - used^{Bus,c}_e (lazy per used edge)
    auto edge_row = std::map<std::pair<int, int>, int> {};
    const auto ensure_edge_row = [&](const std::pair<int, int>& e) -> std::optional<int> {
        if (edge_row.contains(e)) {
            return edge_row.at(e);
        }
        const auto cap = residual_edge_cap(edge_capacity_override, e);
        if (cap == 0) {
            return std::nullopt;
        }
        edge_row[e] = add_le(
            static_cast<double>(cap),
            McfConstraintMeta {
                "edge_capacity",
                std::format(
                    "rhs={} bus_residual={} {}",
                    cap,
                    cap,
                    describe_undirected_edge(graph, arc_index, e.first, e.second, graph.cols))});
        return edge_row[e];
    };

    auto f_entries = std::Vector<std::Vector<std::pair<int, double>>>(f_vars.size());
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto k = f_vars[j].k;
        const auto a = f_vars[j].a;
        const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
        f_entries[j].push_back({ensure_flow_row(k, arc.u), 1.0});
        f_entries[j].push_back({ensure_flow_row(k, arc.v), -1.0});
    }

    for (int k = 0; k < K; ++k) {
        const auto s = local_com[static_cast<std::size_t>(k)].src;
        const auto t = local_com[static_cast<std::size_t>(k)].snk;
        const auto d = local_com[static_cast<std::size_t>(k)].demand;
        const auto rs = ensure_flow_row(k, s);
        const auto rt = ensure_flow_row(k, t);
        row_lo[static_cast<std::size_t>(rs)] = static_cast<double>(d);
        row_up[static_cast<std::size_t>(rs)] = static_cast<double>(d);
        row_lo[static_cast<std::size_t>(rt)] = static_cast<double>(-d);
        row_up[static_cast<std::size_t>(rt)] = static_cast<double>(-d);
        row_meta[static_cast<std::size_t>(rs)] = McfConstraintMeta {
            "flow_conservation",
            std::format(
                "commodity={} node={} rhs=+{} (source)",
                local_com[static_cast<std::size_t>(k)].label,
                node_text(graph, s),
                d),
            {},
            local_com[static_cast<std::size_t>(k)].record_index,
            static_cast<int>(unit_c),
            origin_label(commodity_origin_h[static_cast<std::size_t>(k)])};
        row_meta[static_cast<std::size_t>(rt)] = McfConstraintMeta {
            "flow_conservation",
            std::format(
                "commodity={} node={} rhs=-{} (sink)",
                local_com[static_cast<std::size_t>(k)].label,
                node_text(graph, t),
                d),
            {},
            local_com[static_cast<std::size_t>(k)].record_index,
            static_cast<int>(unit_c),
            origin_label(commodity_origin_h[static_cast<std::size_t>(k)])};
    }

    // SimpleMCF v5 §1-2: x^{c,H}_e on undirected physical edges e
    using UndirectedEdgeKey = std::pair<int, int>;
    using OriginEdgeKey = std::pair<int, UndirectedEdgeKey>;
    auto origin_x_vars = std::Vector<OriginEdgeVar> {};
    auto origin_x_entries = std::Vector<std::Vector<std::pair<int, double>>> {};
    auto origin_x_by_he = std::map<OriginEdgeKey, int> {};
    for (const auto& group : origin_groups) {
        const int h = group.origin_group_id;
        auto edge_keys = std::set<UndirectedEdgeKey> {};
        for (const auto k : group.commodity_local_indices) {
            for (const auto f_var : f_by_k[static_cast<std::size_t>(k)]) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[static_cast<std::size_t>(f_var)].a)];
                if (arc.is_virtual) {
                    continue;
                }
                edge_keys.insert(normalized_edge_key(arc.u, arc.v));
            }
        }
        for (const auto& e : edge_keys) {
            const auto he_key = OriginEdgeKey {h, e};
            if (origin_x_by_he.contains(he_key)) {
                continue;
            }
            const auto edge_row_id = ensure_edge_row(e);
            if (!edge_row_id.has_value()) {
                continue;
            }
            const auto var_id = static_cast<int>(origin_x_vars.size());
            origin_x_vars.push_back(OriginEdgeVar {h, e.first, e.second});
            origin_x_by_he[he_key] = var_id;
            origin_x_entries.emplace_back();
            origin_x_entries[static_cast<std::size_t>(var_id)].push_back({*edge_row_id, 1.0});
        }
    }

    // v5 §2 lower: f^{n}_{ij} + f^{n}_{ji} <= x^H_e (per commodity on undirected edge)
    using CommodityEdgeKey = std::pair<int, UndirectedEdgeKey>;
    auto f_indices_by_ke = std::map<CommodityEdgeKey, std::Vector<int>> {};
    auto f_indices_by_he = std::map<OriginEdgeKey, std::Vector<int>> {};
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto k = f_vars[j].k;
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[j].a)];
        if (arc.is_virtual) {
            continue;
        }
        const auto e = normalized_edge_key(arc.u, arc.v);
        f_indices_by_ke[{k, e}].push_back(static_cast<int>(j));
        f_indices_by_he[{h, e}].push_back(static_cast<int>(j));
    }

    int f_le_x_lower_rows = 0;
    for (const auto& [ke, f_list] : f_indices_by_ke) {
        const auto k = ke.first;
        const auto& e = ke.second;
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        const auto x_it = origin_x_by_he.find({h, e});
        if (x_it == origin_x_by_he.end()) {
            continue;
        }
        const auto row = add_le(
            0.0,
            simple_meta(
                "f_le_x_lower",
                std::format(
                    "origin={} commodity={} {}",
                    origin_label(h),
                    local_com[static_cast<std::size_t>(k)].label,
                    describe_undirected_edge(graph, arc_index, e.first, e.second, graph.cols)),
                h,
                local_com[static_cast<std::size_t>(k)].record_index));
        ++f_le_x_lower_rows;
        for (const auto f_j : f_list) {
            f_entries[static_cast<std::size_t>(f_j)].push_back({row, 1.0});
        }
        origin_x_entries[static_cast<std::size_t>(x_it->second)].push_back({row, -1.0});
    }

    // v5 §2 upper: x^H_e <= Σ_{n∈H.child} (f^n_ij + f^n_ji)
    int f_le_x_upper_rows = 0;
    for (const auto& [he_key, x_var] : origin_x_by_he) {
        const auto f_list = f_indices_by_he[he_key];
        if (f_list.empty()) {
            continue;
        }
        const auto row = add_le(
            0.0,
            simple_meta(
                "f_le_x_upper",
                std::format(
                    "origin={} {}",
                    origin_label(he_key.first),
                    describe_undirected_edge(
                        graph,
                        arc_index,
                        he_key.second.first,
                        he_key.second.second,
                        graph.cols)),
                he_key.first));
        ++f_le_x_upper_rows;
        origin_x_entries[static_cast<std::size_t>(x_var)].push_back({row, 1.0});
        for (const auto f_j : f_list) {
            f_entries[static_cast<std::size_t>(f_j)].push_back({row, -1.0});
        }
    }

    // v5 §5 + v10 mod3: x_e <= o_i; Σ_{e∈δ(i)} x^H_e >= 2·o^H_i (transit) or >= o^H_i (terminal); Σ_H o^H_i <= cap_i
    const auto undirected_incidence = build_undirected_incidence(edge_row);
    auto origin_o_entries = std::Vector<std::Vector<std::pair<int, double>>> {};
    auto origin_o_vars = std::Vector<OriginOVar> {};
    auto node_row = std::map<int, int> {};
    auto physical_nodes_in_use = std::set<int> {};
    for (const auto& xv : origin_x_vars) {
        physical_nodes_in_use.insert(xv.u);
        physical_nodes_in_use.insert(xv.v);
    }
    for (const auto n : physical_nodes_in_use) {
        if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
            continue;
        }
        auto cap = 1;
        if (node_capacity_override.contains(n)) {
            cap = node_capacity_override.at(n);
        }
        node_row[n] = add_le(
            static_cast<double>(cap),
            McfConstraintMeta {
                "node_capacity",
                std::format(
                    "node={} rhs={} bus_used={}",
                    node_text(graph, n),
                    cap,
                    1 - cap)});
    }

    int x_le_o_rows = 0;
    int x_ge_degree_nonterminal_rows = 0;
    int x_ge_degree_terminal_rows = 0;
    auto origin_o_by_hn = std::map<std::pair<int, int>, int> {};
    for (const auto& xv : origin_x_vars) {
        const auto h = xv.h;
        const auto e = UndirectedEdgeKey {xv.u, xv.v};
        const auto x_var = origin_x_by_he.at({h, e});
        for (const auto n : {xv.u, xv.v}) {
            if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
                continue;
            }
            const auto hn = std::make_pair(h, n);
            if (!origin_o_by_hn.contains(hn)) {
                const auto o_var = static_cast<int>(origin_o_vars.size());
                origin_o_by_hn[hn] = o_var;
                origin_o_vars.push_back(OriginOVar {h, n});
                origin_o_entries.emplace_back();
                origin_o_entries[static_cast<std::size_t>(o_var)].push_back({node_row.at(n), 1.0});
            }
            const auto o_var = origin_o_by_hn.at(hn);
            const auto row_x_le_o = add_le(
                0.0,
                simple_meta(
                    "x_le_o",
                    std::format(
                        "origin={} node={} edge={}-{}",
                        origin_label(h),
                        node_text(graph, n),
                        node_text(graph, e.first),
                        node_text(graph, e.second)),
                    h));
            ++x_le_o_rows;
            origin_x_entries[static_cast<std::size_t>(x_var)].push_back({row_x_le_o, 1.0});
            origin_o_entries[static_cast<std::size_t>(o_var)].push_back({row_x_le_o, -1.0});
        }
    }
    auto terminal_nodes_by_h = std::map<int, std::set<int>> {};
    for (const auto& group : origin_groups) {
        terminal_nodes_by_h[group.origin_group_id] =
            build_origin_group_physical_endpoints(group, local_com, graph);
    }
    for (const auto& [hn, o_var] : origin_o_by_hn) {
        const auto h = hn.first;
        const auto n = hn.second;
        const auto delta_it = undirected_incidence.find(n);
        if (delta_it == undirected_incidence.end()) {
            continue;
        }
        auto x_on_delta = std::Vector<int> {};
        for (const auto& e : delta_it->second) {
            const auto he_it = origin_x_by_he.find({h, e});
            if (he_it != origin_x_by_he.end()) {
                x_on_delta.push_back(he_it->second);
            }
        }
        if (x_on_delta.empty()) {
            continue;
        }
        const auto terminal_it = terminal_nodes_by_h.find(h);
        const bool is_terminal =
            terminal_it != terminal_nodes_by_h.end() && terminal_it->second.contains(n);
        const auto row_degree = add_le(
            0.0,
            simple_meta(
                is_terminal ? "x_ge_degree_terminal" : "x_ge_degree_nonterminal",
                std::format(
                    "origin={} node={} role={}",
                    origin_label(h),
                    node_text(graph, n),
                    is_terminal ? "terminal" : "transit"),
                h));
        if (is_terminal) {
            ++x_ge_degree_terminal_rows;
            origin_o_entries[static_cast<std::size_t>(o_var)].push_back({row_degree, 1.0});
        } else {
            ++x_ge_degree_nonterminal_rows;
            origin_o_entries[static_cast<std::size_t>(o_var)].push_back({row_degree, 2.0});
        }
        for (const auto x_idx : x_on_delta) {
            origin_x_entries[static_cast<std::size_t>(x_idx)].push_back({row_degree, -1.0});
        }
    }

    for (const auto& group : origin_groups) {
        const int h = group.origin_group_id;
        const auto endpoints = build_origin_group_physical_endpoints(group, local_com, graph);
        for (const auto n : endpoints) {
            if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
                continue;
            }
            const auto hn = std::make_pair(h, n);
            if (origin_o_by_hn.contains(hn)) {
                continue;
            }
            if (!node_row.contains(n)) {
                auto cap = 1;
                if (node_capacity_override.contains(n)) {
                    cap = node_capacity_override.at(n);
                }
                node_row[n] = add_le(
                    static_cast<double>(cap),
                    McfConstraintMeta {
                        "node_capacity",
                        std::format(
                            "node={} rhs={} bus_used={}",
                            node_text(graph, n),
                            cap,
                            1 - cap)});
            }
            const auto o_var = static_cast<int>(origin_o_vars.size());
            origin_o_by_hn[hn] = o_var;
            origin_o_vars.push_back(OriginOVar {h, n});
            origin_o_entries.emplace_back();
            origin_o_entries[static_cast<std::size_t>(o_var)].push_back({node_row.at(n), 1.0});
        }
    }

    int o_endpoint_eq_rows = 0;
    for (const auto& group : origin_groups) {
        const int h = group.origin_group_id;
        const auto endpoints = build_origin_group_physical_endpoints(group, local_com, graph);
        for (const auto n : endpoints) {
            const auto hn = std::make_pair(h, n);
            const auto o_it = origin_o_by_hn.find(hn);
            if (o_it == origin_o_by_hn.end()) {
                std::size_t record_index = kInvalidRecordIndex;
                const Net_cost_record* record_ptr = nullptr;
                for (const auto k : group.commodity_local_indices) {
                    const auto& commodity = local_com[static_cast<std::size_t>(k)];
                    if (commodity.src == n || commodity.snk == n) {
                        record_index = commodity.record_index;
                        record_ptr = &records[record_index];
                        break;
                    }
                }
                if (record_ptr == nullptr && !group.commodity_local_indices.empty()) {
                    const auto k0 = group.commodity_local_indices.front();
                    record_index = local_com[static_cast<std::size_t>(k0)].record_index;
                    record_ptr = &records[record_index];
                }
                return fail_simple_mcf_early(
                    out,
                    solve_begin,
                    "endpoint_no_o_var",
                    std::format(
                        "{}: endpoint has no o variable after bus residual filter origin={} node={}",
                        stage_name,
                        origin_label(h),
                        node_text(graph, n)),
                    record_index,
                    *record_ptr,
                    unit_c,
                    gurobi_sink,
                    gurobi_meta,
                    std::format("origin={} node={}", origin_label(h), node_text(graph, n)));
            }
            auto role = std::String {"endpoint"};
            for (const auto k : group.commodity_local_indices) {
                const auto& commodity = local_com[static_cast<std::size_t>(k)];
                if (commodity.src == n) {
                    role = "source";
                    break;
                }
                if (commodity.snk == n) {
                    role = "sink";
                }
            }
            const auto o_var = o_it->second;
            const auto row = add_eq(
                1.0,
                simple_meta(
                    "o_endpoint_eq",
                    std::format(
                        "origin={} node={} ({})",
                        origin_label(h),
                        node_text(graph, n),
                        role),
                    h));
            ++o_endpoint_eq_rows;
            origin_o_entries[static_cast<std::size_t>(o_var)].push_back({row, 1.0});
        }
    }

    const auto num_f = static_cast<int>(f_vars.size());
    const auto num_origin_x = static_cast<int>(origin_x_vars.size());
    const auto num_origin_o = static_cast<int>(origin_o_vars.size());
    const auto num_col = num_f + num_origin_x + num_origin_o;
    const auto num_row = static_cast<int>(row_lo.size());

    log_mcf_constraint_rows(
        stage_name,
        {
            {"flow_conservation", static_cast<int>(flow_row.size())},
            {"edge_capacity", static_cast<int>(edge_row.size())},
            {"f_le_x_lower", f_le_x_lower_rows},
            {"f_le_x_upper", f_le_x_upper_rows},
            {"x_le_o", x_le_o_rows},
            {"x_ge_degree_nonterminal", x_ge_degree_nonterminal_rows},
            {"x_ge_degree_terminal", x_ge_degree_terminal_rows},
            {"o_endpoint_eq", o_endpoint_eq_rows},
            {"node_capacity", static_cast<int>(node_row.size())},
        });
    debug::info_fmt(
        "{} variables: f={} x={} o={} origin_groups={} cols={} rows={}",
        stage_name,
        num_f,
        num_origin_x,
        num_origin_o,
        origin_groups.size(),
        num_col,
        num_row);
    debug::info_fmt(
        "{} objective min_sum_x: {}",
        stage_name,
        enable_mcf_obj ? "enabled (--enable-mcf-obj)" : "disabled (feasibility only)");

    // SimpleMCF objective: min Σ x on non-virtual arcs (optional via --enable-mcf-obj)
    const bool apply_symmetry_break =
        enable_mcf_obj && warm_start != nullptr && !warm_start->nodes_by_record_id.empty();
    const auto symmetry_break = apply_symmetry_break
        ? collect_warm_start_used_edges_by_origin(graph, *warm_start, local_com, commodity_origin_h)
        : WarmStartSymmetryBreakInfo {};
    auto col_cost = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_lo = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_up = std::vector<double>(static_cast<std::size_t>(num_col), 1.0);
    auto col_entries = std::Vector<std::Vector<std::pair<int, double>>>(static_cast<std::size_t>(num_col));

    for (int j = 0; j < num_f; ++j) {
        col_entries[static_cast<std::size_t>(j)] = f_entries[static_cast<std::size_t>(j)];
    }
    std::size_t discounted_x_vars = 0;
    for (int j = 0; j < num_origin_x; ++j) {
        const auto col = num_f + j;
        const auto& xv = origin_x_vars[static_cast<std::size_t>(j)];
        double cost = 0.0;
        if (enable_mcf_obj) {
            cost = 1.0;
            if (apply_symmetry_break) {
                const auto e = normalized_edge_key(xv.u, xv.v);
                const auto h_it = symmetry_break.used_edges_by_h.find(xv.h);
                if (h_it != symmetry_break.used_edges_by_h.end() && h_it->second.contains(e)) {
                    cost = kSimpleMcfWarmStartUsedEdgeCost;
                    ++discounted_x_vars;
                }
            }
        }
        col_cost[static_cast<std::size_t>(col)] = cost;
        col_entries[static_cast<std::size_t>(col)] = origin_x_entries[static_cast<std::size_t>(j)];
    }
    if (apply_symmetry_break) {
        debug::info_fmt(
            "{} objective symmetry-break: used_edge_cost={} discounted_x_vars={} total_x_vars={} matched_warm_paths={}",
            stage_name,
            kSimpleMcfWarmStartUsedEdgeCost,
            discounted_x_vars,
            origin_x_vars.size(),
            symmetry_break.matched_paths);
    }
    for (int j = 0; j < num_origin_o; ++j) {
        const auto col = num_f + num_origin_x + j;
        col_entries[static_cast<std::size_t>(col)] = origin_o_entries[static_cast<std::size_t>(j)];
    }

    auto warm_values_by_col = std::map<int, double> {};
    if (refine_incumbent_seed != nullptr) {
        std::size_t x_count = 0;
        std::size_t o_count = 0;
        for (std::size_t j = 0; j < origin_x_vars.size(); ++j) {
            const auto& xv = origin_x_vars[j];
            const auto e = normalized_edge_key(xv.u, xv.v);
            const auto it = refine_incumbent_seed->used_edges.find(e);
            if (it != refine_incumbent_seed->used_edges.end() && it->second >= 1) {
                warm_values_by_col[static_cast<int>(num_f + static_cast<int>(j))] = 1.0;
                ++x_count;
            }
        }
        for (std::size_t j = 0; j < origin_o_vars.size(); ++j) {
            const auto& ov = origin_o_vars[j];
            const auto it = refine_incumbent_seed->used_nodes.find(ov.node);
            if (it != refine_incumbent_seed->used_nodes.end() && it->second >= 1) {
                warm_values_by_col[num_f + num_origin_x + static_cast<int>(j)] = 1.0;
                ++o_count;
            }
        }
        debug::info_fmt(
            "{} refine incumbent warm start: x={} o={}",
            stage_name,
            x_count,
            o_count);
    }
    if (warm_start != nullptr && !warm_start->nodes_by_record_id.empty()) {
        auto origin_o_col_by_h_node = std::map<std::pair<int, int>, int> {};
        for (std::size_t oi = 0; oi < origin_o_vars.size(); ++oi) {
            const auto& ov = origin_o_vars[oi];
            origin_o_col_by_h_node[{ov.h, ov.node}] = num_f + num_origin_x + static_cast<int>(oi);
        }

        std::size_t matched_paths = 0;
        for (int k = 0; k < K; ++k) {
            const auto& commodity = local_com[static_cast<std::size_t>(k)];
            const auto path_it = warm_start->nodes_by_record_id.find(commodity.record_id);
            if (path_it == warm_start->nodes_by_record_id.end()) {
                continue;
            }
            const auto& path = path_it->second;
            if (path.size() < 2) {
                continue;
            }
            ++matched_paths;
            const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                const auto u = path[i];
                const auto v = path[i + 1];
                for (const auto f_col : f_by_k[static_cast<std::size_t>(k)]) {
                    const auto arc_id = f_vars[static_cast<std::size_t>(f_col)].a;
                    const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                    if (arc.u == u && arc.v == v) {
                        warm_values_by_col[f_col] = 1.0;
                        if (!arc.is_virtual) {
                            const auto e = normalized_edge_key(arc.u, arc.v);
                            const auto ox_it = origin_x_by_he.find({h, e});
                            if (ox_it != origin_x_by_he.end()) {
                                warm_values_by_col[num_f + ox_it->second] = 1.0;
                            }
                        }
                        break;
                    }
                }
            }
            for (const auto node : path) {
                if (graph.nodes[static_cast<std::size_t>(node)].is_virtual) {
                    continue;
                }
                const auto it = origin_o_col_by_h_node.find({h, node});
                if (it != origin_o_col_by_h_node.end()) {
                    warm_values_by_col[it->second] = 1.0;
                }
            }
        }

        if (!warm_values_by_col.empty()) {
            debug::info_fmt(
                "{} warm start loaded for Gurobi: matched_paths={}, values={}",
                stage_name,
                matched_paths,
                warm_values_by_col.size());
        }
    }

    const auto cpp_model_build_ms = stage_solve_elapsed_ms(solve_begin);
    const auto solve_res = solve_binary_columns_with_gurobi(
        stage_name,
        col_cost,
        col_lo,
        col_up,
        row_lo,
        row_up,
        col_entries,
        warm_values_by_col,
        &row_meta,
        diag,
        gurobi_sink,
        gurobi_meta,
        gurobi_params);
    out.model_status = solve_res.model_status;
    auto timing = merge_gurobi_stage_timing(cpp_model_build_ms, solve_res.gurobi_timing);
    if (solve_res.solution_class == McfSolutionClass::Failed
        || solve_res.solution_class == McfSolutionClass::TimeLimit) {
        if (!refine_pass && warm_start != nullptr && !warm_values_by_col.empty()) {
            debug::warning_fmt(
                "{} warm start led to {}; retrying without warm start",
                stage_name,
                solution_class_name(solve_res.solution_class));
            auto first_log = out;
            apply_stage_solution_class(first_log, solve_res.solution_class);
            first_log.model_status = solve_res.model_status;
            first_log.message = solve_res.message;
            first_log.infeasibility_hints = solve_res.iis_rows;
            for (const auto& hint : first_log.infeasibility_hints) {
                if (hint.record_index != kInvalidRecordIndex) {
                    append_unique(first_log.failed_record_indices, hint.record_index);
                }
                if (hint.simple_unit >= 0) {
                    append_simple_origin_retry_hint(
                        first_log,
                        static_cast<std::size_t>(hint.simple_unit),
                        hint.simple_origin_key);
                }
            }
            first_log = finish_stage_solve_result(first_log, solve_begin, timing);
            auto retry_meta = gurobi_meta;
            retry_meta.warm_start = false;
            retry_meta.retry_kind = McfGurobiRetryKind::NoWarmStart;
            auto retry = solve_simple_mcf_component(
                graph,
                commodities,
                unit_c,
                simple_ids_for_unit,
                records,
                bbox_ctx,
                expand_state,
                edge_capacity_override,
                node_capacity_override,
                enable_mcf_obj,
                nullptr,
                diag,
                gurobi_sink,
                retry_meta,
                stage_name,
                gurobi_params,
                nullptr,
                refine_pass);
            retry.solve_ms += first_log.solve_ms;
            return retry;
        }
        apply_stage_solution_class(out, solve_res.solution_class);
        out.message = solve_res.message;
        out.infeasibility_hints = solve_res.iis_rows;
        for (const auto& hint : out.infeasibility_hints) {
            if (hint.record_index != kInvalidRecordIndex) {
                append_unique(out.failed_record_indices, hint.record_index);
            }
            if (hint.simple_unit >= 0) {
                append_simple_origin_retry_hint(
                    out,
                    static_cast<std::size_t>(hint.simple_unit),
                    hint.simple_origin_key);
            }
        }
        return finish_stage_solve_result(out, solve_begin, timing);
    }

    apply_stage_solution_class(out, solve_res.solution_class);
    out.message = "ok";
    out.objective = solve_res.objective;

    const auto extract_begin = std::chrono::steady_clock::now();
    auto f_values = std::Vector<int>(f_vars.size(), 0);
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        f_values[j] = static_cast<int>(std::lround(solve_res.col_value[j]));
    }
    for (std::size_t j = 0; j < origin_x_vars.size(); ++j) {
        const auto col = static_cast<std::size_t>(num_f + static_cast<int>(j));
        const auto val = static_cast<int>(std::lround(solve_res.col_value[col]));
        if (val <= 0) {
            continue;
        }
        out.used_edges[{origin_x_vars[j].u, origin_x_vars[j].v}] = 1;
    }
    for (std::size_t j = 0; j < origin_o_vars.size(); ++j) {
        const auto col = static_cast<std::size_t>(num_f + num_origin_x + static_cast<int>(j));
        const auto val = static_cast<int>(std::lround(solve_res.col_value[col]));
        if (val > 0) {
            out.used_nodes[origin_o_vars[j].node] = 1;
        }
    }

    append_paths_from_f_solution(stage_name, graph, local_com, f_vars, f_values, out);
    timing.extract_path_ms = elapsed_ms_between(extract_begin, std::chrono::steady_clock::now());
    return finish_stage_solve_result(out, solve_begin, timing);
}

auto build_refine_incumbent_seed_from_stage1(const StageSolveResult& stage1) -> StageRefineIncumbentSeed {
    return StageRefineIncumbentSeed {stage1.used_edges, stage1.used_nodes};
}

auto build_guide_path_warm_start(const std::Vector<PreparedCommodity>& local_com) -> StageWarmStart {
    auto warm = StageWarmStart {};
    for (const auto& commodity : local_com) {
        if (commodity.guide_path.size() >= 2) {
            warm.nodes_by_record_id.emplace(commodity.record_id, commodity.guide_path);
        }
    }
    return warm;
}

auto solve_simple_mcf_unit(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& bbox_ctx,
    const McfBBoxExpandState* expand_state,
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::map<int, int>& node_capacity_override,
    const bool enable_mcf_obj,
    const StageWarmStart* warm_start,
    const GurobiDiagnosticsOptions& diag,
    McfGurobiLogSink* gurobi_sink,
    McfGurobiSolveMeta gurobi_meta,
    McfGurobiSolveParams gurobi_params,
    const SimpleMcfSolvePass pass,
    const StageRefineIncumbentSeed* refine_incumbent_seed
) -> StageSolveResult {
    auto stage_name = std::format("SimpleMCF_unit{}", unit_c);
    if (pass == SimpleMcfSolvePass::TreeSeed) {
        stage_name += "_tree_seed";
    }
    else if (pass == SimpleMcfSolvePass::Refine) {
        stage_name += "_refine";
    }
    const auto refine_pass = pass == SimpleMcfSolvePass::Refine;
    const auto solve_begin = std::chrono::steady_clock::now();
    if (simple_ids_for_unit.empty()) {
        StageSolveResult out {};
        out.stage_name = stage_name;
        if (gurobi_sink != nullptr) {
            gurobi_sink->write_skipped(McfGurobiLogStage::simple_unit(unit_c), "empty_stage");
        }
        apply_stage_solution_class(out, McfSolutionClass::Skipped);
        out.message = "empty stage";
        return finish_stage_solve_early(out, solve_begin);
    }

    if (pass == SimpleMcfSolvePass::TreeSeed && gurobi_params.mip_gap <= 0.0) {
        gurobi_params.mip_gap = kSimpleMcfTreeSeedMipGap;
    }
    if (gurobi_params.threads <= 0) {
        gurobi_params.threads = 2;
    }
    gurobi_meta.pass = pass;

    const auto [per_vertex, vertex_labels] = refine_pass
        ? preview_simple_refine_candidate_resources(
            graph,
            commodities,
            unit_c,
            simple_ids_for_unit,
            records,
            bbox_ctx,
            expand_state,
            edge_capacity_override,
            node_capacity_override)
        : preview_simple_candidate_resources(
            graph,
            commodities,
            unit_c,
            simple_ids_for_unit,
            records,
            bbox_ctx,
            expand_state,
            edge_capacity_override,
            node_capacity_override);
    const auto components = build_edge_node_conflict_components(per_vertex, vertex_labels);
    debug::info_fmt(
        "{} conflict decomposition: vertices={} components={} pass={}",
        stage_name,
        per_vertex.size(),
        components.size(),
        simple_mcf_solve_pass_name(pass));

    if (components.size() <= 1) {
        auto meta = gurobi_meta;
        meta.component_count = 1;
        meta.component_id = 0;
        if (!vertex_labels.empty()) {
            meta.component_summary = std::format("origins={}", vertex_labels.front());
            for (std::size_t i = 1; i < vertex_labels.size(); ++i) {
                meta.component_summary += std::format(",{}", vertex_labels[i]);
            }
        }
        return solve_simple_mcf_component(
            graph,
            commodities,
            unit_c,
            simple_ids_for_unit,
            records,
            bbox_ctx,
            expand_state,
            edge_capacity_override,
            node_capacity_override,
            enable_mcf_obj,
            warm_start,
            diag,
            gurobi_sink,
            meta,
            stage_name,
            gurobi_params,
            refine_incumbent_seed,
            refine_pass);
    }

    const auto component_count = static_cast<int>(components.size());
    auto cancel_flag = std::atomic<bool> {false};
    auto results = std::Vector<StageSolveResult>(components.size());
    auto results_mutex = std::mutex {};
    auto tasks = std::Vector<std::function<void()>> {};
    tasks.reserve(components.size());
    for (int ci = 0; ci < component_count; ++ci) {
        tasks.push_back([&, ci] {
            if (cancel_flag.load()) {
                return;
            }
            const auto& component = components[static_cast<std::size_t>(ci)];
            const auto subset_ids = refine_pass
                ? simple_ids_for_refine_conflict_component(
                    simple_ids_for_unit, commodities, vertex_labels, component)
                : simple_ids_for_conflict_component(
                    simple_ids_for_unit, commodities, records, vertex_labels, component);
            auto meta = gurobi_meta;
            meta.component_id = ci;
            meta.component_count = component_count;
            meta.component_summary = std::format("origins={}", component.summary);
            const auto comp_stage =
                std::format("{}_c{}of{}", stage_name, ci, component_count);
            auto part = solve_simple_mcf_component(
                graph,
                commodities,
                unit_c,
                subset_ids,
                records,
                bbox_ctx,
                expand_state,
                edge_capacity_override,
                node_capacity_override,
                enable_mcf_obj,
                warm_start,
                diag,
                gurobi_sink,
                meta,
                comp_stage,
                gurobi_params,
                refine_incumbent_seed,
                refine_pass);
            {
                const std::lock_guard lock {results_mutex};
                results[static_cast<std::size_t>(ci)] = std::move(part);
                if (!results[static_cast<std::size_t>(ci)].ok) {
                    cancel_flag.store(true);
                }
            }
        });
    }
    run_mcf_parallel_waves(mcf_gurobi_thread_budget_instance(), gurobi_params.threads, tasks, &cancel_flag);
    return merge_stage_solve_results(stage_name, std::move(results), solve_begin);
}

auto path_to_text(const GlobalGraph& graph, const std::Vector<int>& path) -> std::String {
    if (path.empty()) {
        return "(empty)";
    }
    auto s = std::String {};
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i != 0) {
            s += " -> ";
        }
        s += node_text(graph, path[i]);
    }
    return s;
}

auto count_bumps_for_record(const Net_cost_record& record) -> std::size_t {
    if (record.type == Net_type::Bnet) {
        return record.start_bumps.size() + record.end_bumps.size();
    }
    return record.start_bumps.empty() ? 0U : 1U;
}

auto add_path_physical_nodes(
    const GlobalGraph& graph,
    const std::Vector<int>& path,
    std::set<int>& seen_nodes,
    std::size_t& length
) -> void {
    for (const auto node : path) {
        if (node < 0 || node >= static_cast<int>(graph.nodes.size())) {
            continue;
        }
        const auto& meta = graph.nodes[static_cast<std::size_t>(node)];
        if (meta.is_virtual) {
            continue;
        }
        if (seen_nodes.insert(node).second) {
            ++length;
        }
    }
}

auto actual_end_track_from_path(const GlobalGraph& graph, const std::Vector<int>& path) -> std::optional<std::size_t> {
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        const auto node = *it;
        if (node < 0 || node >= static_cast<int>(graph.nodes.size())) {
            continue;
        }
        const auto& meta = graph.nodes[static_cast<std::size_t>(node)];
        if (!meta.is_virtual) {
            return meta.track;
        }
    }
    return std::nullopt;
}

auto wire_length_single_commodity(
    const GlobalGraph& graph,
    const McfPathInfo& info,
    const Net_cost_record& record
) -> std::size_t {
    auto seen_nodes = std::set<int> {};
    std::size_t length = 0;
    for (const auto& path : info.unit_paths) {
        add_path_physical_nodes(graph, path, seen_nodes, length);
    }
    length += count_bumps_for_record(record);
    return length;
}

struct PathRef {
    const McfPathInfo* info;
    std::size_t cob_unit;
};

auto wire_length_multi_fanout_group(
    const GlobalGraph& graph,
    const std::Vector<PathRef>& paths,
    const std::Vector<Net_cost_record>& records
) -> std::size_t {
    auto seen_nodes = std::set<int> {};
    std::size_t length = 0;
    for (const auto& pr : paths) {
        if (pr.info->record_id >= records.size()) {
            continue;
        }
        const auto& record = records[pr.info->record_id];
        if (pr.info->unit_paths.empty()) {
            length += count_bumps_for_record(record);
            continue;
        }
        add_path_physical_nodes(graph, pr.info->unit_paths.front(), seen_nodes, length);
        length += count_bumps_for_record(record);
    }
    return length;
}

auto wire_length_for_origin_group(
    const GlobalGraph& graph,
    const bool is_bus,
    const bool is_multi_fanout,
    const std::Vector<PathRef>& paths,
    const std::Vector<Net_cost_record>& records
) -> std::size_t {
    if (paths.empty()) {
        return 0;
    }
    if (is_multi_fanout) {
        return wire_length_multi_fanout_group(graph, paths, records);
    }
    if (is_bus && paths.size() > 1) {
        std::size_t sum = 0;
        for (const auto& pr : paths) {
            if (pr.info->record_id >= records.size()) {
                continue;
            }
            sum += wire_length_single_commodity(graph, *pr.info, records[pr.info->record_id]);
        }
        return sum;
    }
    const auto& pr = paths.front();
    if (pr.info->record_id >= records.size()) {
        return 0;
    }
    return wire_length_single_commodity(graph, *pr.info, records[pr.info->record_id]);
}

auto log_mcf_paths_by_origin_net(
    const GlobalGraph& graph,
    const std::array<std::Vector<McfPathInfo>, 16>& paths_by_unit,
    const std::Vector<Net_cost_record>& records
) -> std::size_t {
    struct LogOriginGroup {
        bool is_bus{false};
        std::size_t cob_unit{0};
        std::String group_key {};
        std::String display_name {};
        bool is_multi_fanout{false};
        std::Vector<PathRef> paths {};
    };

    auto refs = std::Vector<PathRef> {};
    for (std::size_t u = 0; u < 16; ++u) {
        for (const auto& info : paths_by_unit[u]) {
            refs.push_back(PathRef {&info, u});
        }
    }
    if (refs.empty()) {
        debug::info("MCF paths grouped by MCF origin: (no paths)");
        return 0;
    }

    auto group_map = std::map<std::String, LogOriginGroup> {};
    for (const auto& pr : refs) {
        const auto& info = *pr.info;
        if (info.record_id >= records.size()) {
            continue;
        }
        const auto& rec = records[info.record_id];
        std::String map_key {};
        LogOriginGroup group {};
        if (is_sync_bus_origin_key(rec.origin_key.empty() ? rec.net_name : rec.origin_key)) {
            map_key = std::format("bus:{}", info.origin_name);
            group.is_bus = true;
            group.cob_unit = pr.cob_unit;
            group.group_key = info.origin_name;
            group.display_name = info.origin_name;
        }
        else {
            const auto gkey = record_origin_group_uid(rec);
            map_key = std::format("simple:{}:{}", pr.cob_unit, gkey);
            group.is_bus = false;
            group.cob_unit = pr.cob_unit;
            group.group_key = gkey;
            group.display_name = rec.origin_key.empty() ? rec.net_name : rec.origin_key;
        }
        auto it = group_map.find(map_key);
        if (it == group_map.end()) {
            group.paths.push_back(pr);
            group_map.emplace(std::move(map_key), std::move(group));
        }
        else {
            it->second.paths.push_back(pr);
        }
    }

    auto ordered_keys = std::Vector<std::String> {};
    ordered_keys.reserve(group_map.size());
    for (const auto& [key, _] : group_map) {
        ordered_keys.push_back(key);
    }
    std::sort(ordered_keys.begin(), ordered_keys.end());

    for (auto& key : ordered_keys) {
        auto& group = group_map.at(key);
        if (group.paths.size() > 1) {
            for (const auto& pr : group.paths) {
                if (pr.info->record_id >= records.size()) {
                    continue;
                }
                const auto& rec = records[pr.info->record_id];
                if (rec.from_track_to_bumps_split || rec.type == Net_type::PNnet) {
                    group.is_multi_fanout = true;
                    break;
                }
            }
        }
        if (group.is_multi_fanout) {
            const auto& first_rec = records[group.paths.front().info->record_id];
            group.display_name = first_rec.origin_key.empty() ? first_rec.net_name : first_rec.origin_key;
        }
        else if (group.paths.size() == 1 && group.paths.front().info->record_id < records.size()) {
            group.display_name = records[group.paths.front().info->record_id].net_name;
        }

        std::sort(group.paths.begin(), group.paths.end(), [&](const PathRef& a, const PathRef& b) {
            const auto bit_a = (a.info->record_id < records.size()) ? records[a.info->record_id].bit_id : 0U;
            const auto bit_b = (b.info->record_id < records.size()) ? records[b.info->record_id].bit_id : 0U;
            if (bit_a != bit_b) {
                return bit_a < bit_b;
            }
            return a.info->label < b.info->label;
        });
    }

    debug::info(
        "MCF paths grouped by MCF origin (BusMCF: SyncNet origin_key; SimpleMCF: COBUnit + origin_uid):");

    auto total_wire_length = std::size_t {0};
    auto summary_rows = std::Vector<std::tuple<std::String, std::size_t, bool, std::size_t>> {};

    for (const auto& key : ordered_keys) {
        const auto& group = group_map.at(key);
        const auto group_wire_length = wire_length_for_origin_group(
            graph,
            group.is_bus,
            group.is_multi_fanout,
            group.paths,
            records);
        total_wire_length += group_wire_length;
        summary_rows.emplace_back(group.display_name, group_wire_length, group.is_multi_fanout, group.paths.size());

        if (group.is_bus) {
            debug::info_fmt(
                "  [BusMCF] origin=\"{}\" commodities={} wire_length={}",
                group.display_name,
                group.paths.size(),
                group_wire_length);
        }
        else {
            debug::info_fmt(
                "  [SimpleMCF] COBUnit={} group_key=\"{}\" display=\"{}\" multi_fanout={} commodities={} wire_length={}",
                group.cob_unit,
                group.group_key,
                group.display_name,
                group.is_multi_fanout,
                group.paths.size(),
                group_wire_length);
        }
        for (const auto& pr : group.paths) {
            const auto& info = *pr.info;
            std::size_t bit_id = 0;
            auto rec_name = std::String("(record_id out of range)");
            const Net_cost_record* rec_ptr = nullptr;
            if (info.record_id < records.size()) {
                bit_id = records[info.record_id].bit_id;
                rec_name = records[info.record_id].net_name;
                rec_ptr = &records[info.record_id];
            }

            const auto commodity_wire_length = (rec_ptr != nullptr && !group.is_multi_fanout)
                ? wire_length_single_commodity(graph, info, *rec_ptr)
                : 0U;

            if (rec_ptr != nullptr && rec_ptr->type == Net_type::PNnet && !info.unit_paths.empty()) {
                const auto actual_end = actual_end_track_from_path(graph, info.unit_paths.front());
                const auto actual_end_text = actual_end.has_value() ? std::format("{}", *actual_end) : std::String("?");
                debug::info_fmt(
                    "    commodity={} record=\"{}\" record_id={} bit={} start_track={} end_track={} actual_end_track={} path_count={}{}",
                    info.label,
                    rec_name,
                    info.record_id,
                    bit_id,
                    info.start_track,
                    info.end_track,
                    actual_end_text,
                    info.unit_paths.size(),
                    group.is_multi_fanout ? std::String("") : std::format(" wire_length={}", commodity_wire_length));
            }
            else if (group.is_bus) {
                debug::info_fmt(
                    "    commodity={} record=\"{}\" record_id={} bit={} start_track={} end_track={} path_count={} wire_length={}",
                    info.label,
                    rec_name,
                    info.record_id,
                    bit_id,
                    info.start_track,
                    info.end_track,
                    info.unit_paths.size(),
                    commodity_wire_length);
            }
            else if (group.is_multi_fanout) {
                debug::info_fmt(
                    "    commodity={} record=\"{}\" record_id={} bit={} start_track={} end_track={} path_count={}",
                    info.label,
                    rec_name,
                    info.record_id,
                    bit_id,
                    info.start_track,
                    info.end_track,
                    info.unit_paths.size());
            }
            else {
                debug::info_fmt(
                    "    commodity={} record=\"{}\" record_id={} bit={} start_track={} end_track={} path_count={} wire_length={}",
                    info.label,
                    rec_name,
                    info.record_id,
                    bit_id,
                    info.start_track,
                    info.end_track,
                    info.unit_paths.size(),
                    commodity_wire_length);
            }
            for (std::size_t pi = 0; pi < info.unit_paths.size(); ++pi) {
                debug::info_fmt("      path#{} {}", pi, path_to_text(graph, info.unit_paths[pi]));
            }
        }
    }

    debug::info_fmt(
        "MCF wire length summary: total_wire_length={} origin_groups={}",
        total_wire_length,
        summary_rows.size());
    for (const auto& [display_name, wire_length, multi_fanout, commodities] : summary_rows) {
        debug::info_fmt(
            "  origin=\"{}\" wire_length={} multi_fanout={} commodities={}",
            display_name,
            wire_length,
            multi_fanout,
            commodities);
    }
    return total_wire_length;
}

template <typename T>
auto sort_unique(std::Vector<T>& values) -> void {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

auto merge_bus_stage_retry_hints(CobMcfRetryHints& hints, const StageSolveResult& bus_res) -> void {
    for (const auto& key : bus_res.failed_bus_keys) {
        append_unique(hints.failed_bus_keys, key);
    }
    for (const auto record_index : bus_res.failed_record_indices) {
        append_unique(hints.failed_record_indices, record_index);
    }
    if ((bus_res.solution_class == McfSolutionClass::Failed
            || bus_res.solution_class == McfSolutionClass::TimeLimit)
        && (bus_res.bus_failure_unlocalized || bus_res.failed_bus_keys.empty())) {
        hints.bus_failure_unlocalized = true;
    }
}

auto add_simple_unit_retry_hints(
    CobMcfRetryHints& hints,
    const std::size_t unit,
    const std::array<std::Vector<std::size_t>, 16>& simple_ids_by_unit,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<Net_cost_record>& records
) -> void {
    append_unique(hints.failed_simple_units, unit);

    auto multi_fanout_origins = std::set<std::String> {};
    for (const auto cid : simple_ids_by_unit[unit]) {
        if (cid >= commodities.size()) {
            continue;
        }
        const auto record_index = commodities[cid].record_index;
        if (record_index >= records.size()) {
            continue;
        }
        append_unique(hints.failed_record_indices, record_index);
        const auto& record = records[record_index];
        if (record.from_track_to_bumps_split || record.type == Net_type::PNnet) {
            multi_fanout_origins.insert(record_origin_group_uid(record));
        }
    }

    for (std::size_t i = 0; i < records.size(); ++i) {
        if (!multi_fanout_origins.contains(record_origin_group_uid(records[i]))) {
            continue;
        }
        append_unique(hints.failed_record_indices, i);
    }
}

auto normalize_retry_hints(CobMcfRetryHints& hints) -> void {
    sort_unique(hints.failed_bus_keys);
    sort_unique(hints.failed_simple_units);
    sort_unique(hints.failed_record_indices);
}

auto collect_failed_bus_keys_for_expand(
    const StageSolveResult& bus_res,
    const McfBBoxContext& bbox_ctx
) -> std::Vector<std::String> {
    if (!bus_res.failed_bus_keys.empty()) {
        return bus_res.failed_bus_keys;
    }
    auto keys = std::Vector<std::String> {};
    keys.reserve(bbox_ctx.per_bus_key.size());
    for (const auto& [key, _] : bbox_ctx.per_bus_key) {
        keys.push_back(key);
    }
    return keys;
}

auto collect_all_origin_groups_in_unit(
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<Net_cost_record>& records
) -> std::Vector<McfSimpleOriginGroupKey> {
    auto seen = std::set<McfSimpleOriginGroupKey> {};
    auto out = std::Vector<McfSimpleOriginGroupKey> {};
    for (const auto sid : simple_ids) {
        if (sid >= commodities.size()) {
            continue;
        }
        const auto& commodity = commodities[sid];
        if (commodity.record_index >= records.size()) {
            continue;
        }
        const auto key = std::make_pair(unit_c, simple_origin_group_key(records[commodity.record_index]));
        if (seen.insert(key).second) {
            out.push_back(key);
        }
    }
    return out;
}

auto collect_failed_simple_origin_groups(
    const StageSolveResult& simple_res,
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<Net_cost_record>& records
) -> std::Vector<McfSimpleOriginGroupKey> {
    auto seen = std::set<McfSimpleOriginGroupKey> {};
    auto out = std::Vector<McfSimpleOriginGroupKey> {};
    for (const auto& key : simple_res.failed_simple_origin_groups) {
        if (key.first != unit_c) {
            continue;
        }
        if (seen.insert(key).second) {
            out.push_back(key);
        }
    }
    if (!out.empty()) {
        return out;
    }
    for (const auto record_index : simple_res.failed_record_indices) {
        if (record_index >= records.size()) {
            continue;
        }
        for (const auto sid : simple_ids) {
            if (sid >= commodities.size() || commodities[sid].record_index != record_index) {
                continue;
            }
            const auto key = std::make_pair(unit_c, simple_origin_group_key(records[record_index]));
            if (seen.insert(key).second) {
                out.push_back(key);
            }
            break;
        }
    }
    if (!out.empty()) {
        return out;
    }
    return collect_all_origin_groups_in_unit(unit_c, simple_ids, commodities, records);
}

struct BusStageOutcome {
    bool ok{false};
    bool bbox_exhausted{false};
    StageSolveResult bus_res {};
    int warm_start_ms{0};
    int bbox_expand_attempts{0};
};

struct UnitStageOutcome {
    bool ok{false};
    bool unit_exhausted{false};
    bool refine_modeling_error{false};
    StageSolveResult simple_res {};
};

auto simple_mcf_unit_log_mode(const bool tree_refine) -> std::String {
    return tree_refine ? std::String {"tree-refine"} : std::String {"standard"};
}

auto simple_mcf_unit_log_prefix(const std::size_t unit_c, const bool tree_refine) -> std::String {
    return std::format("[MCF u{}/{}]", unit_c, simple_mcf_unit_log_mode(tree_refine));
}

class SimpleMcfUnitLogScope {
public:
    SimpleMcfUnitLogScope(
        const std::size_t unit_c,
        const bool tree_refine,
        const UnitStageOutcome* outcome
    )
        : unit_c_ {unit_c}
        , outcome_ {outcome}
        , prefix_ {simple_mcf_unit_log_prefix(unit_c, tree_refine)} {
        debug::info_fmt(
            "======== SimpleMCF unit {} BEGIN ({}) ========",
            unit_c_,
            simple_mcf_unit_log_mode(tree_refine));
    }

    ~SimpleMcfUnitLogScope() {
        if (outcome_ == nullptr) {
            return;
        }
        debug::info_fmt(
            "======== SimpleMCF unit {} END ok={} class={} solve_ms={} paths={} ========",
            unit_c_,
            outcome_->ok,
            solution_class_name(outcome_->simple_res.solution_class),
            outcome_->simple_res.solve_ms,
            outcome_->simple_res.paths.size());
    }

private:
    std::size_t unit_c_;
    const UnitStageOutcome* outcome_;
    debug::ScopedThreadLogPrefix prefix_;
};

auto log_bus_bbox_expand(
    const int attempt,
    const std::Vector<std::String>& bus_keys,
    const McfBBoxExpandResult& expand_result,
    const McfBBoxContext& bbox_ctx
) -> void {
    auto box_parts = std::Vector<std::String> {};
    for (const auto& key : bus_keys) {
        const auto it = bbox_ctx.per_bus_key.find(key);
        if (it != bbox_ctx.per_bus_key.end()) {
            box_parts.push_back(std::format("{}:{}", key, format_bbox(it->second)));
        }
    }
    if (expand_result.any_exhausted) {
        debug::info_fmt(
            "MCF bbox expand: stage=BusMCF exhausted=true attempt={} reason=bus_key={} at full array",
            attempt,
            expand_result.exhausted_key);
        return;
    }
    debug::info_fmt(
        "MCF bbox expand: stage=BusMCF attempt={} bus_keys={} exhausted=false boxes={{{}}}",
        attempt,
        bus_keys.size(),
        fmt_join_parts(box_parts));
}

auto log_simple_bbox_expand(
    const std::size_t unit_c,
    const int attempt,
    const std::Vector<McfSimpleOriginGroupKey>& origin_groups,
    const McfBBoxExpandResult& expand_result,
    const McfBBoxExpandState& expand_state
) -> void {
    if (expand_result.any_exhausted) {
        debug::info_fmt(
            "MCF bbox expand: stage=SimpleMCF_unit{} exhausted=true attempt={} reason={}",
            unit_c,
            attempt,
            expand_result.exhausted_key);
        return;
    }
    auto group_parts = std::Vector<std::String> {};
    for (const auto& key : origin_groups) {
        const auto it = expand_state.simple_origin_hull.find(key);
        if (it != expand_state.simple_origin_hull.end()) {
            group_parts.push_back(std::format("{}:{}", key.second, format_bbox(it->second)));
        }
    }
    debug::info_fmt(
        "MCF bbox expand: stage=SimpleMCF_unit{} attempt={} origin_groups={} exhausted=false boxes={{{}}}",
        unit_c,
        attempt,
        origin_groups.size(),
        fmt_join_parts(group_parts));
}

auto run_bus_mcf_stage_with_bbox_retry(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& bus_ids,
    const bool disable_bus_mcf,
    const bool enable_pre_routing,
    const GurobiDiagnosticsOptions& diag,
    McfBBoxContext& bbox_ctx,
    McfGurobiLogSink* gurobi_sink,
    const int sat_tier_attempt
) -> BusStageOutcome {
    auto out = BusStageOutcome {};
    if (disable_bus_mcf) {
        out.bus_res.stage_name = "BusMCF";
        out.bus_res.message = "skipped (--disable-bus-mcf)";
        apply_stage_solution_class(out.bus_res, McfSolutionClass::Skipped);
        out.ok = true;
        if (gurobi_sink != nullptr) {
            gurobi_sink->write_skipped(McfGurobiLogStage::bus(), "disabled");
        }
        if (!bus_ids.empty()) {
            debug::info_fmt(
                "MCF: BusMCF skipped; {} bus commodities are not routed",
                bus_ids.size());
        }
        return out;
    }

    auto outgoing_arcs = std::Vector<std::Vector<int>> {};
    if (enable_pre_routing) {
        outgoing_arcs.resize(graph.nodes.size());
        for (std::size_t a = 0; a < graph.arcs.size(); ++a) {
            outgoing_arcs[static_cast<std::size_t>(graph.arcs[a].u)].push_back(static_cast<int>(a));
        }
    }

    const auto bus_effective_bbox = [&](const std::size_t cid) -> McfCommodityBBox {
        const auto& commodity = commodities[cid];
        const McfBBoxCommodityInput input {
            commodity.record_index,
            commodity.is_bus,
            commodity.bus_key};
        return resolve_mcf_bbox(bbox_ctx, cid, input, McfArcBBoxMode::Bus, McfCommodityBBox {});
    };

    int bus_solve_ms_total = 0;
    for (std::size_t attempt = 0;; ++attempt) {
        const StageWarmStart* bus_warm_start_ptr = nullptr;
        auto bus_warm_start = StageWarmStart {};
        if (enable_pre_routing) {
            const auto bus_warm_t0 = std::chrono::steady_clock::now();
            auto warm_used_edges = std::map<std::pair<int, int>, int> {};
            auto warm_used_nodes = std::map<int, int> {};
            bus_warm_start = route_mcf_stage_warm_start(
                "BusMCF",
                graph,
                commodities,
                bus_ids,
                bus_effective_bbox,
                outgoing_arcs,
                warm_used_edges,
                warm_used_nodes);
            bus_warm_start_ptr = &bus_warm_start;
            const auto bus_warm_t1 = std::chrono::steady_clock::now();
            out.warm_start_ms += static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(bus_warm_t1 - bus_warm_t0).count());
        }

        McfGurobiSolveMeta bus_meta {
            McfGurobiLogStage::bus(),
            static_cast<int>(bbox_ctx.max_tier),
            sat_tier_attempt,
            static_cast<int>(attempt),
            bus_warm_start_ptr != nullptr,
            McfGurobiRetryKind::None,
        };
        out.bus_res = solve_bus_mcf(
            graph,
            commodities,
            bus_ids,
            bbox_ctx,
            bus_warm_start_ptr,
            diag,
            gurobi_sink,
            bus_meta);
        bus_solve_ms_total += out.bus_res.solve_ms;
        out.bus_res.solve_ms = bus_solve_ms_total;
        if (out.bus_res.ok) {
            out.ok = true;
            return out;
        }

        const auto targets = collect_failed_bus_keys_for_expand(out.bus_res, bbox_ctx);
        if (targets.empty()) {
            out.bbox_exhausted = true;
            return out;
        }

        const auto expand_result = expand_bus_hulls(bbox_ctx, targets);
        out.bbox_expand_attempts = static_cast<int>(attempt + 1);
        log_bus_bbox_expand(static_cast<int>(attempt + 1), targets, expand_result, bbox_ctx);
        if (expand_result.any_exhausted) {
            if (out.bus_res.failed_bus_keys.empty()) {
                for (const auto& key : targets) {
                    append_unique(out.bus_res.failed_bus_keys, key);
                }
                out.bus_res.bus_failure_unlocalized = true;
            }
            out.bbox_exhausted = true;
            debug::info_fmt("MCF: BusMCF bbox expand exhausted; skipping SimpleMCF");
            return out;
        }
    }
}

auto build_unit_pre_route_paths(
    const StageSolveResult& bus_res,
    const StageWarmStart& unit_warm,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c
) -> std::Vector<McfPathInfo>;

auto build_unit_post_solve_paths(
    const StageSolveResult& bus_res,
    const StageSolveResult& simple_res,
    const std::size_t unit_c
) -> std::Vector<McfPathInfo>;

auto run_simple_mcf_unit_with_bbox_retry(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    McfBBoxContext& bbox_ctx,
    McfBBoxExpandState& expand_state,
    const std::map<std::pair<int, int>, int>& edge_cap,
    const std::map<int, int>& node_cap,
    const bool enable_pre_routing,
    const bool enable_mcf_obj,
    const StageSolveResult& bus_res,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    McfResourceUsageSink* resource_sink,
    const std::map<std::pair<int, int>, std::size_t>* arc_index,
    const GurobiDiagnosticsOptions& diag,
    McfGurobiLogSink* gurobi_sink,
    const int sat_tier_attempt
) -> UnitStageOutcome {
    auto out = UnitStageOutcome {};
    if (simple_ids_for_unit.empty()) {
        out.simple_res.stage_name = std::format("SimpleMCF_unit{}", unit_c);
        out.simple_res.message = "empty stage";
        apply_stage_solution_class(out.simple_res, McfSolutionClass::Skipped);
        if (gurobi_sink != nullptr) {
            gurobi_sink->write_skipped(McfGurobiLogStage::simple_unit(unit_c), "empty_stage");
        }
        out.ok = true;
        return out;
    }

    const auto use_tree_refine =
        enable_mcf_obj && unit_has_multi_fanout_origin(simple_ids_for_unit, commodities, records);
    const SimpleMcfUnitLogScope unit_log {unit_c, use_tree_refine, &out};

    const auto base_hull_for = [&](const McfSimpleOriginGroupKey& key) -> IlpBoundingBox {
        return base_simple_origin_hull(key, simple_ids_for_unit, commodities, records, bbox_ctx);
    };

    if (use_tree_refine) {
        debug::info_fmt(
            "SimpleMCF unit {} tree-refine: enabled=true",
            unit_c);
    }

    int unit_solve_ms_total = 0;
    auto last_pre_route_section = std::String {};
    for (std::size_t attempt = 0;; ++attempt) {
        if (!use_tree_refine) {
            const StageWarmStart* unit_warm_ptr = nullptr;
            auto unit_warm = StageWarmStart {};
            if (enable_pre_routing && stage_result_ok(bus_res.solution_class)) {
                unit_warm = run_simple_warm_start_for_unit(
                    graph,
                    commodities,
                    simple_ids_for_unit,
                    records,
                    bbox_ctx,
                    &expand_state,
                    bus_res,
                    unit_c,
                    outgoing_arcs);
                if (resource_sink != nullptr && arc_index != nullptr) {
                    const auto paths = build_unit_pre_route_paths(bus_res, unit_warm, commodities, unit_c);
                    const auto usage = aggregate_mcf_resource_usage_for_unit(graph, *arc_index, unit_c, paths);
                    last_pre_route_section = resource_sink->write_pre_route(unit_c, usage, true);
                }
                if (!unit_warm.nodes_by_record_id.empty()) {
                    unit_warm_ptr = &unit_warm;
                }
            }

            McfGurobiSolveMeta unit_meta {
                McfGurobiLogStage::simple_unit(unit_c),
                static_cast<int>(bbox_ctx.max_tier),
                sat_tier_attempt,
                static_cast<int>(attempt),
                unit_warm_ptr != nullptr,
                McfGurobiRetryKind::None,
            };
            out.simple_res = solve_simple_mcf_unit(
                graph,
                commodities,
                unit_c,
                simple_ids_for_unit,
                records,
                bbox_ctx,
                &expand_state,
                edge_cap,
                node_cap,
                enable_mcf_obj,
                unit_warm_ptr,
                diag,
                gurobi_sink,
                unit_meta,
                default_mcf_gurobi_solve_params(),
                SimpleMcfSolvePass::Standard,
                nullptr);
            unit_solve_ms_total += out.simple_res.solve_ms;
            out.simple_res.solve_ms = unit_solve_ms_total;
            if (out.simple_res.ok) {
                if (resource_sink != nullptr && arc_index != nullptr) {
                    const auto paths = build_unit_post_solve_paths(bus_res, out.simple_res, unit_c);
                    const auto usage = aggregate_mcf_resource_usage_for_unit(graph, *arc_index, unit_c, paths);
                    resource_sink->write_complete(unit_c, last_pre_route_section, usage, true);
                }
                out.ok = true;
                return out;
            }

            const auto origin_groups = collect_failed_simple_origin_groups(
                out.simple_res, unit_c, simple_ids_for_unit, commodities, records);
            if (origin_groups.empty()) {
                if (resource_sink != nullptr) {
                    resource_sink->write_empty(unit_c);
                }
                out.unit_exhausted = true;
                return out;
            }

            const auto expand_result = expand_simple_origin_hulls(expand_state, origin_groups, base_hull_for);
            log_simple_bbox_expand(unit_c, static_cast<int>(attempt + 1), origin_groups, expand_result, expand_state);
            if (expand_result.any_exhausted) {
                if (resource_sink != nullptr) {
                    resource_sink->write_empty(unit_c);
                }
                out.unit_exhausted = true;
                return out;
            }
            continue;
        }

        // Tree-refine path (twelfth edition): stage1 tree-seed -> compress -> stage2 refine.
        const StageWarmStart* unit_warm_ptr = nullptr;
        auto unit_warm = StageWarmStart {};
        if (enable_pre_routing && stage_result_ok(bus_res.solution_class)) {
            unit_warm = run_simple_warm_start_for_unit(
                graph,
                commodities,
                simple_ids_for_unit,
                records,
                bbox_ctx,
                &expand_state,
                bus_res,
                unit_c,
                outgoing_arcs);
            if (resource_sink != nullptr && arc_index != nullptr) {
                const auto paths = build_unit_pre_route_paths(bus_res, unit_warm, commodities, unit_c);
                const auto usage = aggregate_mcf_resource_usage_for_unit(graph, *arc_index, unit_c, paths);
                last_pre_route_section = resource_sink->write_pre_route(unit_c, usage, true);
            }
            if (!unit_warm.nodes_by_record_id.empty()) {
                unit_warm_ptr = &unit_warm;
            }
        }

        McfGurobiSolveMeta seed_meta {
            McfGurobiLogStage::simple_unit(unit_c),
            static_cast<int>(bbox_ctx.max_tier),
            sat_tier_attempt,
            static_cast<int>(attempt),
            unit_warm_ptr != nullptr,
            McfGurobiRetryKind::None,
        };
        seed_meta.pass = SimpleMcfSolvePass::TreeSeed;
        auto seed_params = default_mcf_gurobi_solve_params();
        seed_params.mip_gap = kSimpleMcfTreeSeedMipGap;
        seed_params.threads = 2;

        const auto stage1 = solve_simple_mcf_unit(
            graph,
            commodities,
            unit_c,
            simple_ids_for_unit,
            records,
            bbox_ctx,
            &expand_state,
            edge_cap,
            node_cap,
            enable_mcf_obj,
            unit_warm_ptr,
            diag,
            gurobi_sink,
            seed_meta,
            seed_params,
            SimpleMcfSolvePass::TreeSeed,
            nullptr);

        debug::info_fmt(
            "SimpleMCF unit {} tree-seed pass: mip_gap={:.2f} status={} solve_ms={}",
            unit_c,
            kSimpleMcfTreeSeedMipGap,
            solution_class_name(stage1.solution_class),
            stage1.solve_ms);

        if (!stage1.ok) {
            out.simple_res = stage1;
            unit_solve_ms_total += stage1.solve_ms;
            out.simple_res.solve_ms = unit_solve_ms_total;

            const auto origin_groups = collect_failed_simple_origin_groups(
                out.simple_res, unit_c, simple_ids_for_unit, commodities, records);
            if (origin_groups.empty()) {
                if (resource_sink != nullptr) {
                    resource_sink->write_empty(unit_c);
                }
                out.unit_exhausted = true;
                return out;
            }

            const auto expand_result = expand_simple_origin_hulls(expand_state, origin_groups, base_hull_for);
            log_simple_bbox_expand(unit_c, static_cast<int>(attempt + 1), origin_groups, expand_result, expand_state);
            if (expand_result.any_exhausted) {
                if (resource_sink != nullptr) {
                    resource_sink->write_empty(unit_c);
                }
                out.unit_exhausted = true;
                return out;
            }
            continue;
        }

        const auto compress_summary = compress_unit_multi_fanout_origins(
            graph,
            stage1.paths,
            records,
            unit_c,
            simple_origin_group_key);
        debug::info_fmt(
            "SimpleMCF unit {} tree-compress: refined_segments={} fallback_origins={}",
            unit_c,
            compress_summary.refined_segment_count,
            compress_summary.fallback_origin_count);

        auto [refined_com, refined_ids] =
            build_refined_unit_commodity_list(commodities, simple_ids_for_unit, records, unit_c, compress_summary);
        debug::info_fmt(
            "SimpleMCF unit {} refine model: commodities={}",
            unit_c,
            refined_com.size());

        auto refine_warm = build_guide_path_warm_start(refined_com);
        const StageWarmStart* refine_warm_ptr =
            refine_warm.nodes_by_record_id.empty() ? nullptr : &refine_warm;

        McfGurobiSolveMeta refine_meta {
            McfGurobiLogStage::simple_unit(unit_c),
            static_cast<int>(bbox_ctx.max_tier),
            sat_tier_attempt,
            static_cast<int>(attempt),
            refine_warm_ptr != nullptr,
            McfGurobiRetryKind::None,
        };
        refine_meta.pass = SimpleMcfSolvePass::Refine;
        auto refine_params = default_mcf_gurobi_solve_params();
        refine_params.threads = 2;

        auto refine_incumbent = build_refine_incumbent_seed_from_stage1(stage1);

        auto stage2 = solve_simple_mcf_unit(
            graph,
            refined_com,
            unit_c,
            refined_ids,
            records,
            bbox_ctx,
            &expand_state,
            edge_cap,
            node_cap,
            enable_mcf_obj,
            refine_warm_ptr,
            diag,
            gurobi_sink,
            refine_meta,
            refine_params,
            SimpleMcfSolvePass::Refine,
            &refine_incumbent);

        unit_solve_ms_total += stage1.solve_ms + stage2.solve_ms;
        debug::info_fmt(
            "SimpleMCF unit {} refine pass: status={} solve_ms={}",
            unit_c,
            solution_class_name(stage2.solution_class),
            stage2.solve_ms);

        if (!stage2.ok) {
            debug::error_fmt(
                "SimpleMCF unit {} tree-refine failed: stage1 feasible but refine pass failed ({}); modeling error",
                unit_c,
                stage2.message);
            out.simple_res = stage2;
            out.simple_res.solve_ms = unit_solve_ms_total;
            out.refine_modeling_error = true;
            out.ok = false;
            if (resource_sink != nullptr) {
                resource_sink->write_empty(unit_c);
            }
            return out;
        }

        stage2.paths =
            merge_refined_paths_for_output(graph, stage1.paths, stage2.paths, compress_summary, records);
        stage2.solve_ms = unit_solve_ms_total;
        out.simple_res = std::move(stage2);
        debug::info_fmt(
            "SimpleMCF unit {} tree-refine result: used=refine total_solve_ms={}",
            unit_c,
            unit_solve_ms_total);

        if (resource_sink != nullptr && arc_index != nullptr) {
            const auto paths = build_unit_post_solve_paths(bus_res, out.simple_res, unit_c);
            const auto usage = aggregate_mcf_resource_usage_for_unit(graph, *arc_index, unit_c, paths);
            resource_sink->write_complete(unit_c, last_pre_route_section, usage, true);
        }
        out.ok = true;
        return out;
    }
}

auto build_unit_pre_route_paths(
    const StageSolveResult& bus_res,
    const StageWarmStart& unit_warm,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c
) -> std::Vector<McfPathInfo> {
    auto paths = std::Vector<McfPathInfo> {};
    for (const auto& path_info : bus_res.paths) {
        if (path_info.cob_unit == unit_c) {
            paths.push_back(path_info);
        }
    }

    auto commodity_by_record_id = std::map<std::size_t, const PreparedCommodity*> {};
    for (const auto& commodity : commodities) {
        if (commodity.is_bus) {
            continue;
        }
        commodity_by_record_id.emplace(commodity.record_id, &commodity);
    }

    for (const auto& [record_id, path] : unit_warm.nodes_by_record_id) {
        const auto it = commodity_by_record_id.find(record_id);
        if (it == commodity_by_record_id.end() || it->second == nullptr) {
            continue;
        }
        const auto& c = *it->second;
        if (c.cob_unit != unit_c) {
            continue;
        }
        McfPathInfo info {};
        info.label = c.label;
        info.origin_name = c.origin_name;
        info.record_id = c.record_id;
        info.src = c.src;
        info.snk = c.snk;
        info.demand = c.demand;
        info.cob_unit = c.cob_unit;
        info.start_track = c.start_track;
        info.end_track = c.end_track;
        if (!c.record_indices.empty()) {
            info.record_indices = c.record_indices;
        }
        else {
            info.record_indices.push_back(c.record_id);
        }
        info.unit_paths.push_back(path);
        paths.push_back(std::move(info));
    }
    return paths;
}

auto build_unit_post_solve_paths(
    const StageSolveResult& bus_res,
    const StageSolveResult& simple_res,
    const std::size_t unit_c
) -> std::Vector<McfPathInfo> {
    auto paths = std::Vector<McfPathInfo> {};
    for (const auto& path_info : bus_res.paths) {
        if (path_info.cob_unit == unit_c) {
            paths.push_back(path_info);
        }
    }
    for (const auto& path_info : simple_res.paths) {
        paths.push_back(path_info);
    }
    return paths;
}

} // namespace

auto run_mcf_global_routing_cob_units(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const TobPathPrecomputeCache& path_cache,
    hardware::Interposer* interposer,
    const circuit::BaseDie& basedie,
    const CobMcfGridDims cob_grid,
    const bool enable_mcf_parallel,
    const bool enable_pre_routing,
    const bool enable_mcf_obj,
    const bool defer_interposer_suspend,
    const bool disable_bus_mcf,
    const bool show_resource_usage,
    const GurobiDiagnosticsOptions& diag,
    const int sat_tier_attempt
) -> CobMcfFullResult {

    const auto mcf_start = std::chrono::steady_clock::now();
    const auto peak_before = get_peak_rss_mb();
    McfGurobiLogSink gurobi_sink {std::filesystem::path(diag.log_dir)};
    debug::info_fmt("MCF Gurobi params: {}", format_mcf_gurobi_solve_params(default_mcf_gurobi_solve_params()));
    if (disable_bus_mcf) {
        debug::info_fmt(
            "MCF: SimpleMCF only (--disable-bus-mcf); SimpleMCF objective={}",
            enable_mcf_obj ? "min_sum_x (--enable-mcf-obj)" : "feasibility only");
    }
    else {
        debug::info_fmt(
            "MCF: BusMCF (global) + SimpleMCF (per COBUnit); SimpleMCF objective={}",
            enable_mcf_obj ? "min_sum_x (--enable-mcf-obj)" : "feasibility only");
    }

    CobMcfFullResult out {};
    out.summary.per_cob.resize(16);
    if (records.size() != ilp_result.assignments.size() || records.size() != ilp_result.record_track_endpoints.size()) {
        for (std::size_t u = 0; u < 16; ++u) {
            out.summary.per_cob[u] = CobMcfCobUnitSummary {
                u,
                0,
                false,
                0.0,
                0,
                std::String("record/SAT result size mismatch")};
        }
        out.summary.all_ok = false;
        return out;
    }

    auto graph = build_track_graph(cob_grid);
    auto commodities = prepare_commodities(records, ilp_result, cob_grid, graph);
    auto bus_ids = std::Vector<std::size_t> {};
    auto simple_ids = std::Vector<std::size_t> {};
    auto bus_count_by_unit = std::array<int, 16> {};
    auto simple_count_by_unit = std::array<int, 16> {};
    bus_count_by_unit.fill(0);
    simple_count_by_unit.fill(0);
    for (std::size_t i = 0; i < commodities.size(); ++i) {
        if (commodities[i].is_bus) {
            bus_ids.push_back(i);
            bus_count_by_unit[commodities[i].cob_unit] += 1;
        }
        else {
            simple_ids.push_back(i);
            simple_count_by_unit[commodities[i].cob_unit] += 1;
        }
    }

    auto simple_ids_by_unit = std::array<std::Vector<std::size_t>, 16> {};
    for (const auto id : simple_ids) {
        simple_ids_by_unit[commodities[id].cob_unit].push_back(id);
    }

    const auto bbox_inputs = to_bbox_inputs(commodities);
    auto bbox_ctx = build_mcf_bbox_context(records, bbox_inputs, ilp_result, path_cache);
    debug::info_fmt("MCF using SAT path bbox max_tier={}", bbox_ctx.max_tier);

    auto expand_states = std::array<McfBBoxExpandState, 16> {};
    auto outgoing_arcs = std::Vector<std::Vector<int>> {};
    if (enable_pre_routing) {
        outgoing_arcs.resize(graph.nodes.size());
        for (std::size_t a = 0; a < graph.arcs.size(); ++a) {
            outgoing_arcs[static_cast<std::size_t>(graph.arcs[a].u)].push_back(static_cast<int>(a));
        }
    }

    const auto solve_t0 = std::chrono::steady_clock::now();
    const auto bus_outcome = run_bus_mcf_stage_with_bbox_retry(
        graph,
        commodities,
        bus_ids,
        disable_bus_mcf,
        enable_pre_routing,
        diag,
        bbox_ctx,
        &gurobi_sink,
        sat_tier_attempt);
    auto bus_res = bus_outcome.bus_res;
    out.summary.bus_mcf_solve_ms = bus_res.solve_ms;
    debug::info_fmt("timing phase=mcf_bus_warm_start ms={}", bus_outcome.warm_start_ms);
    debug::info_fmt("timing phase=mcf_bus_solve ms={}", out.summary.bus_mcf_solve_ms);

    McfResourceUsageSink* resource_sink_ptr = nullptr;
    std::unique_ptr<McfResourceUsageSink> resource_sink {};
    std::map<std::pair<int, int>, std::size_t> arc_index {};
    if (show_resource_usage && bus_outcome.ok) {
        arc_index = build_undirected_arc_index(graph);
        resource_sink = std::make_unique<McfResourceUsageSink>(
            build_mcf_resource_catalog(graph),
            std::filesystem::path(kMcfResourceUsageDir));
        resource_sink->prepare_output_dir();
        resource_sink_ptr = resource_sink.get();
    }

    auto simple_results = std::array<StageSolveResult, 16> {};
    auto refine_modeling_error_by_unit = std::array<bool, 16> {};
    refine_modeling_error_by_unit.fill(false);
    auto has_simple_unit = std::array<bool, 16> {};
    has_simple_unit.fill(false);
    bool all_simple_ok = true;
    double simple_objective = 0.0;
    bool serial_abort = false;

    if (!bus_outcome.ok) {
        merge_bus_stage_retry_hints(out.retry_hints, bus_res);
        for (std::size_t u = 0; u < 16; ++u) {
            if (simple_ids_by_unit[u].empty()) {
                apply_stage_solution_class(simple_results[u], McfSolutionClass::Skipped);
                simple_results[u].message = "empty stage";
                gurobi_sink.write_skipped(McfGurobiLogStage::simple_unit(u), "empty_stage");
                out.has_simple_commodities[u] = false;
                out.simple_mcf_ok[u] = true;
                continue;
            }
            has_simple_unit[u] = true;
            out.has_simple_commodities[u] = true;
            apply_stage_solution_class(simple_results[u], McfSolutionClass::Skipped);
            simple_results[u].message = "skipped after BusMCF bbox expand exhausted";
            gurobi_sink.write_skipped(
                McfGurobiLogStage::simple_unit(u),
                "skipped_after_bus_bbox_exhausted");
            out.simple_mcf_ok[u] = false;
            all_simple_ok = false;
        }
    }
    else {
        auto build_edge_residual = [&](const std::size_t unit_c) {
            auto edge_cap = std::map<std::pair<int, int>, int> {};
            for (const auto& [e, used] : bus_res.unit_used_edges[unit_c]) {
                edge_cap[e] = std::max(0, 1 - used);
            }
            return edge_cap;
        };
        auto build_node_residual = [&](const std::size_t unit_c) {
            auto node_cap = std::map<int, int> {};
            for (const auto& [n, used] : bus_res.unit_used_nodes[unit_c]) {
                node_cap[n] = std::max(0, 1 - used);
            }
            return node_cap;
        };

        if (enable_mcf_parallel) {
            constexpr int kUnitParallelThreadHint = 2;
            const auto max_concurrent_units = std::min<std::size_t>(
                16,
                static_cast<std::size_t>(std::max(1, kMcfGurobiThreadCap / kUnitParallelThreadHint)));
            debug::info_fmt(
                "MCF unit parallel: max_concurrent_units={} thread_cap={}",
                max_concurrent_units,
                kMcfGurobiThreadCap);

            auto pending_units = std::Vector<std::size_t> {};
            for (std::size_t u = 0; u < 16; ++u) {
                if (simple_ids_by_unit[u].empty()) {
                    apply_stage_solution_class(simple_results[u], McfSolutionClass::Skipped);
                    simple_results[u].message = "empty stage";
                    gurobi_sink.write_skipped(McfGurobiLogStage::simple_unit(u), "empty_stage");
                    log_skipped_simple_unit_stage(simple_results[u], u);
                    if (resource_sink_ptr != nullptr) {
                        resource_sink_ptr->write_empty(u);
                    }
                    continue;
                }
                has_simple_unit[u] = true;
                pending_units.push_back(u);
            }

            for (std::size_t wave_begin = 0; wave_begin < pending_units.size(); wave_begin += max_concurrent_units) {
                const auto wave_end =
                    std::min(pending_units.size(), wave_begin + max_concurrent_units);
                auto futures = std::Vector<std::future<UnitStageOutcome>> {};
                futures.reserve(wave_end - wave_begin);
                for (std::size_t wi = wave_begin; wi < wave_end; ++wi) {
                    const auto u = pending_units[wi];
                    const auto edge_cap = build_edge_residual(u);
                    const auto node_cap = build_node_residual(u);
                    futures.push_back(std::async(
                        std::launch::async,
                        [&graph,
                         &commodities,
                         &records,
                         &bbox_ctx,
                         &expand_states,
                         &simple_ids_by_unit,
                         &bus_res,
                         &outgoing_arcs,
                         &arc_index,
                         &gurobi_sink,
                         u,
                         edge_cap,
                         node_cap,
                         enable_pre_routing,
                         enable_mcf_obj,
                         resource_sink_ptr,
                         sat_tier_attempt,
                         diag]() {
                            return run_simple_mcf_unit_with_bbox_retry(
                                graph,
                                commodities,
                                u,
                                simple_ids_by_unit[u],
                                records,
                                bbox_ctx,
                                expand_states[u],
                                edge_cap,
                                node_cap,
                                enable_pre_routing,
                                enable_mcf_obj,
                                bus_res,
                                outgoing_arcs,
                                resource_sink_ptr,
                                resource_sink_ptr != nullptr ? &arc_index : nullptr,
                                diag,
                                &gurobi_sink,
                                sat_tier_attempt);
                        }));
                }
                for (std::size_t fi = 0; fi < futures.size(); ++fi) {
                    const auto u = pending_units[wave_begin + fi];
                    const auto unit_outcome = futures[fi].get();
                    simple_results[u] = unit_outcome.simple_res;
                    refine_modeling_error_by_unit[u] = unit_outcome.refine_modeling_error;
                    debug::info_fmt(
                        "SimpleMCF unit {}: ok={} solution_class={} objective={:.0f} paths={} solve_ms={}",
                        u,
                        simple_results[u].ok,
                        solution_class_name(simple_results[u].solution_class),
                        simple_results[u].objective,
                        simple_results[u].paths.size(),
                        simple_results[u].solve_ms);
                    if (!unit_outcome.ok) {
                        all_simple_ok = false;
                    }
                }
            }
        }
        else {
            for (std::size_t u = 0; u < 16; ++u) {
                if (simple_ids_by_unit[u].empty()) {
                    apply_stage_solution_class(simple_results[u], McfSolutionClass::Skipped);
                    simple_results[u].message = "empty stage";
                    gurobi_sink.write_skipped(McfGurobiLogStage::simple_unit(u), "empty_stage");
                    log_skipped_simple_unit_stage(simple_results[u], u);
                    if (resource_sink_ptr != nullptr) {
                        resource_sink_ptr->write_empty(u);
                    }
                    continue;
                }
                if (serial_abort) {
                    has_simple_unit[u] = true;
                    apply_stage_solution_class(simple_results[u], McfSolutionClass::Skipped);
                    simple_results[u].message = "skipped after prior unit bbox expand exhausted";
                    gurobi_sink.write_skipped(
                        McfGurobiLogStage::simple_unit(u),
                        "skipped_after_prior_unit_bbox_exhausted");
                    log_skipped_simple_unit_stage(simple_results[u], u);
                    all_simple_ok = false;
                    if (resource_sink_ptr != nullptr) {
                        resource_sink_ptr->write_empty(u);
                    }
                    continue;
                }
                has_simple_unit[u] = true;
                const auto edge_cap = build_edge_residual(u);
                const auto node_cap = build_node_residual(u);
                const auto unit_outcome = run_simple_mcf_unit_with_bbox_retry(
                    graph,
                    commodities,
                    u,
                    simple_ids_by_unit[u],
                    records,
                    bbox_ctx,
                    expand_states[u],
                    edge_cap,
                    node_cap,
                    enable_pre_routing,
                    enable_mcf_obj,
                    bus_res,
                    outgoing_arcs,
                    resource_sink_ptr,
                    resource_sink_ptr != nullptr ? &arc_index : nullptr,
                    diag,
                    &gurobi_sink,
                    sat_tier_attempt);
                simple_results[u] = unit_outcome.simple_res;
                refine_modeling_error_by_unit[u] = unit_outcome.refine_modeling_error;
                debug::info_fmt(
                    "SimpleMCF unit {}: ok={} solution_class={} objective={:.0f} paths={} solve_ms={}",
                    u,
                    simple_results[u].ok,
                    solution_class_name(simple_results[u].solution_class),
                    simple_results[u].objective,
                    simple_results[u].paths.size(),
                    simple_results[u].solve_ms);
                if (!unit_outcome.ok) {
                    all_simple_ok = false;
                    if (unit_outcome.unit_exhausted) {
                        serial_abort = true;
                        debug::info_fmt(
                            "MCF: SimpleMCF unit {} bbox expand exhausted; serial abort units {}..15",
                            u,
                            u + 1);
                    }
                }
            }
        }
    }

    out.summary.mcf_warm_start_ms = bus_outcome.warm_start_ms;
    debug::info_fmt("timing phase=mcf_warm_start ms={}", out.summary.mcf_warm_start_ms);

    for (std::size_t u = 0; u < 16; ++u) {
        out.has_simple_commodities[u] = has_simple_unit[u];
        if (!has_simple_unit[u]) {
            out.simple_mcf_ok[u] = true;
            continue;
        }
        out.simple_mcf_ok[u] = simple_results[u].ok;
        out.summary.simple_mcf_solve_ms_by_unit[u] = simple_results[u].solve_ms;
        if (!simple_results[u].ok && !refine_modeling_error_by_unit[u]) {
            debug::error_fmt("SimpleMCF unit {} failed: {}", u, simple_results[u].message);
            add_simple_unit_retry_hints(
                out.retry_hints,
                u,
                simple_ids_by_unit,
                commodities,
                records);
        }
        else if (!simple_results[u].ok) {
            debug::error_fmt(
                "SimpleMCF unit {} refine modeling error (no tier retry): {}",
                u,
                simple_results[u].message);
        }
        simple_objective += simple_results[u].objective;
    }

    const auto solve_t1 = std::chrono::steady_clock::now();
    const auto solve_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(solve_t1 - solve_t0).count());
    out.summary.mcf_solve_ms = solve_ms;
    for (std::size_t u = 0; u < 16; ++u) {
        debug::info_fmt(
            "timing phase=simple_mcf_unit{}_solve ms={}",
            u,
            out.summary.simple_mcf_solve_ms_by_unit[u]);
    }
    debug::info_fmt("timing phase=mcf_solve ms={}", solve_ms);

    if (!bus_outcome.ok) {
        debug::error_fmt("BusMCF failed: {}", bus_res.message);
    }
    out.summary.all_ok = bus_outcome.ok && all_simple_ok;
    normalize_retry_hints(out.retry_hints);
    if (!out.summary.all_ok) {
        debug::info_fmt(
            "MCF retry hints: bus_keys={} simple_units={} record_indices={} bus_unlocalized={}",
            out.retry_hints.failed_bus_keys.size(),
            out.retry_hints.failed_simple_units.size(),
            out.retry_hints.failed_record_indices.size(),
            out.retry_hints.bus_failure_unlocalized);
    }

    for (std::size_t u = 0; u < 16; ++u) {
        const auto obj = bus_res.objective + simple_objective;
        out.summary.per_cob[u] = CobMcfCobUnitSummary {
            u,
            bus_count_by_unit[u] + simple_count_by_unit[u],
            out.summary.all_ok,
            obj,
            bus_res.solve_ms + out.summary.simple_mcf_solve_ms_by_unit[u],
            out.summary.all_ok ? std::String("ok") : std::String("stage failed")};
    }

    for (auto& p : bus_res.paths) {
        out.paths_by_unit[p.cob_unit].push_back(std::move(p));
    }
    for (std::size_t u = 0; u < 16; ++u) {
        for (auto& p : simple_results[u].paths) {
            out.paths_by_unit[u].push_back(std::move(p));
        }
    }

    for (std::size_t u = 0; u < 16; ++u) {
        debug::info_fmt(
            "MCF unit {}: bus_com={} simple_com={} ok={}",
            u,
            bus_count_by_unit[u],
            simple_count_by_unit[u],
            out.summary.all_ok);
        for (const auto& info : out.paths_by_unit[u]) {
            debug::info_fmt(
                "  commodity {} rec={} start_track={} end_track={} path_count={} (track path text under origin net below)",
                info.label,
                info.record_id,
                info.start_track,
                info.end_track,
                info.unit_paths.size());
        }
    }
    out.summary.total_wire_length = log_mcf_paths_by_origin_net(graph, out.paths_by_unit, records);

    if (!out.summary.all_ok) {
        log_mcf_infeasibility_summary(bus_res, simple_results);
    }

    const auto mcf_end = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(mcf_end - mcf_start).count();
    const auto peak_after = get_peak_rss_mb();
    const auto stage_peak_delta = std::max(0.0, peak_after - peak_before);
    debug::info_fmt(
        "MCF detailed(track-level) summary: total_elapsed={} ms (mcf_warm_start={} ms, mcf_solve={} ms), peak_rss={:.2f} MB, "
        "stage_peak_delta={:.2f} MB",
        elapsed_ms,
        out.summary.mcf_warm_start_ms,
        out.summary.mcf_solve_ms,
        peak_after,
        stage_peak_delta);
    if (interposer != nullptr && !defer_interposer_suspend) {
        suspend_mcf_paths_on_interposer(interposer, graph, out.paths_by_unit);
    }
    return out;
}

auto build_mcf_track_graph(const CobMcfGridDims grid) -> McfGlobalGraph {
    return build_track_graph(grid);
}

auto is_sync_bus_mcf_origin_key(const std::String& origin_key) -> bool {
    return is_sync_bus_origin_key(origin_key);
}

namespace {

auto track_from_node_meta_impl(hardware::Interposer* interposer, const McfNodeMeta& m) -> hardware::Track* {
    if (interposer == nullptr || m.is_virtual) {
        return nullptr;
    }
    const auto dir = m.track_dir == 0 ? hardware::TrackDirection::Horizontal : hardware::TrackDirection::Vertical;
    const auto tc = hardware::TrackCoord {
        static_cast<std::i64>(m.track_row),
        static_cast<std::i64>(m.track_col),
        dir,
        static_cast<std::usize>(m.track)};
    const auto opt = interposer->get_track(tc);
    return opt.has_value() ? opt.value() : nullptr;
}

} // namespace

auto suspend_mcf_paths_on_interposer(
    hardware::Interposer* interposer,
    const McfGlobalGraph& graph,
    const std::array<std::Vector<McfPathInfo>, 16>& paths_by_unit
) -> void {
    if (interposer == nullptr) {
        return;
    }
    int suspended = 0;
    int skipped_no_adj = 0;
    for (std::size_t u = 0; u < 16; ++u) {
        for (const auto& info : paths_by_unit[u]) {
            for (const auto& path : info.unit_paths) {
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const int na = path[i - 1];
                    const int nb = path[i];
                    if (na < 0 || nb < 0 || static_cast<std::size_t>(na) >= graph.nodes.size()
                        || static_cast<std::size_t>(nb) >= graph.nodes.size()) {
                        continue;
                    }
                    const auto& ma = graph.nodes[static_cast<std::size_t>(na)];
                    const auto& mb = graph.nodes[static_cast<std::size_t>(nb)];
                    if (ma.is_virtual || mb.is_virtual) {
                        continue;
                    }
                    auto* ta = track_from_node_meta_impl(interposer, ma);
                    auto* tb = track_from_node_meta_impl(interposer, mb);
                    if (ta == nullptr || tb == nullptr) {
                        continue;
                    }
                    bool found = false;
                    for (auto [tn, conn] : interposer->adjacent_tracks(ta)) {
                        if (tn != tb) {
                            continue;
                        }
                        found = true;
                        if (!conn.is_occupied()) {
                            conn.suspend();
                            suspended += 1;
                        }
                        break;
                    }
                    if (!found) {
                        skipped_no_adj += 1;
                    }
                }
            }
        }
    }
    if (skipped_no_adj > 0) {
        debug::warning_fmt(
            "MCF→Interposer: {} hop(s) had no matching adjacent_tracks() edge (graph vs hardware mismatch?)",
            skipped_no_adj);
    }
    debug::info_fmt("MCF→Interposer: suspended {} COBConnector(s) along MCF paths", suspended);
}

} // namespace PR_tool
