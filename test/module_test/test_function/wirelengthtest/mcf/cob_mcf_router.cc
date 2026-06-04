#include "mcf/cob_mcf_router.hh"

#include "mcf/mcf_graph.hh"
#include "mcf/mcf_hw_map.hh"

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
#include <future>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <tuple>
#include <utility>
#include <vector>

namespace PR_tool {

namespace {

using namespace mcf;

enum class McfClass : int {
    Plain = 0,
    P = 1,
    N = 2
};

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
    McfClass cls{McfClass::Plain};
    bool is_bus{false};
    std::String bus_key;
    std::Vector<IlpReachStep> reach_steps;
    std::Vector<int> bbox_cobs;
};

struct McfConstraintMeta {
    std::String kind;
    std::String detail;
};

struct StageSolveResult {
    bool ok{false};
    std::String message;
    std::String stage_name;
    double objective{0.0};
    int model_status{0};
    std::map<std::pair<int, int>, int> used_edges;
    std::map<int, int> used_nodes;
    std::array<std::map<std::pair<int, int>, int>, 16> unit_used_edges {};
    std::array<std::map<int, int>, 16> unit_used_nodes {};
    std::Vector<McfPathInfo> paths;
    std::Vector<McfConstraintMeta> infeasibility_hints;
};

struct StageWarmStart {
    std::map<std::size_t, std::Vector<int>> nodes_by_record_id;
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
    std::String message;
    int model_status{0};
    double objective{0.0};
    std::vector<double> col_value;
    std::Vector<McfConstraintMeta> iis_rows;
};

constexpr int kHardwareSwitchesPerCobUnit = 48;
constexpr int kChannelsPerCobLink = 8;
constexpr int kMaxIisLogPerKind = 20;

auto normalized_edge_key(int u, int v) -> std::pair<int, int>;
auto node_text(const GlobalGraph& g, const int node) -> std::String;
auto fmt_join_parts(const std::Vector<std::String>& parts) -> std::String;

struct HChannelKey {
    int r{0};
    int c{0};

    auto operator<=>(const HChannelKey&) const = default;
};

struct VChannelKey {
    int r{0};
    int c{0};

    auto operator<=>(const VChannelKey&) const = default;
};

struct McfResourceCatalog {
    int rows{0};
    int cols{0};
    std::array<std::array<std::array<int, 32>, 32>, 16> switch_modeled {};
    std::array<std::map<HChannelKey, int>, 16> h_channel_total {};
    std::array<std::map<VChannelKey, int>, 16> v_channel_total {};
};

struct McfResourceUsage {
    int rows{0};
    int cols{0};
    std::array<std::array<std::array<int, 32>, 32>, 16> switches_used {};
    std::array<std::map<HChannelKey, int>, 16> h_channels_used {};
    std::array<std::map<VChannelKey, int>, 16> v_channels_used {};
};

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

auto h_channel_key_from_arc(const Arc& arc, const int cols) -> std::optional<HChannelKey> {
    if (arc.is_turn || arc.is_virtual) {
        return std::nullopt;
    }
    const auto [cob_r, cob_c] = arc_cob_row_col(arc, cols);
    const bool lr = (arc.from_dir == hardware::COBDirection::Left && arc.to_dir == hardware::COBDirection::Right)
        || (arc.from_dir == hardware::COBDirection::Right && arc.to_dir == hardware::COBDirection::Left);
    if (!lr) {
        return std::nullopt;
    }
    return HChannelKey {cob_r, cob_c};
}

auto v_channel_key_from_arc(const Arc& arc, const int cols) -> std::optional<VChannelKey> {
    if (arc.is_turn || arc.is_virtual) {
        return std::nullopt;
    }
    const auto [cob_r, cob_c] = arc_cob_row_col(arc, cols);
    const bool ud = (arc.from_dir == hardware::COBDirection::Up && arc.to_dir == hardware::COBDirection::Down)
        || (arc.from_dir == hardware::COBDirection::Down && arc.to_dir == hardware::COBDirection::Up);
    if (!ud) {
        return std::nullopt;
    }
    return VChannelKey {cob_r, cob_c};
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

auto build_mcf_resource_catalog(const GlobalGraph& graph) -> McfResourceCatalog {
    McfResourceCatalog catalog {};
    catalog.rows = graph.rows;
    catalog.cols = graph.cols;
    const auto cols = graph.cols;
    auto switch_seen = std::array<std::array<std::array<std::set<std::pair<int, int>>, 32>, 32>, 16> {};
    auto h_seen = std::array<std::map<HChannelKey, std::set<std::pair<int, int>>>, 16> {};
    auto v_seen = std::array<std::map<VChannelKey, std::set<std::pair<int, int>>>, 16> {};

    for (const auto& arc : graph.arcs) {
        if (arc.is_virtual) {
            continue;
        }
        const auto u = static_cast<std::size_t>(arc.unit);
        if (u >= 16) {
            continue;
        }
        const auto [cob_r, cob_c] = arc_cob_row_col(arc, graph.cols);
        if (cob_r < 0 || cob_c < 0 || cob_r >= catalog.rows || cob_c >= catalog.cols) {
            continue;
        }
        const auto edge_key = normalized_edge_key(arc.u, arc.v);
        if (arc.is_turn) {
            if (!switch_seen[u][static_cast<std::size_t>(cob_r)][static_cast<std::size_t>(cob_c)].contains(edge_key)) {
                switch_seen[u][static_cast<std::size_t>(cob_r)][static_cast<std::size_t>(cob_c)].insert(edge_key);
                ++catalog.switch_modeled[u][static_cast<std::size_t>(cob_r)][static_cast<std::size_t>(cob_c)];
            }
        }
        else if (const auto h = h_channel_key_from_arc(arc, graph.cols)) {
            if (!h_seen[u][*h].contains(edge_key)) {
                h_seen[u][*h].insert(edge_key);
                ++catalog.h_channel_total[u][*h];
            }
        }
        else if (const auto v = v_channel_key_from_arc(arc, graph.cols)) {
            if (!v_seen[u][*v].contains(edge_key)) {
                v_seen[u][*v].insert(edge_key);
                ++catalog.v_channel_total[u][*v];
            }
        }
    }
    return catalog;
}

auto aggregate_mcf_resource_usage(
    const GlobalGraph& graph,
    const std::map<std::pair<int, int>, std::size_t>& arc_index,
    const StageSolveResult& bus_res,
    const std::array<StageSolveResult, 16>& simple_results
) -> McfResourceUsage {
    McfResourceUsage usage {};
    usage.rows = graph.rows;
    usage.cols = graph.cols;

    auto absorb_edge = [&](const std::size_t unit, const std::pair<int, int>& edge) {
        if (unit >= 16) {
            return;
        }
        const auto it = arc_index.find(edge);
        if (it == arc_index.end()) {
            return;
        }
        const auto& arc = graph.arcs[it->second];
        if (arc.unit != unit) {
            return;
        }
        const auto [cob_r, cob_c] = arc_cob_row_col(arc, graph.cols);
        if (cob_r < 0 || cob_c < 0 || cob_r >= usage.rows || cob_c >= usage.cols) {
            return;
        }
        if (arc.is_turn) {
            ++usage.switches_used[unit][static_cast<std::size_t>(cob_r)][static_cast<std::size_t>(cob_c)];
        }
        else if (const auto h = h_channel_key_from_arc(arc, graph.cols)) {
            ++usage.h_channels_used[unit][*h];
        }
        else if (const auto v = v_channel_key_from_arc(arc, graph.cols)) {
            ++usage.v_channels_used[unit][*v];
        }
    };

    for (std::size_t u = 0; u < 16; ++u) {
        for (const auto& [edge, used] : bus_res.unit_used_edges[u]) {
            if (used > 0) {
                absorb_edge(u, edge);
            }
        }
        for (const auto& [edge, used] : simple_results[u].used_edges) {
            if (used > 0) {
                absorb_edge(u, edge);
            }
        }
    }
    return usage;
}

auto log_mcf_resource_usage(
    const McfResourceCatalog& catalog,
    const McfResourceUsage& usage,
    const bool all_ok
) -> void {
    debug::info_fmt("MCF resource usage (post-solve, all_ok={})", all_ok);
    for (std::size_t u = 0; u < 16; ++u) {
        debug::info_fmt("Unit {}:", u);
        int switch_used_sum = 0;
        int switch_total_sum = 0;
        int channel_used_sum = 0;
        int channel_total_sum = 0;
        int switch_modeled_sum = 0;

        for (int r = 0; r < catalog.rows; ++r) {
            for (int c = 0; c < catalog.cols; ++c) {
                const auto used = usage.switches_used[u][static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
                const auto modeled = catalog.switch_modeled[u][static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
                debug::info_fmt("  switches COB({},{})={}/{}", r, c, used, kHardwareSwitchesPerCobUnit);
                switch_used_sum += used;
                switch_total_sum += kHardwareSwitchesPerCobUnit;
                switch_modeled_sum += modeled;
            }
        }
        for (int r = 0; r < catalog.rows; ++r) {
            for (int c = 0; c + 1 < catalog.cols; ++c) {
                const HChannelKey key {r, c};
                const auto used = usage.h_channels_used[u].contains(key) ? usage.h_channels_used[u].at(key) : 0;
                const auto total = catalog.h_channel_total[u].contains(key)
                    ? catalog.h_channel_total[u].at(key)
                    : kChannelsPerCobLink;
                debug::info_fmt(
                    "  channel H COB({},{})-COB({},{})={}/{}",
                    r,
                    c,
                    r,
                    c + 1,
                    used,
                    total);
                channel_used_sum += used;
                channel_total_sum += total;
            }
        }
        for (int r = 0; r + 1 < catalog.rows; ++r) {
            for (int c = 0; c < catalog.cols; ++c) {
                const VChannelKey key {r, c};
                const auto used = usage.v_channels_used[u].contains(key) ? usage.v_channels_used[u].at(key) : 0;
                const auto total = catalog.v_channel_total[u].contains(key)
                    ? catalog.v_channel_total[u].at(key)
                    : kChannelsPerCobLink;
                debug::info_fmt(
                    "  channel V COB({},{})-COB({},{})={}/{}",
                    r,
                    c,
                    r + 1,
                    c,
                    used,
                    total);
                channel_used_sum += used;
                channel_total_sum += total;
            }
        }
        debug::info_fmt(
            "  unit_summary switches={}/{} modeled={} channels={}/{}",
            switch_used_sum,
            switch_total_sum,
            switch_modeled_sum,
            channel_used_sum,
            channel_total_sum);
    }
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
            "MCF failure diagnosis: stage={} status={}({}) message={}",
            bus_res.stage_name.empty() ? std::String("BusMCF") : bus_res.stage_name,
            gurobi_status_name(bus_res.model_status),
            bus_res.model_status,
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
                "MCF failure diagnosis: stage={} status={}({}) message={}",
                simple_results[u].stage_name.empty() ? std::format("SimpleMCF_unit{}", u) : simple_results[u].stage_name,
                gurobi_status_name(simple_results[u].model_status),
                simple_results[u].model_status,
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
    const std::Vector<McfConstraintMeta>* row_meta
) -> GurobiMcfSolveResult {
    auto out = GurobiMcfSolveResult {};
    try {
        GRBEnv env {true};
        env.set(GRB_IntParam_OutputFlag, 0);
        env.start();

        GRBModel model {env};
        model.set(GRB_StringAttr_ModelName, stage_name);
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
        model.set(GRB_IntParam_OutputFlag, 0);
        model.set(GRB_IntParam_Presolve, 1);

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

        model.optimize();
        out.model_status = model.get(GRB_IntAttr_Status);
        if (out.model_status == GRB_OPTIMAL) {
            out.objective = model.get(GRB_DoubleAttr_ObjVal);
            out.col_value.resize(vars.size(), 0.0);
            for (std::size_t c = 0; c < vars.size(); ++c) {
                out.col_value[c] = vars[c].get(GRB_DoubleAttr_X);
            }
        }
        else if (out.model_status == GRB_INFEASIBLE && row_meta != nullptr && row_meta->size() == row_lo.size()) {
            model.computeIIS();
            const auto num_constrs = model.get(GRB_IntAttr_NumConstrs);
            const auto constrs = model.getConstrs();
            auto seen_rows = std::set<std::size_t> {};
            for (int ci = 0; ci < num_constrs; ++ci) {
                const auto& constr = constrs[ci];
                if (constr.get(GRB_IntAttr_IISConstr) == 0) {
                    continue;
                }
                const auto name = constr.get(GRB_StringAttr_ConstrName);
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
        }
        out.ok = true;
        out.message = "ok";
        return out;
    }
    catch (const GRBException& e) {
        out.ok = false;
        out.message = std::format("{}: Gurobi exception {}: {}", stage_name, e.getErrorCode(), e.getMessage());
        return out;
    }
    catch (const std::exception& e) {
        out.ok = false;
        out.message = std::format("{}: Gurobi solve failed: {}", stage_name, e.what());
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
            return std::String("V_P");
        }
        if (meta.virtual_kind == 2) {
            return std::String("V_N");
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
    g.vp_node = add_node(g, NodeMeta {true, 1, 0, 0, 0, 0, 0});
    g.vn_node = add_node(g, NodeMeta {true, 2, 0, 0, 0, 0, 0});

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

auto bbox_cob_indices(
    const CobMcfGridDims& grid,
    const hardware::COBCoord& a,
    const hardware::COBCoord& b
) -> std::Vector<int> {
    std::Vector<int> out {};
    const auto r0 = std::max<std::i64>(0, std::min(a.row, b.row));
    const auto r1 = std::min<std::i64>(grid.rows - 1, std::max(a.row, b.row));
    const auto c0 = std::max<std::i64>(0, std::min(a.col, b.col));
    const auto c1 = std::min<std::i64>(grid.cols - 1, std::max(a.col, b.col));
    for (std::i64 r = r0; r <= r1; ++r) {
        for (std::i64 c = c0; c <= c1; ++c) {
            out.push_back(static_cast<int>(r * grid.cols + c));
        }
    }
    return out;
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

        hardware::COBCoord bbox_start {};
        hardware::COBCoord bbox_end {};
        if (record.start_bumps.empty()) {
            continue;
        }
        c.src = node_from_bump_track(graph, c.cob_unit, record.start_bumps.front().TOB, endpoint.start_track);
        bbox_start = tob_anchor_cob(record.start_bumps.front().TOB);

        if (record.type == Net_type::PNnet) {
            if (record.power_kind == IlpPowerKind::Pose) {
                c.cls = McfClass::P;
                c.snk = graph.vp_node;
            }
            else {
                c.cls = McfClass::N;
                c.snk = graph.vn_node;
            }

            auto virtual_edges_added = 0;
            for (const auto& [end_track, start_tracks] : record.starttrack_by_endtrack) {
                if (!std::binary_search(start_tracks.begin(), start_tracks.end(), endpoint.start_track)) {
                    continue;
                }
                if (map_track(end_track) != c.cob_unit) {
                    continue;
                }
                if (!record.pn_end_track_coord_by_index.contains(end_track)) {
                    continue;
                }
                const auto& tc = record.pn_end_track_coord_by_index.at(end_track);
                const auto n_end = node_from_track_coord(graph, c.cob_unit, tc, end_track);
                if (n_end < 0) {
                    continue;
                }
                add_arc(
                    graph,
                    n_end,
                    c.snk,
                    true,
                    false,
                    c.cob_unit,
                    -1,
                    end_track,
                    end_track);
                add_arc(
                    graph,
                    c.snk,
                    n_end,
                    true,
                    false,
                    c.cob_unit,
                    -1,
                    end_track,
                    end_track);
                bbox_end = track_to_cob(tc);
                c.end_track = end_track;
                virtual_edges_added += 1;
            }
            if (virtual_edges_added == 0 && endpoint.has_end_track
                && record.pn_end_track_coord_by_index.contains(endpoint.end_track)) {
                const auto& tc = record.pn_end_track_coord_by_index.at(endpoint.end_track);
                const auto n_end = node_from_track_coord(graph, c.cob_unit, tc, endpoint.end_track);
                if (n_end >= 0) {
                    add_arc(graph, n_end, c.snk, true, false, c.cob_unit, -1, endpoint.end_track, endpoint.end_track);
                    add_arc(graph, c.snk, n_end, true, false, c.cob_unit, -1, endpoint.end_track, endpoint.end_track);
                    bbox_end = track_to_cob(tc);
                    c.end_track = endpoint.end_track;
                }
            }
        }
        else if (record.type == Net_type::Tnet) {
            c.cls = McfClass::Plain;
            if (!endpoint.has_end_track) {
                continue;
            }
            c.snk = node_from_track_coord(graph, c.cob_unit, record.mcf_end_track, endpoint.end_track);
            bbox_end = track_to_cob(record.mcf_end_track);
        }
        else {
            c.cls = McfClass::Plain;
            if (record.end_bumps.empty() || !endpoint.has_end_track) {
                continue;
            }
            c.snk = node_from_bump_track(graph, c.cob_unit, record.end_bumps.front().TOB, endpoint.end_track);
            bbox_end = tob_anchor_cob(record.end_bumps.front().TOB);
        }

        if (c.src < 0 || c.snk < 0) {
            debug::warning_fmt(
                "MCF prepare: unresolved endpoint node for record {} (src={}, snk={})",
                record.net_name,
                c.src,
                c.snk);
            continue;
        }

        c.bbox_cobs = bbox_cob_indices(grid, bbox_start, bbox_end);
        if (endpoint.has_end_track) {
            const auto it_end = record.reach_by_end_start.find(endpoint.end_track);
            if (it_end != record.reach_by_end_start.end()) {
                const auto it_start = it_end->second.find(endpoint.start_track);
                if (it_start != it_end->second.end()) {
                    c.reach_steps = it_start->second;
                }
            }
        }
        out.push_back(std::move(c));
    }
    return out;
}

auto arc_usable_for_class(
    const GlobalGraph& graph,
    const Arc& arc,
    const McfClass cls,
    const std::size_t unit,
    const int commodity_snk
) -> bool {
    if (arc.unit != unit) {
        return false;
    }
    if (arc.u == graph.vp_node || arc.v == graph.vp_node) {
        return cls == McfClass::P;
    }
    if (arc.u == graph.vn_node || arc.v == graph.vn_node) {
        return cls == McfClass::N;
    }
    return true;
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
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    const std::map<std::pair<int, int>, int>& used_edges,
    const std::map<int, int>& used_nodes
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
            if (!arc_usable_for_class(graph, arc, commodity.cls, commodity.cob_unit, commodity.snk)) {
                continue;
            }
            if (!arc.is_virtual) {
                const auto edge_key = normalized_edge_key(arc.u, arc.v);
                if (const auto it = used_edges.find(edge_key); it != used_edges.end() && it->second >= 1) {
                    continue;
                }
            }
            if (!graph.nodes[static_cast<std::size_t>(arc.v)].is_virtual) {
                if (const auto it = used_nodes.find(arc.v); it != used_nodes.end() && it->second >= 1) {
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

auto route_mcf_stage_warm_start(
    const std::String& stage_name,
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& commodity_ids,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    std::map<std::pair<int, int>, int>& used_edges,
    std::map<int, int>& used_nodes
) -> StageWarmStart {
    auto warm = StageWarmStart {};
    std::size_t routed = 0;
    std::size_t failed = 0;
    for (const auto cid : commodity_ids) {
        const auto& commodity = commodities[cid];
        auto path = route_one_mcf_warm_path(graph, commodity, outgoing_arcs, used_edges, used_nodes);
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

auto append_paths_from_f_solution(
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

auto solve_bus_mcf(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& bus_ids,
    const StageWarmStart* warm_start
) -> StageSolveResult {
    constexpr auto stage_name = "BusMCF";
    StageSolveResult out {};
    out.stage_name = stage_name;
    if (bus_ids.empty()) {
        out.ok = true;
        out.message = "empty stage";
        out.model_status = GRB_OPTIMAL;
        return out;
    }

    const auto K = static_cast<int>(bus_ids.size());
    const auto A = static_cast<int>(graph.arcs.size());
    const auto local_com = build_local_commodities(commodities, bus_ids);

    // BusMCF §1: f^{c,n}_{ij} variables
    auto f_vars = std::Vector<ArcVar> {};
    auto f_by_k = std::Vector<std::Vector<int>>(static_cast<std::size_t>(K));
    f_vars.reserve(static_cast<std::size_t>(K * A / 8 + 1));
    for (int k = 0; k < K; ++k) {
        for (int a = 0; a < A; ++a) {
            if (!arc_usable_for_class(
                    graph,
                    graph.arcs[static_cast<std::size_t>(a)],
                    local_com[static_cast<std::size_t>(k)].cls,
                    local_com[static_cast<std::size_t>(k)].cob_unit,
                    local_com[static_cast<std::size_t>(k)].snk)) {
                continue;
            }
            const auto var_id = static_cast<int>(f_vars.size());
            f_vars.push_back(ArcVar {k, a});
            f_by_k[static_cast<std::size_t>(k)].push_back(var_id);
        }
    }
    if (f_vars.empty()) {
        out.ok = false;
        out.message = std::format("{}: no feasible arc-variable pairs", stage_name);
        return out;
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
                    node_text(graph, n))});
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
                d)};
        row_meta[static_cast<std::size_t>(rt)] = McfConstraintMeta {
            "flow_conservation",
            std::format(
                "commodity={} node={} rhs=-{} (sink)",
                local_com[static_cast<std::size_t>(k)].label,
                node_text(graph, t),
                d)};
    }

    // BusMCF §5: f <= o, Σ_n o_i <= 1
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
        const auto row_link = add_le(
            0.0,
            McfConstraintMeta {
                "f_le_o_link",
                std::format(
                    "commodity={} node={}",
                    local_com[static_cast<std::size_t>(k)].label,
                    node_text(graph, n))});
        for (const auto j : vars) {
            f_entries[static_cast<std::size_t>(j)].push_back({row_link, 1.0});
        }
        o_vars.push_back(OVar {k, n});
        auto col = std::Vector<std::pair<int, double>> {};
        col.push_back({row_link, -2.0});
        col.push_back({node_row.at(n), 1.0});
        o_entries.push_back(std::move(col));
    }

    // BusMCF §6 (第五版): sync equal length
    // total_flow_n = Σ_{(i,j)∈E^c} f^{c,n}_{ij},  ∀n∈Bus ∧ c = n 所在 COBUnit
    // E^c 由 arc_usable_for_class(..., cob_unit) 限定；非虚拟弧求和即 total_flow_n
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
                        local_com[static_cast<std::size_t>(group[gi])].label)});
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

    const auto solve_res = solve_binary_columns_with_gurobi(
        stage_name,
        col_cost,
        col_lo,
        col_up,
        row_lo,
        row_up,
        col_entries,
        warm_values_by_col,
        &row_meta);
    out.model_status = solve_res.model_status;
    if (!solve_res.ok) {
        out.ok = false;
        out.message = solve_res.message;
        return out;
    }
    if (solve_res.model_status != GRB_OPTIMAL) {
        if (warm_start != nullptr) {
            debug::warning_fmt(
                "{} warm start led to non-optimal status ({}); retrying without warm start",
                stage_name,
                solve_res.model_status);
            return solve_bus_mcf(graph, commodities, bus_ids, nullptr);
        }
        out.ok = false;
        out.message = std::format("{}: model not optimal ({})", stage_name, solve_res.model_status);
        out.infeasibility_hints = solve_res.iis_rows;
        return out;
    }

    out.ok = true;
    out.message = "ok";
    out.objective = solve_res.objective;

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

    append_paths_from_f_solution(graph, local_com, f_vars, f_values, out);
    return out;
}

auto solve_simple_mcf_unit(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::map<int, int>& node_capacity_override,
    const bool enable_mcf_obj,
    const StageWarmStart* warm_start,
    const std::Vector<std::set<std::pair<int, int>>>* excluded_origin_x_sets = nullptr
) -> StageSolveResult {
    const auto stage_name = std::format("SimpleMCF_unit{}", unit_c);
    StageSolveResult out {};
    out.stage_name = stage_name;
    if (simple_ids_for_unit.empty()) {
        out.ok = true;
        out.message = "empty stage";
        out.model_status = GRB_OPTIMAL;
        return out;
    }

    const auto K = static_cast<int>(simple_ids_for_unit.size());
    const auto A = static_cast<int>(graph.arcs.size());
    const auto local_com = build_local_commodities(commodities, simple_ids_for_unit);

    auto origin_groups = build_origin_groups(local_com, records);
    log_origin_groups(stage_name, origin_groups);
    auto commodity_origin_h = std::Vector<int>(static_cast<std::size_t>(K), -1);
    for (const auto& group : origin_groups) {
        for (const auto k : group.commodity_local_indices) {
            commodity_origin_h[static_cast<std::size_t>(k)] = group.origin_group_id;
        }
    }

    // SimpleMCF §1: f^{c,n}_{ij} variables
    auto f_vars = std::Vector<ArcVar> {};
    auto f_by_k = std::Vector<std::Vector<int>>(static_cast<std::size_t>(K));
    f_vars.reserve(static_cast<std::size_t>(K * A / 8 + 1));
    for (int k = 0; k < K; ++k) {
        for (int a = 0; a < A; ++a) {
            if (!arc_usable_for_class(
                    graph,
                    graph.arcs[static_cast<std::size_t>(a)],
                    local_com[static_cast<std::size_t>(k)].cls,
                    local_com[static_cast<std::size_t>(k)].cob_unit,
                    local_com[static_cast<std::size_t>(k)].snk)) {
                continue;
            }
            const auto var_id = static_cast<int>(f_vars.size());
            f_vars.push_back(ArcVar {k, a});
            f_by_k[static_cast<std::size_t>(k)].push_back(var_id);
        }
    }
    if (f_vars.empty()) {
        out.ok = false;
        out.message = std::format("{}: no feasible arc-variable pairs", stage_name);
        return out;
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
    auto add_ge = [&](const double rhs, McfConstraintMeta meta) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(rhs);
        row_up.push_back(kGurobiInf);
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
            McfConstraintMeta {
                "flow_conservation",
                std::format(
                    "commodity={} node={} (unset)",
                    local_com[static_cast<std::size_t>(k)].label,
                    node_text(graph, n))});
        flow_row[key] = row;
        return row;
    };

    // SimpleMCF v5 §5: Σ_H x^H_e <= capacity^c_e - used^{Bus,c}_e on undirected physical edge e ∈ E^c
    auto edge_row = std::map<std::pair<int, int>, int> {};
    for (const auto& key : collect_undirected_physical_edge_keys(graph, unit_c)) {
        auto cap = 1;
        if (edge_capacity_override.contains(key)) {
            cap = edge_capacity_override.at(key);
        }
        edge_row[key] = add_le(
            static_cast<double>(cap),
            McfConstraintMeta {
                "edge_capacity",
                std::format(
                    "rhs={} bus_residual={} {}",
                    cap,
                    cap,
                    describe_undirected_edge(graph, arc_index, key.first, key.second, graph.cols))});
    }

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
                d)};
        row_meta[static_cast<std::size_t>(rt)] = McfConstraintMeta {
            "flow_conservation",
            std::format(
                "commodity={} node={} rhs=-{} (sink)",
                local_com[static_cast<std::size_t>(k)].label,
                node_text(graph, t),
                d)};
    }

    // SimpleMCF v5 §1-2: x^{c,H}_e on undirected physical edges e
    using UndirectedEdgeKey = std::pair<int, int>;
    using OriginEdgeKey = std::pair<int, UndirectedEdgeKey>;
    auto origin_x_vars = std::Vector<OriginEdgeVar> {};
    auto origin_x_entries = std::Vector<std::Vector<std::pair<int, double>>> {};
    auto origin_x_by_he = std::map<OriginEdgeKey, int> {};
    auto f_indices_by_he = std::map<OriginEdgeKey, std::Vector<int>> {};
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
            if (!edge_row.contains(e)) {
                continue;
            }
            const auto var_id = static_cast<int>(origin_x_vars.size());
            origin_x_vars.push_back(OriginEdgeVar {h, e.first, e.second});
            origin_x_by_he[he_key] = var_id;
            origin_x_entries.emplace_back();
            origin_x_entries[static_cast<std::size_t>(var_id)].push_back({edge_row.at(e), 1.0});
        }
    }

    // v5 §2 lower: f^{n}_{ij}, f^{n}_{ji} <= x^H_e
    int f_le_x_lower_rows = 0;
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto k = f_vars[j].k;
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[j].a)];
        if (arc.is_virtual) {
            continue;
        }
        const auto e = normalized_edge_key(arc.u, arc.v);
        const auto it = origin_x_by_he.find({h, e});
        if (it == origin_x_by_he.end()) {
            continue;
        }
        const auto row = add_le(
            0.0,
            McfConstraintMeta {
                "f_le_x_lower",
                std::format(
                    "origin={} commodity={} {}",
                    origin_label(h),
                    local_com[static_cast<std::size_t>(k)].label,
                    describe_undirected_edge(graph, arc_index, e.first, e.second, graph.cols))});
        ++f_le_x_lower_rows;
        f_entries[j].push_back({row, 1.0});
        origin_x_entries[static_cast<std::size_t>(it->second)].push_back({row, -1.0});
        f_indices_by_he[{h, e}].push_back(static_cast<int>(j));
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
            McfConstraintMeta {
                "f_le_x_upper",
                std::format(
                    "origin={} {}",
                    origin_label(he_key.first),
                    describe_undirected_edge(
                        graph,
                        arc_index,
                        he_key.second.first,
                        he_key.second.second,
                        graph.cols))});
        ++f_le_x_upper_rows;
        origin_x_entries[static_cast<std::size_t>(x_var)].push_back({row, 1.0});
        for (const auto f_j : f_list) {
            f_entries[static_cast<std::size_t>(f_j)].push_back({row, -1.0});
        }
    }

    // v5 §5: x_e <= o_i, o_i <= Σ_{e∈δ(i)} x_e, Σ_H o^H_i <= 1 - used^{Bus,c}_i
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
    int o_le_sum_x_rows = 0;
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
                McfConstraintMeta {
                    "x_le_o",
                    std::format(
                        "origin={} node={} edge={}-{}",
                        origin_label(h),
                        node_text(graph, n),
                        node_text(graph, e.first),
                        node_text(graph, e.second))});
            ++x_le_o_rows;
            origin_x_entries[static_cast<std::size_t>(x_var)].push_back({row_x_le_o, 1.0});
            origin_o_entries[static_cast<std::size_t>(o_var)].push_back({row_x_le_o, -1.0});
        }
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
        const auto row_o_le_sum = add_le(
            0.0,
            McfConstraintMeta {
                "o_le_sum_x",
                std::format("origin={} node={}", origin_label(h), node_text(graph, n))});
        ++o_le_sum_x_rows;
        origin_o_entries[static_cast<std::size_t>(o_var)].push_back({row_o_le_sum, 1.0});
        for (const auto x_idx : x_on_delta) {
            origin_x_entries[static_cast<std::size_t>(x_idx)].push_back({row_o_le_sum, -1.0});
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
            {"o_le_sum_x", o_le_sum_x_rows},
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
    auto col_cost = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_lo = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_up = std::vector<double>(static_cast<std::size_t>(num_col), 1.0);
    auto col_entries = std::Vector<std::Vector<std::pair<int, double>>>(static_cast<std::size_t>(num_col));

    for (int j = 0; j < num_f; ++j) {
        col_entries[static_cast<std::size_t>(j)] = f_entries[static_cast<std::size_t>(j)];
    }
    for (int j = 0; j < num_origin_x; ++j) {
        const auto col = num_f + j;
        col_cost[static_cast<std::size_t>(col)] = enable_mcf_obj ? 1.0 : 0.0;
        col_entries[static_cast<std::size_t>(col)] = origin_x_entries[static_cast<std::size_t>(j)];
    }
    for (int j = 0; j < num_origin_o; ++j) {
        const auto col = num_f + num_origin_x + j;
        col_entries[static_cast<std::size_t>(col)] = origin_o_entries[static_cast<std::size_t>(j)];
    }

    auto warm_values_by_col = std::map<int, double> {};
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

    if (excluded_origin_x_sets != nullptr && !excluded_origin_x_sets->empty()) {
        for (std::size_t si = 0; si < excluded_origin_x_sets->size(); ++si) {
            const auto& used = excluded_origin_x_sets->at(si);
            const auto row = add_ge(
                1.0 - static_cast<double>(used.size()),
                McfConstraintMeta {
                    "nogood_origin_x",
                    std::format("excluded_solution={} used_edges={}", si, used.size())});
            for (const auto& xv : origin_x_vars) {
                const auto e = normalized_edge_key(xv.u, xv.v);
                const auto ox_it = origin_x_by_he.find({xv.h, e});
                if (ox_it == origin_x_by_he.end()) {
                    continue;
                }
                const auto col = static_cast<std::size_t>(num_f + ox_it->second);
                const double coeff = used.contains(e) ? -1.0 : 1.0;
                col_entries[col].push_back({row, coeff});
            }
        }
    }

    const auto solve_res = solve_binary_columns_with_gurobi(
        stage_name,
        col_cost,
        col_lo,
        col_up,
        row_lo,
        row_up,
        col_entries,
        warm_values_by_col,
        &row_meta);
    out.model_status = solve_res.model_status;
    if (!solve_res.ok) {
        out.ok = false;
        out.message = solve_res.message;
        return out;
    }
    if (solve_res.model_status != GRB_OPTIMAL) {
        if (warm_start != nullptr && excluded_origin_x_sets == nullptr) {
            debug::warning_fmt(
                "{} warm start led to non-optimal status ({}); retrying without warm start",
                stage_name,
                solve_res.model_status);
            return solve_simple_mcf_unit(
                graph,
                commodities,
                unit_c,
                simple_ids_for_unit,
                records,
                edge_capacity_override,
                node_capacity_override,
                enable_mcf_obj,
                nullptr,
                excluded_origin_x_sets);
        }
        out.ok = false;
        out.message = std::format("{}: model not optimal ({})", stage_name, solve_res.model_status);
        out.infeasibility_hints = solve_res.iis_rows;
        return out;
    }

    out.ok = true;
    out.message = "ok";
    out.objective = solve_res.objective;

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

    append_paths_from_f_solution(graph, local_com, f_vars, f_values, out);
    return out;
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

auto log_mcf_paths_by_origin_net(
    const GlobalGraph& graph,
    const std::array<std::Vector<McfPathInfo>, 16>& paths_by_unit,
    const std::Vector<Net_cost_record>& records
) -> void {
    struct PathRef {
        const McfPathInfo* info;
        std::size_t cob_unit;
    };
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
        return;
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
    for (const auto& key : ordered_keys) {
        const auto& group = group_map.at(key);
        if (group.is_bus) {
            debug::info_fmt(
                "  [BusMCF] origin=\"{}\" commodities={}",
                group.display_name,
                group.paths.size());
        }
        else {
            debug::info_fmt(
                "  [SimpleMCF] COBUnit={} group_key=\"{}\" display=\"{}\" multi_fanout={} commodities={}",
                group.cob_unit,
                group.group_key,
                group.display_name,
                group.is_multi_fanout,
                group.paths.size());
        }
        for (const auto& pr : group.paths) {
            const auto& info = *pr.info;
            std::size_t bit_id = 0;
            auto rec_name = std::String("(record_id out of range)");
            if (info.record_id < records.size()) {
                bit_id = records[info.record_id].bit_id;
                rec_name = records[info.record_id].net_name;
            }
            debug::info_fmt(
                "    commodity={} record=\"{}\" record_id={} bit={} start_track={} end_track={} path_count={}",
                info.label,
                rec_name,
                info.record_id,
                bit_id,
                info.start_track,
                info.end_track,
                info.unit_paths.size());
            for (std::size_t pi = 0; pi < info.unit_paths.size(); ++pi) {
                debug::info_fmt("      path#{} {}", pi, path_to_text(graph, info.unit_paths[pi]));
            }
        }
    }
}

auto count_physical_edges_on_path(const GlobalGraph& graph, const std::Vector<int>& path) -> int {
    if (path.size() < 2) {
        return 0;
    }
    auto keys = std::set<std::pair<int, int>> {};
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const auto u = path[i];
        const auto v = path[i + 1];
        if (graph.nodes[static_cast<std::size_t>(u)].is_virtual
            || graph.nodes[static_cast<std::size_t>(v)].is_virtual) {
            continue;
        }
        keys.insert(normalized_edge_key(u, v));
    }
    return static_cast<int>(keys.size());
}

auto build_outgoing_arcs_list(const GlobalGraph& graph) -> std::Vector<std::Vector<int>> {
    auto outgoing = std::Vector<std::Vector<int>>(graph.nodes.size());
    for (std::size_t a = 0; a < graph.arcs.size(); ++a) {
        outgoing[static_cast<std::size_t>(graph.arcs[a].u)].push_back(static_cast<int>(a));
    }
    return outgoing;
}

auto shortest_path_with_bans(
    const GlobalGraph& graph,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    const int src,
    const int snk,
    const McfClass cls,
    const std::size_t cob_unit,
    const std::set<std::pair<int, int>>& banned_edges,
    const std::set<int>& banned_nodes
) -> std::Vector<int> {
    auto prev_node = std::vector<int>(graph.nodes.size(), -1);
    auto q = std::queue<int> {};
    q.push(src);
    prev_node[static_cast<std::size_t>(src)] = src;

    while (!q.empty()) {
        const auto node = q.front();
        q.pop();
        if (node == snk) {
            break;
        }
        for (const auto arc_id : outgoing_arcs[static_cast<std::size_t>(node)]) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!arc_usable_for_class(graph, arc, cls, cob_unit, snk)) {
                continue;
            }
            if (!arc.is_virtual) {
                const auto edge_key = normalized_edge_key(arc.u, arc.v);
                if (banned_edges.contains(edge_key)) {
                    continue;
                }
            }
            if (!graph.nodes[static_cast<std::size_t>(arc.v)].is_virtual) {
                if (banned_nodes.contains(arc.v)) {
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

    if (prev_node[static_cast<std::size_t>(snk)] == -1) {
        return {};
    }
    auto path = std::Vector<int> {};
    auto cur = snk;
    while (cur != src) {
        path.push_back(cur);
        cur = prev_node[static_cast<std::size_t>(cur)];
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    return path;
}

auto make_wirelength_path_rank(
    const GlobalGraph& graph,
    const int rank,
    const std::Vector<int>& path
) -> WirelengthPathRank {
    WirelengthPathRank out {};
    out.rank = rank;
    out.node_path = path;
    out.physical_edges = count_physical_edges_on_path(graph, path);
    out.arc_count = path.size() >= 2 ? static_cast<int>(path.size()) - 1 : 0;
    out.path_text = path_to_text(graph, path);
    return out;
}

auto yen_k_shortest_paths(
    const GlobalGraph& graph,
    const int src,
    const int snk,
    const McfClass cls,
    const std::size_t cob_unit,
    const int k
) -> std::Vector<std::Vector<int>> {
    if (k <= 0) {
        return {};
    }
    const auto outgoing = build_outgoing_arcs_list(graph);
    auto paths = std::Vector<std::Vector<int>> {};
    const auto first = shortest_path_with_bans(
        graph, outgoing, src, snk, cls, cob_unit, {}, {});
    if (first.empty()) {
        return paths;
    }
    paths.push_back(first);

    struct Candidate {
        std::Vector<int> path;
        int length{0};
    };
    auto cmp = [](const Candidate& a, const Candidate& b) {
        return a.length > b.length;
    };

    for (int rank = 2; rank <= k; ++rank) {
        auto candidates = std::Vector<Candidate> {};
        for (const auto& base_path : paths) {
            for (std::size_t spur_idx = 0; spur_idx + 1 < base_path.size(); ++spur_idx) {
                const auto root = std::Vector<int>(base_path.begin(), base_path.begin() + static_cast<std::ptrdiff_t>(spur_idx + 1));
                const auto spur_node = base_path[spur_idx];

                auto banned_nodes = std::set<int> {};
                for (std::size_t i = 0; i < spur_idx; ++i) {
                    banned_nodes.insert(base_path[i]);
                }

                auto banned_edges = std::set<std::pair<int, int>> {};
                for (const auto& prev_path : paths) {
                    if (prev_path.size() <= spur_idx) {
                        continue;
                    }
                    bool same_prefix = true;
                    for (std::size_t i = 0; i <= spur_idx; ++i) {
                        if (prev_path[i] != root[i]) {
                            same_prefix = false;
                            break;
                        }
                    }
                    if (!same_prefix) {
                        continue;
                    }
                    if (spur_idx + 1 < prev_path.size()) {
                        banned_edges.insert(normalized_edge_key(prev_path[spur_idx], prev_path[spur_idx + 1]));
                    }
                }

                const auto spur_path = shortest_path_with_bans(
                    graph,
                    outgoing,
                    spur_node,
                    snk,
                    cls,
                    cob_unit,
                    banned_edges,
                    banned_nodes);
                if (spur_path.empty()) {
                    continue;
                }
                auto total = root;
                if (spur_path.size() > 1) {
                    total.insert(total.end(), spur_path.begin() + 1, spur_path.end());
                }
                candidates.push_back(
                    Candidate {total, count_physical_edges_on_path(graph, total)});
            }
        }
        if (candidates.empty()) {
            break;
        }
        std::sort(candidates.begin(), candidates.end(), cmp);
        const auto& best = candidates.front().path;
        bool duplicate = false;
        for (const auto& existing : paths) {
            if (existing == best) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            break;
        }
        paths.push_back(best);
    }
    return paths;
}

auto used_origin_x_set_from_stage(const StageSolveResult& stage) -> std::set<std::pair<int, int>> {
    auto used = std::set<std::pair<int, int>> {};
    for (const auto& [key, cnt] : stage.used_edges) {
        if (cnt > 0) {
            used.insert(normalized_edge_key(key.first, key.second));
        }
    }
    return used;
}

auto ttb_solution_from_stage(
    const GlobalGraph& graph,
    const StageSolveResult& stage,
    const int rank
) -> WirelengthTtbSolution {
    WirelengthTtbSolution out {};
    out.rank = rank;
    out.total_physical_edges = stage.objective;
    out.per_commodity_paths = stage.paths;
    debug::info_fmt(
        "wirelength ttb rank={} total_physical_edges={:.0f} commodities={}",
        rank,
        out.total_physical_edges,
        out.per_commodity_paths.size());
    for (const auto& info : out.per_commodity_paths) {
        if (!info.unit_paths.empty()) {
            debug::info_fmt(
                "  commodity {} physical_edges={} path={}",
                info.label,
                count_physical_edges_on_path(graph, info.unit_paths.front()),
                path_to_text(graph, info.unit_paths.front()));
        }
    }
    return out;
}

} // namespace

auto run_mcf_global_routing_cob_units(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    hardware::Interposer* interposer,
    const circuit::BaseDie& basedie,
    const CobMcfGridDims cob_grid,
    const bool enable_mcf_parallel,
    const bool enable_pre_routing,
    const bool enable_mcf_obj,
    const bool defer_interposer_suspend,
    const bool disable_bus_mcf
) -> CobMcfFullResult {
    (void)basedie;

    const auto mcf_start = std::chrono::steady_clock::now();
    const auto peak_before = get_peak_rss_mb();
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
                std::String("record/ilp result size mismatch")};
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

    auto bus_warm_start = StageWarmStart {};
    auto simple_warm_start = StageWarmStart {};
    const StageWarmStart* bus_warm_start_ptr = nullptr;
    const StageWarmStart* simple_warm_start_ptr = nullptr;
    const auto mcf_warm_t0 = std::chrono::steady_clock::now();
    if (enable_pre_routing) {
        auto outgoing_arcs = std::Vector<std::Vector<int>>(graph.nodes.size());
        for (std::size_t a = 0; a < graph.arcs.size(); ++a) {
            outgoing_arcs[static_cast<std::size_t>(graph.arcs[a].u)].push_back(static_cast<int>(a));
        }
        auto warm_used_edges = std::map<std::pair<int, int>, int> {};
        auto warm_used_nodes = std::map<int, int> {};
        if (!disable_bus_mcf) {
            bus_warm_start = route_mcf_stage_warm_start(
                "BusMCF",
                graph,
                commodities,
                bus_ids,
                outgoing_arcs,
                warm_used_edges,
                warm_used_nodes);
            bus_warm_start_ptr = &bus_warm_start;
        }
        simple_warm_start = route_mcf_stage_warm_start(
            "SimpleMCF",
            graph,
            commodities,
            simple_ids,
            outgoing_arcs,
            warm_used_edges,
            warm_used_nodes);
        simple_warm_start_ptr = &simple_warm_start;
    }
    const auto mcf_warm_t1 = std::chrono::steady_clock::now();
    out.summary.mcf_warm_start_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(mcf_warm_t1 - mcf_warm_t0).count());
    debug::info_fmt("timing phase=mcf_warm_start ms={}", out.summary.mcf_warm_start_ms);

    const auto solve_t0 = std::chrono::steady_clock::now();
    auto bus_res = StageSolveResult {};
    if (disable_bus_mcf) {
        bus_res.ok = true;
        bus_res.stage_name = "BusMCF";
        bus_res.message = "skipped (--disable-bus-mcf)";
        bus_res.model_status = GRB_OPTIMAL;
        if (!bus_ids.empty()) {
            debug::info_fmt(
                "MCF: BusMCF skipped; {} bus commodities are not routed",
                bus_ids.size());
        }
    }
    else {
        bus_res = solve_bus_mcf(graph, commodities, bus_ids, bus_warm_start_ptr);
    }

    // 得到剩余容量
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

    // solve
    auto simple_results = std::array<StageSolveResult, 16> {};
    auto simple_futures = std::array<std::future<StageSolveResult>, 16> {};
    auto has_simple_unit = std::array<bool, 16> {};
    has_simple_unit.fill(false);

    for (std::size_t u = 0; u < 16; ++u) {
        if (simple_ids_by_unit[u].empty()) {
            simple_results[u].ok = true;
            simple_results[u].message = "empty stage";
            continue;
        }
        has_simple_unit[u] = true;
        const auto edge_cap = build_edge_residual(u);
        const auto node_cap = build_node_residual(u);
        if (enable_mcf_parallel) {
            simple_futures[u] = std::async(
                std::launch::async,
                [&graph, &commodities, &records, &simple_ids_by_unit, u, edge_cap, node_cap, enable_mcf_obj, simple_warm_start_ptr]() {
                    return solve_simple_mcf_unit(
                        graph,
                        commodities,
                        u,
                        simple_ids_by_unit[u],
                        records,
                        edge_cap,
                        node_cap,
                        enable_mcf_obj,
                        simple_warm_start_ptr);
                });
        }
        else {
            simple_results[u] = solve_simple_mcf_unit(
                graph,
                commodities,
                u,
                simple_ids_by_unit[u],
                records,
                edge_cap,
                node_cap,
                enable_mcf_obj,
                simple_warm_start_ptr);
            debug::info_fmt(
                "SimpleMCF unit {}: ok={} objective={:.0f} paths={}",
                u,
                simple_results[u].ok,
                simple_results[u].objective,
                simple_results[u].paths.size());
        }
    }

    // 统计结果
    if (enable_mcf_parallel) {
        for (std::size_t u = 0; u < 16; ++u) {
            if (!has_simple_unit[u]) {
                continue;
            }
            simple_results[u] = simple_futures[u].get();
            debug::info_fmt(
                "SimpleMCF unit {}: ok={} objective={:.0f} paths={}",
                u,
                simple_results[u].ok,
                simple_results[u].objective,
                simple_results[u].paths.size());
        }
    }

    const auto solve_t1 = std::chrono::steady_clock::now();
    const auto solve_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(solve_t1 - solve_t0).count());
    out.summary.mcf_solve_ms = solve_ms;
    debug::info_fmt("timing phase=mcf_solve ms={}", solve_ms);

    if (!bus_res.ok) {
        debug::error_fmt("BusMCF failed: {}", bus_res.message);
    }
    bool all_simple_ok = true;
    double simple_objective = 0.0;
    for (std::size_t u = 0; u < 16; ++u) {
        out.has_simple_commodities[u] = has_simple_unit[u];
        if (!has_simple_unit[u]) {
            out.simple_mcf_ok[u] = true;
            continue;
        }
        out.simple_mcf_ok[u] = simple_results[u].ok;
        if (!simple_results[u].ok) {
            all_simple_ok = false;
            debug::error_fmt("SimpleMCF unit {} failed: {}", u, simple_results[u].message);
        }
        simple_objective += simple_results[u].objective;
    }
    out.summary.all_ok = bus_res.ok && all_simple_ok;

    for (std::size_t u = 0; u < 16; ++u) {
        const auto obj = bus_res.objective + simple_objective;
        out.summary.per_cob[u] = CobMcfCobUnitSummary {
            u,
            bus_count_by_unit[u] + simple_count_by_unit[u],
            out.summary.all_ok,
            obj,
            solve_ms,
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
    log_mcf_paths_by_origin_net(graph, out.paths_by_unit, records);

    const auto resource_catalog = build_mcf_resource_catalog(graph);
    const auto arc_index = build_undirected_arc_index(graph);
    const auto resource_usage = aggregate_mcf_resource_usage(graph, arc_index, bus_res, simple_results);
    log_mcf_resource_usage(resource_catalog, resource_usage, out.summary.all_ok);
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

auto run_wirelength_study_2pin(
    const McfGlobalGraph& graph,
    const int src,
    const int snk,
    const int mcf_class_plain,
    const std::size_t cob_unit,
    const int k,
    const std::Vector<int>* mcf_rank1_path
) -> WirelengthStudy2PinResult {
    const auto cls = static_cast<McfClass>(mcf_class_plain);
    WirelengthStudy2PinResult out {};
    const auto paths = yen_k_shortest_paths(graph, src, snk, cls, cob_unit, k);
    if (paths.empty()) {
        return out;
    }
    out.k_shortest.reserve(paths.size());
    for (std::size_t i = 0; i < paths.size(); ++i) {
        out.k_shortest.push_back(make_wirelength_path_rank(graph, static_cast<int>(i) + 1, paths[i]));
    }
    if (mcf_rank1_path != nullptr && !mcf_rank1_path->empty()) {
        out.mcf_rank1 = make_wirelength_path_rank(graph, 1, *mcf_rank1_path);
    }
    else {
        out.mcf_rank1 = out.k_shortest.front();
    }
    out.rank1_mcf_matches_yen = !out.k_shortest.empty()
        && out.mcf_rank1.physical_edges == out.k_shortest.front().physical_edges;
    return out;
}

auto run_wirelength_study_ttb_origin(
    McfGlobalGraph& graph,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfGridDims& cob_grid,
    const std::String& origin_key,
    const int k
) -> WirelengthStudyTtbResult {
    (void)cob_grid;
    WirelengthStudyTtbResult out {};
    if (k <= 0) {
        return out;
    }

    auto commodities = prepare_commodities(records, ilp_result, cob_grid, graph);
    auto simple_ids = std::Vector<std::size_t> {};
    std::size_t unit_c = 0;
    bool found = false;
    for (std::size_t i = 0; i < commodities.size(); ++i) {
        if (commodities[i].is_bus) {
            continue;
        }
        const auto& rec = records[commodities[i].record_index];
        if (!rec.from_track_to_bumps_split) {
            continue;
        }
        const auto key = rec.origin_key.empty() ? rec.net_name : rec.origin_key;
        if (key != origin_key) {
            continue;
        }
        if (!found) {
            unit_c = commodities[i].cob_unit;
            found = true;
        }
        else if (commodities[i].cob_unit != unit_c) {
            throw std::runtime_error(std::format(
                "wirelength ttb origin '{}' spans multiple COBUnits",
                origin_key));
        }
        simple_ids.push_back(i);
    }
    if (simple_ids.empty()) {
        throw std::runtime_error(std::format("wirelength ttb origin '{}' has no commodities", origin_key));
    }

    const auto empty_cap_edges = std::map<std::pair<int, int>, int> {};
    const auto empty_cap_nodes = std::map<int, int> {};
    auto excluded = std::Vector<std::set<std::pair<int, int>>> {};

    for (int rank = 1; rank <= k; ++rank) {
        const auto* excluded_ptr = excluded.empty() ? nullptr : &excluded;
        auto stage = solve_simple_mcf_unit(
            graph,
            commodities,
            unit_c,
            simple_ids,
            records,
            empty_cap_edges,
            empty_cap_nodes,
            true,
            nullptr,
            excluded_ptr);
        if (!stage.ok) {
            debug::info_fmt("wirelength ttb: rank={} infeasible ({})", rank, stage.message);
            break;
        }
        const auto solution = ttb_solution_from_stage(graph, stage, rank);
        if (rank == 1) {
            out.mcf_rank1 = solution;
        }
        else {
            out.alternates.push_back(solution);
        }
        excluded.push_back(used_origin_x_set_from_stage(stage));
    }
    return out;
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
