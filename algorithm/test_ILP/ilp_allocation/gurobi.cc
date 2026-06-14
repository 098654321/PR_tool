#include "ilp_allocation/gurobi.hh"
#include "ilp_allocation/gurobi_model_stats.hh"
#include "ilp_allocation/tob_ilp_model.hh"

#include "gurobi_c++.h"

#include <algorithm>
#include <format>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <vector>
#include <debug/debug.hh>


namespace PR_tool {

auto tob_ilp_record_type_name(const Net_type type) -> std::String {
    switch (type) {
        case Net_type::Bnet:
            return "Bnet";
        case Net_type::Tnet:
            return "Tnet";
        case Net_type::PNnet:
            return "PNnet";
    }
    return "Unknown";
}

auto tob_ilp_origin_key(const Net_cost_record& record) -> std::String {
    return record.origin_key.empty() ? record.net_name : record.origin_key;
}

auto tob_ilp_record_brief(const Net_cost_record& record) -> std::String {
    return std::format(
        "record_id={} bit_id={} net=\"{}\" origin=\"{}\" type={}",
        record.record_id,
        record.bit_id,
        record.net_name,
        tob_ilp_origin_key(record),
        tob_ilp_record_type_name(record.type));
}

auto tob_ilp_bump_text(
    const std::size_t tob,
    const std::size_t bank,
    const std::size_t group,
    const std::size_t index
) -> std::String {
    return std::format("bump(T{},B{},G{},I{})", tob, bank, group, index);
}

auto tob_ilp_relation_bumps_for(const Net_cost_record& record) -> std::Vector<Bump_coord> {
    auto relation_bumps = std::Vector<Bump_coord> {};
    if (record.type == Net_type::Bnet) {
        relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
        relation_bumps.insert(relation_bumps.end(), record.end_bumps.begin(), record.end_bumps.end());
    }
    else {
        relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
    }
    std::sort(relation_bumps.begin(), relation_bumps.end());
    relation_bumps.erase(std::unique(relation_bumps.begin(), relation_bumps.end()), relation_bumps.end());
    return relation_bumps;
}

auto split_tob_ilp_row_name(const std::String& name) -> std::Vector<std::String> {
    auto parts = std::Vector<std::String> {};
    std::size_t begin = 0;
    while (begin <= name.size()) {
        const auto end = name.find('_', begin);
        if (end == std::String::npos) {
            parts.push_back(name.substr(begin));
            break;
        }
        parts.push_back(name.substr(begin, end - begin));
        begin = end + 1;
    }
    return parts;
}

auto parse_size_part(const std::Vector<std::String>& parts, const std::size_t index) -> std::optional<std::size_t> {
    if (index >= parts.size()) {
        return std::nullopt;
    }
    try {
        return static_cast<std::size_t>(std::stoull(parts[index]));
    }
    catch (const std::exception&) {
        return std::nullopt;
    }
}

auto make_tob_ilp_meta(
    std::String kind,
    std::String detail,
    const std::set<std::size_t>& record_indexes,
    const std::Vector<Net_cost_record>& records
) -> TobIlpConstraintMeta {
    auto meta = TobIlpConstraintMeta {};
    meta.kind = std::move(kind);
    meta.detail = std::move(detail);
    auto origin_seen = std::set<std::String> {};
    for (const auto idx : record_indexes) {
        if (idx >= records.size()) {
            continue;
        }
        meta.related_record_ids.push_back(records[idx].record_id);
        const auto origin = tob_ilp_origin_key(records[idx]);
        if (origin_seen.insert(origin).second) {
            meta.related_origin_keys.push_back(origin);
        }
    }
    return meta;
}

auto make_single_record_meta(
    std::String kind,
    std::String detail,
    const std::size_t record_index,
    const std::Vector<Net_cost_record>& records
) -> TobIlpConstraintMeta {
    auto indexes = std::set<std::size_t> {};
    indexes.insert(record_index);
    return make_tob_ilp_meta(std::move(kind), std::move(detail), indexes, records);
}

auto build_tob_ilp_row_meta(
    const std::Vector<TobIlpLinearRow>& rows,
    const std::Vector<Net_cost_record>& records
) -> std::Vector<TobIlpConstraintMeta> {
    auto records_by_bump = std::map<Bump_coord, std::set<std::size_t>> {};
    auto records_by_tbg = std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::set<std::size_t>> {};
    auto records_by_tb = std::map<std::tuple<std::size_t, std::size_t>, std::set<std::size_t>> {};
    for (std::size_t n = 0; n < records.size(); ++n) {
        for (const auto& bump : tob_ilp_relation_bumps_for(records[n])) {
            records_by_bump[bump].insert(n);
            records_by_tbg[{bump.TOB, bump.Bank, bump.Group}].insert(n);
            records_by_tb[{bump.TOB, bump.Bank}].insert(n);
        }
    }

    auto metas = std::Vector<TobIlpConstraintMeta> {};
    metas.reserve(rows.size());
    for (const auto& row : rows) {
        const auto parts = split_tob_ilp_row_name(row.name);
        if (parts.size() < 2 || parts[0] != "R") {
            metas.push_back(make_tob_ilp_meta("unknown", std::format("row={}", row.name), {}, records));
            continue;
        }
        const auto& tag = parts[1];
        if (tag == "WONE") {
            const auto t = parse_size_part(parts, 2);
            const auto b = parse_size_part(parts, 3);
            const auto g = parse_size_part(parts, 4);
            const auto i = parse_size_part(parts, 5);
            if (t && b && g && i) {
                const auto bump = Bump_coord {*t, *b, *g, *i};
                const auto it = records_by_bump.find(bump);
                metas.push_back(make_tob_ilp_meta(
                    "bump_assignment",
                    std::format("{} must select exactly one (j,k)", tob_ilp_bump_text(*t, *b, *g, *i)),
                    it == records_by_bump.end() ? std::set<std::size_t> {} : it->second,
                    records));
                continue;
            }
        }
        if (tag == "HORI") {
            const auto t = parse_size_part(parts, 2);
            const auto b = parse_size_part(parts, 3);
            const auto g = parse_size_part(parts, 4);
            const auto j = parse_size_part(parts, 5);
            if (t && b && g && j) {
                const auto key = std::tuple<std::size_t, std::size_t, std::size_t> {*t, *b, *g};
                const auto it = records_by_tbg.find(key);
                metas.push_back(make_tob_ilp_meta(
                    "tob_hori_capacity",
                    std::format("TOB={} bank={} group={} horizontal_line_j={} capacity<=1", *t, *b, *g, *j),
                    it == records_by_tbg.end() ? std::set<std::size_t> {} : it->second,
                    records));
                continue;
            }
        }
        if (tag == "VERT") {
            const auto t = parse_size_part(parts, 2);
            const auto b = parse_size_part(parts, 3);
            const auto j = parse_size_part(parts, 4);
            const auto k = parse_size_part(parts, 5);
            if (t && b && j && k) {
                const auto key = std::tuple<std::size_t, std::size_t> {*t, *b};
                const auto it = records_by_tb.find(key);
                metas.push_back(make_tob_ilp_meta(
                    "tob_vert_capacity",
                    std::format("TOB={} bank={} vertical_slot(j={},k={}) capacity<=1", *t, *b, *j, *k),
                    it == records_by_tb.end() ? std::set<std::size_t> {} : it->second,
                    records));
                continue;
            }
        }
        if (tag.starts_with("QS") || tag.starts_with("QW")) {
            const auto t = parse_size_part(parts, 2);
            const auto b = parse_size_part(parts, 3);
            const auto g = parse_size_part(parts, 4);
            const auto i = parse_size_part(parts, 5);
            const auto j = parse_size_part(parts, 6);
            const auto k = parse_size_part(parts, 7);
            if (t && b && g && i && j && k) {
                const auto bump = Bump_coord {*t, *b, *g, *i};
                const auto it = records_by_bump.find(bump);
                metas.push_back(make_tob_ilp_meta(
                    "mux_linearization",
                    std::format("{} {} linearization at j={} k={}", tob_ilp_bump_text(*t, *b, *g, *i), tag, *j, *k),
                    it == records_by_bump.end() ? std::set<std::size_t> {} : it->second,
                    records));
                continue;
            }
        }
        if (tag == "BEND0") {
            const auto n = parse_size_part(parts, 2);
            const auto r = parse_size_part(parts, 3);
            if (n && r && *n < records.size()) {
                metas.push_back(make_single_record_meta(
                    "reachability",
                    std::format("{}: forbidden Bnet end_track={}", tob_ilp_record_brief(records[*n]), *r),
                    *n,
                    records));
                continue;
            }
        }
        if (tag == "BREACH") {
            const auto n = parse_size_part(parts, 2);
            const auto r_end = parse_size_part(parts, 3);
            const auto r_start = parse_size_part(parts, 4);
            if (n && r_end && r_start && *n < records.size()) {
                metas.push_back(make_single_record_meta(
                    "reachability",
                    std::format(
                        "{}: Bnet unreachable pair end_track={} start_track={}",
                        tob_ilp_record_brief(records[*n]),
                        *r_end,
                        *r_start),
                    *n,
                    records));
                continue;
            }
        }
        if (tag == "TREACH0") {
            const auto n = parse_size_part(parts, 2);
            const auto r = parse_size_part(parts, 3);
            if (n && r && *n < records.size()) {
                metas.push_back(make_single_record_meta(
                    "reachability",
                    std::format("{}: Tnet forbidden start_track={}", tob_ilp_record_brief(records[*n]), *r),
                    *n,
                    records));
                continue;
            }
        }
        if (tag == "PNYSUM") {
            const auto n = parse_size_part(parts, 2);
            if (n && *n < records.size()) {
                metas.push_back(make_single_record_meta(
                    "pn_selection",
                    std::format("{}: PNnet must select exactly one end_track", tob_ilp_record_brief(records[*n])),
                    *n,
                    records));
                continue;
            }
        }
        if (tag == "PNREACH") {
            const auto n = parse_size_part(parts, 2);
            const auto r_end = parse_size_part(parts, 3);
            const auto r_start = parse_size_part(parts, 4);
            if (n && r_end && r_start && *n < records.size()) {
                metas.push_back(make_single_record_meta(
                    "reachability",
                    std::format(
                        "{}: PNnet unreachable pair end_track={} start_track={}",
                        tob_ilp_record_brief(records[*n]),
                        *r_end,
                        *r_start),
                    *n,
                    records));
                continue;
            }
        }
        metas.push_back(make_tob_ilp_meta("unknown", std::format("row={}", row.name), {}, records));
    }
    return metas;
}

auto solve_tob_ilp_with_gurobi(
    const std::Vector<Net_cost_record>& records,
    const bool enable_parallel,
    const GurobiDiagnosticsOptions& diag
)
    -> TobIlpResult {
    
    // build model
    TobIlpResult out {};
    TobIlpModel model {};
    build_tob_ilp_model(model, records);
    const auto data = model.linear_data();
    const auto row_meta = build_tob_ilp_row_meta(data.rows, records);
    auto active_bumps = std::set<Bump_coord> {};
    for (const auto& record : records) {
        const auto relation_bumps = tob_ilp_relation_bumps_for(record);
        active_bumps.insert(relation_bumps.begin(), relation_bumps.end());
    }
    std::size_t binary_cols = 0;
    std::size_t nnz = 0;
    for (const auto& col : data.columns) {
        if (col.binary) {
            ++binary_cols;
        }
        nnz += col.entries.size();
    }
    const double density = (data.rows.empty() || data.columns.empty())
        ? 0.0
        : static_cast<double>(nnz) / static_cast<double>(data.rows.size() * data.columns.size());
    debug::info_fmt(
        "TOB ILP model summary: records={} active_bumps={} rows={} cols={} binaries={} nnz={} density={:.6e}",
        records.size(),
        active_bumps.size(),
        data.rows.size(),
        data.columns.size(),
        binary_cols,
        nnz,
        density);

    const unsigned int hw_threads = std::thread::hardware_concurrency();
    const int threads = enable_parallel ? static_cast<int>(hw_threads > 1U ? hw_threads : 1U) : 1;
    auto sol = std::vector<double> {};
    const auto col_index = data.column_index;

    try {
        GRBEnv env {true};
        configure_gurobi_solver_log(env, "TOB_ILP", diag);
        env.start();

        GRBModel grb_model {env};
        grb_model.set(GRB_StringAttr_ModelName, "TOB_ALLOC");
        grb_model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
        if (!diag.enable_gurobi_log) {
            grb_model.set(GRB_IntParam_OutputFlag, 0);
        }
        grb_model.set(GRB_IntParam_Threads, threads);

        auto vars = std::vector<GRBVar> {};
        vars.reserve(data.columns.size());
        for (const auto& col : data.columns) {
            vars.push_back(grb_model.addVar(0.0, 1.0, col.objective, GRB_BINARY, col.name));
        }
        grb_model.update();

        auto row_expr = std::vector<GRBLinExpr>(data.rows.size());
        for (std::size_t c = 0; c < data.columns.size(); ++c) {
            for (const auto& [row, coeff] : data.columns[c].entries) {
                row_expr[row] += coeff * vars[c];
            }
        }
        for (std::size_t r = 0; r < data.rows.size(); ++r) {
            char sense = GRB_EQUAL;
            if (data.rows[r].type == 'L') {
                sense = GRB_LESS_EQUAL;
            }
            else if (data.rows[r].type == 'G') {
                sense = GRB_GREATER_EQUAL;
            }
            grb_model.addConstr(row_expr[r], sense, data.rows[r].rhs, data.rows[r].name);
        }

        log_gurobi_modelinfo(
            diag.log_dir,
            std::format(
                "Gurobi parallel setup: hw_threads={}, requested_threads={}, configured_threads={}",
                hw_threads,
                threads,
                grb_model.get(GRB_IntParam_Threads)));

        auto gurobi_row_meta = std::Vector<GurobiRowMeta> {};
        gurobi_row_meta.reserve(row_meta.size());
        for (const auto& meta : row_meta) {
            gurobi_row_meta.push_back(GurobiRowMeta {meta.kind, meta.detail});
        }
        log_gurobi_matrix_diagnostics(grb_model, "TOB_ILP", diag, &gurobi_row_meta);

        grb_model.optimize();
        out.model_status = grb_model.get(GRB_IntAttr_Status);
        if (out.model_status != GRB_OPTIMAL) {
            out.ok = false;
            out.message = std::format("Gurobi model not optimal (status={})", out.model_status);
            if (out.model_status == GRB_INFEASIBLE && row_meta.size() == data.rows.size()) {
                auto row_index_by_name = std::map<std::String, std::size_t> {};
                for (std::size_t r = 0; r < data.rows.size(); ++r) {
                    row_index_by_name.emplace(data.rows[r].name, r);
                }
                grb_model.computeIIS();
                const auto num_constrs = grb_model.get(GRB_IntAttr_NumConstrs);
                const auto constrs = grb_model.getConstrs();
                auto seen_rows = std::set<std::size_t> {};
                for (int ci = 0; ci < num_constrs; ++ci) {
                    const auto& constr = constrs[ci];
                    if (constr.get(GRB_IntAttr_IISConstr) == 0) {
                        continue;
                    }
                    const auto name = constr.get(GRB_StringAttr_ConstrName);
                    const auto row_it = row_index_by_name.find(name);
                    if (row_it == row_index_by_name.end()) {
                        continue;
                    }
                    const auto row = row_it->second;
                    if (seen_rows.insert(row).second) {
                        out.infeasibility_hints.push_back(row_meta[row]);
                    }
                }
            }
            return out;
        }

        out.objective = grb_model.get(GRB_DoubleAttr_ObjVal);
        sol.resize(vars.size(), 0.0);
        for (std::size_t i = 0; i < vars.size(); ++i) {
            sol[i] = vars[i].get(GRB_DoubleAttr_X);
        }
    }
    catch (const GRBException& e) {
        out.ok = false;
        out.message = std::format("Gurobi exception {}: {}", e.getErrorCode(), e.getMessage());
        return out;
    }
    catch (const std::exception& e) {
        out.ok = false;
        out.message = std::format("Gurobi solve failed: {}", e.what());
        return out;
    }

    if (sol.empty()) {
        out.ok = false;
        out.message = "Gurobi returned empty solution";
        return out;
    }

    // parse solution
    constexpr double z_tol = 0.5;
    const auto is_active = [&](const std::String& var_name) -> bool {
        const auto it = col_index.find(var_name);
        if (it == col_index.end()) {
            return false;
        }
        const auto idx = static_cast<std::size_t>(it->second);
        if (idx >= sol.size()) {
            out.ok = false;
            out.message = std::format("column index out of range for variable '{}'", var_name);
            return false;
        }
        return sol[idx] > z_tol;
    };
    const auto track_from_jk = [](const std::size_t bank, const std::size_t j, const std::size_t k, const bool straight) -> std::size_t {
        const auto v = j * 8 + k;
        std::size_t track = 0;
        if (bank == 0) {
            track = straight ? v : (v + 64);
        }
        else {
            track = straight ? (v + 64) : v;
        }
        return track;
    };
    const auto cob_from_jk = [&](const std::size_t bank, const std::size_t j, const std::size_t k, const bool straight) -> std::size_t {
        return map_track(track_from_jk(bank, j, k, straight));
    };
    const auto relation_bumps_for = [](const Net_cost_record& record) {
        auto relation_bumps = std::Vector<Bump_coord> {};
        if (record.type == Net_type::Bnet) {
            relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
            relation_bumps.insert(relation_bumps.end(), record.end_bumps.begin(), record.end_bumps.end());
        }
        else {
            relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
        }
        std::sort(relation_bumps.begin(), relation_bumps.end());
        relation_bumps.erase(std::unique(relation_bumps.begin(), relation_bumps.end()), relation_bumps.end());
        return relation_bumps;
    };

    const auto pn_bumps_text = [](const Net_cost_record& r) -> std::String {
        if (r.start_bumps.empty()) {
            return std::String("(none)");
        }
        auto s = std::String {};
        for (std::size_t i = 0; i < r.start_bumps.size(); ++i) {
            if (i != 0) {
                s += "; ";
            }
            const auto& b = r.start_bumps[i];
            s += std::format("T{},B{},G{},I{}", b.TOB, b.Bank, b.Group, b.Index);
        }
        return s;
    };

    auto pn_selected_end_track = std::Vector<std::size_t> {};
    pn_selected_end_track.resize(records.size(), std::numeric_limits<std::size_t>::max());
    for (std::size_t n = 0; n < records.size(); ++n) {
        if (records[n].type != Net_type::PNnet) {
            continue;
        }
        int selected = 0;
        std::size_t selected_track = std::numeric_limits<std::size_t>::max();
        for (const auto r_end : records[n].end_tracks) {
            if (!is_active(y_var(n, r_end))) {
                continue;
            }
            selected += 1;
            selected_track = r_end;
        }
        if (selected != 1) {
            out.ok = false;
            const auto& rpn = records[n];
            const auto logical_name = rpn.origin_key.empty() ? std::String("(empty origin_key; use net_name)") : rpn.origin_key;
            auto power_lab = std::string_view {"None"};
            if (rpn.power_kind == IlpPowerKind::Pose) {
                power_lab = "Pose";
            }
            else if (rpn.power_kind == IlpPowerKind::Nege) {
                power_lab = "Nege";
            }
            out.message = std::format(
                "expected exactly one active Y for PNnet: records_index={} record_id={} bit_id={} "
                "2pin_record=\"{}\" logical_net(origin_key)=\"{}\" power_kind={} bump(s)=[{}], got {}",
                n,
                rpn.record_id,
                rpn.bit_id,
                rpn.net_name,
                logical_name,
                power_lab,
                pn_bumps_text(rpn),
                selected);
            return out;
        }
        pn_selected_end_track[n] = selected_track;
    }

    auto all_related_bumps = std::set<Bump_coord> {};
    for (const auto& record : records) {
        const auto relation_bumps = relation_bumps_for(record);
        all_related_bumps.insert(relation_bumps.begin(), relation_bumps.end());
    }

    auto chosen_track_by_bump = std::map<Bump_coord, std::size_t> {};
    for (const auto& bump : all_related_bumps) {
        for (std::size_t j = 0; j < 8; ++j) {
            for (std::size_t k = 0; k < 8; ++k) {
                if (!is_active(w_var(bump, j, k))) {
                    continue;
                }
                const bool qs_active = is_active(qs_var(bump, j, k));
                const bool qw_active = is_active(qw_var(bump, j, k));
                const bool has_track = (qs_active != qw_active);
                const bool use_straight = qs_active && !qw_active;
                const std::size_t track = has_track ? track_from_jk(bump.Bank, j, k, use_straight)
                                                    : std::numeric_limits<std::size_t>::max();
                if (!has_track) {
                    out.ok = false;
                    out.message = std::format(
                        "active W without resolved track for bump(T{},B{},G{},I{})",
                        bump.TOB,
                        bump.Bank,
                        bump.Group,
                        bump.Index);
                    return out;
                }
                if (const auto it = chosen_track_by_bump.find(bump); it == chosen_track_by_bump.end()) {
                    chosen_track_by_bump.emplace(bump, track);
                }
                else if (it->second != track) {
                    out.ok = false;
                    out.message = std::format(
                        "bump(T{},B{},G{},I{}) selects multiple tracks ({}, {})",
                        bump.TOB,
                        bump.Bank,
                        bump.Group,
                        bump.Index,
                        it->second,
                        track);
                    return out;
                }
                out.active_w.push_back(TobIlpWAssignment {bump, j, k, track, has_track, use_straight});
            }
        }
    }
    for (const auto& bump : all_related_bumps) {
        if (!chosen_track_by_bump.contains(bump)) {
            out.ok = false;
            out.message = std::format(
                "no selected track for bump(T{},B{},G{},I{})",
                bump.TOB,
                bump.Bank,
                bump.Group,
                bump.Index);
            return out;
        }
    }

    auto s_by_tv_from_w = std::set<std::pair<std::size_t, std::size_t>> {};
    out.active_s.clear();
    out.active_s.reserve(out.active_w.size());
    for (const auto& w : out.active_w) {
        const auto v = w.j * 8 + w.k;
        const auto key = std::pair<std::size_t, std::size_t> {w.bump.TOB, v};
        if (!s_by_tv_from_w.insert(key).second) {
            continue;
        }
        out.active_s.push_back(TobIlpSAssignment {w.bump.TOB, v, w.j, w.k});
    }

    auto orphan_s_by_tv = std::set<std::pair<std::size_t, std::size_t>> {};
    for (const auto& bump : all_related_bumps) {
        for (std::size_t v = 0; v < 64; ++v) {
            if (!is_active(s_var(bump.TOB, v))) {
                continue;
            }
            const auto key = std::pair<std::size_t, std::size_t> {bump.TOB, v};
            if (s_by_tv_from_w.contains(key)) {
                continue;
            }
            orphan_s_by_tv.insert(key);
        }
    }
    if (!orphan_s_by_tv.empty()) {
        debug::info_fmt("ILP parse: orphan S variables ignored={}", orphan_s_by_tv.size());
    }

    out.assignments.reserve(records.size());
    out.record_track_endpoints.clear();
    out.record_track_endpoints.reserve(records.size());
    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto relation_bumps = relation_bumps_for(records[n]);
        if (relation_bumps.empty()) {
            out.ok = false;
            out.message = std::format("net '{}' has no relation bump", records[n].net_name);
            return out;
        }
        std::size_t derived_cob = std::numeric_limits<std::size_t>::max();
        for (const auto& bump : relation_bumps) {
            const auto tr_it = chosen_track_by_bump.find(bump);
            if (tr_it == chosen_track_by_bump.end()) {
                out.ok = false;
                out.message = std::format(
                    "missing selected track for bump(T{},B{},G{},I{}) in net '{}'",
                    bump.TOB,
                    bump.Bank,
                    bump.Group,
                    bump.Index,
                    records[n].net_name);
                return out;
            }
            const auto cob = map_track(tr_it->second);
            if (derived_cob == std::numeric_limits<std::size_t>::max()) {
                derived_cob = cob;
            }
            else if (derived_cob != cob) {
                out.ok = false;
                out.message = std::format(
                    "track-derived cobunit mismatch in net '{}': {} vs {}",
                    records[n].net_name,
                    derived_cob,
                    cob);
                return out;
            }
        }
        out.assignments.push_back(TobIlpNetAssignment {records[n].net_name, derived_cob});

        TobIlpRecordTrackEndpoint endpoint {};
        endpoint.record_id = records[n].record_id;
        endpoint.cob_unit = derived_cob;
        if (!records[n].start_bumps.empty()) {
            const auto it = chosen_track_by_bump.find(records[n].start_bumps.front());
            if (it != chosen_track_by_bump.end()) {
                endpoint.has_start_track = true;
                endpoint.start_track = it->second;
            }
        }

        if (records[n].type == Net_type::Bnet) {
            if (!records[n].end_bumps.empty()) {
                const auto it = chosen_track_by_bump.find(records[n].end_bumps.front());
                if (it != chosen_track_by_bump.end()) {
                    endpoint.has_end_track = true;
                    endpoint.end_track = it->second;
                }
            }
        }
        else if (records[n].type == Net_type::Tnet) {
            if (records[n].mcf_has_end_track) {
                endpoint.has_end_track = true;
                endpoint.end_track = records[n].mcf_end_track.index;
            }
        }
        else if (records[n].type == Net_type::PNnet) {
            endpoint.has_end_track = true;
            endpoint.end_track = pn_selected_end_track[n];
        }
        out.record_track_endpoints.push_back(endpoint);
    }

    out.route_details.reserve(records.size() * 4);
    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto relation_bumps = relation_bumps_for(records[n]);
        for (const auto& bump : relation_bumps) {
            for (std::size_t j = 0; j < 8; ++j) {
                for (std::size_t k = 0; k < 8; ++k) {
                    if (!is_active(w_var(bump, j, k))) {
                        continue;
                    }
                    const bool qs_active = is_active(qs_var(bump, j, k));
                    const bool qw_active = is_active(qw_var(bump, j, k));
                    if (qs_active == qw_active) {
                        continue;
                    }
                    const bool use_straight = qs_active;
                    const std::size_t s_v = j * 8 + k;
                    out.route_details.push_back(TobIlpNetRouteDetail {
                        records[n].net_name,
                        bump,
                        j,
                        k,
                        s_v,
                        track_from_jk(bump.Bank, j, k, use_straight),
                        cob_from_jk(bump.Bank, j, k, use_straight),
                        use_straight
                    });
                }
            }
        }
    }

    out.ok = true;
    out.message = "ok";
    return out;
}

} // namespace PR_tool
