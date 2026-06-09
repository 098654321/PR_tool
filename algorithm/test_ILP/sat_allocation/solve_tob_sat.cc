#include "sat_allocation/solve_tob_sat.hh"

#include "precompute/tob_reach_with_range.hh"
#include "sat_allocation/tob_allocation_result.hh"
#include "sat_allocation/tob_sat_encoder.hh"

#include <algorithm>
#include <chrono>
#include <debug/debug.hh>
#include <format>

namespace PR_tool {

namespace {

auto sat_failure_expansion_records(const std::Vector<Net_cost_record>& records) -> std::Vector<std::size_t> {
    auto out = std::Vector<std::size_t> {};
    for (std::size_t i = 0; i < records.size(); ++i) {
        if (records[i].type == Net_type::Tnet || records[i].type == Net_type::PNnet) {
            out.push_back(i);
        }
    }
    return out;
}

auto format_record_indices(const std::Vector<std::size_t>& indices) -> std::String {
    if (indices.empty()) {
        return "{}";
    }
    auto out = std::String {"{"};
    for (std::size_t i = 0; i < indices.size(); ++i) {
        if (i != 0) {
            out += ",";
        }
        out += std::format("{}", indices[i]);
    }
    out += "}";
    return out;
}

auto set_bbox_state_on_result(TobIlpResult& out, const TobBBoxExpansionState& state) -> void {
    out.bbox_expand_by_record = state.rho_by_record;
    out.range_level = state.max_rho();
}

} // namespace

auto solve_tob_sat_with_bbox_state(
    std::Vector<Net_cost_record>& records,
    const TobBBoxExpansionState& state,
    const CadicalDiagnosticsOptions& base_diag
) -> TobIlpResult {
    auto out = TobIlpResult {};
    set_bbox_state_on_result(out, state);

    const auto reach_t0 = std::chrono::steady_clock::now();
    const auto reach_stats = precompute_reach_for_bbox_state(records, state);
    const auto reach_t1 = std::chrono::steady_clock::now();
    const auto reach_ms = std::chrono::duration_cast<std::chrono::milliseconds>(reach_t1 - reach_t0).count();
    debug::info_fmt(
        "SAT reach precompute: max_rho={} records={} total_endtracks={} total_starttrack_edges={} ms={}",
        state.max_rho(),
        reach_stats.total_records,
        reach_stats.total_endtracks,
        reach_stats.total_starttrack_edges,
        reach_ms);

    if (base_diag.verbose_reach_endpoints) {
        log_reach_endpoints_for_bbox_state(records, state);
    }

    const auto build_t0 = std::chrono::steady_clock::now();
    const auto cnf = build_tob_sat_cnf(records);
    const auto build_t1 = std::chrono::steady_clock::now();
    const auto build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(build_t1 - build_t0).count();
    debug::info_fmt(
        "SAT model: max_rho={} vars={} clauses={} build_ms={}",
        state.max_rho(),
        cnf.num_vars,
        cnf.num_clauses,
        build_ms);

    auto diag = base_diag;
    diag.range_level = state.max_rho();

    const auto solve_t0 = std::chrono::steady_clock::now();
    const auto sat = solve_tob_sat_cnf(cnf, diag);
    const auto solve_t1 = std::chrono::steady_clock::now();
    const auto solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(solve_t1 - solve_t0).count();
    debug::info_fmt(
        "SAT max_rho={} status={} solve_ms={}",
        state.max_rho(),
        sat.message,
        solve_ms);

    if (!sat.ok) {
        out.ok = false;
        out.message = sat.message;
        out.model_status = sat.status;
        set_bbox_state_on_result(out, state);
        return out;
    }

    const auto is_true = [&](const std::string_view name) -> bool {
        const auto it = sat.assignment.find(std::String(name));
        return it != sat.assignment.end() && it->second;
    };
    out = build_tob_ilp_result_from_assignment(records, is_true);
    set_bbox_state_on_result(out, state);
    out.model_status = sat.status;
    return out;
}

auto solve_tob_sat_at_range_level(
    std::Vector<Net_cost_record>& records,
    const std::size_t range_level,
    const CadicalDiagnosticsOptions& base_diag
) -> TobIlpResult {
    return solve_tob_sat_with_bbox_state(records, TobBBoxExpansionState::uniform(records.size(), range_level), base_diag);
}

auto solve_tob_sat_with_cadical(
    std::Vector<Net_cost_record>& records,
    const CadicalDiagnosticsOptions& base_diag
) -> TobIlpResult {
    auto out = TobIlpResult {};
    auto state = TobBBoxExpansionState::initial(records.size());
    const auto fail_set = sat_failure_expansion_records(records);
    const auto max_attempts = records.size() * kTobBBoxMaxExpand + 1;

    for (std::size_t attempt = 0; attempt < max_attempts; ++attempt) {
        debug::info_fmt(
            "SAT-only bbox attempt={} max_rho={} rho_by_record={}",
            attempt,
            state.max_rho(),
            state.rho_summary());
        out = solve_tob_sat_with_bbox_state(records, state, base_diag);
        if (!out.ok) {
            if (out.model_status != 20) {
                debug::error_fmt("SAT assignment parse failed at max_rho={}: {}", state.max_rho(), out.message);
                return out;
            }
            const auto changed = state.expand_records(fail_set);
            debug::info_fmt(
                "SAT-only bbox expand: attempt={} changed_records={} max_rho={} rho_by_record={}",
                attempt,
                format_record_indices(changed),
                state.max_rho(),
                state.rho_summary());
            if (changed.empty()) {
                out.ok = false;
                out.message = std::format(
                    "SAT: UNSAT and no Tnet/PNnet bbox record can expand further (max_rho={})",
                    state.max_rho());
                set_bbox_state_on_result(out, state);
                return out;
            }
            continue;
        }
        return out;
    }

    out.ok = false;
    out.message = std::format("SAT: bbox retry attempts exhausted (attempts={})", max_attempts);
    set_bbox_state_on_result(out, state);
    return out;
}

} // namespace PR_tool
