#include "sat_allocation/solve_tob_sat.hh"

#include "precompute/tob_reach_with_range.hh"
#include "sat_allocation/tob_allocation_result.hh"
#include "sat_allocation/tob_sat_encoder.hh"

#include <chrono>
#include <debug/debug.hh>
#include <format>

namespace PR_tool {

auto solve_tob_sat_with_cadical(
    std::Vector<Net_cost_record>& records,
    const CadicalDiagnosticsOptions& base_diag
) -> TobIlpResult {
    auto out = TobIlpResult {};
    constexpr std::size_t kMaxRangeLevel = 4;

    for (std::size_t range_level = 0; range_level <= kMaxRangeLevel; ++range_level) {
        const auto reach_t0 = std::chrono::steady_clock::now();
        const auto reach_stats = precompute_reach_for_range(records, range_level);
        const auto reach_t1 = std::chrono::steady_clock::now();
        const auto reach_ms = std::chrono::duration_cast<std::chrono::milliseconds>(reach_t1 - reach_t0).count();
        debug::info_fmt(
            "SAT reach precompute: range_level={} records={} total_endtracks={} total_starttrack_edges={} ms={}",
            reach_stats.range_level,
            reach_stats.total_records,
            reach_stats.total_endtracks,
            reach_stats.total_starttrack_edges,
            reach_ms);
        if (base_diag.verbose_reach_endpoints) {
            log_reach_endpoints_for_range(records, range_level);
        }

        const auto build_t0 = std::chrono::steady_clock::now();
        const auto cnf = build_tob_sat_cnf(records);
        const auto build_t1 = std::chrono::steady_clock::now();
        const auto build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(build_t1 - build_t0).count();
        debug::info_fmt(
            "SAT model: range_level={} vars={} clauses={} build_ms={}",
            range_level,
            cnf.num_vars,
            cnf.num_clauses,
            build_ms);

        auto diag = base_diag;
        diag.range_level = range_level;

        const auto solve_t0 = std::chrono::steady_clock::now();
        const auto sat = solve_tob_sat_cnf(cnf, diag);
        const auto solve_t1 = std::chrono::steady_clock::now();
        const auto solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(solve_t1 - solve_t0).count();
        debug::info_fmt(
            "SAT range_level={} status={} solve_ms={}",
            range_level,
            sat.message,
            solve_ms);

        if (!sat.ok) {
            continue;
        }

        const auto is_true = [&](const std::string_view name) -> bool {
            const auto it = sat.assignment.find(std::String(name));
            return it != sat.assignment.end() && it->second;
        };
        out = build_tob_ilp_result_from_assignment(records, is_true);
        out.range_level = range_level;
        out.model_status = sat.status;
        if (out.ok) {
            return out;
        }
        debug::error_fmt("SAT assignment parse failed at range_level={}: {}", range_level, out.message);
    }

    out.ok = false;
    out.message = "SAT: all range levels UNSAT (0..4)";
    return out;
}

} // namespace PR_tool
