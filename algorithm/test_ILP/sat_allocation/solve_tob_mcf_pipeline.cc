#include "sat_allocation/solve_tob_mcf_pipeline.hh"

#include "mcf/mcf_graph.hh"
#include "sat_allocation/solve_tob_sat.hh"

#include <chrono>
#include <debug/debug.hh>
#include <format>

namespace PR_tool {

namespace {

auto log_mcf_failure_reason(const CobMcfFullResult& mcf, const std::size_t range_level) -> void {
    auto failed_units = std::Vector<std::size_t> {};
    for (std::size_t u = 0; u < 16; ++u) {
        if (mcf.has_simple_commodities[u] && !mcf.simple_mcf_ok[u]) {
            failed_units.push_back(u);
        }
    }
    if (!failed_units.empty()) {
        auto unit_text = std::String {};
        for (std::size_t i = 0; i < failed_units.size(); ++i) {
            if (i > 0) {
                unit_text += ",";
            }
            unit_text += std::to_string(failed_units[i]);
        }
        debug::info_fmt(
            "range iteration: MCF failed at level={}, expanding range (SimpleMCF units {})",
            range_level,
            unit_text);
        return;
    }
    debug::info_fmt(
        "range iteration: MCF failed at level={}, expanding range (BusMCF)",
        range_level);
}

} // namespace

auto solve_tob_mcf_with_range_iteration(
    std::Vector<Net_cost_record>& records,
    hardware::Interposer* interposer,
    const circuit::BaseDie& basedie,
    const CobMcfGridDims cob_grid,
    const bool enable_mcf_parallel,
    const bool enable_pre_routing,
    const bool enable_mcf_obj,
    const bool disable_bus_mcf,
    const bool apply_interposer_suspend_on_success,
    const CadicalDiagnosticsOptions& sat_diag,
    const GurobiDiagnosticsOptions& gurobi_diag
) -> TobMcfPipelineResult {
    auto out = TobMcfPipelineResult {};

    for (std::size_t range_level = 0; range_level <= kMaxRangeLevel; ++range_level) {
        out.attempts += 1;

        const auto sat_t0 = std::chrono::steady_clock::now();
        auto tob = solve_tob_sat_at_range_level(records, range_level, sat_diag);
        const auto sat_t1 = std::chrono::steady_clock::now();
        out.tob_sat_solve_ms += std::chrono::duration_cast<std::chrono::milliseconds>(sat_t1 - sat_t0).count();

        if (!tob.ok) {
            if (tob.model_status == 20) {
                debug::info_fmt("range iteration: level={} SAT=UNSAT", range_level);
            }
            else {
                debug::error_fmt(
                    "range iteration: level={} SAT=parse_fail ({})",
                    range_level,
                    tob.message);
            }
            continue;
        }

        debug::info_fmt("range iteration: level={} SAT=SAT", range_level);

        const auto mcf = run_mcf_global_routing_cob_units(
            records,
            tob,
            interposer,
            basedie,
            cob_grid,
            enable_mcf_parallel,
            enable_pre_routing,
            enable_mcf_obj,
            true,
            disable_bus_mcf,
            gurobi_diag);

        out.mcf_warm_start_ms += mcf.summary.mcf_warm_start_ms;
        out.mcf_solve_ms += mcf.summary.mcf_solve_ms;

        if (!mcf.summary.all_ok) {
            debug::info_fmt("range iteration: level={} SAT=SAT MCF=fail", range_level);
            log_mcf_failure_reason(mcf, range_level);
            continue;
        }

        debug::info_fmt("range iteration: level={} SAT=SAT MCF=ok", range_level);

        if (apply_interposer_suspend_on_success && interposer != nullptr) {
            const auto graph = build_mcf_track_graph(cob_grid);
            suspend_mcf_paths_on_interposer(interposer, graph, mcf.paths_by_unit);
        }

        out.ok = true;
        out.message = std::format("SAT+MCF solved at range_level={}", range_level);
        out.tob = std::move(tob);
        out.mcf = mcf;
        out.range_level = range_level;
        debug::info_fmt(
            "SAT+MCF solved at range_level={} (attempts={})",
            range_level,
            out.attempts);
        return out;
    }

    out.ok = false;
    out.message = std::format("SAT+MCF: all range levels failed (0..{})", kMaxRangeLevel);
    debug::error_fmt("{}", out.message);
    return out;
}

} // namespace PR_tool
