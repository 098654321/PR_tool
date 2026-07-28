#include "sat_backend.hh"

#include <sat/solve_unified_sat.hh>

#include <circuit/basedie.hh>
#include <debug/debug.hh>
#include <hardware/interposer.hh>

#include <stdexcept>

namespace PR_tool::algo {

auto SatRouterBackend::run(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const RouterOptions& options
) -> RouteStatus {
#if !PR_TOOL_HAS_SAT_ROUTER
    (void)interposer;
    (void)basedie;
    (void)options;
    throw std::runtime_error("sat router unsupported");
#else
    if (options.kind != RouterKind::Sat) {
        throw std::runtime_error("SatRouterBackend: kind/backend mismatch");
    }
    if (options.try_all_modes) {
        debug::error("SAT router does not support --try-all-modes");
        return RouteStatus::Failed;
    }
    if (options.mode != 0) {
        debug::error("SAT router does not support incremental control-bit modes");
        return RouteStatus::Failed;
    }
    if (options.compare.has_value()) {
        debug::error("SAT router does not support --compare");
        return RouteStatus::Failed;
    }

    UnifiedSatSolveOptions sat_options {};
    sat_options.verbose_level = options.sat.verbose_level;
    sat_options.cadical.verbose_level = options.sat.verbose_level;
    sat_options.cadical.enable_sat_log = options.sat.enable_sat_log;
    sat_options.cadical.max_rss_mb = options.sat.max_rss_mb;
    sat_options.initial_scope_pad = options.sat.initial_scope_pad;
    sat_options.initial_delay_pad = options.sat.initial_delay_pad;
    sat_options.ilp_optimize.enabled = options.sat.enable_ilp_optimize;
    sat_options.ilp_optimize.verbose_level = options.sat.verbose_level;
    sat_options.ilp_optimize.gurobi_log_dir = options.sat.gurobi_log_dir;
    if (options.sat.ilp_stretch_threshold_percent.has_value()) {
        sat_options.ilp_optimize.stretch_threshold_percent =
            options.sat.ilp_stretch_threshold_percent.value();
    }
    if (options.sat.ilp_segment_bbox_pad.has_value()) {
        sat_options.ilp_optimize.segment_bbox_pad = options.sat.ilp_segment_bbox_pad.value();
    }
    if (options.sat.ilp_time_limit_hours.has_value()) {
        sat_options.ilp_optimize.time_limit_hours = options.sat.ilp_time_limit_hours.value();
    }

    debug::debug("Start SAT routing ...");
    const auto result = solve_unified_sat_and_commit(interposer, *basedie, sat_options);
    if (!result.ok) {
        debug::error_fmt("SAT routing failed: {}", result.message);
        return RouteStatus::Failed;
    }

    debug::info_fmt(
        "SAT routing ok: paths={} total_wirelength={} feedback_rounds={}",
        result.paths.size(),
        result.total_wirelength,
        result.feedback_rounds);
    return RouteStatus::Ok;
#endif
}

} // namespace PR_tool::algo
