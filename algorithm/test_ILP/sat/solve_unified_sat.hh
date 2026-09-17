#pragma once

#include "common/routing_types.hh"
#include "sat_allocation/cadical_solver.hh"

#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

namespace PR_tool {

struct UnifiedSatSolveOptions {
    CadicalDiagnosticsOptions cadical {};
    int verbose_level{0};
    std::size_t max_feedback_rounds{200};
    int initial_scope_pad{0};
    int initial_delay_pad{0};
    bool enable_z3_optimize{false};
    bool enable_global_route_v17{false};
    bool enable_global_route_v18{false};
    std::String highs_log_path;
    int highs_time_limit_minutes{0};
};

auto solve_unified_sat(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult;

} // namespace PR_tool
