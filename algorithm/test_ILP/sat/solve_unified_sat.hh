#pragma once

#include "common/routing_types.hh"
#include "sat_allocation/cadical_solver.hh"

#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

namespace PR_tool {

struct UnifiedSatSolveOptions {
    CadicalDiagnosticsOptions cadical {};
    int verbose_level{0};
    std::size_t max_feedback_rounds{64};
    int initial_scope_pad{0};
    int initial_delay_pad{0};
};

auto solve_unified_sat(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult;

} // namespace PR_tool
