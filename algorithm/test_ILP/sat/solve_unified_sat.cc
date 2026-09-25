#include "sat/solve_unified_sat.hh"

#include "sat/routing_feedback.hh"

namespace PR_tool {

auto solve_unified_sat(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult {
    return solve_with_feedback(interposer, basedie, options);
}

} // namespace PR_tool
