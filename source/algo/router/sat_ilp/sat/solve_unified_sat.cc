#include "sat/solve_unified_sat.hh"

#include "commit_paths.hh"
#include "sat/routing_feedback.hh"

namespace PR_tool {

auto solve_unified_sat(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult {
    return solve_with_feedback(interposer, basedie, options);
}

auto solve_unified_sat_and_commit(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult {
    auto commit_options = options;
    commit_options.commit_to_nets = true;
    return solve_with_feedback(interposer, basedie, commit_options);
}

} // namespace PR_tool
