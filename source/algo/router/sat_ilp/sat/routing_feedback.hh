#pragma once

#include "common/routing_types.hh"
#include "sat/solve_unified_sat.hh"
#include "scope/pair_routing_state.hh"

#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

namespace PR_tool {

enum class FeedbackExpansionStatus {
    Expanded,
    Exhausted,
};

auto apply_feedback_expansion(
    RoutingProblemState& state,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<PairKey>& critical_pairs
) -> FeedbackExpansionStatus;

auto solve_with_feedback(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult;

} // namespace PR_tool
