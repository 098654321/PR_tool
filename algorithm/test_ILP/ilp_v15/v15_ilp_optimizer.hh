#pragma once

#include "ilp_v15/v15_ilp_types.hh"
#include "delay/pair_delay_precompute.hh"
#include "sat/unified_sat_scope.hh"

#include <hardware/interposer.hh>

namespace PR_tool {

auto optimize_v15_routes(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const DelayPrecomputeResult& delays,
    const SatRoutingResult& sat_result,
    const V15IlpOptimizeOptions& options
) -> V15IlpOptimizeResult;

} // namespace PR_tool
