#pragma once

#include "ilp_v15/v15_ilp_types.hh"
#include "sat/routing_round_diagnostics.hh"

#include <set>

namespace PR_tool {

auto select_v15_net_ids(
    const std::Vector<NetStretchInfo>& entries,
    double threshold_percent
) -> std::set<std::size_t>;

auto collect_v15_locked_resources(
    const UnifiedGraph& graph,
    const SatRoutingResult& sat_result,
    const std::set<std::size_t>& selected_net_ids
) -> V15LockedResources;

auto build_v15_sat_mip_start(
    const V15PrepareResult& prepared,
    const SatRoutingResult& sat_result
) -> V15MipStart;

} // namespace PR_tool
