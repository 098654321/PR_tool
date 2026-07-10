#pragma once

#include "ilp_v15/v15_ilp_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"
#include "sat/routing_round_diagnostics.hh"

#include <set>

namespace PR_tool {

auto select_v15_net_ids(
    const std::Vector<NetStretchInfo>& entries,
    double threshold_percent
) -> std::set<std::size_t>;

auto build_v15_commodities(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const std::set<std::size_t>& selected_net_ids
) -> std::Vector<IlpCommodity>;

auto collect_v15_locked_resources(
    const UnifiedGraph& graph,
    const SatRoutingResult& sat_result,
    const std::set<std::size_t>& selected_net_ids
) -> V15LockedResources;

auto build_v15_mip_start(
    const UnifiedGraph& graph,
    const std::Vector<IlpCommodity>& commodities,
    const SatRoutingResult& sat_result
) -> V15MipStart;

} // namespace PR_tool
