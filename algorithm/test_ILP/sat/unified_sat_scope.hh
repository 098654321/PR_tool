#pragma once

#include "common/routing_scope.hh"
#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"

#include <std/collection.hh>

namespace PR_tool {

auto build_scope(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const std::Vector<int>& source_nodes,
    const std::Vector<int>& sink_nodes,
    const std::Vector<int>& extra_nodes
) -> UnifiedSatNetScope;

auto build_all_scopes(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets
) -> std::Vector<UnifiedSatNetScope>;

auto resolve_endpoint_nodes(
    const UnifiedGraph& graph,
    const RoutingNet& net
) -> std::pair<std::Vector<int>, std::Vector<int>>;

} // namespace PR_tool
