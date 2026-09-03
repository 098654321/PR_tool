#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"

#include <std/collection.hh>

namespace PR_tool {

struct UnifiedSatNetScope {
    std::size_t net_id{0};
    std::Vector<int> node_ids;
    std::Vector<int> arc_ids;
    // Global ID -> compact offset, or -1 when absent.
    std::Vector<int> node_offset;
    std::Vector<int> arc_offset;
};

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
