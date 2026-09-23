#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"

namespace PR_tool {

auto is_wirelength_resource_node(const UnifiedGraph& graph, int node) -> bool;
auto net_wirelength(const UnifiedGraph& graph,
                    const std::Vector<const SourceSinkPairPath*>& paths)
    -> std::size_t;
auto total_wirelength(const UnifiedGraph& graph, const RoutingResult& result)
    -> std::size_t;

} // namespace PR_tool
