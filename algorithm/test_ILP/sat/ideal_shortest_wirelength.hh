#pragma once

#include "common/routing_types.hh"
#include "delay/pair_delay_precompute.hh"
#include "graph/unified_routing_graph.hh"

#include <hardware/interposer.hh>

#include <cstddef>
#include <std/collection.hh>

namespace PR_tool {

auto shortest_unified_node_path(
    const UnifiedGraph& graph,
    int source_node,
    int sink_node
) -> std::Vector<int>;

auto unified_path_wirelength(const UnifiedGraph& graph, const std::Vector<int>& node_path) -> std::size_t;

auto ideal_net_wirelength(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const DelayPrecomputeResult& delays
) -> std::size_t;

} // namespace PR_tool
