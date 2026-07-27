#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"

#include <cstddef>
#include <cstdint>

namespace PR_tool {

auto unit_bit(std::size_t unit) -> std::uint16_t;

auto compute_source_unit_mask(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    int source_node
) -> std::uint16_t;

auto node_unit_eligible(
    const UnifiedNode& node,
    std::uint16_t source_unit_mask
) -> bool;

auto arc_unit_eligible(
    const UnifiedGraph& graph,
    const UnifiedArc& arc,
    std::uint16_t source_unit_mask
) -> bool;

} // namespace PR_tool
