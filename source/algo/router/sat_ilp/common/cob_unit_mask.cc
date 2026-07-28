#include "common/cob_unit_mask.hh"

#include "graph/unified_routing_graph.hh"

namespace PR_tool {

auto unit_bit(const std::size_t unit) -> std::uint16_t {
    return unit < 16 ? static_cast<std::uint16_t>(std::uint16_t {1} << unit) : 0;
}

auto compute_source_unit_mask(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const int source_node
) -> std::uint16_t {
    if (net.kind == RoutingNetKind::PNnet) {
        std::uint16_t mask = 0;
        for (const auto& source_ref : net.sources) {
            const int candidate = resolve_graph_node(graph, source_ref);
            if (candidate < 0) {
                continue;
            }
            const auto& node = graph.nodes[static_cast<std::size_t>(candidate)];
            if (node.kind == UnifiedNodeKind::Track) {
                mask = static_cast<std::uint16_t>(mask | unit_bit(node.unit));
            }
        }
        return mask;
    }
    const auto& node = graph.nodes[static_cast<std::size_t>(source_node)];
    if (node.kind == UnifiedNodeKind::Track) {
        return unit_bit(node.unit);
    }
    return std::uint16_t {0xffff};
}

auto node_unit_eligible(
    const UnifiedNode& node,
    const std::uint16_t source_unit_mask
) -> bool {
    if (node.kind == UnifiedNodeKind::Track) {
        return (source_unit_mask & unit_bit(node.unit)) != 0;
    }
    if (node.kind == UnifiedNodeKind::VLine) {
        const std::size_t local_unit = node.line_index % 8;
        const auto pair_mask = static_cast<std::uint16_t>(
            unit_bit(local_unit) | unit_bit(local_unit + 8));
        return (source_unit_mask & pair_mask) != 0;
    }
    return true;
}

auto arc_unit_eligible(
    const UnifiedGraph& graph,
    const UnifiedArc& arc,
    const std::uint16_t source_unit_mask
) -> bool {
    return node_unit_eligible(
               graph.nodes[static_cast<std::size_t>(arc.u)],
               source_unit_mask)
        && node_unit_eligible(
               graph.nodes[static_cast<std::size_t>(arc.v)],
               source_unit_mask);
}

} // namespace PR_tool
