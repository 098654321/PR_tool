#include "direct_ilp/undirected_graph.hh"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace PR_tool {
namespace {

auto key(int u, int v) -> std::uint64_t {
    const auto lo = static_cast<std::uint32_t>(std::min(u, v));
    const auto hi = static_cast<std::uint32_t>(std::max(u, v));
    return (static_cast<std::uint64_t>(lo) << 32U) | hi;
}

auto matching_attributes(const UnifiedArc& a, const DirectEdge& e) -> bool {
    return a.physical_switch_id == e.switch_id
        && a.physical_switch_kind == e.switch_kind
        && a.mode_group_id == e.mode_group
        && a.is_vline_track_straight == e.straight
        && a.is_vline_track_swap == e.swap;
}

} // namespace

auto build_direct_graph(const UnifiedGraph& graph) -> DirectGraph {
    auto out = DirectGraph{};
    out.incident.resize(graph.nodes.size());
    out.edge_by_arc.assign(graph.arcs.size(), -1);
    auto by_pair = std::unordered_map<std::uint64_t, int>{};
    by_pair.reserve(graph.arcs.size() / 2);
    for (int arc_id = 0; arc_id < static_cast<int>(graph.arcs.size()); ++arc_id) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        if (arc.u < 0 || arc.v < 0 || arc.u == arc.v
            || static_cast<std::size_t>(arc.u) >= graph.nodes.size()
            || static_cast<std::size_t>(arc.v) >= graph.nodes.size()) {
            throw std::logic_error("invalid unified arc endpoint");
        }
        if (arc.is_virtual_source_arc) {
            if (graph.nodes[static_cast<std::size_t>(arc.u)].kind
                    != UnifiedNodeKind::VirtualSource
                || graph.nodes[static_cast<std::size_t>(arc.v)].kind
                    != UnifiedNodeKind::Track) {
                throw std::logic_error("invalid PN virtual source arc");
            }
            const int id = static_cast<int>(out.edges.size());
            out.edges.push_back({arc.u, arc.v, {arc_id}, -1,
                                 PhysicalSwitchKind::None, -1, false, false, true});
            out.incident[static_cast<std::size_t>(arc.u)].push_back(id);
            out.incident[static_cast<std::size_t>(arc.v)].push_back(id);
            out.edge_by_arc[static_cast<std::size_t>(arc_id)] = id;
            continue;
        }
        if (graph.nodes[static_cast<std::size_t>(arc.u)].kind
                == UnifiedNodeKind::VirtualSource
            || graph.nodes[static_cast<std::size_t>(arc.v)].kind
                == UnifiedNodeKind::VirtualSource) {
            throw std::logic_error("physical arc touches PN virtual source");
        }
        const auto pair = key(arc.u, arc.v);
        const auto [it, inserted] = by_pair.emplace(pair, static_cast<int>(out.edges.size()));
        if (inserted) {
            const int id = it->second;
            out.edges.push_back({arc.u, arc.v, {arc_id}, arc.physical_switch_id,
                                 arc.physical_switch_kind, arc.mode_group_id,
                                 arc.is_vline_track_straight,
                                 arc.is_vline_track_swap, false});
            out.incident[static_cast<std::size_t>(arc.u)].push_back(id);
            out.incident[static_cast<std::size_t>(arc.v)].push_back(id);
        } else {
            auto& edge = out.edges[static_cast<std::size_t>(it->second)];
            if (edge.arcs.size() != 1 || edge.u != arc.v || edge.v != arc.u
                || !matching_attributes(arc, edge)) {
                throw std::logic_error("physical arcs cannot form one undirected edge");
            }
            edge.arcs.push_back(arc_id);
        }
        out.edge_by_arc[static_cast<std::size_t>(arc_id)] = it->second;
    }
    for (const auto& edge : out.edges) {
        if (!edge.virtual_source && edge.arcs.size() != 2) {
            throw std::logic_error("physical arc has no matching reverse arc");
        }
    }
    return out;
}

} // namespace PR_tool
