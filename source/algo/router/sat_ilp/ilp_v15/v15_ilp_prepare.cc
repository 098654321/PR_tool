#include "ilp_v15/v15_ilp_prepare.hh"

#include "graph/unified_routing_graph.hh"

#include <algorithm>
#include <map>
#include <stdexcept>

namespace PR_tool {

namespace {

auto find_arc_id(const UnifiedGraph& graph, int u, int v) -> int {
    if (u < 0 || static_cast<std::size_t>(u) >= graph.out_arc_ids.size()) {
        return -1;
    }
    for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
        if (graph.arcs[static_cast<std::size_t>(arc_id)].v == v) {
            return arc_id;
        }
    }
    return -1;
}

auto is_physical_node(const UnifiedGraph& graph, int node) -> bool {
    return node >= 0
        && static_cast<std::size_t>(node) < graph.nodes.size()
        && graph.nodes[static_cast<std::size_t>(node)].kind != UnifiedNodeKind::VirtualSource;
}

} // namespace

auto select_v15_net_ids(
    const std::Vector<NetStretchInfo>& entries,
    double threshold_percent
) -> std::set<std::size_t> {
    auto selected = std::set<std::size_t> {};
    for (const auto& entry : entries) {
        if (entry.shortest != 0 && entry.delta_percent >= threshold_percent) {
            selected.insert(entry.net_id);
        }
    }
    return selected;
}

auto collect_v15_locked_resources(
    const UnifiedGraph& graph,
    const SatRoutingResult& sat_result,
    const std::set<std::size_t>& selected_net_ids
) -> V15LockedResources {
    auto out = V15LockedResources {};
    out.node_used.assign(graph.nodes.size(), false);
    for (const auto& path : sat_result.paths) {
        if (selected_net_ids.contains(path.net_id)) {
            continue;
        }
        for (const int node : path.node_path) {
            if (is_physical_node(graph, node)) {
                out.node_used[static_cast<std::size_t>(node)] = true;
            }
        }
        for (std::size_t i = 1; i < path.node_path.size(); ++i) {
            const int arc_id = find_arc_id(graph, path.node_path[i - 1], path.node_path[i]);
            if (arc_id < 0) {
                continue;
            }
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (arc.physical_switch_id >= 0) {
                out.switch_used.insert(arc.physical_switch_id);
            }
        }
    }
    return out;
}

auto build_v15_sat_mip_start(
    const V15PrepareResult& prepared,
    const SatRoutingResult& sat_result
) -> V15MipStart {
    auto out = V15MipStart {};
    for (const auto& [group, straight] : sat_result.vline_mode_straight_by_group) {
        out.mode_straight.emplace(static_cast<int>(group), straight);
    }

    for (const auto& segment : prepared.segments) {
        auto& flow_arcs = out.segment_flow_arc_ids[segment.segment_id];
        for (const int arc_id : segment.guide_arc_ids) {
            flow_arcs.insert(arc_id);
        }
    }

    for (const auto& parent : prepared.parents) {
        auto& parent_arcs = out.parent_arc_ids[parent.parent_id];
        auto& parent_nodes = out.parent_node_ids[parent.parent_id];
        parent_nodes.insert(parent.root_node);
        for (const int sink : parent.origin_sinks) {
            parent_nodes.insert(sink);
        }
        for (const int track : parent.fixed_track_nodes) {
            parent_nodes.insert(track);
        }
        for (const std::size_t segment_id : parent.segment_ids) {
            const auto it = std::find_if(
                prepared.segments.begin(),
                prepared.segments.end(),
                [&](const V15Segment& segment) { return segment.segment_id == segment_id; });
            if (it == prepared.segments.end()) {
                continue;
            }
            for (const int arc_id : it->guide_arc_ids) {
                parent_arcs.insert(arc_id);
            }
            for (const int node : it->guide_node_path) {
                parent_nodes.insert(node);
            }
        }
    }

    out.available = !prepared.segments.empty();
    return out;
}

} // namespace PR_tool
