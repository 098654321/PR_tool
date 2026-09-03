#include "ilp_v15/v15_ilp_domain.hh"

#include "common/cob_unit_mask.hh"

#include <algorithm>
#include <queue>

namespace PR_tool {

namespace {

auto is_physical_node(const UnifiedGraph& graph, int node) -> bool {
    return node >= 0
        && static_cast<std::size_t>(node) < graph.nodes.size()
        && graph.nodes[static_cast<std::size_t>(node)].kind != UnifiedNodeKind::VirtualSource;
}

} // namespace

auto build_v15_scope_from_bbox(
    const UnifiedGraph& graph,
    const IlpBoundingBox& bbox,
    const std::Vector<int>& force_nodes,
    const std::set<int>& allowed_virtual_nodes
) -> V15SegmentScope {
    auto included = std::Vector<bool>(graph.nodes.size(), false);
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        if (is_physical_node(graph, node) && node_in_scope(graph, node, bbox)) {
            included[static_cast<std::size_t>(node)] = true;
        }
    }
    for (const int node : force_nodes) {
        if (node >= 0
            && static_cast<std::size_t>(node) < included.size()
            && (is_physical_node(graph, node) || allowed_virtual_nodes.contains(node))) {
            included[static_cast<std::size_t>(node)] = true;
        }
    }
    auto out = V15SegmentScope {};
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        if (included[static_cast<std::size_t>(node)]) {
            out.node_ids.push_back(node);
        }
    }
    for (int arc_id = 0; arc_id < static_cast<int>(graph.arcs.size()); ++arc_id) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        if (included[static_cast<std::size_t>(arc.u)]
            && included[static_cast<std::size_t>(arc.v)]) {
            out.arc_ids.push_back(arc_id);
        }
    }
    return out;
}

auto refine_v15_segment_scope(
    const UnifiedGraph& graph,
    const V15SegmentScope& raw_scope,
    const V15Parent& parent,
    const V15Segment& segment,
    const V15LockedResources& locked
) -> V15SegmentScope {
    auto allowed = std::Vector<bool>(graph.nodes.size(), false);
    auto in_raw = std::Vector<bool>(graph.nodes.size(), false);
    for (const int node : raw_scope.node_ids) {
        if (node >= 0 && static_cast<std::size_t>(node) < in_raw.size()) {
            in_raw[static_cast<std::size_t>(node)] = true;
        }
    }
    for (const int node : raw_scope.node_ids) {
        const bool virtual_node = !is_physical_node(graph, node);
        const bool endpoint = node == segment.endpoint_a || node == segment.endpoint_b;
        if (virtual_node && (!segment.is_virtual_hop || !endpoint)) {
            continue;
        }
        const bool locked_node = is_physical_node(graph, node)
            && static_cast<std::size_t>(node) < locked.node_used.size()
            && locked.node_used[static_cast<std::size_t>(node)];
        const bool unit_ok = node_unit_eligible(
            graph.nodes[static_cast<std::size_t>(node)],
            parent.source_unit_mask);
        if (!in_raw[static_cast<std::size_t>(node)]) {
            continue;
        }
        allowed[static_cast<std::size_t>(node)] =
            (endpoint || !locked_node) && (endpoint || unit_ok);
    }

    auto candidate_arcs = std::Vector<int> {};
    auto forward = std::Vector<std::Vector<int>>(graph.nodes.size());
    auto reverse = std::Vector<std::Vector<int>>(graph.nodes.size());
    for (const int arc_id : raw_scope.arc_ids) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        const bool touches_virtual = !is_physical_node(graph, arc.u)
            || !is_physical_node(graph, arc.v);
        if ((!segment.is_virtual_hop && touches_virtual)
            || (segment.is_virtual_hop
                && (segment.guide_arc_ids.size() != 1 || arc_id != segment.guide_arc_ids.front()))) {
            continue;
        }
        if (!allowed[static_cast<std::size_t>(arc.u)]
            || !allowed[static_cast<std::size_t>(arc.v)]
            || !arc_unit_eligible(graph, arc, parent.source_unit_mask)) {
            continue;
        }
        candidate_arcs.push_back(arc_id);
        forward[static_cast<std::size_t>(arc.u)].push_back(arc.v);
        reverse[static_cast<std::size_t>(arc.v)].push_back(arc.u);
    }

    auto reachable_from_a = std::Vector<bool>(graph.nodes.size(), false);
    auto queue = std::queue<int> {};
    if (allowed[static_cast<std::size_t>(segment.endpoint_a)]) {
        reachable_from_a[static_cast<std::size_t>(segment.endpoint_a)] = true;
        queue.push(segment.endpoint_a);
    }
    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        for (const int next : forward[static_cast<std::size_t>(node)]) {
            if (!reachable_from_a[static_cast<std::size_t>(next)]) {
                reachable_from_a[static_cast<std::size_t>(next)] = true;
                queue.push(next);
            }
        }
    }

    auto can_reach_b = std::Vector<bool>(graph.nodes.size(), false);
    if (allowed[static_cast<std::size_t>(segment.endpoint_b)]) {
        can_reach_b[static_cast<std::size_t>(segment.endpoint_b)] = true;
        queue.push(segment.endpoint_b);
    }
    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        for (const int previous : reverse[static_cast<std::size_t>(node)]) {
            if (!can_reach_b[static_cast<std::size_t>(previous)]) {
                can_reach_b[static_cast<std::size_t>(previous)] = true;
                queue.push(previous);
            }
        }
    }

    auto out = V15SegmentScope {};
    for (const int node : raw_scope.node_ids) {
        if (reachable_from_a[static_cast<std::size_t>(node)]
            && can_reach_b[static_cast<std::size_t>(node)]) {
            out.node_ids.push_back(node);
        }
    }
    for (const int arc_id : candidate_arcs) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        if (reachable_from_a[static_cast<std::size_t>(arc.u)]
            && can_reach_b[static_cast<std::size_t>(arc.u)]
            && reachable_from_a[static_cast<std::size_t>(arc.v)]
            && can_reach_b[static_cast<std::size_t>(arc.v)]) {
            out.arc_ids.push_back(arc_id);
        }
    }
    return out;
}

} // namespace PR_tool
