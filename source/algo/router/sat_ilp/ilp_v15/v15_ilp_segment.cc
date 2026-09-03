#include "ilp_v15/v15_ilp_segment.hh"

#include "common/cob_unit_mask.hh"
#include "common/hw_map.hh"
#include "ilp_v15/v15_ilp_domain.hh"
#include "scope/scope_bbox.hh"

#include <algorithm>
#include <format>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>

namespace PR_tool {

namespace {

auto scope_index_for_net(
    const std::Vector<UnifiedSatNetScope>& scopes,
    std::size_t net_id
) -> std::size_t {
    const auto it = std::find_if(
        scopes.begin(),
        scopes.end(),
        [&](const UnifiedSatNetScope& scope) { return scope.net_id == net_id; });
    if (it == scopes.end()) {
        throw std::invalid_argument(std::format("v15 net {} has no SAT scope", net_id));
    }
    return static_cast<std::size_t>(std::distance(scopes.begin(), it));
}

auto source_node_for(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const RoutingDemand& demand
) -> std::pair<int, std::size_t> {
    if (net.kind == RoutingNetKind::PNnet) {
        if (net.virtual_source_node < 0) {
            throw std::invalid_argument(std::format("v15 PNnet {} has no virtual root", net.net_id));
        }
        return {net.virtual_source_node, 0};
    }
    if (demand.candidate_source_indices.size() != 1) {
        throw std::invalid_argument(std::format(
            "v15 net {} demand {} requires one physical source",
            net.net_id,
            demand.demand_id));
    }
    const auto source_index = demand.candidate_source_indices.front();
    const int node = resolve_graph_node(graph, net.sources[source_index]);
    if (node < 0) {
        throw std::invalid_argument(std::format(
            "v15 net {} demand {} source is unresolved",
            net.net_id,
            demand.demand_id));
    }
    return {node, source_index};
}

auto sink_node_for(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const RoutingDemand& demand
) -> int {
    const int node = resolve_graph_node(graph, demand.sink);
    if (node < 0) {
        throw std::invalid_argument(std::format(
            "v15 net {} demand {} sink is unresolved",
            net.net_id,
            demand.demand_id));
    }
    return node;
}

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

auto path_arc_ids(const UnifiedGraph& graph, const std::Vector<int>& path) -> std::Vector<int> {
    auto arc_ids = std::Vector<int> {};
    for (std::size_t i = 1; i < path.size(); ++i) {
        const int arc_id = find_arc_id(graph, path[i - 1], path[i]);
        if (arc_id < 0) {
            throw V15PreparationInvariantError(std::format(
                "v15 segment guide path has a missing arc {}->{}",
                path[i - 1],
                path[i]));
        }
        arc_ids.push_back(arc_id);
    }
    return arc_ids;
}

auto is_physical_node(const UnifiedGraph& graph, int node) -> bool {
    return node >= 0
        && static_cast<std::size_t>(node) < graph.nodes.size()
        && graph.nodes[static_cast<std::size_t>(node)].kind != UnifiedNodeKind::VirtualSource;
}

auto merge_node_into_bbox(const UnifiedGraph& graph, int node, IlpBoundingBox& box) -> void {
    if (!is_physical_node(graph, node)) {
        return;
    }
    const auto& unified = graph.nodes[static_cast<std::size_t>(node)];
    switch (unified.kind) {
        case UnifiedNodeKind::Track:
            merge_coord_into_bbox(box, track_to_cob(hardware::TrackCoord {
                unified.track_row,
                unified.track_col,
                unified.track_dir == 0
                    ? hardware::TrackDirection::Horizontal
                    : hardware::TrackDirection::Vertical,
                unified.track_index}));
            break;
        case UnifiedNodeKind::Bump:
            merge_coord_into_bbox(box, tob_anchor_cob(unified.bump.TOB));
            break;
        case UnifiedNodeKind::HLine:
        case UnifiedNodeKind::VLine: {
            const auto [tob_row, tob_col] = tob_index_from_linear(unified.tob);
            const auto [cob0, cob1] = tob_pair_cob_coords(tob_row, tob_col);
            merge_coord_into_bbox(box, cob0);
            merge_coord_into_bbox(box, cob1);
            break;
        }
        case UnifiedNodeKind::VirtualSource:
            break;
    }
}

auto bbox_from_guide(
    const UnifiedGraph& graph,
    const std::Vector<int>& guide,
    int pad
) -> IlpBoundingBox {
    auto box = IlpBoundingBox {
        std::numeric_limits<std::i64>::max(),
        std::numeric_limits<std::i64>::min(),
        std::numeric_limits<std::i64>::max(),
        std::numeric_limits<std::i64>::min()};
    for (const int node : guide) {
        merge_node_into_bbox(graph, node, box);
    }
    if (box.row_min > box.row_max) {
        box = full_chip_bbox();
    }
    for (int i = 0; i < pad; ++i) {
        box = expand_pair_bbox_one_cell(box);
    }
    return clamp_bbox_to_cob_array(box);
}

auto collect_sat_paths_for_net(
    const SatRoutingResult& sat_result,
    std::size_t net_id
) -> std::Vector<SourceSinkPairPath> {
    auto out = std::Vector<SourceSinkPairPath> {};
    for (const auto& path : sat_result.paths) {
        if (path.net_id == net_id) {
            out.push_back(path);
        }
    }
    return out;
}

auto build_rooted_tree(
    const std::Vector<std::Vector<int>>& paths,
    int root,
    const std::set<int>& origin_sinks
) -> std::pair<std::map<int, int>, std::set<std::pair<int, int>>> {
    auto undirected = std::map<int, std::set<int>> {};
    for (const auto& path : paths) {
        for (std::size_t i = 1; i < path.size(); ++i) {
            const int u = path[i - 1];
            const int v = path[i];
            undirected[u].insert(v);
            undirected[v].insert(u);
        }
    }

    auto parent = std::map<int, int> {};
    auto visited = std::set<int> {root};
    auto queue = std::queue<int> {};
    queue.push(root);
    parent[root] = root;
    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        const auto it = undirected.find(node);
        if (it == undirected.end()) {
            continue;
        }
        for (const int next : it->second) {
            if (!visited.insert(next).second) {
                continue;
            }
            parent[next] = node;
            queue.push(next);
        }
    }

    // The BFS parent relation, not the original path union, defines the
    // recovered tree.  Mark exactly the parent chains required by original
    // sinks so remerging branches discarded by BFS cannot leak into segments.
    auto retained = std::set<int> {root};
    for (const int sink : origin_sinks) {
        if (!parent.contains(sink)) {
            throw V15PreparationInvariantError(std::format(
                "v15 recovered tree cannot reach original sink {} from root {}", sink, root));
        }
        int current = sink;
        while (current != root) {
            retained.insert(current);
            const auto parent_it = parent.find(current);
            if (parent_it == parent.end() || parent_it->second == current) {
                throw V15PreparationInvariantError(std::format(
                    "v15 recovered tree has no parent chain from sink {} to root {}", sink, root));
            }
            current = parent_it->second;
        }
    }

    auto tree_edges = std::set<std::pair<int, int>> {};
    for (const auto& [node, p] : parent) {
        if (node == root || !retained.contains(node)) {
            continue;
        }
        tree_edges.insert({p, node});
    }
    auto pruned_parent = std::map<int, int> {};
    pruned_parent[root] = root;
    for (const auto& [u, v] : tree_edges) {
        pruned_parent[v] = u;
    }
    return {pruned_parent, tree_edges};
}

auto undirected_degree(
    const std::set<std::pair<int, int>>& tree_edges
) -> std::map<int, int> {
    auto degree = std::map<int, int> {};
    for (const auto& [u, v] : tree_edges) {
        ++degree[u];
        ++degree[v];
    }
    return degree;
}

auto tree_children(
    int root,
    const std::set<std::pair<int, int>>& tree_edges
) -> std::map<int, std::Vector<int>> {
    auto children = std::map<int, std::Vector<int>> {};
    for (const auto& [u, v] : tree_edges) {
        children[u].push_back(v);
    }
    for (auto& [node, child_list] : children) {
        (void)node;
        std::sort(child_list.begin(), child_list.end());
    }
    (void)root;
    return children;
}

auto emit_segments_from_tree(
    int root,
    const std::set<int>& critical,
    const std::map<int, std::Vector<int>>& children,
    std::Vector<std::tuple<int, int, std::Vector<int>>>& out
) -> void {
    const auto emit_from = [&](int start, auto&& self) -> void {
        const auto child_it = children.find(start);
        if (child_it == children.end()) {
            return;
        }
        for (const int first : child_it->second) {
            auto path = std::Vector<int> {start};
            int current = first;
            path.push_back(current);
            while (!critical.contains(current)) {
                const auto next_it = children.find(current);
                if (next_it == children.end() || next_it->second.empty()) {
                    throw V15PreparationInvariantError(std::format(
                        "v15 recovered tree reached non-critical leaf {} from segment start {}",
                        current,
                        start));
                }
                if (next_it->second.size() != 1) {
                    throw V15PreparationInvariantError(std::format(
                        "v15 recovered tree has non-critical branch node {}", current));
                }
                current = next_it->second.front();
                path.push_back(current);
            }
            out.emplace_back(start, current, path);
            if (children.contains(current)) {
                self(current, self);
            }
        }
    };
    emit_from(root, emit_from);
}

auto scope_from_sat_net(
    const UnifiedSatNetScope& scope
) -> V15SegmentScope {
    return V15SegmentScope {scope.node_ids, scope.arc_ids};
}

auto selected_track_unit_mask(
    const UnifiedGraph& graph,
    const std::Vector<int>& tracks
) -> std::uint16_t {
    auto mask = std::uint16_t {0};
    for (const int track : tracks) {
        if (track < 0 || static_cast<std::size_t>(track) >= graph.nodes.size()
            || graph.nodes[static_cast<std::size_t>(track)].kind != UnifiedNodeKind::Track) {
            throw V15PreparationInvariantError(std::format(
                "v15 PN selected source {} is not a physical track", track));
        }
        mask = static_cast<std::uint16_t>(
            mask | unit_bit(graph.nodes[static_cast<std::size_t>(track)].unit));
    }
    if (mask == 0) {
        throw V15PreparationInvariantError("v15 PN has no SAT-selected track unit");
    }
    return mask;
}

auto count_union_nodes(const std::Vector<std::Vector<int>>& paths) -> std::size_t {
    auto nodes = std::set<int> {};
    for (const auto& path : paths) {
        nodes.insert(path.begin(), path.end());
    }
    return nodes.size();
}

auto count_union_edges(const std::Vector<std::Vector<int>>& paths) -> std::size_t {
    auto edges = std::set<std::pair<int, int>> {};
    for (const auto& path : paths) {
        for (std::size_t i = 1; i < path.size(); ++i) {
            edges.emplace(path[i - 1], path[i]);
        }
    }
    return edges.size();
}

auto ensure_guide_is_covered(
    const V15Segment& segment,
    std::size_t routing_net_id
) -> void {
    const auto node_ids = std::set<int>(segment.scope.node_ids.begin(), segment.scope.node_ids.end());
    const auto arc_ids = std::set<int>(segment.scope.arc_ids.begin(), segment.scope.arc_ids.end());
    for (const int node : segment.guide_node_path) {
        if (!node_ids.contains(node)) {
            throw V15PreparationInvariantError(std::format(
                "v15 guide coverage failed: net={} parent={} segment={} missing node {}",
                routing_net_id,
                segment.parent_id,
                segment.segment_id,
                node));
        }
    }
    for (const int arc_id : segment.guide_arc_ids) {
        if (!arc_ids.contains(arc_id)) {
            throw V15PreparationInvariantError(std::format(
                "v15 guide coverage failed: net={} parent={} segment={} missing arc {}",
                routing_net_id,
                segment.parent_id,
                segment.segment_id,
                arc_id));
        }
    }
}

} // namespace

auto build_v15_parents_and_segments(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const SatRoutingResult& sat_result,
    const std::set<std::size_t>& selected_net_ids,
    int segment_bbox_pad,
    const V15LockedResources& locked
) -> V15PrepareResult {
    auto result = V15PrepareResult {};
    for (const auto& net : nets) {
        if (!selected_net_ids.contains(net.net_id)) {
            continue;
        }
        const auto scope_index = scope_index_for_net(scopes, net.net_id);
        const auto& sat_scope = scopes[scope_index];

        if (net.is_sync_bus) {
            for (const auto& demand : net.demands) {
                const auto [root, source_index] = source_node_for(graph, net, demand);
                const int sink = sink_node_for(graph, net, demand);
                auto parent = V15Parent {};
                parent.parent_id = result.parents.size();
                parent.routing_net_id = net.net_id;
                parent.scope_index = scope_index;
                parent.root_node = root;
                parent.origin_sinks = {sink};
                parent.demand_ids = {demand.demand_id};
                parent.source_indices = {source_index};
                parent.is_bus_member = true;
                parent.decomposed = false;
                parent.source_unit_mask = compute_source_unit_mask(graph, net, root);

                auto guide = std::Vector<int> {};
                for (const auto& path : sat_result.paths) {
                    if (path.net_id == net.net_id && path.demand_id == demand.demand_id) {
                        guide = path.node_path;
                        break;
                    }
                }
                if (guide.empty()) {
                    throw std::invalid_argument(std::format(
                        "v15 bus net {} demand {} missing SAT guide",
                        net.net_id,
                        demand.demand_id));
                }

                auto segment = V15Segment {};
                segment.segment_id = result.segments.size();
                segment.parent_id = parent.parent_id;
                segment.endpoint_a = root;
                segment.endpoint_b = sink;
                segment.guide_node_path = guide;
                segment.guide_arc_ids = path_arc_ids(graph, guide);
                segment.is_virtual_hop = false;
                const auto raw_scope = scope_from_sat_net(sat_scope);
                segment.scope = refine_v15_segment_scope(graph, raw_scope, parent, segment, locked);
                ensure_guide_is_covered(segment, net.net_id);

                parent.segment_ids.push_back(segment.segment_id);
                result.parents.push_back(std::move(parent));
                result.segments.push_back(std::move(segment));
            }
            continue;
        }

        if (net.demands.empty()) {
            throw std::invalid_argument(std::format("v15 net {} has no demand", net.net_id));
        }

        const auto [root, first_source_index] = source_node_for(graph, net, net.demands.front());
        auto origin_sinks = std::Vector<int> {};
        auto demand_ids = std::Vector<std::size_t> {};
        auto source_indices = std::Vector<std::size_t> {};
        for (const auto& demand : net.demands) {
            const auto [candidate_source, source_index] = source_node_for(graph, net, demand);
            if (candidate_source != root) {
                throw std::invalid_argument(std::format(
                    "v15 non-bus net {} has multiple physical roots",
                    net.net_id));
            }
            origin_sinks.push_back(sink_node_for(graph, net, demand));
            demand_ids.push_back(demand.demand_id);
            source_indices.push_back(
                net.kind == RoutingNetKind::PNnet ? 0 : source_index);
        }
        (void)first_source_index;

        const bool needs_decompose =
            net.kind == RoutingNetKind::PNnet || origin_sinks.size() > 1;

        if (!needs_decompose) {
            auto parent = V15Parent {};
            parent.parent_id = result.parents.size();
            parent.routing_net_id = net.net_id;
            parent.scope_index = scope_index;
            parent.root_node = root;
            parent.origin_sinks = origin_sinks;
            parent.demand_ids = demand_ids;
            parent.source_indices = source_indices;
            parent.decomposed = false;
            parent.source_unit_mask = compute_source_unit_mask(graph, net, root);

            auto guide = std::Vector<int> {};
            for (const auto& path : sat_result.paths) {
                if (path.net_id == net.net_id && path.demand_id == demand_ids.front()) {
                    guide = path.node_path;
                    break;
                }
            }
            if (guide.empty()) {
                throw std::invalid_argument(std::format(
                    "v15 net {} missing SAT guide path",
                    net.net_id));
            }

            auto segment = V15Segment {};
            segment.segment_id = result.segments.size();
            segment.parent_id = parent.parent_id;
            segment.endpoint_a = root;
            segment.endpoint_b = origin_sinks.front();
            segment.guide_node_path = guide;
            segment.guide_arc_ids = path_arc_ids(graph, guide);
            segment.is_virtual_hop = false;
            const auto raw_scope = scope_from_sat_net(sat_scope);
            segment.scope = refine_v15_segment_scope(graph, raw_scope, parent, segment, locked);
            ensure_guide_is_covered(segment, net.net_id);

            parent.segment_ids.push_back(segment.segment_id);
            result.parents.push_back(std::move(parent));
            result.segments.push_back(std::move(segment));
            continue;
        }

        auto parent = V15Parent {};
        parent.parent_id = result.parents.size();
        parent.routing_net_id = net.net_id;
        parent.scope_index = scope_index;
        parent.root_node = root;
        parent.origin_sinks = origin_sinks;
        parent.demand_ids = demand_ids;
        parent.source_indices = source_indices;
        parent.decomposed = true;
        parent.source_unit_mask = compute_source_unit_mask(graph, net, root);

        auto path_lists = std::Vector<std::Vector<int>> {};
        auto selected_tracks = std::set<int> {};
        const auto sat_paths = collect_sat_paths_for_net(sat_result, net.net_id);
        for (const auto& path : sat_paths) {
            auto nodes = path.node_path;
            if (net.kind == RoutingNetKind::PNnet) {
                const int fixed_track = path.physical_source_node;
                if (fixed_track < 0) {
                    throw std::invalid_argument(std::format(
                        "v15 PNnet {} path missing physical track",
                        net.net_id));
                }
                selected_tracks.insert(fixed_track);
                if (nodes.empty() || nodes.front() != root) {
                    nodes.insert(nodes.begin(), root);
                }
                if (nodes.size() < 2 || nodes[1] != fixed_track) {
                    nodes.insert(nodes.begin() + 1, fixed_track);
                }
            }
            path_lists.push_back(std::move(nodes));
        }
        if (path_lists.empty()) {
            throw V15PreparationInvariantError(std::format(
                "v15 net {} has no SAT guide paths", net.net_id));
        }
        parent.fixed_track_nodes.assign(selected_tracks.begin(), selected_tracks.end());
        if (net.kind == RoutingNetKind::PNnet) {
            parent.source_unit_mask = selected_track_unit_mask(graph, parent.fixed_track_nodes);
        }
        parent.tree_nodes_before_prune = count_union_nodes(path_lists);
        parent.tree_edges_before_prune = count_union_edges(path_lists);

        const auto sink_set = std::set<int>(origin_sinks.begin(), origin_sinks.end());
        const auto [parent_map, tree_edges] = build_rooted_tree(path_lists, root, sink_set);
        if (net.kind == RoutingNetKind::PNnet) {
            for (const int track : parent.fixed_track_nodes) {
                if (!parent_map.contains(track)) {
                    throw V15PreparationInvariantError(std::format(
                        "v15 PNnet {} selected track {} is removed by BFS remerge pruning; "
                        "the parent-tree model cannot preserve this source-to-sink branch",
                        net.net_id,
                        track));
                }
            }
        }
        parent.tree_nodes_after_prune = parent_map.size();
        parent.tree_edges_after_prune = tree_edges.size();
        const auto degree = undirected_degree(tree_edges);
        auto critical = std::set<int> {root};
        critical.insert(origin_sinks.begin(), origin_sinks.end());
        critical.insert(parent.fixed_track_nodes.begin(), parent.fixed_track_nodes.end());
        for (const auto& [node, deg] : degree) {
            if (deg >= 3) {
                critical.insert(node);
            }
        }

        auto segment_specs = std::Vector<std::tuple<int, int, std::Vector<int>>> {};
        const auto children = tree_children(root, tree_edges);
        emit_segments_from_tree(root, critical, children, segment_specs);

        if (net.kind == RoutingNetKind::PNnet) {
            segment_specs.erase(
                std::remove_if(
                    segment_specs.begin(),
                    segment_specs.end(),
                    [&](const auto& spec) {
                        return std::get<0>(spec) == root
                            && selected_tracks.contains(std::get<1>(spec));
                    }),
                segment_specs.end());
            for (auto it = parent.fixed_track_nodes.rbegin(); it != parent.fixed_track_nodes.rend(); ++it) {
                const int arc_id = find_arc_id(graph, root, *it);
                if (arc_id < 0) {
                    throw V15PreparationInvariantError(std::format(
                        "v15 PNnet {} selected track {} has no virtual root arc",
                        net.net_id,
                        *it));
                }
                segment_specs.insert(
                    segment_specs.begin(),
                    std::make_tuple(root, *it, std::Vector<int>({root, *it})));
            }
        }

        for (const auto& [a, b, guide] : segment_specs) {
            auto segment = V15Segment {};
            segment.segment_id = result.segments.size();
            segment.parent_id = parent.parent_id;
            segment.endpoint_a = a;
            segment.endpoint_b = b;
            segment.guide_node_path = guide;
            segment.guide_arc_ids = path_arc_ids(graph, guide);
            segment.is_virtual_hop =
                net.kind == RoutingNetKind::PNnet
                && a == root
                && selected_tracks.contains(b)
                && guide.size() == 2;

            V15SegmentScope raw_scope {};
            if (segment.is_virtual_hop) {
                raw_scope.node_ids = {a, b};
                const int arc_id = find_arc_id(graph, a, b);
                if (arc_id >= 0) {
                    raw_scope.arc_ids.push_back(arc_id);
                }
            }
            else {
                const auto bbox = bbox_from_guide(graph, guide, segment_bbox_pad);
                raw_scope = build_v15_scope_from_bbox(graph, bbox, {a, b});
            }
            segment.scope = refine_v15_segment_scope(graph, raw_scope, parent, segment, locked);
            ensure_guide_is_covered(segment, net.net_id);
            parent.segment_ids.push_back(segment.segment_id);
            result.segments.push_back(std::move(segment));
        }

        result.parents.push_back(std::move(parent));
    }
    return result;
}

} // namespace PR_tool
