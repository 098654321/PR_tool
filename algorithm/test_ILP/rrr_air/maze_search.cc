#include "maze_search.hh"

#include <algorithm>
#include <format>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace PR_tool::air316 {

namespace {

struct HeapItem {
    double cost{0};
    int seq{0};
    int node{0};
    int unit{-1};
};

struct HeapCompare {
    auto operator()(const HeapItem& lhs, const HeapItem& rhs) const -> bool {
        if (lhs.cost != rhs.cost) {
            return lhs.cost > rhs.cost;
        }
        return lhs.seq > rhs.seq;
    }
};

auto node_kind(const UnifiedGraph& graph, int node_id) -> UnifiedNodeKind {
    return graph.nodes[static_cast<std::size_t>(node_id)].kind;
}

auto valid_node(const UnifiedGraph& graph, int node_id) -> bool {
    return node_id >= 0 && node_id < static_cast<int>(graph.nodes.size());
}

auto is_vline_track(const UnifiedGraph& graph, const UnifiedArc& arc) -> bool {
    if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack) {
        return true;
    }
    const auto ku = node_kind(graph, arc.u);
    const auto kv = node_kind(graph, arc.v);
    return (ku == UnifiedNodeKind::VLine && kv == UnifiedNodeKind::Track)
        || (ku == UnifiedNodeKind::Track && kv == UnifiedNodeKind::VLine);
}

auto track_endpoint(const UnifiedGraph& graph, const UnifiedArc& arc) -> int {
    if (node_kind(graph, arc.u) == UnifiedNodeKind::Track) {
        return arc.u;
    }
    if (node_kind(graph, arc.v) == UnifiedNodeKind::Track) {
        return arc.v;
    }
    return -1;
}

struct StartMembership {
    const std::Vector<int>* sources{nullptr};
    const std::Set<int>* tree{nullptr};
    const std::Set<int>* explicit_starts{nullptr};

    auto contains(const UnifiedGraph& graph, int node) const -> bool {
        if (explicit_starts != nullptr) {
            return explicit_starts->contains(node);
        }
        if (sources != nullptr && std::binary_search(sources->begin(), sources->end(), node)) {
            return true;
        }
        return tree != nullptr && valid_node(graph, node)
            && node_kind(graph, node) == UnifiedNodeKind::Track && tree->contains(node);
    }
};

auto push_key(std::Vector<ResourceKey>& keys, const ResourceKey& key) -> void {
    for (const auto& existing : keys) {
        if (existing == key) {
            return;
        }
    }
    keys.push_back(key);
}

auto arc_keys(const UnifiedGraph& graph, const UnifiedArc& arc) -> const ArcResourceKeys& {
    return cached_arc_resource_keys(graph, arc);
}

auto owner_holds_key(const ResourceModel& resources, OwnerId owner, const ResourceKey& key) -> bool {
    return resources.holds(owner, key);
}

auto predicted_owner_count(
    const ResourceModel& resources,
    OwnerId owner,
    const ResourceKey& key
) -> int {
    if (is_mux_port_key(key)) {
        const int peers = resources.mux_distinct_peers(key);
        if (resources.has_any_owner(key)) {
            return peers;
        }
        return peers + 1;
    }
    const int count = resources.occupancy_count(key);
    return resources.holds(owner, key) ? count : count + 1;
}

auto opposite_mode_key(const ResourceKey& mode_key) -> ResourceKey {
    if (mode_key.kind == ResourceKind::ModeStraight) {
        return mode_swap_key(mode_key.id);
    }
    return mode_straight_key(mode_key.id);
}

auto would_introduce_opposite_mode(
    const ResourceModel& resources,
    OwnerId owner,
    const ResourceKey& mode_key
) -> bool {
    if (mode_key.kind != ResourceKind::ModeStraight && mode_key.kind != ResourceKind::ModeSwap) {
        return false;
    }
    if (owner_holds_key(resources, owner, mode_key)) {
        return false;
    }
    return resources.has_any_owner(opposite_mode_key(mode_key));
}

auto arc_cost(
    const UnifiedGraph& graph, const ResourceModel& resources, OwnerId owner,
    const UnifiedArc& arc, const StartMembership& /*starts*/
) -> double {
    // Match 0d6e86f test_ILP: no extra free-node rule for the search start set.
    return resources.arc_cost(graph, owner, arc);
}

auto find_out_arc(const UnifiedGraph& graph, int u, int v) -> int {
    for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        if (arc.v == v) {
            return arc_id;
        }
    }
    return -1;
}

auto ordered_out_arc_ids(
    const UnifiedGraph& graph,
    int node_id,
    hardware::Interposer* interposer
) -> const std::Vector<int>& {
    return cached_track_out_arc_ids(graph, node_id, interposer);
}

auto arc_is_hard_blocked(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const UnifiedArc& arc,
    const std::Set<ResourceKey>& hard_block
) -> bool {
    for (const auto& key : arc_keys(graph, arc)) {
        if (resources.mux_has_other_peer(owner, key)) {
            return true;
        }
        if (hard_block.empty()) {
            continue;
        }
        if (owner_holds_key(resources, owner, key)) {
            continue;
        }
        if (hard_block.contains(key)) {
            return true;
        }
    }
    return false;
}

auto violates_capacity(const ResourceModel& resources, OwnerId owner,
                       const UnifiedGraph& graph, const UnifiedArc& arc) -> bool {
    for (const auto& key : arc_keys(graph, arc)) {
        if (is_physical_occupancy_key(key) && !resources.holds(owner, key)
            && predicted_owner_count(resources, owner, key) > 1) return true;
        if (would_introduce_opposite_mode(resources, owner, key)) return true;
    }
    return false;
}

constexpr int kBnetStateCount = 17;

struct DenseSearchScratch {
    std::Vector<double> dist;
    std::Vector<int> parent;
    std::Vector<unsigned int> dist_epoch;
    std::Vector<unsigned int> closed_epoch;
    unsigned int epoch{0};

    auto begin(std::size_t state_count) -> void {
        if (dist.size() != state_count) {
            dist.resize(state_count);
            parent.resize(state_count);
            dist_epoch.assign(state_count, 0);
            closed_epoch.assign(state_count, 0);
            epoch = 0;
        }
        ++epoch;
        if (epoch == 0) {
            std::fill(dist_epoch.begin(), dist_epoch.end(), 0);
            std::fill(closed_epoch.begin(), closed_epoch.end(), 0);
            epoch = 1;
        }
    }

    auto has_distance(std::size_t id) const -> bool { return dist_epoch[id] == epoch; }
    auto is_closed(std::size_t id) const -> bool { return closed_epoch[id] == epoch; }
};

auto dense_search_scratch() -> DenseSearchScratch& {
    static thread_local DenseSearchScratch scratch;
    return scratch;
}

auto state_id(int node, int unit) -> std::size_t {
    return static_cast<std::size_t>(node) * kBnetStateCount + static_cast<std::size_t>(unit + 1);
}

auto reconstruct_path(const DenseSearchScratch& scratch, std::size_t sink_state) -> std::Vector<int> {
    auto path = std::Vector<int> {};
    auto current = static_cast<int>(sink_state);
    while (current >= 0) {
        path.push_back(current / kBnetStateCount);
        const int parent = scratch.parent[static_cast<std::size_t>(current)];
        if (parent < 0) {
            break;
        }
        current = parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace

auto arc_resource_keys(const UnifiedGraph& graph, const UnifiedArc& arc) -> const ArcResourceKeys& {
    return arc_keys(graph, arc);
}

auto arc_capacity_blocked(const UnifiedGraph& graph, const ResourceModel& resources,
                          OwnerId owner, const UnifiedArc& arc) -> bool {
    return violates_capacity(resources, owner, graph, arc);
}

auto path_resource_keys(
    const UnifiedGraph& graph,
    const std::Vector<int>& node_path,
    bool is_bnet
) -> std::Vector<ResourceKey> {
    auto keys = std::Vector<ResourceKey> {};
    if (node_path.empty()) {
        return keys;
    }
    for (const int node_id : node_path) {
        if (valid_node(graph, node_id)) {
            push_key(keys, node_resource(node_id));
        }
    }
    for (std::size_t i = 0; i + 1 < node_path.size(); ++i) {
        const int arc_id = find_out_arc(graph, node_path[i], node_path[i + 1]);
        if (arc_id < 0) {
            continue;
        }
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        for (const auto& key : arc_keys(graph, arc)) {
            push_key(keys, key);
        }
        if (!is_bnet || !is_vline_track(graph, arc)) {
            continue;
        }
        const int track_node = track_endpoint(graph, arc);
        if (track_node >= 0) {
            push_key(
                keys,
                bnet_unit_key(static_cast<int>(graph.nodes[static_cast<std::size_t>(track_node)].unit)));
        }
    }
    return keys;
}

auto maze_ordered_out_arc_ids(
    const UnifiedGraph& graph,
    int node_id,
    hardware::Interposer* interposer
) -> const std::Vector<int>& {
    return ordered_out_arc_ids(graph, node_id, interposer);
}

auto arc_incremental_cost(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const UnifiedArc& arc,
    const std::Set<int>& starts,
    const RrrParams& params
) -> double {
    const auto start_membership = StartMembership {.explicit_starts = &starts};
    return arc_cost(graph, resources, owner, arc, start_membership);
}

auto route_demand(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const std::Vector<int>& sources,
    int sink,
    const RrrParams& params,
    const std::Vector<int>& tree,
    bool is_bnet,
    hardware::Interposer* interposer,
    const std::Set<ResourceKey>& hard_block
) -> std::Vector<int> {
    auto tree_set = std::Set<int> {};
    for (const int node : tree) {
        if (valid_node(graph, node) && node_kind(graph, node) == UnifiedNodeKind::Track) {
            tree_set.insert(node);
        }
    }
    return route_demand_from_tree(
        graph, resources, owner, sources, sink, params, tree_set, is_bnet, interposer, hard_block);
}

auto route_demand_from_tree(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const std::Vector<int>& sources,
    int sink,
    const RrrParams& params,
    const std::Set<int>& tree,
    bool is_bnet,
    hardware::Interposer* interposer,
    const std::Set<ResourceKey>& hard_block
) -> std::Vector<int> {
    auto ordered_sources = std::Vector<int> {};
    ordered_sources.reserve(sources.size());
    for (const int node : sources) {
        if (valid_node(graph, node)) {
            ordered_sources.push_back(node);
        }
    }
    std::sort(ordered_sources.begin(), ordered_sources.end());
    ordered_sources.erase(std::unique(ordered_sources.begin(), ordered_sources.end()), ordered_sources.end());
    if (ordered_sources.empty() && tree.empty()) {
        throw std::runtime_error(
            std::format("maze: sink {} unreachable from given sources", sink));
    }
    if (!valid_node(graph, sink)) {
        throw std::runtime_error(
            std::format("maze: sink {} unreachable from given sources", sink));
    }

    const int initial_unit = is_bnet ? resources.selected_unit(owner) : -1;
    const auto start_membership = StartMembership {.sources = &ordered_sources, .tree = &tree};
    auto& scratch = dense_search_scratch();
    scratch.begin(graph.nodes.size() * kBnetStateCount);
    std::priority_queue<HeapItem, std::Vector<HeapItem>, HeapCompare> heap;
    int seq = 0;

    const auto push_start = [&](int node) {
        if (!node_in_search_box(graph, node, params)) return;
        const auto state = state_id(node, initial_unit);
        if (scratch.has_distance(state)) {
            return;
        }
        scratch.dist_epoch[state] = scratch.epoch;
        scratch.dist[state] = 0;
        scratch.parent[state] = -1;
        heap.push(HeapItem {0, seq++, node, initial_unit});
    };
    auto source_it = ordered_sources.begin();
    auto tree_it = tree.begin();
    while (source_it != ordered_sources.end() || tree_it != tree.end()) {
        while (tree_it != tree.end()
            && (!valid_node(graph, *tree_it) || node_kind(graph, *tree_it) != UnifiedNodeKind::Track)) {
            ++tree_it;
        }
        if (tree_it == tree.end() || (source_it != ordered_sources.end() && *source_it < *tree_it)) {
            push_start(*source_it++);
        } else if (source_it == ordered_sources.end() || *tree_it < *source_it) {
            push_start(*tree_it++);
        } else {
            push_start(*source_it++);
            ++tree_it;
        }
    }

    while (!heap.empty()) {
        const auto item = heap.top();
        heap.pop();
        const auto state = state_id(item.node, item.unit);
        if (!scratch.has_distance(state) || item.cost > scratch.dist[state]) {
            continue;
        }
        if (scratch.is_closed(state)) {
            continue;
        }
        scratch.closed_epoch[state] = scratch.epoch;
        if (item.node == sink) {
            return reconstruct_path(scratch, state);
        }

        for (const int arc_id : ordered_out_arc_ids(graph, item.node, interposer)) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!valid_node(graph, arc.v)) {
                continue;
            }
            if (!node_in_search_box(graph, arc.v, params)) continue;
            if (node_kind(graph, arc.v) == UnifiedNodeKind::Bump && arc.v != sink) continue;
            if (arc_is_hard_blocked(graph, resources, owner, arc, hard_block)) {
                continue;
            }
            if (params.hard_capacity && violates_capacity(resources, owner, graph, arc)) continue;
            int next_unit = item.unit;
            if (is_bnet && is_vline_track(graph, arc)) {
                const int track_node = track_endpoint(graph, arc);
                const int unit = track_node >= 0
                    ? static_cast<int>(graph.nodes[static_cast<std::size_t>(track_node)].unit)
                    : -1;
                const int locked = item.unit >= 0 ? item.unit : initial_unit;
                if (locked >= 0 && unit != locked) {
                    continue;
                }
                next_unit = locked >= 0 ? locked : unit;
            }

            const double step = arc_cost(
                graph,
                resources,
                owner,
                arc,
                start_membership);
            const double next_cost = item.cost + step;
            const auto next = state_id(arc.v, next_unit);
            if (!scratch.has_distance(next) || next_cost < scratch.dist[next]) {
                scratch.dist_epoch[next] = scratch.epoch;
                scratch.dist[next] = next_cost;
                scratch.parent[next] = static_cast<int>(state);
                heap.push(HeapItem {next_cost, seq++, arc.v, next_unit});
            }
        }
    }

    throw std::runtime_error(
        std::format("maze: sink {} unreachable from given sources", sink));
}

} // namespace PR_tool::air316
