#include "maze_search.hh"

#include "hw_map.hh"

#include <hardware/cob/cob.hh>
#include <hardware/cob/cobdirection.hh>
#include <hardware/track/track.hh>
#include <hardware/track/trackcoord.hh>

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace PR_tool {

namespace {

struct SearchState {
    int node{-1};
    int unit{-1};

    auto operator<(const SearchState& other) const -> bool {
        return std::tie(node, unit) < std::tie(other.node, other.unit);
    }
};

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

auto push_key(std::Vector<ResourceKey>& keys, const ResourceKey& key) -> void {
    for (const auto& existing : keys) {
        if (existing == key) {
            return;
        }
    }
    keys.push_back(key);
}

auto arc_keys(const UnifiedGraph& graph, const UnifiedArc& arc) -> std::Vector<ResourceKey> {
    auto keys = std::Vector<ResourceKey> {};
    const auto ku = node_kind(graph, arc.u);
    const auto kv = node_kind(graph, arc.v);
    push_key(keys, node_resource(arc.v));
    if (ku == UnifiedNodeKind::HLine || ku == UnifiedNodeKind::VLine) {
        push_key(keys, node_resource(arc.u));
    }

    if (arc.physical_switch_id >= 0) {
        push_key(keys, switch_resource(arc.physical_switch_id));
    }

    const int bump = ku == UnifiedNodeKind::Bump ? arc.u : (kv == UnifiedNodeKind::Bump ? arc.v : -1);
    const int hline = ku == UnifiedNodeKind::HLine ? arc.u : (kv == UnifiedNodeKind::HLine ? arc.v : -1);
    const int vline = ku == UnifiedNodeKind::VLine ? arc.u : (kv == UnifiedNodeKind::VLine ? arc.v : -1);

    if (bump >= 0 && hline >= 0) {
        push_key(keys, matching_endpoint_key(bump, 0));
        push_key(keys, matching_endpoint_key(hline, 1));
    }
    if (hline >= 0 && vline >= 0) {
        push_key(keys, matching_endpoint_key(hline, 2));
        push_key(keys, matching_endpoint_key(vline, 3));
    }

    if (is_vline_track(graph, arc) && arc.mode_group_id >= 0) {
        if (arc.is_vline_track_straight) {
            push_key(keys, mode_straight_key(arc.mode_group_id));
        } else if (arc.is_vline_track_swap) {
            push_key(keys, mode_swap_key(arc.mode_group_id));
        }
    }
    return keys;
}

auto owner_holds_key(const ResourceModel& resources, OwnerId owner, const ResourceKey& key) -> bool {
    for (const auto& item : resources.owners_of(key)) {
        if (item == owner) {
            return true;
        }
    }
    return false;
}

auto predicted_owner_count(
    const ResourceModel& resources,
    OwnerId owner,
    const ResourceKey& key
) -> int {
    int count = 0;
    bool has_owner = false;
    for (const auto& item : resources.owners_of(key)) {
        ++count;
        if (item == owner) {
            has_owner = true;
        }
    }
    if (has_owner) {
        return count;
    }
    return count + 1;
}

auto logistic_p(int u, const RrrParams& params) -> double {
    const int cap = 1;
    const double logistic =
        static_cast<double>(params.H) / (std::exp(params.k * static_cast<double>(cap - u)) + 1.0);
    double over = 0;
    if (u > cap) {
        over = static_cast<double>(params.H) / static_cast<double>(params.s) * static_cast<double>(u - cap);
    }
    return 1.0 + logistic + over;
}

auto key_is_free(
    const ResourceModel& resources,
    OwnerId owner,
    const ResourceKey& key,
    int from_node,
    const std::Set<int>& starts
) -> bool {
    if (owner_holds_key(resources, owner, key)) {
        return true;
    }
    if (key.kind == ResourceKind::Node && (key.id == from_node || starts.contains(key.id))) {
        return true;
    }
    return false;
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
    return !resources.owners_of(opposite_mode_key(mode_key)).empty();
}

auto key_incremental_cost(
    const ResourceModel& resources,
    OwnerId owner,
    const ResourceKey& key,
    const RrrParams& params
) -> double {
    const int u = predicted_owner_count(resources, owner, key);
    const double present = static_cast<double>(resources.type_weight(key)) * logistic_p(u, params);
    return present + params.history_weight * resources.history(key);
}

auto arc_cost(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const UnifiedArc& arc,
    const std::Set<int>& starts,
    const RrrParams& params,
    int row_min,
    int row_max,
    int col_min,
    int col_max,
    bool has_bbox
) -> double {
    double cost = 1.0;
    for (const auto& key : arc_keys(graph, arc)) {
        if (key_is_free(resources, owner, key, arc.u, starts)) {
            continue;
        }
        cost += key_incremental_cost(resources, owner, key, params);
        if (would_introduce_opposite_mode(resources, owner, key)) {
            const auto conflict = mode_conflict_key(key.id);
            cost += key_incremental_cost(resources, owner, conflict, params);
        }
    }
    if (params.detour_bias != 0 && has_bbox && node_kind(graph, arc.v) == UnifiedNodeKind::Track) {
        const auto& dest = graph.nodes[static_cast<std::size_t>(arc.v)];
        if (dest.track_row < row_min || dest.track_row > row_max || dest.track_col < col_min
            || dest.track_col > col_max) {
            cost += params.detour_bias;
        }
    }
    return cost;
}

auto cob_cardinal_rank(const UnifiedNode& track, const hardware::COBCoord& cob) -> int {
    if (cob.row < track.track_row) {
        return 0;
    }
    if (cob.col > track.track_col) {
        return 1;
    }
    if (cob.row > track.track_row) {
        return 2;
    }
    if (cob.col < track.track_col) {
        return 3;
    }
    return track.track_dir == 1 ? 2 : 1;
}

auto lookup_track_node(const UnifiedGraph& graph, const hardware::TrackCoord& coord) -> int {
    const std::size_t unit = map_track(coord.index);
    const int dir = coord.dir == hardware::TrackDirection::Horizontal ? 0 : 1;
    const auto it = graph.track_node_by_key.find(
        {unit, dir, static_cast<int>(coord.row), static_cast<int>(coord.col), coord.index});
    if (it == graph.track_node_by_key.end()) {
        return -1;
    }
    return it->second;
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
) -> std::Vector<int> {
    const auto& raw = graph.out_arc_ids[static_cast<std::size_t>(node_id)];
    if (interposer == nullptr || node_kind(graph, node_id) != UnifiedNodeKind::Track) {
        return raw;
    }
    const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
    const auto coord = hardware::TrackCoord {
        node.track_row,
        node.track_col,
        node.track_dir == 0 ? hardware::TrackDirection::Horizontal : hardware::TrackDirection::Vertical,
        node.track_index};
    const auto track_opt = interposer->get_track(coord);
    if (!track_opt.has_value()) {
        return raw;
    }
    auto* track = *track_opt;
    auto cobs = track->adjacent_cob_coords();
    std::sort(cobs.begin(), cobs.end(), [&](const auto& lhs, const auto& rhs) {
        const auto& cob_l = std::get<1>(lhs);
        const auto& cob_r = std::get<1>(rhs);
        const auto rank_l = cob_cardinal_rank(node, cob_l);
        const auto rank_r = cob_cardinal_rank(node, cob_r);
        if (rank_l != rank_r) {
            return rank_l < rank_r;
        }
        if (cob_l.row != cob_r.row) {
            return cob_l.row < cob_r.row;
        }
        return cob_l.col < cob_r.col;
    });

    auto ordered = std::Vector<int> {};
    auto seen = std::Set<int> {};
    for (const auto& [from_dir, cob_coord] : cobs) {
        const auto cob_opt = interposer->get_cob(cob_coord);
        if (!cob_opt.has_value()) {
            continue;
        }
        auto* cob = *cob_opt;
        for (auto& connector : cob->adjacent_connectors(from_dir, coord.index, cob_coord)) {
            const auto dest_coord = cob->to_dir_track_coord(connector.to_dir(), connector.to_track_index());
            const int dest = lookup_track_node(graph, dest_coord);
            if (dest < 0) {
                continue;
            }
            const int arc_id = find_out_arc(graph, node_id, dest);
            if (arc_id >= 0 && seen.insert(arc_id).second) {
                ordered.push_back(arc_id);
            }
        }
    }
    for (const int arc_id : raw) {
        if (seen.insert(arc_id).second) {
            ordered.push_back(arc_id);
        }
    }
    return ordered;
}

auto arc_is_hard_blocked(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const UnifiedArc& arc,
    const std::Set<ResourceKey>& hard_block
) -> bool {
    if (hard_block.empty()) {
        return false;
    }
    for (const auto& key : arc_keys(graph, arc)) {
        if (owner_holds_key(resources, owner, key)) {
            continue;
        }
        if (hard_block.contains(key)) {
            return true;
        }
    }
    return false;
}

auto reconstruct_path(
    const std::Map<SearchState, SearchState>& parent,
    SearchState sink
) -> std::Vector<int> {
    auto path = std::Vector<int> {};
    auto cur = sink;
    while (cur.node >= 0) {
        path.push_back(cur.node);
        const auto it = parent.find(cur);
        if (it == parent.end()) {
            break;
        }
        cur = it->second;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace

auto arc_resource_keys(const UnifiedGraph& graph, const UnifiedArc& arc) -> std::Vector<ResourceKey> {
    return arc_keys(graph, arc);
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
) -> std::Vector<int> {
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
    return arc_cost(graph, resources, owner, arc, starts, params, 0, 0, 0, 0, false);
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
    auto starts = std::Set<int> {};
    for (const int node : sources) {
        if (valid_node(graph, node)) {
            starts.insert(node);
        }
    }
    for (const int node : tree) {
        if (valid_node(graph, node)) {
            starts.insert(node);
        }
    }
    if (starts.empty() || !valid_node(graph, sink)) {
        throw std::runtime_error(
            std::format("maze: sink {} unreachable from given sources", sink));
    }

    int row_min = std::numeric_limits<int>::max();
    int row_max = std::numeric_limits<int>::min();
    int col_min = std::numeric_limits<int>::max();
    int col_max = std::numeric_limits<int>::min();
    bool has_bbox = false;
    auto consider_bbox = [&](int node_id) {
        if (node_kind(graph, node_id) != UnifiedNodeKind::Track) {
            return;
        }
        const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
        has_bbox = true;
        row_min = std::min(row_min, node.track_row);
        row_max = std::max(row_max, node.track_row);
        col_min = std::min(col_min, node.track_col);
        col_max = std::max(col_max, node.track_col);
    };
    for (const int node : starts) {
        consider_bbox(node);
    }
    consider_bbox(sink);

    const int initial_unit = is_bnet ? resources.selected_unit(owner) : -1;
    constexpr double inf = std::numeric_limits<double>::infinity();
    auto dist = std::Map<SearchState, double> {};
    auto parent = std::Map<SearchState, SearchState> {};
    auto closed = std::Set<SearchState> {};
    std::priority_queue<HeapItem, std::Vector<HeapItem>, HeapCompare> heap;
    int seq = 0;

    for (const int node : starts) {
        const SearchState state {node, initial_unit};
        dist[state] = 0;
        parent[state] = SearchState {-1, initial_unit};
        heap.push(HeapItem {0, seq++, node, initial_unit});
    }

    while (!heap.empty()) {
        const auto item = heap.top();
        heap.pop();
        const SearchState state {item.node, item.unit};
        const auto dist_it = dist.find(state);
        if (dist_it == dist.end() || item.cost > dist_it->second) {
            continue;
        }
        if (!closed.insert(state).second) {
            continue;
        }
        if (item.node == sink) {
            return reconstruct_path(parent, state);
        }

        for (const int arc_id : ordered_out_arc_ids(graph, item.node, interposer)) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!valid_node(graph, arc.v)) {
                continue;
            }
            if (arc_is_hard_blocked(graph, resources, owner, arc, hard_block)) {
                continue;
            }
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
                starts,
                params,
                row_min,
                row_max,
                col_min,
                col_max,
                has_bbox);
            const double next_cost = item.cost + step;
            const SearchState next {arc.v, next_unit};
            auto next_it = dist.find(next);
            if (next_it == dist.end() || next_cost < next_it->second) {
                dist[next] = next_cost;
                parent[next] = state;
                heap.push(HeapItem {next_cost, seq++, arc.v, next_unit});
            }
        }
    }

    throw std::runtime_error(
        std::format("maze: sink {} unreachable from given sources", sink));
}

} // namespace PR_tool
