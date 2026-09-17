#include "sync_equalize.hh"

#include "maze_search.hh"

#include <debug/debug.hh>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace PR_tool {

namespace {

struct TailRec {
    int node{-1};
    int unit{-1};
    int parent{-1};
    int track_count{0};
};

struct TailState {
    int node{-1};
    int unit{-1};
    int track_count{0};

    auto operator<(const TailState& other) const -> bool {
        return std::tie(node, unit, track_count) < std::tie(other.node, other.unit, other.track_count);
    }
};

struct TailItem {
    double cost{0};
    int seq{0};
    int rec{-1};
};

struct TailCompare {
    auto operator()(const TailItem& lhs, const TailItem& rhs) const -> bool {
        if (lhs.cost != rhs.cost) {
            return lhs.cost > rhs.cost;
        }
        return lhs.seq > rhs.seq;
    }
};

enum class TailKind { Equal, Longer, Fail };

struct TailOutcome {
    TailKind kind{TailKind::Fail};
    std::Vector<int> path;
    std::size_t n_f{0};
};

auto valid_node(const UnifiedGraph& graph, int node_id) -> bool {
    return node_id >= 0 && node_id < static_cast<int>(graph.nodes.size());
}

auto is_track_node(const UnifiedGraph& graph, int node_id) -> bool {
    return valid_node(graph, node_id)
        && graph.nodes[static_cast<std::size_t>(node_id)].kind == UnifiedNodeKind::Track;
}

auto is_vline_track(const UnifiedGraph& graph, const UnifiedArc& arc) -> bool {
    if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack) {
        return true;
    }
    if (!valid_node(graph, arc.u) || !valid_node(graph, arc.v)) {
        return false;
    }
    const auto ku = graph.nodes[static_cast<std::size_t>(arc.u)].kind;
    const auto kv = graph.nodes[static_cast<std::size_t>(arc.v)].kind;
    return (ku == UnifiedNodeKind::VLine && kv == UnifiedNodeKind::Track)
        || (ku == UnifiedNodeKind::Track && kv == UnifiedNodeKind::VLine);
}

auto track_endpoint(const UnifiedGraph& graph, const UnifiedArc& arc) -> int {
    if (is_track_node(graph, arc.u)) {
        return arc.u;
    }
    if (is_track_node(graph, arc.v)) {
        return arc.v;
    }
    return -1;
}

auto track_nodes_of(const UnifiedGraph& graph, const std::Vector<int>& path) -> std::Vector<int> {
    auto tracks = std::Vector<int> {};
    for (const int node : path) {
        if (is_track_node(graph, node)) {
            tracks.push_back(node);
        }
    }
    return tracks;
}

auto prefix_through_cut(
    const UnifiedGraph& graph,
    const std::Vector<int>& path,
    std::size_t keep_tracks
) -> std::Vector<int> {
    if (keep_tracks == 0) {
        return {};
    }
    auto prefix = std::Vector<int> {};
    std::size_t seen = 0;
    for (const int node : path) {
        prefix.push_back(node);
        if (is_track_node(graph, node)) {
            ++seen;
            if (seen == keep_tracks) {
                break;
            }
        }
    }
    return prefix;
}

auto join_prefix_tail(const std::Vector<int>& prefix, const std::Vector<int>& tail) -> std::Vector<int> {
    if (prefix.empty()) {
        return tail;
    }
    if (tail.empty()) {
        return prefix;
    }
    auto out = prefix;
    const std::size_t begin = (tail.front() == prefix.back()) ? 1 : 0;
    for (std::size_t i = begin; i < tail.size(); ++i) {
        out.push_back(tail[i]);
    }
    return out;
}

auto on_parent_chain(const std::Vector<TailRec>& recs, int idx, int node) -> bool {
    while (idx >= 0) {
        if (recs[static_cast<std::size_t>(idx)].node == node) {
            return true;
        }
        idx = recs[static_cast<std::size_t>(idx)].parent;
    }
    return false;
}

auto reconstruct_tail(const std::Vector<TailRec>& recs, int idx) -> std::Vector<int> {
    auto path = std::Vector<int> {};
    while (idx >= 0) {
        path.push_back(recs[static_cast<std::size_t>(idx)].node);
        idx = recs[static_cast<std::size_t>(idx)].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

auto arc_hard_blocked(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const UnifiedArc& arc,
    const std::Set<ResourceKey>& hard_block
) -> bool {
    if (hard_block.empty()) {
        return false;
    }
    for (const auto& key : arc_resource_keys(graph, arc)) {
        if (resources.holds(owner, key)) {
            continue;
        }
        if (hard_block.contains(key)) {
            return true;
        }
    }
    return false;
}

auto sibling_hard_block(
    const UnifiedGraph& graph,
    const std::Vector<SyncLaneState>& lanes,
    std::size_t skip
) -> std::Set<ResourceKey> {
    auto keys = std::Set<ResourceKey> {};
    for (std::size_t i = 0; i < lanes.size(); ++i) {
        if (i == skip) {
            continue;
        }
        for (const auto& key : path_resource_keys(graph, lanes[i].path, lanes[i].is_bnet)) {
            if (is_physical_occupancy_key(key)) {
                keys.insert(key);
            }
        }
    }
    return keys;
}

auto install_lane_paths(
    const UnifiedGraph& graph,
    ResourceModel& resources,
    std::Vector<SyncLaneState>& lanes,
    const std::Vector<std::Vector<int>>& paths
) -> void {
    for (auto& lane : lanes) {
        resources.release(lane.id);
    }
    for (std::size_t i = 0; i < lanes.size(); ++i) {
        lanes[i].path = i < paths.size() ? paths[i] : std::Vector<int> {};
        if (lanes[i].path.empty()) {
            continue;
        }
        resources.claim(lanes[i].id, path_resource_keys(graph, lanes[i].path, lanes[i].is_bnet));
    }
}

auto claim_prefix_only(
    const UnifiedGraph& graph,
    ResourceModel& resources,
    SyncLaneState& lane,
    const std::Vector<int>& prefix
) -> void {
    resources.release(lane.id);
    if (prefix.empty()) {
        return;
    }
    resources.claim(lane.id, path_resource_keys(graph, prefix, lane.is_bnet));
}

auto group_max_length(
    const UnifiedGraph& graph,
    hardware::Interposer* interposer,
    const std::Vector<SyncLaneState>& lanes
) -> std::size_t {
    std::size_t n_max = 0;
    for (const auto& lane : lanes) {
        n_max = std::max(n_max, sync_lane_length(graph, lane.path, interposer, lane.is_bnet));
    }
    return n_max;
}

auto all_lanes_equal(
    const UnifiedGraph& graph,
    hardware::Interposer* interposer,
    const std::Vector<SyncLaneState>& lanes
) -> bool {
    if (lanes.empty()) {
        return true;
    }
    const auto expected = sync_lane_length(graph, lanes.front().path, interposer, lanes.front().is_bnet);
    for (const auto& lane : lanes) {
        if (lane.path.empty()) {
            return false;
        }
        if (sync_lane_length(graph, lane.path, interposer, lane.is_bnet) != expected) {
            return false;
        }
    }
    return true;
}

auto short_lane_indices(
    const UnifiedGraph& graph,
    hardware::Interposer* interposer,
    const std::Vector<SyncLaneState>& lanes,
    std::size_t n_max
) -> std::Vector<std::size_t> {
    auto shorts = std::Vector<std::size_t> {};
    for (std::size_t i = 0; i < lanes.size(); ++i) {
        if (sync_lane_length(graph, lanes[i].path, interposer, lanes[i].is_bnet) < n_max) {
            shorts.push_back(i);
        }
    }
    return shorts;
}

auto maze_tail(
    const UnifiedGraph& graph,
    ResourceModel& resources,
    const RrrParams& params,
    hardware::Interposer* interposer,
    const SyncLaneState& lane,
    const std::Vector<int>& prefix,
    const std::Set<ResourceKey>& hard_block,
    std::size_t target_length,
    std::size_t length_limit
) -> TailOutcome {
    auto starts = std::Set<int> {};
    if (prefix.empty()) {
        for (const int node : lane.sources) {
            if (valid_node(graph, node)) {
                starts.insert(node);
            }
        }
    } else {
        starts.insert(prefix.back());
    }
    if (starts.empty() || !valid_node(graph, lane.sink)) {
        return {};
    }

    auto forbidden = std::Set<int> {};
    for (const int node : prefix) {
        if (!starts.contains(node)) {
            forbidden.insert(node);
        }
    }

    const int initial_unit = lane.is_bnet ? resources.selected_unit(lane.id) : -1;
    const auto tob_constant = sync_tob_length_constant(lane.is_bnet);
    const int max_tracks = length_limit > tob_constant
        ? static_cast<int>(length_limit - tob_constant)
        : 0;
    const std::size_t recs_cap = std::min(
        static_cast<std::size_t>(graph.nodes.size()) * 3 + 2048,
        static_cast<std::size_t>(80000));
    auto recs = std::Vector<TailRec> {};
    auto dist = std::Map<TailState, double> {};
    auto closed = std::Set<TailState> {};
    std::priority_queue<TailItem, std::Vector<TailItem>, TailCompare> heap;
    int seq = 0;
    bool logged_track_limit = false;
    debug::debug_fmt(
        "FPIA RRR: sync tail maze sink={} prefix={} target={} limit={} recs_cap={}",
        lane.sink,
        prefix.size(),
        target_length,
        length_limit,
        recs_cap);

    for (const int node : starts) {
        const int tracks = is_track_node(graph, node) ? 1 : 0;
        const TailState state {node, initial_unit, tracks};
        dist[state] = 0;
        recs.push_back(TailRec {node, initial_unit, -1, tracks});
        heap.push(TailItem {0, seq++, static_cast<int>(recs.size() - 1)});
    }

    auto best_over = TailOutcome {};

    while (!heap.empty()) {
        const auto item = heap.top();
        heap.pop();
        const TailRec rec = recs[static_cast<std::size_t>(item.rec)];
        const TailState state {rec.node, rec.unit, rec.track_count};
        const auto dist_it = dist.find(state);
        if (dist_it == dist.end() || item.cost > dist_it->second) {
            continue;
        }
        if (!closed.insert(state).second) {
            continue;
        }

        if (rec.track_count > max_tracks) {
            if (!logged_track_limit) {
                debug::info_fmt(
                    "FPIA RRR: sync tail cutoff sink={} target={} max_track_nodes={} extra_tracks={}",
                    lane.sink,
                    target_length,
                    max_tracks,
                    params.sync_tail_extra_tracks);
                logged_track_limit = true;
            }
            continue;
        }

        if (rec.node == lane.sink) {
            const auto tail = reconstruct_tail(recs, item.rec);
            const auto full = join_prefix_tail(prefix, tail);
            const auto n_f = sync_lane_length(graph, full, interposer, lane.is_bnet);
            if (n_f == target_length) {
                return TailOutcome {TailKind::Equal, full, n_f};
            }
            if (n_f > target_length && n_f <= length_limit
                && (best_over.kind == TailKind::Fail || n_f < best_over.n_f)) {
                best_over = TailOutcome {TailKind::Longer, full, n_f};
            }
            continue;
        }

        if (recs.size() >= recs_cap) {
            continue;
        }

        for (const int arc_id : maze_ordered_out_arc_ids(graph, rec.node, interposer)) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!valid_node(graph, arc.v)) {
                continue;
            }
            if (forbidden.contains(arc.v) || on_parent_chain(recs, item.rec, arc.v)) {
                continue;
            }
            if (arc_hard_blocked(graph, resources, lane.id, arc, hard_block)) {
                continue;
            }

            int next_unit = rec.unit;
            if (lane.is_bnet && is_vline_track(graph, arc)) {
                const int track_node = track_endpoint(graph, arc);
                const int unit = track_node >= 0
                    ? static_cast<int>(graph.nodes[static_cast<std::size_t>(track_node)].unit)
                    : -1;
                const int locked = rec.unit >= 0 ? rec.unit : initial_unit;
                if (locked >= 0 && unit != locked) {
                    continue;
                }
                next_unit = locked >= 0 ? locked : unit;
            }

            const int next_tracks = rec.track_count + (is_track_node(graph, arc.v) ? 1 : 0);
            const double step =
                arc_incremental_cost(graph, resources, lane.id, arc, starts, params);
            const double next_cost = item.cost + step;
            const TailState next {arc.v, next_unit, next_tracks};
            auto next_it = dist.find(next);
            if (next_it != dist.end() && next_cost >= next_it->second) {
                continue;
            }
            dist[next] = next_cost;
            recs.push_back(TailRec {arc.v, next_unit, item.rec, next_tracks});
            heap.push(TailItem {next_cost, seq++, static_cast<int>(recs.size() - 1)});
        }
    }

    return best_over;
}

} // namespace

auto sync_track_cut_index(std::size_t Ni, double r) -> std::size_t {
    if (Ni == 0 || r >= 1.0) {
        return 0;
    }
    if (r <= 0.0) {
        return Ni;
    }
    return static_cast<std::size_t>(std::floor(static_cast<double>(Ni) * (1.0 - r)));
}

auto sync_lane_length(
    const UnifiedGraph& graph,
    const std::Vector<int>& node_path,
    hardware::Interposer* interposer,
    bool is_bnet
) -> std::size_t {
    if (interposer == nullptr) {
        throw std::runtime_error("sync_lane_length: Interposer is required");
    }
    std::size_t n_track = 0;
    for (const int node_id : node_path) {
        if (!is_track_node(graph, node_id)) {
            continue;
        }
        ++n_track;
    }
    return n_track + sync_tob_length_constant(is_bnet);
}

auto equalize_sync_group(
    const UnifiedGraph& graph,
    ResourceModel& resources,
    const RrrParams& params,
    hardware::Interposer* interposer,
    std::Vector<SyncLaneState>& lanes
) -> bool {
    if (lanes.size() <= 1) {
        return !lanes.empty() && !lanes.front().path.empty();
    }
    if (interposer == nullptr) {
        return false;
    }
    if (all_lanes_equal(graph, interposer, lanes)) {
        return true;
    }

    const auto snapshot = [&]() {
        auto paths = std::Vector<std::Vector<int>> {};
        paths.reserve(lanes.size());
        for (const auto& lane : lanes) {
            paths.push_back(lane.path);
        }
        return paths;
    }();

    auto ratios = params.r_sequence;
    if (ratios.empty()) {
        ratios = {0.5, 0.75, 1.0};
    }

    const auto base_length = group_max_length(graph, interposer, lanes);
    const auto length_limit = base_length
        + static_cast<std::size_t>(std::max(0, params.sync_tail_extra_tracks));
    for (const double r : ratios) {
        auto target_length = base_length;
        while (target_length <= length_limit) {
            install_lane_paths(graph, resources, lanes, snapshot);
            const auto queue = short_lane_indices(graph, interposer, lanes, target_length);
            debug::debug_fmt(
                "FPIA RRR: sync equalize r={} target={} limit={} short_lanes={}",
                r,
                target_length,
                length_limit,
                queue.size());

            bool failed = false;
            std::size_t next_target = 0;
            for (const std::size_t idx : queue) {
                auto& lane = lanes[idx];
                if (sync_lane_length(graph, lane.path, interposer, lane.is_bnet) >= target_length) {
                    continue;
                }
                const auto tracks = track_nodes_of(graph, lane.path);
                const auto keep = sync_track_cut_index(tracks.size(), r);
                const auto prefix = prefix_through_cut(graph, lane.path, keep);
                claim_prefix_only(graph, resources, lane, prefix);
                const auto blocked = sibling_hard_block(graph, lanes, idx);
                const auto outcome = maze_tail(
                    graph,
                    resources,
                    params,
                    interposer,
                    lane,
                    prefix,
                    blocked,
                    target_length,
                    length_limit);
                if (outcome.kind != TailKind::Equal || outcome.path.empty()) {
                    if (outcome.kind == TailKind::Longer) {
                        next_target = outcome.n_f;
                    }
                    failed = true;
                    break;
                }
                resources.release(lane.id);
                resources.claim(lane.id, path_resource_keys(graph, outcome.path, lane.is_bnet));
                lane.path = outcome.path;
            }
            if (!failed && all_lanes_equal(graph, interposer, lanes)) {
                return true;
            }
            if (!failed || next_target <= target_length || next_target > length_limit) {
                break;
            }
            debug::debug_fmt(
                "FPIA RRR: sync target advance from {} to {}",
                target_length,
                next_target);
            target_length = next_target;
        }
    }

    install_lane_paths(graph, resources, lanes, snapshot);
    return false;
}

} // namespace PR_tool
