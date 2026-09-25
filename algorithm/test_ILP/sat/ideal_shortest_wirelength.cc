#include "sat/ideal_shortest_wirelength.hh"

#include "common/hw_map.hh"
#include "sat/routing_path_log.hh"
#include "scope/pair_routing_state.hh"

#include <hardware/bump/bump.hh>
#include <hardware/interposer.hh>
#include <hardware/track/track.hh>

#include <algorithm>
#include <queue>
#include <set>
#include <stdexcept>

namespace PR_tool {

namespace {

auto routing_bump_index(const Bump_coord& bump) -> std::size_t {
    return bump.Bank * 64 + bump.Group * 8 + bump.Index;
}

auto resolve_bump(hardware::Interposer* interposer, const Bump_coord& bump) -> hardware::Bump* {
    const auto [tr, tc] = tob_index_from_linear(bump.TOB);
    const auto opt = interposer->get_bump(
        static_cast<std::i64>(tr),
        static_cast<std::i64>(tc),
        routing_bump_index(bump));
    return opt.has_value() ? opt.value() : nullptr;
}

auto resolve_track(hardware::Interposer* interposer, const GraphNodeRef& ref) -> hardware::Track* {
    if (ref.kind != GraphNodeRef::Kind::Track) {
        return nullptr;
    }
    const auto opt = interposer->get_track(ref.track_coord);
    return opt.has_value() ? opt.value() : nullptr;
}

auto maze_track_path(
    hardware::Interposer* interposer,
    const std::Vector<hardware::Track*>& begin_tracks,
    const std::HashSet<hardware::Track*>& end_tracks
) -> std::Vector<hardware::Track*> {
    if (begin_tracks.empty() || end_tracks.empty()) {
        return {};
    }

    auto prev = std::HashMap<hardware::Track*, hardware::Track*> {};
    auto queue = std::queue<hardware::Track*> {};
    for (auto* track : begin_tracks) {
        if (track == nullptr) {
            continue;
        }
        if (!prev.contains(track)) {
            prev.emplace(track, nullptr);
            queue.push(track);
        }
    }

    hardware::Track* goal = nullptr;
    while (!queue.empty()) {
        auto* current = queue.front();
        queue.pop();
        if (end_tracks.contains(current)) {
            goal = current;
            break;
        }
        for (auto& [next, connector] : interposer->adjacent_idle_tracks(current)) {
            (void)connector;
            if (prev.contains(next)) {
                continue;
            }
            prev.emplace(next, current);
            queue.push(next);
        }
    }

    if (goal == nullptr) {
        return {};
    }

    auto path = std::Vector<hardware::Track*> {};
    for (auto* cur = goal; cur != nullptr; cur = prev.at(cur)) {
        path.push_back(cur);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

auto ideal_tree_wirelength(
    hardware::Interposer* interposer,
    const RoutingNet& net,
    bool include_source_bump
) -> std::size_t {
    auto begin_tracks = std::Vector<hardware::Track*> {};
    auto used_tracks = std::HashSet<hardware::Track*> {};
    auto used_bumps = std::HashSet<hardware::Bump*> {};

    if (net.kind == RoutingNetKind::PNnet) {
        for (const auto& source_ref : net.sources) {
            if (auto* track = resolve_track(interposer, source_ref)) {
                begin_tracks.push_back(track);
                used_tracks.insert(track);
            }
        }
    }
    else {
        if (net.sources.empty()) {
            return 0;
        }
        const auto& source_ref = net.sources.front();
        if (source_ref.kind == GraphNodeRef::Kind::Track) {
            if (auto* track = resolve_track(interposer, source_ref)) {
                begin_tracks.push_back(track);
                used_tracks.insert(track);
            }
        }
        else if (include_source_bump) {
            if (auto* bump = resolve_bump(interposer, source_ref.bump)) {
                used_bumps.insert(bump);
                const auto begin_map = interposer->available_tracks_bump_to_track(bump);
                for (auto& [track, connector] : begin_map) {
                    (void)connector;
                    begin_tracks.push_back(track);
                    used_tracks.insert(track);
                }
            }
        }
    }

    if (begin_tracks.empty()) {
        return 0;
    }

    for (const auto& demand : net.demands) {
        if (demand.sink.kind != GraphNodeRef::Kind::Bump) {
            continue;
        }
        auto* sink_bump = resolve_bump(interposer, demand.sink.bump);
        if (sink_bump == nullptr) {
            continue;
        }
        const auto end_map = interposer->available_tracks_track_to_bump(sink_bump);
        auto end_tracks = std::HashSet<hardware::Track*> {};
        for (auto& [track, connector] : end_map) {
            (void)connector;
            end_tracks.insert(track);
        }
        if (end_tracks.empty()) {
            continue;
        }

        const auto path = maze_track_path(interposer, begin_tracks, end_tracks);
        if (path.empty()) {
            continue;
        }
        for (auto* track : path) {
            used_tracks.insert(track);
            begin_tracks.push_back(track);
        }
        used_bumps.insert(sink_bump);
    }

    return used_tracks.size() + used_bumps.size();
}

auto pairs_for_net(const DelayPrecomputeResult& delays, std::size_t net_id)
    -> std::Vector<const PairDelayInfo*> {
    auto out = std::Vector<const PairDelayInfo*> {};
    for (const auto& pair : delays.pairs) {
        if (pair.net_id == net_id) {
            out.push_back(&pair);
        }
    }
    return out;
}

auto ideal_two_pin_wirelength(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const DelayPrecomputeResult& delays
) -> std::size_t {
    const auto net_pairs = pairs_for_net(delays, net.net_id);
    if (net_pairs.empty() || net.demands.empty()) {
        return 0;
    }
    const auto& pair = *net_pairs.front();
    const auto path = shortest_unified_node_path(graph, pair.source_node, pair.sink_node);
    if (path.empty()) {
        return 0;
    }
    return unified_path_wirelength(graph, path);
}

auto ideal_sync_bus_wirelength(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const DelayPrecomputeResult& delays
) -> std::size_t {
    const auto net_pairs = pairs_for_net(delays, net.net_id);
    if (net_pairs.empty()) {
        return 0;
    }

    int bus_d_min = 0;
    for (const auto* pair : net_pairs) {
        const int member_shortest = pair->member_shortest_delay >= 0
            ? pair->member_shortest_delay
            : max_delay(pair->delays);
        bus_d_min = std::max(bus_d_min, member_shortest);
    }

    const PairDelayInfo* reference = nullptr;
    for (const auto* pair : net_pairs) {
        const int member_shortest = pair->member_shortest_delay >= 0
            ? pair->member_shortest_delay
            : max_delay(pair->delays);
        if (member_shortest == bus_d_min) {
            reference = pair;
            break;
        }
    }
    if (reference == nullptr) {
        reference = net_pairs.front();
    }

    const auto path =
        shortest_unified_node_path(graph, reference->source_node, reference->sink_node);
    if (path.empty()) {
        return 0;
    }
    const auto member_wirelength = unified_path_wirelength(graph, path);
    return member_wirelength * net.demands.size();
}

} // namespace

auto shortest_unified_node_path(
    const UnifiedGraph& graph,
    int source_node,
    int sink_node
) -> std::Vector<int> {
    if (source_node < 0
        || sink_node < 0
        || static_cast<std::size_t>(source_node) >= graph.nodes.size()
        || static_cast<std::size_t>(sink_node) >= graph.nodes.size()) {
        return {};
    }

    auto distances = std::Vector<int>(graph.nodes.size(), -1);
    auto previous = std::Vector<int>(graph.nodes.size(), -1);
    auto queue = std::queue<int> {};
    distances[static_cast<std::size_t>(source_node)] = 0;
    queue.push(source_node);

    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        if (current == sink_node) {
            break;
        }
        for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(current)]) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            const int next = arc.v;
            if (next < 0 || static_cast<std::size_t>(next) >= graph.nodes.size()) {
                continue;
            }
            if (distances[static_cast<std::size_t>(next)] >= 0) {
                continue;
            }
            distances[static_cast<std::size_t>(next)] =
                distances[static_cast<std::size_t>(current)] + 1;
            previous[static_cast<std::size_t>(next)] = current;
            queue.push(next);
        }
    }

    if (distances[static_cast<std::size_t>(sink_node)] < 0) {
        return {};
    }

    auto path = std::Vector<int> {};
    for (int current = sink_node; current >= 0; current = previous[static_cast<std::size_t>(current)]) {
        path.push_back(current);
        if (current == source_node) {
            break;
        }
    }
    if (path.empty() || path.back() != source_node) {
        return {};
    }
    std::reverse(path.begin(), path.end());
    return path;
}

auto unified_path_wirelength(const UnifiedGraph& graph, const std::Vector<int>& node_path)
    -> std::size_t {
    return path_wirelength(graph, node_path);
}

auto ideal_net_wirelength(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const DelayPrecomputeResult& delays
) -> std::size_t {
    const auto display_kind = infer_net_display_kind(net);
    switch (display_kind) {
        case NetDisplayKind::SyncBus:
            return ideal_sync_bus_wirelength(graph, net, delays);
        case NetDisplayKind::TrackToBumps:
        case NetDisplayKind::TracksToBumps:
            if (interposer == nullptr) {
                return 0;
            }
            return ideal_tree_wirelength(
                interposer,
                net,
                display_kind == NetDisplayKind::TrackToBumps
                    && !net.sources.empty()
                    && net.sources.front().kind == GraphNodeRef::Kind::Bump);
        case NetDisplayKind::TwoPin:
        default:
            return ideal_two_pin_wirelength(graph, net, delays);
    }
}

} // namespace PR_tool
