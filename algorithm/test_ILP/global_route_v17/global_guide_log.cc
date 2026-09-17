#include "global_route_v17/global_guide_log.hh"

#include "common/hw_map.hh"
#include "sat/routing_path_log.hh"

#include <algorithm>
#include <chrono>
#include <debug/debug.hh>
#include <deque>
#include <format>
#include <iterator>
#include <limits>
#include <map>
#include <set>

namespace PR_tool {

namespace {

auto net_for(const std::Vector<RoutingNet>& nets, std::size_t id)
    -> const RoutingNet* {
    const auto it =
        std::find_if(nets.begin(), nets.end(),
                     [=](const RoutingNet& net) { return net.net_id == id; });
    return it == nets.end() ? nullptr : &*it;
}

auto demand_for(const RoutingNet& net, std::size_t id) -> const RoutingDemand* {
    const auto it =
        std::find_if(net.demands.begin(), net.demands.end(),
                     [=](const RoutingDemand& d) { return d.demand_id == id; });
    return it == net.demands.end() ? nullptr : &*it;
}

auto channel_text(const GlobalChannelGraph& graph, int id) -> std::String {
    if (id < 0 || static_cast<std::size_t>(id) >= graph.channels.size()) {
        return std::format("Channel(?)#{}", id);
    }
    const auto& c = graph.channels[static_cast<std::size_t>(id)];
    return std::format("Channel({},{},{})", c.dir == 0 ? 'H' : 'V', c.row,
                       c.col);
}

auto node_text(const GlobalChannelGraph& graph, int id) -> std::String {
    if (id < 0 || static_cast<std::size_t>(id) >= graph.nodes.size()) {
        return std::format("N{}", id);
    }
    const auto& n = graph.nodes[static_cast<std::size_t>(id)];
    switch (n.kind) {
    case GlobalRouteNodeKind::Cob:
        return std::format("COB({},{})", n.row, n.col);
    case GlobalRouteNodeKind::TobTerminal: {
        const auto [row, col] = tob_index_from_linear(n.tob);
        return std::format("TOB({},{})", row, col);
    }
    case GlobalRouteNodeKind::PortTerminal:
        return std::format("Port({},{},{},i{})", n.port.dir == 0 ? 'H' : 'V',
                           n.port.row, n.port.col, n.port.track_index);
    case GlobalRouteNodeKind::BoundaryTerminal:
        return std::format("Boundary({})", channel_text(graph, n.channel));
    }
    return std::format("N{}", id);
}

auto endpoint_text(const GraphNodeRef& ref) -> std::String {
    return format_graph_node_ref(ref);
}

auto format_channels(const GlobalChannelGraph& graph, const std::set<int>& ids)
    -> std::String {
    auto out = std::String{"{"};
    bool first = true;
    for (const int id : ids) {
        if (!first)
            out += ", ";
        first = false;
        out += channel_text(graph, id);
    }
    return out + "}";
}

} // namespace

auto log_global_route_guides(const GlobalRouteResult& route,
                             const GlobalChannelGraph& graph,
                             const RoutingProblemState& state,
                             const std::Vector<RoutingNet>& nets) -> long long {
    const auto begin = std::chrono::steady_clock::now();
    debug::info("========== V20 Global Routing raw guide paths (diagnostic; "
                "timing excluded) ==========");
    for (const auto& [key, raw_arc_ids] : route.selected_arc_ids_by_pair) {
        const auto* net = net_for(nets, key.net_id);
        if (net == nullptr || key.source_index >= net->sources.size()) {
            debug::warning_fmt(
                "global guide pair net={} demand={} source={} cannot "
                "resolve endpoints",
                key.net_id, key.demand_id, key.source_index);
            continue;
        }
        const auto* demand = demand_for(*net, key.demand_id);
        if (demand == nullptr) {
            debug::warning_fmt(
                "global guide pair net={} demand={} cannot resolve sink",
                key.net_id, key.demand_id);
            continue;
        }
        auto source_node = -1;
        const auto& source = net->sources[key.source_index];
        if (source.kind == GraphNodeRef::Kind::Bump) {
            if (const auto it = graph.tob_node_by_tob.find(source.bump.TOB);
                it != graph.tob_node_by_tob.end())
                source_node = it->second;
        } else if (source.kind == GraphNodeRef::Kind::Track) {
            const GlobalPortKey port{
                source.track_coord.dir == hardware::TrackDirection::Horizontal
                    ? 0
                    : 1,
                static_cast<int>(source.track_coord.row),
                static_cast<int>(source.track_coord.col), source.track_index};
            if (const auto it = graph.port_node_by_key.find(port);
                it != graph.port_node_by_key.end())
                source_node = it->second;
        }
        auto sink_node = -1;
        const auto& sink = demand->sink;
        if (sink.kind == GraphNodeRef::Kind::Bump) {
            if (const auto it = graph.tob_node_by_tob.find(sink.bump.TOB);
                it != graph.tob_node_by_tob.end())
                sink_node = it->second;
        } else if (sink.kind == GraphNodeRef::Kind::Track) {
            const GlobalPortKey port{
                sink.track_coord.dir == hardware::TrackDirection::Horizontal
                    ? 0
                    : 1,
                static_cast<int>(sink.track_coord.row),
                static_cast<int>(sink.track_coord.col), sink.track_index};
            if (const auto it = graph.port_node_by_key.find(port);
                it != graph.port_node_by_key.end())
                sink_node = it->second;
        }

        auto selected = std::set<int>{};
        for (const int id : raw_arc_ids)
            if (id >= 0 && static_cast<std::size_t>(id) < graph.arcs.size())
                selected.insert(id);
        auto outgoing = std::map<int, std::Vector<int>>{};
        auto indegree = std::map<int, int>{};
        for (const int id : selected) {
            const auto& a = graph.arcs[static_cast<std::size_t>(id)];
            outgoing[a.u].push_back(id);
            ++indegree[a.v];
        }
        // A selected MCF flow may branch at the source or contain a cycle. Pick
        // one deterministic directed BFS walk for readability, then report
        // every remaining selected arc explicitly instead of claiming
        // uniqueness.
        auto predecessor = std::map<int, int>{};
        auto visited = std::set<int>{};
        auto queue = std::deque<int>{};
        if (source_node >= 0) {
            visited.insert(source_node);
            queue.push_back(source_node);
        }
        while (!queue.empty() && !visited.contains(sink_node)) {
            const int current = queue.front();
            queue.pop_front();
            const auto out_it = outgoing.find(current);
            if (out_it == outgoing.end())
                continue;
            for (const int arc_id : out_it->second) {
                const int next = graph.arcs[static_cast<std::size_t>(arc_id)].v;
                if (visited.insert(next).second) {
                    predecessor.emplace(next, arc_id);
                    queue.push_back(next);
                }
            }
        }
        const bool reaches_sink = visited.contains(sink_node);
        auto walk = std::Vector<int>{};
        if (reaches_sink) {
            for (int node = sink_node; node != source_node;) {
                const int arc = predecessor.at(node);
                walk.push_back(arc);
                node = graph.arcs[static_cast<std::size_t>(arc)].u;
            }
            std::reverse(walk.begin(), walk.end());
        }
        auto walk_arcs = std::set<int>{walk.begin(), walk.end()};
        bool cycle = false;
        // An edge inside the selected directed graph that closes a DFS/BFS tree
        // is conservatively reported through residual_selected_arcs.
        if (!reaches_sink || selected.size() != walk_arcs.size())
            cycle = true;
        bool branch = false;
        for (const auto& [_, ids] : outgoing)
            if (ids.size() > 1)
                branch = true;
        for (const auto& [_, degree] : indegree)
            if (degree > 1)
                branch = true;
        std::set<int> channels{};
        std::set<std::pair<int, int>> cobs{};
        for (const int id : selected) {
            const auto& a = graph.arcs[static_cast<std::size_t>(id)];
            if (a.channel >= 0)
                channels.insert(a.channel);
            for (const int v : {a.u, a.v}) {
                if (v >= 0 &&
                    static_cast<std::size_t>(v) < graph.nodes.size()) {
                    const auto& n = graph.nodes[static_cast<std::size_t>(v)];
                    if (n.kind == GlobalRouteNodeKind::Cob)
                        cobs.emplace(n.row, n.col);
                }
            }
        }
        auto unit = std::String{"n/a"};
        if (const auto it = route.selected_unit_by_pair.find(key);
            it != route.selected_unit_by_pair.end()) {
            unit = std::to_string(it->second);
        } else if (const auto it = route.unit_by_owner.find(
                       GlobalUnitOwnerKey{key.net_id, key.source_index});
                   it != route.unit_by_owner.end()) {
            unit = std::to_string(it->second);
        }
        debug::info_fmt(
            "global guide net=\"{}\" id={} pair=(demand={},source={}) unit={}",
            net->name, key.net_id, key.demand_id, key.source_index, unit);
        debug::info_fmt("  source: {}", endpoint_text(source));
        debug::info_fmt("  target: {}", endpoint_text(sink));
        auto text = source_node >= 0 ? node_text(graph, source_node)
                                     : endpoint_text(source);
        int previous_channel = -1;
        for (const int id : walk) {
            const auto& a = graph.arcs[static_cast<std::size_t>(id)];
            // Only a consecutive same physical Channel is a TOB half-edge
            // duplicate. A later revisit is semantically a real detour and must
            // stay visible in the ordered walk.
            if (a.channel >= 0 && a.channel != previous_channel)
                text += " -> " + channel_text(graph, a.channel);
            text += " -> " + node_text(graph, a.v);
            previous_channel = a.channel;
        }
        debug::info_fmt(
            "  selected: {}{}", text,
            reaches_sink ? "" : " [no selected directed walk reaches target]");
        std::set<int> residual{};
        std::set_difference(selected.begin(), selected.end(), walk_arcs.begin(),
                            walk_arcs.end(),
                            std::inserter(residual, residual.end()));
        debug::info_fmt("  selected_channels={} selected_cobs={} macro_arcs={} "
                        "has_cycle_or_branch={}",
                        channels.size(), cobs.size(), selected.size(),
                        cycle || branch || !reaches_sink);
        if (!residual.empty()) {
            auto items = std::String{};
            for (const int id : residual) {
                if (!items.empty())
                    items += ", ";
                const auto& a = graph.arcs[static_cast<std::size_t>(id)];
                items +=
                    std::format("{}:{}->{}", channel_text(graph, a.channel),
                                node_text(graph, a.u), node_text(graph, a.v));
            }
            debug::info_fmt("  residual_selected_arcs={{{}}}", items);
        }
        const auto* pair = find_pair_state(state, key);
        auto patch = std::set<int>{};
        auto final_channels = std::set<int>{};
        if (pair != nullptr) {
            for (const auto& c : pair->allowed_channels) {
                if (const auto it = graph.channel_id_by_coord.find(c);
                    it != graph.channel_id_by_coord.end()) {
                    final_channels.insert(it->second);
                    const auto raw = route.pair_channels.find(key);
                    if (raw == route.pair_channels.end() ||
                        !raw->second.contains(c)) {
                        patch.insert(it->second);
                    }
                }
            }
        }
        debug::info_fmt(
            "  scope_added_by_tob_patch={} final_pair_scope_channels={}",
            format_channels(graph, patch), final_channels.size());
    }
    for (const auto& net : nets) {
        const auto pairs = state.pair_indices_by_net.find(net.net_id);
        if (pairs == state.pair_indices_by_net.end() ||
            pairs->second.size() < 2)
            continue;
        std::set<GlobalChannelCoord> joined{};
        for (const auto index : pairs->second)
            joined.insert(state.pairs[index].allowed_channels.begin(),
                          state.pairs[index].allowed_channels.end());
        debug::info_fmt("global guide owner union net=\"{}\" id={} pairs={} "
                        "final_scope_channels={}",
                        net.name, net.net_id, pairs->second.size(),
                        joined.size());
        if (net.is_sync_bus) {
            auto min_count = std::numeric_limits<std::size_t>::max();
            auto max_count = std::size_t{0};
            for (const auto index : pairs->second) {
                const auto& pair = state.pairs[index];
                const auto it = route.pair_channels.find(pair.key);
                const auto count =
                    it == route.pair_channels.end() ? 0 : it->second.size();
                min_count = std::min(min_count, count);
                max_count = std::max(max_count, count);
                debug::info_fmt("  Sync guide member demand={} source={} "
                                "selected_channels={}",
                                pair.key.demand_id, pair.key.source_index,
                                count);
            }
            debug::info_fmt(
                "  Sync guide member selected_channels: min={} max={}",
                min_count == std::numeric_limits<std::size_t>::max()
                    ? 0
                    : min_count,
                max_count);
        }
    }
    const auto body_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - begin)
                             .count();
    debug::info_fmt("V20 Global Routing guide paths complete: body_log_ms={} "
                    "(full diagnostic time excluded from routing timing)",
                    body_ms);
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - begin)
        .count();
}

} // namespace PR_tool
