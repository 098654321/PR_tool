#include "delay/pair_delay_precompute.hh"

#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"
#include "scope/pair_routing_state.hh"

#include <algorithm>
#include <debug/debug.hh>
#include <format>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace PR_tool {

namespace {

auto scope_node_in_bounds(const UnifiedSatNetScope& scope, int node) -> bool {
    return node >= 0
        && static_cast<std::size_t>(node) < scope.node_offset.size()
        && scope.node_offset[static_cast<std::size_t>(node)] >= 0;
}

auto set_pair_delay(PairDelayInfo& pair, const std::Vector<int>& delays) -> void {
    pair.delays = delays;
    pair.target_delay = max_delay(delays);
}

auto copy_delays_to_state(
    RoutingProblemState& state,
    const PairDelayInfo& pair
) -> void {
    auto* pair_state = find_pair_state(
        state,
        PairKey {pair.net_id, pair.demand_id, pair.source_index});
    if (pair_state != nullptr) {
        pair_state->delays = pair.delays;
    }
}

auto pnnet_initial_delay(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    const RoutingNet& net,
    int sink_node
) -> int {
    int best = -1;
    for (const auto& demand : net.demands) {
        if (resolve_graph_node(graph, demand.sink) != sink_node) {
            continue;
        }
        for (const std::size_t source_index : demand.candidate_source_indices) {
            if (source_index >= net.sources.size()) {
                continue;
            }
            const int track_node = resolve_graph_node(graph, net.sources[source_index]);
            const int shortest = bfs_shortest_delay(graph, scope, track_node, sink_node);
            if (shortest < 0) {
                continue;
            }
            const int shifted = shortest + 1;
            best = best < 0 ? shifted : std::min(best, shifted);
        }
        break;
    }
    if (best < 0) {
        for (const auto& source_ref : net.sources) {
            const int track_node = resolve_graph_node(graph, source_ref);
            const int shortest = bfs_shortest_delay(graph, scope, track_node, sink_node);
            if (shortest < 0) {
                continue;
            }
            const int shifted = shortest + 1;
            best = best < 0 ? shifted : std::min(best, shifted);
        }
    }
    return best;
}

} // namespace

auto bfs_reachability(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    int source_node,
    int d_max_bound
) -> ReachableDelayTable {
    if (!scope_node_in_bounds(scope, source_node)) {
        throw std::invalid_argument("BFS source is outside scope");
    }
    const auto source_offset =
        static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(source_node)]);
    auto table = ReachableDelayTable {};
    table.d_max = d_max_bound;
    table.reachable.assign(
        scope.node_ids.size(),
        std::Vector<bool>(static_cast<std::size_t>(d_max_bound) + 1, false));

    auto seen = std::Vector<std::Vector<bool>>(
        scope.node_ids.size(),
        std::Vector<bool>(static_cast<std::size_t>(d_max_bound) + 1, false));
    auto queue = std::queue<std::pair<std::size_t, int>> {};
    queue.emplace(source_offset, 0);
    seen[source_offset][0] = true;
    table.reachable[source_offset][0] = true;

    while (!queue.empty()) {
        const auto [node_offset, delay] = queue.front();
        queue.pop();
        const int node = scope.node_ids[node_offset];
        for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(node)]) {
            const int arc_offset = scope.arc_offset[static_cast<std::size_t>(arc_id)];
            if (arc_offset < 0) {
                continue;
            }
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            const int next_offset = scope.node_offset[static_cast<std::size_t>(arc.v)];
            if (next_offset < 0) {
                continue;
            }
            const int next_delay = delay + 1;
            if (next_delay > d_max_bound) {
                continue;
            }
            const auto next_offset_u = static_cast<std::size_t>(next_offset);
            if (seen[next_offset_u][static_cast<std::size_t>(next_delay)]) {
                continue;
            }
            seen[next_offset_u][static_cast<std::size_t>(next_delay)] = true;
            table.reachable[next_offset_u][static_cast<std::size_t>(next_delay)] = true;
            queue.emplace(next_offset_u, next_delay);
        }
    }
    return table;
}

auto bfs_shortest_delay(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    int source_node,
    int sink_node
) -> int {
    const int bound = static_cast<int>(scope.node_ids.size());
    const auto table = bfs_reachability(graph, scope, source_node, bound);
    if (!scope_node_in_bounds(scope, sink_node)) {
        return -1;
    }
    const auto sink_offset =
        static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(sink_node)]);
    for (int delay = 0; delay <= table.d_max; ++delay) {
        if (table.reachable[sink_offset][static_cast<std::size_t>(delay)]) {
            return delay;
        }
    }
    return -1;
}

auto compute_pair_delays(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    RoutingProblemState* problem_state
) -> DelayPrecomputeResult {
    auto result = DelayPrecomputeResult {};
    auto source_key_to_index = std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::size_t> {};

    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        const auto& net = nets[net_index];
        const auto& scope = scopes[net_index];
        const auto [source_nodes, sink_nodes] = resolve_endpoint_nodes(graph, net);

        auto member_pairs = std::Vector<PairDelayInfo> {};
        member_pairs.reserve(net.demands.size());

        for (std::size_t demand_index = 0; demand_index < net.demands.size(); ++demand_index) {
            const auto& demand = net.demands[demand_index];
            const std::size_t source_index =
                net.kind == RoutingNetKind::PNnet ? 0 : demand.candidate_source_indices.front();
            const int source_node = net.kind == RoutingNetKind::PNnet
                ? net.virtual_source_node
                : source_nodes[source_index];
            const int sink_node = sink_nodes[demand_index];
            const auto pair_key = PairKey {net.net_id, demand.demand_id, source_index};
            const PairRoutingState* existing = nullptr;
            if (problem_state != nullptr) {
                existing = find_pair_state(*problem_state, pair_key);
            }

            std::Vector<int> delay_values {};
            int member_shortest = -1;
            if (existing != nullptr && !existing->delays.empty()) {
                delay_values = existing->delays;
            }
            else if (net.kind == RoutingNetKind::PNnet) {
                const int shortest = pnnet_initial_delay(graph, scope, net, sink_node);
                if (shortest < 0) {
                    throw std::runtime_error(std::format(
                        "net {} demand {} has no scoped path from any candidate source to sink {}",
                        net.net_id,
                        demand.demand_id,
                        sink_node));
                }
                delay_values = {shortest};
                member_shortest = shortest;
            }
            else {
                const int shortest = bfs_shortest_delay(graph, scope, source_node, sink_node);
                if (shortest < 0) {
                    throw std::runtime_error(std::format(
                        "net {} demand {} has no scoped path from source {} to sink {}",
                        net.net_id,
                        demand.demand_id,
                        source_node,
                        sink_node));
                }
                delay_values = {shortest};
                member_shortest = shortest;
            }

            auto pair = PairDelayInfo {
                net.net_id,
                demand.demand_id,
                source_index,
                source_node,
                sink_node,
                delay_values,
                max_delay(delay_values),
                net.is_sync_bus ? (member_shortest >= 0 ? member_shortest : max_delay(delay_values)) : -1};
            member_pairs.push_back(std::move(pair));
        }

        if (net.is_sync_bus) {
            bool already_expanded = false;
            for (const auto& pair : member_pairs) {
                if (pair.delays.size() > 1) {
                    already_expanded = true;
                    break;
                }
            }
            if (!already_expanded) {
                int bus_d_min = 0;
                for (const auto& pair : member_pairs) {
                    bus_d_min = std::max(bus_d_min, pair.member_shortest_delay);
                }
                for (auto& pair : member_pairs) {
                    set_pair_delay(pair, std::Vector<int> {bus_d_min});
                }
            }
        }

        if (net.kind == RoutingNetKind::PNnet) {
            const auto key = std::tuple {net.net_id, std::size_t {0}, net_index};
            if (!source_key_to_index.contains(key)) {
                int d_max = 0;
                for (const auto& pair : member_pairs) {
                    d_max = std::max(d_max, max_delay(pair.delays));
                }
                const int source_node = net.virtual_source_node;
                auto reach = bfs_reachability(graph, scope, source_node, d_max);
                source_key_to_index.emplace(key, result.sources.size());
                result.sources.push_back(SourceDelayReachability {
                    net.net_id,
                    0,
                    source_node,
                    net_index,
                    d_max,
                    std::move(reach)});
            }
        }
        else {
            for (std::size_t source_index = 0; source_index < net.sources.size(); ++source_index) {
                const auto key = std::tuple {net.net_id, source_index, net_index};
                if (source_key_to_index.contains(key)) {
                    continue;
                }
                int d_max = 0;
                for (const auto& pair : member_pairs) {
                    if (pair.source_index == source_index) {
                        d_max = std::max(d_max, max_delay(pair.delays));
                    }
                }
                const int source_node = source_nodes[source_index];
                auto reach = bfs_reachability(graph, scope, source_node, d_max);
                source_key_to_index.emplace(key, result.sources.size());
                result.sources.push_back(SourceDelayReachability {
                    net.net_id,
                    source_index,
                    source_node,
                    net_index,
                    d_max,
                    std::move(reach)});
            }
        }

        for (auto& pair : member_pairs) {
            if (problem_state != nullptr) {
                copy_delays_to_state(*problem_state, pair);
            }
            result.pairs.push_back(std::move(pair));
        }
    }
    return result;
}

auto log_delay_precompute(
    const std::Vector<RoutingNet>& nets,
    const DelayPrecomputeResult& delays,
    int verbose_level
) -> void {
    if (verbose_level < 1) {
        return;
    }
    for (const auto& pair : delays.pairs) {
        const auto net_it = std::find_if(
            nets.begin(),
            nets.end(),
            [&](const RoutingNet& net) { return net.net_id == pair.net_id; });
        const auto net_name = net_it == nets.end() ? std::String {"?"} : net_it->name;
        debug::info_fmt(
            "delay net=\"{}\" id={} demand={} src={} snk={} delays={}",
            net_name,
            pair.net_id,
            pair.demand_id,
            pair.source_node,
            pair.sink_node,
            [&] {
                auto text = std::String {};
                for (std::size_t i = 0; i < pair.delays.size(); ++i) {
                    if (i != 0) {
                        text += ",";
                    }
                    text += std::to_string(pair.delays[i]);
                }
                return text;
            }());
    }
    for (const auto& net : nets) {
        if (!net.is_sync_bus) {
            continue;
        }
        int bus_d_min = -1;
        auto member_shortest = std::Vector<int> {};
        for (const auto& pair : delays.pairs) {
            if (pair.net_id != net.net_id) {
                continue;
            }
            bus_d_min = pair.target_delay;
            if (pair.member_shortest_delay >= 0) {
                member_shortest.push_back(pair.member_shortest_delay);
            }
        }
        auto member_text = std::String {};
        for (std::size_t i = 0; i < member_shortest.size(); ++i) {
            if (i != 0) {
                member_text += ", ";
            }
            member_text += std::to_string(member_shortest[i]);
        }
        debug::info_fmt(
            "delay bus net=\"{}\" id={} bus_d_min={} member_shortest=[{}] members={}",
            net.name,
            net.net_id,
            bus_d_min,
            member_text,
            net.demands.size());
    }
}

} // namespace PR_tool
