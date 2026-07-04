#include "delay/pair_delay_precompute.hh"

#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"
#include "scope/pair_routing_state.hh"

#include <algorithm>
#include <cstdint>
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

auto bfs_shortest_distances(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    int source_node
) -> std::Vector<int> {
    if (!scope_node_in_bounds(scope, source_node)) {
        throw std::invalid_argument("BFS source is outside scope");
    }
    const auto source_offset =
        static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(source_node)]);
    auto distances = std::Vector<int>(scope.node_ids.size(), -1);
    auto queue = std::queue<std::size_t> {};
    queue.push(source_offset);
    distances[source_offset] = 0;

    while (!queue.empty()) {
        const auto node_offset = queue.front();
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
            const auto next_offset_u = static_cast<std::size_t>(next_offset);
            if (distances[next_offset_u] >= 0) {
                continue;
            }
            distances[next_offset_u] = distances[node_offset] + 1;
            queue.push(next_offset_u);
        }
    }
    return distances;
}

auto delay_slot(
    const std::size_t node_offset,
    const int delay,
    const int d_max
) -> std::size_t {
    return node_offset * (static_cast<std::size_t>(d_max) + 1)
        + static_cast<std::size_t>(delay);
}

auto unit_bit(const std::size_t unit) -> std::uint16_t {
    return unit < 16 ? static_cast<std::uint16_t>(std::uint16_t {1} << unit) : 0;
}

auto compute_source_unit_mask(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const int source_node
) -> std::uint16_t {
    if (net.kind == RoutingNetKind::PNnet) {
        std::uint16_t mask = 0;
        for (const auto& source_ref : net.sources) {
            const int candidate = resolve_graph_node(graph, source_ref);
            if (candidate < 0) {
                continue;
            }
            const auto& node = graph.nodes[static_cast<std::size_t>(candidate)];
            if (node.kind == UnifiedNodeKind::Track) {
                mask = static_cast<std::uint16_t>(mask | unit_bit(node.unit));
            }
        }
        return mask;
    }
    const auto& node = graph.nodes[static_cast<std::size_t>(source_node)];
    if (node.kind == UnifiedNodeKind::Track) {
        return unit_bit(node.unit);
    }
    return std::uint16_t {0xffff};
}

auto node_unit_eligible(
    const UnifiedNode& node,
    const std::uint16_t source_unit_mask
) -> bool {
    if (node.kind == UnifiedNodeKind::Track) {
        return (source_unit_mask & unit_bit(node.unit)) != 0;
    }
    if (node.kind == UnifiedNodeKind::VLine) {
        const std::size_t local_unit = node.line_index % 8;
        const auto pair_mask = static_cast<std::uint16_t>(
            unit_bit(local_unit) | unit_bit(local_unit + 8));
        return (source_unit_mask & pair_mask) != 0;
    }
    return true;
}

auto arc_unit_eligible(
    const UnifiedGraph& graph,
    const UnifiedArc& arc,
    const std::uint16_t source_unit_mask
) -> bool {
    return node_unit_eligible(
               graph.nodes[static_cast<std::size_t>(arc.u)],
               source_unit_mask)
        && node_unit_eligible(
               graph.nodes[static_cast<std::size_t>(arc.v)],
               source_unit_mask);
}

auto compute_active_delay_mask(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    const int source_node,
    const int d_max,
    const std::Vector<PairDelayInfo>& pairs,
    const std::size_t source_index,
    const std::uint16_t source_unit_mask
) -> std::Vector<std::uint8_t> {
    const auto slot_count =
        scope.node_ids.size() * (static_cast<std::size_t>(d_max) + 1);
    auto forward = std::Vector<std::uint8_t>(slot_count, 0);
    auto active = std::Vector<std::uint8_t>(slot_count, 0);
    const auto source_offset =
        static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(source_node)]);
    forward[delay_slot(source_offset, 0, d_max)] = 1;

    for (int delay = 1; delay <= d_max; ++delay) {
        for (const int arc_id : scope.arc_ids) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!arc_unit_eligible(graph, arc, source_unit_mask)) {
                continue;
            }
            const auto u_offset =
                static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(arc.u)]);
            const auto v_offset =
                static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(arc.v)]);
            if (forward[delay_slot(u_offset, delay - 1, d_max)] != 0) {
                forward[delay_slot(v_offset, delay, d_max)] = 1;
            }
        }
    }

    for (const auto& pair : pairs) {
        if (pair.source_index != source_index) {
            continue;
        }
        const int sink_offset_i =
            scope.node_offset[static_cast<std::size_t>(pair.sink_node)];
        if (sink_offset_i < 0) {
            continue;
        }
        const auto sink_offset = static_cast<std::size_t>(sink_offset_i);
        auto backward = std::Vector<std::uint8_t>(slot_count, 0);
        backward[delay_slot(sink_offset, 0, d_max)] = 1;
        for (int remaining = 1; remaining <= d_max; ++remaining) {
            for (const int arc_id : scope.arc_ids) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                if (!arc_unit_eligible(graph, arc, source_unit_mask)) {
                    continue;
                }
                const auto u_offset =
                    static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(arc.u)]);
                const auto v_offset =
                    static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(arc.v)]);
                if (backward[delay_slot(v_offset, remaining - 1, d_max)] != 0) {
                    backward[delay_slot(u_offset, remaining, d_max)] = 1;
                }
            }
        }
        for (const int target_delay : pair.delays) {
            if (target_delay < 0 || target_delay > d_max) {
                continue;
            }
            for (int delay = 0; delay <= target_delay; ++delay) {
                const int remaining = target_delay - delay;
                for (std::size_t node_offset = 0;
                     node_offset < scope.node_ids.size();
                     ++node_offset) {
                    const auto slot = delay_slot(node_offset, delay, d_max);
                    if (forward[slot] != 0
                        && backward[delay_slot(node_offset, remaining, d_max)] != 0) {
                        active[slot] = 1;
                    }
                }
            }
        }
    }

    active[delay_slot(source_offset, 0, d_max)] = 1;
    for (int delay = 1; delay <= d_max; ++delay) {
        active[delay_slot(source_offset, delay, d_max)] = 0;
    }
    return active;
}

auto count_unit_eligible_nodes(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    const std::uint16_t source_unit_mask
) -> std::size_t {
    return static_cast<std::size_t>(std::count_if(
        scope.node_ids.begin(),
        scope.node_ids.end(),
        [&](const int node_id) {
            return node_unit_eligible(
                graph.nodes[static_cast<std::size_t>(node_id)],
                source_unit_mask);
        }));
}

auto count_unit_eligible_tob_arcs(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    const std::uint16_t source_unit_mask
) -> std::size_t {
    return static_cast<std::size_t>(std::count_if(
        scope.arc_ids.begin(),
        scope.arc_ids.end(),
        [&](const int arc_id) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            return arc.physical_switch_kind != PhysicalSwitchKind::None
                && arc_unit_eligible(graph, arc, source_unit_mask);
        }));
}

} // namespace

auto bfs_shortest_delay(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    int source_node,
    int sink_node
) -> int {
    if (!scope_node_in_bounds(scope, sink_node)) {
        return -1;
    }
    const auto distances = bfs_shortest_distances(graph, scope, source_node);
    const auto sink_offset =
        static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(sink_node)]);
    return distances[sink_offset];
}

auto compute_pair_delays(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    RoutingProblemState* problem_state
) -> DelayPrecomputeResult {
    auto result = DelayPrecomputeResult {};
    auto source_key_to_index = std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::size_t> {};
    auto shortest_by_source =
        std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::Vector<int>> {};

    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        const auto& net = nets[net_index];
        const auto& scope = scopes[net_index];
        const auto [source_nodes, sink_nodes] = resolve_endpoint_nodes(graph, net);
        const auto shortest_for = [&](const std::size_t source_index,
                                      const int source_node,
                                      const int sink_node) -> int {
            const auto key = std::tuple {net.net_id, source_index, net_index};
            auto [it, inserted] = shortest_by_source.try_emplace(key);
            if (inserted) {
                it->second = bfs_shortest_distances(graph, scope, source_node);
            }
            if (!scope_node_in_bounds(scope, sink_node)) {
                return -1;
            }
            const auto sink_offset =
                static_cast<std::size_t>(scope.node_offset[static_cast<std::size_t>(sink_node)]);
            return it->second[sink_offset];
        };

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
                const int shortest = shortest_for(0, source_node, sink_node);
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
                const int shortest = shortest_for(source_index, source_node, sink_node);
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

        for (const auto& member : member_pairs) {
            const auto key = std::tuple {net.net_id, member.source_index, net_index};
            if (!source_key_to_index.contains(key)) {
                int d_max = 0;
                for (const auto& pair : member_pairs) {
                    if (pair.source_index == member.source_index) {
                        d_max = std::max(d_max, max_delay(pair.delays));
                    }
                }
                const auto source_unit_mask =
                    compute_source_unit_mask(graph, net, member.source_node);
                source_key_to_index.emplace(key, result.sources.size());
                result.sources.push_back(SourceDelayDomain {
                    net.net_id,
                    member.source_index,
                    member.source_node,
                    net_index,
                    d_max,
                    source_unit_mask,
                    count_unit_eligible_nodes(graph, scope, source_unit_mask),
                    count_unit_eligible_tob_arcs(graph, scope, source_unit_mask),
                    compute_active_delay_mask(
                        graph,
                        scope,
                        member.source_node,
                        d_max,
                        member_pairs,
                        member.source_index,
                        source_unit_mask)});
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
