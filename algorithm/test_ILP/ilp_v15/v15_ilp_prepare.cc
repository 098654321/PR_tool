#include "ilp_v15/v15_ilp_prepare.hh"

#include <algorithm>
#include <format>
#include <map>
#include <queue>
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
    if (source_index >= net.sources.size()) {
        throw std::invalid_argument(std::format(
            "v15 net {} demand {} source index is invalid",
            net.net_id,
            demand.demand_id));
    }
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

auto build_v15_commodities(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const std::set<std::size_t>& selected_net_ids
) -> std::Vector<IlpCommodity> {
    auto out = std::Vector<IlpCommodity> {};
    for (const auto& net : nets) {
        if (!selected_net_ids.contains(net.net_id)) {
            continue;
        }
        const auto scope_index = scope_index_for_net(scopes, net.net_id);
        if (net.is_sync_bus) {
            for (const auto& demand : net.demands) {
                const auto [source_node, source_index] = source_node_for(graph, net, demand);
                out.push_back(IlpCommodity {
                    out.size(),
                    net.net_id,
                    scope_index,
                    source_node,
                    {sink_node_for(graph, net, demand)},
                    {demand.demand_id},
                    {source_index},
                    1,
                    true});
            }
            continue;
        }
        if (net.demands.empty()) {
            throw std::invalid_argument(std::format("v15 net {} has no demand", net.net_id));
        }
        const auto [source_node, first_source_index] =
            source_node_for(graph, net, net.demands.front());
        auto commodity = IlpCommodity {};
        commodity.commodity_id = out.size();
        commodity.routing_net_id = net.net_id;
        commodity.scope_index = scope_index;
        commodity.source_node = source_node;
        for (const auto& demand : net.demands) {
            const auto [candidate_source, source_index] = source_node_for(graph, net, demand);
            if (candidate_source != source_node) {
                throw std::invalid_argument(std::format(
                    "v15 non-bus net {} has multiple physical roots",
                    net.net_id));
            }
            commodity.sink_nodes.push_back(sink_node_for(graph, net, demand));
            commodity.demand_ids.push_back(demand.demand_id);
            commodity.source_indices.push_back(
                net.kind == RoutingNetKind::PNnet ? 0 : source_index);
        }
        (void)first_source_index;
        commodity.k = static_cast<int>(commodity.sink_nodes.size());
        out.push_back(std::move(commodity));
    }
    return out;
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
            if (node >= 0 && static_cast<std::size_t>(node) < graph.nodes.size()
                && graph.nodes[static_cast<std::size_t>(node)].kind
                    != UnifiedNodeKind::VirtualSource) {
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

auto build_v15_mip_start(
    const UnifiedGraph& graph,
    const std::Vector<IlpCommodity>& commodities,
    const SatRoutingResult& sat_result
) -> V15MipStart {
    auto out = V15MipStart {};
    for (const auto& [group, straight] : sat_result.vline_mode_straight_by_group) {
        out.mode_straight.emplace(static_cast<int>(group), straight);
    }

    for (const auto& commodity : commodities) {
        auto candidate_arc_ids = std::set<int> {};
        for (const auto& path : sat_result.paths) {
            if (path.net_id != commodity.routing_net_id
                || std::find(
                       commodity.demand_ids.begin(),
                       commodity.demand_ids.end(),
                       path.demand_id)
                    == commodity.demand_ids.end()) {
                continue;
            }
            int previous = commodity.source_node;
            if (!path.node_path.empty() && previous == path.node_path.front()) {
                previous = path.node_path.front();
            }
            for (const int node : path.node_path) {
                if (node == previous) {
                    continue;
                }
                const int arc_id = find_arc_id(graph, previous, node);
                if (arc_id < 0) {
                    return out;
                }
                candidate_arc_ids.insert(arc_id);
                previous = node;
            }
        }

        auto parent_arc = std::map<int, int> {};
        auto visited = std::set<int> {commodity.source_node};
        auto queue = std::queue<int> {};
        queue.push(commodity.source_node);
        while (!queue.empty()) {
            const int node = queue.front();
            queue.pop();
            for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(node)]) {
                if (!candidate_arc_ids.contains(arc_id)) {
                    continue;
                }
                const int next = graph.arcs[static_cast<std::size_t>(arc_id)].v;
                if (visited.insert(next).second) {
                    parent_arc.emplace(next, arc_id);
                    queue.push(next);
                }
            }
        }

        auto& selected = out.selected_arc_ids[commodity.commodity_id];
        auto& used_nodes = out.used_node_ids[commodity.commodity_id];
        used_nodes.insert(commodity.source_node);
        for (const int sink : commodity.sink_nodes) {
            int current = sink;
            used_nodes.insert(current);
            while (current != commodity.source_node) {
                const auto parent_it = parent_arc.find(current);
                if (parent_it == parent_arc.end()) {
                    out.selected_arc_ids.clear();
                    out.flow_by_arc.clear();
                    out.used_node_ids.clear();
                    return out;
                }
                const int arc_id = parent_it->second;
                selected.insert(arc_id);
                ++out.flow_by_arc[{commodity.commodity_id, arc_id}];
                current = graph.arcs[static_cast<std::size_t>(arc_id)].u;
                used_nodes.insert(current);
            }
        }
    }
    out.available = true;
    return out;
}

} // namespace PR_tool
