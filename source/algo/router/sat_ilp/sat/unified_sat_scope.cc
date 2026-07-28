#include "sat/unified_sat_scope.hh"

#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

auto checked_endpoint(
    const UnifiedGraph& graph,
    const GraphNodeRef& ref,
    const std::size_t net_id,
    const std::string_view role
) -> int {
    const int node = resolve_graph_node(graph, ref);
    if (node < 0) {
        throw std::invalid_argument(
            std::format("net {} has unresolved {} endpoint", net_id, role));
    }
    return node;
}

} // namespace

auto build_scope(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const std::Vector<int>& source_nodes,
    const std::Vector<int>& sink_nodes,
    const std::Vector<int>& extra_nodes
) -> UnifiedSatNetScope {
    auto included = std::Vector<bool>(graph.nodes.size(), false);
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        if (node_in_scope(graph, node, net.scope_bbox)) {
            included[static_cast<std::size_t>(node)] = true;
        }
    }
    for (const int node : source_nodes) {
        if (node >= 0 && static_cast<std::size_t>(node) < included.size()) {
            included[static_cast<std::size_t>(node)] = true;
        }
    }
    for (const int node : sink_nodes) {
        if (node >= 0 && static_cast<std::size_t>(node) < included.size()) {
            included[static_cast<std::size_t>(node)] = true;
        }
    }
    for (const int node : extra_nodes) {
        if (node >= 0 && static_cast<std::size_t>(node) < included.size()) {
            included[static_cast<std::size_t>(node)] = true;
        }
    }

    auto scope = UnifiedSatNetScope {};
    scope.net_id = net.net_id;
    scope.node_offset.assign(graph.nodes.size(), -1);
    scope.arc_offset.assign(graph.arcs.size(), -1);
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        if (included[static_cast<std::size_t>(node)]) {
            scope.node_offset[static_cast<std::size_t>(node)] =
                static_cast<int>(scope.node_ids.size());
            scope.node_ids.push_back(node);
        }
    }
    for (int arc_id = 0; arc_id < static_cast<int>(graph.arcs.size()); ++arc_id) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        if (included[static_cast<std::size_t>(arc.u)]
            && included[static_cast<std::size_t>(arc.v)]) {
            scope.arc_offset[static_cast<std::size_t>(arc_id)] =
                static_cast<int>(scope.arc_ids.size());
            scope.arc_ids.push_back(arc_id);
        }
    }
    return scope;
}

auto resolve_endpoint_nodes(
    const UnifiedGraph& graph,
    const RoutingNet& net
) -> std::pair<std::Vector<int>, std::Vector<int>> {
    if (net.demands.empty()) {
        throw std::invalid_argument(std::format("net {} has no routing demands", net.net_id));
    }

    if (net.kind == RoutingNetKind::PNnet) {
        if (net.virtual_source_node < 0) {
            throw std::invalid_argument(std::format(
                "PNnet {} has no virtual source node",
                net.net_id));
        }
        auto source_nodes = std::Vector<int> {net.virtual_source_node};
        auto sink_nodes = std::Vector<int> {};
        sink_nodes.reserve(net.demands.size());
        for (const auto& demand : net.demands) {
            if (demand.candidate_source_indices.empty()) {
                throw std::invalid_argument(std::format(
                    "net {} demand {} has no candidate sources",
                    net.net_id,
                    demand.demand_id));
            }
            const int sink_node = checked_endpoint(graph, demand.sink, net.net_id, "sink");
            sink_nodes.push_back(sink_node);
        }
        return {std::move(source_nodes), std::move(sink_nodes)};
    }

    auto source_nodes = std::Vector<int> {};
    for (const auto& source : net.sources) {
        source_nodes.push_back(checked_endpoint(graph, source, net.net_id, "source"));
    }
    auto sink_nodes = std::Vector<int> {};
    for (const auto& demand : net.demands) {
        if (demand.candidate_source_indices.empty()) {
            throw std::invalid_argument(std::format(
                "net {} demand {} has no candidate sources",
                net.net_id,
                demand.demand_id));
        }
        if (demand.candidate_source_indices.size() != 1) {
            throw std::invalid_argument(std::format(
                "net {} demand {} has {} candidate sources; v14 requires exactly one",
                net.net_id,
                demand.demand_id,
                demand.candidate_source_indices.size()));
        }
        sink_nodes.push_back(checked_endpoint(graph, demand.sink, net.net_id, "sink"));
        const auto source_index = demand.candidate_source_indices.front();
        if (source_index >= source_nodes.size()) {
            throw std::invalid_argument(std::format(
                "net {} demand {} candidate source index {} is out of range",
                net.net_id,
                demand.demand_id,
                source_index));
        }
        if (source_nodes[source_index] == sink_nodes.back()) {
            throw std::invalid_argument(std::format(
                "net {} demand {} source equals sink",
                net.net_id,
                demand.demand_id));
        }
    }
    return {source_nodes, sink_nodes};
}

auto build_all_scopes(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets
) -> std::Vector<UnifiedSatNetScope> {
    auto scopes = std::Vector<UnifiedSatNetScope> {};
    scopes.reserve(nets.size());
    for (const auto& net : nets) {
        const auto [source_nodes, sink_nodes] = resolve_endpoint_nodes(graph, net);
        auto extra_nodes = std::Vector<int> {};
        if (net.kind == RoutingNetKind::PNnet) {
            for (const auto& source_ref : net.sources) {
                extra_nodes.push_back(resolve_graph_node(graph, source_ref));
            }
        }
        scopes.push_back(build_scope(graph, net, source_nodes, sink_nodes, extra_nodes));
    }
    return scopes;
}

} // namespace PR_tool
