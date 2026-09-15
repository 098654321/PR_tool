#include "sat/unified_sat_scope.hh"

#include "common/cob_unit_mask.hh"

#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

auto global_guide_unit_mask(const RoutingNet& net) -> std::uint16_t {
    if (net.kind == RoutingNetKind::Bnet) {
        if (!net.released_global_unit_sources.empty()) {
            return std::uint16_t {0xffff};
        }
        std::uint16_t mask = 0;
        for (const auto& [_, unit] : net.global_unit_by_source) {
            mask = static_cast<std::uint16_t>(mask | unit_bit(unit));
        }
        return mask == 0 ? std::uint16_t {0xffff} : mask;
    }
    std::uint16_t mask = 0;
    for (std::size_t source_index = 0; source_index < net.sources.size(); ++source_index) {
        if (!net.global_selected_pn_source_indices.empty()
            && !net.global_selected_pn_source_indices.contains(source_index)) {
            continue;
        }
        const auto& source = net.sources[source_index];
        if (source.kind == GraphNodeRef::Kind::Track) {
            mask = static_cast<std::uint16_t>(mask | unit_bit(map_track(source.track_index)));
        }
    }
    return mask == 0 ? std::uint16_t {0xffff} : mask;
}

auto terminal_tobs(const RoutingNet& net) -> std::set<std::size_t> {
    auto result = std::set<std::size_t> {};
    for (const auto& source : net.sources) {
        if (source.kind == GraphNodeRef::Kind::Bump) {
            result.insert(source.bump.TOB);
        }
    }
    for (const auto& demand : net.demands) {
        if (demand.sink.kind == GraphNodeRef::Kind::Bump) {
            result.insert(demand.sink.bump.TOB);
        }
    }
    return result;
}

auto node_in_global_guide(
    const int node_id,
    const UnifiedNode& node,
    const RoutingNet& net,
    const std::uint16_t unit_mask,
    const std::set<std::size_t>& tob_set
) -> bool {
    if (node.kind == UnifiedNodeKind::VirtualSource) {
        return net.kind == RoutingNetKind::PNnet
            && node_id == net.virtual_source_node;
    }
    if (node.kind == UnifiedNodeKind::Track) {
        return net.global_route_channels.contains(GlobalChannelCoord {
                   node.track_dir,
                   node.track_row,
                   node.track_col})
            && node_unit_eligible(node, unit_mask);
    }
    return tob_set.contains(node.tob) && node_unit_eligible(node, unit_mask);
}

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
    const auto guide_unit_mask = global_guide_unit_mask(net);
    const auto guide_terminal_tobs = terminal_tobs(net);
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        const bool include = net.has_global_route_guide
            ? node_in_global_guide(
                node,
                graph.nodes[static_cast<std::size_t>(node)],
                net,
                guide_unit_mask,
                guide_terminal_tobs)
            : node_in_scope(graph, node, net.scope_bbox);
        if (include) {
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
        if (net.has_global_route_guide
            && net.kind == RoutingNetKind::PNnet
            && is_virtual_source_arc(arc)
            && !net.global_selected_pn_source_indices.empty()) {
            bool selected_candidate = false;
            for (const auto source_index : net.global_selected_pn_source_indices) {
                if (source_index < net.sources.size()
                    && resolve_graph_node(graph, net.sources[source_index]) == arc.v) {
                    selected_candidate = true;
                    break;
                }
            }
            if (!selected_candidate) {
                continue;
            }
        }
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
            for (std::size_t source_index = 0; source_index < net.sources.size(); ++source_index) {
                if (!net.global_selected_pn_source_indices.empty()
                    && !net.global_selected_pn_source_indices.contains(source_index)) {
                    continue;
                }
                extra_nodes.push_back(resolve_graph_node(graph, net.sources[source_index]));
            }
        }
        scopes.push_back(build_scope(graph, net, source_nodes, sink_nodes, extra_nodes));
    }
    return scopes;
}

} // namespace PR_tool
