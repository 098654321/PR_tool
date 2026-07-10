#include "ilp_v15/v15_ilp_validate.hh"

#include "sat/routing_path_log.hh"

#include <algorithm>
#include <format>
#include <map>
#include <set>

namespace PR_tool {

namespace {

auto net_by_id(const std::Vector<RoutingNet>& nets, std::size_t net_id) -> const RoutingNet* {
    const auto it = std::find_if(
        nets.begin(),
        nets.end(),
        [&](const RoutingNet& net) { return net.net_id == net_id; });
    return it == nets.end() ? nullptr : &*it;
}

auto scope_by_id(
    const std::Vector<UnifiedSatNetScope>& scopes,
    std::size_t net_id
) -> const UnifiedSatNetScope* {
    const auto it = std::find_if(
        scopes.begin(),
        scopes.end(),
        [&](const UnifiedSatNetScope& scope) { return scope.net_id == net_id; });
    return it == scopes.end() ? nullptr : &*it;
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

auto add_violation(V15ValidationReport& report, std::String message) -> void {
    report.violations.push_back(std::move(message));
}

} // namespace

auto validate_v15_routing_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const SatRoutingResult& result
) -> V15ValidationReport {
    auto report = V15ValidationReport {};
    auto demand_counts = std::map<std::pair<std::size_t, std::size_t>, std::size_t> {};
    auto node_owner = std::map<int, std::pair<std::size_t, std::size_t>> {};
    auto used_switches = std::set<int> {};
    auto switch_exemplar = std::map<int, const UnifiedArc*> {};
    auto bus_lengths = std::map<std::size_t, std::set<std::size_t>> {};

    for (const auto& path : result.paths) {
        ++report.checked_paths;
        ++demand_counts[{path.net_id, path.demand_id}];
        const auto* net = net_by_id(nets, path.net_id);
        const auto* scope = scope_by_id(scopes, path.net_id);
        if (net == nullptr || scope == nullptr || path.demand_id >= (net == nullptr ? 0 : net->demands.size())) {
            add_violation(report, std::format(
                "path net={} demand={} has no routing metadata",
                path.net_id,
                path.demand_id));
            continue;
        }
        const auto& demand = net->demands[path.demand_id];
        const int sink = resolve_graph_node(graph, demand.sink);
        int source = -1;
        if (net->kind == RoutingNetKind::PNnet) {
            source = path.physical_source_node;
        }
        else if (path.source_index < net->sources.size()) {
            source = resolve_graph_node(graph, net->sources[path.source_index]);
        }
        if (path.node_path.empty() || path.node_path.front() != source || path.node_path.back() != sink) {
            add_violation(report, std::format(
                "path net={} demand={} has invalid endpoints",
                path.net_id,
                path.demand_id));
            continue;
        }
        const auto owner = std::pair {
            path.net_id,
            net->is_sync_bus ? path.demand_id : std::size_t {0}};
        for (const int node : path.node_path) {
            if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size()) {
                add_violation(report, std::format("path uses invalid node {}", node));
                continue;
            }
            if (static_cast<std::size_t>(node) >= scope->node_offset.size()
                || scope->node_offset[static_cast<std::size_t>(node)] < 0) {
                add_violation(report, std::format(
                    "path net={} demand={} leaves its scope at node {}",
                    path.net_id,
                    path.demand_id,
                    node));
            }
            if (graph.nodes[static_cast<std::size_t>(node)].kind == UnifiedNodeKind::VirtualSource) {
                add_violation(report, "extracted path contains a virtual source");
                continue;
            }
            const auto [it, inserted] = node_owner.emplace(node, owner);
            if (!inserted && it->second != owner) {
                add_violation(report, std::format("physical node {} has multiple owners", node));
            }
        }
        for (std::size_t i = 1; i < path.node_path.size(); ++i) {
            const int arc_id = find_arc_id(graph, path.node_path[i - 1], path.node_path[i]);
            if (arc_id < 0) {
                add_violation(report, std::format(
                    "path net={} demand={} has a missing graph arc",
                    path.net_id,
                    path.demand_id));
                continue;
            }
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (static_cast<std::size_t>(arc_id) >= scope->arc_offset.size()
                || scope->arc_offset[static_cast<std::size_t>(arc_id)] < 0) {
                add_violation(report, std::format(
                    "path net={} demand={} uses an arc outside its scope",
                    path.net_id,
                    path.demand_id));
            }
            if (arc.physical_switch_id >= 0) {
                used_switches.insert(arc.physical_switch_id);
                switch_exemplar.try_emplace(arc.physical_switch_id, &arc);
            }
            if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack
                && arc.mode_group_id >= 0) {
                const auto mode = result.vline_mode_straight_by_group.find(
                    static_cast<std::size_t>(arc.mode_group_id));
                if (mode == result.vline_mode_straight_by_group.end()
                    || (arc.is_vline_track_straight && !mode->second)
                    || (arc.is_vline_track_swap && mode->second)) {
                    add_violation(report, std::format(
                        "vline-track mode group {} conflicts with path",
                        arc.mode_group_id));
                }
            }
        }
        if (net->is_sync_bus) {
            bus_lengths[net->net_id].insert(path_wirelength(graph, path.node_path));
        }
    }

    for (const auto& net : nets) {
        for (const auto& demand : net.demands) {
            if (demand_counts[{net.net_id, demand.demand_id}] != 1) {
                add_violation(report, std::format(
                    "net {} demand {} does not have exactly one path",
                    net.net_id,
                    demand.demand_id));
            }
        }
    }
    for (const auto& [net_id, lengths] : bus_lengths) {
        if (lengths.size() > 1) {
            add_violation(report, std::format("bus net {} has unequal L", net_id));
        }
    }
    const auto reported_switches = std::set<int> {
        result.used_tob_switch_ids.begin(), result.used_tob_switch_ids.end()};
    if (reported_switches != used_switches) {
        add_violation(report, "reported TOB switches do not match extracted paths");
    }
    auto partial_matching = std::map<std::pair<int, int>, std::size_t> {};
    for (const auto& [switch_id, arc] : switch_exemplar) {
        (void)switch_id;
        const auto u_kind = graph.nodes[static_cast<std::size_t>(arc->u)].kind;
        const auto v_kind = graph.nodes[static_cast<std::size_t>(arc->v)].kind;
        if (u_kind == UnifiedNodeKind::Bump && v_kind == UnifiedNodeKind::HLine) {
            ++partial_matching[{0, arc->u}];
            ++partial_matching[{1, arc->v}];
        }
        else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::Bump) {
            ++partial_matching[{0, arc->v}];
            ++partial_matching[{1, arc->u}];
        }
        else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::VLine) {
            ++partial_matching[{2, arc->u}];
            ++partial_matching[{3, arc->v}];
        }
        else if (u_kind == UnifiedNodeKind::VLine && v_kind == UnifiedNodeKind::HLine) {
            ++partial_matching[{2, arc->v}];
            ++partial_matching[{3, arc->u}];
        }
    }
    for (const auto& [key, count] : partial_matching) {
        if (count > 1) {
            add_violation(report, std::format(
                "TOB partial-matching endpoint class={} node={} uses {} switches",
                key.first,
                key.second,
                count));
        }
    }
    if (total_wirelength(graph, result) != result.total_wirelength) {
        add_violation(report, "reported total wirelength does not match extracted paths");
    }
    report.ok = report.violations.empty();
    return report;
}

} // namespace PR_tool
