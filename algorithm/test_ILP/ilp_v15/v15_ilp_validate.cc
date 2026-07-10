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

auto parent_solution_by_id(
    const V15IlpModelResult& result,
    std::size_t parent_id
) -> const V15ParentModelSolution* {
    const auto it = std::find_if(
        result.parents.begin(),
        result.parents.end(),
        [&](const V15ParentModelSolution& solution) { return solution.parent_id == parent_id; });
    return it == result.parents.end() ? nullptr : &*it;
}

auto segment_solution_by_id(
    const V15IlpModelResult& result,
    std::size_t segment_id
) -> const V15SegmentModelSolution* {
    const auto it = std::find_if(
        result.segments.begin(),
        result.segments.end(),
        [&](const V15SegmentModelSolution& solution) { return solution.segment_id == segment_id; });
    return it == result.segments.end() ? nullptr : &*it;
}

auto validate_v15_model_solution(
    const UnifiedGraph& graph,
    const V15PrepareResult& prepared,
    const V15IlpModelResult& model_result,
    V15ValidationReport& report
) -> void {
    for (const auto& parent : prepared.parents) {
        const auto* parent_solution = parent_solution_by_id(model_result, parent.parent_id);
        if (parent_solution == nullptr) {
            add_violation(report, std::format("v15 parent {} has no model solution", parent.parent_id));
            continue;
        }
        auto parent_scope_arcs = std::set<int> {};
        auto flow_supported_arcs = std::set<int> {};
        auto selected_virtual_tracks = std::set<int> {};

        for (const std::size_t segment_id : parent.segment_ids) {
            if (segment_id >= prepared.segments.size()) {
                add_violation(report, std::format(
                    "v15 parent {} references missing segment {}", parent.parent_id, segment_id));
                continue;
            }
            const auto& segment = prepared.segments[segment_id];
            const auto* segment_solution = segment_solution_by_id(model_result, segment_id);
            if (segment_solution == nullptr) {
                add_violation(report, std::format("v15 segment {} has no flow solution", segment_id));
                continue;
            }
            const auto scope_arcs = std::set<int>(segment.scope.arc_ids.begin(), segment.scope.arc_ids.end());
            const auto scope_nodes = std::set<int>(segment.scope.node_ids.begin(), segment.scope.node_ids.end());
            parent_scope_arcs.insert(scope_arcs.begin(), scope_arcs.end());
            auto flow_in = std::map<int, int> {};
            auto flow_out = std::map<int, int> {};
            for (const int arc_id : segment_solution->flow_arc_ids) {
                if (!scope_arcs.contains(arc_id)
                    || arc_id < 0
                    || static_cast<std::size_t>(arc_id) >= graph.arcs.size()) {
                    add_violation(report, std::format(
                        "v15 segment {} flow uses arc {} outside its scope", segment_id, arc_id));
                    continue;
                }
                const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                ++flow_out[arc.u];
                ++flow_in[arc.v];
                flow_supported_arcs.insert(arc_id);
            }
            for (const int node : scope_nodes) {
                const int out = flow_out[node];
                const int in = flow_in[node];
                const int expected = node == segment.endpoint_a ? 1 : node == segment.endpoint_b ? -1 : 0;
                if (out - in != expected) {
                    add_violation(report, std::format(
                        "v15 segment {} violates unit flow at node {}: out-in={} expected={}",
                        segment_id,
                        node,
                        out - in,
                        expected));
                }
            }
        }

        for (const int arc_id : parent_solution->selected_arc_ids) {
            if (!parent_scope_arcs.contains(arc_id)) {
                add_violation(report, std::format(
                    "v15 parent {} selects arc {} outside all segment scopes",
                    parent.parent_id,
                    arc_id));
                continue;
            }
            if (!flow_supported_arcs.contains(arc_id)) {
                add_violation(report, std::format(
                    "v15 parent {} selects arc {} without segment flow support",
                    parent.parent_id,
                    arc_id));
            }
            if (arc_id >= 0 && static_cast<std::size_t>(arc_id) < graph.arcs.size()) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                if (graph.nodes[static_cast<std::size_t>(arc.u)].kind == UnifiedNodeKind::VirtualSource) {
                    selected_virtual_tracks.insert(arc.v);
                    if (arc.u != parent.root_node
                        || std::find(
                               parent.fixed_track_nodes.begin(),
                               parent.fixed_track_nodes.end(),
                               arc.v)
                            == parent.fixed_track_nodes.end()) {
                        add_violation(report, std::format(
                            "v15 parent {} selects non-SAT PN virtual arc {}->{}",
                            parent.parent_id,
                            arc.u,
                            arc.v));
                    }
                }
            }
        }
        for (const int track : parent.fixed_track_nodes) {
            if (!parent_solution->used_node_ids.contains(track)) {
                add_violation(report, std::format(
                    "v15 parent {} does not use fixed SAT-selected track {}",
                    parent.parent_id,
                    track));
            }
        }
        if (!parent.fixed_track_nodes.empty()
            && selected_virtual_tracks
                != std::set<int>(parent.fixed_track_nodes.begin(), parent.fixed_track_nodes.end())) {
            add_violation(report, std::format(
                "v15 parent {} virtual root tracks differ from SAT-selected tracks",
                parent.parent_id));
        }
    }
}

} // namespace

auto validate_v15_routing_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const V15PrepareResult& prepared,
    const V15IlpModelResult& model_result,
    const SatRoutingResult& result,
    const std::set<std::size_t>& v15_selected_net_ids
) -> V15ValidationReport {
    auto report = V15ValidationReport {};
    validate_v15_model_solution(graph, prepared, model_result, report);
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
        const bool skip_scope_checks = v15_selected_net_ids.contains(path.net_id);
        for (const int node : path.node_path) {
            if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size()) {
                add_violation(report, std::format("path uses invalid node {}", node));
                continue;
            }
            if (!skip_scope_checks
                && (static_cast<std::size_t>(node) >= scope->node_offset.size()
                    || scope->node_offset[static_cast<std::size_t>(node)] < 0)) {
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
            if (!skip_scope_checks
                && (static_cast<std::size_t>(arc_id) >= scope->arc_offset.size()
                    || scope->arc_offset[static_cast<std::size_t>(arc_id)] < 0)) {
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
