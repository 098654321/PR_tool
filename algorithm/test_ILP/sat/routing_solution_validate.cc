#include "sat/routing_solution_validate.hh"

#include <algorithm>
#include <debug/debug.hh>
#include <format>
#include <set>
#include <string_view>

namespace PR_tool {

namespace {

using PairDelayKey = std::tuple<std::size_t, std::size_t, std::size_t>;

auto violation_kind_name(const ViolationKind kind) -> std::string_view {
    switch (kind) {
        case ViolationKind::EndpointMismatch:
            return "EndpointMismatch";
        case ViolationKind::MissingArc:
            return "MissingArc";
        case ViolationKind::InvalidNodeTransition:
            return "InvalidNodeTransition";
        case ViolationKind::DelayReplayMismatch:
            return "DelayReplayMismatch";
        case ViolationKind::TrackBumpConflict:
            return "TrackBumpConflict";
        case ViolationKind::TobSwitchConflict:
            return "TobSwitchConflict";
        case ViolationKind::PartialMatchingConflict:
            return "PartialMatchingConflict";
        case ViolationKind::BnetUnitConflict:
            return "BnetUnitConflict";
        case ViolationKind::SyncBusDelayMismatch:
            return "SyncBusDelayMismatch";
        case ViolationKind::PnnetTrackRule:
            return "PnnetTrackRule";
        case ViolationKind::NodeOutOfScope:
            return "NodeOutOfScope";
    }
    return "Unknown";
}

auto add_violation(
    ValidationReport& report,
    ViolationKind kind,
    std::size_t net_id,
    std::size_t demand_id,
    std::size_t source_index,
    int node_id,
    int arc_id,
    const std::String& detail
) -> void {
    report.violations.push_back(Violation {
        kind,
        net_id,
        demand_id,
        source_index,
        node_id,
        arc_id,
        detail});
    report.category_counts[kind] += 1;
}

auto find_arc_id(const UnifiedGraph& graph, const int u, const int v) -> int {
    if (u < 0 || v < 0
        || static_cast<std::size_t>(u) >= graph.out_arc_ids.size()) {
        return -1;
    }
    for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        if (arc.v == v) {
            return arc_id;
        }
    }
    return -1;
}

auto d_literal_for(
    const UnifiedSatModel& model,
    const SourceDelayVars& source,
    const int node,
    const int delay
) -> int {
    const auto& scope = model.scopes[source.scope_index];
    if (node < 0 || static_cast<std::size_t>(node) >= scope.node_offset.size()) {
        return 0;
    }
    if (delay < 0 || delay > source.d_max) {
        return 0;
    }
    const int node_offset = scope.node_offset[static_cast<std::size_t>(node)];
    if (node_offset < 0) {
        return 0;
    }
    const int lit =
        source.d_var[static_cast<std::size_t>(node_offset)][static_cast<std::size_t>(delay)];
    return lit > 0 ? lit : 0;
}

auto track_unit_of_vline_track_arc(const UnifiedGraph& graph, const UnifiedArc& arc) -> int {
    if (static_cast<std::size_t>(arc.u) >= graph.nodes.size()
        || static_cast<std::size_t>(arc.v) >= graph.nodes.size()) {
        return -1;
    }
    const auto& u = graph.nodes[static_cast<std::size_t>(arc.u)];
    const auto& v = graph.nodes[static_cast<std::size_t>(arc.v)];
    if (u.kind == UnifiedNodeKind::Track) {
        return static_cast<int>(u.unit);
    }
    if (v.kind == UnifiedNodeKind::Track) {
        return static_cast<int>(v.unit);
    }
    return -1;
}

auto format_violation_counts(const ValidationReport& report) -> std::String {
    auto text = std::String {};
    for (const auto kind : {
             ViolationKind::EndpointMismatch,
             ViolationKind::MissingArc,
             ViolationKind::InvalidNodeTransition,
             ViolationKind::DelayReplayMismatch,
             ViolationKind::TrackBumpConflict,
             ViolationKind::TobSwitchConflict,
             ViolationKind::PartialMatchingConflict,
             ViolationKind::BnetUnitConflict,
             ViolationKind::SyncBusDelayMismatch,
             ViolationKind::PnnetTrackRule,
             ViolationKind::NodeOutOfScope}) {
        const auto it = report.category_counts.find(kind);
        const std::size_t count = it == report.category_counts.end() ? 0 : it->second;
        if (!text.empty()) {
            text += " ";
        }
        text += std::format("{}={}", violation_kind_name(kind), count);
    }
    return text;
}

} // namespace

auto validate_routing_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const UnifiedSatModel& model,
    const CadicalSession& session,
    const SatRoutingResult& out
) -> ValidationReport {
    auto report = ValidationReport {};
    report.net_count = nets.size();
    report.path_count = out.paths.size();
    report.mode_groups = out.vline_mode_straight_by_group.size();
    auto net_by_id = std::map<std::size_t, const RoutingNet*> {};
    for (const auto& net : nets) {
        net_by_id.emplace(net.net_id, &net);
    }

    auto source_by_key = std::map<std::pair<std::size_t, std::size_t>, const SourceDelayVars*> {};
    for (const auto& source : model.sources) {
        source_by_key.emplace(
            std::pair {source.net_id, source.source_index},
            &source);
    }
    auto pair_by_key = std::map<PairDelayKey, const PairDelayInfo*> {};
    for (const auto& pair : model.pair_delays) {
        pair_by_key.emplace(
            PairDelayKey {pair.net_id, pair.demand_id, pair.source_index},
            &pair);
    }

    auto demand_path_count = std::map<std::pair<std::size_t, std::size_t>, std::size_t> {};
    auto track_bump_owner = std::map<int, std::size_t> {};
    auto unique_track_bump_nodes = std::set<int> {};
    auto switch_nets_from_paths = std::map<int, std::set<std::size_t>> {};
    auto used_switches_from_paths = std::set<int> {};
    auto bnet_source_units = std::map<std::pair<std::size_t, std::size_t>, std::set<int>> {};
    auto sync_bus_hops = std::map<std::size_t, std::Vector<int>> {};

    for (const auto& path : out.paths) {
        demand_path_count[{path.net_id, path.demand_id}] += 1;
        const auto net_it = net_by_id.find(path.net_id);
        if (net_it == net_by_id.end()) {
            add_violation(
                report,
                ViolationKind::EndpointMismatch,
                path.net_id,
                path.demand_id,
                path.source_index,
                -1,
                -1,
                "path references unknown net");
            continue;
        }
        const auto& net = *net_it->second;
        const auto demand_it = std::find_if(
            net.demands.begin(),
            net.demands.end(),
            [&](const RoutingDemand& demand) { return demand.demand_id == path.demand_id; });
        if (demand_it == net.demands.end()) {
            add_violation(
                report,
                ViolationKind::EndpointMismatch,
                path.net_id,
                path.demand_id,
                path.source_index,
                -1,
                -1,
                "path references unknown demand");
            continue;
        }

        if (net.is_sync_bus && !path.node_path.empty()) {
            sync_bus_hops[net.net_id].push_back(static_cast<int>(path.node_path.size()) - 1);
        }

        const auto model_source_index = net.kind == RoutingNetKind::PNnet ? std::size_t {0} : path.source_index;
        const auto source_it = source_by_key.find({net.net_id, model_source_index});
        const SourceDelayVars* source = source_it == source_by_key.end() ? nullptr : source_it->second;
        const auto pair_it = pair_by_key.find(
            PairDelayKey {net.net_id, path.demand_id, model_source_index});
        const PairDelayInfo* pair = pair_it == pair_by_key.end() ? nullptr : pair_it->second;
        if (source == nullptr) {
            add_violation(
                report,
                ViolationKind::DelayReplayMismatch,
                net.net_id,
                path.demand_id,
                path.source_index,
                -1,
                -1,
                "source delay variable block is missing");
        }
        if (pair == nullptr) {
            add_violation(
                report,
                ViolationKind::DelayReplayMismatch,
                net.net_id,
                path.demand_id,
                path.source_index,
                -1,
                -1,
                "pair delay metadata is missing");
        }

        int expected_source_node = -1;
        if (net.kind == RoutingNetKind::PNnet) {
            if (path.source_index != 0) {
                add_violation(
                    report,
                    ViolationKind::PnnetTrackRule,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    -1,
                    -1,
                    "PNnet path must use source_index=0");
            }
            expected_source_node = path.physical_source_node;
            auto candidate_source_nodes = std::set<int> {};
            for (const auto& source_ref : net.sources) {
                const int node = resolve_graph_node(graph, source_ref);
                if (node >= 0) {
                    candidate_source_nodes.insert(node);
                }
            }
            if (expected_source_node < 0) {
                add_violation(
                    report,
                    ViolationKind::PnnetTrackRule,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    -1,
                    -1,
                    "PNnet path is missing physical_source_node");
            }
            else if (!candidate_source_nodes.contains(expected_source_node)) {
                add_violation(
                    report,
                    ViolationKind::PnnetTrackRule,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    expected_source_node,
                    -1,
                    "PNnet physical source is outside candidate source set");
            }
        }
        else {
            if (demand_it->candidate_source_indices.empty()) {
                add_violation(
                    report,
                    ViolationKind::EndpointMismatch,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    -1,
                    -1,
                    "demand has no candidate sources");
            }
            else {
                const auto expected_source_index = demand_it->candidate_source_indices.front();
                if (path.source_index != expected_source_index) {
                    add_violation(
                        report,
                        ViolationKind::EndpointMismatch,
                        net.net_id,
                        path.demand_id,
                        path.source_index,
                        -1,
                        -1,
                        std::format(
                            "path source_index={} does not match demand source_index={}",
                            path.source_index,
                            expected_source_index));
                }
                if (path.source_index >= net.sources.size()) {
                    add_violation(
                        report,
                        ViolationKind::EndpointMismatch,
                        net.net_id,
                        path.demand_id,
                        path.source_index,
                        -1,
                        -1,
                        "path source_index is outside net source list");
                }
                else {
                    expected_source_node = resolve_graph_node(
                        graph,
                        net.sources[path.source_index]);
                }
            }
        }

        const int expected_sink_node = resolve_graph_node(graph, demand_it->sink);
        if (path.node_path.empty()) {
            add_violation(
                report,
                ViolationKind::EndpointMismatch,
                net.net_id,
                path.demand_id,
                path.source_index,
                -1,
                -1,
                "path node list is empty");
            continue;
        }

        if (expected_source_node >= 0 && path.node_path.front() != expected_source_node) {
            add_violation(
                report,
                ViolationKind::EndpointMismatch,
                net.net_id,
                path.demand_id,
                path.source_index,
                path.node_path.front(),
                -1,
                std::format(
                    "path starts at node {} but expected {}",
                    path.node_path.front(),
                    expected_source_node));
        }
        if (expected_sink_node >= 0 && path.node_path.back() != expected_sink_node) {
            add_violation(
                report,
                ViolationKind::EndpointMismatch,
                net.net_id,
                path.demand_id,
                path.source_index,
                path.node_path.back(),
                -1,
                std::format(
                    "path ends at node {} but expected sink {}",
                    path.node_path.back(),
                    expected_sink_node));
        }

        const int delay_offset = net.kind == RoutingNetKind::PNnet ? 1 : 0;
        if (source != nullptr && net.kind == RoutingNetKind::PNnet) {
            const int lit = d_literal_for(model, *source, source->source_node, 0);
            if (lit > 0 && !session.value(lit)) {
                add_violation(
                    report,
                    ViolationKind::DelayReplayMismatch,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    source->source_node,
                    -1,
                    "D(virtual_source,0) is false");
            }
        }

        for (std::size_t i = 0; i < path.node_path.size(); ++i) {
            const int node = path.node_path[i];
            if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size()) {
                add_violation(
                    report,
                    ViolationKind::InvalidNodeTransition,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    node,
                    -1,
                    "path contains node outside graph range");
                continue;
            }
            const auto& graph_node = graph.nodes[static_cast<std::size_t>(node)];
            if (graph_node.kind == UnifiedNodeKind::VirtualSource) {
                add_violation(
                    report,
                    ViolationKind::InvalidNodeTransition,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    node,
                    -1,
                    "extracted path must not contain virtual source node");
            }
            if (net.has_scope_bbox && !node_in_scope(graph, node, net.scope_bbox)) {
                add_violation(
                    report,
                    ViolationKind::NodeOutOfScope,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    node,
                    -1,
                    "path node lies outside net scope bbox");
            }
            if (graph_node.kind == UnifiedNodeKind::Track
                || graph_node.kind == UnifiedNodeKind::Bump) {
                unique_track_bump_nodes.insert(node);
                const auto owner_it = track_bump_owner.find(node);
                if (owner_it == track_bump_owner.end()) {
                    track_bump_owner.emplace(node, net.net_id);
                }
                else if (owner_it->second != net.net_id) {
                    add_violation(
                        report,
                        ViolationKind::TrackBumpConflict,
                        net.net_id,
                        path.demand_id,
                        path.source_index,
                        node,
                        -1,
                        std::format(
                            "node {} is shared by net {} and net {}",
                            node,
                            owner_it->second,
                            net.net_id));
                }
            }

            if (source != nullptr) {
                const int replay_delay = static_cast<int>(i) + delay_offset;
                const int d_lit = d_literal_for(model, *source, node, replay_delay);
                if (d_lit > 0 && !session.value(d_lit)) {
                    add_violation(
                        report,
                        ViolationKind::DelayReplayMismatch,
                        net.net_id,
                        path.demand_id,
                        path.source_index,
                        node,
                        -1,
                        std::format(
                            "D(node={},delay={}) is false while replaying path",
                            node,
                            replay_delay));
                }
            }
        }

        for (std::size_t i = 1; i < path.node_path.size(); ++i) {
            const int u = path.node_path[i - 1];
            const int v = path.node_path[i];
            const int arc_id = find_arc_id(graph, u, v);
            if (arc_id < 0) {
                add_violation(
                    report,
                    ViolationKind::MissingArc,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    v,
                    -1,
                    std::format("path hop {} -> {} has no graph arc", u, v));
                continue;
            }
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (arc.is_virtual_source_arc) {
                add_violation(
                    report,
                    ViolationKind::InvalidNodeTransition,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    v,
                    arc_id,
                    "extracted path contains a virtual-source arc");
            }
            if (arc.physical_switch_id >= 0
                && arc.physical_switch_kind != PhysicalSwitchKind::None) {
                used_switches_from_paths.insert(arc.physical_switch_id);
                switch_nets_from_paths[arc.physical_switch_id].insert(net.net_id);
                if (net.kind == RoutingNetKind::Bnet
                    && arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack) {
                    const int unit = track_unit_of_vline_track_arc(graph, arc);
                    if (unit >= 0) {
                        bnet_source_units[{net.net_id, path.source_index}].insert(unit);
                    }
                }
            }

            if (source != nullptr && is_tob_arc(arc)) {
                const int replay_delay = static_cast<int>(i) + delay_offset;
                const int a_lit = tob_a_literal(
                    model,
                    source->model_source_index,
                    arc_id,
                    replay_delay);
                if (a_lit > 0 && !session.value(a_lit)) {
                    add_violation(
                        report,
                        ViolationKind::DelayReplayMismatch,
                        net.net_id,
                        path.demand_id,
                        path.source_index,
                        v,
                        arc_id,
                        std::format(
                            "A(arc={},delay={}) is false while replaying path",
                            arc_id,
                            replay_delay));
                }
            }
        }

        if (pair != nullptr) {
            const int path_delay =
                static_cast<int>(path.node_path.size()) - 1 + delay_offset;
            if (!std::ranges::contains(pair->delays, path_delay)) {
                add_violation(
                    report,
                    ViolationKind::DelayReplayMismatch,
                    net.net_id,
                    path.demand_id,
                    path.source_index,
                    path.node_path.back(),
                    -1,
                    std::format(
                        "path delay {} not in pair delay set [{}]",
                        path_delay,
                        [&] {
                            auto text = std::String {};
                            for (std::size_t i = 0; i < pair->delays.size(); ++i) {
                                if (i != 0) {
                                    text += ",";
                                }
                                text += std::to_string(pair->delays[i]);
                            }
                            return text;
                        }()));
            }
        }
    }

    for (const auto& net : nets) {
        for (const auto& demand : net.demands) {
            const auto count_it = demand_path_count.find({net.net_id, demand.demand_id});
            const std::size_t count = count_it == demand_path_count.end() ? 0 : count_it->second;
            if (count != 1) {
                add_violation(
                    report,
                    ViolationKind::EndpointMismatch,
                    net.net_id,
                    demand.demand_id,
                    0,
                    -1,
                    -1,
                    std::format(
                        "demand has {} extracted paths (expected 1)",
                        count));
            }
        }
    }

    for (const auto& [switch_id, net_ids] : switch_nets_from_paths) {
        if (net_ids.size() > 1) {
            add_violation(
                report,
                ViolationKind::TobSwitchConflict,
                *net_ids.begin(),
                0,
                0,
                -1,
                -1,
                std::format(
                    "physical switch {} is used across {} nets",
                    switch_id,
                    net_ids.size()));
        }
    }

    auto reported_switches = std::set<int> {};
    for (const int switch_id : out.used_tob_switch_ids) {
        reported_switches.insert(switch_id);
        const auto switch_it = model.switch_var_by_id.find(switch_id);
        if (switch_it == model.switch_var_by_id.end()) {
            add_violation(
                report,
                ViolationKind::TobSwitchConflict,
                0,
                0,
                0,
                -1,
                -1,
                std::format(
                    "used_tob_switch_ids includes unknown switch {}",
                    switch_id));
            continue;
        }
        if (!session.value(switch_it->second)) {
            add_violation(
                report,
                ViolationKind::TobSwitchConflict,
                0,
                0,
                0,
                -1,
                -1,
                std::format(
                    "used_tob_switch_ids includes switch {} with false Y literal",
                    switch_id));
        }
    }
    for (const auto& [switch_id, y_var] : model.switch_var_by_id) {
        if (session.value(y_var) && !reported_switches.contains(switch_id)) {
            add_violation(
                report,
                ViolationKind::TobSwitchConflict,
                0,
                0,
                0,
                -1,
                -1,
                std::format(
                    "switch {} has true Y literal but is missing from used_tob_switch_ids",
                    switch_id));
        }
    }
    for (const int switch_id : used_switches_from_paths) {
        if (!reported_switches.contains(switch_id)) {
            add_violation(
                report,
                ViolationKind::TobSwitchConflict,
                0,
                0,
                0,
                -1,
                -1,
                std::format(
                    "switch {} is used by a path but missing in used_tob_switch_ids",
                    switch_id));
        }
    }

    auto matching = std::map<std::pair<int, int>, std::set<int>> {};
    for (const int switch_id : reported_switches) {
        const auto arc_it = std::find_if(
            graph.arcs.begin(),
            graph.arcs.end(),
            [&](const UnifiedArc& arc) { return arc.physical_switch_id == switch_id; });
        if (arc_it == graph.arcs.end()) {
            continue;
        }
        const auto& arc = *arc_it;
        const auto u_kind = graph.nodes[static_cast<std::size_t>(arc.u)].kind;
        const auto v_kind = graph.nodes[static_cast<std::size_t>(arc.v)].kind;
        if (u_kind == UnifiedNodeKind::Bump && v_kind == UnifiedNodeKind::HLine) {
            matching[{0, arc.u}].insert(switch_id);
            matching[{1, arc.v}].insert(switch_id);
        }
        else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::Bump) {
            matching[{0, arc.v}].insert(switch_id);
            matching[{1, arc.u}].insert(switch_id);
        }
        else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::VLine) {
            matching[{2, arc.u}].insert(switch_id);
            matching[{3, arc.v}].insert(switch_id);
        }
        else if (u_kind == UnifiedNodeKind::VLine && v_kind == UnifiedNodeKind::HLine) {
            matching[{2, arc.v}].insert(switch_id);
            matching[{3, arc.u}].insert(switch_id);
        }
    }
    for (const auto& [key, switches] : matching) {
        if (switches.size() > 1) {
            add_violation(
                report,
                ViolationKind::PartialMatchingConflict,
                0,
                0,
                0,
                key.second,
                -1,
                std::format(
                    "partial matching key ({},{}) has {} active switches",
                    key.first,
                    key.second,
                    switches.size()));
        }
    }

    for (const auto& [source_key, units] : bnet_source_units) {
        if (units.size() > 1) {
            add_violation(
                report,
                ViolationKind::BnetUnitConflict,
                source_key.first,
                0,
                source_key.second,
                -1,
                -1,
                std::format(
                    "Bnet source (net={},source={}) spans {} track units",
                    source_key.first,
                    source_key.second,
                    units.size()));
        }
    }

    for (const auto& net : nets) {
        if (!net.is_sync_bus) {
            continue;
        }
        const auto hops_it = sync_bus_hops.find(net.net_id);
        if (hops_it == sync_bus_hops.end() || hops_it->second.empty()) {
            continue;
        }
        const int reference_hop = hops_it->second.front();
        for (const int hop : hops_it->second) {
            if (hop != reference_hop) {
                add_violation(
                    report,
                    ViolationKind::SyncBusDelayMismatch,
                    net.net_id,
                    0,
                    0,
                    -1,
                    -1,
                    std::format(
                        "sync bus paths have mismatched hops: reference={} observed={}",
                        reference_hop,
                        hop));
                break;
            }
        }
    }

    for (const auto& net : nets) {
        if (net.kind != RoutingNetKind::PNnet) {
            continue;
        }
        const auto source_it = source_by_key.find({net.net_id, std::size_t {0}});
        if (source_it == source_by_key.end()) {
            add_violation(
                report,
                ViolationKind::PnnetTrackRule,
                net.net_id,
                0,
                0,
                -1,
                -1,
                "PNnet source delay block is missing");
            continue;
        }
        const auto& source = *source_it->second;
        for (const auto& source_ref : net.sources) {
            const int track_node = resolve_graph_node(graph, source_ref);
            if (track_node < 0) {
                continue;
            }
            for (int delay = 0; delay <= source.d_max; ++delay) {
                if (delay == 1) {
                    continue;
                }
                const int lit = d_literal_for(model, source, track_node, delay);
                if (lit > 0 && session.value(lit)) {
                    add_violation(
                        report,
                        ViolationKind::PnnetTrackRule,
                        net.net_id,
                        0,
                        0,
                        track_node,
                        -1,
                        std::format(
                            "PNnet candidate track node {} is true at forbidden delay {}",
                            track_node,
                            delay));
                }
            }
        }
    }

    report.track_bump_nodes = unique_track_bump_nodes.size();
    report.tob_switches = reported_switches.size();
    report.violations_count = report.violations.size();
    report.pass = report.violations_count == 0;
    return report;
}

auto log_validation_report(const ValidationReport& report, int verbose_level) -> void {
    (void)verbose_level;
    if (report.pass) {
        debug::info_fmt(
            "validate summary: PASS nets={} paths={} violations=0 track_bump_nodes={} tob_switches={} mode_groups={}",
            report.net_count,
            report.path_count,
            report.track_bump_nodes,
            report.tob_switches,
            report.mode_groups);
        return;
    }

    debug::info_fmt(
        "validate summary: FAIL nets={} paths={} violations={} track_bump_nodes={} tob_switches={} mode_groups={}",
        report.net_count,
        report.path_count,
        report.violations_count,
        report.track_bump_nodes,
        report.tob_switches,
        report.mode_groups);
    debug::info_fmt("validate categories: {}", format_violation_counts(report));
    constexpr std::size_t kMaxLines = 50;
    const std::size_t lines = std::min(kMaxLines, report.violations.size());
    for (std::size_t i = 0; i < lines; ++i) {
        const auto& violation = report.violations[i];
        debug::info_fmt(
            "validate violation[{}]: kind={} net={} demand={} source={} node={} arc={} detail={}",
            i,
            violation_kind_name(violation.kind),
            violation.net_id,
            violation.demand_id,
            violation.source_index,
            violation.node_id,
            violation.arc_id,
            violation.detail);
    }
}

} // namespace PR_tool
