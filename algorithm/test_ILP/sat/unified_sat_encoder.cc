#include "sat/unified_sat_encoder.hh"

#include "sat/sat_constraint_kits.hh"
#include "sat/sat_encoding_stats.hh"

#include <algorithm>
#include <bit>
#include <debug/debug.hh>
#include <format>
#include <set>
#include <stdexcept>
#include <tuple>

namespace PR_tool {

namespace {

constexpr int kGlobalVlineModeGroupCount = 16 * 64;

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

auto build_scope(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const std::Vector<int>& source_nodes,
    const std::Vector<int>& sink_nodes
) -> UnifiedSatNetScope {
    auto included = std::Vector<bool>(graph.nodes.size(), false);
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        if (node_in_scope(graph, node, net.scope_bbox)) {
            included[static_cast<std::size_t>(node)] = true;
        }
    }
    for (const int node : source_nodes) {
        included[static_cast<std::size_t>(node)] = true;
    }
    for (const int node : sink_nodes) {
        included[static_cast<std::size_t>(node)] = true;
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

auto vars(CadicalSession& session, const std::size_t count) -> std::Vector<int> {
    auto out = std::Vector<int> {};
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        out.push_back(session.new_var());
    }
    return out;
}

auto scoped_incident_vars(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    const UnifiedSatPairVars& pair,
    const int node,
    const bool incoming
) -> std::Vector<int> {
    auto out = std::Vector<int> {};
    const auto& incident = incoming
        ? graph.in_arc_ids[static_cast<std::size_t>(node)]
        : graph.out_arc_ids[static_cast<std::size_t>(node)];
    for (const int arc_id : incident) {
        const int offset = scope.arc_offset[static_cast<std::size_t>(arc_id)];
        if (offset >= 0) {
            out.push_back(pair.x_vars[static_cast<std::size_t>(offset)]);
        }
    }
    return out;
}

auto add_unit_clause(
    CadicalSession& session,
    SatEncodingStats* stats,
    const SatClauseCategory cat,
    const int literal
) -> void {
    session.add_clause({literal});
    if (stats != nullptr) {
        stats->add_clauses(cat, 1);
    }
}

auto add_sum_equals_activation(
    CadicalSession& session,
    const std::Vector<int>& literals,
    const int activation,
    SatEncodingStats* stats
) -> void {
    auto clause = literals;
    clause.push_back(-activation);
    session.add_clause(clause);
    if (stats != nullptr) {
        stats->add_clauses(SatClauseCategory::Connectivity, 1);
    }
    for (const int literal : literals) {
        add_implies(
            session,
            literal,
            activation,
            stats,
            SatClauseCategory::Connectivity);
    }
    add_pairwise_at_most_one(
        session,
        literals,
        stats,
        SatClauseCategory::Connectivity);
}

auto encode_pair_variable_relations(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    const UnifiedSatPairVars& pair,
    SatEncodingStats* stats
) -> void {
    const int source_offset = scope.node_offset[static_cast<std::size_t>(pair.source_node)];
    const int sink_offset = scope.node_offset[static_cast<std::size_t>(pair.sink_node)];
    if (source_offset < 0 || sink_offset < 0) {
        throw std::invalid_argument(std::format(
            "net {} demand {} endpoint is not in its scope",
            pair.net_id,
            pair.demand_id));
    }

    const int p_source = pair.p_vars[static_cast<std::size_t>(source_offset)];
    const int p_sink = pair.p_vars[static_cast<std::size_t>(sink_offset)];
    add_equiv(session, p_source, pair.activation, stats, SatClauseCategory::VariableRelation);
    add_equiv(session, p_sink, pair.activation, stats, SatClauseCategory::VariableRelation);

    for (std::size_t offset = 0; offset < scope.arc_ids.size(); ++offset) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(scope.arc_ids[offset])];
        const int x = pair.x_vars[offset];
        add_implies(
            session,
            x,
            pair.p_vars[static_cast<std::size_t>(
                scope.node_offset[static_cast<std::size_t>(arc.u)])],
            stats,
            SatClauseCategory::VariableRelation);
        add_implies(
            session,
            x,
            pair.p_vars[static_cast<std::size_t>(
                scope.node_offset[static_cast<std::size_t>(arc.v)])],
            stats,
            SatClauseCategory::VariableRelation);
    }
}

auto encode_pair_connectivity(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    const UnifiedSatPairVars& pair,
    SatEncodingStats* stats
) -> void {
    const auto source_out = scoped_incident_vars(graph, scope, pair, pair.source_node, false);
    add_sum_equals_activation(session, source_out, pair.activation, stats);
    for (const int literal : scoped_incident_vars(graph, scope, pair, pair.source_node, true)) {
        add_unit_clause(session, stats, SatClauseCategory::Connectivity, -literal);
    }

    const auto sink_in = scoped_incident_vars(graph, scope, pair, pair.sink_node, true);
    add_sum_equals_activation(session, sink_in, pair.activation, stats);
    for (const int literal : scoped_incident_vars(graph, scope, pair, pair.sink_node, false)) {
        add_unit_clause(session, stats, SatClauseCategory::Connectivity, -literal);
    }

    for (std::size_t node_offset = 0; node_offset < scope.node_ids.size(); ++node_offset) {
        const int node = scope.node_ids[node_offset];
        if (node == pair.source_node || node == pair.sink_node) {
            continue;
        }
        const int p = pair.p_vars[node_offset];
        const auto incoming = scoped_incident_vars(graph, scope, pair, node, true);
        const auto outgoing = scoped_incident_vars(graph, scope, pair, node, false);
        add_or_equiv(session, p, incoming, stats, SatClauseCategory::Connectivity);
        add_or_equiv(session, p, outgoing, stats, SatClauseCategory::Connectivity);
        add_pairwise_at_most_one(
            session,
            incoming,
            stats,
            SatClauseCategory::Connectivity);
        add_pairwise_at_most_one(
            session,
            outgoing,
            stats,
            SatClauseCategory::Connectivity);
        add_implies(session, p, pair.activation, stats, SatClauseCategory::Connectivity);
    }
}

auto add_vline_track_mode_constraints(
    CadicalSession& session,
    const UnifiedGraph& graph,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void {
    for (int group_id = 0; group_id < kGlobalVlineModeGroupCount; ++group_id) {
        model.mode_var_by_group.emplace(group_id, session.new_var());
        if (stats != nullptr) {
            ++stats->mode_vars;
        }
    }

    for (const auto& pair : model.pairs) {
        const auto& scope = model.scopes[pair.scope_index];
        for (std::size_t offset = 0; offset < scope.arc_ids.size(); ++offset) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(scope.arc_ids[offset])];
            const int x = pair.x_vars[offset];
            if (arc.mode_group_id >= 0
                && (arc.is_vline_track_straight || arc.is_vline_track_swap)) {
                const auto mode_it = model.mode_var_by_group.find(arc.mode_group_id);
                if (mode_it == model.mode_var_by_group.end()) {
                    throw std::invalid_argument(std::format(
                        "VLineTrack mode group {} is outside the global 0..{} range",
                        arc.mode_group_id,
                        kGlobalVlineModeGroupCount - 1));
                }
                if (arc.is_vline_track_straight) {
                    add_implies(
                        session,
                        x,
                        mode_it->second,
                        stats,
                        SatClauseCategory::VlineTrackMode);
                }
                if (arc.is_vline_track_swap) {
                    add_implies(
                        session,
                        x,
                        -mode_it->second,
                        stats,
                        SatClauseCategory::VlineTrackMode);
                }
            }
        }
    }
}

auto add_tob_switch_constraints(
    CadicalSession& session,
    const UnifiedGraph& graph,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void {
    auto switch_uses = std::map<int, std::Vector<int>> {};
    auto switch_endpoints = std::map<int, std::pair<int, int>> {};
    for (const auto& pair : model.pairs) {
        const auto& scope = model.scopes[pair.scope_index];
        for (std::size_t offset = 0; offset < scope.arc_ids.size(); ++offset) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(scope.arc_ids[offset])];
            const int x = pair.x_vars[offset];
            if (arc.physical_switch_id >= 0
                && (arc.physical_switch_kind == PhysicalSwitchKind::BumpH
                    || arc.physical_switch_kind == PhysicalSwitchKind::HLineVLine)) {
                switch_uses[arc.physical_switch_id].push_back(x);
                switch_endpoints.emplace(
                    arc.physical_switch_id, std::pair {arc.u, arc.v});
            }
        }
    }

    for (auto& [switch_id, uses] : switch_uses) {
        (void)switch_id;
        std::sort(uses.begin(), uses.end());
        uses.erase(std::unique(uses.begin(), uses.end()), uses.end());
        const int y = session.new_var();
        model.switch_var_by_id.emplace(switch_id, y);
        if (stats != nullptr) {
            ++stats->switch_vars;
        }
        add_or_equiv(session, y, uses, stats, SatClauseCategory::TobSwitchUniqueness);
    }

    auto matching = std::map<std::pair<int, int>, std::Vector<int>> {};
    for (const auto& [switch_id, y] : model.switch_var_by_id) {
        const auto endpoint_it = switch_endpoints.find(switch_id);
        if (endpoint_it == switch_endpoints.end()) {
            continue;
        }
        const auto [u, v] = endpoint_it->second;
        const auto u_kind = graph.nodes[static_cast<std::size_t>(u)].kind;
        const auto v_kind = graph.nodes[static_cast<std::size_t>(v)].kind;
        if (u_kind == UnifiedNodeKind::Bump && v_kind == UnifiedNodeKind::HLine) {
            matching[{0, u}].push_back(y);
            matching[{1, v}].push_back(y);
        }
        else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::Bump) {
            matching[{0, v}].push_back(y);
            matching[{1, u}].push_back(y);
        }
        else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::VLine) {
            matching[{2, u}].push_back(y);
            matching[{3, v}].push_back(y);
        }
        else if (u_kind == UnifiedNodeKind::VLine && v_kind == UnifiedNodeKind::HLine) {
            matching[{2, v}].push_back(y);
            matching[{3, u}].push_back(y);
        }
    }
    for (auto& [key, ys] : matching) {
        (void)key;
        std::sort(ys.begin(), ys.end());
        ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
        add_pairwise_at_most_one(
            session,
            ys,
            stats,
            SatClauseCategory::TobSwitchUniqueness);
    }
}

auto add_sync_bus_constraints(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void {
    for (const auto& net : nets) {
        if (!net.is_sync_bus) {
            continue;
        }
        auto bus_pairs = std::Vector<UnifiedSatPairVars*> {};
        for (const auto& demand : net.demands) {
            if (!demand.fixed_pair || demand.candidate_source_indices.size() != 1) {
                throw std::invalid_argument(std::format(
                    "Sync bus net {} demand {} must be fixed with exactly one candidate pair",
                    net.net_id,
                    demand.demand_id));
            }
            for (auto& pair : model.pairs) {
                if (pair.net_id == net.net_id && pair.demand_id == demand.demand_id) {
                    bus_pairs.push_back(&pair);
                    break;
                }
            }
        }
        if (bus_pairs.size() != net.demands.size()) {
            throw std::logic_error(std::format(
                "Sync bus net {} is missing an encoded demand", net.net_id));
        }

        for (auto* pair : bus_pairs) {
            const auto& scope = model.scopes[pair->scope_index];
            std::size_t width = 1;
            while ((std::size_t {1} << width) <= scope.node_ids.size()) {
                ++width;
            }
            auto distance = std::Vector<std::Vector<int>>(
                scope.node_ids.size(), std::Vector<int>(width));
            for (auto& bits : distance) {
                bits = vars(session, width);
                if (stats != nullptr) {
                    stats->bus_distance_vars += width;
                }
            }
            auto successor = std::Vector<BinarySuccessorVars> {};
            successor.reserve(scope.node_ids.size());
            for (const auto& bits : distance) {
                successor.push_back(add_binary_successor(
                    session,
                    bits,
                    stats,
                    SatClauseCategory::SyncBusLoopElimination));
            }
            const int source_offset =
                scope.node_offset[static_cast<std::size_t>(pair->source_node)];
            for (std::size_t bit = 0; bit < width; ++bit) {
                add_unit_clause(
                    session,
                    stats,
                    SatClauseCategory::SyncBusLoopElimination,
                    bit == 0
                        ? distance[static_cast<std::size_t>(source_offset)][bit]
                        : -distance[static_cast<std::size_t>(source_offset)][bit]);
            }
            for (std::size_t offset = 0; offset < scope.arc_ids.size(); ++offset) {
                const auto& arc =
                    graph.arcs[static_cast<std::size_t>(scope.arc_ids[offset])];
                const auto u_offset = static_cast<std::size_t>(
                    scope.node_offset[static_cast<std::size_t>(arc.u)]);
                const auto v_offset = static_cast<std::size_t>(
                    scope.node_offset[static_cast<std::size_t>(arc.v)]);
                add_conditional_successor(
                    session,
                    pair->x_vars[offset],
                    successor[u_offset],
                    distance[v_offset],
                    stats,
                    SatClauseCategory::SyncBusLoopElimination);
            }
            pair->sink_distance_bits = distance[static_cast<std::size_t>(
                scope.node_offset[static_cast<std::size_t>(pair->sink_node)])];
        }

        if (!bus_pairs.empty()) {
            const auto& reference = bus_pairs.front()->sink_distance_bits;
            for (std::size_t member = 1; member < bus_pairs.size(); ++member) {
                const auto& bits = bus_pairs[member]->sink_distance_bits;
                if (bits.size() != reference.size()) {
                    throw std::invalid_argument(std::format(
                        "Sync bus net {} members have incompatible distance widths",
                        net.net_id));
                }
                for (std::size_t bit = 0; bit < bits.size(); ++bit) {
                    add_equiv(
                        session,
                        bits[bit],
                        reference[bit],
                        stats,
                        SatClauseCategory::SyncBusEqualLength);
                }
            }
        }
    }
}

} // namespace

auto build_unified_sat_model(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    SatEncodingStats* stats
) -> UnifiedSatModel {
    auto model = UnifiedSatModel {};
    auto source_nodes_by_net = std::Vector<std::Vector<int>> {};
    auto sink_nodes_by_net = std::Vector<std::Vector<int>> {};
    source_nodes_by_net.reserve(nets.size());
    sink_nodes_by_net.reserve(nets.size());

    for (const auto& net : nets) {
        if (net.demands.empty()) {
            throw std::invalid_argument(std::format("net {} has no routing demands", net.net_id));
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
            sink_nodes.push_back(checked_endpoint(graph, demand.sink, net.net_id, "sink"));
            for (const auto source_index : demand.candidate_source_indices) {
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
        }
        source_nodes_by_net.push_back(source_nodes);
        sink_nodes_by_net.push_back(sink_nodes);
        model.scopes.push_back(build_scope(graph, net, source_nodes, sink_nodes));
    }

    auto logical_source_node_count = std::Vector<std::size_t>(graph.nodes.size(), 0);
    auto pair_indices_by_scope_source =
        std::Vector<std::Vector<std::Vector<std::size_t>>> {};
    pair_indices_by_scope_source.resize(nets.size());
    auto p_occupancy_by_node =
        std::Vector<std::Vector<int>>(graph.nodes.size());

    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        const auto& net = nets[net_index];
        const auto& scope = model.scopes[net_index];
        pair_indices_by_scope_source[net_index].resize(net.sources.size());
        for (std::size_t source_index = 0; source_index < net.sources.size(); ++source_index) {
            auto source = UnifiedSatLogicalSourceVars {
                net.net_id,
                source_index,
                source_nodes_by_net[net_index][source_index],
                net_index,
                vars(session, scope.node_ids.size())};
            if (stats != nullptr) {
                stats->p_logical_vars += source.p_vars.size();
            }
            ++logical_source_node_count[static_cast<std::size_t>(source.source_node)];
            for (std::size_t node_offset = 0; node_offset < scope.node_ids.size(); ++node_offset) {
                p_occupancy_by_node[static_cast<std::size_t>(scope.node_ids[node_offset])]
                    .push_back(source.p_vars[node_offset]);
            }
            model.logical_sources.push_back(std::move(source));
        }
        for (std::size_t demand_index = 0; demand_index < net.demands.size(); ++demand_index) {
            const auto& demand = net.demands[demand_index];
            auto sink_choices = std::Vector<int> {};
            for (const auto source_index : demand.candidate_source_indices) {
                auto pair = UnifiedSatPairVars {};
                pair.net_id = net.net_id;
                pair.demand_id = demand.demand_id;
                pair.source_index = source_index;
                pair.source_node = source_nodes_by_net[net_index][source_index];
                pair.sink_node = sink_nodes_by_net[net_index][demand_index];
                pair.scope_index = net_index;
                pair.activation = session.new_var();
                pair.p_vars = vars(session, scope.node_ids.size());
                pair.x_vars = vars(session, scope.arc_ids.size());
                if (stats != nullptr) {
                    ++stats->activation_vars;
                    stats->pair_p_vars += pair.p_vars.size();
                    stats->x_vars += pair.x_vars.size();
                }
                pair.p_sink = pair.p_vars[static_cast<std::size_t>(
                    scope.node_offset[static_cast<std::size_t>(pair.sink_node)])];
                sink_choices.push_back(pair.p_sink);
                pair_indices_by_scope_source[net_index][source_index].push_back(
                    model.pairs.size());
                model.pairs.push_back(std::move(pair));
            }
            add_exactly_one(
                session,
                sink_choices,
                stats,
                SatClauseCategory::Constant);
        }
    }

    for (const auto& pair : model.pairs) {
        encode_pair_variable_relations(
            session,
            graph,
            model.scopes[pair.scope_index],
            pair,
            stats);
        encode_pair_connectivity(
            session,
            graph,
            model.scopes[pair.scope_index],
            pair,
            stats);
    }

    for (auto& source : model.logical_sources) {
        const auto& scope = model.scopes[source.scope_index];
        for (std::size_t node_offset = 0; node_offset < scope.node_ids.size(); ++node_offset) {
            const int node = scope.node_ids[node_offset];
            if (node == source.source_node) {
                add_unit_clause(
                    session,
                    stats,
                    SatClauseCategory::Constant,
                    source.p_vars[node_offset]);
                continue;
            }
            if (logical_source_node_count[static_cast<std::size_t>(node)] != 0) {
                add_unit_clause(
                    session,
                    stats,
                    SatClauseCategory::Constant,
                    -source.p_vars[node_offset]);
            }
            auto pair_p = std::Vector<int> {};
            const auto& pair_indices =
                pair_indices_by_scope_source[source.scope_index][source.source_index];
            pair_p.reserve(pair_indices.size());
            for (const auto pair_index : pair_indices) {
                pair_p.push_back(model.pairs[pair_index].p_vars[node_offset]);
            }
            add_or_equiv(
                session,
                source.p_vars[node_offset],
                pair_p,
                stats,
                SatClauseCategory::VariableRelation);
        }
    }

    for (const auto& occupancy : p_occupancy_by_node) {
        add_sequential_at_most_one(
            session,
            occupancy,
            stats,
            SatClauseCategory::Exclusivity);
    }

    add_vline_track_mode_constraints(session, graph, model, stats);
    add_tob_switch_constraints(session, graph, model, stats);
    add_sync_bus_constraints(session, graph, nets, model, stats);

    debug::info_fmt(
        "unified numeric SAT model: scopes={} pairs={} sources={} vars={} clauses={}",
        model.scopes.size(),
        model.pairs.size(),
        model.logical_sources.size(),
        session.num_vars(),
        session.num_clauses());
    return model;
}

} // namespace PR_tool
