#include "sat/unified_sat_encoder.hh"

#include "graph/unified_routing_graph.hh"
#include "sat/encode_bus_sync.hh"
#include "sat/encode_tob_special.hh"
#include "sat/sat_constraint_kits.hh"
#include "sat/sat_encoding_stats.hh"

#include <algorithm>
#include <debug/debug.hh>
#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

constexpr int kGlobalVlineModeGroupCount = 16 * 64;

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

auto d_literal(
    const UnifiedSatModel& model,
    std::size_t model_source_index,
    const UnifiedSatNetScope& scope,
    int node,
    int delay
) -> int {
    if (node < 0 || static_cast<std::size_t>(node) >= scope.node_offset.size()) {
        return 0;
    }
    const int node_offset = scope.node_offset[static_cast<std::size_t>(node)];
    if (node_offset < 0) {
        return 0;
    }
    const auto& source = model.sources[model_source_index];
    if (delay < 0 || delay > source.d_max) {
        return 0;
    }
    const int lit = source.d_var[static_cast<std::size_t>(node_offset)]
                        [static_cast<std::size_t>(delay)];
    return lit > 0 ? lit : 0;
}

auto collect_node_d_literals(
    const UnifiedGraph& graph,
    const UnifiedSatModel& model,
    const std::Vector<UnifiedSatNetScope>& scopes
) -> std::Vector<std::Vector<int>> {
    auto occupancy = std::Vector<std::Vector<int>>(scopes.empty() ? 0 : scopes[0].node_offset.size());
    if (!scopes.empty()) {
        occupancy.assign(scopes[0].node_offset.size(), {});
    }
    for (const auto& source : model.sources) {
        const auto& scope = scopes[source.scope_index];
        for (std::size_t node_offset = 0; node_offset < source.d_var.size(); ++node_offset) {
            const int global_node = scope.node_ids[node_offset];
            if (global_node >= 0
                && static_cast<std::size_t>(global_node) < graph.nodes.size()
                && graph.nodes[static_cast<std::size_t>(global_node)].kind
                    == UnifiedNodeKind::VirtualSource) {
                continue;
            }
            for (int delay = 0; delay <= source.d_max; ++delay) {
                const int lit = source.d_var[node_offset][static_cast<std::size_t>(delay)];
                if (lit > 0) {
                    occupancy[static_cast<std::size_t>(global_node)].push_back(lit);
                }
            }
        }
    }
    return occupancy;
}

auto encode_connectivity(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const std::Vector<UnifiedSatNetScope>& scopes,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void {
    for (const auto& source : model.sources) {
        const auto& scope = scopes[source.scope_index];
        for (std::size_t node_offset = 0; node_offset < scope.node_ids.size(); ++node_offset) {
            const int node = scope.node_ids[node_offset];
            if (node == source.source_node) {
                continue;
            }
            for (int delay = 1; delay <= source.d_max; ++delay) {
                const int d_lit = source.d_var[node_offset][static_cast<std::size_t>(delay)];
                if (d_lit <= 0) {
                    continue;
                }
                auto predecessors = std::Vector<int> {};
                for (const int arc_id : graph.in_arc_ids[static_cast<std::size_t>(node)]) {
                    const int arc_offset = scope.arc_offset[static_cast<std::size_t>(arc_id)];
                    if (arc_offset < 0) {
                        continue;
                    }
                    const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                    if (is_tob_arc(arc)) {
                        const int a_lit = tob_a_literal(
                            model,
                            source.model_source_index,
                            arc_id,
                            delay);
                        if (a_lit > 0) {
                            predecessors.push_back(a_lit);
                        }
                        continue;
                    }
                    if (is_virtual_source_arc(arc)) {
                        const int pred_lit = d_literal(
                            model,
                            source.model_source_index,
                            scope,
                            arc.u,
                            delay - 1);
                        if (pred_lit > 0) {
                            predecessors.push_back(pred_lit);
                        }
                        continue;
                    }
                    const int pred_lit = d_literal(
                        model,
                        source.model_source_index,
                        scope,
                        arc.u,
                        delay - 1);
                    if (pred_lit > 0) {
                        predecessors.push_back(pred_lit);
                    }
                }
                if (predecessors.empty()) {
                    add_unit_clause(
                        session,
                        stats,
                        SatClauseCategory::Connectivity,
                        -d_lit);
                    continue;
                }
                auto clause = predecessors;
                clause.push_back(-d_lit);
                session.add_clause(clause);
                if (stats != nullptr) {
                    stats->add_clauses(SatClauseCategory::Connectivity, 1);
                }
            }
        }
    }
}

auto encode_pnnet_track_delay_forbid(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void {
    for (const auto& net : nets) {
        if (net.kind != RoutingNetKind::PNnet) {
            continue;
        }
        const auto source_it = std::find_if(
            model.sources.begin(),
            model.sources.end(),
            [&](const SourceDelayVars& source) {
                return source.net_id == net.net_id && source.source_index == 0;
            });
        if (source_it == model.sources.end()) {
            continue;
        }
        const auto& scope = scopes[source_it->scope_index];
        for (const auto& source_ref : net.sources) {
            const int track_node = resolve_graph_node(graph, source_ref);
            if (track_node < 0) {
                continue;
            }
            const int node_offset = scope.node_offset[static_cast<std::size_t>(track_node)];
            if (node_offset < 0) {
                continue;
            }
            for (int delay = 0; delay <= source_it->d_max; ++delay) {
                if (delay == 1) {
                    continue;
                }
                const int lit =
                    source_it->d_var[static_cast<std::size_t>(node_offset)][static_cast<std::size_t>(delay)];
                if (lit > 0) {
                    add_unit_clause(session, stats, SatClauseCategory::Constant, -lit);
                }
            }
        }
    }
}

} // namespace

auto is_tob_arc(const UnifiedArc& arc) -> bool {
    return arc.physical_switch_kind != PhysicalSwitchKind::None;
}

auto find_tob_arc_vars(
    const UnifiedSatModel& model,
    std::size_t model_source_index,
    int arc_global_id
) -> const TobArcDelayVars* {
    const auto it = model.tob_arc_index.find({model_source_index, arc_global_id});
    if (it == model.tob_arc_index.end() || it->second >= model.tob_arcs.size()) {
        return nullptr;
    }
    return &model.tob_arcs[it->second];
}

auto tob_a_literal(
    const UnifiedSatModel& model,
    std::size_t model_source_index,
    int arc_global_id,
    int delay
) -> int {
    const auto* vars = find_tob_arc_vars(model, model_source_index, arc_global_id);
    if (vars == nullptr || delay < 0 || delay > vars->d_max) {
        return 0;
    }
    const int lit = vars->a_var[static_cast<std::size_t>(delay)];
    return lit > 0 ? lit : 0;
}

auto build_unified_sat_model(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const DelayPrecomputeResult& delays,
    SatEncodingStats* stats
) -> UnifiedSatModel {
    auto model = UnifiedSatModel {};
    model.scopes = scopes;
    model.pair_delays = delays.pairs;

    auto logical_source_nodes = std::Vector<std::size_t>(graph.nodes.size(), 0);
    for (std::size_t domain_index = 0; domain_index < delays.sources.size(); ++domain_index) {
        const auto& domain = delays.sources[domain_index];
        const auto& scope = scopes[domain.scope_index];
        auto source = SourceDelayVars {};
        source.net_id = domain.net_id;
        source.source_index = domain.source_index;
        source.source_node = domain.source_node;
        source.scope_index = domain.scope_index;
        source.model_source_index = domain_index;
        source.d_max = domain.d_max;
        const auto net_it = std::find_if(
            nets.begin(),
            nets.end(),
            [&](const RoutingNet& net) { return net.net_id == domain.net_id; });
        if (net_it == nets.end()) {
            throw std::logic_error(std::format(
                "missing routing net {} for source unit selection",
                domain.net_id));
        }
        if (net_it->kind == RoutingNetKind::Bnet
            && graph.nodes[static_cast<std::size_t>(domain.source_node)].kind
                == UnifiedNodeKind::Bump) {
            source.unit_selector_var_by_unit.reserve(16);
            for (std::size_t unit = 0; unit < 16; ++unit) {
                source.unit_selector_var_by_unit.push_back(session.new_var());
                if (stats != nullptr) {
                    ++stats->unit_selector_vars;
                }
            }
            add_at_least_one(
                session,
                source.unit_selector_var_by_unit,
                stats,
                SatClauseCategory::SourceUnitSelection);
            add_sequential_at_most_one(
                session,
                source.unit_selector_var_by_unit,
                stats,
                SatClauseCategory::SourceUnitSelection);
        }
        source.d_var.assign(
            scope.node_ids.size(),
            std::Vector<int>(static_cast<std::size_t>(domain.d_max) + 1, 0));
        const auto expected_slots =
            scope.node_ids.size() * (static_cast<std::size_t>(domain.d_max) + 1);
        if (domain.active_d.size() != expected_slots) {
            throw std::logic_error(std::format(
                "net {} source {} active D mask has {} slots, expected {}",
                domain.net_id,
                domain.source_index,
                domain.active_d.size(),
                expected_slots));
        }
        if (stats != nullptr) {
            stats->dense_d_slots += expected_slots;
            stats->unit_eligible_d_slots +=
                domain.unit_eligible_node_count
                * (static_cast<std::size_t>(domain.d_max) + 1);
        }
        for (std::size_t node_offset = 0; node_offset < scope.node_ids.size(); ++node_offset) {
            for (int delay = 0; delay <= domain.d_max; ++delay) {
                const auto slot =
                    node_offset * (static_cast<std::size_t>(domain.d_max) + 1)
                    + static_cast<std::size_t>(delay);
                if (domain.active_d[slot] == 0) {
                    source.d_var[node_offset][static_cast<std::size_t>(delay)] = -1;
                    continue;
                }
                source.d_var[node_offset][static_cast<std::size_t>(delay)] = session.new_var();
                if (stats != nullptr) {
                    ++stats->d_vars;
                }
            }
        }
        ++logical_source_nodes[static_cast<std::size_t>(domain.source_node)];
        model.sources.push_back(std::move(source));
    }

    for (auto& source : model.sources) {
        const auto& scope = scopes[source.scope_index];
        for (std::size_t node_offset = 0; node_offset < scope.node_ids.size(); ++node_offset) {
            const int node = scope.node_ids[node_offset];
            if (node == source.source_node) {
                const int d0 = source.d_var[node_offset][0];
                if (d0 > 0) {
                    add_unit_clause(
                        session,
                        stats,
                        SatClauseCategory::Constant,
                        d0);
                }
                for (int delay = 1; delay <= source.d_max; ++delay) {
                    const int lit =
                        source.d_var[node_offset][static_cast<std::size_t>(delay)];
                    if (lit > 0) {
                        add_unit_clause(
                            session,
                            stats,
                            SatClauseCategory::Constant,
                            -lit);
                    }
                }
                continue;
            }
            const int d0 = source.d_var[node_offset][0];
            if (d0 > 0) {
                add_unit_clause(
                    session,
                    stats,
                    SatClauseCategory::Constant,
                    -d0);
            }
            if (logical_source_nodes[static_cast<std::size_t>(node)] != 0) {
                for (int delay = 1; delay <= source.d_max; ++delay) {
                    const int lit =
                        source.d_var[node_offset][static_cast<std::size_t>(delay)];
                    if (lit > 0) {
                        add_unit_clause(
                            session,
                            stats,
                            SatClauseCategory::Constant,
                            -lit);
                    }
                }
            }
        }
    }

    encode_pnnet_track_delay_forbid(session, graph, nets, scopes, model, stats);

    // Allocate TOB A vars before connectivity implications that reference them.
    for (std::size_t source_index = 0; source_index < model.sources.size(); ++source_index) {
        const auto& source = model.sources[source_index];
        const auto& domain = delays.sources[source_index];
        const auto& scope = scopes[source.scope_index];
        if (stats != nullptr) {
            stats->dense_a_slots +=
                static_cast<std::size_t>(source.d_max)
                * static_cast<std::size_t>(std::count_if(
                    scope.arc_ids.begin(),
                    scope.arc_ids.end(),
                    [&](const int arc_id) {
                        return is_tob_arc(graph.arcs[static_cast<std::size_t>(arc_id)]);
                    }));
            stats->unit_eligible_a_slots +=
                static_cast<std::size_t>(source.d_max)
                * domain.unit_eligible_tob_arc_count;
        }
        for (const int arc_id : scope.arc_ids) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!is_tob_arc(arc)) {
                continue;
            }
            auto tob_arc = TobArcDelayVars {};
            tob_arc.arc_global_id = arc_id;
            tob_arc.model_source_index = source_index;
            tob_arc.d_max = source.d_max;
            tob_arc.a_var.assign(static_cast<std::size_t>(source.d_max) + 1, 0);
            for (int delay = 1; delay <= source.d_max; ++delay) {
                const int d_u = d_literal(model, source_index, scope, arc.u, delay - 1);
                const int d_v = d_literal(model, source_index, scope, arc.v, delay);
                if (d_u > 0 && d_v > 0) {
                    tob_arc.a_var[static_cast<std::size_t>(delay)] = session.new_var();
                    if (stats != nullptr) {
                        ++stats->a_vars;
                    }
                }
            }
            const auto tob_arc_index = model.tob_arcs.size();
            model.tob_arcs.push_back(std::move(tob_arc));
            model.tob_arc_index.emplace(
                std::pair {source_index, arc_id},
                tob_arc_index);
        }
    }

    for (const auto& tob_arc : model.tob_arcs) {
        const auto& source = model.sources[tob_arc.model_source_index];
        if (source.unit_selector_var_by_unit.empty()) {
            continue;
        }
        const auto& arc = graph.arcs[static_cast<std::size_t>(tob_arc.arc_global_id)];
        if (arc.physical_switch_kind != PhysicalSwitchKind::VLineTrack) {
            continue;
        }
        const auto& u_node = graph.nodes[static_cast<std::size_t>(arc.u)];
        const auto& v_node = graph.nodes[static_cast<std::size_t>(arc.v)];
        const UnifiedNode* track = nullptr;
        if (u_node.kind == UnifiedNodeKind::Track) {
            track = &u_node;
        }
        else if (v_node.kind == UnifiedNodeKind::Track) {
            track = &v_node;
        }
        if (track == nullptr || track->unit >= source.unit_selector_var_by_unit.size()) {
            throw std::logic_error(std::format(
                "VLineTrack arc {} has no valid track unit",
                tob_arc.arc_global_id));
        }
        const int q_lit = source.unit_selector_var_by_unit[track->unit];
        for (int delay = 1; delay <= tob_arc.d_max; ++delay) {
            const int a_lit = tob_arc.a_var[static_cast<std::size_t>(delay)];
            if (a_lit > 0) {
                add_implies(
                    session,
                    a_lit,
                    q_lit,
                    stats,
                    SatClauseCategory::SourceUnitSelection);
            }
        }
    }

    encode_connectivity(session, graph, scopes, model, stats);

    const auto occupancy = collect_node_d_literals(graph, model, scopes);
    for (const auto& literals : occupancy) {
        if (literals.size() > 1) {
            add_sequential_at_most_one(
                session,
                literals,
                stats,
                SatClauseCategory::Exclusivity);
        }
    }

    for (int group_id = 0; group_id < kGlobalVlineModeGroupCount; ++group_id) {
        model.mode_var_by_group.emplace(group_id, session.new_var());
        if (stats != nullptr) {
            ++stats->mode_vars;
        }
    }

    encode_tob_special_constraints(session, graph, model, stats);
    encode_bus_sync_constraints(session, nets, model, stats);

    for (const auto& pair : model.pair_delays) {
        const auto source_it = std::find_if(
            model.sources.begin(),
            model.sources.end(),
            [&](const SourceDelayVars& source) {
                return source.net_id == pair.net_id && source.source_index == pair.source_index;
            });
        if (source_it == model.sources.end()) {
            throw std::logic_error(std::format(
                "missing source vars for alpha net {} source index {}",
                pair.net_id,
                pair.source_index));
        }
        const auto& scope = scopes[source_it->scope_index];
        auto sink_literals = std::Vector<int> {};
        sink_literals.reserve(pair.delays.size());
        for (int delay : pair.delays) {
            const int sink_lit = d_literal(
                model,
                source_it->model_source_index,
                scope,
                pair.sink_node,
                delay);
            if (sink_lit > 0) {
                sink_literals.push_back(sink_lit);
            }
        }
        const int alpha_lit = session.new_var();
        if (stats != nullptr) {
            ++stats->alpha_vars;
        }
        const auto pair_key = PairKey {pair.net_id, pair.demand_id, pair.source_index};
        model.alpha_vars.push_back(PairAlphaVar {pair_key, alpha_lit});
        model.alpha_lit_by_pair.emplace(pair_key, alpha_lit);
        auto alpha_clause = sink_literals;
        alpha_clause.push_back(-alpha_lit);
        session.add_clause(alpha_clause);
        if (stats != nullptr) {
            stats->add_clauses(SatClauseCategory::VariableRelation, 1);
        }
    }

    debug::info_fmt(
        "unified numeric SAT model v14: scopes={} sources={} pairs={} tob_arcs={} vars={} clauses={}",
        model.scopes.size(),
        model.sources.size(),
        model.pair_delays.size(),
        model.tob_arcs.size(),
        session.num_vars(),
        session.num_clauses());
    return model;
}

} // namespace PR_tool
