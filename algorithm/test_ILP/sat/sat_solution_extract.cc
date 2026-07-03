#include "sat/sat_solution_extract.hh"

#include "graph/unified_routing_graph.hh"

#include <algorithm>
#include <debug/debug.hh>
#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

auto find_source_vars(
    const UnifiedSatModel& model,
    const PairDelayInfo& pair
) -> const SourceDelayVars* {
    for (const auto& source : model.sources) {
        if (source.net_id == pair.net_id && source.source_index == pair.source_index) {
            return &source;
        }
    }
    return nullptr;
}

auto d_value(
    const CadicalSession& session,
    const UnifiedSatModel& model,
    const SourceDelayVars& source,
    int node,
    int delay
) -> bool {
    const auto& scope = model.scopes[source.scope_index];
    if (node < 0 || static_cast<std::size_t>(node) >= scope.node_offset.size()) {
        return false;
    }
    const int node_offset = scope.node_offset[static_cast<std::size_t>(node)];
    if (node_offset < 0 || delay < 0 || delay > source.d_max) {
        return false;
    }
    const int lit = source.d_var[static_cast<std::size_t>(node_offset)]
                        [static_cast<std::size_t>(delay)];
    return lit > 0 && session.value(lit);
}

auto a_value(
    const CadicalSession& session,
    const UnifiedSatModel& model,
    std::size_t model_source_index,
    int arc_id,
    int delay
) -> bool {
    const int lit = tob_a_literal(model, model_source_index, arc_id, delay);
    return lit > 0 && session.value(lit);
}

} // namespace

auto extract_sat_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const UnifiedSatModel& model,
    const CadicalSession& session,
    const CadicalSolveResult& solve_result
) -> SatRoutingResult {
    auto out = SatRoutingResult {};
    out.num_vars = session.num_vars();
    out.num_clauses = session.num_clauses();
    if (!solve_result.ok) {
        out.message = solve_result.message;
        return out;
    }

    std::size_t expected_paths = 0;
    for (const auto& net : nets) {
        expected_paths += net.demands.size();
        for (const auto& demand : net.demands) {
            const PairDelayInfo* pair = nullptr;
            for (const auto& candidate : model.pair_delays) {
                if (candidate.net_id == net.net_id && candidate.demand_id == demand.demand_id) {
                    pair = &candidate;
                    break;
                }
            }
            if (pair == nullptr) {
                out.message = std::format(
                    "net {} demand {} has no delay pair",
                    net.net_id,
                    demand.demand_id);
                return out;
            }
            const SourceDelayVars* source = find_source_vars(model, *pair);
            if (source == nullptr) {
                out.message = std::format(
                    "net {} demand {} has no source vars",
                    net.net_id,
                    demand.demand_id);
                return out;
            }
            if (!d_value(session, model, *source, pair->source_node, 0)) {
                out.message = std::format(
                    "net {} demand {} source is not active at delay 0",
                    net.net_id,
                    demand.demand_id);
                return out;
            }
            int sink_delay = -1;
            for (int delay : pair->delays) {
                if (d_value(session, model, *source, pair->sink_node, delay)) {
                    sink_delay = delay;
                    break;
                }
            }
            if (sink_delay < 0) {
                out.message = std::format(
                    "net {} demand {} sink is not active at any target delay",
                    net.net_id,
                    demand.demand_id);
                return out;
            }

            auto path = std::Vector<int> {pair->sink_node};
            int current = pair->sink_node;
            int delay = sink_delay;
            while (delay > 0) {
                int previous = -1;
                for (const int arc_id : graph.in_arc_ids[static_cast<std::size_t>(current)]) {
                    const int arc_offset =
                        model.scopes[source->scope_index].arc_offset[static_cast<std::size_t>(arc_id)];
                    if (arc_offset < 0) {
                        continue;
                    }
                    const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                    bool matches = false;
                    if (is_tob_arc(arc)) {
                        matches = a_value(
                            session,
                            model,
                            source->model_source_index,
                            arc_id,
                            delay);
                    }
                    else if (is_virtual_source_arc(arc)) {
                        matches = d_value(
                            session,
                            model,
                            *source,
                            arc.u,
                            delay - 1);
                    }
                    else {
                        matches = d_value(
                            session,
                            model,
                            *source,
                            arc.u,
                            delay - 1);
                    }
                    if (matches) {
                        previous = arc.u;
                        break;
                    }
                }
                if (previous < 0) {
                    out.message = std::format(
                        "net {} demand {} path has no predecessor at node {} delay {}",
                        net.net_id,
                        demand.demand_id,
                        current,
                        delay);
                    return out;
                }
                path.push_back(previous);
                current = previous;
                --delay;
            }
            if (current != pair->source_node) {
                out.message = std::format(
                    "net {} demand {} path reaches node {} instead of source {} at delay 0",
                    net.net_id,
                    demand.demand_id,
                    current,
                    pair->source_node);
                return out;
            }
            std::reverse(path.begin(), path.end());
            int physical_source_node = -1;
            if (net.kind == RoutingNetKind::PNnet
                && !path.empty()
                && path.front() == net.virtual_source_node
                && path.size() > 1) {
                physical_source_node = path[1];
                path.erase(path.begin());
            }
            out.paths.push_back(SourceSinkPairPath {
                net.net_id,
                pair->source_index,
                demand.demand_id,
                physical_source_node,
                std::move(path)});
        }
    }
    if (out.paths.size() != expected_paths) {
        out.message = std::format(
            "SAT extraction produced {} paths for {} demands",
            out.paths.size(),
            expected_paths);
        return out;
    }

    for (const auto& [group_id, variable] : model.mode_var_by_group) {
        out.vline_mode_straight_by_group.emplace(
            static_cast<std::size_t>(group_id),
            session.value(variable));
    }
    for (const auto& [switch_id, variable] : model.switch_var_by_id) {
        if (session.value(variable)) {
            out.used_tob_switch_ids.push_back(switch_id);
        }
    }

    out.ok = true;
    out.message = "SAT";
    return out;
}

} // namespace PR_tool
