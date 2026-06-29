#include "sat/sat_solution_extract.hh"

#include <debug/debug.hh>
#include <format>
#include <set>
#include <stdexcept>

namespace PR_tool {

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
            const UnifiedSatPairVars* active = nullptr;
            for (const auto& pair : model.pairs) {
                if (pair.net_id != net.net_id || pair.demand_id != demand.demand_id
                    || !session.value(pair.activation)) {
                    continue;
                }
                if (active != nullptr) {
                    out.message = std::format(
                        "net {} demand {} has multiple active candidate pairs",
                        net.net_id,
                        demand.demand_id);
                    return out;
                }
                active = &pair;
            }
            if (active == nullptr) {
                out.message = std::format(
                    "net {} demand {} has no active candidate pair",
                    net.net_id,
                    demand.demand_id);
                return out;
            }

            const auto& scope = model.scopes[active->scope_index];
            auto path = std::Vector<int> {active->source_node};
            auto visited = std::set<int> {active->source_node};
            int current = active->source_node;
            while (current != active->sink_node) {
                int next = -1;
                std::size_t true_outgoing = 0;
                for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(current)]) {
                    const int offset = scope.arc_offset[static_cast<std::size_t>(arc_id)];
                    if (offset < 0
                        || !session.value(active->x_vars[static_cast<std::size_t>(offset)])) {
                        continue;
                    }
                    ++true_outgoing;
                    next = graph.arcs[static_cast<std::size_t>(arc_id)].v;
                }
                if (true_outgoing == 0) {
                    out.message = std::format(
                        "net {} demand {} path has no next arc before reaching sink",
                        net.net_id,
                        demand.demand_id);
                    return out;
                }
                if (true_outgoing > 1) {
                    out.message = std::format(
                        "net {} demand {} path has multiple next arcs",
                        net.net_id,
                        demand.demand_id);
                    return out;
                }
                if (visited.contains(next)) {
                    out.message = std::format(
                        "net {} demand {} path revisits node {}",
                        net.net_id,
                        demand.demand_id,
                        next);
                    return out;
                }
                visited.insert(next);
                path.push_back(next);
                current = next;
            }

            out.paths.push_back(SourceSinkPairPath {
                net.net_id,
                active->source_index,
                demand.demand_id,
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
    debug::info_fmt("extracted {} routed demand path(s)", out.paths.size());
    return out;
}

} // namespace PR_tool
