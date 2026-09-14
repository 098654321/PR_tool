#include "sat/node_occupancy.hh"

#include "sat/routing_path_log.hh"

namespace PR_tool {

auto add_node_occupancy_variables(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const UnifiedSatModel& model
) -> NodeOccupancyVars {
    auto out = NodeOccupancyVars {};
    for (const auto& source : model.sources) {
        const auto& scope = model.scopes[source.scope_index];
        for (std::size_t offset = 0; offset < scope.node_ids.size(); ++offset) {
            const int node = scope.node_ids[offset];
            if (!is_wirelength_resource_node(graph, node)) {
                continue;
            }
            auto d_literals = std::Vector<int> {};
            for (int delay = 0; delay <= source.d_max; ++delay) {
                const int d = source.d_var[offset][static_cast<std::size_t>(delay)];
                if (d > 0) {
                    d_literals.push_back(d);
                }
            }
            if (d_literals.empty()) {
                continue;
            }
            const auto [it, inserted] = out.u_var_by_node.emplace(node, 0);
            if (inserted) {
                it->second = session.new_var();
            }
            for (const int d : d_literals) {
                session.add_clause({-d, it->second});
                ++out.implication_clause_count;
            }
        }
    }
    return out;
}

} // namespace PR_tool
