#pragma once

#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat_allocation/cadical_solver.hh"

#include <map>

namespace PR_tool {

struct NodeOccupancyVars {
    // Global physical node id -> U_v.  Existing D-state exclusivity makes one
    // global variable sufficient for the total union-wirelength objective.
    std::map<int, int> u_var_by_node;
    std::size_t implication_clause_count{0};
};

auto add_node_occupancy_variables(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const UnifiedSatModel& model
) -> NodeOccupancyVars;

} // namespace PR_tool
