#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat_allocation/cadical_solver.hh"

namespace PR_tool {

auto extract_sat_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const UnifiedSatModel& model,
    const CadicalSession& session,
    const CadicalSolveResult& solve_result
) -> SatRoutingResult;

} // namespace PR_tool
