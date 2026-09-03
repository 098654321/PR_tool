#pragma once

#include "ilp_v15/v15_ilp_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"

#include <stdexcept>

namespace PR_tool {

class V15PreparationInvariantError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

auto build_v15_parents_and_segments(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const SatRoutingResult& sat_result,
    const std::set<std::size_t>& selected_net_ids,
    int segment_bbox_pad,
    const V15LockedResources& locked
) -> V15PrepareResult;

} // namespace PR_tool
