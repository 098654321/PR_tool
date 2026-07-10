#pragma once

#include "ilp_v15/v15_ilp_model.hh"
#include "sat/unified_sat_scope.hh"

namespace PR_tool {

struct V15ValidationReport {
    bool ok{false};
    std::size_t checked_paths{0};
    std::Vector<std::String> violations;
};

auto validate_v15_routing_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const V15PrepareResult& prepared,
    const V15IlpModelResult& model_result,
    const SatRoutingResult& result,
    const std::set<std::size_t>& v15_selected_net_ids = {}
) -> V15ValidationReport;

} // namespace PR_tool
