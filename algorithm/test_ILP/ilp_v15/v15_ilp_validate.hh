#pragma once

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
    const SatRoutingResult& result
) -> V15ValidationReport;

} // namespace PR_tool
