#pragma once

#include "common/routing_scope.hh"
#include "graph/unified_routing_graph.hh"

namespace PR_tool {

// The representation is shared with the maze router; no SAT or global guide
// is needed to construct these scopes.
auto build_direct_scopes(const UnifiedGraph& graph,
                         const std::Vector<RoutingNet>& nets, int verbose_level)
    -> std::Vector<RoutingScope>;

} // namespace PR_tool
