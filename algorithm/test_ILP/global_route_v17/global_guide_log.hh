#pragma once

#include "global_route_v17/global_router.hh"

namespace PR_tool {

// Prints the raw MCF guide separately from the TOB-repair/final detailed scope.
// The returned wall time is deliberately diagnostic-only: callers must exclude
// it from end-to-end routing timing summaries.
auto log_global_route_guides(const GlobalRouteResult& route,
                             const GlobalChannelGraph& graph,
                             const RoutingProblemState& state,
                             const std::Vector<RoutingNet>& nets) -> long long;

} // namespace PR_tool
