#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"

#include <string_view>

namespace PR_tool {

struct PostSatIlpOptions {
    int verbose_level{0};
    std::string_view highs_log_path;
    int highs_time_limit_minutes{0};
};

// Generates full-space, fixed-unit/source route-tree candidates and selects one
// whole candidate per non-Sync net with a resource-conflict ILP. Any generation,
// solve, validation or non-improvement failure returns the original SAT routing.
auto optimize_post_sat_routes(
    const UnifiedGraph &graph, const std::Vector<RoutingNet> &nets,
    const std::Vector<UnifiedSatNetScope> &scopes,
    const SatRoutingResult &sat_result,
    const PostSatIlpOptions &options) -> SatRoutingResult;

} // namespace PR_tool
