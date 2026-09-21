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

// First applies the final-scope V20 Maze/RRR refinement, then discards scope and
// generates full-space, fixed-unit/source route-tree candidates. The Maze route
// trees are ILP incumbent candidates and the fallback for any non-improvement.
auto optimize_post_sat_routes(
    const UnifiedGraph &graph, const std::Vector<RoutingNet> &nets,
    const std::Vector<UnifiedSatNetScope> &scopes,
    const SatRoutingResult &sat_result,
    const PostSatIlpOptions &options) -> SatRoutingResult;

} // namespace PR_tool
