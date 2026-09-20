#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"
#include "scope/pair_routing_state.hh"

#include <string_view>

namespace PR_tool {

struct PostSatIlpOptions {
    int verbose_level{0};
    std::string_view highs_log_path;
    int highs_time_limit_minutes{0};
};

// V22 selects every non-SyncBus net. SyncBus is the sole fixed obstacle.
auto is_post_sat_ilp_target(const RoutingNet &net) -> bool;

// Always returns a legal routing result.  Any preparation, solve, extraction,
// validation or non-degradation failure returns the original SAT routing and
// records the fallback reason in the post_sat_ilp_* summary fields.
auto optimize_post_sat_routes(
    const UnifiedGraph &graph, const std::Vector<RoutingNet> &nets,
    const std::Vector<UnifiedSatNetScope> &scopes,
    const SatRoutingResult &sat_result, const PostSatIlpOptions &options,
    // Production passes the final per-pair guide state.
    // A null pointer is retained only for synthetic legacy
    // callers, where the supplied net scope is the pair scope.
    const RoutingProblemState *pair_state = nullptr) -> SatRoutingResult;

} // namespace PR_tool
