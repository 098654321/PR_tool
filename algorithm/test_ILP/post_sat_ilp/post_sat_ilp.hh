#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"

#include <string_view>

namespace PR_tool {

struct PostSatIlpOptions {
    int verbose_level{0};
    int segment_bbox_pad{0};
    std::string_view highs_log_path;
    int highs_time_limit_minutes{0};
};

// V20 initially refines only fixed physical-Track -> Bump trees.  This covers
// TrackToBump(s) and the Pose/Nege trees produced by V18 PN preselection while
// deliberately leaving Sync, BumpToBump and BumpToTrack routes locked.
auto is_post_sat_ilp_target(const RoutingNet& net) -> bool;

// Always returns a legal routing result.  Any preparation, solve, extraction,
// validation or non-degradation failure returns the original SAT routing and
// records the fallback reason in the post_sat_ilp_* summary fields.
auto optimize_post_sat_routes(const UnifiedGraph& graph,
                              const std::Vector<RoutingNet>& nets,
                              const std::Vector<UnifiedSatNetScope>& scopes,
                              const SatRoutingResult& sat_result,
                              const PostSatIlpOptions& options)
    -> SatRoutingResult;

} // namespace PR_tool
