#include "post_sat_ilp/post_sat_ilp.hh"

#include "post_sat_ilp/candidate_router.hh"
#include "post_sat_rrr/post_sat_rrr.hh"

#include <debug/debug.hh>

namespace PR_tool {

auto optimize_post_sat_routes(
    const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const SatRoutingResult& sat_result,
    const PostSatIlpOptions& options) -> SatRoutingResult {
    auto maze_result = optimize_post_sat_routes_rrr(
        graph, nets, scopes, sat_result,
        PostSatRrrOptions{.verbose_level = options.verbose_level});
    debug::info_fmt(
        "V22 post-SAT pipeline handoff: SAT_wirelength={} maze_status={} maze_wirelength={} -> full-space candidate ILP",
        sat_result.total_wirelength, maze_result.post_sat_maze_status,
        maze_result.total_wirelength);
    // Scope is deliberately discarded at this boundary.  Candidate generation
    // keeps the Maze-selected unit/PN source and complete route tree as its
    // incumbent, then searches the complete fixed-unit detailed graph.
    return optimize_post_sat_candidate_routes(
        graph, nets, maze_result, options);
}

} // namespace PR_tool
