#include "post_sat_ilp/post_sat_ilp.hh"

#include "post_sat_ilp/candidate_router.hh"

namespace PR_tool {

auto optimize_post_sat_routes(
    const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const SatRoutingResult& sat_result,
    const PostSatIlpOptions& options) -> SatRoutingResult {
    // V22 deliberately searches the complete fixed-unit detailed graph.  The
    // final SAT scopes remain relevant only to the separate --maze-optimize.
    (void)scopes;
    return optimize_post_sat_candidate_routes(
        graph, nets, sat_result, options);
}

} // namespace PR_tool
