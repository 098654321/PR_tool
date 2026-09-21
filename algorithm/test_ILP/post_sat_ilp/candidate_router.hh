#pragma once

#include "post_sat_ilp/post_sat_ilp.hh"

namespace PR_tool {

auto optimize_post_sat_candidate_routes(
    const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
    const SatRoutingResult& sat_result,
    const PostSatIlpOptions& options) -> SatRoutingResult;

} // namespace PR_tool
