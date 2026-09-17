#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"

namespace PR_tool {

struct PostSatRrrOptions {
  int verbose_level{0};
  int max_iterations{64};
  int stagnation_limit{8};
  int max_sweeps{2};
};

// Uses one non-Sync net as a perturbation seed, permits temporary conflicts
// with other non-Sync nets, then repairs the affected owner set by RRR. Sync
// routes are immutable hard obstacles. Every failed/non-improving transaction
// returns to the previous legal incumbent.
auto optimize_post_sat_routes_rrr(const UnifiedGraph &graph,
                                  const std::Vector<RoutingNet> &nets,
                                  const std::Vector<UnifiedSatNetScope> &scopes,
                                  const SatRoutingResult &sat_result,
                                  const PostSatRrrOptions &options = {})
    -> SatRoutingResult;

} // namespace PR_tool
