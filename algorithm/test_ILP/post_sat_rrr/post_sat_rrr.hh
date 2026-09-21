#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"

#include <optional>

namespace PR_tool {

struct PostSatRrrOptions {
  int verbose_level{0};
  int max_iterations{1000};
  int stagnation_limit{100};
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

// Rebuilds every non-Sync logical owner from an empty non-Sync routing state.
// Sync paths remain immutable hard obstacles.  Unlike the public V20 sweep,
// this primitive does not touch post_sat_maze_* statistics; V22 uses it after
// regenerating compact Channel guides and owns its own acceptance/fallback.
auto rebuild_all_non_sync_routes_rrr(
    const UnifiedGraph &graph, const std::Vector<RoutingNet> &nets,
    const std::Vector<UnifiedSatNetScope> &scopes,
    const SatRoutingResult &sat_result,
    const PostSatRrrOptions &options = {}) -> std::optional<SatRoutingResult>;

} // namespace PR_tool
