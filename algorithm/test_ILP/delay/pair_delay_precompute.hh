#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"
#include "scope/pair_routing_state.hh"

#include <std/collection.hh>

namespace PR_tool {

struct PairDelayInfo {
    std::size_t net_id{0};
    std::size_t demand_id{0};
    std::size_t source_index{0};
    int source_node{-1};
    int sink_node{-1};
    // Allowed sink delays for constraint 3: {d_min} initially, then optionally expanded.
    std::Vector<int> delays;
    // Maximum allowed delay after precompute.
    int target_delay{0};
    // For sync-bus members only: shortest delay before bus-wide max alignment.
    int member_shortest_delay{-1};
};

struct SourceDelayDomain {
    std::size_t net_id{0};
    std::size_t source_index{0};
    int source_node{-1};
    std::size_t scope_index{0};
    int d_max{0};
};

struct DelayPrecomputeResult {
    std::Vector<PairDelayInfo> pairs;
    std::Vector<SourceDelayDomain> sources;
};

auto bfs_shortest_delay(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    int source_node,
    int sink_node
) -> int;

auto compute_pair_delays(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    RoutingProblemState* problem_state = nullptr
) -> DelayPrecomputeResult;

auto log_delay_precompute(
    const std::Vector<RoutingNet>& nets,
    const DelayPrecomputeResult& delays,
    int verbose_level
) -> void;

} // namespace PR_tool
