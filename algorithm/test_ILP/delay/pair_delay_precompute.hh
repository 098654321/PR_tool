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
    // delays(s,t) for constraint 3; always a singleton {d_min} in the current v14 subset.
    std::Vector<int> delays;
    // Alias of delays.front() after precompute; kept for call-site readability.
    int target_delay{0};
    // For sync-bus members only: shortest delay before bus-wide max alignment.
    int member_shortest_delay{-1};
};

struct ReachableDelayTable {
    // reachable[compact_node_offset][d] = true when (node,d) is reachable from source
    std::Vector<std::Vector<bool>> reachable;
    int d_max{0};
};

struct SourceDelayReachability {
    std::size_t net_id{0};
    std::size_t source_index{0};
    int source_node{-1};
    std::size_t scope_index{0};
    int d_max{0};
    ReachableDelayTable table;
};

struct DelayPrecomputeResult {
    std::Vector<PairDelayInfo> pairs;
    std::Vector<SourceDelayReachability> sources;
};

auto bfs_reachability(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    int source_node,
    int d_max_bound
) -> ReachableDelayTable;

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
