#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"

#include <std/collection.hh>

namespace PR_tool {

struct PairKey {
    std::size_t net_id{0};
    std::size_t demand_id{0};
    std::size_t source_index{0};

    auto operator<=>(const PairKey&) const = default;
};

struct PairRoutingState {
    PairKey key;
    std::Vector<int> delays;
    IlpBoundingBox pair_bbox {};
};

struct RoutingProblemState {
    std::Vector<PairRoutingState> pairs;
    std::map<PairKey, std::size_t> pair_index_by_key;
    std::map<std::size_t, std::Vector<std::size_t>> pair_indices_by_net;
};

auto init_routing_problem_state(const std::Vector<RoutingNet>& nets) -> RoutingProblemState;

auto apply_state_to_nets(const RoutingProblemState& state, std::Vector<RoutingNet>& nets) -> void;

auto find_pair_state(RoutingProblemState& state, const PairKey& key) -> PairRoutingState*;

auto find_pair_state(const RoutingProblemState& state, const PairKey& key) -> const PairRoutingState*;

auto append_delays(std::Vector<int>& delays, int d1, int d2) -> void;

auto max_delay(const std::Vector<int>& delays) -> int;

auto expand_pair_delays(PairRoutingState& pair) -> void;

auto merge_delays_union(std::Vector<int>& target, const std::Vector<int>& extra) -> void;

auto sync_fanout_delays(RoutingProblemState& state, std::size_t net_id) -> void;

auto sync_bus_after_expand(
    RoutingProblemState& state,
    const std::Vector<RoutingNet>& nets,
    std::size_t net_id
) -> void;

auto all_pair_bboxes_full(const RoutingProblemState& state) -> bool;

auto apply_initial_search_padding(
    RoutingProblemState& state,
    std::Vector<RoutingNet>& nets,
    const UnifiedGraph& graph,
    int scope_pad,
    int delay_pad
) -> void;

} // namespace PR_tool
