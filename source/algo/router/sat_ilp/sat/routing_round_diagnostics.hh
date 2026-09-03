#pragma once

#include "common/routing_types.hh"
#include "delay/pair_delay_precompute.hh"
#include "graph/unified_routing_graph.hh"
#include "scope/pair_routing_state.hh"

#include <hardware/interposer.hh>

#include <cstddef>
#include <std/collection.hh>

namespace PR_tool {

enum class FeedbackRoundStatus {
    SatSuccess,
    UnsatExpand,
    UnsatExhausted,
    SolverError,
    ValidationFailed,
    MemoryLimit,
    MaxRoundsExceeded,
};

struct NetStretchInfo {
    std::size_t net_id{0};
    std::String name;
    std::size_t actual{0};
    std::size_t shortest{0};
    double delta_percent{0.0};
};

auto collect_net_stretch_info(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const DelayPrecomputeResult& delays,
    const SatRoutingResult& result
) -> std::Vector<NetStretchInfo>;

auto feedback_round_status_name(FeedbackRoundStatus status) -> std::String;

auto unique_failed_net_ids(const std::Vector<PairKey>& critical) -> std::Vector<std::size_t>;

auto log_feedback_round_begin(std::size_t round) -> void;

auto log_feedback_round_end(std::size_t round, FeedbackRoundStatus status) -> void;

auto log_failed_nets(
    const std::Vector<RoutingNet>& nets,
    const std::Vector<PairKey>& critical
) -> void;

auto log_non_shortest_nets(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const DelayPrecomputeResult& delays,
    const SatRoutingResult& result
) -> void;

} // namespace PR_tool
