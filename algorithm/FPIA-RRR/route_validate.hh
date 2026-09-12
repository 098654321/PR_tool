#pragma once

#include "hardware_graph.hh"
#include "rrr_types.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct TobMuxFanoutHit {
    int node_id{-1};
    UnifiedNodeKind kind{UnifiedNodeKind::Track};
    std::String node;
    std::String side;
    int peer_count{0};
    std::Vector<int> peers;
};

auto collect_illegal_tob_fanout(
    const UnifiedGraph& graph,
    const std::Vector<std::Vector<int>>& net_paths
) -> std::Vector<TobMuxFanoutHit>;

auto validate_rrr_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const RrrResult& result,
    hardware::Interposer* interposer = nullptr
) -> bool;

} // namespace PR_tool
