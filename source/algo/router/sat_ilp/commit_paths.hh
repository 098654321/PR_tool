#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"

#include <circuit/basedie.hh>
#include <hardware/interposer.hh>
#include <std/string.hh>

namespace PR_tool {

struct CommitPathsResult {
    bool ok{false};
    std::String message;
};

auto commit_sat_paths_to_nets(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& routing_nets,
    const SatRoutingResult& sat_result
) -> CommitPathsResult;

} // namespace PR_tool
