#pragma once

#include "hardware_graph.hh"
#include "resource_model.hh"
#include "rrr_types.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>

namespace PR_tool {

auto arc_resource_keys(const UnifiedGraph& graph, const UnifiedArc& arc) -> std::Vector<ResourceKey>;

auto path_resource_keys(
    const UnifiedGraph& graph,
    const std::Vector<int>& node_path,
    bool is_bnet = false
) -> std::Vector<ResourceKey>;

auto maze_ordered_out_arc_ids(
    const UnifiedGraph& graph,
    int node_id,
    hardware::Interposer* interposer
) -> std::Vector<int>;

auto arc_incremental_cost(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const UnifiedArc& arc,
    const std::Set<int>& starts,
    const RrrParams& params
) -> double;

auto route_demand(
    const UnifiedGraph& graph,
    const ResourceModel& resources,
    OwnerId owner,
    const std::Vector<int>& sources,
    int sink,
    const RrrParams& params,
    const std::Vector<int>& tree = {},
    bool is_bnet = false,
    hardware::Interposer* interposer = nullptr,
    const std::Set<ResourceKey>& hard_block = {}
) -> std::Vector<int>;

} // namespace PR_tool
