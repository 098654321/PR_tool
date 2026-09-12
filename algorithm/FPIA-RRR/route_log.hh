#pragma once

#include "hardware_graph.hh"

#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

auto format_path_node(const UnifiedGraph& graph, int node_id) -> std::String;

auto path_wirelength(const UnifiedGraph& graph, const std::Vector<int>& node_path) -> std::size_t;

auto net_wirelength(
    const UnifiedGraph& graph,
    const std::Vector<std::Vector<int>>& node_paths
) -> std::size_t;

auto total_wirelength(
    const UnifiedGraph& graph,
    const std::Vector<std::Vector<std::Vector<int>>>& paths_by_net
) -> std::size_t;

} // namespace PR_tool
