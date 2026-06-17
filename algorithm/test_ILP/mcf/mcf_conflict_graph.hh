#pragma once

#include <std/collection.hh>
#include <std/string.hh>

#include <set>
#include <utility>

namespace PR_tool {

struct McfCandidateResources {
    std::set<std::pair<int, int>> physical_edges;
    std::set<int> physical_nodes;
};

struct McfConflictComponent {
    std::Vector<int> vertex_ids;
    std::String summary;
};

auto build_edge_node_conflict_components(
    const std::Vector<McfCandidateResources>& per_vertex,
    const std::Vector<std::String>& vertex_labels
) -> std::Vector<McfConflictComponent>;

} // namespace PR_tool
