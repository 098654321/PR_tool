#pragma once

#include "graph/unified_routing_graph.hh"

namespace PR_tool {

struct DirectEdge {
    int u{-1};
    int v{-1};
    std::Vector<int> arcs;
    int switch_id{-1};
    PhysicalSwitchKind switch_kind{PhysicalSwitchKind::None};
    int mode_group{-1};
    bool straight{false};
    bool swap{false};
    bool virtual_source{false};
};

struct DirectGraph {
    std::Vector<DirectEdge> edges;
    std::Vector<std::Vector<int>> incident;
    std::Vector<int> edge_by_arc;
};

auto build_direct_graph(const UnifiedGraph& graph) -> DirectGraph;

} // namespace PR_tool
