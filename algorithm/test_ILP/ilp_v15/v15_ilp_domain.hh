#pragma once

#include "ilp_v15/v15_ilp_types.hh"
#include "graph/unified_routing_graph.hh"

#include <set>

namespace PR_tool {

auto refine_v15_segment_scope(
    const UnifiedGraph& graph,
    const V15SegmentScope& raw_scope,
    const V15Parent& parent,
    const V15Segment& segment,
    const V15LockedResources& locked
) -> V15SegmentScope;

auto build_v15_scope_from_bbox(
    const UnifiedGraph& graph,
    const IlpBoundingBox& bbox,
    const std::Vector<int>& force_nodes,
    const std::set<int>& allowed_virtual_nodes = {}
) -> V15SegmentScope;

} // namespace PR_tool
