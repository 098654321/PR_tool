#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"

#include <hardware/track/trackcoord.hh>

#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

enum class NetDisplayKind {
    TwoPin,
    SyncBus,
    TrackToBumps,
    TracksToBumps
};

auto infer_net_display_kind(const RoutingNet& net) -> NetDisplayKind;

auto net_display_kind_name(NetDisplayKind kind) -> std::String;

auto format_track_coord(const hardware::TrackCoord& coord) -> std::String;

auto format_graph_node_ref(const GraphNodeRef& ref) -> std::String;

auto format_path_node(const UnifiedGraph& graph, int node_id) -> std::String;

auto format_bbox_corners(const IlpBoundingBox& box) -> std::String;

auto format_path_hops(const UnifiedGraph& graph, const std::Vector<int>& node_path) -> std::String;

auto path_wirelength(const UnifiedGraph& graph, const std::Vector<int>& node_path) -> std::size_t;

auto net_wirelength(
    const UnifiedGraph& graph,
    const std::Vector<const SourceSinkPairPath*>& paths
) -> std::size_t;

auto total_wirelength(const UnifiedGraph& graph, const SatRoutingResult& result) -> std::size_t;

auto log_scope_bboxes(const std::Vector<RoutingNet>& nets, int verbose_level) -> void;

auto log_routing_paths(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const SatRoutingResult& result
) -> void;

} // namespace PR_tool
