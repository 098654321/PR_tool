#pragma once

#include "common/routing_types.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>
#include <set>
#include <tuple>

namespace PR_tool {

enum class UnifiedNodeKind {
    Track,
    Bump,
    HLine,
    VLine
};

enum class PhysicalSwitchKind {
    None,
    BumpH,
    HLineVLine,
    VLineTrack
};

struct UnifiedNode {
    UnifiedNodeKind kind{UnifiedNodeKind::Track};
    std::size_t unit{0};
    int track_dir{0};
    int track_row{0};
    int track_col{0};
    std::size_t track_index{0};
    Bump_coord bump {};
    std::size_t tob{0};
    std::size_t bank{0};
    std::size_t group{0};
    std::size_t line_index{0};
};

struct UnifiedArc {
    int u{0};
    int v{0};
    bool is_vline_track_straight{false};
    bool is_vline_track_swap{false};
    int mode_group_id{-1};
    int physical_switch_id{-1};
    PhysicalSwitchKind physical_switch_kind{PhysicalSwitchKind::None};
};

struct UnifiedGraph {
    int rows{0};
    int cols{0};
    std::Vector<UnifiedNode> nodes;
    std::Vector<UnifiedArc> arcs;
    std::Vector<std::Vector<int>> in_arc_ids;
    std::Vector<std::Vector<int>> out_arc_ids;
    std::map<std::tuple<std::size_t, int, int, int, std::size_t>, int> track_node_by_key;
    std::map<Bump_coord, int> bump_node_by_key;
    std::map<std::tuple<std::size_t, std::size_t, std::size_t, std::size_t>, int> hline_node_by_key;
    std::map<std::tuple<std::size_t, std::size_t>, int> vline_node_by_key;
    std::set<std::pair<int, int>> directed_arc_set;
    std::size_t track_node_count{0};
    std::size_t tob_node_count{0};
};

auto build_unified_graph(
    hardware::Interposer* interposer,
    const std::Vector<RoutingNet>& nets
) -> UnifiedGraph;

auto resolve_graph_node(const UnifiedGraph& graph, const GraphNodeRef& ref) -> int;

auto node_in_scope(const UnifiedGraph& graph, int node_id, const IlpBoundingBox& scope) -> bool;

auto format_unified_node(const UnifiedGraph& graph, int node_id) -> std::String;

} // namespace PR_tool
