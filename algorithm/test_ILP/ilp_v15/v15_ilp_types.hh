#pragma once

#include "common/routing_types.hh"

#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct V15IlpOptimizeOptions {
    bool enabled{false};
    double stretch_threshold_percent{0.0};
    int segment_bbox_pad{0};
    int verbose_level{0};
    std::String gurobi_log_dir{"./gurobi"};
    // Gurobi optimize wall time; unset means unlimited.
    std::optional<double> time_limit_hours;
};

struct V15SegmentScope {
    std::Vector<int> node_ids;
    std::Vector<int> arc_ids;
};

struct V15Segment {
    std::size_t segment_id{0};
    std::size_t parent_id{0};
    int endpoint_a{-1};
    int endpoint_b{-1};
    std::Vector<int> guide_node_path;
    std::Vector<int> guide_arc_ids;
    V15SegmentScope scope;
    bool is_virtual_hop{false};
};

struct V15Parent {
    std::size_t parent_id{0};
    std::size_t routing_net_id{0};
    std::size_t scope_index{0};
    int root_node{-1};
    // PNnet only: every physical candidate track selected by the SAT seed.
    // v15 keeps this set fixed instead of reopening source selection.
    std::Vector<int> fixed_track_nodes;
    std::Vector<int> origin_sinks;
    std::Vector<std::size_t> demand_ids;
    std::Vector<std::size_t> source_indices;
    bool is_bus_member{false};
    bool decomposed{false};
    std::uint16_t source_unit_mask{0xffff};
    std::Vector<std::size_t> segment_ids;
    std::size_t tree_nodes_before_prune{0};
    std::size_t tree_edges_before_prune{0};
    std::size_t tree_nodes_after_prune{0};
    std::size_t tree_edges_after_prune{0};
};

struct V15PrepareResult {
    std::Vector<V15Parent> parents;
    std::Vector<V15Segment> segments;
};

struct V15LockedResources {
    std::Vector<bool> node_used;
    std::set<int> switch_used;
};

struct V15MipStart {
    bool available{false};
    std::map<std::size_t, std::set<int>> segment_flow_arc_ids;
    std::map<std::size_t, std::set<int>> parent_arc_ids;
    std::map<std::size_t, std::set<int>> parent_node_ids;
    std::map<int, bool> mode_straight;
};

enum class V15IlpStatus {
    SkippedNoCandidates,
    Optimal,
    Suboptimal,
    Failed,
};

struct V15IlpStats {
    std::size_t selected_nets{0};
    std::size_t locked_nets{0};
    std::size_t parents{0};
    std::size_t segments{0};
    std::size_t f_vars{0};
    std::size_t x_vars{0};
    std::size_t y_vars{0};
    std::size_t mode_vars{0};
    std::size_t constraints{0};
    std::size_t nonzeros{0};
    long long model_build_ms{0};
    long long solve_ms{0};
    // Full optimize_v15_routes wall time.
    long long total_ms{0};
    // Wall time from ILP begin until model.optimize() starts (prep + model build).
    long long pre_ms{0};
    double objective{0.0};
    double best_bound{0.0};
    double mip_gap{0.0};
    int solution_count{0};
};

struct V15IlpOptimizeResult {
    V15IlpStatus status{V15IlpStatus::Failed};
    std::String message;
    SatRoutingResult routing;
    V15IlpStats stats;
    std::set<std::size_t> selected_net_ids;
};

} // namespace PR_tool
