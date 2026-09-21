#pragma once

#include "common/ilp_types.hh"

#include <hardware/track/trackcoord.hh>

#include <cstddef>
#include <std/collection.hh>
#include <std/string.hh>
#include <tuple>

namespace PR_tool {

struct IlpBoundingBox {
    std::i64 row_min{0};
    std::i64 row_max{0};
    std::i64 col_min{0};
    std::i64 col_max{0};
};

struct GlobalChannelCoord {
    int dir{0};
    int row{0};
    int col{0};

    auto operator<=>(const GlobalChannelCoord&) const = default;
};

enum class RoutingNetKind {
    Bnet,
    Tnet,
    PNnet
};

struct GraphNodeRef {
    enum class Kind {
        Track,
        Bump,
        HLine,
        VLine
    };

    Kind kind{Kind::Track};
    hardware::TrackCoord track_coord {};
    std::size_t track_index{0};
    Bump_coord bump {};
    std::size_t tob{0};
    std::size_t bank{0};
    std::size_t group{0};
    std::size_t line_index{0};
};

struct RoutingDemand {
    std::size_t demand_id{0};
    GraphNodeRef sink;
    std::Vector<std::size_t> candidate_source_indices;
    bool fixed_pair{false};
};

struct RoutingNet {
    std::size_t net_id{0};
    std::String name;
    std::String origin_uid;
    std::String origin_key;
    RoutingNetKind kind{RoutingNetKind::Tnet};
    std::Vector<GraphNodeRef> sources;
    std::Vector<RoutingDemand> demands;
    IlpBoundingBox scope_bbox {};
    bool has_scope_bbox{false};
    bool is_sync_bus{false};
    // V18 fixed-source tree transformed from a Pose/Nege PNnet.
    bool pn_source_tree{false};
    // Legacy V20 marker retained for initial scope expansion of TrackToBump(s)
    // and PN source trees. V22 post-SAT ILP selects all non-SyncBus nets and
    // does not use this field as an optimization target filter.
    bool post_sat_ilp_target{false};
    // V17 only: non-rectangular Channel guide shared by this normalized net.
    bool has_global_route_guide{false};
    std::set<GlobalChannelCoord> global_route_channels;
    // Bnet only: initial Global Routing unit per logical source.
    std::map<std::size_t, std::size_t> global_unit_by_source;
    // Sources whose Global Routing unit assumption was removed by a Z3 core.
    std::set<std::size_t> released_global_unit_sources;
    // PNnet only: candidate sources selected by its per-demand global commodities.
    std::set<std::size_t> global_selected_pn_source_indices;
    // PNnet only: global graph node id for virtual source r_n (-1 when unset).
    int virtual_source_node{-1};
};

struct SourceSinkPairPath {
    std::size_t net_id{0};
    std::size_t source_index{0};
    std::size_t demand_id{0};
    // PNnet: selected physical track graph node (-1 when unused).
    int physical_source_node{-1};
    std::Vector<int> node_path;
};

struct SatRoutingResult {
    bool ok{false};
    std::String message;
    std::size_t num_vars{0};
    std::size_t num_clauses{0};
    // Detailed-routing objective support. V18 pure SAT keeps all three at zero.
    std::size_t occupancy_vars{0};
    std::size_t occupancy_implication_clauses{0};
    std::size_t occupancy_soft_clauses{0};
    // Sum of CaDiCal solve() wall time across all feedback rounds.
    long long solve_ms{0};
    // Wall time of the whole SAT phase (graph/padding/feedback/extract), excluding ILP.
    long long sat_total_ms{0};
    // sat_total_ms - solve_ms (precompute, model build, extract, diagnostics, ...).
    long long sat_pre_ms{0};
    std::size_t feedback_rounds{0};
    // Sum of per-net deduplicated bump+track resource counts (post-solve stat only).
    std::size_t total_wirelength{0};
    std::Vector<SourceSinkPairPath> paths;
    std::map<std::size_t, bool> vline_mode_straight_by_group;
    std::Vector<int> used_tob_switch_ids;
    // V17/V18 Global Routing diagnostics; detailed routing may use Z3 or CaDiCaL.
    bool global_route_requested{false};
    std::String global_route_status;
    std::size_t global_route_nodes{0};
    std::size_t global_route_cob_nodes{0};
    std::size_t global_route_tob_terminal_nodes{0};
    std::size_t global_route_port_terminal_nodes{0};
    std::size_t global_route_boundary_terminal_nodes{0};
    std::size_t global_route_channels{0};
    std::size_t global_route_arcs{0};
    std::size_t global_route_owners{0};
    std::size_t global_route_commodities{0};
    std::size_t global_route_vars{0};
    std::size_t global_route_constraints{0};
    std::size_t global_route_objective{0};
    std::size_t global_route_estimated_wirelength{0};
    std::size_t global_route_released_sources{0};
    bool global_route_capacity_cuts_enabled{false};
    std::size_t global_route_capacity_cut_rounds{0};
    std::size_t global_route_capacity_cuts{0};
    long long global_route_build_ms{0};
    long long global_route_solve_ms{0};
    long long global_route_total_ms{0};
    // Diagnostic work deliberately excluded from all routing wall-clock summaries.
    long long excluded_diagnostic_ms{0};
    // V22 post-Maze whole-route-candidate selection ILP summary. A failed
    // refinement preserves the complete Maze result (or its SAT fallback).
    bool post_sat_ilp_attempted{false};
    bool post_sat_ilp_accepted{false};
    std::String post_sat_ilp_status;
    long long post_sat_ilp_total_ms{0};
    long long post_sat_ilp_build_ms{0};
    long long post_sat_ilp_solve_ms{0};
    std::size_t post_sat_ilp_baseline_wirelength{0};
    std::size_t post_sat_ilp_wirelength{0};
    // Legacy field name; V22 stores the number of optimized non-Sync nets.
    std::size_t post_sat_ilp_parents{0};
    // Legacy field name; V22 stores the number of route-tree candidates here.
    std::size_t post_sat_ilp_segments{0};
    std::size_t post_sat_ilp_components{0};
    std::size_t post_sat_ilp_variables{0};
    std::size_t post_sat_ilp_constraints{0};
    double post_sat_ilp_objective{0.0};
    double post_sat_ilp_bound{0.0};
    double post_sat_ilp_gap{0.0};
    // V20 post-SAT maze/RRR summary. The SAT result is retained on every
    // failed or non-improving transaction.
    bool post_sat_maze_attempted{false};
    bool post_sat_maze_accepted{false};
    std::String post_sat_maze_status;
    long long post_sat_maze_total_ms{0};
    std::size_t post_sat_maze_baseline_wirelength{0};
    std::size_t post_sat_maze_wirelength{0};
    std::size_t post_sat_maze_triggers{0};
    std::size_t post_sat_maze_accepted_triggers{0};
    std::size_t post_sat_maze_rrr_iterations{0};
    std::size_t post_sat_maze_rerouted_owners{0};
};

} // namespace PR_tool
