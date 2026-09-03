#pragma once

#include "common/ilp_types.hh"

#include <hardware/track/trackcoord.hh>

#include <cstddef>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct IlpBoundingBox {
    std::i64 row_min{0};
    std::i64 row_max{0};
    std::i64 col_min{0};
    std::i64 col_max{0};
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
    // Optional v15 post-optimization diagnostics.  The SAT fields above keep
    // their v14 meanings even when Gurobi is enabled.
    bool ilp_optimization_requested{false};
    bool ilp_optimization_applied{false};
    bool ilp_fallback_to_sat{false};
    std::String ilp_status;
    std::size_t ilp_model_vars{0};
    std::size_t ilp_model_constraints{0};
    long long ilp_model_build_ms{0};
    long long ilp_solve_ms{0};
    // Wall time of optimize_v15_routes (prep + model + optimize + extract).
    long long ilp_total_ms{0};
    // Wall time from ILP begin until model.optimize() starts (prep + model build).
    long long ilp_pre_ms{0};
};

} // namespace PR_tool
