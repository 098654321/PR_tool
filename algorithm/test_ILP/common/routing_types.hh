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
    RoutingNetKind kind{RoutingNetKind::Tnet};
    std::Vector<GraphNodeRef> sources;
    std::Vector<RoutingDemand> demands;
    IlpBoundingBox scope_bbox {};
    bool has_scope_bbox{false};
    bool is_sync_bus{false};
    // V18 fixed-source tree transformed from a Pose/Nege PNnet.
    bool pn_source_tree{false};
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

struct RoutingResult {
    bool ok{false};
    std::String message;
    std::size_t total_wirelength{0};
    std::Vector<SourceSinkPairPath> paths;
    std::map<std::size_t, bool> vline_mode_straight_by_group;
    std::Vector<int> used_tob_switch_ids;
    bool rrr_attempted{false};
    bool rrr_accepted{false};
    std::String rrr_status;
    long long rrr_total_ms{0};
    std::size_t rrr_baseline_wirelength{0};
    std::size_t rrr_wirelength{0};
    std::size_t rrr_triggers{0};
    std::size_t rrr_accepted_triggers{0};
    std::size_t rrr_iterations{0};
    std::size_t rrr_rerouted_owners{0};
};

} // namespace PR_tool
