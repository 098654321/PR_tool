#pragma once

#include <hardware/track/trackcoord.hh>

#include <cstddef>
#include <std/collection.hh>
#include <std/integer.hh>
#include <std/string.hh>
#include <tuple>

namespace PR_tool {

struct RrrCliOptions {
    std::String config_path;
    std::String output_dir{"."};
    int verbose_level{0};
    int max_iterations{64};
    int seed{1};
};

struct RrrResult {
    std::String status;
    int iterations{0};
    int best_overflow{0};
    int unequal_sync_groups{0};
    std::size_t total_sync_gap{0};
    std::size_t total_wirelength{0};
    std::Vector<std::Vector<std::Vector<int>>> paths;
    std::i64 routing_ms{0};
};

struct OwnerId {
    std::size_t net_id{0};
    std::size_t demand_id{0};

    auto operator==(const OwnerId& other) const -> bool {
        return net_id == other.net_id && demand_id == other.demand_id;
    }

    auto operator<(const OwnerId& other) const -> bool {
        return std::tie(net_id, demand_id) < std::tie(other.net_id, other.demand_id);
    }
};

enum class ResourceKind {
    Node,
    PhysicalSwitch,
    MatchingEndpoint,
    ModeStraight,
    ModeSwap,
    ModeConflict,
    BnetUnit,
    TobMuxInput,
    TobMuxOutput
};

struct ResourceKey {
    ResourceKind kind{ResourceKind::Node};
    int id{-1};
    int extra{-1};

    auto operator==(const ResourceKey& other) const -> bool {
        return kind == other.kind && id == other.id && extra == other.extra;
    }

    auto operator<(const ResourceKey& other) const -> bool {
        return std::tie(kind, id, extra) < std::tie(other.kind, other.id, other.extra);
    }
};

inline auto node_resource(int node_id) -> ResourceKey {
    return ResourceKey {ResourceKind::Node, node_id, -1};
}

inline auto switch_resource(int physical_switch_id) -> ResourceKey {
    return ResourceKey {ResourceKind::PhysicalSwitch, physical_switch_id, -1};
}

inline auto matching_endpoint_key(int node_id, int side) -> ResourceKey {
    return ResourceKey {ResourceKind::MatchingEndpoint, node_id, side};
}

inline auto mode_straight_key(int mode_group_id) -> ResourceKey {
    return ResourceKey {ResourceKind::ModeStraight, mode_group_id, -1};
}

inline auto mode_swap_key(int mode_group_id) -> ResourceKey {
    return ResourceKey {ResourceKind::ModeSwap, mode_group_id, -1};
}

inline auto mode_conflict_key(int mode_group_id) -> ResourceKey {
    return ResourceKey {ResourceKind::ModeConflict, mode_group_id, -1};
}

inline auto bnet_unit_key(int unit) -> ResourceKey {
    return ResourceKey {ResourceKind::BnetUnit, unit, -1};
}

inline auto tob_mux_input_key(int node_id, int peer_node_id) -> ResourceKey {
    return ResourceKey {ResourceKind::TobMuxInput, node_id, peer_node_id};
}

inline auto tob_mux_output_key(int node_id, int peer_node_id) -> ResourceKey {
    return ResourceKey {ResourceKind::TobMuxOutput, node_id, peer_node_id};
}

inline auto is_mux_port_key(const ResourceKey& key) -> bool {
    return key.kind == ResourceKind::TobMuxInput || key.kind == ResourceKind::TobMuxOutput;
}

inline auto is_physical_occupancy_key(const ResourceKey& key) -> bool {
    return key.kind == ResourceKind::Node
        || key.kind == ResourceKind::PhysicalSwitch
        || key.kind == ResourceKind::MatchingEndpoint
        || is_mux_port_key(key);
}

struct RrrParams {
    int H{4};
    double k{1.0};
    int s{20};
    double decay{0.9};
    double increment{1};
    double history_weight{1};
    double detour_bias{0};
    int max_iterations{64};
    int stagnation_limit{8};
    int sync_tail_extra_tracks{64};
    int seed{1};
    std::Vector<double> r_sequence{0.5, 0.75, 1.0};
};

struct Bump_coord {
    std::size_t TOB;
    std::size_t Bank;
    std::size_t Group;
    std::size_t Index;

    auto operator==(const Bump_coord& other) const -> bool {
        return TOB == other.TOB && Bank == other.Bank && Group == other.Group && Index == other.Index;
    }

    auto operator<(const Bump_coord& other) const -> bool {
        return std::tie(TOB, Bank, Group, Index) < std::tie(other.TOB, other.Bank, other.Group, other.Index);
    }
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
    bool is_sync_bus{false};
};

} // namespace PR_tool
