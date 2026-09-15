#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "scope/pair_routing_state.hh"

#include <cstddef>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

enum class GlobalRouteNodeKind {
    Cob,
    TobTerminal,
    PortTerminal,
    BoundaryTerminal
};

struct GlobalPortKey {
    int dir{0};
    int row{0};
    int col{0};
    std::size_t track_index{0};

    auto operator<=>(const GlobalPortKey&) const = default;
};

struct GlobalRouteNode {
    GlobalRouteNodeKind kind{GlobalRouteNodeKind::Cob};
    int row{-1};
    int col{-1};
    std::size_t tob{0};
    int channel{-1};
    GlobalPortKey port {};
};

struct GlobalChannelArc {
    int u{-1};
    int v{-1};
    // Every traversal arc consumes exactly one physical Channel resource.
    int channel{-1};
    // Port attachment arcs are private to commodities that own this terminal.
    int restricted_port_node{-1};
};

struct GlobalChannelGraph {
    std::Vector<GlobalRouteNode> nodes;
    std::Vector<GlobalChannelCoord> channels;
    std::Vector<GlobalChannelArc> arcs;
    std::map<GlobalChannelCoord, int> channel_id_by_coord;
    std::map<std::pair<int, int>, int> cob_node_by_coord;
    std::map<std::size_t, int> tob_node_by_tob;
    std::map<GlobalPortKey, int> port_node_by_key;
    std::map<int, int> boundary_node_by_channel;
    std::Vector<std::Vector<int>> in_arc_ids;
    std::Vector<std::Vector<int>> out_arc_ids;
    std::Vector<std::Vector<int>> arc_ids_by_channel;
    std::Vector<std::Vector<int>> adjacent_channel_ids;
    std::size_t cob_node_count{0};
    std::size_t tob_terminal_node_count{0};
    std::size_t port_terminal_node_count{0};
    std::size_t boundary_terminal_node_count{0};
};

struct GlobalUnitOwnerKey {
    std::size_t net_id{0};
    std::size_t owner_index{0};

    auto operator<=>(const GlobalUnitOwnerKey&) const = default;
};

struct GlobalRouteStats {
    struct ModelBreakdown {
        // Binary variables.
        std::size_t q_vars{0};
        std::size_t x_vars{0};
        std::size_t w_vars{0};
        std::size_t w_dense_slots{0};
        std::size_t f_vars{0};
        std::size_t source_choice_vars{0};

        // Linear constraints, grouped by their modeling role.
        std::size_t q_exactly_one{0};
        std::size_t source_exactly_one{0};
        std::size_t w_linearization{0};
        std::size_t flow_conservation{0};
        std::size_t flow_implies_channel{0};
        std::size_t source_implies_channel{0};
        std::size_t terminal_channel{0};
        std::size_t channel_flow_support{0};
        std::size_t pn_source_unit_coupling{0};
        std::size_t channel_unit_capacity{0};
        std::size_t tob_unit_capacity{0};
        std::size_t tob_bank_residue_capacity{0};
        std::size_t sync_bus_equal_length{0};

        [[nodiscard]] auto total_variables() const -> std::size_t;
        [[nodiscard]] auto total_constraints() const -> std::size_t;
    } model;

    std::size_t nodes{0};
    std::size_t cob_nodes{0};
    std::size_t tob_terminal_nodes{0};
    std::size_t port_terminal_nodes{0};
    std::size_t boundary_terminal_nodes{0};
    std::size_t channels{0};
    std::size_t arcs{0};
    std::size_t owners{0};
    std::size_t commodities{0};
    std::size_t variables{0};
    std::size_t constraints{0};
    std::size_t objective{0};
    long long build_ms{0};
    long long solve_ms{0};
    long long total_ms{0};
};

struct GlobalRouteResult {
    bool ok{false};
    std::String message;
    GlobalRouteStats stats;
    std::map<PairKey, std::set<GlobalChannelCoord>> pair_channels;
    // Selected directed macro arcs and their derived physical COBs per commodity.
    std::map<PairKey, std::Vector<int>> selected_arc_ids_by_pair;
    std::map<PairKey, std::set<std::pair<int, int>>> pair_cobs;
    std::map<GlobalUnitOwnerKey, std::size_t> unit_by_owner;
    std::map<PairKey, std::size_t> selected_source_index_by_pair;
    std::map<GlobalUnitOwnerKey, std::size_t> channel_count_by_owner;
    std::map<PairKey, int> detailed_distance_cap_by_pair;
};

auto build_global_channel_graph(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets
) -> GlobalChannelGraph;

auto solve_global_route_v17(
    const UnifiedGraph& graph,
    const GlobalChannelGraph& channel_graph,
    const std::Vector<RoutingNet>& nets,
    int verbose_level
) -> GlobalRouteResult;

auto apply_global_route_v17(
    const GlobalRouteResult& route,
    RoutingProblemState& state,
    std::Vector<RoutingNet>& nets
) -> void;

auto expand_global_route_guides_one_hop(
    const GlobalChannelGraph& graph,
    RoutingProblemState& state,
    const std::Vector<PairKey>& critical_pairs
) -> std::size_t;

auto all_global_route_guides_full(const RoutingProblemState& state, std::size_t channel_count)
    -> bool;

} // namespace PR_tool
