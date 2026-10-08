#pragma once

#include "rrr_router.hh"
#include "resource_model.hh"

#include <limits>
#include <utility>

namespace PR_tool::rrr_detail {

struct OwnerRecord {
    OwnerId id {};
    std::size_t net_index{0};
    std::Vector<std::size_t> demand_indices;
    bool is_bnet{false};
    bool is_sync{false};
    int hpwl{0};
    int group_hpwl{0};
    int port_count{0};
    int lane_count{0};
    int retry{0};
    std::Vector<std::Vector<int>> demand_paths;
    std::Set<int> tree;
    std::Set<ResourceKey> claimed;
};

struct BestSnapshot {
    bool valid{false};
    int overflow{std::numeric_limits<int>::max()};
    int unequal_sync_groups{std::numeric_limits<int>::max()};
    std::size_t sync_gap{std::numeric_limits<std::size_t>::max()};
    std::size_t wirelength{std::numeric_limits<std::size_t>::max()};
    ResourceModel resources;
    std::Vector<OwnerRecord> owners;
};

struct SyncViolation {
    int unequal_groups{0};
    std::size_t total_gap{0};
};

auto ref_row_col(const GraphNodeRef& ref) -> std::pair<int, int>;

auto hpwl_of_refs(const std::Vector<GraphNodeRef>& refs) -> int;

auto net_terminals(const RoutingNet& net) -> std::Vector<GraphNodeRef>;

auto member_terminals(const RoutingNet& net, const RoutingDemand& demand) -> std::Vector<GraphNodeRef>;

auto owner_exposure(const OwnerRecord& owner, const ResourceModel& resources) -> int;

auto owner_is_dirty(const OwnerRecord& owner, const ResourceModel& resources) -> bool;

auto max_resource_overflow(const std::Vector<OwnerRecord>& owners, const ResourceModel& resources)
    -> int;

auto collect_paths_by_net(
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners
) -> std::Vector<std::Vector<std::Vector<int>>>;

auto current_wirelength(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners
) -> std::size_t;

auto demand_source_nodes(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const RoutingDemand& demand
) -> std::Vector<int>;

auto build_owners(const std::Vector<RoutingNet>& nets) -> std::Vector<OwnerRecord>;

auto initial_order(const std::Vector<OwnerRecord>& owners) -> std::Vector<std::size_t>;

auto find_owner_index(const std::Vector<OwnerRecord>& owners, OwnerId id) -> std::size_t;

auto add_tree_node(OwnerRecord& owner, const UnifiedGraph& graph, int node) -> void;

auto rip_owner(OwnerRecord& owner, ResourceModel& resources) -> void;

auto route_owner(
    OwnerRecord& owner,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    ResourceModel& resources,
    RrrParams& params,
    hardware::Interposer* interposer
) -> void;

auto group_owner_indices(const std::Vector<OwnerRecord>& owners, std::size_t net_index)
    -> std::Vector<std::size_t>;

auto sibling_hard_block(
    const std::Vector<OwnerRecord>& owners,
    const std::Vector<std::size_t>& group,
    std::size_t skip
) -> std::Set<ResourceKey>;

auto refresh_owner_from_path(OwnerRecord& owner, const UnifiedGraph& graph) -> void;

auto route_sync_group(
    std::size_t net_index,
    std::Vector<OwnerRecord>& owners,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    ResourceModel& resources,
    RrrParams& params,
    hardware::Interposer* interposer
) -> bool;

auto sync_violation(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners,
    hardware::Interposer* interposer
) -> SyncViolation;

auto unequal_sync_owner_ids(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners,
    hardware::Interposer* interposer
) -> std::Vector<OwnerId>;

auto save_best_if_improved(
    BestSnapshot& best,
    int overflow,
    const SyncViolation& sync,
    std::size_t wirelength,
    const ResourceModel& resources,
    const std::Vector<OwnerRecord>& owners
) -> bool;

auto routing_kind_name(const RoutingNet& net) -> std::String;

auto format_node_ref(const UnifiedGraph& graph, const GraphNodeRef& ref) -> std::String;

auto format_sources(const UnifiedGraph& graph, const RoutingNet& net) -> std::String;

auto format_node_path(const UnifiedGraph& graph, const std::Vector<int>& path) -> std::String;

auto dump_paths(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners,
    hardware::Interposer* interposer
) -> void;

auto format_overflow_owner(const OwnerId& id, const std::Vector<RoutingNet>& nets) -> std::String;

auto dump_overflows(
    const std::Vector<OwnerRecord>& owners,
    const std::Vector<RoutingNet>& nets,
    const ResourceModel& resources
) -> void;

auto all_demands_connected(
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners
) -> bool;

} // namespace PR_tool::rrr_detail
