#pragma once

#include "rrr_router.hh"
#include "resource_model.hh"

#include <limits>
#include <utility>

namespace PR_tool::rrr::rrr_detail {

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
    std::size_t sync_target{0};
    std::Vector<std::Vector<int>> demand_paths;
    std::Set<int> tree;
    std::Set<ResourceKey> claimed;
};

struct SyncViolation {
    int unequal_groups{0};
    std::size_t total_gap{0};
};

auto ref_row_col(const GraphNodeRef& ref) -> std::pair<int, int>;

auto hpwl_of_refs(const std::Vector<GraphNodeRef>& refs) -> int;

auto net_terminals(const RoutingNet& net) -> std::Vector<GraphNodeRef>;

auto member_terminals(const RoutingNet& net, const RoutingDemand& demand) -> std::Vector<GraphNodeRef>;

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

auto owner_paths_valid(const UnifiedGraph& graph, const OwnerRecord& owner) -> bool;

auto initial_order(const std::Vector<OwnerRecord>& owners) -> std::Vector<std::size_t>;

auto initial_type_order(
    const std::Vector<OwnerRecord>& owners,
    const std::Vector<RoutingNet>& nets
) -> std::Vector<std::size_t>;

auto add_tree_node(OwnerRecord& owner, const UnifiedGraph& graph, int node) -> void;

auto rip_owner(OwnerRecord& owner, ResourceModel& resources) -> void;

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

} // namespace PR_tool::rrr::rrr_detail
