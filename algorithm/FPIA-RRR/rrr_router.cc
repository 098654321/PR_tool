#include "rrr_router.hh"

#include "hw_map.hh"
#include "maze_search.hh"
#include "resource_model.hh"
#include "route_log.hh"
#include "sync_equalize.hh"

#include <debug/debug.hh>
#include <utility/elapsed.hh>

#include <algorithm>
#include <chrono>
#include <format>
#include <limits>
#include <stdexcept>
#include <utility>

namespace PR_tool {

auto rrr_is_better(
    int overflow,
    std::size_t wirelength,
    int best_overflow,
    std::size_t best_wirelength
) -> bool {
    if (overflow != best_overflow) {
        return overflow < best_overflow;
    }
    return wirelength < best_wirelength;
}

auto rrr_dirty_before(
    int exposure_a,
    int retry_a,
    int hpwl_a,
    OwnerId id_a,
    int exposure_b,
    int retry_b,
    int hpwl_b,
    OwnerId id_b
) -> bool {
    if (exposure_a != exposure_b) {
        return exposure_a > exposure_b;
    }
    if (retry_a != retry_b) {
        return retry_a > retry_b;
    }
    if (hpwl_a != hpwl_b) {
        return hpwl_a > hpwl_b;
    }
    return id_a < id_b;
}

auto rrr_initial_before(
    bool bus_a,
    int primary_a,
    int hpwl_a,
    OwnerId id_a,
    bool bus_b,
    int primary_b,
    int hpwl_b,
    OwnerId id_b
) -> bool {
    if (bus_a != bus_b) {
        return bus_a && !bus_b;
    }
    if (primary_a != primary_b) {
        return primary_a > primary_b;
    }
    if (hpwl_a != hpwl_b) {
        return hpwl_a > hpwl_b;
    }
    return id_a < id_b;
}

namespace {

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
    std::size_t wirelength{std::numeric_limits<std::size_t>::max()};
    ResourceModel resources;
    std::Vector<OwnerRecord> owners;
};

auto ref_row_col(const GraphNodeRef& ref) -> std::pair<int, int> {
    if (ref.kind == GraphNodeRef::Kind::Track) {
        return {
            static_cast<int>(ref.track_coord.row),
            static_cast<int>(ref.track_coord.col)};
    }
    if (ref.kind == GraphNodeRef::Kind::Bump) {
        const auto cob = tob_anchor_cob(ref.bump.TOB);
        return {static_cast<int>(cob.row), static_cast<int>(cob.col)};
    }
    return {0, 0};
}

auto hpwl_of_refs(const std::Vector<GraphNodeRef>& refs) -> int {
    if (refs.empty()) {
        return 0;
    }
    int row_min = std::numeric_limits<int>::max();
    int row_max = std::numeric_limits<int>::min();
    int col_min = std::numeric_limits<int>::max();
    int col_max = std::numeric_limits<int>::min();
    for (const auto& ref : refs) {
        const auto [row, col] = ref_row_col(ref);
        row_min = std::min(row_min, row);
        row_max = std::max(row_max, row);
        col_min = std::min(col_min, col);
        col_max = std::max(col_max, col);
    }
    return (row_max - row_min) + (col_max - col_min);
}

auto net_terminals(const RoutingNet& net) -> std::Vector<GraphNodeRef> {
    auto refs = net.sources;
    for (const auto& demand : net.demands) {
        refs.push_back(demand.sink);
    }
    return refs;
}

auto member_terminals(const RoutingNet& net, const RoutingDemand& demand) -> std::Vector<GraphNodeRef> {
    auto refs = std::Vector<GraphNodeRef> {};
    for (const std::size_t index : demand.candidate_source_indices) {
        if (index < net.sources.size()) {
            refs.push_back(net.sources[index]);
        }
    }
    refs.push_back(demand.sink);
    return refs;
}

auto owner_exposure(const OwnerRecord& owner, const ResourceModel& resources) -> int {
    int exposure = 0;
    for (const auto& key : owner.claimed) {
        exposure += resources.overflow(key);
        if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
            exposure += resources.overflow(mode_conflict_key(key.id));
        }
    }
    return exposure;
}

auto owner_is_dirty(const OwnerRecord& owner, const ResourceModel& resources) -> bool {
    int unit_keys = 0;
    for (const auto& key : owner.claimed) {
        if (resources.overflow(key) > 0) {
            return true;
        }
        if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
            if (resources.overflow(mode_conflict_key(key.id)) > 0) {
                return true;
            }
        }
        if (key.kind == ResourceKind::BnetUnit) {
            ++unit_keys;
        }
    }
    return unit_keys > 1;
}

auto max_resource_overflow(const std::Vector<OwnerRecord>& owners, const ResourceModel& resources)
    -> int {
    int max_ov = 0;
    for (const auto& owner : owners) {
        int unit_keys = 0;
        for (const auto& key : owner.claimed) {
            max_ov = std::max(max_ov, resources.overflow(key));
            if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
                max_ov = std::max(max_ov, resources.overflow(mode_conflict_key(key.id)));
            }
            if (key.kind == ResourceKind::BnetUnit) {
                ++unit_keys;
            }
        }
        max_ov = std::max(max_ov, std::max(0, unit_keys - 1));
    }
    return max_ov;
}

auto collect_paths_by_net(
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners
) -> std::Vector<std::Vector<std::Vector<int>>> {
    auto paths = std::Vector<std::Vector<std::Vector<int>>> {};
    paths.resize(nets.size());
    for (const auto& owner : owners) {
        if (owner.net_index >= paths.size()) {
            continue;
        }
        for (const auto& path : owner.demand_paths) {
            if (!path.empty()) {
                paths[owner.net_index].push_back(path);
            }
        }
    }
    return paths;
}

auto current_wirelength(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners
) -> std::size_t {
    return total_wirelength(graph, collect_paths_by_net(nets, owners));
}

auto demand_source_nodes(
    const UnifiedGraph& graph,
    const RoutingNet& net,
    const RoutingDemand& demand
) -> std::Vector<int> {
    auto nodes = std::Vector<int> {};
    for (const std::size_t index : demand.candidate_source_indices) {
        if (index >= net.sources.size()) {
            continue;
        }
        const int node = resolve_graph_node(graph, net.sources[index]);
        if (node >= 0) {
            nodes.push_back(node);
        }
    }
    return nodes;
}

auto tree_as_vector(const std::Set<int>& tree) -> std::Vector<int> {
    auto nodes = std::Vector<int> {};
    nodes.reserve(tree.size());
    for (const int node : tree) {
        nodes.push_back(node);
    }
    return nodes;
}

auto build_owners(const std::Vector<RoutingNet>& nets) -> std::Vector<OwnerRecord> {
    auto owners = std::Vector<OwnerRecord> {};
    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        const auto& net = nets[net_index];
        const bool is_bnet = net.kind == RoutingNetKind::Bnet;
        if (net.is_sync_bus) {
            const int group_hpwl = hpwl_of_refs(net_terminals(net));
            const int lanes = static_cast<int>(net.demands.size());
            for (const auto& demand : net.demands) {
                OwnerRecord owner {};
                owner.id = OwnerId {net.net_id, demand.demand_id};
                owner.net_index = net_index;
                owner.demand_indices.push_back(demand.demand_id);
                owner.is_bnet = is_bnet;
                owner.is_sync = true;
                owner.hpwl = hpwl_of_refs(member_terminals(net, demand));
                owner.group_hpwl = group_hpwl;
                owner.port_count = 2;
                owner.lane_count = lanes;
                owner.demand_paths.resize(1);
                owners.push_back(std::move(owner));
            }
            continue;
        }
        OwnerRecord owner {};
        owner.id = OwnerId {net.net_id, 0};
        owner.net_index = net_index;
        for (const auto& demand : net.demands) {
            owner.demand_indices.push_back(demand.demand_id);
        }
        owner.is_bnet = is_bnet;
        owner.hpwl = hpwl_of_refs(net_terminals(net));
        owner.group_hpwl = owner.hpwl;
        owner.port_count = static_cast<int>(net.sources.size() + net.demands.size());
        owner.demand_paths.resize(net.demands.size());
        owners.push_back(std::move(owner));
    }
    return owners;
}

auto initial_order(const std::Vector<OwnerRecord>& owners) -> std::Vector<std::size_t> {
    auto order = std::Vector<std::size_t> {};
    order.reserve(owners.size());
    for (std::size_t i = 0; i < owners.size(); ++i) {
        order.push_back(i);
    }
    std::sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs) {
        const auto& a = owners[lhs];
        const auto& b = owners[rhs];
        const int primary_a = a.is_sync ? a.lane_count : a.port_count;
        const int primary_b = b.is_sync ? b.lane_count : b.port_count;
        const int hpwl_a = a.is_sync ? a.group_hpwl : a.hpwl;
        const int hpwl_b = b.is_sync ? b.group_hpwl : b.hpwl;
        return rrr_initial_before(
            a.is_sync,
            primary_a,
            hpwl_a,
            a.id,
            b.is_sync,
            primary_b,
            hpwl_b,
            b.id);
    });
    return order;
}

auto find_owner_index(const std::Vector<OwnerRecord>& owners, OwnerId id) -> std::size_t {
    for (std::size_t i = 0; i < owners.size(); ++i) {
        if (owners[i].id == id) {
            return i;
        }
    }
    return owners.size();
}

auto rip_owner(OwnerRecord& owner, ResourceModel& resources) -> void {
    resources.release(owner.id);
    owner.claimed.clear();
    owner.tree.clear();
    for (auto& path : owner.demand_paths) {
        path.clear();
    }
    owner.retry += 1;
}

auto route_owner(
    OwnerRecord& owner,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    ResourceModel& resources,
    RrrParams& params,
    hardware::Interposer* interposer
) -> void {
    const auto& net = nets[owner.net_index];
    owner.claimed.clear();
    owner.tree.clear();
    for (auto& path : owner.demand_paths) {
        path.clear();
    }

    for (std::size_t i = 0; i < owner.demand_indices.size(); ++i) {
        const std::size_t demand_id = owner.demand_indices[i];
        if (demand_id >= net.demands.size()) {
            throw std::runtime_error(
                std::format("RRR: demand {} missing on net {}", demand_id, net.net_id));
        }
        const auto& demand = net.demands[demand_id];
        const auto sources = demand_source_nodes(graph, net, demand);
        const int sink = resolve_graph_node(graph, demand.sink);
        auto path = route_demand(
            graph,
            resources,
            owner.id,
            sources,
            sink,
            params,
            tree_as_vector(owner.tree),
            owner.is_bnet,
            interposer);
        if (path.empty() || path.back() != sink) {
            throw std::runtime_error(
                std::format("maze: sink {} unreachable from given sources", sink));
        }
        const auto keys = path_resource_keys(graph, path, owner.is_bnet);
        resources.claim(owner.id, keys);
        for (const auto& key : keys) {
            owner.claimed.insert(key);
        }
        for (const int node : path) {
            owner.tree.insert(node);
        }
        owner.demand_paths[i] = std::move(path);
    }
}

auto group_owner_indices(const std::Vector<OwnerRecord>& owners, std::size_t net_index)
    -> std::Vector<std::size_t> {
    auto indices = std::Vector<std::size_t> {};
    for (std::size_t i = 0; i < owners.size(); ++i) {
        if (owners[i].is_sync && owners[i].net_index == net_index) {
            indices.push_back(i);
        }
    }
    std::sort(indices.begin(), indices.end(), [&](std::size_t lhs, std::size_t rhs) {
        return owners[lhs].id < owners[rhs].id;
    });
    return indices;
}

auto sibling_hard_block(
    const std::Vector<OwnerRecord>& owners,
    const std::Vector<std::size_t>& group,
    std::size_t skip
) -> std::Set<ResourceKey> {
    auto keys = std::Set<ResourceKey> {};
    for (const std::size_t index : group) {
        if (index == skip) {
            continue;
        }
        for (const auto& key : owners[index].claimed) {
            if (is_physical_occupancy_key(key)) {
                keys.insert(key);
            }
        }
    }
    return keys;
}

auto refresh_owner_from_path(OwnerRecord& owner, const UnifiedGraph& graph) -> void {
    owner.claimed.clear();
    owner.tree.clear();
    if (owner.demand_paths.empty()) {
        return;
    }
    const auto& path = owner.demand_paths.front();
    for (const auto& key : path_resource_keys(graph, path, owner.is_bnet)) {
        owner.claimed.insert(key);
    }
    for (const int node : path) {
        owner.tree.insert(node);
    }
}

auto route_sync_group(
    std::size_t net_index,
    std::Vector<OwnerRecord>& owners,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    ResourceModel& resources,
    RrrParams& params,
    hardware::Interposer* interposer
) -> bool {
    const auto group = group_owner_indices(owners, net_index);
    if (group.empty()) {
        return true;
    }
    const auto& net = nets[net_index];
    for (const std::size_t index : group) {
        auto& owner = owners[index];
        owner.claimed.clear();
        owner.tree.clear();
        for (auto& path : owner.demand_paths) {
            path.clear();
        }
        if (owner.demand_indices.empty()) {
            throw std::runtime_error(
                std::format("RRR: SyncNet owner ({},{}) has no demand", owner.id.net_id, owner.id.demand_id));
        }
        const std::size_t demand_id = owner.demand_indices.front();
        if (demand_id >= net.demands.size()) {
            throw std::runtime_error(
                std::format("RRR: demand {} missing on net {}", demand_id, net.net_id));
        }
        const auto& demand = net.demands[demand_id];
        const auto sources = demand_source_nodes(graph, net, demand);
        const int sink = resolve_graph_node(graph, demand.sink);
        const auto blocked = sibling_hard_block(owners, group, index);
        auto path = route_demand(
            graph,
            resources,
            owner.id,
            sources,
            sink,
            params,
            {},
            owner.is_bnet,
            interposer,
            blocked);
        if (path.empty() || path.back() != sink) {
            throw std::runtime_error(
                std::format("maze: sink {} unreachable from given sources", sink));
        }
        const auto keys = path_resource_keys(graph, path, owner.is_bnet);
        resources.claim(owner.id, keys);
        for (const auto& key : keys) {
            owner.claimed.insert(key);
        }
        for (const int node : path) {
            owner.tree.insert(node);
        }
        owner.demand_paths[0] = std::move(path);
    }

    if (interposer == nullptr) {
        throw std::invalid_argument("RRR: SyncNet routing requires an Interposer");
    }

    auto lanes = std::Vector<SyncLaneState> {};
    lanes.reserve(group.size());
    for (const std::size_t index : group) {
        auto& owner = owners[index];
        const std::size_t demand_id = owner.demand_indices.front();
        const auto& demand = net.demands[demand_id];
        SyncLaneState lane {};
        lane.id = owner.id;
        lane.is_bnet = owner.is_bnet;
        lane.sources = demand_source_nodes(graph, net, demand);
        lane.sink = resolve_graph_node(graph, demand.sink);
        lane.path = owner.demand_paths[0];
        lanes.push_back(std::move(lane));
    }

    const bool equal = equalize_sync_group(graph, resources, params, interposer, lanes);
    for (std::size_t i = 0; i < group.size(); ++i) {
        owners[group[i]].demand_paths[0] = lanes[i].path;
        refresh_owner_from_path(owners[group[i]], graph);
    }
    return equal;
}

auto sync_groups_equal_length(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners,
    hardware::Interposer* interposer
) -> bool {
    if (interposer == nullptr) {
        return false;
    }
    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        if (!nets[net_index].is_sync_bus) {
            continue;
        }
        const auto group = group_owner_indices(owners, net_index);
        if (group.size() <= 1) {
            continue;
        }
        std::size_t expected = 0;
        bool have = false;
        for (const std::size_t index : group) {
            if (owners[index].demand_paths.empty() || owners[index].demand_paths.front().empty()) {
                return false;
            }
            const auto n_i = sync_lane_length(
                graph,
                owners[index].demand_paths.front(),
                interposer,
                owners[index].is_bnet);
            if (!have) {
                expected = n_i;
                have = true;
            } else if (n_i != expected) {
                return false;
            }
        }
    }
    return true;
}

auto unequal_sync_owner_ids(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners,
    hardware::Interposer* interposer
) -> std::Vector<OwnerId> {
    auto ids = std::Vector<OwnerId> {};
    if (interposer == nullptr) {
        for (const auto& owner : owners) {
            if (owner.is_sync) {
                ids.push_back(owner.id);
            }
        }
        return ids;
    }
    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        if (!nets[net_index].is_sync_bus) {
            continue;
        }
        const auto group = group_owner_indices(owners, net_index);
        if (group.size() <= 1) {
            continue;
        }
        std::size_t expected = 0;
        bool have = false;
        bool equal = true;
        for (const std::size_t index : group) {
            if (owners[index].demand_paths.empty() || owners[index].demand_paths.front().empty()) {
                equal = false;
                break;
            }
            const auto n_i = sync_lane_length(
                graph,
                owners[index].demand_paths.front(),
                interposer,
                owners[index].is_bnet);
            if (!have) {
                expected = n_i;
                have = true;
            } else if (n_i != expected) {
                equal = false;
                break;
            }
        }
        if (!equal) {
            for (const std::size_t index : group) {
                ids.push_back(owners[index].id);
            }
        }
    }
    return ids;
}

auto save_best_if_improved(
    BestSnapshot& best,
    int overflow,
    std::size_t wirelength,
    const ResourceModel& resources,
    const std::Vector<OwnerRecord>& owners
) -> bool {
    if (best.valid && !rrr_is_better(overflow, wirelength, best.overflow, best.wirelength)) {
        return false;
    }
    best.valid = true;
    best.overflow = overflow;
    best.wirelength = wirelength;
    best.resources = resources;
    best.owners = owners;
    return true;
}

auto save_legal_best(
    BestSnapshot& best,
    int overflow,
    std::size_t wirelength,
    const ResourceModel& resources,
    const std::Vector<OwnerRecord>& owners,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    hardware::Interposer* interposer
) -> bool {
    if (overflow == 0 && !sync_groups_equal_length(graph, nets, owners, interposer)) {
        return false;
    }
    return save_best_if_improved(best, overflow, wirelength, resources, owners);
}

auto routing_kind_name(const RoutingNet& net) -> std::String {
    if (net.is_sync_bus) {
        return net.kind == RoutingNetKind::Bnet ? "SyncNet-Bnet" : "SyncNet-Tnet";
    }
    switch (net.kind) {
    case RoutingNetKind::Bnet:
        return "Bnet";
    case RoutingNetKind::Tnet:
        return "Tnet";
    case RoutingNetKind::PNnet:
        return "PNnet";
    }
    return "Unknown";
}

auto format_node_ref(const UnifiedGraph& graph, const GraphNodeRef& ref) -> std::String {
    const int node = resolve_graph_node(graph, ref);
    return node >= 0 ? format_path_node(graph, node) : "<unresolved>";
}

auto format_sources(const UnifiedGraph& graph, const RoutingNet& net) -> std::String {
    auto out = std::String {};
    for (std::size_t i = 0; i < net.sources.size(); ++i) {
        if (i != 0) {
            out += ", ";
        }
        out += format_node_ref(graph, net.sources[i]);
    }
    return out;
}

auto format_node_path(const UnifiedGraph& graph, const std::Vector<int>& path) -> std::String {
    auto out = std::String {};
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i != 0) {
            out += " -> ";
        }
        out += format_path_node(graph, path[i]);
    }
    return out;
}

auto dump_paths(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners,
    hardware::Interposer* interposer
) -> void {
    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        const auto& net = nets[net_index];
        auto paths_by_demand = std::Vector<const std::Vector<int>*> {};
        paths_by_demand.resize(net.demands.size(), nullptr);
        auto net_paths = std::Vector<std::Vector<int>> {};

        for (const auto& owner : owners) {
            if (owner.net_index != net_index) {
                continue;
            }
            for (std::size_t i = 0; i < owner.demand_paths.size(); ++i) {
                const std::size_t demand_id =
                    i < owner.demand_indices.size() ? owner.demand_indices[i] : i;
                const auto& path = owner.demand_paths[i];
                if (demand_id >= paths_by_demand.size() || path.empty()) {
                    continue;
                }
                paths_by_demand[demand_id] = &path;
                net_paths.push_back(path);
            }
        }

        debug::info_fmt(
            "FPIA RRR: route net_id={} name={} kind={} demands={} wirelength={} sources=[{}]",
            net.net_id,
            net.name,
            routing_kind_name(net),
            net.demands.size(),
            net_wirelength(graph, net_paths),
            format_sources(graph, net));
        for (std::size_t demand_id = 0; demand_id < paths_by_demand.size(); ++demand_id) {
            const auto* path = paths_by_demand[demand_id];
            const int sink = resolve_graph_node(graph, net.demands[demand_id].sink);
            const auto sink_text = sink >= 0 ? format_path_node(graph, sink) : "<unresolved>";
            if (path == nullptr) {
                debug::info_fmt("  demand={} sink={} path=<missing>", demand_id, sink_text);
                continue;
            }
            if (net.is_sync_bus) {
                const auto n_i = interposer == nullptr
                    ? 0
                    : sync_lane_length(graph, *path, interposer, net.kind == RoutingNetKind::Bnet);
                debug::info_fmt(
                    "  lane={} source={} sink={} N_i={} path=[{}]",
                    demand_id,
                    format_path_node(graph, path->front()),
                    sink_text,
                    n_i,
                    format_node_path(graph, *path));
                continue;
            }
            debug::info_fmt(
                "  demand={} start={} sink={} path=[{}]",
                demand_id,
                format_path_node(graph, path->front()),
                sink_text,
                format_node_path(graph, *path));
        }
    }
}

auto format_overflow_owner(const OwnerId& id, const std::Vector<RoutingNet>& nets) -> std::String {
    for (const auto& net : nets) {
        if (net.net_id != id.net_id) {
            continue;
        }
        if (net.is_sync_bus) {
            return std::format("{}#{}", net.name, id.demand_id);
        }
        return net.name;
    }
    return std::format("net{}#{}", id.net_id, id.demand_id);
}

auto dump_overflows(
    const std::Vector<OwnerRecord>& owners,
    const std::Vector<RoutingNet>& nets,
    const ResourceModel& resources
) -> void {
    auto seen = std::Set<ResourceKey> {};
    for (const auto& owner : owners) {
        for (const auto& key : owner.claimed) {
            auto report = [&](const ResourceKey& overflow_key) {
                if (resources.overflow(overflow_key) <= 0 || !seen.insert(overflow_key).second) {
                    return;
                }
                auto names = std::String {};
                for (const auto& item : resources.owners_of(overflow_key)) {
                    if (!names.empty()) {
                        names += ",";
                    }
                    names += format_overflow_owner(item, nets);
                }
                debug::debug_fmt("FPIA RRR: overflow owners=[{}]", names);
            };
            report(key);
            if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
                report(mode_conflict_key(key.id));
            }
        }
    }
}

auto all_demands_connected(
    const std::Vector<RoutingNet>& nets,
    const std::Vector<OwnerRecord>& owners
) -> bool {
    auto seen = std::Vector<std::size_t> {};
    seen.resize(nets.size(), 0);
    for (const auto& owner : owners) {
        if (owner.net_index >= nets.size()) {
            return false;
        }
        for (const auto& path : owner.demand_paths) {
            if (path.empty()) {
                return false;
            }
            seen[owner.net_index] += 1;
        }
    }
    for (std::size_t i = 0; i < nets.size(); ++i) {
        if (seen[i] != nets[i].demands.size()) {
            return false;
        }
    }
    return true;
}

} // namespace

auto run_rrr(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const RrrParams& input_params,
    hardware::Interposer* interposer,
    int verbose_level
) -> RrrResult {
    if (interposer == nullptr) {
        throw std::invalid_argument("RRR: run_rrr requires a non-null Interposer");
    }
    auto params = input_params;
    auto resources = ResourceModel {params};
    auto owners = build_owners(nets);
    const int route_owners = static_cast<int>(owners.size());

    debug::info_fmt(
        "FPIA RRR: graph nodes={} arcs={} route_owners={}",
        graph.nodes.size(),
        graph.arcs.size(),
        route_owners);
    debug::info_fmt(
        "FPIA RRR: params max_iterations={} seed={} H={} k={} s={} sync_tail_extra_tracks={}",
        params.max_iterations,
        params.seed,
        params.H,
        params.k,
        params.s,
        params.sync_tail_extra_tracks);

    const auto routing_start = std::chrono::steady_clock::now();
    auto result = RrrResult {};
    auto best = BestSnapshot {};
    int stagnant = 0;
    int iterations = 0;
    auto status = std::String {"iteration_limit"};

    auto finish = [&](const std::String& final_status) {
        const auto routing_end = std::chrono::steady_clock::now();
        const auto routing_ms = static_cast<std::i64>(
            std::chrono::duration_cast<std::chrono::milliseconds>(routing_end - routing_start).count());
        if (best.valid) {
            resources = best.resources;
            owners = best.owners;
        }
        const auto paths = collect_paths_by_net(nets, owners);
        result.status = final_status;
        result.iterations = iterations;
        result.best_overflow = best.valid ? best.overflow : resources.overflow();
        result.total_wirelength = best.valid ? best.wirelength : total_wirelength(graph, paths);
        result.paths = paths;
        result.routing_ms = routing_ms;
        if (final_status == "success"
            && (result.best_overflow != 0 || !all_demands_connected(nets, owners)
                || !sync_groups_equal_length(graph, nets, owners, interposer))) {
            result.status = "unroutable";
        }
        const auto elapsed_ms = Elapsed::milliseconds();
        debug::info_fmt(
            "FPIA RRR: status={} iterations={} best_overflow={} routing_ms={} elapsed_ms={}",
            result.status,
            result.iterations,
            result.best_overflow,
            result.routing_ms,
            elapsed_ms);
        debug::info_fmt(
            "routing result: total_wirelength={} RRR_routing_time={} elapsed_ms={}",
            result.total_wirelength,
            result.routing_ms,
            elapsed_ms);
        dump_paths(graph, nets, owners, interposer);
        if (verbose_level >= 1) {
            dump_overflows(owners, nets, resources);
        }
        return result;
    };

    try {
        auto routed_sync = std::Set<std::size_t> {};
        for (const std::size_t index : initial_order(owners)) {
            if (owners[index].is_sync) {
                if (!routed_sync.insert(owners[index].net_index).second) {
                    continue;
                }
                route_sync_group(
                    owners[index].net_index, owners, graph, nets, resources, params, interposer);
                continue;
            }
            route_owner(owners[index], graph, nets, resources, params, interposer);
        }
    }
    catch (const std::runtime_error&) {
        return finish("unroutable");
    }

    {
        const int initial_overflow = resources.overflow();
        const auto initial_wl = current_wirelength(graph, nets, owners);
        debug::info_fmt(
            "FPIA RRR: initial overflow={} total_wirelength={}",
            initial_overflow,
            initial_wl);
    }

    for (int iter = 0; iter < params.max_iterations; ++iter) {
        const int overflow = resources.overflow();
        const auto wirelength = current_wirelength(graph, nets, owners);
        const int max_ov = max_resource_overflow(owners, resources);
        save_legal_best(best, overflow, wirelength, resources, owners, graph, nets, interposer);

        const bool sync_equal = sync_groups_equal_length(graph, nets, owners, interposer);
        if (overflow == 0 && sync_equal) {
            iterations = iter;
            debug::info_fmt(
                "FPIA RRR: iter={} overflow={} max_resource_overflow={} dirty_owners={} rerouted={} total_wirelength={}",
                iter,
                overflow,
                max_ov,
                0,
                0,
                wirelength);
            status = "success";
            break;
        }

        resources.history_next();
        if (stagnant > 0 && stagnant % 4 == 0) {
            params.H = std::min(params.H + 4, 16);
        }

        auto dirty_ids = std::Vector<OwnerId> {};
        auto dirty_set = std::Set<OwnerId> {};
        for (const auto& owner : owners) {
            if (owner_is_dirty(owner, resources) && dirty_set.insert(owner.id).second) {
                dirty_ids.push_back(owner.id);
            }
        }
        auto dirty_sync_nets = std::Set<std::size_t> {};
        for (const auto id : dirty_ids) {
            const std::size_t index = find_owner_index(owners, id);
            if (index < owners.size() && owners[index].is_sync) {
                dirty_sync_nets.insert(owners[index].net_index);
            }
        }
        for (const auto& owner : owners) {
            if (owner.is_sync && dirty_sync_nets.contains(owner.net_index)
                && dirty_set.insert(owner.id).second) {
                dirty_ids.push_back(owner.id);
            }
        }
        if (!sync_equal) {
            for (const auto id : unequal_sync_owner_ids(graph, nets, owners, interposer)) {
                if (dirty_set.insert(id).second) {
                    dirty_ids.push_back(id);
                }
            }
        }

        if (dirty_ids.empty()) {
            iterations = iter;
            status = sync_equal && overflow == 0 ? "success" : "stagnated";
            debug::info_fmt(
                "FPIA RRR: iter={} overflow={} max_resource_overflow={} dirty_owners={} rerouted={} total_wirelength={}",
                iter,
                overflow,
                max_ov,
                0,
                0,
                wirelength);
            break;
        }

        std::sort(dirty_ids.begin(), dirty_ids.end(), [&](OwnerId lhs, OwnerId rhs) {
            const auto& a = owners[find_owner_index(owners, lhs)];
            const auto& b = owners[find_owner_index(owners, rhs)];
            return rrr_dirty_before(
                owner_exposure(a, resources),
                a.retry,
                a.hpwl,
                a.id,
                owner_exposure(b, resources),
                b.retry,
                b.hpwl,
                b.id);
        });

        const int dirty_count = static_cast<int>(dirty_ids.size());
        for (const auto id : dirty_ids) {
            const std::size_t index = find_owner_index(owners, id);
            if (index < owners.size()) {
                rip_owner(owners[index], resources);
            }
        }

        int rerouted = 0;
        try {
            auto routed_sync = std::Set<std::size_t> {};
            for (const auto id : dirty_ids) {
                const std::size_t index = find_owner_index(owners, id);
                if (index >= owners.size()) {
                    continue;
                }
                if (owners[index].is_sync) {
                    if (!routed_sync.insert(owners[index].net_index).second) {
                        continue;
                    }
                    route_sync_group(
                        owners[index].net_index, owners, graph, nets, resources, params, interposer);
                    rerouted += static_cast<int>(group_owner_indices(owners, owners[index].net_index).size());
                    continue;
                }
                route_owner(owners[index], graph, nets, resources, params, interposer);
                ++rerouted;
            }
        }
        catch (const std::runtime_error&) {
            iterations = iter + 1;
            return finish("unroutable");
        }

        const int new_overflow = resources.overflow();
        const auto new_wirelength = current_wirelength(graph, nets, owners);
        const bool improved = save_legal_best(
            best,
            new_overflow,
            new_wirelength,
            resources,
            owners,
            graph,
            nets,
            interposer);
        if (improved) {
            stagnant = 0;
        } else {
            ++stagnant;
        }

        debug::info_fmt(
            "FPIA RRR: iter={} overflow={} max_resource_overflow={} dirty_owners={} rerouted={} total_wirelength={}",
            iter,
            overflow,
            max_ov,
            dirty_count,
            rerouted,
            new_wirelength);
        if (verbose_level >= 1) {
            dump_overflows(owners, nets, resources);
        }

        iterations = iter + 1;
        if (new_overflow == 0 && sync_groups_equal_length(graph, nets, owners, interposer)) {
            status = "success";
            break;
        }
        if (stagnant >= params.stagnation_limit) {
            status = "stagnated";
            break;
        }
        status = "iteration_limit";
    }

    if (status != "success") {
        const int overflow = resources.overflow();
        const auto wirelength = current_wirelength(graph, nets, owners);
        save_legal_best(best, overflow, wirelength, resources, owners, graph, nets, interposer);
        if (best.valid && best.overflow == 0) {
            status = "success";
        }
    }

    return finish(status);
}

} // namespace PR_tool
