#pragma once

#include "air_search.hh"
#include "route_log.hh"
#include "sync_equalize.hh"

#include <algo/router/command_mode/command/sort.hh>
#include <algo/router/command_mode/command/resources.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>

namespace PR_tool::rrr_detail {

struct AirNetState {
    AirSpatial spatial;
    std::size_t reference{std::numeric_limits<std::size_t>::max()};
};

inline auto air_order(const std::Vector<RoutingNet>& nets, hardware::Interposer* interposer,
                      const std::Vector<std::Rc<circuit::Net>>* raw) -> std::Vector<std::size_t> {
    auto result = std::Vector<std::size_t>{};
    if (raw != nullptr) {
        if (raw->size() != nets.size()) throw std::invalid_argument("AIR raw/routing net count mismatch");
        const auto maze = algo::MazeRouteStrategy{};
        const auto allocator = algo::HK{};
        auto engine = algo::RouteEngine{{{0, *raw}}, maze, allocator, 0, false, false, false, interposer};
        algo::Sort{}.execute(interposer, engine);
        algo::Resources{}.execute(interposer, engine);
        for (auto* n : engine.nets()) {
            const auto it = std::find_if(raw->begin(), raw->end(), [&](const auto& value) { return value.get() == n; });
            result.push_back(it - raw->begin());
        }
    } else {
        const auto owners = build_owners(nets);
        for (auto index : initial_type_order(owners, nets)) {
            const auto ni = owners[index].net_index;
            if (std::find(result.begin(), result.end(), ni) == result.end()) result.push_back(ni);
        }
    }
    for (auto ni : result) debug::info_fmt("FPIA RRR: initial order net_id={} name={}", nets[ni].net_id, nets[ni].name);
    return result;
}

// demand_paths may temporarily end at a pruned, root-connected branch point.
// A nonempty retained prefix is NOT a routed sink.
inline auto air_connected(const UnifiedGraph& graph, const RoutingNet& net,
                          const OwnerRecord& owner, std::size_t slot) -> bool {
    const auto& path = owner.demand_paths[slot];
    if (path.empty() || path.back() != resolve_graph_node(graph, net.demands[owner.demand_indices[slot]].sink)) return false;
    const auto roots = demand_source_nodes(graph, net, net.demands[owner.demand_indices[slot]]);
    return std::find(roots.begin(), roots.end(), path.front()) != roots.end();
}

inline auto air_missing(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                        const std::Vector<OwnerRecord>& owners) -> int {
    int count = 0;
    for (const auto& owner : owners) {
        bool connected = !owner.demand_paths.empty();
        for (std::size_t slot = 0; slot < owner.demand_paths.size(); ++slot)
            connected = connected && air_connected(graph, nets[owner.net_index], owner, slot);
        if (!connected) ++count;
    }
    return count;
}

inline auto air_refresh(const UnifiedGraph& graph, OwnerRecord& owner, ResourceModel& resources,
                        AirSpatial* spatial = nullptr) -> void {
    resources.release(owner.id); owner.claimed.clear(); owner.tree.clear();
    for (const auto& path : owner.demand_paths) {
        for (const auto& key : path_resource_keys(graph, path, owner.is_bnet)) owner.claimed.insert(key);
        for (int node : path) add_tree_node(owner, graph, node);
    }
    resources.claim(owner.id, std::Vector<ResourceKey>(owner.claimed.begin(), owner.claimed.end()));
    if (spatial != nullptr) {
        spatial->bins.clear();
        for (int n : owner.tree) {
            const auto [r, c] = air_position(graph, n);
            spatial->bins[spatial->bin(r, c)].push_back(n);
        }
    }
}

inline auto air_route_owner(const UnifiedGraph& graph, const RoutingNet& net, OwnerRecord& owner,
                            ResourceModel& resources, AirSpatial& spatial) -> void {
    air_refresh(graph, owner, resources, &spatial);
    for (std::size_t slot = 0; slot < owner.demand_paths.size(); ++slot) {
        if (air_connected(graph, net, owner, slot)) continue;
        const auto& demand = net.demands[owner.demand_indices[slot]];
        const auto roots = demand_source_nodes(graph, net, demand);
        const int sink = resolve_graph_node(graph, demand.sink);
        if (sink < 0) continue;
        auto path = air_root_prefix(owner, roots, sink);
        if (!path.empty()) {
            auto candidate = owner; candidate.demand_paths[slot] = path;
            if (!owner_paths_valid(graph, candidate)) path.clear();
        }
        auto full = std::Vector<int>(owner.tree.begin(), owner.tree.end());
        for (int root : roots)
            if (std::find(full.begin(), full.end(), root) == full.end()) full.push_back(root);
        if (path.empty() && net.demands.size() >= 16) {
            const auto [r, c] = air_position(graph, sink);
            const auto it = spatial.bins.find(spatial.bin(r, c));
            if (it != spatial.bins.end() && !it->second.empty())
                path = air_connection(graph, net, owner, slot, resources, spatial, it->second);
        }
        if (path.empty()) path = air_connection(graph, net, owner, slot, resources, spatial, full);
        if (path.empty()) continue;
        owner.demand_paths[slot] = std::move(path);
        air_refresh(graph, owner, resources, &spatial);
    }
}

inline auto air_rip_net(std::size_t ni, std::Vector<OwnerRecord>& owners, ResourceModel& resources) -> void {
    for (auto& owner : owners) if (owner.net_index == ni) {
        rip_owner(owner, resources);
        if (owner.is_sync) owner.sync_target = 0;
    }
}

inline auto air_route_sync(std::size_t ni, const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                           std::Vector<OwnerRecord>& owners, ResourceModel& resources,
                           RrrParams params, hardware::Interposer* interposer) -> bool {
    air_rip_net(ni, owners, resources);
    // Original sync policy: pre-route all lanes, use their maximum as the
    // common target, then apply the existing whole-group tail equalizer.
    bool valid = false;
    try {
        valid = route_sync_group(ni, owners, graph, nets, resources, params, interposer);
        for (auto index : group_owner_indices(owners, ni)) valid = valid && owner_paths_valid(graph, owners[index]);
    } catch (const std::runtime_error& error) {
        if (!std::string_view(error.what()).starts_with("maze:")) throw;
    }
    if (!valid) air_rip_net(ni, owners, resources);
    return valid;
}

inline auto air_net_complete(std::size_t ni, const UnifiedGraph& graph,
                             const std::Vector<RoutingNet>& nets, const std::Vector<OwnerRecord>& owners) -> bool {
    for (const auto& owner : owners) if (owner.net_index == ni)
        for (std::size_t slot = 0; slot < owner.demand_paths.size(); ++slot)
            if (!air_connected(graph, nets[ni], owner, slot)) return false;
    return true;
}

inline auto air_reference(std::size_t ni, const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                          AirNetState& state, RrrParams params, hardware::Interposer* interposer) -> void {
    auto owners = build_owners(nets);
    auto empty = ResourceModel{params};
    if (nets[ni].is_sync_bus) air_route_sync(ni, graph, nets, owners, empty, params, interposer);
    else for (auto& owner : owners) if (owner.net_index == ni)
        air_route_owner(graph, nets[ni], owner, empty, state.spatial);
    if (air_net_complete(ni, graph, nets, owners)) {
        const auto paths = collect_paths_by_net(nets, owners);
        state.reference = net_wirelength(graph, paths[ni]);
        debug::info_fmt("FPIA RRR: reference net_id={} wirelength={} (uncongested maze, not a lower bound)",
            nets[ni].net_id, state.reference);
    } else debug::info_fmt("FPIA RRR: reference unavailable net_id={}", nets[ni].net_id);
    state.spatial.bins.clear(); // Reference paths are never the actual warm start.
}

inline auto air_key_conflicted(const ResourceModel& snapshot, const ResourceKey& key) -> bool {
    return snapshot.overflow(key) > 0 ||
        ((key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap)
         && snapshot.overflow(mode_conflict_key(key.id)) > 0);
}

inline auto air_path_cut(const UnifiedGraph& graph, const ResourceModel& snapshot,
                         const std::Vector<int>& path) -> std::size_t {
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (air_key_conflicted(snapshot, node_resource(path[i]))) return i;
        if (i == 0) continue;
        for (int aid : graph.out_arc_ids[path[i-1]]) if (graph.arcs[aid].v == path[i])
            for (const auto& key : arc_resource_keys(graph, graph.arcs[aid]))
                if (air_key_conflicted(snapshot, key)) return i;
    }
    return path.size();
}

inline auto air_prune_illegal(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                              std::Vector<OwnerRecord>& owners, ResourceModel& resources,
                              std::Vector<AirNetState>& states, hardware::Interposer* interposer) -> int {
    // All claimants see the SAME congestion snapshot, even after earlier nets
    // release their branches. Keep only root-connected prefixes, never islands.
    const auto snapshot = resources;
    const auto unequal = unequal_sync_owner_ids(graph, nets, owners, interposer);
    int pruned = 0;
    for (std::size_t ni = 0; ni < nets.size(); ++ni) {
        bool conflict = false;
        for (const auto& owner : owners) if (owner.net_index == ni) {
            conflict = conflict || !owner_paths_valid(graph, owner);
            for (const auto& key : owner.claimed) conflict = conflict || air_key_conflicted(snapshot, key);
            if (owner.is_sync && std::find(unequal.begin(), unequal.end(), owner.id) != unequal.end()) conflict = true;
        }
        if (!conflict) continue;
        ++pruned;
        if (nets[ni].is_sync_bus) { air_rip_net(ni, owners, resources); continue; }
        for (auto& owner : owners) if (owner.net_index == ni) {
            if (owner.port_count < 16 || !owner_paths_valid(graph, owner)) rip_owner(owner, resources);
            else {
                for (auto& path : owner.demand_paths) path.resize(air_path_cut(graph, snapshot, path));
                air_refresh(graph, owner, resources, &states[ni].spatial);
            }
        }
    }
    return pruned;
}

inline auto air_wirelength_candidates(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                                      const std::Vector<OwnerRecord>& owners,
                                      const std::Vector<AirNetState>& states) -> std::Vector<std::size_t> {
    const auto paths = collect_paths_by_net(nets, owners);
    auto selected = std::Vector<std::size_t>{};
    for (std::size_t ni = 0; ni < nets.size(); ++ni)
        if (net_wirelength(graph, paths[ni]) > states[ni].reference) selected.push_back(ni);
    if (selected.empty())
        for (std::size_t ni = 0; ni < nets.size(); ++ni)
            if (nets[ni].demands.size() > 1 || nets[ni].sources.size() > 1) selected.push_back(ni);
    if (selected.empty())
        for (std::size_t ni = 0; ni < nets.size(); ++ni) selected.push_back(ni);
    return selected;
}

} // namespace PR_tool::rrr_detail
