#pragma once

#include "rrr_routing.hh"
#include "net_adapter.hh"
#include "route_validate.hh"
#include "maze_search.hh"
#include "sync_equalize.hh"

#include <algo/router/command_mode/command/sort.hh>
#include <algo/router/command_mode/command/resources.hh>
#include <algo/router/command_mode/command/route.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <algo/router/routeerror.hh>
#include <circuit/net/types/syncnet.hh>
#include <debug/debug.hh>
#include <queue>

namespace PR_tool::rrr_detail {

inline auto clear_initial_reservations(hardware::Interposer* interposer) -> void {
    interposer->reset_regs();
    // source COBUnit::reset_regs currently omits LU (and repeats LD).
    // Also clear unrecorded LU reservations left by an interrupted fanout.
    for (const auto& [coord, cob] : interposer->cobs()) {
        (void)coord;
        for (std::size_t i = 0; i < hardware::COB::INDEX_SIZE; ++i)
            cob->sw_register(hardware::COBDirection::Left, i, hardware::COBDirection::Up)->reset();
    }
}

inline auto raw_track_node(const UnifiedGraph& graph, hardware::Track* track) -> int {
    GraphNodeRef ref{};
    ref.kind = GraphNodeRef::Kind::Track;
    ref.track_coord = track->coord();
    ref.track_index = track->coord().index;
    const int node = resolve_graph_node(graph, ref);
    if (node < 0) throw std::logic_error("initial maze Track is absent from graph");
    return node;
}

inline auto import_package(OwnerRecord& owner, const circuit::PathPackage& package,
                           const UnifiedGraph& graph, const RoutingNet& net,
                           hardware::Interposer* interposer) -> void {
    auto adjacent = std::map<int, std::set<int>>{};
    const auto edge = [&](int u, int v) {
        if (!graph.directed_arc_set.contains({u,v}) || !graph.directed_arc_set.contains({v,u}))
            throw std::logic_error("initial maze connection is absent from graph");
        adjacent[u].insert(v); adjacent[v].insert(u);
    };
    for (const auto& [track, connector] : package._regular_path) {
        (void)track;
        if (!connector.has_value()) continue;
        auto* cob = interposer->get_cob(connector->coord()).value();
        auto* from = interposer->get_track(cob->to_dir_track_coord(
            connector->from_dir(), connector->from_track_index())).value();
        auto* to = interposer->get_track(cob->to_dir_track_coord(
            connector->to_dir(), connector->to_track_index())).value();
        edge(raw_track_node(graph, from), raw_track_node(graph, to));
    }
    const auto access = [&](const auto& entries) {
        for (const auto& [bump, connector, track] : entries) {
            const auto bc = bump_to_routing_coord(bump);
            const auto h = connector.hori_index();
            const int b = graph.bump_node_by_key.at(bc);
            const int hn = graph.hline_node_by_key.at({bc.TOB, h/64, (h%64)/8, h%8});
            const int vn = graph.vline_node_by_key.at({bc.TOB, connector.vert_index()});
            edge(b, hn); edge(hn, vn); edge(vn, raw_track_node(graph, track));
        }
    };
    access(package._tob_to_track); access(package._track_to_tob);
    for (std::size_t i = 0; i < owner.demand_indices.size(); ++i) {
        const auto& demand = net.demands[owner.demand_indices[i]];
        const int sink = resolve_graph_node(graph, demand.sink);
        auto previous = std::map<int, int>{};
        auto queue = std::queue<int>{};
        for (const auto source : demand_source_nodes(graph, net, demand)) {
            previous.emplace(source, -1); queue.push(source);
        }
        while (!queue.empty() && !previous.contains(sink)) {
            const int u = queue.front(); queue.pop();
            for (int v : adjacent[u])
                if (previous.emplace(v, u).second) queue.push(v);
        }
        if (!previous.contains(sink)) throw algo::RetryExpt("initial maze package misses demand");
        auto path = std::Vector<int>{};
        for (int n = sink; n >= 0; n = previous.at(n)) path.push_back(n);
        std::reverse(path.begin(), path.end());
        owner.demand_paths[i] = std::move(path);
    }
    owner.claimed.clear(); owner.tree.clear();
    for (const auto& path : owner.demand_paths) {
        const auto keys = path_resource_keys(graph, path, owner.is_bnet);
        owner.claimed.insert(keys.begin(), keys.end());
        for (int n : path) add_tree_node(owner, graph, n);
    }
}

inline auto original_initial_route(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                                   std::Vector<OwnerRecord>& owners, ResourceModel& resources,
                                   hardware::Interposer* interposer,
                                   const std::Vector<std::Rc<circuit::Net>>& raw_nets)
    -> std::Vector<std::size_t> {
    struct Cleanup {
        hardware::Interposer* hardware;
        const std::Vector<std::Rc<circuit::Net>>& nets;
        ~Cleanup() { for (const auto& net : nets) net->clear_current_package(); clear_initial_reservations(hardware); }
    } cleanup{interposer, raw_nets};
    const auto strategy = algo::MazeRouteStrategy{};
    const auto allocator = algo::HK{};
    const auto by_mode = std::HashMap<int, std::Vector<std::Rc<circuit::Net>>>{{0, raw_nets}};
    auto engine = algo::RouteEngine{by_mode, strategy, allocator, 0, false, false, false, interposer};
    algo::Sort{}.execute(interposer, engine);
    algo::Resources{}.execute(interposer, engine);
    auto order = std::Vector<std::size_t>{};
    for (auto* raw : engine.nets()) {
        const auto it = std::find_if(raw_nets.begin(), raw_nets.end(), [&](const auto& n) { return n.get() == raw; });
        const auto ni = static_cast<std::size_t>(it - raw_nets.begin());
        order.push_back(ni);
        debug::info_fmt("FPIA RRR: initial order net_id={} name={}", nets[ni].net_id, raw->name());
    }
    try {
        algo::Route{}.execute(interposer, engine);
    } catch (const algo::RetryExpt& error) {
        const auto position = engine.position();
        const auto ni = order[position];
        debug::info_fmt("FPIA RRR: initial routing interrupted position={} net_id={} name={} reason={}",
            position, nets[ni].net_id, raw_nets[ni]->name(), error.what());
    }
    // The original Route stops at the first failure. Import only its completed
    // prefix; never import a failed net's partial branches or synchronous lanes.
    for (std::size_t position = 0; position < engine.position(); ++position) {
        const auto ni = order[position];
        auto* raw = raw_nets[ni].get();
        const auto saved_owners = owners;
        const auto saved_resources = resources;
        try {
            if (auto* sync = dynamic_cast<circuit::SyncNet*>(raw)) {
                std::size_t lane = 0;
                const auto import = [&](auto& members) {
                    for (auto& member : members) {
                        auto& owner = owners[find_owner_index(owners, {nets[ni].net_id, lane++})];
                        import_package(owner, member->pathpackage(), graph, nets[ni], interposer);
                        resources.claim(owner.id, std::Vector<ResourceKey>(owner.claimed.begin(), owner.claimed.end()));
                        owner.sync_target = sync_lane_length(graph, owner.demand_paths[0],
                            interposer, owner.is_bnet);
                    }
                };
                import(sync->btbnets()); import(sync->bttnets()); import(sync->ttbnets());
                debug::info_fmt("FPIA RRR: initial sync routed net_id={} target={}",
                    nets[ni].net_id, owners[find_owner_index(owners, {nets[ni].net_id, 0})].sync_target);
            } else {
                const auto index = find_owner_index(owners, {nets[ni].net_id, 0});
                import_package(owners[index], raw->pathpackage(), graph, nets[ni], interposer);
                resources.claim(owners[index].id,
                    std::Vector<ResourceKey>(owners[index].claimed.begin(), owners[index].claimed.end()));
            }
            auto candidate = RrrResult{}; candidate.paths = collect_paths_by_net(nets, owners);
            if (resources.overflow() != 0 || !validate_rrr_solution(graph, nets, candidate, interposer, true))
                throw algo::RetryExpt("initial maze candidate failed physical validation", raw);
        } catch (const algo::RetryExpt& error) {
            owners = saved_owners; resources = saved_resources;
            debug::info_fmt("FPIA RRR: initial routing interrupted position={} net_id={} name={} reason={}",
                position, nets[ni].net_id, raw->name(), error.what());
            break;
        }
    }
    return order;
}

} // namespace PR_tool::rrr_detail
