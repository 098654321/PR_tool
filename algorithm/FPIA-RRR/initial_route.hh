#pragma once

#include "rrr_routing.hh"
#include "net_adapter.hh"
#include "route_validate.hh"
#include "maze_search.hh"
#include "hw_map.hh"
#include "initial_sync_route.hh"
#include "sync_equalize.hh"

#include <algo/router/command_mode/command/sort.hh>
#include <algo/router/command_mode/command/resources.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <algo/router/routeerror.hh>
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

inline auto hardware_track(const UnifiedGraph& graph, int n,
                           hardware::Interposer* interposer) -> hardware::Track* {
    const auto& node = graph.nodes[n];
    return interposer->get_track(node.track_row, node.track_col,
        node.track_dir == 0 ? hardware::TrackDirection::Horizontal : hardware::TrackDirection::Vertical,
        node.track_index).value();
}

// Only the original maze warm start reserves hardware. RRR itself never does.
inline auto export_sync_package(const UnifiedGraph& graph, std::Vector<int> path,
                                hardware::Interposer* interposer, bool reverse)
    -> circuit::PathPackage {
    if (reverse) std::reverse(path.begin(), path.end());
    auto package = circuit::PathPackage{};
    hardware::Track* previous = nullptr;
    for (int n : path) {
        if (graph.nodes[n].kind != UnifiedNodeKind::Track) continue;
        auto* track = hardware_track(graph, n, interposer);
        auto connector = std::Option<hardware::COBConnector>{};
        if (previous != nullptr) {
            for (const auto& [to, c] : interposer->adjacent_tracks(previous))
                if (to == track) { connector = c; break; }
            if (!connector.has_value()) throw std::logic_error("sync export missing COB connector");
        }
        package._regular_path.emplace_back(track, connector);
        previous = track;
    }
    const auto access = [&](bool head) {
        auto nodes = path;
        if (!head) std::reverse(nodes.begin(), nodes.end());
        if (graph.nodes[nodes.front()].kind != UnifiedNodeKind::Bump) return;
        if (nodes.size() < 4) throw std::logic_error("sync export incomplete TOB access");
        const auto bc = graph.nodes[nodes[0]].bump;
        const auto& hn = graph.nodes[nodes[1]];
        const auto& vn = graph.nodes[nodes[2]];
        auto* bump = interposer->get_bump(bc.TOB / hardware::Interposer::TOB_ARRAY_WIDTH,
            bc.TOB % hardware::Interposer::TOB_ARRAY_WIDTH, bc.Bank*64 + bc.Group*8 + bc.Index).value();
        const auto direction = head ? hardware::TOBSignalDirection::BumpToTrack
                                    : hardware::TOBSignalDirection::TrackToBump;
        for (const auto& c : bump->tob()->available_connectors(bump->index(), direction)) {
            if (c.hori_index() == hn.bank*64 + hn.group*8 + hn.line_index &&
                c.vert_index() == vn.line_index && c.track_index() == graph.nodes[nodes[3]].track_index) {
                auto& entries = head ? package._tob_to_track : package._track_to_tob;
                entries.emplace_back(bump, c, hardware_track(graph, nodes[3], interposer));
                return;
            }
        }
        throw std::runtime_error("sync export TOB access is not available");
    };
    access(true); access(false);
    package._length = package._regular_path.size() + package._tob_to_track.size() + package._track_to_tob.size();
    return package;
}

inline auto original_initial_route(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                                   std::Vector<OwnerRecord>& owners, ResourceModel& resources,
                                   RrrParams params, hardware::Interposer* interposer,
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
    params.hard_capacity = true;
    for (std::size_t position = 0; position < order.size(); ++position) {
        const auto ni = order[position];
        auto* raw = raw_nets[ni].get();
        const auto saved_owners = owners;
        const auto saved_resources = resources;
        try {
            if (auto* sync = dynamic_cast<circuit::SyncNet*>(raw)) {
                original_sync_preroute(interposer, sync);
                std::size_t lane = 0;
                bool at_target = true;
                const auto import = [&](auto& members) {
                    for (auto& member : members) {
                        auto& owner = owners[find_owner_index(owners, {nets[ni].net_id, lane++})];
                        import_package(owner, member->pathpackage(), graph, nets[ni], interposer);
                        resources.claim(owner.id, std::Vector<ResourceKey>(owner.claimed.begin(), owner.claimed.end()));
                        at_target = at_target && sync_lane_length(graph, owner.demand_paths[0],
                            interposer, owner.is_bnet) == owner.sync_target;
                    }
                };
                import(sync->btbnets()); import(sync->bttnets()); import(sync->ttbnets());
                debug::info_fmt("FPIA RRR: initial sync preroute net_id={} at_target={}", nets[ni].net_id, at_target);
                if (at_target) {
                    sync->collect_package();
                } else {
                    // Retain the original maze paths as the equalizer's snapshot;
                    // release only this group's hardware reservations before export.
                    sync->clear_current_package();
                    try {
                        if (!equalize_routed_sync_group(ni, owners, graph, nets, resources, params, interposer))
                            throw algo::RetryExpt("sync initial equalization failed", raw);
                    } catch (const std::runtime_error& error) {
                        if (!std::string_view(error.what()).starts_with("maze:")) throw;
                        throw algo::RetryExpt(error.what(), raw);
                    }
                    lane = 0;
                    const auto install = [&](auto& members, bool reverse) {
                        for (auto& member : members) {
                            const auto index = find_owner_index(owners, {nets[ni].net_id, lane++});
                            auto package = export_sync_package(graph, owners[index].demand_paths[0], interposer, reverse);
                            member->set_pathpackage(package); package.occupy_all();
                        }
                    };
                    install(sync->btbnets(), false); install(sync->bttnets(), true); install(sync->ttbnets(), false);
                    sync->collect_package();
                }
            } else {
                raw->set_reuse_type(false);
                auto routed = engine.routed_nets(); raw->search_related_nets(routed);
                raw->route(interposer, strategy);
                const auto index = find_owner_index(owners, {nets[ni].net_id, 0});
                import_package(owners[index], raw->pathpackage(), graph, nets[ni], interposer);
                resources.claim(owners[index].id,
                    std::Vector<ResourceKey>(owners[index].claimed.begin(), owners[index].claimed.end()));
            }
            auto candidate = RrrResult{}; candidate.paths = collect_paths_by_net(nets, owners);
            if (resources.overflow() != 0 || !validate_rrr_solution(graph, nets, candidate, interposer, true))
                throw algo::RetryExpt("initial maze candidate failed physical validation", raw);
            engine.move_on();
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
