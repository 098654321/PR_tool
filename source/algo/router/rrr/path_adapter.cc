#include "signal_owners.hh"
#include "path_adapter.hh"
#include "net_adapter.hh"
#include "path_metrics.hh"
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/btnet.hh>
#include <queue>
#include <stdexcept>

namespace PR_tool::rrr {
namespace {
auto track_coord(const UnifiedNode& n) -> hardware::TrackCoord {
    return {n.track_row, n.track_col, n.track_dir == 0 ? hardware::TrackDirection::Horizontal
                                                    : hardware::TrackDirection::Vertical, n.track_index};
}
auto track_id(const UnifiedGraph& graph, hardware::TrackCoord c) -> int {
    GraphNodeRef ref;
    ref.kind = GraphNodeRef::Kind::Track; ref.track_coord = c; ref.track_index = c.index;
    const int id = resolve_graph_node(graph, ref);
    if (id < 0) throw std::runtime_error("RRR import: unresolved Track");
    return id;
}
using Adjacent = std::Map<int, std::Set<int>>;
auto add_edge(const UnifiedGraph& graph, Adjacent& adjacent, int a, int b) -> void {
    if (!graph.directed_arc_set.contains({a,b})) throw std::runtime_error("RRR import: invalid hardware edge");
    adjacent[a].insert(b); adjacent[b].insert(a);
}
auto package_edges(hardware::Interposer* interposer, const UnifiedGraph& graph,
                   const circuit::PathPackage& package) -> Adjacent {
    Adjacent adjacent;
    for (const auto& [track, connector] : package._regular_path) {
        adjacent[track_id(graph, track->coord())];
        if (!connector) continue;
        auto* cob = interposer->get_cob(connector->coord()).value();
        add_edge(graph, adjacent,
                 track_id(graph, cob->to_dir_track_coord(connector->from_dir(), connector->from_track_index())),
                 track_id(graph, cob->to_dir_track_coord(connector->to_dir(), connector->to_track_index())));
    }
    const auto add_tob = [&](const auto& connections) {
        for (const auto& [bump, connector, track] : connections) {
            const auto b = bump_to_routing_coord(bump);
            const auto h = connector.hori_index();
            const int bn = graph.bump_node_by_key.at(b);
            const int hn = graph.hline_node_by_key.at({b.TOB,h/64,(h%64)/8,h%8});
            const int vn = graph.vline_node_by_key.at({b.TOB,connector.vert_index()});
            add_edge(graph, adjacent, bn, hn);
            add_edge(graph, adjacent, hn, vn);
            add_edge(graph, adjacent, vn, track_id(graph, track->coord()));
        }
    };
    add_tob(package._tob_to_track); add_tob(package._track_to_tob);
    return adjacent;
}

auto demand_path(const UnifiedGraph& graph, const RoutingNet& net,
                 const RoutingDemand& demand, const Adjacent& adjacent) -> std::Vector<int> {
    std::Map<int,int> previous;
    std::Queue<int> pending;
    for (auto index : demand.candidate_source_indices) {
        const int root = resolve_graph_node(graph, net.sources.at(index));
        if (root >= 0) { previous.emplace(root,-1); pending.push(root); }
    }
    const int sink = resolve_graph_node(graph, demand.sink);
    while (!pending.empty() && !previous.contains(sink)) {
        const int n = pending.front(); pending.pop();
        auto it = adjacent.find(n);
        if (it == adjacent.end()) continue;
        for (int next : it->second) if (previous.emplace(next,n).second) pending.push(next);
    }
    if (!previous.contains(sink)) return {};
    std::Vector<int> path;
    for (int n = sink; n >= 0; n = previous.at(n)) path.push_back(n);
    std::reverse(path.begin(),path.end());
    return path;
}

auto member(circuit::Net* net, std::size_t demand) -> circuit::Net* {
    auto* sync = dynamic_cast<circuit::SyncNet*>(net);
    if (!sync) return net;
    if (demand < sync->btbnets().size()) return sync->btbnets()[demand].get();
    demand -= sync->btbnets().size();
    if (demand < sync->bttnets().size()) return sync->bttnets()[demand].get();
    demand -= sync->bttnets().size();
    return sync->ttbnets().at(demand).get();
}

// Build from exact graph mux nodes, never reselect an available TOB connector.
auto make_package(hardware::Interposer* interposer, const UnifiedGraph& graph,
                  const std::Vector<std::Vector<int>>& paths, bool reverse) -> circuit::PathPackage {
    circuit::PathPackage package;
    std::Set<int> tracks, bumps;
    std::Set<std::pair<int,int>> edges;
    for (auto path : paths) {
        if (reverse) std::reverse(path.begin(), path.end());
        for (std::size_t i = 0; i < path.size(); ++i) {
            const auto& node = graph.nodes.at(path[i]);
            if (node.kind == UnifiedNodeKind::Track) {
                tracks.insert(path[i]);
                if (i+1 < path.size() && graph.nodes[path[i+1]].kind == UnifiedNodeKind::Track
                    && edges.emplace(path[i],path[i+1]).second) {
                    auto* from = interposer->get_track(track_coord(node)).value();
                    auto* to = interposer->get_track(track_coord(graph.nodes[path[i+1]])).value();
                    bool found = false;
                    for (const auto& [next, connector] : interposer->adjacent_tracks(from)) {
                        if (next != to) continue;
                        package._regular_path.emplace_back(from,connector); found = true; break;
                    }
                    if (!found) throw std::runtime_error("RRR commit: missing COB connector");
                }
            }
            if (node.kind != UnifiedNodeKind::Bump || !bumps.insert(path[i]).second) continue;
            const bool source = i == 0;
            if ((!source && i+1 != path.size()) || path.size() < 4)
                throw std::runtime_error("RRR commit: invalid bump position");
            const auto& h = graph.nodes.at(path[source ? 1 : i-1]);
            const auto& v = graph.nodes.at(path[source ? 2 : i-2]);
            const auto& t = graph.nodes.at(path[source ? 3 : i-3]);
            if (h.kind != UnifiedNodeKind::HLine || v.kind != UnifiedNodeKind::VLine || t.kind != UnifiedNodeKind::Track)
                throw std::runtime_error("RRR commit: malformed TOB chain");
            auto* bump = interposer->get_bump(node.tob / hardware::Interposer::TOB_ARRAY_WIDTH,
                                           node.tob % hardware::Interposer::TOB_ARRAY_WIDTH,
                                           node.bump.Bank*64+node.bump.Group*8+node.bump.Index).value();
            circuit::TOBConnectorInfo info{bump->index(), h.bank*64+h.group*8+h.line_index,
                v.line_index,t.track_index, source ? hardware::TOBSignalDirection::BumpToTrack
                                                  : hardware::TOBSignalDirection::TrackToBump,
                bump->tob()->coord()};
            auto connection = std::make_tuple(bump,info.create_tobconnector(interposer),
                                              interposer->get_track(track_coord(t)).value());
            if (source) package._tob_to_track.push_back(connection);
            else package._track_to_tob.push_back(connection);
        }
    }
    // Include terminal tracks and single-track paths; connector entries carry all edges.
    std::Set<hardware::Track*> listed;
    for (const auto& [track, connector] : package._regular_path) listed.insert(track);
    for (int n : tracks) {
        auto* track = interposer->get_track(track_coord(graph.nodes[n])).value();
        if (!listed.contains(track)) package._regular_path.emplace_back(track,std::nullopt);
    }
    package._length = tracks.size()+bumps.size();
    return package;
}
}

auto import_paths(hardware::Interposer* interposer, const UnifiedGraph& graph,
                  const std::Vector<RoutingNet>& nets, const std::Vector<circuit::Net*>& raw) -> NetPaths {
    NetPaths paths(nets.size());
    const auto aliases=signal_owners(graph,nets);
    const auto canonical=[&](OwnerId id){auto it=aliases.find(id);return it==aliases.end()?id:it->second;};
    std::Map<OwnerId,Adjacent> signals;
    for(std::size_t ni=0;ni<nets.size();++ni) {
        const auto adjacent=package_edges(interposer,graph,raw.at(ni)->pathpackage());
        for(const auto& d:nets[ni].demands) {
            auto& merged=signals[canonical(demand_owner(nets[ni],d))];
            for(const auto& [u,vs]:adjacent)merged[u].insert(vs.begin(),vs.end());
        }
    }
    for(std::size_t ni=0;ni<nets.size();++ni)
        for(const auto& d:nets[ni].demands)
            paths[ni].push_back(demand_path(graph,nets[ni],d,signals.at(canonical(demand_owner(nets[ni],d)))));
    return paths;
}

auto commit_paths(hardware::Interposer* interposer, const UnifiedGraph& graph,
                  const std::Vector<RoutingNet>& nets, const NetPaths& paths,
                  const std::Vector<circuit::Net*>& raw) -> void {
    // Prepare all packages before mutating hardware. Each sync lane gets its own package.
    std::Vector<std::pair<circuit::Net*,circuit::PathPackage>> prepared;
    for (std::size_t ni = 0; ni < nets.size(); ++ni) {
        if (nets[ni].is_sync_bus) {
            for (std::size_t d = 0; d < nets[ni].demands.size(); ++d) {
                auto* lane = member(raw.at(ni),d);
                prepared.emplace_back(lane,make_package(interposer,graph,{paths.at(ni).at(d)},
                    dynamic_cast<circuit::BumpToTrackNet*>(lane) != nullptr));
            }
        } else {
            prepared.emplace_back(raw.at(ni),make_package(interposer,graph,paths.at(ni),
                dynamic_cast<circuit::BumpToTrackNet*>(raw.at(ni)) != nullptr));
        }
    }
    for (auto* net : raw) net->clear_path();
    interposer->reset_regs();
    for (auto& [net,package] : prepared) { net->set_pathpackage(package); net->pathpackage().occupy_all(); }
    for (auto* net : raw) if (auto* sync = dynamic_cast<circuit::SyncNet*>(net)) sync->collect_package();
    if (import_paths(interposer,graph,nets,raw) != paths)
        throw std::runtime_error("RRR commit: path round-trip mismatch");
}
}
