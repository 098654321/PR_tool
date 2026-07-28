#include "commit_paths.hh"

#include "common/hw_map.hh"
#include "sat/routing_path_log.hh"
#include "scope/build_routing_nets.hh"

#include <algo/router/common/maze/path_length.hh>

#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/path/pathpackage.hh>
#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

auto routing_bump_index(const Bump_coord& bump) -> std::size_t {
    return bump.Bank * 64 + bump.Group * 8 + bump.Index;
}

auto resolve_bump(hardware::Interposer* interposer, const Bump_coord& bump) -> hardware::Bump* {
    const auto [tr, tc] = tob_index_from_linear(bump.TOB);
    const auto opt = interposer->get_bump(
        static_cast<std::i64>(tr),
        static_cast<std::i64>(tc),
        routing_bump_index(bump));
    return opt.has_value() ? opt.value() : nullptr;
}

auto track_coord_from_node(const UnifiedNode& node) -> hardware::TrackCoord {
    return hardware::TrackCoord {
        node.track_row,
        node.track_col,
        node.track_dir == 0 ? hardware::TrackDirection::Horizontal
                            : hardware::TrackDirection::Vertical,
        node.track_index};
}

auto resolve_track(hardware::Interposer* interposer, const UnifiedNode& node) -> hardware::Track* {
    if (node.kind != UnifiedNodeKind::Track) {
        return nullptr;
    }
    const auto opt = interposer->get_track(track_coord_from_node(node));
    return opt.has_value() ? opt.value() : nullptr;
}

auto find_circuit_net(circuit::BaseDie& basedie, const RoutingNet& routing_net) -> circuit::Net* {
    for (const auto& net : basedie.nets_to_vector()) {
        if (net->uid() == routing_net.origin_uid) {
            return net.get();
        }
    }
    for (const auto& net : basedie.nets_to_vector()) {
        if (net->name() == routing_net.name) {
            return net.get();
        }
    }
    return nullptr;
}

auto find_cob_info(
    hardware::Interposer* interposer,
    hardware::Track* from,
    hardware::Track* to
) -> std::Option<circuit::COBConnectorInfo> {
    for (auto& [adj_track, connector] : interposer->adjacent_tracks(from)) {
        if (adj_track == to) {
            return circuit::COBConnectorInfo{
                connector.coord(),
                connector.from_dir(),
                connector.from_track_index(),
                connector.to_dir(),
                connector.to_track_index(),
            };
        }
    }
    return std::nullopt;
}

auto tob_info_from(hardware::Bump* bump, const hardware::TOBConnector& connector)
    -> circuit::TOBConnectorInfo {
    return circuit::TOBConnectorInfo{
        connector.bump_index(),
        connector.hori_index(),
        connector.vert_index(),
        connector.track_index(),
        connector.single_direction(),
        bump->tob()->coord(),
    };
}

auto collect_ordered_tracks(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const std::Vector<int>& node_path
) -> std::Vector<hardware::Track*> {
    auto tracks = std::Vector<hardware::Track*> {};
    for (const int node_id : node_path) {
        if (node_id < 0 || static_cast<std::size_t>(node_id) >= graph.nodes.size()) {
            continue;
        }
        const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
        if (node.kind != UnifiedNodeKind::Track) {
            continue;
        }
        auto* track = resolve_track(interposer, node);
        if (track == nullptr) {
            throw std::runtime_error(std::format(
                "commit: track node {} not found in interposer",
                format_path_node(graph, node_id)));
        }
        if (!tracks.empty() && tracks.back() == track) {
            continue;
        }
        tracks.push_back(track);
    }
    return tracks;
}

auto append_track_chain(
    hardware::Interposer* interposer,
    const std::Vector<hardware::Track*>& tracks,
    circuit::HistoryPathPackage& history
) -> void {
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        std::Option<circuit::COBConnectorInfo> connector_info {std::nullopt};
        if (i + 1 < tracks.size()) {
            connector_info = find_cob_info(interposer, tracks[i], tracks[i + 1]);
            if (!connector_info.has_value()) {
                throw std::runtime_error(std::format(
                    "commit: missing COB connector between {} and {}",
                    tracks[i]->coord().to_string(),
                    tracks[i + 1]->coord().to_string()));
            }
        }
        history._regular_path.emplace_back(tracks[i]->coord(), connector_info);
    }
}

auto append_bump_to_track(
    hardware::Interposer* interposer,
    hardware::Bump* bump,
    hardware::Track* track,
    circuit::HistoryPathPackage& history
) -> void {
    auto tracks_map = interposer->available_tracks_bump_to_track(bump, true);
    const auto iter = tracks_map.find(track);
    if (iter == tracks_map.end()) {
        throw std::runtime_error(std::format(
            "commit: cannot find bump_to_track connector for bump {} -> track {}",
            bump->coord().to_string(),
            track->coord().to_string()));
    }
    history._tob_to_track.emplace_back(
        bump->coord(), tob_info_from(bump, iter->second), track->coord());
}

auto append_track_to_bump(
    hardware::Interposer* interposer,
    hardware::Bump* bump,
    hardware::Track* track,
    circuit::HistoryPathPackage& history
) -> void {
    auto tracks_map = interposer->available_tracks_track_to_bump(bump, true);
    const auto iter = tracks_map.find(track);
    if (iter == tracks_map.end()) {
        throw std::runtime_error(std::format(
            "commit: cannot find track_to_bump connector for track {} -> bump {}",
            track->coord().to_string(),
            bump->coord().to_string()));
    }
    history._track_to_tob.emplace_back(
        bump->coord(), tob_info_from(bump, iter->second), track->coord());
}

auto source_ref_for_pair(const RoutingNet& net, const SourceSinkPairPath& pair_path) -> GraphNodeRef {
    if (pair_path.source_index < net.sources.size()) {
        return net.sources[pair_path.source_index];
    }
    return {};
}

auto build_single_history_package(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const RoutingNet& routing_net,
    const SourceSinkPairPath& pair_path
) -> circuit::HistoryPathPackage {
    if (pair_path.demand_id >= routing_net.demands.size()) {
        throw std::runtime_error(std::format(
            "commit: net '{}' demand {} out of range",
            routing_net.name,
            pair_path.demand_id));
    }

    const auto& demand = routing_net.demands[pair_path.demand_id];
    const auto tracks = collect_ordered_tracks(interposer, graph, pair_path.node_path);
    if (tracks.empty()) {
        throw std::runtime_error(std::format(
            "commit: net '{}' demand {} has no track nodes in SAT path",
            routing_net.name,
            pair_path.demand_id));
    }

    circuit::PathPackage empty_package {};
    circuit::HistoryPathPackage history{empty_package};
    history.clear_all();

    append_track_chain(interposer, tracks, history);

    if (routing_net.kind == RoutingNetKind::Bnet) {
        const auto source_ref = source_ref_for_pair(routing_net, pair_path);
        if (source_ref.kind != GraphNodeRef::Kind::Bump) {
            throw std::runtime_error(std::format(
                "commit: Bnet '{}' expected bump source",
                routing_net.name));
        }
        if (demand.sink.kind != GraphNodeRef::Kind::Bump) {
            throw std::runtime_error(std::format(
                "commit: Bnet '{}' expected bump sink",
                routing_net.name));
        }
        auto* begin_bump = resolve_bump(interposer, source_ref.bump);
        auto* end_bump = resolve_bump(interposer, demand.sink.bump);
        if (begin_bump == nullptr || end_bump == nullptr) {
            throw std::runtime_error(std::format(
                "commit: Bnet '{}' could not resolve endpoint bumps",
                routing_net.name));
        }
        append_bump_to_track(interposer, begin_bump, tracks.front(), history);
        append_track_to_bump(interposer, end_bump, tracks.back(), history);
    }
    else if (routing_net.kind == RoutingNetKind::Tnet) {
        const auto source_ref = source_ref_for_pair(routing_net, pair_path);
        if (source_ref.kind != GraphNodeRef::Kind::Track) {
            throw std::runtime_error(std::format(
                "commit: Tnet '{}' expected track source",
                routing_net.name));
        }
        if (demand.sink.kind != GraphNodeRef::Kind::Bump) {
            throw std::runtime_error(std::format(
                "commit: Tnet '{}' expected bump sink",
                routing_net.name));
        }
        auto* source_track = resolve_track(
            interposer,
            [&] {
                UnifiedNode node {};
                node.kind = UnifiedNodeKind::Track;
                node.track_row = source_ref.track_coord.row;
                node.track_col = source_ref.track_coord.col;
                node.track_dir =
                    source_ref.track_coord.dir == hardware::TrackDirection::Horizontal ? 0 : 1;
                node.track_index = source_ref.track_index;
                return node;
            }());
        auto* end_bump = resolve_bump(interposer, demand.sink.bump);
        if (source_track == nullptr || end_bump == nullptr) {
            throw std::runtime_error(std::format(
                "commit: Tnet '{}' could not resolve endpoints",
                routing_net.name));
        }
        if (source_track != tracks.front()) {
            throw std::runtime_error(std::format(
                "commit: Tnet '{}' path does not start at expected source track {} (got {})",
                routing_net.name,
                source_track->coord().to_string(),
                tracks.front()->coord().to_string()));
        }
        append_track_to_bump(interposer, end_bump, tracks.back(), history);
    }
    else {
        throw std::runtime_error(std::format(
            "commit: unsupported routing net kind for '{}'",
            routing_net.name));
    }

    std::usize path_l = algo::path_length(tracks);
    if (!history._tob_to_track.empty()) {
        path_l += 1;
    }
    if (!history._track_to_tob.empty()) {
        path_l += 1;
    }
    history._length = path_l;
    return history;
}

auto merge_track_to_bumps_history(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const RoutingNet& routing_net,
    const std::Vector<const SourceSinkPairPath*>& paths
) -> circuit::HistoryPathPackage {
    circuit::PathPackage empty_package {};
    circuit::HistoryPathPackage history{empty_package};
    history.clear_all();
    std::usize total_length {0};

    for (const auto* pair_path : paths) {
        auto member_history = build_single_history_package(interposer, graph, routing_net, *pair_path);
        history._regular_path.insert(
            history._regular_path.end(),
            member_history._regular_path.begin(),
            member_history._regular_path.end());
        history._track_to_tob.insert(
            history._track_to_tob.end(),
            member_history._track_to_tob.begin(),
            member_history._track_to_tob.end());

        auto path_tracks = std::Vector<hardware::Track*> {};
        for (const auto& [track_coord, connector_info] : member_history._regular_path) {
            (void)connector_info;
            auto track = interposer->get_track(track_coord);
            if (!track.has_value()) {
                throw std::runtime_error(std::format(
                    "commit: track not found during merge: {}",
                    track_coord.to_string()));
            }
            path_tracks.push_back(track.value());
        }
        if (!path_tracks.empty()) {
            total_length += algo::path_length(path_tracks);
        }
    }

    history._length = total_length + 1;
    return history;
}

auto commit_history_to_net(
    hardware::Interposer* interposer,
    circuit::Net* circuit_net,
    circuit::HistoryPathPackage history,
    bool occupy
) -> void {
    circuit::PathPackage package{history, interposer};
    if (occupy) {
        package.occupy_all();
    }
    circuit_net->set_pathpackage(package);
}

auto paths_for_net(
    const SatRoutingResult& sat_result,
    std::size_t net_id
) -> std::Vector<const SourceSinkPairPath*> {
    auto out = std::Vector<const SourceSinkPairPath*> {};
    for (const auto& path : sat_result.paths) {
        if (path.net_id == net_id) {
            out.push_back(&path);
        }
    }
    return out;
}

} // namespace

auto commit_sat_paths_to_nets(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& routing_nets,
    const SatRoutingResult& sat_result
) -> CommitPathsResult {
    if (!sat_result.ok) {
        return CommitPathsResult {false, "commit skipped: SAT result not ok"};
    }

    try {
        for (const auto& routing_net : routing_nets) {
            if (routing_net.kind == RoutingNetKind::PNnet) {
                return CommitPathsResult {
                    false,
                    std::format(
                        "commit not implemented for PNnet '{}'",
                        routing_net.name)};
            }

            auto* circuit_net = find_circuit_net(basedie, routing_net);
            if (circuit_net == nullptr) {
                return CommitPathsResult {
                    false,
                    std::format(
                        "commit: circuit net not found for routing net '{}' uid='{}'",
                        routing_net.name,
                        routing_net.origin_uid)};
            }

            const auto paths = paths_for_net(sat_result, routing_net.net_id);
            if (paths.empty()) {
                return CommitPathsResult {
                    false,
                    std::format(
                        "commit: no SAT paths for routing net '{}'",
                        routing_net.name)};
            }

            if (routing_net.is_sync_bus) {
                for (const auto* pair_path : paths) {
                    auto history = build_single_history_package(interposer, graph, routing_net, *pair_path);
                    commit_history_to_net(interposer, circuit_net, std::move(history), false);
                }
                if (auto* sync_net = dynamic_cast<circuit::SyncNet*>(circuit_net)) {
                    sync_net->collect_package();
                    sync_net->pathpackage().occupy_all();
                }
                continue;
            }

            if (paths.size() == 1) {
                auto history = build_single_history_package(interposer, graph, routing_net, *paths.front());
                commit_history_to_net(interposer, circuit_net, std::move(history), true);
                continue;
            }

            if (routing_net.kind == RoutingNetKind::Tnet
                && dynamic_cast<circuit::TrackToBumpsNet*>(circuit_net) != nullptr) {
                auto history = merge_track_to_bumps_history(interposer, graph, routing_net, paths);
                commit_history_to_net(interposer, circuit_net, std::move(history), true);
                continue;
            }

            return CommitPathsResult {
                false,
                std::format(
                    "commit: unsupported multi-path net '{}' ({} paths)",
                    routing_net.name,
                    paths.size())};
        }

        return CommitPathsResult {true, "committed"};
    }
    catch (const std::exception& error) {
        return CommitPathsResult {false, error.what()};
    }
}

} // namespace PR_tool
