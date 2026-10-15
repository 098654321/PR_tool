#pragma once

#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/maze/path_length.hh>
#include <algo/router/routeerror.hh>
#include <circuit/net/types/syncnet.hh>
#include <debug/debug.hh>

#include <algorithm>
#include <type_traits>

namespace PR_tool::rrr_detail {

// Local adaptation of MazeRouteStrategy::{sync_preroute,route_path,maze_search}:
// those helpers are private, and source/ must remain unchanged. Keep their BFS,
// source COB-unit ordering and reserved synchronous Track ports; no equalization.
inline auto original_sync_path(hardware::Interposer* interposer,
                               const std::Vector<hardware::Track*>& sources,
                               const std::HashSet<hardware::Track*>& sinks,
                               const std::HashSet<hardware::Track*>& blocked) -> algo::routed_path {
    auto queue = std::Queue<hardware::Track*>{};
    auto previous = std::HashMap<hardware::Track*,
        std::Option<std::Tuple<hardware::Track*, hardware::COBConnector>>>{};
    for (auto* track : sources) { queue.push(track); previous.insert({track, std::nullopt}); }
    while (!queue.empty()) {
        auto* track = queue.front(); queue.pop();
        if (algo::check_found(sinks, track)) {
            auto path = algo::routed_path{};
            while (previous.at(track).has_value()) {
                const auto& [from, connector] = *previous.at(track);
                path.emplace_back(track, connector); track = from;
            }
            path.emplace_back(track, std::nullopt);
            std::reverse(path.begin(), path.end());
            for (auto& [node, connector] : path) {
                (void)node;
                if (connector.has_value()) connector->suspend();
            }
            return path;
        }
        for (const auto& [next, connector] : interposer->adjacent_idle_tracks(track)) {
            if (previous.contains(next) || blocked.contains(next)) continue;
            queue.push(next); previous.insert({next, std::Tuple{track, connector}});
        }
    }
    throw algo::RetryExpt("MazeRouteStrategy::maze_search(): path not found");
}

template<class Net>
inline auto original_sync_members(hardware::Interposer* interposer,
                                  std::Vector<std::Rc<Net>>& members,
                                  std::HashSet<hardware::Track*>& blocked) -> void {
    constexpr bool begins_at_bump = !std::is_same_v<Net, circuit::TrackToBumpNet>;
    constexpr bool ends_at_bump = !std::is_same_v<Net, circuit::BumpToTrackNet>;
    for (auto& member : members) {
        auto sources = std::Vector<hardware::Track*>{};
        auto sinks = std::HashSet<hardware::Track*>{};
        auto source_access = std::HashMap<hardware::Track*, hardware::TOBConnector>{};
        auto sink_access = std::HashMap<hardware::Track*, hardware::TOBConnector>{};
        if constexpr (begins_at_bump) {
            source_access = interposer->available_tracks_bump_to_track(member->begin_bump());
            for (const auto& [track, connector] : source_access) { (void)connector; sources.push_back(track); }
            const auto& usage = member->begin_bump()->tob()->cobunit_resources();
            const auto unit = [](hardware::Track* track) {
                const auto i = track->coord().index;
                return (i / (hardware::TOB::INDEX_SIZE / 2)) * hardware::COBUnit::WILTON_SIZE
                    + i % hardware::COBUnit::WILTON_SIZE;
            };
            std::sort(sources.begin(), sources.end(), [&](auto* a, auto* b) { return usage[unit(a)] < usage[unit(b)]; });
        } else sources.push_back(member->begin_track());
        if constexpr (ends_at_bump) {
            sink_access = interposer->available_tracks_track_to_bump(member->end_bump());
            for (const auto& [track, connector] : sink_access) { (void)connector; sinks.insert(track); }
        } else sinks.insert(member->end_track());
        if constexpr (begins_at_bump && ends_at_bump) {
            if (member->begin_bump()->tob() == member->end_bump()->tob())
                throw algo::FinalError("MazeRouteStrategy::sync_preroute(): begin_bump_tob == end_bump_tob");
        }
        if constexpr (!begins_at_bump) blocked.erase(member->begin_track());
        if constexpr (!ends_at_bump) blocked.erase(member->end_track());
        auto package = circuit::PathPackage{};
        package._regular_path = original_sync_path(interposer, sources, sinks, blocked);
        if constexpr (begins_at_bump) {
            auto* track = std::get<0>(package._regular_path.front());
            auto connector = source_access.at(track); connector.give_out();
            package._tob_to_track.emplace_back(member->begin_bump(), connector, track);
        }
        if constexpr (ends_at_bump) {
            auto* track = std::get<0>(package._regular_path.back());
            auto connector = sink_access.at(track); connector.give_out();
            package._track_to_tob.emplace_back(member->end_bump(), connector, track);
        }
        package._length = algo::path_length(package._regular_path) + begins_at_bump + ends_at_bump;
        member->set_pathpackage(package);
    }
}

inline auto original_sync_preroute(hardware::Interposer* interposer, circuit::SyncNet* sync) -> void {
    debug::info_fmt("Maze routing for synchronized {}", sync->name());
    auto blocked = std::HashSet<hardware::Track*>{};
    for (const auto& net : sync->bttnets()) blocked.insert(net->end_track());
    for (const auto& net : sync->ttbnets()) blocked.insert(net->begin_track());
    original_sync_members(interposer, sync->btbnets(), blocked);
    original_sync_members(interposer, sync->ttbnets(), blocked);
    original_sync_members(interposer, sync->bttnets(), blocked);
}

} // namespace PR_tool::rrr_detail
