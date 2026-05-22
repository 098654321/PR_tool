#include "ilp_apply_interposer.hh"

#include "ilp_types.hh"

#include <algo/router/common/maze/mazererouter.hh>
#include <algo/router/common/maze/path_length.hh>
#include <algo/router/routeerror.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/path/pathpackage.hh>
#include <debug/debug.hh>
#include <hardware/interposer.hh>
#include <hardware/tob/tob.hh>
#include <hardware/track/trackcoord.hh>

#include <algorithm>
#include <format>
#include <map>
#include <queue>
#include <stdexcept>

namespace PR_tool {

namespace {

auto tob_from_linear(const std::size_t tob_linear) -> hardware::TOBCoord {
    const auto width = static_cast<std::size_t>(hardware::Interposer::TOB_ARRAY_WIDTH);
    return hardware::TOBCoord {
        static_cast<std::i64>(tob_linear / width),
        static_cast<std::i64>(tob_linear % width)};
}

auto bump_index_from_coord(const Bump_coord& bump) -> std::size_t {
    return bump.Bank * 64 + bump.Group * 8 + bump.Index;
}

auto find_bump(hardware::Interposer* interposer, const Bump_coord& bump_coord) -> hardware::Bump* {
    const auto tob = interposer->get_tob(tob_from_linear(bump_coord.TOB));
    if (!tob.has_value()) {
        return nullptr;
    }
    const auto bump_index = bump_index_from_coord(bump_coord);
    const auto bump = (*tob)->get_bump(bump_index);
    if (!bump.has_value()) {
        return nullptr;
    }
    return bump.value();
}

} // namespace

auto apply_tob_ilp_result_to_interposer(
    hardware::Interposer* interposer,
    const TobIlpResult& result
) -> void {
    if (interposer == nullptr) {
        return;
    }

    auto hori_mux_by_tob_v = std::map<std::pair<std::size_t, std::size_t>, std::size_t> {};
    for (const auto& w : result.active_w) {
        if (!w.has_track) {
            continue;
        }
        const auto key = std::pair<std::size_t, std::size_t> {w.bump.TOB, w.j * 8 + w.k};
        if (!hori_mux_by_tob_v.contains(key)) {
            hori_mux_by_tob_v[key] = bump_index_from_coord(w.bump) / hardware::TOB::BUMP_TO_HORI_MUX_SIZE;
        }
    }

    std::size_t s_applied = 0;
    for (const auto& s : result.active_s) {
        const auto tob = interposer->get_tob(tob_from_linear(s.tob));
        if (!tob.has_value()) {
            debug::warning_fmt("apply ILP: TOB {} not found for S(v={})", s.tob, s.v);
            continue;
        }
        const auto key = std::pair<std::size_t, std::size_t> {s.tob, s.v};
        const auto it = hori_mux_by_tob_v.find(key);
        if (it == hori_mux_by_tob_v.end()) {
            debug::warning_fmt("apply ILP: no W assignment for S on TOB {} v={}", s.tob, s.v);
            continue;
        }
        const auto hori_index = it->second * hardware::TOB::HORI_TO_VERI_MUX_SIZE + s.j;
        const auto hori_info = hardware::TOB::hori_to_vert_mux_info(hori_index);
        try {
            auto mux_conn = (*tob)->hori_to_vert_muxs(std::get<0>(hori_info))->connector(std::get<1>(hori_info), s.k, false);
            mux_conn.connect();
            s_applied += 1;
        }
        catch (const std::exception& e) {
            debug::warning_fmt("apply ILP: S on TOB {} v={} failed: {}", s.tob, s.v, e.what());
        }
    }

    std::size_t w_applied = 0;
    for (const auto& w : result.active_w) {
        if (!w.has_track) {
            continue;
        }
        auto* bump = find_bump(interposer, w.bump);
        if (bump == nullptr) {
            debug::warning_fmt(
                "apply ILP: bump(T{},B{},G{},I{}) not found",
                w.bump.TOB,
                w.bump.Bank,
                w.bump.Group,
                w.bump.Index);
            continue;
        }
        const auto bump_index = bump_index_from_coord(w.bump);
        auto* tob = bump->tob();
        if (tob == nullptr) {
            debug::warning_fmt("apply ILP: bump(T{},B{},G{},I{}) has no TOB", w.bump.TOB, w.bump.Bank, w.bump.Group, w.bump.Index);
            continue;
        }
        try {
            const auto& bump_coord = bump->coord();
            const auto track_coord = hardware::TrackCoord {
                bump_coord.row,
                bump_coord.col,
                hardware::TrackDirection::Vertical,
                w.track};
            const auto track = interposer->get_track(track_coord);
            if (!track.has_value()) {
                debug::warning_fmt(
                    "apply ILP: track {} not found for bump(T{},B{},G{},I{})",
                    w.track,
                    w.bump.TOB,
                    w.bump.Bank,
                    w.bump.Group,
                    w.bump.Index);
                continue;
            }
            bump->set_allocated_track(track.value());
            bump->intersect_access_unit(std::HashSet<std::usize> {map_track(w.track)});
            w_applied += 1;
        }
        catch (const std::exception& e) {
            debug::warning_fmt(
                "apply ILP: W bump(T{},B{},G{},I{}) track={} failed: {}",
                w.bump.TOB,
                w.bump.Bank,
                w.bump.Group,
                w.bump.Index,
                w.track,
                e.what());
        }
    }

    debug::info_fmt("apply ILP to interposer: S configured={}, W bumps configured={}", s_applied, w_applied);
}

namespace {

using routed_path = std::Vector<std::Tuple<hardware::Track*, std::Option<hardware::COBConnector>>>;

auto maze_route_path(
    hardware::Interposer* interposer,
    const std::Vector<hardware::Track*>& begin_tracks,
    const std::HashSet<hardware::Track*>& end_tracks,
    const std::HashSet<hardware::Track*>& occupied_tracks
) -> routed_path {
    auto queue = std::queue<hardware::Track*> {};
    auto prev_track_infos = std::HashMap<hardware::Track*, std::Option<std::Tuple<hardware::Track*, hardware::COBConnector>>> {};

    for (auto* t : begin_tracks) {
        queue.push(t);
        prev_track_infos.insert({t, std::nullopt});
    }

    while (!queue.empty()) {
        auto* track = queue.front();
        queue.pop();

        if (algo::check_found(end_tracks, track)) {
            auto path = routed_path {};
            auto* cur_track = track;
            while (true) {
                const auto prev_it = prev_track_infos.find(cur_track);
                if (prev_it == prev_track_infos.end()) {
                    throw std::runtime_error("ilp TTB maze: missing previous track");
                }
                if (!prev_it->second.has_value()) {
                    break;
                }
                path.emplace_back(cur_track, std::get<1>(*prev_it->second));
                cur_track = std::get<0>(*prev_it->second);
            }
            path.emplace_back(cur_track, std::nullopt);

            routed_path positive {};
            std::transform(path.rbegin(), path.rend(), std::back_inserter(positive), [](const auto& p) { return p; });
            for (auto& [t, cobconnector] : positive) {
                if (cobconnector.has_value()) {
                    cobconnector.value().suspend();
                }
            }
            return positive;
        }

        for (auto& [next_track, connector] : interposer->adjacent_idle_tracks(track)) {
            if (prev_track_infos.contains(next_track) || occupied_tracks.contains(next_track)) {
                continue;
            }
            queue.push(next_track);
            prev_track_infos.insert({next_track, std::Tuple<hardware::Track*, hardware::COBConnector> {track, connector}});
        }
    }

    throw algo::RetryExpt("ilp TTB maze: path not found");
}

auto existing_path_tracks(hardware::Track* track, circuit::Net* net) -> std::Vector<hardware::Track*> {
    auto tracks = std::Vector<hardware::Track*> {};
    for (auto* related : net->related_nets<hardware::Track>(track)) {
        for (auto& [t, _] : related->pathpackage()._regular_path) {
            tracks.emplace_back(t);
        }
    }
    return tracks;
}

} // namespace

auto route_track_to_bumps_nets_post_mcf(
    hardware::Interposer* interposer,
    const std::Vector<std::Rc<circuit::Net>>& all_nets,
    const std::Vector<std::Rc<circuit::Net>>& track_to_bumps_nets
) -> void {
    if (interposer == nullptr || track_to_bumps_nets.empty()) {
        return;
    }

    auto routed_nets = std::Vector<circuit::Net*> {};
    routed_nets.reserve(all_nets.size());
    for (const auto& net : all_nets) {
        routed_nets.push_back(net.get());
    }
    for (const auto& net : track_to_bumps_nets) {
        net->search_related_nets(routed_nets);
    }

    debug::info_fmt(
        "TrackToBumpsNet post-MCF maze: routing {} net(s) using ILP-allocated end tracks",
        track_to_bumps_nets.size());

    for (const auto& net_rc : track_to_bumps_nets) {
        auto* ttbn = dynamic_cast<circuit::TrackToBumpsNet*>(net_rc.get());
        if (ttbn == nullptr) {
            debug::warning_fmt("TrackToBumpsNet maze: unexpected net type for \"{}\"", net_rc->name());
            continue;
        }
        try {
            debug::info_fmt("Maze routing for {}", ttbn->name());
            auto* begin_track = ttbn->begin_track();
            auto begin_tracks_vec = existing_path_tracks(begin_track, ttbn);
            begin_tracks_vec.emplace_back(begin_track);

            circuit::PathPackage path_package {};
            routed_path total_regular_path {};
            std::size_t total_length {0};

            for (auto* end_bump : ttbn->end_bumps()) {
                auto* end_track = end_bump->allocated_track();
                if (end_track == nullptr) {
                    debug::warning_fmt(
                        "TrackToBumpsNet maze: bump at ({},{}) has no ILP-allocated track",
                        end_bump->coord().row,
                        end_bump->coord().col);
                    continue;
                }

                auto end_tracks_set = std::HashSet<hardware::Track*> {end_track};
                auto regular_path = maze_route_path(interposer, begin_tracks_vec, end_tracks_set, std::HashSet<hardware::Track*> {});
                total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());

                auto path = std::Vector<hardware::Track*> {};
                for (auto& [t, _] : regular_path) {
                    path.emplace_back(t);
                }
                const auto bump_index = end_bump->index();
                auto end_connector = end_bump->tob()->bump_track_connectors_chain(
                    bump_index,
                    end_track->coord().index,
                    hardware::TOBBumpDirection::TOBToBump);
                end_connector.give_out();
                path_package._track_to_tob.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*> {
                        end_bump, end_connector, end_track});

                begin_tracks_vec.insert(begin_tracks_vec.end(), path.begin(), path.end());
                total_length += algo::path_length(path);
            }

            path_package._regular_path = total_regular_path;
            path_package._length = total_length + 1;
            ttbn->set_pathpackage(path_package);

            debug::info_fmt(
                "TrackToBumpsNet \"{}\" routed path (length={}), begin_track={}:",
                ttbn->name(),
                path_package._length,
                begin_track->coord());
            path_package.show();
        }
        catch (const algo::RouteExpt& e) {
            debug::error_fmt("TrackToBumpsNet maze failed ({}): {}", net_rc->name(), e.what());
        }
        catch (const std::exception& e) {
            debug::error_fmt("TrackToBumpsNet maze failed ({}): {}", net_rc->name(), e.what());
        }
    }
}

} // namespace PR_tool
