#include "maze_check/maze_route_ilp_fixed.hh"

#include "mcf/mcf_graph.hh"

#include <algo/router/routeerror.hh>
#include <circuit/net/net.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/bbsnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/btsnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <hardware/bump/bump.hh>
#include <hardware/cob/cobconnector.hh>
#include <hardware/tob/tob.hh>
#include <hardware/tob/tobconnector.hh>
#include <hardware/track/track.hh>
#include <hardware/track/trackcoord.hh>
#include <debug/debug.hh>

#include <algorithm>
#include <cctype>
#include <format>
#include <map>
#include <optional>
#include <set>

namespace PR_tool {

using RoutedPath = algo::routed_path;

namespace {

auto bump_to_coord(const hardware::Bump* bump) -> Bump_coord {
    const auto bump_index = bump->index();
    const auto tob_coord = bump->tob()->coord();
    return Bump_coord {
        static_cast<std::size_t>(tob_coord.row * hardware::Interposer::TOB_ARRAY_WIDTH + tob_coord.col),
        bump_index / 64,
        (bump_index % 64) / 8,
        bump_index % 8
    };
}

auto bumps_equal(const Bump_coord& a, const Bump_coord& b) -> bool {
    return a.TOB == b.TOB && a.Bank == b.Bank && a.Group == b.Group && a.Index == b.Index;
}

auto tob_from_linear(const std::size_t tob_linear) -> hardware::TOBCoord {
    const auto width = static_cast<std::size_t>(hardware::Interposer::TOB_ARRAY_WIDTH);
    return hardware::TOBCoord {
        static_cast<std::i64>(tob_linear / width),
        static_cast<std::i64>(tob_linear % width)};
}

auto find_bump(hardware::Interposer* interposer, const Bump_coord& bump_coord) -> hardware::Bump* {
    const auto tob = interposer->get_tob(tob_from_linear(bump_coord.TOB));
    if (!tob.has_value()) {
        return nullptr;
    }
    const auto bump_index = bump_coord.Bank * 64 + bump_coord.Group * 8 + bump_coord.Index;
    const auto bump = (*tob)->get_bump(bump_index);
    if (!bump.has_value()) {
        return nullptr;
    }
    return bump.value();
}

auto track_from_bump_and_index(
    hardware::Interposer* interposer,
    const Bump_coord& bump_coord,
    const std::size_t track_index
) -> hardware::Track* {
    auto* bump = find_bump(interposer, bump_coord);
    if (bump == nullptr) {
        return nullptr;
    }
    const auto& bump_hw_coord = bump->coord();
    const auto track_coord = hardware::TrackCoord {
        bump_hw_coord.row,
        bump_hw_coord.col,
        hardware::TrackDirection::Vertical,
        track_index};
    const auto track = interposer->get_track(track_coord);
    if (!track.has_value()) {
        return nullptr;
    }
    return track.value();
}

auto track_from_coord(hardware::Interposer* interposer, const hardware::TrackCoord& tc) -> hardware::Track* {
    const auto track = interposer->get_track(tc);
    if (!track.has_value()) {
        return nullptr;
    }
    return track.value();
}

auto resolve_start_track(
    hardware::Interposer* interposer,
    const Net_cost_record& record,
    const TobIlpRecordTrackEndpoint& endpoint
) -> hardware::Track* {
    if (!endpoint.has_start_track || record.start_bumps.empty()) {
        return nullptr;
    }
    return track_from_bump_and_index(interposer, record.start_bumps.front(), endpoint.start_track);
}

auto resolve_end_track(
    hardware::Interposer* interposer,
    const Net_cost_record& record,
    const TobIlpRecordTrackEndpoint& endpoint
) -> hardware::Track* {
    if (!endpoint.has_end_track) {
        return nullptr;
    }
    if (record.type == Net_type::Bnet) {
        if (record.end_bumps.empty()) {
            return nullptr;
        }
        return track_from_bump_and_index(interposer, record.end_bumps.front(), endpoint.end_track);
    }
    if (record.type == Net_type::Tnet) {
        if (!record.mcf_has_end_track) {
            return nullptr;
        }
        return track_from_coord(interposer, record.mcf_end_track);
    }
    if (record.type == Net_type::PNnet) {
        const auto it = record.pn_end_track_coord_by_index.find(endpoint.end_track);
        if (it == record.pn_end_track_coord_by_index.end()) {
            return nullptr;
        }
        return track_from_coord(interposer, it->second);
    }
    return nullptr;
}

// Mirrors MazeRouteStrategy::maze_search (BFS on adjacent_idle_tracks).
auto ilp_fixed_maze_search(
    hardware::Interposer* interposer,
    const std::Vector<hardware::Track*>& begin_tracks,
    const std::HashSet<hardware::Track*>& end_tracks,
    const std::HashSet<hardware::Track*>& occupied_tracks
) -> RoutedPath {
    using namespace hardware;

    auto queue = std::Queue<Track*> {};
    auto prev_track_infos = std::HashMap<Track*, std::Option<std::Tuple<Track*, COBConnector>>> {};

    for (auto* t : begin_tracks) {
        queue.push(t);
        prev_track_infos.insert({t, std::nullopt});
    }

    while (!queue.empty()) {
        auto* track = queue.front();
        queue.pop();

        if (algo::check_found(end_tracks, track)) {
            auto path = std::Vector<std::Tuple<Track*, std::Option<COBConnector>>> {};
            auto* cur_track = track;
            while (true) {
                const auto prev_track_info = prev_track_infos.find(cur_track);
                if (prev_track_info == prev_track_infos.end()) {
                    throw algo::FinalError("ilp_fixed_maze_search: cannot find previous track");
                }
                if (!prev_track_info->second.has_value()) {
                    break;
                }
                path.emplace_back(cur_track, std::get<1>(*prev_track_info->second));
                cur_track = std::get<0>(*prev_track_info->second);
            }
            path.emplace_back(cur_track, std::nullopt);
            return path;
        }

        for (auto& [next_track, connector] : interposer->adjacent_idle_tracks(track)) {
            if (prev_track_infos.contains(next_track) || occupied_tracks.contains(next_track)) {
                continue;
            }
            queue.push(next_track);
            prev_track_infos.insert({next_track, std::Tuple<Track*, COBConnector> {track, connector}});
        }
    }

    throw algo::RetryExpt("ilp_fixed_maze_search: path not found");
}

// Mirrors MazeRouteStrategy::route_path (reverse path + suspend COB connectors).
auto ilp_fixed_route_path(
    hardware::Interposer* interposer,
    const std::Vector<hardware::Track*>& begin_tracks,
    const std::HashSet<hardware::Track*>& end_tracks,
    const std::HashSet<hardware::Track*>& occupied_tracks
) -> RoutedPath {
    auto path_info = ilp_fixed_maze_search(interposer, begin_tracks, end_tracks, occupied_tracks);
    auto path = RoutedPath {};
    path.reserve(path_info.size());
    for (auto it = path_info.rbegin(); it != path_info.rend(); ++it) {
        path.push_back(*it);
    }
    for (auto& [track, cobconnector] : path) {
        (void)track;
        if (cobconnector.has_value()) {
            cobconnector.value().suspend();
        }
    }
    return path;
}

auto track_set_from_ptr(hardware::Track* track) -> std::HashSet<hardware::Track*> {
    auto set = std::HashSet<hardware::Track*> {};
    set.emplace(track);
    return set;
}

auto find_record_index(
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const auto& predicate
) -> std::optional<std::size_t> {
    for (const auto idx : record_indices) {
        if (predicate(records[idx])) {
            return idx;
        }
    }
    return std::nullopt;
}

auto attach_bump_to_track_tob(
    hardware::Interposer* interposer,
    hardware::Bump* bump,
    hardware::Track* track,
    circuit::PathPackage& package
) -> void {
    auto tracks_map = interposer->available_tracks_bump_to_track(bump);
    for (auto& [t, connector] : tracks_map) {
        if (t->coord() == track->coord()) {
            connector.give_out();
            package._tob_to_track.emplace_back(
                std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*> {bump, connector, track});
            return;
        }
    }
}

auto attach_track_to_bump_tob(
    hardware::Interposer* interposer,
    hardware::Bump* bump,
    hardware::Track* track,
    circuit::PathPackage& package
) -> void {
    auto tracks_map = interposer->available_tracks_track_to_bump(bump);
    for (auto& [t, connector] : tracks_map) {
        if (t->coord() == track->coord()) {
            connector.give_out();
            package._track_to_tob.emplace_back(
                std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*> {bump, connector, track});
            return;
        }
    }
}

auto route_single_ilp_segment(
    hardware::Interposer* interposer,
    hardware::Track* start_track,
    hardware::Track* end_track,
    const std::HashSet<hardware::Track*>& occupied_tracks
) -> RoutedPath {
    if (start_track == nullptr || end_track == nullptr) {
        throw algo::RetryExpt("route_single_ilp_segment: null endpoint track");
    }
    auto begin_vec = std::Vector<hardware::Track*> {start_track};
    return ilp_fixed_route_path(interposer, begin_vec, track_set_from_ptr(end_track), occupied_tracks);
}

auto route_record_ilp_segment(
    hardware::Interposer* interposer,
    const Net_cost_record& record,
    const TobIlpRecordTrackEndpoint& endpoint,
    const std::HashSet<hardware::Track*>& occupied_tracks
) -> RoutedPath {
    auto* start = resolve_start_track(interposer, record, endpoint);
    auto* end = resolve_end_track(interposer, record, endpoint);
    return route_single_ilp_segment(interposer, start, end, occupied_tracks);
}

auto find_record_for_btb(
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const hardware::Bump* begin_bump,
    const hardware::Bump* end_bump
) -> std::optional<std::size_t> {
    const auto begin = bump_to_coord(begin_bump);
    const auto end = bump_to_coord(end_bump);
    return find_record_index(record_indices, records, [&](const Net_cost_record& r) {
        return r.type == Net_type::Bnet && !r.start_bumps.empty() && !r.end_bumps.empty()
            && bumps_equal(r.start_bumps.front(), begin) && bumps_equal(r.end_bumps.front(), end);
    });
}

auto find_record_for_btt(
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const hardware::Bump* begin_bump,
    const hardware::Track* end_track
) -> std::optional<std::size_t> {
    const auto begin = bump_to_coord(begin_bump);
    const auto end_coord = end_track->coord();
    return find_record_index(record_indices, records, [&](const Net_cost_record& r) {
        return r.type == Net_type::Tnet && !r.start_bumps.empty() && bumps_equal(r.start_bumps.front(), begin)
            && r.mcf_has_end_track && r.mcf_end_track == end_coord;
    });
}

auto find_record_for_ttb(
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const hardware::Track* begin_track,
    const hardware::Bump* end_bump
) -> std::optional<std::size_t> {
    const auto end = bump_to_coord(end_bump);
    const auto begin_coord = begin_track->coord();
    return find_record_index(record_indices, records, [&](const Net_cost_record& r) {
        return r.type == Net_type::Tnet && !r.start_bumps.empty() && bumps_equal(r.start_bumps.front(), end)
            && r.mcf_has_end_track && r.mcf_end_track == begin_coord;
    });
}

auto find_record_for_bump(
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const hardware::Bump* bump
) -> std::optional<std::size_t> {
    const auto coord = bump_to_coord(bump);
    return find_record_index(record_indices, records, [&](const Net_cost_record& r) {
        return !r.start_bumps.empty() && bumps_equal(r.start_bumps.front(), coord);
    });
}

auto bump_from_record(hardware::Interposer* interposer, const Net_cost_record& record) -> hardware::Bump* {
    if (record.start_bumps.empty()) {
        throw algo::RetryExpt("bump_from_record: missing start bump");
    }
    auto* bump = find_bump(interposer, record.start_bumps.front());
    if (bump == nullptr) {
        throw algo::RetryExpt(std::format("bump_from_record: bump not found for net \"{}\"", record.net_name));
    }
    return bump;
}

auto track_set_from_begin_tracks(circuit::TracksToBumpsNet* net) -> std::HashSet<hardware::Track*> {
    auto set = std::HashSet<hardware::Track*> {};
    for (auto* t : net->begin_tracks()) {
        set.emplace(t);
    }
    return set;
}

auto dedupe_track_vector(const std::Vector<hardware::Track*>& tracks) -> std::Vector<hardware::Track*> {
    auto out = std::Vector<hardware::Track*> {};
    auto seen = std::HashSet<hardware::Track*> {};
    out.reserve(tracks.size());
    for (auto* t : tracks) {
        if (t == nullptr || seen.contains(t)) {
            continue;
        }
        seen.emplace(t);
        out.emplace_back(t);
    }
    return out;
}

auto store_segment(OriginRouteSegments* segments_out, const std::size_t rec_idx, const RoutedPath& path) -> void {
    if (segments_out != nullptr) {
        segments_out->by_record_index[rec_idx] = path;
    }
}

auto track_coord_text_local(const hardware::TrackCoord& tc) -> std::String {
    const auto dir = tc.dir == hardware::TrackDirection::Horizontal ? "H" : "V";
    return std::format("({},{},{},idx={})", tc.row, tc.col, dir, tc.index);
}

auto count_connector_hops(const RoutedPath& path) -> std::size_t {
    std::size_t hops {0};
    for (const auto& hop : path) {
        if (std::get<1>(hop).has_value()) {
            ++hops;
        }
    }
    return hops;
}

auto routed_path_text(const RoutedPath& path) -> std::String {
    if (path.empty()) {
        return "(empty)";
    }
    auto parts = std::Vector<std::String> {};
    parts.reserve(path.size());
    for (const auto& hop : path) {
        const auto* track = std::get<0>(hop);
        if (track == nullptr) {
            parts.emplace_back("(null-track)");
            continue;
        }
        parts.emplace_back(track_coord_text_local(track->coord()));
    }
    auto text = std::String {};
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (i != 0) {
            text += " -> ";
        }
        text += parts[i];
    }
    return text;
}

auto record_origin_display_key(const Net_cost_record& record) -> std::String {
    return record.origin_key.empty() ? record.net_name : record.origin_key;
}

auto log_maze_record_route(
    const char* prefix,
    const Net_cost_record& record,
    const TobIlpRecordTrackEndpoint& endpoint,
    const char* result,
    const char* reason,
    const std::size_t path_hops,
    const std::String& path_text,
    const char* end_track_label
) -> void {
    if (prefix == nullptr) {
        return;
    }
    const auto start_text = endpoint.has_start_track ? std::to_string(endpoint.start_track) : std::String {"-"};
    const auto end_text = end_track_label != nullptr ? std::String {end_track_label}
        : (endpoint.has_end_track ? std::to_string(endpoint.end_track) : std::String {"-"});
    debug::info_fmt(
        "{} record_id={} net=\"{}\" origin=\"{}\" COBUnit={} start_track={} end_track={} result={} path_hops={} path=\"{}\"{}",
        prefix,
        record.record_id,
        record.net_name,
        record_origin_display_key(record),
        endpoint.cob_unit,
        start_text,
        end_text,
        result,
        path_hops,
        path_text,
        (reason != nullptr && reason[0] != '\0') ? std::format(" reason=\"{}\"", reason) : std::String {});
}

auto mark_failed_record(const MazeIlpFixedContext* ctx, const std::size_t rec_idx) -> void {
    if (ctx != nullptr && ctx->last_failed_record_index != nullptr) {
        *ctx->last_failed_record_index = rec_idx;
    }
}

[[noreturn]] auto fail_maze_record_route(
    const MazeIlpFixedContext* ctx,
    const std::size_t rec_idx,
    const Net_cost_record& record,
    const TobIlpRecordTrackEndpoint& endpoint,
    const std::String& reason,
    const char* end_track_label
) -> void {
    mark_failed_record(ctx, rec_idx);
    log_maze_record_route(
        ctx != nullptr ? ctx->log_prefix : nullptr,
        record,
        endpoint,
        "FAILED",
        reason.c_str(),
        0,
        "",
        end_track_label);
    throw algo::RetryExpt(reason);
}

auto log_maze_record_success(
    const MazeIlpFixedContext* ctx,
    const std::size_t rec_idx,
    const Net_cost_record& record,
    const TobIlpRecordTrackEndpoint& endpoint,
    const RoutedPath& path,
    const char* end_track_label
) -> void {
    if (ctx == nullptr || ctx->log_prefix == nullptr || !ctx->verbose_records) {
        return;
    }
    log_maze_record_route(
        ctx->log_prefix,
        record,
        endpoint,
        "OK",
        "",
        path.size(),
        routed_path_text(path),
        end_track_label);
    (void)rec_idx;
}

auto is_trivial_pnnet_path(
    const RoutedPath& path,
    hardware::Track* start,
    const std::HashSet<hardware::Track*>& end_targets
) -> bool {
    if (path.empty()) {
        return true;
    }
    auto* back = std::get<0>(path.back());
    if (back == nullptr || !end_targets.contains(back)) {
        return false;
    }
    if (count_connector_hops(path) > 0) {
        return false;
    }
    if (start != nullptr && back == start) {
        return false;
    }
    return true;
}

auto is_trivial_two_pin_path(
    const RoutedPath& path,
    hardware::Track* start,
    hardware::Track* end
) -> bool {
    if (path.empty() || start == nullptr || end == nullptr) {
        return true;
    }
    if (start == end) {
        return false;
    }
    return count_connector_hops(path) == 0;
}


auto track_from_mcf_graph_index(
    hardware::Interposer* interposer,
    const McfGlobalGraph& graph,
    const std::size_t cob_unit,
    const std::size_t track_index
) -> hardware::Track* {
    for (const auto& meta : graph.nodes) {
        if (meta.is_virtual || meta.unit != cob_unit || meta.track != track_index) {
            continue;
        }
        const auto tc = hardware::TrackCoord {
            static_cast<std::i64>(meta.track_row),
            static_cast<std::i64>(meta.track_col),
            meta.track_dir == 0 ? hardware::TrackDirection::Horizontal : hardware::TrackDirection::Vertical,
            track_index};
        const auto track = interposer->get_track(tc);
        if (track.has_value()) {
            return track.value();
        }
    }
    return nullptr;
}

auto collect_origin_mcf_seed_tracks(
    hardware::Interposer* interposer,
    const McfGlobalGraph& graph,
    const CobMcfFullResult& mcf_result,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const std::String& origin_uid,
    const std::set<std::size_t>& exclude_record_indices,
    const bool use_session_paths
) -> std::Vector<hardware::Track*> {
    auto out = std::Vector<hardware::Track*> {};
    if (records.size() != ilp_result.record_track_endpoints.size()) {
        return out;
    }
    for (std::size_t i = 0; i < records.size(); ++i) {
        if (exclude_record_indices.contains(i)) {
            continue;
        }
        const auto& record = records[i];
        if (record.type != Net_type::PNnet || record_origin_group_uid(record) != origin_uid) {
            continue;
        }
        const auto cob_unit = ilp_result.record_track_endpoints[i].cob_unit;
        if (cob_unit >= 16) {
            continue;
        }
        if (!use_session_paths) {
            const auto unit_ok = mcf_result.simple_mcf_ok[cob_unit];
            if (!unit_ok) {
                continue;
            }
        }
        for (const auto& info : mcf_result.paths_by_unit[cob_unit]) {
            if (info.record_id != record.record_id) {
                continue;
            }
            for (const auto& track_path : info.track_paths) {
                for (const auto track_index : track_path) {
                    auto* t = track_from_mcf_graph_index(interposer, graph, cob_unit, track_index);
                    if (t != nullptr) {
                        out.emplace_back(t);
                    }
                }
            }
            break;
        }
    }
    return out;
}

auto route_bump_to_bump_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::BumpToBumpNet* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    const auto rec_idx = find_record_for_btb(record_indices, records, net->begin_bump(), net->end_bump());
    if (!rec_idx.has_value()) {
        throw algo::RetryExpt("route_bump_to_bump_net_ilp_fixed: no matching SAT record");
    }
    const auto& record = records[*rec_idx];
    const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
    auto package = circuit::PathPackage {};
    package._regular_path = route_record_ilp_segment(interposer, record, endpoint, {});
    store_segment(segments_out, *rec_idx, package._regular_path);
    auto* begin_track = resolve_start_track(interposer, record, endpoint);
    auto* end_track = resolve_end_track(interposer, record, endpoint);
    attach_bump_to_track_tob(interposer, net->begin_bump(), begin_track, package);
    attach_track_to_bump_tob(interposer, net->end_bump(), end_track, package);
    package._length = package._regular_path.size() + 2;
    net->set_pathpackage(package);
    return package;
}

auto route_bump_to_track_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::BumpToTrackNet* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    const auto rec_idx = find_record_for_btt(record_indices, records, net->begin_bump(), net->end_track());
    if (!rec_idx.has_value()) {
        throw algo::RetryExpt("route_bump_to_track_net_ilp_fixed: no matching SAT record");
    }
    const auto& record = records[*rec_idx];
    const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
    auto occupied = std::HashSet<hardware::Track*> {};
    occupied.emplace(net->end_track());
    auto package = circuit::PathPackage {};
    package._regular_path = route_record_ilp_segment(interposer, record, endpoint, occupied);
    store_segment(segments_out, *rec_idx, package._regular_path);
    auto* begin_track = resolve_start_track(interposer, record, endpoint);
    attach_bump_to_track_tob(interposer, net->begin_bump(), begin_track, package);
    package._length = package._regular_path.size() + 1;
    net->set_pathpackage(package);
    return package;
}

auto route_track_to_bump_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::TrackToBumpNet* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    const auto rec_idx = find_record_for_ttb(record_indices, records, net->begin_track(), net->end_bump());
    if (!rec_idx.has_value()) {
        throw algo::RetryExpt("route_track_to_bump_net_ilp_fixed: no matching SAT record");
    }
    const auto& record = records[*rec_idx];
    const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
    auto occupied = std::HashSet<hardware::Track*> {};
    occupied.emplace(net->begin_track());
    auto package = circuit::PathPackage {};
    package._regular_path = route_record_ilp_segment(interposer, record, endpoint, occupied);
    store_segment(segments_out, *rec_idx, package._regular_path);
    auto* end_track = resolve_end_track(interposer, record, endpoint);
    attach_track_to_bump_tob(interposer, net->end_bump(), end_track, package);
    package._length = package._regular_path.size() + 1;
    net->set_pathpackage(package);
    return package;
}

auto route_bump_to_bumps_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::BumpToBumpsNet* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    auto begin_tracks_vec = std::Vector<hardware::Track*> {};
    auto total_regular_path = RoutedPath {};
    auto package = circuit::PathPackage {};
    std::size_t total_length {0};

    for (auto* end_bump : net->end_bumps()) {
        const auto rec_idx = find_record_for_btb(record_indices, records, net->begin_bump(), end_bump);
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_bump_to_bumps_net_ilp_fixed: no matching SAT record for end bump");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto* start = resolve_start_track(interposer, record, endpoint);
        auto* end = resolve_end_track(interposer, record, endpoint);
        if (begin_tracks_vec.empty()) {
            begin_tracks_vec.push_back(start);
        }
        const auto regular_path = route_single_ilp_segment(interposer, begin_tracks_vec.front(), end, {});
        store_segment(segments_out, *rec_idx, regular_path);
        total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());

        auto path = std::Vector<hardware::Track*> {};
        for (auto& [t, connector] : regular_path) {
            (void)connector;
            path.emplace_back(t);
        }
        attach_track_to_bump_tob(interposer, end_bump, end, package);
        begin_tracks_vec.insert(begin_tracks_vec.end(), path.begin(), path.end());
        total_length += path.size() + 1;
    }

    attach_bump_to_track_tob(interposer, net->begin_bump(), begin_tracks_vec.front(), package);
    package._regular_path = total_regular_path;
    package._length = total_length;
    net->set_pathpackage(package);
    return package;
}

auto route_bump_to_tracks_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::BumpToTracksNet* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    auto begin_tracks_vec = std::Vector<hardware::Track*> {};
    auto total_regular_path = RoutedPath {};
    auto package = circuit::PathPackage {};
    std::size_t total_length {0};

    for (auto* end_track : net->end_tracks()) {
        const auto rec_idx = find_record_for_btt(record_indices, records, net->begin_bump(), end_track);
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_bump_to_tracks_net_ilp_fixed: no matching SAT record for end track");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto* start = resolve_start_track(interposer, record, endpoint);
        auto occupied = std::HashSet<hardware::Track*> {};
        occupied.emplace(end_track);
        const auto regular_path = route_single_ilp_segment(interposer, start, end_track, occupied);
        store_segment(segments_out, *rec_idx, regular_path);
        total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());

        auto path = std::Vector<hardware::Track*> {};
        for (auto& [t, connector] : regular_path) {
            (void)connector;
            path.emplace_back(t);
        }
        attach_bump_to_track_tob(interposer, net->begin_bump(), path.front(), package);
        begin_tracks_vec.insert(begin_tracks_vec.end(), path.begin(), path.end());
        total_length += path.size() + 1;
    }

    package._regular_path = total_regular_path;
    package._length = total_length;
    net->set_pathpackage(package);
    return package;
}

auto route_track_to_bumps_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::TrackToBumpsNet* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const MazeIlpFixedContext* mcf_ctx,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    (void)net->begin_track();
    auto begin_tracks_vec = std::Vector<hardware::Track*> {};
    auto total_regular_path = RoutedPath {};
    auto package = circuit::PathPackage {};
    std::size_t total_length {0};

    for (auto* end_bump : net->end_bumps()) {
        const auto rec_idx = find_record_for_bump(record_indices, records, end_bump);
        if (!rec_idx.has_value()) {
            const auto reason = std::String {
                "route_track_to_bumps_net_ilp_fixed: no matching SAT record for end bump"};
            if (mcf_ctx != nullptr && mcf_ctx->log_prefix != nullptr) {
                debug::info_fmt(
                    "{} record_id=? net=\"?\" result=FAILED reason=\"{}\"",
                    mcf_ctx->log_prefix,
                    reason);
            }
            mark_failed_record(mcf_ctx, 0);
            throw algo::RetryExpt(reason);
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto* start = resolve_start_track(interposer, record, endpoint);
        auto* end = resolve_end_track(interposer, record, endpoint);
        if (start == nullptr) {
            fail_maze_record_route(
                mcf_ctx,
                *rec_idx,
                record,
                endpoint,
                std::format("route_track_to_bumps_net_ilp_fixed: null SAT start track for \"{}\"", record.net_name),
                nullptr);
        }
        if (end == nullptr) {
            fail_maze_record_route(
                mcf_ctx,
                *rec_idx,
                record,
                endpoint,
                std::format("route_track_to_bumps_net_ilp_fixed: null SAT end track for \"{}\"", record.net_name),
                nullptr);
        }

        auto begin_vec = dedupe_track_vector(begin_tracks_vec);
        begin_vec.insert(begin_vec.begin(), start);
        begin_vec = dedupe_track_vector(begin_vec);

        RoutedPath regular_path {};
        try {
            regular_path = ilp_fixed_route_path(interposer, begin_vec, track_set_from_ptr(end), {});
        }
        catch (const algo::RouteExpt& e) {
            fail_maze_record_route(
                mcf_ctx,
                *rec_idx,
                record,
                endpoint,
                e.what(),
                nullptr);
        }

        if (regular_path.empty()) {
            fail_maze_record_route(
                mcf_ctx,
                *rec_idx,
                record,
                endpoint,
                std::format("route_track_to_bumps_net_ilp_fixed: empty path for \"{}\"", record.net_name),
                nullptr);
        }
        auto* path_back = std::get<0>(regular_path.back());
        if (path_back != end) {
            fail_maze_record_route(
                mcf_ctx,
                *rec_idx,
                record,
                endpoint,
                "route_track_to_bumps_net_ilp_fixed: path does not end at SAT COB track",
                nullptr);
        }
        if (is_trivial_two_pin_path(regular_path, start, end)) {
            fail_maze_record_route(
                mcf_ctx,
                *rec_idx,
                record,
                endpoint,
                "route_track_to_bumps_net_ilp_fixed: trivial path (no COB hops)",
                nullptr);
        }

        store_segment(segments_out, *rec_idx, regular_path);
        total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());

        auto path = std::Vector<hardware::Track*> {};
        for (auto& [t, connector] : regular_path) {
            (void)connector;
            path.emplace_back(t);
        }
        attach_track_to_bump_tob(interposer, end_bump, start, package);
        for (auto* t : path) {
            begin_tracks_vec.emplace_back(t);
        }
        total_length += path.size() + 1;
        log_maze_record_success(mcf_ctx, *rec_idx, record, endpoint, regular_path, nullptr);
    }

    package._regular_path = total_regular_path;
    package._length = total_length + 1;
    net->set_pathpackage(package);
    return package;
}

auto route_tracks_to_bumps_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::TracksToBumpsNet* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const MazeIlpFixedContext* mcf_ctx,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    if (record_indices.empty()) {
        throw algo::RetryExpt("route_tracks_to_bumps_net_ilp_fixed: empty record_indices");
    }

    auto sorted_indices = record_indices;
    std::sort(sorted_indices.begin(), sorted_indices.end());

    const auto origin_uid = record_origin_group_uid(records[sorted_indices.front()]);

    auto begin_tracks_vec = std::Vector<hardware::Track*> {};
    const auto end_targets = track_set_from_begin_tracks(net);
    auto total_regular_path = RoutedPath {};
    auto package = circuit::PathPackage {};
    std::size_t total_length {0};

    for (const auto rec_idx : sorted_indices) {
        const auto& record = records[rec_idx];
        if (record.type != Net_type::PNnet) {
            fail_maze_record_route(
                mcf_ctx,
                rec_idx,
                record,
                ilp_result.record_track_endpoints[rec_idx],
                std::format(
                    "route_tracks_to_bumps_net_ilp_fixed: expected PNnet record for \"{}\"",
                    record.net_name),
                "any_0/1_port");
        }
        const auto& endpoint = ilp_result.record_track_endpoints[rec_idx];
        auto* bump = bump_from_record(interposer, record);
        auto* start = resolve_start_track(interposer, record, endpoint);
        if (start == nullptr) {
            fail_maze_record_route(
                mcf_ctx,
                rec_idx,
                record,
                endpoint,
                std::format("route_tracks_to_bumps_net_ilp_fixed: null SAT start track for \"{}\"", record.net_name),
                "any_0/1_port");
        }

        auto begin_vec = dedupe_track_vector(begin_tracks_vec);
        if (mcf_ctx != nullptr && mcf_ctx->graph != nullptr && mcf_ctx->mcf_result != nullptr) {
            const auto exclude_current = std::set<std::size_t> {rec_idx};
            const auto mcf_seeds = collect_origin_mcf_seed_tracks(
                interposer,
                *mcf_ctx->graph,
                *mcf_ctx->mcf_result,
                records,
                ilp_result,
                origin_uid,
                exclude_current,
                mcf_ctx->use_session_paths);
            begin_vec.insert(begin_vec.end(), mcf_seeds.begin(), mcf_seeds.end());
        }
        begin_vec.insert(begin_vec.begin(), start);
        begin_vec = dedupe_track_vector(begin_vec);

        RoutedPath regular_path {};
        try {
            regular_path = ilp_fixed_route_path(interposer, begin_vec, end_targets, {});
        }
        catch (const algo::RouteExpt& e) {
            fail_maze_record_route(mcf_ctx, rec_idx, record, endpoint, e.what(), "any_0/1_port");
        }

        store_segment(segments_out, rec_idx, regular_path);
        total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());

        auto path = std::Vector<hardware::Track*> {};
        path.reserve(regular_path.size());
        for (auto& [t, connector] : regular_path) {
            (void)connector;
            path.emplace_back(t);
        }
        if (path.empty()) {
            fail_maze_record_route(
                mcf_ctx,
                rec_idx,
                record,
                endpoint,
                std::format("route_tracks_to_bumps_net_ilp_fixed: empty path for \"{}\"", record.net_name),
                "any_0/1_port");
        }
        auto* end_track = path.back();
        if (!end_targets.contains(end_track)) {
            fail_maze_record_route(
                mcf_ctx,
                rec_idx,
                record,
                endpoint,
                "route_tracks_to_bumps_net_ilp_fixed: end track not in 0/1 port set",
                "any_0/1_port");
        }
        if (is_trivial_pnnet_path(regular_path, start, end_targets)) {
            fail_maze_record_route(
                mcf_ctx,
                rec_idx,
                record,
                endpoint,
                "route_tracks_to_bumps_net_ilp_fixed: trivial path (0/1 port hit without COB hops)",
                "any_0/1_port");
        }
        attach_track_to_bump_tob(interposer, bump, start, package);
        for (auto* t : path) {
            begin_tracks_vec.emplace_back(t);
        }
        total_length += path.size() + 1;
        log_maze_record_success(mcf_ctx, rec_idx, record, endpoint, regular_path, "any_0/1_port");
    }

    package._regular_path = total_regular_path;
    package._length = total_length;
    net->set_pathpackage(package);
    return package;
}

auto route_sync_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::SyncNet* sync_net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    auto occupied_tracks = std::HashSet<hardware::Track*> {};
    for (const auto& btt : sync_net->bttnets()) {
        occupied_tracks.emplace(btt->end_track());
    }
    for (const auto& ttb : sync_net->ttbnets()) {
        occupied_tracks.emplace(ttb->begin_track());
    }

    for (const auto& btb : sync_net->btbnets()) {
        const auto rec_idx = find_record_for_btb(record_indices, records, btb->begin_bump(), btb->end_bump());
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_sync_net_ilp_fixed: no matching BTB SAT record");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto package = circuit::PathPackage {};
        package._regular_path = route_record_ilp_segment(interposer, record, endpoint, occupied_tracks);
        store_segment(segments_out, *rec_idx, package._regular_path);
        auto* begin_track = resolve_start_track(interposer, record, endpoint);
        auto* end_track = resolve_end_track(interposer, record, endpoint);
        attach_bump_to_track_tob(interposer, btb->begin_bump(), begin_track, package);
        attach_track_to_bump_tob(interposer, btb->end_bump(), end_track, package);
        package._length = package._regular_path.size() + 2;
        btb->set_pathpackage(package);
    }

    for (const auto& ttb : sync_net->ttbnets()) {
        const auto rec_idx = find_record_for_ttb(record_indices, records, ttb->begin_track(), ttb->end_bump());
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_sync_net_ilp_fixed: no matching TTB SAT record");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto local_occupied = occupied_tracks;
        local_occupied.erase(ttb->begin_track());
        auto package = circuit::PathPackage {};
        package._regular_path = route_record_ilp_segment(interposer, record, endpoint, local_occupied);
        store_segment(segments_out, *rec_idx, package._regular_path);
        auto* end_track = resolve_end_track(interposer, record, endpoint);
        attach_track_to_bump_tob(interposer, ttb->end_bump(), end_track, package);
        package._length = package._regular_path.size() + 1;
        ttb->set_pathpackage(package);
    }

    for (const auto& btt : sync_net->bttnets()) {
        const auto rec_idx = find_record_for_btt(record_indices, records, btt->begin_bump(), btt->end_track());
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_sync_net_ilp_fixed: no matching BTT SAT record");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto local_occupied = occupied_tracks;
        local_occupied.erase(btt->end_track());
        auto package = circuit::PathPackage {};
        package._regular_path = route_record_ilp_segment(interposer, record, endpoint, local_occupied);
        store_segment(segments_out, *rec_idx, package._regular_path);
        auto* begin_track = resolve_start_track(interposer, record, endpoint);
        attach_bump_to_track_tob(interposer, btt->begin_bump(), begin_track, package);
        package._length = package._regular_path.size() + 1;
        btt->set_pathpackage(package);
    }

    if (!sync_net->collect_package()) {
        throw algo::RetryExpt("route_sync_net_ilp_fixed: collect_package failed");
    }
    return sync_net->pathpackage();
}

auto route_origin_net_ilp_fixed_impl(
    hardware::Interposer* interposer,
    circuit::Net* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const MazeIlpFixedContext* mcf_ctx,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    if (auto* bb = dynamic_cast<circuit::BumpToBumpNet*>(net)) {
        return route_bump_to_bump_net_ilp_fixed(interposer, bb, record_indices, records, ilp_result, segments_out);
    }
    if (auto* bt = dynamic_cast<circuit::BumpToTrackNet*>(net)) {
        return route_bump_to_track_net_ilp_fixed(interposer, bt, record_indices, records, ilp_result, segments_out);
    }
    if (auto* tb = dynamic_cast<circuit::TrackToBumpNet*>(net)) {
        return route_track_to_bump_net_ilp_fixed(interposer, tb, record_indices, records, ilp_result, segments_out);
    }
    if (auto* tbs = dynamic_cast<circuit::TrackToBumpsNet*>(net)) {
        return route_track_to_bumps_net_ilp_fixed(
            interposer, tbs, record_indices, records, ilp_result, mcf_ctx, segments_out);
    }
    if (auto* bbs = dynamic_cast<circuit::BumpToBumpsNet*>(net)) {
        return route_bump_to_bumps_net_ilp_fixed(interposer, bbs, record_indices, records, ilp_result, segments_out);
    }
    if (auto* bts = dynamic_cast<circuit::BumpToTracksNet*>(net)) {
        return route_bump_to_tracks_net_ilp_fixed(interposer, bts, record_indices, records, ilp_result, segments_out);
    }
    if (auto* tsbs = dynamic_cast<circuit::TracksToBumpsNet*>(net)) {
        return route_tracks_to_bumps_net_ilp_fixed(
            interposer, tsbs, record_indices, records, ilp_result, mcf_ctx, segments_out);
    }
    if (auto* sync = dynamic_cast<circuit::SyncNet*>(net)) {
        return route_sync_net_ilp_fixed(interposer, sync, record_indices, records, ilp_result, segments_out);
    }
    throw algo::RetryExpt(std::format("route_origin_net_ilp_fixed: unsupported net type \"{}\"", net->name()));
}

} // namespace

auto route_origin_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::Net* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const MazeIlpFixedContext* mcf_ctx,
    OriginRouteSegments* segments_out
) -> circuit::PathPackage {
    return route_origin_net_ilp_fixed_impl(
        interposer, net, record_indices, records, ilp_result, mcf_ctx, segments_out);
}

auto track_coord_text(const hardware::TrackCoord& tc) -> std::String {
    const auto dir = tc.dir == hardware::TrackDirection::Horizontal ? "H" : "V";
    return std::format("({},{},{},idx={})", tc.row, tc.col, dir, tc.index);
}

auto pathpackage_regular_path_text(const circuit::PathPackage& package) -> std::String {
    if (package._regular_path.empty()) {
        return "(empty regular_path)";
    }
    auto parts = std::Vector<std::String> {};
    parts.reserve(package._regular_path.size());
    for (const auto& hop : package._regular_path) {
        const auto* track = std::get<0>(hop);
        if (track == nullptr) {
            parts.emplace_back("(null-track)");
            continue;
        }
        parts.emplace_back(track_coord_text(track->coord()));
    }
    auto text = std::String {};
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (i != 0) {
            text += " -> ";
        }
        text += parts[i];
    }
    return text;
}

auto segments_path_text(const OriginRouteSegments& segments) -> std::String {
    if (segments.by_record_index.empty()) {
        return "(empty segments)";
    }
    auto parts = std::Vector<std::String> {};
    parts.reserve(segments.by_record_index.size());
    for (const auto& [rec_idx, path] : segments.by_record_index) {
        auto hop_parts = std::Vector<std::String> {};
        hop_parts.reserve(path.size());
        for (const auto& hop : path) {
            const auto* track = std::get<0>(hop);
            if (track == nullptr) {
                hop_parts.emplace_back("(null-track)");
                continue;
            }
            hop_parts.emplace_back(track_coord_text(track->coord()));
        }
        auto hop_text = std::String {};
        for (std::size_t i = 0; i < hop_parts.size(); ++i) {
            if (i != 0) {
                hop_text += " -> ";
            }
            hop_text += hop_parts[i];
        }
        parts.emplace_back(std::format("[rec={}] {}", rec_idx, hop_text));
    }
    auto text = std::String {};
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (i != 0) {
            text += " | ";
        }
        text += parts[i];
    }
    return text;
}


} // namespace PR_tool
