#include "maze_check/maze_check.hh"

#include "common/ilp_types.hh"
#include "ilp_allocation/ilp_apply_interposer.hh"
#include "mcf/mcf_graph.hh"

#include <algo/router/common/maze/mazererouter.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/routeerror.hh>
#include <circuit/basedie.hh>
#include <circuit/net/net.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/bbsnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/btsnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <circuit/path/pathpackage.hh>
#include <debug/debug.hh>
#include <hardware/bump/bump.hh>
#include <hardware/cob/cobconnector.hh>
#include <hardware/interposer.hh>
#include <hardware/tob/tob.hh>
#include <hardware/tob/tobconnector.hh>
#include <hardware/track/track.hh>
#include <hardware/track/trackcoord.hh>

#include <chrono>
#include <format>
#include <map>
#include <optional>
#include <set>
#include <tuple>

namespace PR_tool {
namespace {

using RoutedPath = algo::routed_path;

enum class MazeOriginOutcome {
    Pending,
    Ok,
    Failed,
    Skipped
};

struct FailedRecordRef {
    std::size_t record_index{0};
    std::size_t record_id{0};
    std::String net_name;
    std::String origin_key;
    std::size_t cob_unit{0};
};

struct OriginMazeState {
    MazeOriginOutcome outcome{MazeOriginOutcome::Pending};
    std::String message;
    std::String path_text;
    std::size_t path_hops{0};
    std::size_t representative_cob_unit{0};
    std::Vector<std::size_t> record_indices;
};

struct MazeCheckContext {
    std::String log_prefix;
    std::String timing_phase;
    std::Vector<FailedRecordRef> failed_records;
    std::map<std::String, OriginMazeState> origin_states;
    MazeCheckSummary summary;
};

auto record_origin_key(const Net_cost_record& record) -> std::String {
    return record.origin_key.empty() ? record.net_name : record.origin_key;
}

auto is_simple_mcf_record(const Net_cost_record& record) -> bool {
    return !is_sync_bus_mcf_origin_key(record_origin_key(record));
}

auto track_coord_text(const hardware::TrackCoord& tc) -> std::String {
    const auto dir = tc.dir == hardware::TrackDirection::Horizontal ? "H" : "V";
    return std::format("({},{},{},idx={})", tc.row, tc.col, dir, tc.index);
}

auto pathpackage_to_text(const circuit::PathPackage& package) -> std::String {
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

auto collect_failed_simple_records(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const std::String& log_prefix
) -> std::Vector<FailedRecordRef> {
    auto out = std::Vector<FailedRecordRef> {};
    if (records.size() != ilp_result.record_track_endpoints.size()) {
        debug::warning_fmt("{}: record/ilp endpoint size mismatch; skip failed-record collection", log_prefix);
        return out;
    }
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto& record = records[i];
        if (!is_simple_mcf_record(record)) {
            continue;
        }
        const auto cob_unit = ilp_result.record_track_endpoints[i].cob_unit;
        if (cob_unit >= 16) {
            continue;
        }
        if (!mcf_result.has_simple_commodities[cob_unit]) {
            continue;
        }
        if (mcf_result.simple_mcf_ok[cob_unit]) {
            continue;
        }
        out.push_back(FailedRecordRef {
            i,
            record.record_id,
            record.net_name,
            record_origin_key(record),
            cob_unit});
    }
    return out;
}

auto find_net_by_name(circuit::BaseDie* basedie, const std::String& name) -> circuit::Net* {
    if (basedie == nullptr) {
        return nullptr;
    }
    for (const auto& net : basedie->nets_to_vector()) {
        if (net->name() == name) {
            return net.get();
        }
    }
    return nullptr;
}

auto outcome_label(const MazeOriginOutcome outcome) -> const char* {
    switch (outcome) {
        case MazeOriginOutcome::Ok:
            return "OK";
        case MazeOriginOutcome::Failed:
            return "FAILED";
        case MazeOriginOutcome::Skipped:
            return "SKIP";
        case MazeOriginOutcome::Pending:
        default:
            return "PENDING";
    }
}

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

auto route_bump_to_bump_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::BumpToBumpNet* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result
) -> circuit::PathPackage {
    const auto rec_idx = find_record_for_btb(record_indices, records, net->begin_bump(), net->end_bump());
    if (!rec_idx.has_value()) {
        throw algo::RetryExpt("route_bump_to_bump_net_ilp_fixed: no matching ILP record");
    }
    const auto& record = records[*rec_idx];
    const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
    auto package = circuit::PathPackage {};
    package._regular_path = route_record_ilp_segment(interposer, record, endpoint, {});
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
    const TobIlpResult& ilp_result
) -> circuit::PathPackage {
    const auto rec_idx = find_record_for_btt(record_indices, records, net->begin_bump(), net->end_track());
    if (!rec_idx.has_value()) {
        throw algo::RetryExpt("route_bump_to_track_net_ilp_fixed: no matching ILP record");
    }
    const auto& record = records[*rec_idx];
    const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
    auto occupied = std::HashSet<hardware::Track*> {};
    occupied.emplace(net->end_track());
    auto package = circuit::PathPackage {};
    package._regular_path = route_record_ilp_segment(interposer, record, endpoint, occupied);
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
    const TobIlpResult& ilp_result
) -> circuit::PathPackage {
    const auto rec_idx = find_record_for_ttb(record_indices, records, net->begin_track(), net->end_bump());
    if (!rec_idx.has_value()) {
        throw algo::RetryExpt("route_track_to_bump_net_ilp_fixed: no matching ILP record");
    }
    const auto& record = records[*rec_idx];
    const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
    auto occupied = std::HashSet<hardware::Track*> {};
    occupied.emplace(net->begin_track());
    auto package = circuit::PathPackage {};
    package._regular_path = route_record_ilp_segment(interposer, record, endpoint, occupied);
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
    const TobIlpResult& ilp_result
) -> circuit::PathPackage {
    auto begin_tracks_vec = std::Vector<hardware::Track*> {};
    auto total_regular_path = RoutedPath {};
    auto package = circuit::PathPackage {};
    std::size_t total_length {0};

    for (auto* end_bump : net->end_bumps()) {
        const auto rec_idx = find_record_for_btb(record_indices, records, net->begin_bump(), end_bump);
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_bump_to_bumps_net_ilp_fixed: no matching ILP record for end bump");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto* start = resolve_start_track(interposer, record, endpoint);
        auto* end = resolve_end_track(interposer, record, endpoint);
        if (begin_tracks_vec.empty()) {
            begin_tracks_vec.push_back(start);
        }
        const auto regular_path = route_single_ilp_segment(interposer, begin_tracks_vec.front(), end, {});
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
    const TobIlpResult& ilp_result
) -> circuit::PathPackage {
    auto begin_tracks_vec = std::Vector<hardware::Track*> {};
    auto total_regular_path = RoutedPath {};
    auto package = circuit::PathPackage {};
    std::size_t total_length {0};

    for (auto* end_track : net->end_tracks()) {
        const auto rec_idx = find_record_for_btt(record_indices, records, net->begin_bump(), end_track);
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_bump_to_tracks_net_ilp_fixed: no matching ILP record for end track");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto* start = resolve_start_track(interposer, record, endpoint);
        auto occupied = std::HashSet<hardware::Track*> {};
        occupied.emplace(end_track);
        const auto regular_path = route_single_ilp_segment(interposer, start, end_track, occupied);
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
    const TobIlpResult& ilp_result
) -> circuit::PathPackage {
    auto begin_tracks_vec = std::Vector<hardware::Track*> {net->begin_track()};
    auto total_regular_path = RoutedPath {};
    auto package = circuit::PathPackage {};
    std::size_t total_length {0};

    for (auto* end_bump : net->end_bumps()) {
        const auto rec_idx = find_record_for_bump(record_indices, records, end_bump);
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_track_to_bumps_net_ilp_fixed: no matching ILP record for end bump");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto* end = resolve_end_track(interposer, record, endpoint);
        const auto regular_path = route_single_ilp_segment(interposer, begin_tracks_vec.front(), end, {});
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
    const TobIlpResult& ilp_result
) -> circuit::PathPackage {
    auto begin_tracks_vec = std::Vector<hardware::Track*> {};
    for (auto* t : net->begin_tracks()) {
        begin_tracks_vec.emplace_back(t);
    }
    auto total_regular_path = RoutedPath {};
    auto package = circuit::PathPackage {};
    std::size_t total_length {0};

    for (auto* end_bump : net->end_bumps()) {
        const auto rec_idx = find_record_for_bump(record_indices, records, end_bump);
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_tracks_to_bumps_net_ilp_fixed: no matching ILP record for end bump");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto* start = resolve_start_track(interposer, record, endpoint);
        auto* end = resolve_end_track(interposer, record, endpoint);
        auto begin_vec = begin_tracks_vec;
        if (start != nullptr) {
            begin_vec.insert(begin_vec.begin(), start);
        }
        const auto regular_path = route_single_ilp_segment(interposer, begin_vec.front(), end, {});
        total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());

        auto path = std::Vector<hardware::Track*> {};
        for (auto& [t, connector] : regular_path) {
            (void)connector;
            path.emplace_back(t);
        }
        attach_track_to_bump_tob(interposer, end_bump, end, package);
        for (auto* t : path) {
            begin_tracks_vec.emplace_back(t);
        }
        total_length += path.size() + 1;
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
    const TobIlpResult& ilp_result
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
            throw algo::RetryExpt("route_sync_net_ilp_fixed: no matching BTB ILP record");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto package = circuit::PathPackage {};
        package._regular_path = route_record_ilp_segment(interposer, record, endpoint, occupied_tracks);
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
            throw algo::RetryExpt("route_sync_net_ilp_fixed: no matching TTB ILP record");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto local_occupied = occupied_tracks;
        local_occupied.erase(ttb->begin_track());
        auto package = circuit::PathPackage {};
        package._regular_path = route_record_ilp_segment(interposer, record, endpoint, local_occupied);
        auto* end_track = resolve_end_track(interposer, record, endpoint);
        attach_track_to_bump_tob(interposer, ttb->end_bump(), end_track, package);
        package._length = package._regular_path.size() + 1;
        ttb->set_pathpackage(package);
    }

    for (const auto& btt : sync_net->bttnets()) {
        const auto rec_idx = find_record_for_btt(record_indices, records, btt->begin_bump(), btt->end_track());
        if (!rec_idx.has_value()) {
            throw algo::RetryExpt("route_sync_net_ilp_fixed: no matching BTT ILP record");
        }
        const auto& record = records[*rec_idx];
        const auto& endpoint = ilp_result.record_track_endpoints[*rec_idx];
        auto local_occupied = occupied_tracks;
        local_occupied.erase(btt->end_track());
        auto package = circuit::PathPackage {};
        package._regular_path = route_record_ilp_segment(interposer, record, endpoint, local_occupied);
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

auto route_origin_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::Net* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result
) -> circuit::PathPackage {
    if (auto* bb = dynamic_cast<circuit::BumpToBumpNet*>(net)) {
        return route_bump_to_bump_net_ilp_fixed(interposer, bb, record_indices, records, ilp_result);
    }
    if (auto* bt = dynamic_cast<circuit::BumpToTrackNet*>(net)) {
        return route_bump_to_track_net_ilp_fixed(interposer, bt, record_indices, records, ilp_result);
    }
    if (auto* tb = dynamic_cast<circuit::TrackToBumpNet*>(net)) {
        return route_track_to_bump_net_ilp_fixed(interposer, tb, record_indices, records, ilp_result);
    }
    if (auto* tbs = dynamic_cast<circuit::TrackToBumpsNet*>(net)) {
        return route_track_to_bumps_net_ilp_fixed(interposer, tbs, record_indices, records, ilp_result);
    }
    if (auto* bbs = dynamic_cast<circuit::BumpToBumpsNet*>(net)) {
        return route_bump_to_bumps_net_ilp_fixed(interposer, bbs, record_indices, records, ilp_result);
    }
    if (auto* bts = dynamic_cast<circuit::BumpToTracksNet*>(net)) {
        return route_bump_to_tracks_net_ilp_fixed(interposer, bts, record_indices, records, ilp_result);
    }
    if (auto* tsbs = dynamic_cast<circuit::TracksToBumpsNet*>(net)) {
        return route_tracks_to_bumps_net_ilp_fixed(interposer, tsbs, record_indices, records, ilp_result);
    }
    if (auto* sync = dynamic_cast<circuit::SyncNet*>(net)) {
        return route_sync_net_ilp_fixed(interposer, sync, record_indices, records, ilp_result);
    }
    throw algo::RetryExpt(std::format("route_origin_net_ilp_fixed: unsupported net type \"{}\"", net->name()));
}

auto prepare_maze_check_context(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const std::String& log_prefix,
    const std::String& timing_phase
) -> MazeCheckContext {
    auto ctx = MazeCheckContext {};
    ctx.log_prefix = log_prefix;
    ctx.timing_phase = timing_phase;
    ctx.failed_records = collect_failed_simple_records(records, ilp_result, mcf_result, log_prefix);
    ctx.summary.failed_records = static_cast<int>(ctx.failed_records.size());

    auto failed_units = std::set<std::size_t> {};
    for (const auto& ref : ctx.failed_records) {
        failed_units.insert(ref.cob_unit);
    }
    ctx.summary.failed_units = static_cast<int>(failed_units.size());

    for (const auto& ref : ctx.failed_records) {
        auto& state = ctx.origin_states[ref.origin_key];
        if (state.record_indices.empty()) {
            state.representative_cob_unit = ref.cob_unit;
        }
        state.record_indices.push_back(ref.record_index);
    }

    return ctx;
}

auto log_maze_check_summary(const MazeCheckContext& ctx) -> void {
    debug::info_fmt(
        "{} summary: origins_routed={} ok={} failed={} skipped={}",
        ctx.log_prefix,
        ctx.summary.unique_origins_routed,
        ctx.summary.maze_ok,
        ctx.summary.maze_failed,
        ctx.summary.maze_skipped);
}

auto log_record_shared_results(const MazeCheckContext& ctx) -> void {
    for (const auto& ref : ctx.failed_records) {
        const auto it = ctx.origin_states.find(ref.origin_key);
        if (it == ctx.origin_states.end()) {
            continue;
        }
        debug::info_fmt(
            "{} record_id={} net=\"{}\" origin=\"{}\" COBUnit={} maze={} (shared origin result)",
            ctx.log_prefix,
            ref.record_id,
            ref.net_name,
            ref.origin_key,
            ref.cob_unit,
            outcome_label(it->second.outcome));
    }
}

auto run_maze_check_loop(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfGridDims cob_grid,
    const CobMcfFullResult& mcf_result,
    MazeCheckContext& ctx,
    const auto& route_origin
) -> void {
    if (ctx.failed_records.empty()) {
        debug::info_fmt("{}: no failed SimpleMCF units, skip", ctx.log_prefix);
        return;
    }

    auto failed_units = std::set<std::size_t> {};
    for (const auto& ref : ctx.failed_records) {
        failed_units.insert(ref.cob_unit);
    }
    auto failed_units_text = std::String {};
    for (const auto u : failed_units) {
        if (!failed_units_text.empty()) {
            failed_units_text += ",";
        }
        failed_units_text += std::to_string(u);
    }

    debug::info_fmt(
        "{}: failed SimpleMCF units=[{}] failed_records={} unique_origins={}",
        ctx.log_prefix,
        failed_units_text,
        ctx.summary.failed_records,
        ctx.origin_states.size());

    if (interposer == nullptr || basedie == nullptr) {
        debug::error_fmt("{}: interposer or basedie is null; cannot run maze routing", ctx.log_prefix);
        for (auto& [origin, state] : ctx.origin_states) {
            (void)origin;
            state.outcome = MazeOriginOutcome::Skipped;
            state.message = "null interposer or basedie";
            ctx.summary.maze_skipped += 1;
        }
        return;
    }

    apply_tob_ilp_result_to_interposer(interposer, ilp_result);
    const auto graph = build_mcf_track_graph(cob_grid);
    suspend_mcf_paths_on_interposer(interposer, graph, mcf_result.paths_by_unit);

    auto routed_nets = std::Vector<circuit::Net*> {};

    for (auto& [origin, state] : ctx.origin_states) {
        auto* net = find_net_by_name(basedie, origin);
        if (net == nullptr) {
            state.outcome = MazeOriginOutcome::Skipped;
            state.message = "net not found in basedie";
            ctx.summary.maze_skipped += 1;
            debug::info_fmt(
                "{} origin=\"{}\" COBUnit={} records={} result=SKIP reason={}",
                ctx.log_prefix,
                origin,
                state.representative_cob_unit,
                state.record_indices.size(),
                state.message);
            continue;
        }

        try {
            net->check_accessable_cobunit();
            interposer->manage_cobunit_resources();
            net->search_related_nets(routed_nets);
            const auto package = route_origin(interposer, net, state.record_indices, records, ilp_result);
            routed_nets.push_back(net);

            state.outcome = MazeOriginOutcome::Ok;
            state.path_text = pathpackage_to_text(package);
            state.path_hops = package._regular_path.size();
            ctx.summary.maze_ok += 1;
            ctx.summary.unique_origins_routed += 1;

            debug::info_fmt(
                "{} origin=\"{}\" COBUnit={} records={} result=OK path_len={} path=\"{}\"",
                ctx.log_prefix,
                origin,
                state.representative_cob_unit,
                state.record_indices.size(),
                state.path_hops,
                state.path_text);
        }
        catch (const algo::RouteExpt& e) {
            state.outcome = MazeOriginOutcome::Failed;
            state.message = e.what();
            ctx.summary.maze_failed += 1;
            ctx.summary.unique_origins_routed += 1;
            debug::info_fmt(
                "{} origin=\"{}\" COBUnit={} records={} result=FAILED reason=\"{}\"",
                ctx.log_prefix,
                origin,
                state.representative_cob_unit,
                state.record_indices.size(),
                state.message);
        }
        catch (const std::exception& e) {
            state.outcome = MazeOriginOutcome::Failed;
            state.message = e.what();
            ctx.summary.maze_failed += 1;
            ctx.summary.unique_origins_routed += 1;
            debug::info_fmt(
                "{} origin=\"{}\" COBUnit={} records={} result=FAILED reason=\"{}\"",
                ctx.log_prefix,
                origin,
                state.representative_cob_unit,
                state.record_indices.size(),
                state.message);
        }
    }
}

auto run_maze_check_impl(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const CobMcfGridDims cob_grid,
    const std::String& log_prefix,
    const std::String& timing_phase,
    const auto& route_origin
) -> MazeCheckSummary {
    const auto t0 = std::chrono::steady_clock::now();
    auto ctx = prepare_maze_check_context(records, ilp_result, mcf_result, log_prefix, timing_phase);
    run_maze_check_loop(interposer, basedie, records, ilp_result, cob_grid, mcf_result, ctx, route_origin);
    log_record_shared_results(ctx);
    log_maze_check_summary(ctx);
    const auto t1 = std::chrono::steady_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    debug::info_fmt("timing phase={} ms={}", timing_phase, ms);
    return ctx.summary;
}

} // namespace

auto run_maze_check_ilp_mcf_after_mcf(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const CobMcfGridDims cob_grid
) -> MazeCheckSummary {
    const auto maze = algo::MazeRouteStrategy {};
    return run_maze_check_impl(
        interposer,
        basedie,
        records,
        ilp_result,
        mcf_result,
        cob_grid,
        "maze-check-ilp-mcf",
        "maze_check_ilp_mcf",
        [&](hardware::Interposer* ip,
            circuit::Net* net,
            const std::Vector<std::size_t>&,
            const std::Vector<Net_cost_record>&,
            const TobIlpResult&) {
            net->route(ip, maze);
            return net->pathpackage();
        });
}

auto run_maze_check_mcf_after_mcf(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const CobMcfGridDims cob_grid
) -> MazeCheckSummary {
    return run_maze_check_impl(
        interposer,
        basedie,
        records,
        ilp_result,
        mcf_result,
        cob_grid,
        "maze-check-mcf",
        "maze_check_mcf",
        route_origin_net_ilp_fixed);
}

} // namespace PR_tool
