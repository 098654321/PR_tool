#include "precompute/tob_reach_with_range.hh"

#include "ilp_allocation/ilp_speedup.hh"
#include "precompute/ilp_bounding_box.hh"
#include "precompute/tob_channel_kshortest.hh"

#include <debug/debug.hh>
#include <hardware/track/trackcoord.hh>

#include <algorithm>
#include <format>
#include <string_view>

namespace PR_tool {

namespace {

auto net_type_label(const Net_type type) -> std::string_view {
    switch (type) {
        case Net_type::Bnet:
            return "Bnet";
        case Net_type::Tnet:
            return "Tnet";
        case Net_type::PNnet:
            return "PNnet";
    }
    return "Unknown";
}

auto format_bbox(const IlpBoundingBox& bbox) -> std::String {
    return std::format(
        "rows=[{},{}] cols=[{},{}]",
        bbox.row_min,
        bbox.row_max,
        bbox.col_min,
        bbox.col_max);
}

auto format_track_set(const std::Vector<std::size_t>& tracks) -> std::String {
    if (tracks.empty()) {
        return "{}";
    }
    auto out = std::String {"{"};
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        if (i != 0) {
            out += ", ";
        }
        out += std::format("{}", tracks[i]);
    }
    out += "}";
    return out;
}

auto same_cobunit_tracks_for_end_track(const std::size_t end_track) -> std::Vector<std::size_t> {
    return cobunit_to_tracks(map_track(end_track));
}

auto merge_extra_start_tracks_without_reach(
    std::Vector<std::size_t>& starts,
    std::map<std::size_t, std::Vector<IlpReachStep>>& reaches,
    const std::Vector<std::size_t>& extra
) -> void {
    for (const auto t : extra) {
        if (!reaches.contains(t)) {
            starts.push_back(t);
            reaches[t] = {};
        }
    }
    std::sort(starts.begin(), starts.end());
    starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
}

auto merge_nearest_new_start_tracks_without_reach(
    std::Vector<std::size_t>& starts,
    std::map<std::size_t, std::Vector<IlpReachStep>>& reaches,
    const std::Vector<std::size_t>& extra,
    const std::size_t limit
) -> std::size_t {
    auto added = std::size_t {0};
    for (const auto t : extra) {
        if (reaches.contains(t)) {
            continue;
        }
        starts.push_back(t);
        reaches[t] = {};
        ++added;
        if (added >= limit) {
            break;
        }
    }
    std::sort(starts.begin(), starts.end());
    starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
    return added;
}

auto end_track_coord(const Net_cost_record& record, std::size_t end_track) -> std::Option<hardware::TrackCoord> {
    if (record.type == Net_type::Tnet) {
        if (record.mcf_has_end_track && record.mcf_end_track.index == end_track) {
            return record.mcf_end_track;
        }
        if (record.mcf_has_start_track && record.mcf_start_track.index == end_track) {
            return record.mcf_start_track;
        }
        return std::nullopt;
    }
    if (record.type == Net_type::PNnet) {
        const auto it = record.pn_end_track_coord_by_index.find(end_track);
        if (it != record.pn_end_track_coord_by_index.end()) {
            return it->second;
        }
        return std::nullopt;
    }
    return std::nullopt;
}

auto candidate_start_tracks_for_bbox(
    const Net_cost_record& record,
    const std::size_t end_track,
    const IlpBoundingBox& bbox
) -> std::Vector<std::size_t> {
    constexpr auto kAllCandidateTracks = std::size_t {128};
    const auto start_tob = record.start_bumps.front().TOB;
    if (record.type == Net_type::Bnet) {
        if (record.end_bumps.empty()) {
            return {};
        }
        const auto end_tob = record.end_bumps.front().TOB;
        const auto end_channel = tob_channel_track_coords(end_tob);
        if (end_track >= end_channel.size()) {
            return {};
        }
        return kshortest_reachable_tob_tracks(end_channel[end_track], start_tob, bbox, kAllCandidateTracks);
    }

    const auto end_coord_opt = end_track_coord(record, end_track);
    if (!end_coord_opt.has_value()) {
        return {};
    }
    return kshortest_reachable_tob_tracks(*end_coord_opt, start_tob, bbox, kAllCandidateTracks);
}

} // namespace

auto precompute_reach_for_bbox_state(
    std::Vector<Net_cost_record>& records,
    const TobBBoxExpansionState& state
) -> TobReachRangeStats {
    const auto base_stats = precompute_reach_for_records(records);
    auto stats = TobReachRangeStats {};
    stats.range_level = state.max_rho();
    stats.total_records = base_stats.total_records;
    stats.total_endtracks = base_stats.total_endtracks;

    if (state.max_rho() == 0) {
        stats.total_starttrack_edges = base_stats.total_starttrack_edges;
        return stats;
    }

    for (std::size_t record_index = 0; record_index < records.size(); ++record_index) {
        auto& record = records[record_index];
        const auto rho = state.rho_for_record(record_index);
        if (rho == 0) {
            continue;
        }
        if (record.start_bumps.empty()) {
            continue;
        }

        for (const auto end_track : record.end_tracks) {
            auto& starts = record.starttrack_by_endtrack[end_track];
            auto& reaches = record.reach_by_end_start[end_track];

            const auto nearest_levels = std::min(rho, kTobReachMaxRangeLevel - 1);
            for (std::size_t level = 1; level <= nearest_levels; ++level) {
                const auto bbox = compute_bounding_box(record, level);
                const auto extra = candidate_start_tracks_for_bbox(record, end_track, bbox);
                (void)merge_nearest_new_start_tracks_without_reach(starts, reaches, extra, 2);
            }

            if (rho >= kTobReachMaxRangeLevel) {
                merge_extra_start_tracks_without_reach(
                    starts,
                    reaches,
                    same_cobunit_tracks_for_end_track(end_track));
            }
        }
    }

    for (const auto& record : records) {
        for (const auto& [end_track, starts] : record.starttrack_by_endtrack) {
            (void)end_track;
            stats.total_starttrack_edges += starts.size();
        }
    }
    return stats;
}

auto precompute_reach_for_range(std::Vector<Net_cost_record>& records, const std::size_t range_level) -> TobReachRangeStats {
    return precompute_reach_for_bbox_state(records, TobBBoxExpansionState::uniform(records.size(), range_level));
}

auto log_reach_endpoints_for_bbox_state(
    const std::Vector<Net_cost_record>& records,
    const TobBBoxExpansionState& state
) -> void {
    debug::info_fmt(
        "SAT reach/bbox endpoints: max_rho={} records={}",
        state.max_rho(),
        records.size());
    for (std::size_t record_index = 0; record_index < records.size(); ++record_index) {
        const auto& record = records[record_index];
        const auto rho = state.rho_for_record(record_index);
        const auto bbox = compute_bounding_box(record, rho);
        const auto origin = record.origin_key.empty() ? record.net_name : record.origin_key;
        const auto bus_tag = is_sync_bus_record(record) ? "sync_bus=yes" : "sync_bus=no";
        debug::info_fmt(
            "  record_id={} bit_id={} type={} net=\"{}\" origin=\"{}\" {} rho={} bbox={}",
            record.record_id,
            record.bit_id,
            net_type_label(record.type),
            record.net_name,
            origin,
            bus_tag,
            rho,
            format_bbox(bbox));

        if (record.end_tracks.empty()) {
            debug::info("    (no end_tracks)");
            continue;
        }

        for (const auto end_track : record.end_tracks) {
            const auto it = record.starttrack_by_endtrack.find(end_track);
            if (it == record.starttrack_by_endtrack.end()) {
                debug::info_fmt("    end_track={:>3}  start_tracks={}", end_track, "{}");
                continue;
            }
            debug::info_fmt(
                "    end_track={:>3}  start_tracks={} (|S|={})",
                end_track,
                format_track_set(it->second),
                it->second.size());
        }
    }
}

auto log_reach_endpoints_for_range(
    const std::Vector<Net_cost_record>& records,
    const std::size_t range_level
) -> void {
    log_reach_endpoints_for_bbox_state(records, TobBBoxExpansionState::uniform(records.size(), range_level));
}

} // namespace PR_tool
