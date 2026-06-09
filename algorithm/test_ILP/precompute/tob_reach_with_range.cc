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

} // namespace

auto precompute_reach_for_range(std::Vector<Net_cost_record>& records, const std::size_t range_level) -> TobReachRangeStats {
    const auto base_stats = precompute_reach_for_records(records);
    auto stats = TobReachRangeStats {};
    stats.range_level = range_level;
    stats.total_records = base_stats.total_records;
    stats.total_endtracks = base_stats.total_endtracks;

    if (range_level == 0) {
        stats.total_starttrack_edges = base_stats.total_starttrack_edges;
        return stats;
    }

    const bool use_same_cobunit_tracks = range_level == kTobReachMaxRangeLevel;

    for (auto& record : records) {
        if (record.start_bumps.empty()) {
            continue;
        }
        const auto bbox = compute_bounding_box(record, range_level);
        const auto k = 1 + 2 * range_level;
        const auto start_tob = record.start_bumps.front().TOB;

        for (const auto end_track : record.end_tracks) {
            auto& starts = record.starttrack_by_endtrack[end_track];
            auto& reaches = record.reach_by_end_start[end_track];

            if (use_same_cobunit_tracks) {
                merge_extra_start_tracks_without_reach(
                    starts,
                    reaches,
                    same_cobunit_tracks_for_end_track(end_track));
                continue;
            }

            if (record.type == Net_type::Bnet) {
                if (record.end_bumps.empty()) {
                    continue;
                }
                const auto end_tob = record.end_bumps.front().TOB;
                const auto end_channel = tob_channel_track_coords(end_tob);
                if (end_track >= end_channel.size()) {
                    continue;
                }
                const auto extra = kshortest_reachable_tob_tracks(
                    end_channel[end_track],
                    start_tob,
                    bbox,
                    k);
                merge_extra_start_tracks_without_reach(starts, reaches, extra);
            }
            else {
                const auto end_coord_opt = end_track_coord(record, end_track);
                if (!end_coord_opt.has_value()) {
                    continue;
                }
                const auto extra = kshortest_reachable_tob_tracks(*end_coord_opt, start_tob, bbox, k);
                merge_extra_start_tracks_without_reach(starts, reaches, extra);
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

auto log_reach_endpoints_for_range(
    const std::Vector<Net_cost_record>& records,
    const std::size_t range_level
) -> void {
    debug::info_fmt("SAT reach/bbox endpoints: range_level={} records={}", range_level, records.size());
    for (const auto& record : records) {
        const auto bbox = compute_bounding_box(record, range_level);
        const auto origin = record.origin_key.empty() ? record.net_name : record.origin_key;
        const auto bus_tag = is_sync_bus_record(record) ? "sync_bus=yes" : "sync_bus=no";
        debug::info_fmt(
            "  record_id={} bit_id={} type={} net=\"{}\" origin=\"{}\" {} bbox={}",
            record.record_id,
            record.bit_id,
            net_type_label(record.type),
            record.net_name,
            origin,
            bus_tag,
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

} // namespace PR_tool
