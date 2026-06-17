#include "precompute/tob_reach_with_range.hh"

#include "precompute/ilp_bounding_box.hh"

#include <debug/debug.hh>
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

auto format_bbox_corners(const IlpBoundingBox& bbox) -> std::String {
    return std::format(
        "({},{}) ({},{}) ({},{}) ({},{})",
        bbox.row_min,
        bbox.col_min,
        bbox.row_min,
        bbox.col_max,
        bbox.row_max,
        bbox.col_min,
        bbox.row_max,
        bbox.col_max);
}

auto length_layer_for_start_track(
    const TobEndTrackPrecompute& end_data,
    const std::size_t start_track
) -> std::optional<std::pair<std::size_t, std::size_t>> {
    for (std::size_t layer = 0; layer < end_data.length_layers.size(); ++layer) {
        for (const auto track : end_data.length_layers[layer]) {
            if (track == start_track) {
                const auto path_it = end_data.by_start_track.find(start_track);
                if (path_it == end_data.by_start_track.end()) {
                    return std::nullopt;
                }
                return std::pair {layer, path_it->second.path_length};
            }
        }
    }
    return std::nullopt;
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

} // namespace

auto log_path_precompute_cache(
    const std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache
) -> void {
    debug::info_fmt(
        "path precompute detail: records={}",
        records.size());
    for (std::size_t record_index = 0; record_index < records.size(); ++record_index) {
        const auto& record = records[record_index];
        if (record_index >= cache.by_record.size()) {
            continue;
        }
        const auto& per_end = cache.by_record[record_index];
        for (const auto end_track : record.end_tracks) {
            const auto end_it = per_end.find(end_track);
            if (end_it == per_end.end()) {
                debug::info_fmt(
                    "  record_id={} type={} net=\"{}\" origin_key=\"{}\" end_track={} start_tracks={{}}",
                    record.record_id,
                    net_type_label(record.type),
                    record.net_name,
                    record.origin_key,
                    end_track);
                continue;
            }
            const auto& end_data = end_it->second;
            debug::info_fmt(
                "  record_id={} type={} net=\"{}\" origin_key=\"{}\" end_track={} start_tracks={}",
                record.record_id,
                net_type_label(record.type),
                record.net_name,
                record.origin_key,
                end_track,
                format_track_set(end_data.candidate_start_tracks));
            for (const auto start_track : end_data.candidate_start_tracks) {
                const auto path_it = end_data.by_start_track.find(start_track);
                if (path_it == end_data.by_start_track.end()) {
                    continue;
                }
                const auto layer_opt = length_layer_for_start_track(end_data, start_track);
                if (layer_opt.has_value()) {
                    debug::info_fmt(
                        "    start_track={} length_layer={} path_len={} area={} bbox_corners={}",
                        start_track,
                        layer_opt->first,
                        layer_opt->second,
                        path_it->second.area,
                        format_bbox_corners(path_it->second.path_bbox));
                } else {
                    debug::info_fmt(
                        "    start_track={} path_len={} area={} bbox_corners={}",
                        start_track,
                        path_it->second.path_length,
                        path_it->second.area,
                        format_bbox_corners(path_it->second.path_bbox));
                }
            }
        }
    }
}

auto apply_tier_precompute_for_sat(
    std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache,
    const TobTierState& state
) -> TobReachRangeStats {
    auto stats = TobReachRangeStats {};
    stats.max_tier = state.max_tier();
    stats.total_records = records.size();
    stats.total_starttrack_edges = apply_tier_to_starttracks(records, cache, state);
    for (const auto& record : records) {
        stats.total_endtracks += record.end_tracks.size();
    }
    return stats;
}

auto log_reach_endpoints_for_tier_state(
    const std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache,
    const TobTierState& state
) -> void {
    debug::info_fmt(
        "SAT tier-active endpoints: max_tier={} records={}",
        state.max_tier(),
        records.size());
    for (std::size_t record_index = 0; record_index < records.size(); ++record_index) {
        const auto& record = records[record_index];
        const auto tier = state.tier_for_record(record_index);
        for (const auto end_track : record.end_tracks) {
            const auto starts_it = record.starttrack_by_endtrack.find(end_track);
            const auto starts = starts_it == record.starttrack_by_endtrack.end()
                ? std::Vector<std::size_t> {}
                : starts_it->second;
            debug::info_fmt(
                "  record_id={} type={} net=\"{}\" origin_key=\"{}\" tier={} end_track={} active_start_tracks={}",
                record.record_id,
                net_type_label(record.type),
                record.net_name,
                record.origin_key,
                tier,
                end_track,
                format_track_set(starts));
            if (record_index >= cache.by_record.size()) {
                continue;
            }
            const auto end_it = cache.by_record[record_index].find(end_track);
            if (end_it == cache.by_record[record_index].end()) {
                continue;
            }
            for (const auto start_track : starts) {
                const auto path_it = end_it->second.by_start_track.find(start_track);
                if (path_it == end_it->second.by_start_track.end()) {
                    continue;
                }
                const auto layer_opt = length_layer_for_start_track(end_it->second, start_track);
                if (layer_opt.has_value()) {
                    debug::info_fmt(
                        "    start_track={} length_layer={} path_len={} area={} bbox_corners={}",
                        start_track,
                        layer_opt->first,
                        layer_opt->second,
                        path_it->second.area,
                        format_bbox_corners(path_it->second.path_bbox));
                } else {
                    debug::info_fmt(
                        "    start_track={} path_len={} area={} bbox_corners={}",
                        start_track,
                        path_it->second.path_length,
                        path_it->second.area,
                        format_bbox_corners(path_it->second.path_bbox));
                }
            }
        }
    }
}

} // namespace PR_tool
