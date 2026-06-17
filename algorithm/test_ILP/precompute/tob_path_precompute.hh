#pragma once

#include "common/ilp_types.hh"
#include "common/tob_bbox_expansion.hh"
#include "precompute/ilp_bounding_box.hh"

#include <hardware/interposer.hh>
#include <hardware/track/trackcoord.hh>

#include <cstddef>
#include <map>
#include <optional>
#include <std/collection.hh>

namespace PR_tool {

struct TobPathEntry {
    std::size_t path_length{0};
    IlpBoundingBox path_bbox {};
    std::size_t area{0};
};

struct TobEndTrackPrecompute {
    std::Vector<std::size_t> candidate_start_tracks;
    std::map<std::size_t, TobPathEntry> by_start_track;
    std::Vector<std::Vector<std::size_t>> length_layers;
};

struct TobPathPrecomputeCache {
    std::Vector<std::map<std::size_t, TobEndTrackPrecompute>> by_record;
};

struct TobPathPrecomputeStats {
    std::size_t total_records{0};
    std::size_t total_endtracks{0};
    std::size_t total_path_pairs{0};
    std::size_t unreachable_pairs{0};
};

struct ExpandTierResult {
    std::Vector<std::size_t> changed_records;
    std::size_t new_starttrack_edges{0};
};

auto tob_channel_track_coords(std::size_t tob_linear) -> std::Vector<hardware::TrackCoord>;

auto precompute_all_path_caches(
    std::Vector<Net_cost_record>& records,
    hardware::Interposer* interposer,
    bool enable_parallel = false
) -> TobPathPrecomputeCache;

auto count_starttrack_edges(const std::Vector<Net_cost_record>& records) -> std::size_t;

auto apply_tier_to_starttracks(
    std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache,
    const TobTierState& state
) -> std::size_t;

auto expand_tier(
    TobTierState& state,
    const std::Vector<std::size_t>& record_indices,
    std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache
) -> ExpandTierResult;

auto lookup_path_bbox(
    const TobPathPrecomputeCache& cache,
    std::size_t record_index,
    std::size_t end_track,
    std::size_t start_track
) -> std::optional<IlpBoundingBox>;

} // namespace PR_tool
