#pragma once

#include "precompute/ilp_bounding_box.hh"

#include <hardware/track/trackcoord.hh>

#include <cstddef>
#include <std/collection.hh>

namespace PR_tool {

auto tob_channel_track_coords(std::size_t tob_linear) -> std::Vector<hardware::TrackCoord>;

auto kshortest_reachable_tob_tracks(
    const hardware::TrackCoord& end_track,
    std::size_t start_tob_linear,
    const IlpBoundingBox& bbox,
    std::size_t k
) -> std::Vector<std::size_t>;

} // namespace PR_tool
