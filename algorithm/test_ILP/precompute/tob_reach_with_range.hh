#pragma once

#include "common/ilp_types.hh"
#include "common/tob_bbox_expansion.hh"
#include "precompute/tob_path_precompute.hh"

#include <cstddef>
#include <std/collection.hh>

namespace PR_tool {

struct TobReachRangeStats {
    std::size_t max_tier{0};
    std::size_t total_records{0};
    std::size_t total_endtracks{0};
    std::size_t total_starttrack_edges{0};
};

auto apply_tier_precompute_for_sat(
    std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache,
    const TobTierState& state
) -> TobReachRangeStats;

auto log_path_precompute_cache(
    const std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache
) -> void;

auto log_reach_endpoints_for_tier_state(
    const std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache,
    const TobTierState& state
) -> void;

} // namespace PR_tool
