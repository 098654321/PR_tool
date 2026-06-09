#pragma once

#include "common/ilp_types.hh"
#include "precompute/ilp_reach_precompute.hh"

#include <cstddef>
#include <std/collection.hh>

namespace PR_tool {

// Must match solve_tob_sat.hh::kMaxRangeLevel (SAT+MCF outer loop 0..4).
inline constexpr std::size_t kTobReachMaxRangeLevel = 4;

struct TobReachRangeStats {
    std::size_t range_level{0};
    std::size_t total_records{0};
    std::size_t total_endtracks{0};
    std::size_t total_starttrack_edges{0};
};

auto precompute_reach_for_range(std::Vector<Net_cost_record>& records, std::size_t range_level) -> TobReachRangeStats;

auto log_reach_endpoints_for_range(
    const std::Vector<Net_cost_record>& records,
    std::size_t range_level
) -> void;

} // namespace PR_tool
