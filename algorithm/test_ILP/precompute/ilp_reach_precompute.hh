#pragma once

#include "common/ilp_types.hh"

#include <cstddef>
#include <std/collection.hh>

namespace PR_tool {

struct IlpReachPrecomputeStats {
    std::size_t total_records{0};
    std::size_t bnet_records{0};
    std::size_t tnet_records{0};
    std::size_t pnnet_records{0};
    std::size_t total_endtracks{0};
    std::size_t total_starttrack_edges{0};
};

auto precompute_reach_for_records(std::Vector<Net_cost_record>& records) -> IlpReachPrecomputeStats;

} // namespace PR_tool
