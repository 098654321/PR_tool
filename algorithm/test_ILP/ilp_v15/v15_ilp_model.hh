#pragma once

#include "ilp_v15/v15_ilp_prepare.hh"

#include <map>
#include <set>

namespace PR_tool {

struct V15CommodityModelSolution {
    std::size_t commodity_id{0};
    std::set<int> selected_arc_ids;
    std::map<int, int> flow_by_arc;
    std::set<int> used_node_ids;
};

struct V15IlpModelResult {
    bool ok{false};
    V15IlpStatus status{V15IlpStatus::Failed};
    std::String message;
    V15IlpStats stats;
    std::Vector<V15CommodityModelSolution> commodities;
    std::map<int, bool> mode_straight;
};

auto solve_v15_ilp_model(
    const UnifiedGraph& graph,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const std::Vector<IlpCommodity>& commodities,
    const V15LockedResources& locked,
    const V15IlpOptimizeOptions& options,
    const V15MipStart& mip_start
) -> V15IlpModelResult;

} // namespace PR_tool
