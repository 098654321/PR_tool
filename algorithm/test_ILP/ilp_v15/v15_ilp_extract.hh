#pragma once

#include "ilp_v15/v15_ilp_model.hh"

namespace PR_tool {

auto extract_v15_routing_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<IlpCommodity>& commodities,
    const V15IlpModelResult& model_result,
    const SatRoutingResult& sat_result,
    const std::set<std::size_t>& selected_net_ids
) -> SatRoutingResult;

} // namespace PR_tool
