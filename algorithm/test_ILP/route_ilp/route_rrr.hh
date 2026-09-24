#pragma once

#include "route_ilp/route_master.hh"

#include <chrono>

namespace PR_tool {

auto optimize_route_columns_rrr(const UnifiedGraph& graph,
                                const std::Vector<RoutingNet>& nets,
                                const std::Vector<RoutingScope>& scopes,
                                const RouteIlpResult& baseline,
                                int verbose_level = 0,
                                std::chrono::steady_clock::time_point deadline =
                                    std::chrono::steady_clock::time_point::max())
    -> RoutingResult;

} // namespace PR_tool
