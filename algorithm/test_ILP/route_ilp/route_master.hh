#pragma once

#include "route_ilp/route_big_m.hh"
#include "route_ilp/route_search.hh"

namespace PR_tool {

struct RouteIlpOptions {
    int verbose_level{};
    int time_limit_minutes{};
    std::String highs_log_path;
    RouteBigMMode big_m_mode{RouteBigMMode::Default};
    const RoutingResult* initial_sat_route{};
};

struct RouteIlpResult {
    bool has_integer_solution{};
    RoutingResult route;
    std::set<RouteOwner> missing;
    std::map<std::size_t, std::size_t> bus_lengths;
    std::String status;
    double big_m{};
};

auto solve_route_ilp(const UnifiedGraph& graph,
                     const std::Vector<RoutingNet>& nets,
                     const std::Vector<RoutingScope>& scopes,
                     const RouteIlpOptions& options) -> RouteIlpResult;

} // namespace PR_tool
