#pragma once

#include "route_ilp/route_search.hh"

#include <optional>

namespace PR_tool {

struct RouteMipStart {
    std::Vector<std::Vector<double>> x;
    std::Vector<double> slack;
    double objective{};
    std::size_t routed{};
    std::String source;
};

auto same_route_column(const RouteColumn& a, const RouteColumn& b) -> bool;

auto complete_integer_lp_columns(
    const std::Vector<std::Vector<RouteColumn>>& pool,
    const std::Vector<std::Vector<double>>& x,
    const std::Vector<double>& slack
) -> std::optional<std::Vector<RouteColumn>>;

auto discard_old_sync_columns(std::Vector<RouteColumn>& saved,
    const std::Vector<RoutingNet>& nets,
    const std::map<std::size_t, std::size_t>& bus_lengths
) -> std::Vector<RouteOwner>;

auto remap_mip_start(const std::Vector<std::Vector<RouteColumn>>& pool,
    const std::Vector<RouteColumn>& saved, double big_m) -> RouteMipStart;

} // namespace PR_tool
