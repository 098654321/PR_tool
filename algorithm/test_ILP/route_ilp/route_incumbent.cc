#include "route_ilp/route_incumbent.hh"

#include <algorithm>
#include <cmath>

namespace PR_tool {

auto minimum_dual_resources(const std::map<RouteResource, double>& duals)
    -> std::set<RouteResource> {
    auto result = std::set<RouteResource>{};
    if (duals.empty()) return result;
    const double minimum = std::min_element(duals.begin(), duals.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; })->second;
    for (const auto& [resource, dual] : duals)
        if (std::abs(dual - minimum) <= 1e-8) result.insert(resource);
    return result;
}

auto same_route_column(const RouteColumn& a, const RouteColumn& b) -> bool {
    if (a.paths.size() != b.paths.size() || a.resources != b.resources) return false;
    for (const auto& path : a.paths)
        if (std::ranges::none_of(b.paths, [&](const auto& other) {
            return path.demand_id == other.demand_id &&
                   path.source_index == other.source_index &&
                   path.node_path == other.node_path;
        })) return false;
    return true;
}

auto complete_integer_lp_columns(
    const std::Vector<std::Vector<RouteColumn>>& pool,
    const std::Vector<std::Vector<double>>& x,
    const std::Vector<double>& slack
) -> std::optional<std::Vector<RouteColumn>> {
    auto saved = std::Vector<RouteColumn>{};
    for (std::size_t i = 0; i < pool.size(); ++i) {
        if (!std::isfinite(slack[i]) || std::abs(slack[i]) > 1e-6) return std::nullopt;
        std::optional<std::size_t> selected;
        for (std::size_t j = 0; j < pool[i].size(); ++j) {
            const double value = x[i][j];
            if (!std::isfinite(value)) return std::nullopt;
            if (std::abs(value) <= 1e-6) continue;
            if (std::abs(value - 1.0) > 1e-6 || selected) return std::nullopt;
            selected = j;
        }
        if (!selected) return std::nullopt;
        saved.push_back(pool[i][*selected]);
    }
    return saved;
}

auto discard_old_sync_columns(std::Vector<RouteColumn>& saved,
    const std::Vector<RoutingNet>& nets,
    const std::map<std::size_t, std::size_t>& bus_lengths
) -> std::Vector<RouteOwner> {
    auto discarded = std::Vector<RouteOwner>{};
    for (auto& column : saved) {
        if (column.paths.empty()) continue;
        const auto& net = net_for_owner(nets, column.owner);
        if (net.is_sync_bus && column.wirelength != bus_lengths.at(net.net_id)) {
            discarded.push_back(column.owner);
            column = RouteColumn{column.owner};
        }
    }
    return discarded;
}

auto remap_mip_start(const std::Vector<std::Vector<RouteColumn>>& pool,
    const std::Vector<RouteColumn>& saved, double big_m) -> RouteMipStart {
    auto start = RouteMipStart{};
    start.x.resize(pool.size());
    start.slack.assign(pool.size(), 1.0);
    for (std::size_t i = 0; i < pool.size(); ++i) {
        start.x[i].assign(pool[i].size(), 0.0);
        const auto match = saved[i].paths.empty() ? pool[i].end() :
            std::find_if(pool[i].begin(), pool[i].end(), [&](const auto& column) {
                return same_route_column(column, saved[i]);
            });
        if (match == pool[i].end()) {
            start.objective += big_m;
        } else {
            start.x[i][static_cast<std::size_t>(match - pool[i].begin())] = 1.0;
            start.slack[i] = 0.0;
            start.objective += match->wirelength;
            ++start.routed;
        }
    }
    return start;
}

} // namespace PR_tool
