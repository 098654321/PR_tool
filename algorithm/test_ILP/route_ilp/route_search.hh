#pragma once

#include "common/routing_scope.hh"
#include "graph/unified_routing_graph.hh"

#include <map>
#include <set>

namespace PR_tool {

struct RouteOwner {
    std::size_t net_id{};
    std::size_t demand_id{}; // ignored for a non-SyncBus net
    auto operator<=>(const RouteOwner&) const = default;
};

struct RouteResource {
    int kind{}; // 0 node, 1 switch, 2 matching port, 3 mode conflict pair
    int id{};
    int extra{};
    auto operator<=>(const RouteResource&) const = default;
};

struct RouteColumn {
    RouteOwner owner;
    std::Vector<SourceSinkPairPath> paths;
    std::set<RouteResource> resources;
    std::size_t wirelength{};
};

struct RouteSearchOptions {
    std::map<RouteResource, double> prices;
    std::set<int> discouraged_nodes;
    int variant{};
    std::size_t exact_length{}; // zero means ordinary shortest route
    int max_expansions{300000};
};

auto route_owners(const std::Vector<RoutingNet>& nets) -> std::Vector<RouteOwner>;
auto net_for_owner(const std::Vector<RoutingNet>& nets, RouteOwner owner)
    -> const RoutingNet&;
auto scope_for_owner(const std::Vector<RoutingScope>& scopes, RouteOwner owner)
    -> const RoutingScope&;
auto route_column_from_paths(const UnifiedGraph& graph, const RoutingNet& net,
                             RouteOwner owner,
                             const std::Vector<SourceSinkPairPath>& paths)
    -> RouteColumn;
auto find_route_column(const UnifiedGraph& graph, const RoutingNet& net,
                       const RoutingScope& scope, RouteOwner owner,
                       const RouteSearchOptions& options,
                       const RouteColumn* base = nullptr,
                       std::size_t replace_demand = 0)
    -> RouteColumn;
auto find_sync_prefix_column(const UnifiedGraph& graph, const RoutingNet& net,
                             const RoutingScope& scope, RouteOwner owner,
                             const RouteSearchOptions& options,
                             const RouteColumn& former, int tail_percent)
    -> RouteColumn;
auto route_columns_conflict(const RouteColumn& a, const RouteColumn& b) -> bool;

} // namespace PR_tool
