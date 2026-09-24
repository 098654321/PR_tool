#pragma once

#include "direct_ilp/direct_scope.hh"

namespace PR_tool {

auto validate_direct_route(const UnifiedGraph& graph,
                           const std::Vector<RoutingNet>& nets,
                           const std::Vector<RoutingScope>& scopes,
                           const RoutingResult& route) -> bool;

auto validate_partial_route(const UnifiedGraph& graph,
                            const std::Vector<RoutingNet>& nets,
                            const std::Vector<RoutingScope>& scopes,
                            const RoutingResult& route,
                            const std::map<std::size_t, std::size_t>& bus_lengths) -> bool;

} // namespace PR_tool
