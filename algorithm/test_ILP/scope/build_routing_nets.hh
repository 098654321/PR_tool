#pragma once

#include "common/routing_types.hh"

#include <circuit/net/net.hh>
#include <std/collection.hh>

namespace PR_tool {

auto bump_to_routing_coord(const hardware::Bump* bump) -> Bump_coord;

auto build_routing_nets(const std::Vector<std::Rc<circuit::Net>>& nets) -> std::Vector<RoutingNet>;

} // namespace PR_tool
