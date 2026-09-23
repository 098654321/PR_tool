#pragma once

#include <cstddef>
#include <std/collection.hh>

namespace PR_tool {

struct RoutingScope {
    std::size_t net_id{0};
    std::Vector<int> node_ids;
    std::Vector<int> arc_ids;
    // Global ID -> compact offset, or -1 when absent.
    std::Vector<int> node_offset;
    std::Vector<int> arc_offset;
};

// Historical SAT code uses this name for the same plain scope container.
using UnifiedSatNetScope = RoutingScope;

} // namespace PR_tool
