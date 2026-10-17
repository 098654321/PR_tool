#pragma once

#include "air_routing.hh"

namespace PR_tool::air316::rrr_detail {

// ILP can supply paths outside terminal bbox+1. Contain them before applying
// the unchanged d==0 round-end expansion; bins and reference lengths stay fixed.
inline auto air_import_boxes(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                             const std::Vector<OwnerRecord>& owners,
                             std::Vector<AirNetState>& states) -> void {
    for (std::size_t ni = 0; ni < nets.size(); ++ni) {
        auto& spatial = states[ni].spatial;
        auto& box = spatial.router_bb;
        const auto before = box;
        for (const auto& owner : owners) if (owner.net_index == ni)
            for (const auto& path : owner.demand_paths)
                for (int node : path) {
                    const auto [r,c] = routing_node_position(graph,node);
                    box.include(r,c);
                }
        box.r0 = std::max(box.r0,spatial.device.r0);
        box.r1 = std::min(box.r1,spatial.device.r1);
        box.c0 = std::max(box.c0,spatial.device.c0);
        box.c1 = std::min(box.c1,spatial.device.c1);
        if (box != before)
            debug::info_fmt("route RRR import bbox: net_id={} old=[r{}:{} c{}:{}] new=[r{}:{} c{}:{}] reason=contain_ILP_paths",
                nets[ni].net_id,before.r0,before.r1,before.c0,before.c1,box.r0,box.r1,box.c0,box.c1);
    }
}

} // namespace PR_tool::air316::rrr_detail
