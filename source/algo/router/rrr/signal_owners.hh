#pragma once
#include "hardware_graph.hh"

namespace PR_tool::rrr {
inline auto demand_owner(const RoutingNet& net,const RoutingDemand& demand) -> OwnerId {
    return {net.net_id,net.is_sync_bus?demand.demand_id:0};
}
// Production permits a sync lane and an ordinary fanout branch to share one
// driver. They keep separate route/sync records but own the same electrical net.
inline auto signal_owners(const UnifiedGraph& graph,const std::Vector<RoutingNet>& nets)
    -> std::Map<OwnerId,OwnerId> {
    std::Map<OwnerId,OwnerId> parent;
    std::Map<int,OwnerId> drivers;
    const auto root=[&](auto&& self,OwnerId id)->OwnerId {
        auto [it,inserted]=parent.emplace(id,id);
        if(!inserted && it->second!=id)it->second=self(self,it->second);
        return it->second;
    };
    for(const auto& net:nets)for(const auto& d:net.demands) {
        const auto owner=demand_owner(net,d);
        std::Vector<int> sources;
        if(d.reverse_physical)sources.push_back(resolve_graph_node(graph,d.sink));
        else for(auto si:d.candidate_source_indices)sources.push_back(resolve_graph_node(graph,net.sources.at(si)));
        for(int s:sources) {
            if(s<0)continue;
            auto [it,inserted]=drivers.emplace(s,owner);
            if(!inserted)parent[root(root,owner)]=root(root,it->second);
        }
    }
    for(auto& [id,p]:parent)p=root(root,id);
    return parent;
}
}
