#pragma once
#include <algo/router/rrr/rrr_router.hh>
#include <numeric>
inline auto run_empty(const PR_tool::rrr::UnifiedGraph& graph,
                      const std::vector<PR_tool::rrr::RoutingNet>& nets,
                      const PR_tool::rrr::RrrParams& params,
                      PR_tool::hardware::Interposer* interposer) -> PR_tool::rrr::RrrResult {
    PR_tool::rrr::NetPaths initial(nets.size());
    for (std::size_t i=0; i<nets.size(); ++i) initial[i].resize(nets[i].demands.size());
    std::vector<std::size_t> order(nets.size()); std::iota(order.begin(),order.end(),0);
    return PR_tool::rrr::run_rrr(graph,nets,params,interposer,initial,order);
}
