#include "production.hh"
#include "net_adapter.hh"
#include "path_adapter.hh"
#include "rrr_router.hh"
#include <algo/router/common/routing_budget.hh>
#include <circuit/basedie.hh>
#include <numeric>

namespace PR_tool::rrr {
auto finish_routing(hardware::Interposer* interposer, circuit::BaseDie* basedie,
                    algo::RouteEngine& engine, const RrrParams& params) -> bool {
    const auto raw = engine.nets();
    std::Vector<std::Rc<circuit::Net>> sorted;
    const auto& all = basedie->nets().at(engine.mode());
    for (auto* net : raw) {
        auto it = std::find_if(all.begin(),all.end(),[&](const auto& p) { return p.get() == net; });
        if (it == all.end()) throw std::runtime_error("RRR: sorted net not found");
        sorted.push_back(*it);
    }
    const auto nets = build_routing_nets(sorted);
    const bool first_complete = engine.position() == raw.size() && engine.failed_net_names().empty();
    std::size_t first_length = 0;
    for (const auto* net : raw) first_length += net->length();
    debug::info_fmt("First maze pass: complete={} routed={}/{} wirelength={}",
        first_complete, engine.position(), raw.size(), first_length);
    UnifiedGraph graph;
    try {
        if (first_complete) {
            // Preserve/validate a completed first pass even when the deadline
            // expires during conversion. This builds no new routing solution.
            algo::RoutingBudget cleanup;
            graph = build_hardware_graph(interposer,nets);
        } else {
            algo::RoutingBudget::check();
            graph = build_hardware_graph(interposer,nets);
        }
    } catch (const algo::RoutingTimeout&) {
        debug::info("Routing stopped: time limit; no complete legal solution found");
        return false;
    }
    const auto initial = import_paths(interposer,graph,nets,raw);
    std::Vector<std::size_t> order(nets.size());
    std::iota(order.begin(),order.end(),0);
    auto result = run_rrr(graph,nets,params,interposer,initial,order);
    if (result.status != "success") return false;
    algo::RoutingBudget cleanup;
    commit_paths(interposer,graph,nets,result.paths,raw);
    engine.clear_failed_net_names();
    return true;
}
}
