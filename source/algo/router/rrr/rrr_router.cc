#include "signal_owners.hh"
#include "rrr_router.hh"
#include "air_routing.hh"
#include "path_metrics.hh"
#include "route_validate.hh"
#include <algo/router/common/routing_budget.hh>
#include <numeric>

namespace PR_tool::rrr {
auto rrr_initial_before(bool bus_a, int primary_a, int hpwl_a, OwnerId id_a,
                        bool bus_b, int primary_b, int hpwl_b, OwnerId id_b) -> bool {
    if (bus_a != bus_b) return bus_a;
    if (primary_a != primary_b) return primary_a > primary_b;
    if (hpwl_a != hpwl_b) return hpwl_a > hpwl_b;
    return id_a < id_b;
}

auto run_rrr(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
             const RrrParams& params, hardware::Interposer* interposer,
             const NetPaths& initial, const std::Vector<std::size_t>& order) -> RrrResult {
    if (!interposer) throw std::invalid_argument("RRR requires an Interposer");
    using namespace rrr_detail;
    const auto start = params.budget_start == algo::RoutingBudget::Clock::time_point{}
        ? algo::RoutingBudget::Clock::now() : params.budget_start;
    const auto elapsed = [&]() { return std::chrono::duration_cast<std::chrono::milliseconds>(
        algo::RoutingBudget::Clock::now()-start).count(); };
    algo::RoutingBudget budget(params.time_budget_seconds > 0
        ? std::optional<double>(params.time_budget_seconds) : std::nullopt,start);
    auto owners = build_owners(nets);
    auto resources = ResourceModel{params};
    resources.set_signal_owners(signal_owners(graph,nets));
    std::Vector<AirNetState> states;
    for (const auto& net : nets) states.push_back({air_spatial(graph,net)});
    for (auto& owner : owners) {
        for (std::size_t slot = 0; slot < owner.demand_indices.size(); ++slot)
            owner.demand_paths[slot] = initial.at(owner.net_index).at(owner.demand_indices[slot]);
        air_refresh(graph,owner,resources);
    }
    RrrResult result;
    LegalConvergence convergence;
    bool legal = false;
    const auto evaluate = [&]() {
        const int missing = air_missing(graph,nets,owners);
        const int overflow = resources.overflow();
        const auto sync = sync_violation(graph,nets,owners,interposer);
        if ((result.iterations <= 2 || result.iterations % 100 == 0) && debug::is_debug_level_enough(debug::DebugLevel::Debug)) {
            for(const auto& owner:owners)for(std::size_t slot=0;slot<owner.demand_paths.size();++slot)
                if(!air_connected(graph,nets[owner.net_index],owner,slot))
                    debug::debug_fmt("RRR pending: net={} demand={} retained_nodes={}",
                        nets[owner.net_index].name,owner.demand_indices[slot],owner.demand_paths[slot].size());
            std::Set<ResourceKey> seen;
            for (const auto& owner : owners) for (const auto& key : owner.claimed) {
                const auto conflict=(key.kind==ResourceKind::ModeStraight || key.kind==ResourceKind::ModeSwap)
                    ? mode_conflict_key(key.id) : key;
                if (resources.overflow(conflict)>0 && seen.insert(conflict).second)
                    debug::debug_fmt("RRR conflict: kind={} id={} extra={} overflow={}",
                        static_cast<int>(conflict.kind),conflict.id,conflict.extra,resources.overflow(conflict));
            }
        }
        legal = missing == 0 && overflow == 0 && sync.unequal_groups == 0;
        RrrResult candidate;
        if (legal) {
            candidate.paths = collect_paths_by_net(nets,owners);
            legal = validate_rrr_solution(graph,nets,candidate,interposer);
        }
        if (legal) {
            ++result.convergences;
            const auto length = total_wirelength(graph,candidate.paths);
            if (convergence.observe(length)) {
                result.paths = std::move(candidate.paths);
                result.total_wirelength = length;
                result.best_legal_ms = elapsed();
                if (result.first_legal_ms < 0) {
                    result.first_legal_ms = result.best_legal_ms;
                    result.first_legal_wirelength = length;
                }
                debug::info_fmt("RRR best legal: wirelength={} streak={}",length,convergence.streak);
            }
        }
        debug::info_fmt("RRR iteration {}: missing={} overflow={} sync_unequal={} legal={} best={} streak={}",
            result.iterations,missing,overflow,sync.unequal_groups,legal,
            convergence.streak ? result.total_wirelength : 0,convergence.streak);
        result.missing_owners = missing;
    };
    // Count a fully legal warm start exactly once, before reference searches.
    evaluate();
    auto rerouted = std::Vector<bool>(nets.size(),false);
    try {
        budget.check();
        for (auto ni : order) {
            budget.check();
            air_reference(ni,graph,nets,states[ni],params,interposer);
        }
        while (true) {
            budget.check();
            if (params.time_budget_seconds <= 0 && convergence.converged()) {
                result.stop_reason = "converged"; break;
            }
            air_update_router_boxes(graph,nets,owners,states,rerouted,interposer);
            resources.history_next();
            const int pruned = air_prune_illegal(graph,nets,owners,resources,states,interposer);
            std::Vector<std::size_t> selected;
            if (legal) {
                selected = air_wirelength_candidates(graph,nets,owners,states);
                for (auto ni : selected) air_rip_net(ni,owners,resources);
            }
            debug::debug_fmt("RRR prepare: conflict_ripup={} length_ripup={}",pruned,selected.size());
            std::fill(rerouted.begin(),rerouted.end(),false);
            for (auto ni : order) {
                budget.check();
                if (nets[ni].is_sync_bus) {
                    if (!air_net_complete(ni,graph,nets,owners)) {
                        auto search = params; search.search_box = states[ni].spatial.router_bb;
                        rerouted[ni] = air_route_sync(ni,graph,nets,owners,resources,search,interposer);
                    }
                } else {
                    for (auto& owner : owners) if (owner.net_index == ni)
                        rerouted[ni] = air_route_owner(graph,nets[ni],owner,resources,states[ni].spatial) || rerouted[ni];
                }
            }
            for(auto ni:order)if(air_retry_stalled(ni,graph,nets,owners,resources,rerouted[ni]))
                debug::debug_fmt("RRR retry stalled net: {}",nets[ni].name);
            ++result.iterations;
            if (convergence.streak) ++result.optimization_rounds;
            evaluate();
            // An empty netlist has one immutable legal solution and needs no searches.
            if (nets.empty()) { result.stop_reason = "converged"; break; }
        }
    } catch (const algo::RoutingTimeout&) { result.stop_reason = "time_limit"; }
    result.status = convergence.streak ? "success" : "no_legal_solution";
    if(!convergence.streak)for(auto ni:order)
        if(!air_net_complete(ni,graph,nets,owners))debug::info_fmt("RRR unresolved net: {}",nets[ni].name);

    result.routing_ms = result.budget_elapsed_ms = elapsed();
    if (convergence.streak) {
        if (!validate_rrr_solution(graph,nets,result,interposer))
            throw std::runtime_error("RRR: saved best solution failed validation");
        result.missing_owners = 0;
    }
    debug::info_fmt("RRR finished: status={} reason={} wirelength={} legal_solutions={} elapsed_ms={}",
        result.status,result.stop_reason,result.total_wirelength,result.convergences,result.routing_ms);
    return result;
}
}
