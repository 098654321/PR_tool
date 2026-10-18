#include "rrr_router.hh"
#include "stop_policy.hh"
#include "import_bbox.hh"
#include "air_routing.hh"
#include "rrr_routing.hh"
#include "route_validate.hh"

#include "route_log.hh"

#include <debug/debug.hh>
#include <utility/elapsed.hh>

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace PR_tool::air316 {

auto rrr_initial_before(
    bool bus_a,
    int primary_a,
    int hpwl_a,
    OwnerId id_a,
    bool bus_b,
    int primary_b,
    int hpwl_b,
    OwnerId id_b
) -> bool {
    if (bus_a != bus_b) {
        return bus_a && !bus_b;
    }
    if (primary_a != primary_b) {
        return primary_a > primary_b;
    }
    if (hpwl_a != hpwl_b) {
        return hpwl_a > hpwl_b;
    }
    return id_a < id_b;
}

auto run_rrr(
    const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
    const RrrParams& params, hardware::Interposer* interposer, int verbose_level,
    const std::Vector<std::Rc<circuit::Net>>* raw_nets,
    const std::Vector<std::Vector<std::Vector<int>>>& initial_paths
) -> RrrResult {
    using namespace rrr_detail;
    if (interposer == nullptr) throw std::invalid_argument("RRR: run_rrr requires a non-null Interposer");
    const auto routing_start = std::chrono::steady_clock::now();
    const auto budget_start = params.budget_start == std::chrono::steady_clock::time_point{}
        ? routing_start : params.budget_start;
    const auto elapsed_ms = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - budget_start).count();
    };
    const auto expired = [&]() {
        return std::chrono::steady_clock::now() >= params.deadline;
    };
    auto result = RrrResult{};
    auto owners = build_owners(nets);
    auto resources = ResourceModel{params};
    auto states = std::Vector<AirNetState>{};
    states.reserve(nets.size());
    for (const auto& net : nets) states.push_back({air_spatial(graph, net)});
    const auto order = air_order(nets, interposer, raw_nets);
    debug::info_fmt("FPIA RRR: graph nodes={} arcs={} route_owners={}", graph.nodes.size(), graph.arcs.size(), owners.size());
    debug::info_fmt("FPIA RRR: params seed={} time_budget_seconds={} algorithm=AIR source=316e477 cost_source=0d6e86f:test_ILP cost_policy=fpia_additive H=4 k=1 s=20 arc_base=1 decay=0.9 increment=1 history_weight=1 iteration_limit=none legal_stagnation_limit=10",
        params.seed, params.time_budget_seconds);
    for (auto ni : order) {
        if (expired()) break;
        const auto& s = states[ni].spatial;
        debug::info_fmt("FPIA RRR: spatial net_id={} terminal_bb=[r{}:{} c{}:{}] router_bb=[r{}:{} c{}:{}] bin_side={} port_threshold={}",
            nets[ni].net_id, s.initial.r0, s.initial.r1, s.initial.c0, s.initial.c1,
            s.router_bb.r0, s.router_bb.r1, s.router_bb.c0, s.router_bb.c1,
            s.bin_size, kAirIncrementalPortThreshold);
        air_reference(ni, graph, nets, states[ni], params, interposer);
    }
    // Treat the ILP paths as the completed first round, before round-end preparation.
    if (initial_paths.size() != nets.size()) throw std::logic_error("AIR initial net count mismatch");
    for (auto& owner : owners) {
        for (std::size_t slot = 0; slot < owner.demand_paths.size(); ++slot)
            owner.demand_paths[slot] = initial_paths.at(owner.net_index).at(owner.demand_indices[slot]);
        air_refresh(graph, owner, resources, &states[owner.net_index].spatial);
    }
    air_import_boxes(graph, nets, owners, states);
    auto best = std::Vector<OwnerRecord>{};
    std::size_t best_length = std::numeric_limits<std::size_t>::max();
    bool has_legal = false;
    auto stop = StopPolicy{params.deadline};
    bool imported_round = true;
    while (true) {
        auto rerouted = std::Vector<bool>(nets.size(), false);
        if (has_legal) ++result.optimization_rounds;
        if (imported_round) {
            for (auto ni : order) rerouted[ni] = air_net_complete(ni, graph, nets, owners);
        }
        for (auto ni : order) {
            if (imported_round || expired()) break;
            if (nets[ni].is_sync_bus) {
                if (!air_net_complete(ni, graph, nets, owners)) {
                    auto search_params = params;
                    search_params.search_box = states[ni].spatial.router_bb;
                    rerouted[ni] = air_route_sync(ni, graph, nets, owners, resources, search_params, interposer);
                }
            } else {
                for (auto& owner : owners) if (owner.net_index == ni)
                    rerouted[ni] = air_route_owner(graph, nets[ni], owner, resources, states[ni].spatial) || rerouted[ni];
            }
        }
        imported_round = false;
        ++result.iterations;
        const int missing = air_missing(graph, nets, owners);
        const int overflow = resources.overflow();
        const auto sync = sync_violation(graph, nets, owners, interposer);
        const auto wirelength = current_wirelength(graph, nets, owners);
        bool legal = missing == 0 && overflow == 0 && sync.unequal_groups == 0;
        if (legal) {
            auto candidate = RrrResult{}; candidate.paths = collect_paths_by_net(nets, owners);
            legal = validate_rrr_solution(graph, nets, candidate, interposer);
        }
        if (result.iterations == 1)
            debug::info_fmt("FPIA RRR: initial overflow={} unequal_sync_groups={} sync_gap={} total_wirelength={} missing_owners={}",
                overflow, sync.unequal_groups, sync.total_gap, wirelength, missing);
        const bool improved = stop.observe(legal, wirelength);
        result.convergences = stop.legal_count;
        if (legal) {
            if (improved) {
                has_legal = true; best_length = wirelength; best = owners;
                result.best_legal_ms = elapsed_ms();
                if (result.first_legal_ms < 0) result.first_legal_ms = result.best_legal_ms;
                debug::info_fmt("FPIA RRR: legal incumbent wirelength={} first_legal_ms={} best_legal_ms={}",
                    wirelength, result.first_legal_ms, result.best_legal_ms);
            }
        }
        debug::info_fmt("FPIA RRR: iter={} overflow={} total_wirelength={} unequal_sync_groups={} sync_gap={} missing_owners={} convergences={} has_legal={} budget_elapsed_ms={}",
            result.iterations-1, overflow, wirelength, sync.unequal_groups, sync.total_gap,
            missing, result.convergences, has_legal, elapsed_ms());
        if (verbose_level >= 1) dump_overflows(owners, nets, resources);
        debug::info_fmt("route RRR convergence: legal={} improved={} best_wirelength={} stagnant={}/10 budget_ms={}",
            legal, improved, has_legal ? best_length : 0, stop.stagnant, params.time_budget_seconds * 1000);
        if (const auto* reason = stop.reason()) { result.stop_reason = reason; break; }
        // Inspect complete results BEFORE pruning. Cross-net overflow is allowed.
        const int boxes_updated = air_update_router_boxes(graph, nets, owners, states, rerouted, interposer);
        // Price congestion BEFORE pruning it, retaining the AIR update point.
        resources.history_next();
        const int pruned = air_prune_illegal(graph, nets, owners, resources, states, interposer);
        auto selected = std::Vector<std::size_t>{};
        if (legal) {
            selected = air_wirelength_candidates(graph, nets, owners, states);
            for (auto ni : selected) air_rip_net(ni, owners, resources);
        }
        debug::info_fmt("FPIA RRR: prepare illegal_ripup={} wirelength_ripup={} bbox_updates={} phase={}",
            pruned, selected.size(), boxes_updated, legal ? "optimize" : "repair");
    }
    // With no complete incumbent, return the verified ILP partial input rather
    // than the possibly congested live search state. Time expiry never waits for legality.
    if (has_legal) owners = std::move(best);
    else for (auto& owner : owners) {
        for (std::size_t slot = 0; slot < owner.demand_paths.size(); ++slot)
            owner.demand_paths[slot] = initial_paths[owner.net_index][owner.demand_indices[slot]];
        auto empty = ResourceModel{params};
        air_refresh(graph, owner, empty);
    }
    resources = ResourceModel{params};
    for (const auto& owner : owners)
        resources.claim(owner.id, std::Vector<ResourceKey>(owner.claimed.begin(), owner.claimed.end()));
    const auto sync = sync_violation(graph, nets, owners, interposer);
    result.status = has_legal ? "success" : "incomplete";
    result.missing_owners = air_missing(graph, nets, owners);
    result.best_overflow = resources.overflow();
    result.unequal_sync_groups = sync.unequal_groups; result.total_sync_gap = sync.total_gap;
    // The baseline collector compacts missing SyncBus lanes. The imported
    // partial fallback must retain empty demand slots for the ILP handoff.
    result.paths = has_legal ? collect_paths_by_net(nets, owners) : initial_paths;
    result.total_wirelength = total_wirelength(graph, result.paths);
    if (has_legal && !validate_rrr_solution(graph, nets, result, interposer))
        throw std::logic_error("AIR best legal solution failed independent validation");
    result.routing_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - routing_start).count();
    result.budget_elapsed_ms = elapsed_ms();
    debug::info_fmt("FPIA RRR: status={} stop_reason={} iterations={} optimization_rounds={} convergences={} best_overflow={} unequal_sync_groups={} sync_gap={} routing_ms={} first_legal_ms={} best_legal_ms={} budget_elapsed_ms={} missing_owners={}",
        result.status, result.stop_reason, result.iterations, result.optimization_rounds, result.convergences,
        result.best_overflow, sync.unequal_groups, sync.total_gap, result.routing_ms,
        result.first_legal_ms, result.best_legal_ms, result.budget_elapsed_ms, result.missing_owners);
    debug::info_fmt("routing result: total_wirelength={} RRR_routing_time={} elapsed_ms={}",
        result.total_wirelength, result.routing_ms, Elapsed::milliseconds());
    if (has_legal) dump_paths(graph, nets, owners, interposer);
    else debug::info("route RRR: returning original ILP partial paths with demand slots preserved");
    return result;
}

} // namespace PR_tool::air316
