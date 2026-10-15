#include "rrr_router.hh"
#include "initial_route.hh"
#include "transaction.hh"
#include "rrr_routing.hh"
#include "route_validate.hh"

#include "route_log.hh"

#include <debug/debug.hh>
#include <utility/elapsed.hh>

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace PR_tool {

auto rrr_is_better(
    int overflow,
    int unequal_sync_groups,
    std::size_t sync_gap,
    std::size_t wirelength,
    int best_overflow,
    int best_unequal_sync_groups,
    std::size_t best_sync_gap,
    std::size_t best_wirelength
) -> bool {
    if (overflow != best_overflow) {
        return overflow < best_overflow;
    }
    if (unequal_sync_groups != best_unequal_sync_groups) {
        return unequal_sync_groups < best_unequal_sync_groups;
    }
    if (sync_gap != best_sync_gap) {
        return sync_gap < best_sync_gap;
    }
    return wirelength < best_wirelength;
}

auto rrr_dirty_before(
    int exposure_a,
    int retry_a,
    int hpwl_a,
    OwnerId id_a,
    int exposure_b,
    int retry_b,
    int hpwl_b,
    OwnerId id_b
) -> bool {
    if (exposure_a != exposure_b) {
        return exposure_a > exposure_b;
    }
    if (retry_a != retry_b) {
        return retry_a > retry_b;
    }
    if (hpwl_a != hpwl_b) {
        return hpwl_a > hpwl_b;
    }
    return id_a < id_b;
}

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
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const RrrParams& input_params,
    hardware::Interposer* interposer,
    int verbose_level,
    const std::Vector<std::Rc<circuit::Net>>* raw_nets
) -> RrrResult {
    using namespace rrr_detail;
    if (interposer == nullptr) throw std::invalid_argument("RRR: run_rrr requires a non-null Interposer");
    auto params = input_params;
    auto resources = ResourceModel{params};
    auto owners = build_owners(nets);
    const auto routing_start = std::chrono::steady_clock::now();
    const auto budget_start = params.budget_start == std::chrono::steady_clock::time_point{}
        ? routing_start : params.budget_start;
    const bool budget_enabled = params.time_budget_seconds > 0;
    const auto elapsed_ms = [&]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - budget_start).count();
    };
    const auto budget_expired = [&]() {
        return budget_enabled && elapsed_ms() >= params.time_budget_seconds * 1000;
    };
    debug::info_fmt("FPIA RRR: graph nodes={} arcs={} route_owners={}",
        graph.nodes.size(), graph.arcs.size(), owners.size());
    debug::info_fmt(
        "FPIA RRR: params max_iterations={} seed={} H={} k={} s={} sync_tail_extra_tracks={} time_budget_seconds={} optimization_scope=transaction history_decay={} cost_policy=additive",
        params.max_iterations, params.seed, params.H, params.k, params.s,
        params.sync_tail_extra_tracks, params.time_budget_seconds, params.decay);
    auto result = RrrResult{};
    auto best_owners = owners;
    bool has_legal = false;
    auto best_wirelength = std::numeric_limits<std::size_t>::max();
    auto stop_reason = std::String{"iteration_limit"};
    int iterations = 0, optimization_rounds = 0;
    bool reported_overrun = false;
    const auto checkpoint = [&]() {
        const auto wirelength = current_wirelength(graph, nets, owners);
        if (missing_owners(owners) == 0 && resources.overflow() == 0 &&
            sync_violation(graph, nets, owners, interposer).unequal_groups == 0 &&
            (!has_legal || wirelength < best_wirelength)) {
            auto candidate = RrrResult{}; candidate.paths = collect_paths_by_net(nets, owners);
            if (!validate_rrr_solution(graph, nets, candidate, interposer))
                throw std::logic_error("RRR committed complete route failed validation");
            has_legal = true; best_wirelength = wirelength; best_owners = owners;
            result.best_legal_ms = elapsed_ms();
            if (result.first_legal_ms < 0) result.first_legal_ms = result.best_legal_ms;
            debug::info_fmt("FPIA RRR: legal incumbent wirelength={} first_legal_ms={} best_legal_ms={}",
                wirelength, result.first_legal_ms, result.best_legal_ms);
        }
        debug::info_fmt("FPIA RRR: checkpoint iterations={} budget_elapsed_ms={} has_legal={} best_legal_wirelength={} missing_owners={}",
            iterations, elapsed_ms(), has_legal, has_legal ? best_wirelength : 0, missing_owners(owners));
    };
    const auto should_stop = [&]() {
        if (budget_expired() && has_legal) { stop_reason = "time_budget"; return true; }
        if (budget_expired() && !reported_overrun) {
            debug::info("FPIA RRR: budget exceeded without legal solution; continue until first legal solution");
            reported_overrun = true;
        }
        return false;
    };
    auto order = std::Vector<std::size_t>{};
    try {
        initialize_sync_targets(graph, nets, owners, params, interposer);
        if (raw_nets != nullptr) {
            if (raw_nets->size() != nets.size()) throw std::invalid_argument("initial raw/routing net count mismatch");
            order = original_initial_route(graph, nets, owners, resources, interposer, *raw_nets);
        } else {
            // Graph-only unit-test seam; production always executes the actual Sort/Resources/Maze.
            for (auto index : initial_type_order(owners, nets)) {
                const auto ni = owners[index].net_index;
                if (std::find(order.begin(), order.end(), ni) == order.end()) order.push_back(ni);
            }
            auto strict = params; strict.hard_capacity = true;
            for (auto ni : order) {
                const auto before = owners;
                const auto occupancy = resources;
                try {
                    if (nets[ni].is_sync_bus) {
                        if (!route_sync_group(ni, owners, graph, nets, resources, strict, interposer))
                            throw std::runtime_error("maze: initial SyncNet failed");
                    } else route_owner(owners[unit_indices(owners, ni).front()],
                        graph, nets, resources, strict, interposer);
                    bool valid = resources.overflow() == 0;
                    for (auto index : unit_indices(owners, ni)) valid = valid && owner_paths_valid(graph, owners[index]);
                    if (!valid) throw std::runtime_error("maze: initial graph route invalid");
                } catch (const std::runtime_error& error) {
                    if (!std::string_view(error.what()).starts_with("maze:")) throw;
                    owners = before; resources = occupancy;
                    debug::info_fmt("FPIA RRR: initial routing interrupted net_id={} reason={}", ni, error.what());
                    break;
                }
            }
        }
    } catch (const std::runtime_error& error) {
        if (!std::string_view(error.what()).starts_with("maze:")) throw;
        stop_reason = "unroutable";
        debug::info_fmt("FPIA RRR: initial routing failed: {}", error.what());
    }
    debug::info_fmt("FPIA RRR: initial overflow={} unequal_sync_groups={} sync_gap={} total_wirelength={} missing_owners={}",
        resources.overflow(), sync_violation(graph, nets, owners, interposer).unequal_groups,
        sync_violation(graph, nets, owners, interposer).total_gap,
        current_wirelength(graph, nets, owners), missing_owners(owners));
    checkpoint();
    bool stop = should_stop() || stop_reason == "unroutable";
    for (int sweep = 0; !stop && sweep < params.max_iterations; ++sweep) {
        const bool optimize = missing_owners(owners) == 0;
        if (optimize) ++optimization_rounds;
        auto sweep_order = std::Vector<std::size_t>{};
        for (bool connected : {false, true})
            for (auto ni : order)
                if (unit_connected(owners, ni) == connected) sweep_order.push_back(ni);
        bool improved = false;
        int attempts = 0, accepted = 0;
        for (auto ni : sweep_order) {
            for (int variant = 0; variant < 6; ++variant) {
                if (should_stop()) { stop = true; break; }
                auto trial = owners;
                auto trial_resources = resources;
                ++attempts;
                if (!transaction(ni, variant, graph, nets, trial, trial_resources, params, interposer)) continue;
                const int old_missing = missing_owners(owners), new_missing = missing_owners(trial);
                const auto old_w = current_wirelength(graph, nets, owners);
                const auto new_w = current_wirelength(graph, nets, trial);
                if (new_missing > old_missing || (new_missing == old_missing && new_w >= old_w)) continue;
                owners = std::move(trial); resources = std::move(trial_resources);
                improved = true; ++accepted;
                debug::info_fmt("FPIA RRR: transaction accept sweep={} net_id={} variant={} missing={}->{} wirelength={}->{}",
                    sweep, nets[ni].net_id, variant, old_missing, new_missing, old_w, new_w);
                checkpoint();
                if (should_stop()) stop = true;
                break;
            }
            if (stop) break;
        }
        // Never charge failed probes. Only the committed legal partial state supplies overflow.
        resources.history_next();
        double history_total = 0;
        auto history_keys = std::set<ResourceKey>{};
        for (const auto& owner : owners)
            for (auto key : owner.claimed) {
                if (is_mux_port_key(key)) key.extra = -1;
                if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap)
                    key = mode_conflict_key(key.id);
                history_keys.insert(key);
            }
        for (const auto& key : history_keys) history_total += resources.history(key);
        debug::info_fmt("FPIA RRR: history sweep={} committed_overflow={} total={} accepted={}",
            sweep, resources.overflow(), history_total, accepted);
        iterations = sweep + 1;
        debug::info_fmt(
            "FPIA RRR: iter={} overflow=0 new_overflow={} max_resource_overflow=0 dirty_owners={} rerouted={} total_wirelength={} unequal_sync_groups={} sync_gap={} H={} phase={} history_decay={} missing_owners={} accepted={}",
            sweep, resources.overflow(), sweep_order.size(), attempts,
            current_wirelength(graph, nets, owners),
            sync_violation(graph, nets, owners, interposer).unequal_groups,
            sync_violation(graph, nets, owners, interposer).total_gap,
            params.H, optimize ? "optimize" : "repair", params.decay, missing_owners(owners), accepted);
        if (verbose_level >= 1) dump_overflows(owners, nets, resources);
        checkpoint();
        stop = stop || should_stop();
        if (!budget_enabled && !stop && !improved) { stop_reason = "stagnated"; stop = true; }
        if (!budget_enabled && !stop && sweep + 1 >= 2) { stop_reason = "sweep_limit"; stop = true; }
    }
    if (has_legal) owners = best_owners;
    result.status = has_legal ? "success" : stop_reason;
    result.stop_reason = stop_reason;
    result.iterations = iterations;
    result.optimization_rounds = optimization_rounds;
    result.missing_owners = missing_owners(owners);
    // Rebuild after restoring an incumbent; absence of overflow alone is not success.
    resources = ResourceModel{params};
    for (const auto& owner : owners)
        resources.claim(owner.id, std::Vector<ResourceKey>(owner.claimed.begin(), owner.claimed.end()));
    const auto sync = sync_violation(graph, nets, owners, interposer);
    result.best_overflow = resources.overflow();
    result.unequal_sync_groups = sync.unequal_groups;
    result.total_sync_gap = sync.total_gap;
    result.paths = collect_paths_by_net(nets, owners);
    result.total_wirelength = total_wirelength(graph, result.paths);
    result.routing_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - routing_start).count();
    result.budget_elapsed_ms = elapsed_ms();
    debug::info_fmt(
        "FPIA RRR: status={} stop_reason={} iterations={} optimization_rounds={} best_overflow={} unequal_sync_groups={} sync_gap={} routing_ms={} elapsed_ms={} first_legal_ms={} best_legal_ms={} budget_elapsed_ms={} missing_owners={}",
        result.status, result.stop_reason, iterations, optimization_rounds, result.best_overflow,
        sync.unequal_groups, sync.total_gap, result.routing_ms, Elapsed::milliseconds(),
        result.first_legal_ms, result.best_legal_ms, result.budget_elapsed_ms, result.missing_owners);
    debug::info_fmt("routing result: total_wirelength={} RRR_routing_time={} elapsed_ms={}",
        result.total_wirelength, result.routing_ms, Elapsed::milliseconds());
    dump_paths(graph, nets, owners, interposer);
    return result;
}

} // namespace PR_tool
