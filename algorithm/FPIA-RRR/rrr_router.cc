#include "rrr_router.hh"
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
    int verbose_level
) -> RrrResult {
    using namespace rrr_detail;
    if (interposer == nullptr) {
        throw std::invalid_argument("RRR: run_rrr requires a non-null Interposer");
    }
    auto params = input_params;
    auto resources = ResourceModel {params};
    auto owners = build_owners(nets);
    const auto routing_start = std::chrono::steady_clock::now();
    const auto budget_start = params.budget_start == std::chrono::steady_clock::time_point {}
        ? routing_start : params.budget_start;
    const bool budget_enabled = params.time_budget_seconds > 0;
    auto elapsed_ms = [&]() -> std::i64 {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - budget_start).count();
    };
    auto budget_expired = [&]() -> bool {
        return budget_enabled && std::chrono::duration<double>(
            std::chrono::steady_clock::now() - budget_start).count() >= params.time_budget_seconds;
    };

    debug::info_fmt(
        "FPIA RRR: graph nodes={} arcs={} route_owners={}",
        graph.nodes.size(), graph.arcs.size(), owners.size());
    debug::info_fmt(
        "FPIA RRR: params max_iterations={} seed={} H={} k={} s={} sync_tail_extra_tracks={} time_budget_seconds={} optimization_excess_percent=20",
        params.max_iterations, params.seed, params.H, params.k, params.s,
        params.sync_tail_extra_tracks, params.time_budget_seconds);

    auto result = RrrResult {};
    auto best = BestSnapshot {};
    auto best_legal = BestSnapshot {};
    int stagnant = 0;
    int iterations = 0;
    int optimization_rounds = 0;
    bool retry_all = false;
    bool reported_overrun = false;
    auto stop_reason = std::String {"iteration_limit"};

    // Only complete, independently validated solutions enter the legal incumbent.
    auto checkpoint = [&]() -> bool {
        const int overflow = resources.overflow();
        const auto sync = sync_violation(graph, nets, owners, interposer);
        const auto wirelength = current_wirelength(graph, nets, owners);
        const bool connected = all_demands_connected(nets, owners);
        const bool improved = connected && save_best_if_improved(
            best, overflow, sync, wirelength, resources, owners);
        if (connected && overflow == 0 && sync.unequal_groups == 0
            && (!best_legal.valid || wirelength < best_legal.wirelength)) {
            auto candidate = RrrResult {};
            candidate.paths = collect_paths_by_net(nets, owners);
            if (validate_rrr_solution(graph, nets, candidate, interposer)) {
                save_best_if_improved(best_legal, overflow, sync, wirelength, resources, owners);
                result.best_legal_ms = elapsed_ms();
                if (result.first_legal_ms < 0) {
                    result.first_legal_ms = result.best_legal_ms;
                }
                debug::info_fmt(
                    "FPIA RRR: legal incumbent wirelength={} first_legal_ms={} best_legal_ms={}",
                    wirelength, result.first_legal_ms, result.best_legal_ms);
            }
        }
        debug::info_fmt(
            "FPIA RRR: checkpoint iterations={} budget_elapsed_ms={} has_legal={} best_legal_wirelength={}",
            iterations, elapsed_ms(), best_legal.valid, best_legal.valid ? best_legal.wirelength : 0);
        return improved;
    };
    auto should_stop = [&]() -> bool {
        if (budget_expired()) {
            if (best_legal.valid) {
                stop_reason = "time_budget";
                return true;
            }
            if (!reported_overrun) {
                debug::info_fmt("FPIA RRR: budget exceeded without legal solution; continue until first legal solution");
                reported_overrun = true;
            }
        }
        if (!budget_enabled && best_legal.valid) {
            stop_reason = "feasible";
            return true;
        }
        return false;
    };
    auto finish = [&]() -> RrrResult {
        const auto& saved = best_legal.valid ? best_legal : best;
        if (saved.valid) {
            resources = saved.resources;
            owners = saved.owners;
        }
        const auto sync = sync_violation(graph, nets, owners, interposer);
        result.status = best_legal.valid ? "success" : stop_reason;
        result.stop_reason = stop_reason;
        result.iterations = iterations;
        result.optimization_rounds = optimization_rounds;
        result.best_overflow = resources.overflow();
        result.unequal_sync_groups = sync.unequal_groups;
        result.total_sync_gap = sync.total_gap;
        result.paths = collect_paths_by_net(nets, owners);
        result.total_wirelength = total_wirelength(graph, result.paths);
        result.routing_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - routing_start).count();
        result.budget_elapsed_ms = elapsed_ms();
        debug::info_fmt(
            "FPIA RRR: status={} stop_reason={} iterations={} optimization_rounds={} best_overflow={} unequal_sync_groups={} sync_gap={} routing_ms={} elapsed_ms={} first_legal_ms={} best_legal_ms={} budget_elapsed_ms={}",
            result.status, result.stop_reason, result.iterations, result.optimization_rounds,
            result.best_overflow, result.unequal_sync_groups, result.total_sync_gap,
            result.routing_ms, Elapsed::milliseconds(), result.first_legal_ms,
            result.best_legal_ms, result.budget_elapsed_ms);
        debug::info_fmt(
            "routing result: total_wirelength={} RRR_routing_time={} elapsed_ms={}",
            result.total_wirelength, result.routing_ms, Elapsed::milliseconds());
        dump_paths(graph, nets, owners, interposer);
        if (verbose_level >= 1) {
            dump_overflows(owners, nets, resources);
        }
        return result;
    };
    auto reroute = [&](const std::Vector<OwnerId>& ids) -> int {
        int rerouted = 0;
        auto routed_sync = std::Set<std::size_t> {};
        for (const auto id : ids) {
            const auto index = find_owner_index(owners, id);
            if (owners[index].is_sync) {
                if (!routed_sync.insert(owners[index].net_index).second) {
                    continue;
                }
                route_sync_group(owners[index].net_index, owners, graph, nets,
                                 resources, params, interposer);
                rerouted += static_cast<int>(group_owner_indices(owners, owners[index].net_index).size());
            } else {
                route_owner(owners[index], graph, nets, resources, params, interposer);
                ++rerouted;
            }
        }
        return rerouted;
    };
    auto full_order = [&](int shift, const std::Set<std::size_t>* selected = nullptr) -> std::Vector<OwnerId> {
        auto order = initial_order(owners);
        if (selected != nullptr) {
            std::erase_if(order, [&](std::size_t index) {
                return !selected->contains(owners[index].net_index);
            });
        }
        auto starts = std::Vector<std::size_t> {};
        for (std::size_t i = 0; i < order.size(); ++i) {
            if (i == 0 || owners[order[i]].net_index != owners[order[i - 1]].net_index) {
                starts.push_back(i);
            }
        }
        // Rotate only at net/group boundaries, never inside a SyncNet.
        if (!starts.empty()) {
            const auto offset = starts[static_cast<std::size_t>(shift) % starts.size()];
            std::rotate(order.begin(), order.begin() + offset, order.end());
        }
        auto ids = std::Vector<OwnerId> {};
        for (const auto index : order) {
            ids.push_back(owners[index].id);
        }
        return ids;
    };

    const auto references = budget_enabled
        ? reference_wirelengths(graph, nets, params, interposer)
        : std::Vector<std::optional<std::size_t>> {};
    if (budget_enabled) {
        debug::info_fmt("FPIA RRR: reference complete budget_elapsed_ms={}", elapsed_ms());
    }

    try {
        reroute(full_order(0));
    }
    catch (const std::runtime_error& error) {
        debug::info_fmt("FPIA RRR: initial routing failed: {}", error.what());
        if (!budget_enabled) {
            stop_reason = "unroutable";
            return finish();
        }
        retry_all = true;
    }
    debug::info_fmt(
        "FPIA RRR: initial overflow={} unequal_sync_groups={} sync_gap={} total_wirelength={}",
        resources.overflow(), sync_violation(graph, nets, owners, interposer).unequal_groups,
        sync_violation(graph, nets, owners, interposer).total_gap, current_wirelength(graph, nets, owners));
    checkpoint();
    if (should_stop()) {
        const auto initial_sync = sync_violation(graph, nets, owners, interposer);
        debug::info_fmt(
            "FPIA RRR: iter=0 overflow={} new_overflow={} max_resource_overflow={} dirty_owners=0 rerouted=0 total_wirelength={} unequal_sync_groups={} sync_gap={} H={}",
            resources.overflow(), resources.overflow(), max_resource_overflow(owners, resources),
            current_wirelength(graph, nets, owners), initial_sync.unequal_groups,
            initial_sync.total_gap, params.H);
        return finish();
    }

    for (int iter = 0; iter < params.max_iterations; ++iter) {
        const int overflow = resources.overflow();
        const int max_ov = max_resource_overflow(owners, resources);
        const auto sync = sync_violation(graph, nets, owners, interposer);
        const bool sync_equal = sync.unequal_groups == 0;
        const bool optimize = budget_enabled && overflow == 0 && sync_equal
            && all_demands_connected(nets, owners) && !retry_all;

        resources.history_next();
        auto dirty_ids = std::Vector<OwnerId> {};
        if (optimize || retry_all) {
            if (optimize) {
                const auto selected = optimization_nets(graph, nets, owners, references);
                if (selected.empty()) {
                    stop_reason = "no_optimization_candidates";
                    debug::info_fmt("FPIA RRR: no optimization candidates above 20%; return best legal solution");
                    if (iterations == 0) {
                        debug::info_fmt(
                            "FPIA RRR: iter=0 overflow=0 new_overflow=0 max_resource_overflow=0 dirty_owners=0 rerouted=0 total_wirelength={} unequal_sync_groups=0 sync_gap=0 H={}",
                            current_wirelength(graph, nets, owners), params.H);
                    }
                    return finish();
                }
                ++optimization_rounds;
                dirty_ids = full_order(optimization_rounds, &selected);
            } else {
                dirty_ids = full_order(iter + 1);
            }
            debug::info_fmt(
                "FPIA RRR: schedule={} optimization_rounds={} first_net={}",
                optimize ? "optimize" : "retry", optimization_rounds,
                dirty_ids.empty() ? -1 : static_cast<int>(dirty_ids.front().net_id));
        } else {
            auto dirty_set = std::Set<OwnerId> {};
            for (const auto& owner : owners) {
                if (owner_is_dirty(owner, resources) && dirty_set.insert(owner.id).second) {
                    dirty_ids.push_back(owner.id);
                }
            }
            auto dirty_sync_nets = std::Set<std::size_t> {};
            for (const auto id : dirty_ids) {
                const std::size_t index = find_owner_index(owners, id);
                if (index < owners.size() && owners[index].is_sync) {
                    dirty_sync_nets.insert(owners[index].net_index);
                }
            }
            for (const auto& owner : owners) {
                if (owner.is_sync && dirty_sync_nets.contains(owner.net_index)
                    && dirty_set.insert(owner.id).second) {
                    dirty_ids.push_back(owner.id);
                }
            }
            if (!sync_equal) {
                for (const auto id : unequal_sync_owner_ids(graph, nets, owners, interposer)) {
                    if (dirty_set.insert(id).second) {
                        dirty_ids.push_back(id);
                    }
                }
            }

            std::sort(dirty_ids.begin(), dirty_ids.end(), [&](OwnerId lhs, OwnerId rhs) {
                const auto& a = owners[find_owner_index(owners, lhs)];
                const auto& b = owners[find_owner_index(owners, rhs)];
                return rrr_dirty_before(
                    owner_exposure(a, resources),
                    a.retry,
                    a.hpwl,
                    a.id,
                    owner_exposure(b, resources),
                    b.retry,
                    b.hpwl,
                    b.id);
            });

        }
        if (dirty_ids.empty()) {
            if (!budget_enabled) {
                stop_reason = "stagnated";
                break;
            }
            dirty_ids = full_order(iter + 1);
        }

        for (const auto id : dirty_ids) {
            rip_owner(owners[find_owner_index(owners, id)], resources);
        }
        int rerouted = 0;
        try {
            rerouted = reroute(dirty_ids);
            retry_all = false;
        }
        catch (const std::runtime_error& error) {
            debug::info_fmt("FPIA RRR: iteration={} routing failed: {}", iter + 1, error.what());
            iterations = iter + 1;
            if (!budget_enabled) {
                stop_reason = "unroutable";
                return finish();
            }
            // Never keep a partially routed round; restart from a complete snapshot.
            const auto& saved = best_legal.valid ? best_legal : best;
            if (saved.valid) {
                resources = saved.resources;
                owners = saved.owners;
            } else {
                resources = ResourceModel {params};
                owners = build_owners(nets);
            }
            retry_all = true;
            checkpoint();
            if (should_stop()) {
                return finish();
            }
            continue;
        }

        iterations = iter + 1;
        const bool improved = checkpoint();
        stagnant = improved ? 0 : stagnant + 1;
        constexpr int kMaxH = 16;
        if (!improved && stagnant >= 4 && params.H < kMaxH) {
            params.H = std::min(params.H + 4, kMaxH);
            stagnant = 0;
            debug::info_fmt("FPIA RRR: congestion boost H={}", params.H);
        }
        const auto new_sync = sync_violation(graph, nets, owners, interposer);
        debug::info_fmt(
            "FPIA RRR: iter={} overflow={} new_overflow={} max_resource_overflow={} dirty_owners={} rerouted={} total_wirelength={} unequal_sync_groups={} sync_gap={} H={} phase={}",
            iter, overflow, resources.overflow(), max_ov, dirty_ids.size(), rerouted,
            current_wirelength(graph, nets, owners), new_sync.unequal_groups,
            new_sync.total_gap, params.H, optimize ? "optimize" : "repair");
        if (verbose_level >= 1) {
            dump_overflows(owners, nets, resources);
        }
        if (should_stop()) {
            return finish();
        }
        if (!budget_enabled && params.H >= kMaxH && stagnant >= params.stagnation_limit) {
            stop_reason = "stagnated";
            break;
        }
    }
    return finish();
}

} // namespace PR_tool
