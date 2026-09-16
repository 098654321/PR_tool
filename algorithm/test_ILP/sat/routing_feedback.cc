#include "sat/routing_feedback.hh"

#include "delay/pair_delay_precompute.hh"
#include "global_route_v17/global_router.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/routing_path_log.hh"
#include "sat/routing_round_diagnostics.hh"
#include "sat/routing_solution_validate.hh"
#include "sat/sat_encoding_stats.hh"
#include "sat/sat_solution_extract.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat/unified_sat_scope.hh"
#include "scope/build_routing_nets.hh"
#include "scope/pair_routing_state.hh"
#include "scope/scope_bbox.hh"

#include <algorithm>
#include <chrono>
#include <debug/debug.hh>
#include <format>
#include <optional>
#include <set>
#include <string_view>

namespace PR_tool {

namespace {

auto net_by_id(const std::Vector<RoutingNet>& nets, std::size_t net_id) -> const RoutingNet* {
    const auto it = std::find_if(
        nets.begin(),
        nets.end(),
        [&](const RoutingNet& net) { return net.net_id == net_id; });
    return it == nets.end() ? nullptr : &*it;
}

auto collect_critical_pairs(
    const UnifiedSatModel& model,
    const CadicalSolveResult& solve_result,
    const RoutingProblemState& state,
    const bool fallback_when_core_empty
) -> std::Vector<PairKey> {
    auto critical = std::Vector<PairKey> {};
    for (const auto& alpha : model.alpha_vars) {
        if (std::find(
                solve_result.failed_assumption_literals.begin(),
                solve_result.failed_assumption_literals.end(),
                alpha.alpha_lit)
            != solve_result.failed_assumption_literals.end()) {
            critical.push_back(alpha.key);
        }
    }
    if (!critical.empty() || !fallback_when_core_empty) {
        return critical;
    }
    debug::info("UNSAT with empty failed-assumption core; falling back to max-delay pair");
    const PairRoutingState* fallback = nullptr;
    for (const auto& pair : state.pairs) {
        if (fallback == nullptr || max_delay(pair.delays) > max_delay(fallback->delays)) {
            fallback = &pair;
        }
    }
    if (fallback != nullptr) {
        critical.push_back(fallback->key);
    }
    return critical;
}

auto collect_failed_unit_sources(
    const UnifiedSatModel& model,
    const CadicalSolveResult& solve_result
) -> std::set<std::pair<std::size_t, std::size_t>> {
    auto failed = std::set<std::pair<std::size_t, std::size_t>> {};
    for (const auto& gamma : model.unit_assumption_vars) {
        if (std::ranges::find(
                solve_result.failed_assumption_literals,
                gamma.assumption_lit)
            != solve_result.failed_assumption_literals.end()) {
            failed.emplace(gamma.net_id, gamma.source_index);
        }
    }
    return failed;
}

auto release_failed_global_units(
    std::Vector<RoutingNet>& nets,
    const std::set<std::pair<std::size_t, std::size_t>>& failed
) -> std::size_t {
    std::size_t released = 0;
    for (const auto& [net_id, source_index] : failed) {
        const auto net_it = std::find_if(
            nets.begin(),
            nets.end(),
            [&](const RoutingNet& net) { return net.net_id == net_id; });
        if (net_it == nets.end()) {
            continue;
        }
        released += net_it->released_global_unit_sources.insert(source_index).second ? 1 : 0;
        debug::info_fmt(
            "V18 unit assumption released: net={} source={} assigned_unit={}",
            net_id,
            source_index,
            net_it->global_unit_by_source.contains(source_index)
                ? std::to_string(net_it->global_unit_by_source.at(source_index))
                : "n/a");
    }
    return released;
}

auto apply_v18_pair_feedback(
    RoutingProblemState& state,
    const std::Vector<RoutingNet>& nets,
    const GlobalChannelGraph& channel_graph,
    const std::Vector<PairKey>& critical
) -> std::size_t {
    auto nets_to_expand = std::set<std::size_t> {};
    for (const auto& key : critical) {
        if (auto* pair = find_pair_state(state, key); pair != nullptr) {
            expand_pair_delay_one(*pair);
            nets_to_expand.insert(key.net_id);
        }
    }
    auto guide_pairs = std::Vector<PairKey> {};
    for (const auto net_id : nets_to_expand) {
        const int failure_count = ++state.feedback_failure_count_by_net[net_id];
        if (failure_count % 2 == 0) {
            const auto indices = state.pair_indices_by_net.find(net_id);
            if (indices != state.pair_indices_by_net.end()) {
                for (const auto index : indices->second) {
                    guide_pairs.push_back(state.pairs[index].key);
                }
            }
        }
        const auto net_it = std::find_if(
            nets.begin(),
            nets.end(),
            [&](const RoutingNet& net) { return net.net_id == net_id; });
        if (net_it != nets.end() && net_it->is_sync_bus) {
            sync_bus_after_expand(state, nets, net_id);
        }
    }
    return expand_global_route_guides_one_hop(channel_graph, state, guide_pairs);
}

auto critical_net_ids(const std::Vector<PairKey>& critical_pairs) -> std::set<std::size_t> {
    auto out = std::set<std::size_t> {};
    for (const auto& key : critical_pairs) {
        out.insert(key.net_id);
    }
    return out;
}

auto increment_feedback_failure_count(RoutingProblemState& state, std::size_t net_id) -> int {
    return ++state.feedback_failure_count_by_net[net_id];
}

} // namespace

auto apply_feedback_expansion(
    RoutingProblemState& state,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<PairKey>& critical_pairs
) -> FeedbackExpansionStatus {
    if (all_pair_bboxes_full(state)) {
        return FeedbackExpansionStatus::Exhausted;
    }

    const auto critical_nets = critical_net_ids(critical_pairs);
    auto full_critical_nets = std::set<std::size_t> {};
    for (const auto& key : critical_pairs) {
        const auto* pair = find_pair_state(state, key);
        if (pair != nullptr && is_full_chip_bbox(pair->pair_bbox)) {
            full_critical_nets.insert(pair->key.net_id);
        }
    }

    auto touched_nets = std::set<std::size_t> {};
    if (!full_critical_nets.empty()) {
        for (const std::size_t net_id : critical_nets) {
            if (full_critical_nets.contains(net_id)) {
                increment_feedback_failure_count(state, net_id);
            }
        }

        auto other_nets = std::set<std::size_t> {};
        for (const auto& pair : state.pairs) {
            if (!full_critical_nets.contains(pair.key.net_id)) {
                other_nets.insert(pair.key.net_id);
            }
        }
        for (const std::size_t net_id : other_nets) {
            const int failure_count = increment_feedback_failure_count(state, net_id);
            const auto net_it = state.pair_indices_by_net.find(net_id);
            if (net_it == state.pair_indices_by_net.end()) {
                continue;
            }
            for (const std::size_t pair_index : net_it->second) {
                apply_feedback_step_to_pair(state.pairs[pair_index], failure_count);
            }
            touched_nets.insert(net_id);
        }
    }
    else {
        for (const std::size_t net_id : critical_nets) {
            increment_feedback_failure_count(state, net_id);
        }
        for (const auto& key : critical_pairs) {
            auto* pair = find_pair_state(state, key);
            if (pair == nullptr) {
                continue;
            }
            const int failure_count = state.feedback_failure_count_by_net[key.net_id];
            apply_feedback_step_to_pair(*pair, failure_count);
            touched_nets.insert(key.net_id);
        }
    }

    if (touched_nets.empty()) {
        return FeedbackExpansionStatus::Exhausted;
    }

    for (const std::size_t net_id : touched_nets) {
        const auto* net = net_by_id(nets, net_id);
        if (net != nullptr && net->is_sync_bus) {
            sync_bus_after_expand(state, nets, net_id);
        }
    }
    return FeedbackExpansionStatus::Expanded;
}

auto solve_with_feedback(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult {
    if (options.enable_z3_optimize) {
        return solve_with_z3_optimize_feedback(interposer, basedie, options);
    }
    auto out = SatRoutingResult {};
    auto nets = build_routing_nets(basedie.nets_to_vector());
    auto problem_state = init_routing_problem_state(nets);
    apply_state_to_nets(problem_state, nets);

    auto graph = build_unified_graph(interposer, nets);
    augment_graph_for_pnnet(graph, nets);
    debug::info_fmt(
        "unified graph: nodes={} arcs={} track_nodes={} tob_nodes={}",
        graph.nodes.size(),
        graph.arcs.size(),
        graph.track_node_count,
        graph.tob_node_count);

    auto global_channel_graph = GlobalChannelGraph {};
    auto global_route = std::optional<GlobalRouteResult> {};
    const auto stamp_global_route = [&](SatRoutingResult& result) {
        if (!global_route.has_value()) {
            return;
        }
        result.global_route_requested = true;
        result.global_route_status = global_route->ok ? "OPTIMAL" : global_route->message;
        result.global_route_nodes = global_route->stats.nodes;
        result.global_route_cob_nodes = global_route->stats.cob_nodes;
        result.global_route_tob_terminal_nodes = global_route->stats.tob_terminal_nodes;
        result.global_route_port_terminal_nodes = global_route->stats.port_terminal_nodes;
        result.global_route_boundary_terminal_nodes = global_route->stats.boundary_terminal_nodes;
        result.global_route_channels = global_route->stats.channels;
        result.global_route_arcs = global_route->stats.arcs;
        result.global_route_owners = global_route->stats.owners;
        result.global_route_commodities = global_route->stats.commodities;
        result.global_route_vars = global_route->stats.variables;
        result.global_route_constraints = global_route->stats.constraints;
        result.global_route_objective = global_route->stats.objective;
        result.global_route_estimated_wirelength =
            global_route->stats.estimated_wirelength;
        result.global_route_capacity_cuts_enabled =
            global_route->stats.capacity_cuts_enabled;
        result.global_route_capacity_cut_rounds =
            global_route->stats.capacity_cut_rounds;
        result.global_route_capacity_cuts = global_route->stats.capacity_cuts;
        result.global_route_build_ms = global_route->stats.build_ms;
        result.global_route_solve_ms = global_route->stats.solve_ms;
        result.global_route_total_ms = global_route->stats.total_ms;
        result.global_route_released_sources = 0;
        for (const auto& net : nets) {
            result.global_route_released_sources += net.released_global_unit_sources.size();
        }
    };
    if (options.enable_global_route_v18) {
        global_channel_graph = build_global_channel_graph(graph, nets);
        global_route = solve_global_route_v17(
            graph,
            global_channel_graph,
            nets,
            options.verbose_level,
            GlobalRouteCapacityMode::IterativeCuts);
        if (!global_route->ok) {
            debug::error(
                "V18 front-end produced no guide; this is not a proof that the full detailed-routing design is UNSAT");
            out.message = std::format("GLOBAL_ROUTE_{}", global_route->message);
            stamp_global_route(out);
            return out;
        }
        apply_global_route_v17(*global_route, problem_state, nets);
        debug::info_fmt(
            "V18 guide initialization: guided_pairs={} assigned_units={} objective_channels={}",
            global_route->pair_channels.size(),
            global_route->unit_by_owner.size(),
            global_route->stats.objective);
    }
    log_scope_bboxes(nets, options.verbose_level);
    const auto sat_begin = std::chrono::steady_clock::now();

    if (options.initial_scope_pad > 0 || options.initial_delay_pad > 0) {
        apply_initial_search_padding(
            problem_state,
            nets,
            graph,
            options.initial_scope_pad,
            options.initial_delay_pad);
        if (options.verbose_level >= 1) {
            debug::info("scope after initial search padding:");
            log_scope_bboxes(nets, options.verbose_level);
        }
    }

    const auto stamp_sat_timing = [&](SatRoutingResult& result) {
        const auto sat_end = std::chrono::steady_clock::now();
        result.sat_total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            sat_end - sat_begin).count();
        result.sat_pre_ms = result.sat_total_ms - result.solve_ms;
        if (result.sat_pre_ms < 0) {
            result.sat_pre_ms = 0;
        }
        stamp_global_route(result);
    };

    for (std::size_t round = 0; round < options.max_feedback_rounds; ++round) {
        log_feedback_round_begin(round);

        const auto precompute_begin = std::chrono::steady_clock::now();
        apply_state_to_nets(problem_state, nets);
        const auto scopes = build_all_scopes(graph, nets);
        auto delays_holder = std::optional<DelayPrecomputeResult> {};
        try {
            delays_holder = compute_pair_delays(graph, nets, scopes, &problem_state);
        }
        catch (const std::runtime_error& error) {
            if (!options.enable_global_route_v18
                || std::string_view {error.what()}.find("has no scoped path")
                    == std::string_view::npos) {
                throw;
            }
            auto all_pairs = std::Vector<PairKey> {};
            all_pairs.reserve(problem_state.pairs.size());
            for (const auto& pair : problem_state.pairs) {
                all_pairs.push_back(pair.key);
            }
            const auto added = expand_global_route_guides_one_hop(
                global_channel_graph, problem_state, all_pairs);
            apply_state_to_nets(problem_state, nets);
            debug::info_fmt(
                "V18 scoped delay precompute had no detailed path: round={} guide_channels_added={} reason={}",
                round,
                added,
                error.what());
            if (added == 0) {
                out.message = std::format("DETAILED_SCOPE_UNREACHABLE: {}", error.what());
                log_feedback_round_end(round, FeedbackRoundStatus::UnsatExhausted);
                stamp_sat_timing(out);
                return out;
            }
            log_feedback_round_end(round, FeedbackRoundStatus::UnsatExpand);
            continue;
        }
        const auto& delays = *delays_holder;
        const auto precompute_end = std::chrono::steady_clock::now();
        const auto precompute_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                precompute_end - precompute_begin)
                .count();
        debug::info_fmt(
            "delay precompute finished: round={} sources={} pairs={} delay_precompute_ms={}",
            round,
            delays.sources.size(),
            delays.pairs.size(),
            precompute_ms);
        log_delay_precompute(nets, delays, options.verbose_level);

        auto session = CadicalSession {options.cadical};
        long long solve_ms = 0;
        try {
            debug::info("streaming unified numeric SAT model into CaDiCal...");
            SatEncodingStats encoding_stats {};
            SatEncodingStats* stats_ptr = options.verbose_level >= 1 ? &encoding_stats : nullptr;
            const auto model_build_begin = std::chrono::steady_clock::now();
            const auto model = build_unified_sat_model(
                session,
                graph,
                nets,
                scopes,
                delays,
                stats_ptr);
            const auto model_build_end = std::chrono::steady_clock::now();
            const auto model_build_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    model_build_end - model_build_begin)
                    .count();
            debug::info_fmt("unified SAT model built: model_build_ms={}", model_build_ms);
            if (stats_ptr != nullptr) {
                encoding_stats.finalize_variables(session.num_vars());
                log_sat_encoding_stats(
                    encoding_stats,
                    session.num_vars(),
                    session.num_clauses());
            }

            for (const auto& alpha : model.alpha_vars) {
                session.assume(alpha.alpha_lit);
            }
            for (const auto& gamma : model.unit_assumption_vars) {
                session.assume(gamma.assumption_lit);
            }

            debug::info_fmt(
                "solving unified SAT with CaDiCal: round={} vars={} hard_clauses={} alpha_assumptions={} unit_assumptions={} occupancy_soft_clauses=0",
                round,
                session.num_vars(),
                session.num_clauses(),
                model.alpha_vars.size(),
                model.unit_assumption_vars.size());
            const auto solve_begin = std::chrono::steady_clock::now();
            const auto solve_result = session.solve();
            const auto solve_end = std::chrono::steady_clock::now();
            solve_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_begin)
                    .count();

            out.num_vars = session.num_vars();
            out.num_clauses = session.num_clauses();
            out.solve_ms += solve_ms;
            out.feedback_rounds = round;

            if (solve_result.memory_limit_exceeded) {
                out.message = "MEMORY_LIMIT";
                debug::error_fmt(
                    "unified SAT failed: MEMORY_LIMIT (vars={} clauses={} round={})",
                    out.num_vars,
                    out.num_clauses,
                    round);
                log_feedback_round_end(round, FeedbackRoundStatus::MemoryLimit);
                stamp_sat_timing(out);
                return out;
            }

            if (solve_result.ok) {
                const long long total_solve_ms = out.solve_ms;
                out = extract_sat_solution(graph, nets, model, session, solve_result);
                out.num_vars = session.num_vars();
                out.num_clauses = session.num_clauses();
                out.solve_ms = total_solve_ms;
                out.feedback_rounds = round;
                out.total_wirelength = total_wirelength(graph, out);
                log_routing_paths(graph, nets, out);
                const auto validation = validate_routing_solution(graph, nets, model, session, out);
                log_validation_report(validation, options.verbose_level);
                if (!validation.pass) {
                    out.ok = false;
                    out.message = "PHYSICAL_VALIDATION_FAILED";
                    debug::error_fmt(
                        "unified SAT physical validation failed: violations={} round={}",
                        validation.violations_count,
                        round);
                    stamp_sat_timing(out);
                    log_feedback_round_end(round, FeedbackRoundStatus::SolverError);
                    return out;
                }
                debug::info_fmt(
                    "unified SAT ok: paths={} vars={} clauses={} round_solve_ms={} total_solve_ms={} total_wirelength={} round={}",
                    out.paths.size(),
                    out.num_vars,
                    out.num_clauses,
                    solve_ms,
                    out.solve_ms,
                    out.total_wirelength,
                    round);
                log_non_shortest_nets(interposer, graph, nets, delays, out);
                stamp_sat_timing(out);
                log_feedback_round_end(round, FeedbackRoundStatus::SatSuccess);
                return out;
            }

            if (solve_result.message != "UNSAT") {
                out.message = solve_result.message;
                debug::error_fmt(
                    "unified SAT failed: {} (vars={} clauses={} round={})",
                    out.message,
                    out.num_vars,
                    out.num_clauses,
                    round);
                log_feedback_round_end(round, FeedbackRoundStatus::SolverError);
                stamp_sat_timing(out);
                return out;
            }

            const auto critical = collect_critical_pairs(
                model,
                solve_result,
                problem_state,
                !options.enable_global_route_v18);
            const auto failed_units = options.enable_global_route_v18
                ? collect_failed_unit_sources(model, solve_result)
                : std::set<std::pair<std::size_t, std::size_t>> {};
            if (options.enable_global_route_v18) {
                debug::info_fmt(
                    "V18 CaDiCaL hard-UNSAT: vars={} clauses={} core_size={} alpha_core_pairs={} unit_core_sources={} round_solve_ms={} total_solve_ms={} round={}",
                    out.num_vars,
                    out.num_clauses,
                    solve_result.failed_assumption_literals.size(),
                    critical.size(),
                    failed_units.size(),
                    solve_ms,
                    out.solve_ms,
                    round);
                if (critical.empty() && failed_units.empty()) {
                    out.message = "BASE_HARD_UNSAT";
                    debug::error(
                        "V18 CaDiCaL hard constraints are UNSAT without alpha/unit core");
                    log_feedback_round_end(round, FeedbackRoundStatus::SolverError);
                    stamp_sat_timing(out);
                    return out;
                }
            }
            if (!critical.empty()) {
                log_failed_nets(nets, critical);
            }
            if (options.verbose_level >= 1) {
                for (const auto& key : critical) {
                    const auto* pair = find_pair_state(problem_state, key);
                    if (pair == nullptr) {
                        continue;
                    }
                    debug::info_fmt(
                        "feedback critical net={} demand={} delays=[{}] bbox={}",
                        key.net_id,
                        key.demand_id,
                        [&] {
                            auto text = std::String {};
                            for (std::size_t i = 0; i < pair->delays.size(); ++i) {
                                if (i != 0) {
                                    text += ",";
                                }
                                text += std::to_string(pair->delays[i]);
                            }
                            return text;
                        }(),
                        format_bbox(pair->pair_bbox));
                }
            }

            const auto released = release_failed_global_units(nets, failed_units);
            if (options.enable_global_route_v18) {
                const auto added_channels = apply_v18_pair_feedback(
                    problem_state, nets, global_channel_graph, critical);
                apply_state_to_nets(problem_state, nets);
                debug::info_fmt(
                    "V18 feedback applied: critical_pairs={} released_unit_sources={} guide_channels_added={} guides_full={}",
                    critical.size(),
                    released,
                    added_channels,
                    all_global_route_guides_full(
                        problem_state, global_channel_graph.channels.size()));
            }
            else if (apply_feedback_expansion(problem_state, nets, critical)
                     == FeedbackExpansionStatus::Exhausted) {
                out.message = "UNSAT";
                debug::error_fmt(
                    "unified SAT failed: UNSAT after full-chip bbox expansion (round={})",
                    round);
                log_feedback_round_end(round, FeedbackRoundStatus::UnsatExhausted);
                stamp_sat_timing(out);
                return out;
            }
            log_feedback_round_end(round, FeedbackRoundStatus::UnsatExpand);
        }
        catch (const MemoryLimitExceeded&) {
            out.message = "MEMORY_LIMIT";
            out.num_vars = session.num_vars();
            out.num_clauses = session.num_clauses();
            out.feedback_rounds = round;
            log_feedback_round_end(round, FeedbackRoundStatus::MemoryLimit);
            stamp_sat_timing(out);
            return out;
        }
    }

    out.message = options.enable_global_route_v18 ? "SEARCH_LIMIT" : "UNSAT";
    out.feedback_rounds = options.max_feedback_rounds;
    debug::error_fmt(
        "unified SAT failed: exceeded max_feedback_rounds={}",
        options.max_feedback_rounds);
    log_feedback_round_end(
        options.max_feedback_rounds,
        FeedbackRoundStatus::MaxRoundsExceeded);
    stamp_sat_timing(out);
    return out;
}

} // namespace PR_tool
