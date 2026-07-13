#include "sat/routing_feedback.hh"

#include "delay/pair_delay_precompute.hh"
#include "graph/unified_routing_graph.hh"
#include "ilp_v15/v15_ilp_optimizer.hh"
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
#include <set>

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
    const RoutingProblemState& state
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
    if (!critical.empty()) {
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
    const auto sat_begin = std::chrono::steady_clock::now();
    auto out = SatRoutingResult {};
    auto nets = build_routing_nets(basedie.nets_to_vector());
    auto problem_state = init_routing_problem_state(nets);
    apply_state_to_nets(problem_state, nets);
    log_scope_bboxes(nets, options.verbose_level);

    auto graph = build_unified_graph(interposer, nets);
    augment_graph_for_pnnet(graph, nets);
    debug::info_fmt(
        "unified graph: nodes={} arcs={} track_nodes={} tob_nodes={}",
        graph.nodes.size(),
        graph.arcs.size(),
        graph.track_node_count,
        graph.tob_node_count);

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
    };

    for (std::size_t round = 0; round < options.max_feedback_rounds; ++round) {
        log_feedback_round_begin(round);

        const auto precompute_begin = std::chrono::steady_clock::now();
        apply_state_to_nets(problem_state, nets);
        const auto scopes = build_all_scopes(graph, nets);
        const auto delays = compute_pair_delays(graph, nets, scopes, &problem_state);
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

            debug::info_fmt(
                "solving unified SAT with CaDiCal (assumptions={})...",
                model.alpha_vars.size());
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
                if (options.ilp_optimize.enabled) {
                    const auto sat_total_ms = out.sat_total_ms;
                    const auto sat_pre_ms = out.sat_pre_ms;
                    const auto sat_solve_ms = out.solve_ms;
                    const auto ilp = optimize_v15_routes(
                        interposer,
                        graph,
                        nets,
                        scopes,
                        delays,
                        out,
                        options.ilp_optimize);
                    out = ilp.routing;
                    out.solve_ms = sat_solve_ms;
                    out.sat_total_ms = sat_total_ms;
                    out.sat_pre_ms = sat_pre_ms;
                    out.ilp_optimization_requested = true;
                    out.ilp_optimization_applied = ilp.status == V15IlpStatus::Optimal
                        || ilp.status == V15IlpStatus::Suboptimal;
                    out.ilp_fallback_to_sat = ilp.status == V15IlpStatus::Failed;
                    out.ilp_status = ilp.message;
                    out.ilp_model_vars = ilp.stats.f_vars + ilp.stats.x_vars
                        + ilp.stats.y_vars + ilp.stats.mode_vars;
                    out.ilp_model_constraints = ilp.stats.constraints;
                    out.ilp_model_build_ms = ilp.stats.model_build_ms;
                    out.ilp_solve_ms = ilp.stats.solve_ms;
                    out.ilp_total_ms = ilp.stats.total_ms;
                    out.ilp_pre_ms = ilp.stats.pre_ms;
                    if (ilp.status == V15IlpStatus::Optimal
                        || ilp.status == V15IlpStatus::Suboptimal) {
                        log_routing_paths(graph, nets, out);
                        log_non_shortest_nets(interposer, graph, nets, delays, out);
                    }
                }
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

            const auto critical = collect_critical_pairs(model, solve_result, problem_state);
            log_failed_nets(nets, critical);
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

            if (apply_feedback_expansion(problem_state, nets, critical)
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

    out.message = "UNSAT";
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
