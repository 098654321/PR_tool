#include "sat/routing_feedback.hh"

#include "delay/pair_delay_precompute.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/routing_path_log.hh"
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

} // namespace

auto apply_feedback_expansion(
    RoutingProblemState& state,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<PairKey>& critical_pairs
) -> FeedbackExpansionStatus {
    if (all_pair_bboxes_full(state)) {
        return FeedbackExpansionStatus::Exhausted;
    }

    auto full_critical_nets = std::set<std::size_t> {};
    for (const auto& key : critical_pairs) {
        const auto* pair = find_pair_state(state, key);
        if (pair != nullptr && is_full_chip_bbox(pair->pair_bbox)) {
            full_critical_nets.insert(pair->key.net_id);
        }
    }

    auto touched_nets = std::set<std::size_t> {};
    if (!full_critical_nets.empty()) {
        for (auto& pair : state.pairs) {
            if (full_critical_nets.contains(pair.key.net_id)) {
                continue;
            }
            expand_pair_delays(pair);
            pair.pair_bbox = expand_pair_bbox_one_cell(pair.pair_bbox);
            touched_nets.insert(pair.key.net_id);
        }
    }
    else {
        for (const auto& key : critical_pairs) {
            auto* pair = find_pair_state(state, key);
            if (pair == nullptr) {
                continue;
            }
            expand_pair_delays(*pair);
            pair->pair_bbox = expand_pair_bbox_one_cell(pair->pair_bbox);
            touched_nets.insert(pair->key.net_id);
        }
    }

    if (touched_nets.empty()) {
        return FeedbackExpansionStatus::Exhausted;
    }

    for (const std::size_t net_id : touched_nets) {
        sync_fanout_delays(state, net_id);
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

    for (std::size_t round = 0; round < options.max_feedback_rounds; ++round) {
        if (options.verbose_level >= 1) {
            debug::info_fmt("feedback round={} begin", round);
        }

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
                return out;
            }

            const auto critical = collect_critical_pairs(model, solve_result, problem_state);
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
                return out;
            }
        }
        catch (const MemoryLimitExceeded&) {
            out.message = "MEMORY_LIMIT";
            out.num_vars = session.num_vars();
            out.num_clauses = session.num_clauses();
            out.feedback_rounds = round;
            return out;
        }
    }

    out.message = "UNSAT";
    out.feedback_rounds = options.max_feedback_rounds;
    debug::error_fmt(
        "unified SAT failed: exceeded max_feedback_rounds={}",
        options.max_feedback_rounds);
    return out;
}

} // namespace PR_tool
