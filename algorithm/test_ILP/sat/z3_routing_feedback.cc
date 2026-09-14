#include "sat/routing_feedback.hh"

#include "delay/pair_delay_precompute.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/node_occupancy.hh"
#include "sat/routing_path_log.hh"
#include "sat/routing_round_diagnostics.hh"
#include "sat/routing_solution_validate.hh"
#include "sat/sat_encoding_stats.hh"
#include "sat/sat_solution_extract.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat/unified_sat_scope.hh"
#include "sat_allocation/z3_optimize_solver.hh"
#include "scope/build_routing_nets.hh"
#include "scope/pair_routing_state.hh"
#include "scope/scope_bbox.hh"

#include <algorithm>
#include <chrono>
#include <debug/debug.hh>
#include <string>

namespace PR_tool {

namespace {

auto collect_critical_pairs(
    const UnifiedSatModel& model,
    const std::Vector<int>& failed_literals
) -> std::Vector<PairKey> {
    auto critical = std::Vector<PairKey> {};
    for (const auto& alpha : model.alpha_vars) {
        if (std::find(failed_literals.begin(), failed_literals.end(), alpha.alpha_lit)
            != failed_literals.end()) {
            critical.push_back(alpha.key);
        }
    }
    return critical;
}

} // namespace

auto solve_with_z3_optimize_feedback(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult {
#ifndef USE_Z3
    (void)interposer;
    (void)basedie;
    (void)options;
    auto out = SatRoutingResult {};
    out.message = "Z3 backend is unavailable; install Z3 or configure xmake with --z3=y";
    debug::error(out.message);
    return out;
#else
    const auto solve_begin = std::chrono::steady_clock::now();
    auto out = SatRoutingResult {};
    auto nets = build_routing_nets(basedie.nets_to_vector());
    auto state = init_routing_problem_state(nets);
    apply_state_to_nets(state, nets);
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
            state, nets, graph, options.initial_scope_pad, options.initial_delay_pad);
        if (options.verbose_level >= 1) {
            debug::info("scope after initial search padding:");
            log_scope_bboxes(nets, options.verbose_level);
        }
    }

    const auto stamp_timing = [&](SatRoutingResult& result) {
        const auto end = std::chrono::steady_clock::now();
        result.sat_total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - solve_begin).count();
        result.sat_pre_ms = result.sat_total_ms - result.solve_ms;
        if (result.sat_pre_ms < 0) {
            result.sat_pre_ms = 0;
        }
    };

    for (std::size_t round = 0; round < options.max_feedback_rounds; ++round) {
        log_feedback_round_begin(round);
        const auto precompute_begin = std::chrono::steady_clock::now();
        apply_state_to_nets(state, nets);
        const auto scopes = build_all_scopes(graph, nets);
        const auto delays = compute_pair_delays(graph, nets, scopes, &state);
        const auto precompute_end = std::chrono::steady_clock::now();
        const auto precompute_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            precompute_end - precompute_begin).count();
        debug::info_fmt(
            "delay precompute finished: round={} sources={} pairs={} delay_precompute_ms={}",
            round,
            delays.sources.size(),
            delays.pairs.size(),
            precompute_ms);
        log_delay_precompute(nets, delays, options.verbose_level);

        auto capture_options = options.cadical;
        capture_options.capture_clauses = true;
        auto session = CadicalSession {capture_options};
        try {
            debug::info("capturing unified numeric SAT hard clauses for Z3 Optimize...");
            SatEncodingStats encoding_stats {};
            auto* stats = options.verbose_level >= 1 ? &encoding_stats : nullptr;
            const auto model_build_begin = std::chrono::steady_clock::now();
            const auto model = build_unified_sat_model(session, graph, nets, scopes, delays, stats);
            const auto occupancy = add_node_occupancy_variables(session, graph, model);
            const auto model_build_end = std::chrono::steady_clock::now();
            const auto model_build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                model_build_end - model_build_begin).count();

            auto request = Z3OptimizeRequest {};
            request.num_vars = session.num_vars();
            request.hard_clauses = session.clauses();
            request.external_assumptions.reserve(model.alpha_vars.size());
            for (const auto& alpha : model.alpha_vars) {
                request.external_assumptions.push_back(alpha.alpha_lit);
            }
            request.soft_negated_vars.reserve(occupancy.u_var_by_node.size());
            for (const auto& [_, u] : occupancy.u_var_by_node) {
                request.soft_negated_vars.push_back(u);
            }
            out.num_vars = request.num_vars;
            out.num_clauses = session.num_clauses();
            out.feedback_rounds = round;
            if (stats != nullptr) {
                encoding_stats.occupancy_vars = occupancy.u_var_by_node.size();
                encoding_stats.occupancy_implication_clauses = occupancy.implication_clause_count;
                encoding_stats.add_clauses(
                    SatClauseCategory::NodeOccupancy,
                    occupancy.implication_clause_count);
                encoding_stats.finalize_variables(out.num_vars);
                log_sat_encoding_stats(encoding_stats, out.num_vars, out.num_clauses);
            }
            debug::info_fmt(
                "unified Z3 model built: model_build_ms={} occupancy_vars={} occupancy_implication_clauses={}",
                model_build_ms,
                occupancy.u_var_by_node.size(),
                occupancy.implication_clause_count);

            debug::info_fmt(
                "solving Z3 Optimize: round={} vars={} hard_clauses={} alpha_assumptions={} occupancy_soft_clauses={}",
                round,
                out.num_vars,
                out.num_clauses,
                request.external_assumptions.size(),
                request.soft_negated_vars.size());
            const auto round_begin = std::chrono::steady_clock::now();
            const auto result = solve_z3_optimize(request);
            const auto round_end = std::chrono::steady_clock::now();
            const auto round_ms = std::chrono::duration_cast<std::chrono::milliseconds>(round_end - round_begin).count();
            out.solve_ms += round_ms;

            if (result.status == Z3OptimizeStatus::Optimal) {
                const auto total_solve_ms = out.solve_ms;
                const auto num_vars = out.num_vars;
                const auto num_clauses = out.num_clauses;
                out = extract_sat_solution(
                    graph,
                    nets,
                    model,
                    [&](const int variable) { return result.value(variable); },
                    num_vars,
                    num_clauses);
                out.solve_ms = total_solve_ms;
                out.feedback_rounds = round;
                out.total_wirelength = total_wirelength(graph, out);
                if (out.total_wirelength != result.objective_cost) {
                    out.ok = false;
                    out.message = "Z3 occupancy objective does not match reconstructed union wirelength";
                    debug::error_fmt(
                        "Z3 objective invariant failed: objective={} reconstructed_wirelength={}",
                        result.objective_cost,
                        out.total_wirelength);
                    stamp_timing(out);
                    return out;
                }
                log_routing_paths(graph, nets, out);
                const auto validation = validate_routing_solution(
                    graph,
                    nets,
                    model,
                    [&](const int variable) { return result.value(variable); },
                    out);
                log_validation_report(validation, options.verbose_level);
                log_non_shortest_nets(interposer, graph, nets, delays, out);
                stamp_timing(out);
                debug::info_fmt(
                    "unified Z3 Optimize optimum: paths={} vars={} clauses={} objective={} total_wirelength={} round_solve_ms={} total_solve_ms={} routing_total_ms={} round={}",
                    out.paths.size(),
                    out.num_vars,
                    out.num_clauses,
                    result.objective_cost,
                    out.total_wirelength,
                    round_ms,
                    out.solve_ms,
                    out.sat_total_ms,
                    round);
                log_feedback_round_end(round, FeedbackRoundStatus::SatSuccess);
                return out;
            }

            if (result.status == Z3OptimizeStatus::Unknown) {
                out.message = result.message;
                debug::error_fmt(
                    "unified Z3 Optimize failed without proof: {} (vars={} clauses={} round_solve_ms={} total_solve_ms={} round={})",
                    out.message,
                    out.num_vars,
                    out.num_clauses,
                    round_ms,
                    out.solve_ms,
                    round);
                log_feedback_round_end(round, FeedbackRoundStatus::SolverError);
                stamp_timing(out);
                return out;
            }

            const auto critical = collect_critical_pairs(model, result.failed_assumption_literals);
            debug::info_fmt(
                "unified Z3 Optimize hard-UNSAT: vars={} clauses={} alpha_core_size={} round_solve_ms={} total_solve_ms={} round={}",
                out.num_vars,
                out.num_clauses,
                result.failed_assumption_literals.size(),
                round_ms,
                out.solve_ms,
                round);
            if (critical.empty()) {
                out.message = "BASE_HARD_UNSAT";
                debug::error("unified Z3 Optimize hard constraints are UNSAT without alpha core");
                log_feedback_round_end(round, FeedbackRoundStatus::SolverError);
                stamp_timing(out);
                return out;
            }
            log_failed_nets(nets, critical);
            if (options.verbose_level >= 1) {
                for (const auto& key : critical) {
                    const auto* pair = find_pair_state(state, key);
                    if (pair == nullptr) {
                        continue;
                    }
                    auto delay_text = std::String {};
                    for (std::size_t index = 0; index < pair->delays.size(); ++index) {
                        if (index != 0) {
                            delay_text += ",";
                        }
                        delay_text += std::to_string(pair->delays[index]);
                    }
                    debug::info_fmt(
                        "feedback critical net={} demand={} delays=[{}] bbox={}",
                        key.net_id,
                        key.demand_id,
                        delay_text,
                        format_bbox(pair->pair_bbox));
                }
            }
            if (apply_feedback_expansion(state, nets, critical) == FeedbackExpansionStatus::Exhausted) {
                out.message = "UNSAT";
                log_feedback_round_end(round, FeedbackRoundStatus::UnsatExhausted);
                stamp_timing(out);
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
            stamp_timing(out);
            return out;
        }
    }
    out.message = "UNSAT";
    out.feedback_rounds = options.max_feedback_rounds;
    log_feedback_round_end(options.max_feedback_rounds, FeedbackRoundStatus::MaxRoundsExceeded);
    stamp_timing(out);
    return out;
#endif
}

} // namespace PR_tool
