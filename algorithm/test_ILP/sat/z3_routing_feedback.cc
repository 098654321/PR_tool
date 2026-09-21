#include "sat/routing_feedback.hh"

#include "delay/pair_delay_precompute.hh"
#include "global_route_v17/global_router.hh"
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
#include <optional>
#include <set>
#include <string>
#include <string_view>

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

auto collect_failed_unit_sources(
    const UnifiedSatModel& model,
    const std::Vector<int>& failed_literals
) -> std::set<std::pair<std::size_t, std::size_t>> {
    auto failed = std::set<std::pair<std::size_t, std::size_t>> {};
    for (const auto& gamma : model.unit_assumption_vars) {
        if (std::find(
                failed_literals.begin(),
                failed_literals.end(),
                gamma.assumption_lit)
            != failed_literals.end()) {
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
            "V17 unit assumption released: net={} source={} assigned_unit={}",
            net_id,
            source_index,
            net_it->global_unit_by_source.contains(source_index)
                ? std::to_string(net_it->global_unit_by_source.at(source_index))
                : "n/a");
    }
    return released;
}

auto apply_v17_pair_feedback(
    RoutingProblemState& state,
    const std::Vector<RoutingNet>& nets,
    const GlobalChannelGraph& channel_graph,
    const std::Vector<PairKey>& critical
) -> std::size_t {
    return apply_global_route_feedback_step(channel_graph, state, nets, critical).added_channels;
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
    auto out = SatRoutingResult {};
    auto nets = build_routing_nets(basedie.nets_to_vector());
    auto state = init_routing_problem_state(nets);
    apply_state_to_nets(state, nets);
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
    if (options.enable_global_route_v17) {
        global_channel_graph = build_global_channel_graph(graph, nets);
        global_route = solve_global_route_v17(
            graph,
            global_channel_graph,
            nets,
            options.verbose_level,
            GlobalRouteCapacityMode::DenseW,
            options.highs_log_path,
            false,
            GlobalRouteScopeMode::FullGraph,
            options.highs_time_limit_minutes);
        if (!global_route->ok) {
            debug::error(
                "V17 front-end produced no guide; this is not a proof that the full detailed-routing design is UNSAT");
            out.message = std::format("GLOBAL_ROUTE_{}", global_route->message);
            out.global_route_requested = true;
            out.global_route_status = global_route->message;
            out.global_route_nodes = global_route->stats.nodes;
            out.global_route_cob_nodes = global_route->stats.cob_nodes;
            out.global_route_tob_terminal_nodes = global_route->stats.tob_terminal_nodes;
            out.global_route_port_terminal_nodes = global_route->stats.port_terminal_nodes;
            out.global_route_boundary_terminal_nodes = global_route->stats.boundary_terminal_nodes;
            out.global_route_channels = global_route->stats.channels;
            out.global_route_arcs = global_route->stats.arcs;
            out.global_route_owners = global_route->stats.owners;
            out.global_route_commodities = global_route->stats.commodities;
            out.global_route_vars = global_route->stats.variables;
            out.global_route_constraints = global_route->stats.constraints;
            out.global_route_total_ms = global_route->stats.total_ms;
            out.global_route_build_ms = global_route->stats.build_ms;
            out.global_route_solve_ms = global_route->stats.solve_ms;
            return out;
        }
        apply_global_route_v17(*global_route, global_channel_graph, state, nets);
        debug::info_fmt(
            "V17 guide initialization: guided_pairs={} assigned_units={} objective_channels={}",
            global_route->pair_channels.size(),
            global_route->unit_by_owner.size(),
            global_route->stats.objective);
    }
    log_scope_bboxes(nets, options.verbose_level);
    const auto solve_begin = std::chrono::steady_clock::now();

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
        if (global_route.has_value()) {
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
        }
    };

    for (std::size_t round = 0; round < options.max_feedback_rounds; ++round) {
        log_feedback_round_begin(round);
        const auto precompute_begin = std::chrono::steady_clock::now();
        apply_state_to_nets(state, nets);
        const auto scopes = build_all_scopes(graph, nets);
        auto delays_holder = std::optional<DelayPrecomputeResult> {};
        try {
            delays_holder = compute_pair_delays(graph, nets, scopes, &state);
        }
        catch (const ScopedPathUnavailable& error) {
            if (!options.enable_global_route_v17) {
                throw;
            }
            const auto expansion = expand_global_route_guides_one_hop(
                global_channel_graph, state, {error.pair_key});
            apply_state_to_nets(state, nets);
            debug::info_fmt(
                "V17 scoped delay precompute had no detailed path: round={} net={} demand={} source={} pair_local_channels_added={} net_union_channels_added={} reason={}",
                round,
                error.pair_key.net_id,
                error.pair_key.demand_id,
                error.pair_key.source_index,
                expansion.pair_local_added_channels,
                expansion.added_channels,
                error.what());
            if (expansion.pair_local_added_channels == 0) {
                out.message = std::format("DETAILED_SCOPE_UNREACHABLE: {}", error.what());
                log_feedback_round_end(round, FeedbackRoundStatus::UnsatExhausted);
                stamp_timing(out);
                return out;
            }
            log_feedback_round_end(round, FeedbackRoundStatus::UnsatExpand);
            continue;
        }
        const auto& delays = *delays_holder;
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
            request.external_assumptions.reserve(
                model.alpha_vars.size() + model.unit_assumption_vars.size());
            for (const auto& alpha : model.alpha_vars) {
                request.external_assumptions.push_back(alpha.alpha_lit);
            }
            for (const auto& gamma : model.unit_assumption_vars) {
                request.external_assumptions.push_back(gamma.assumption_lit);
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
                "solving Z3 Optimize: round={} vars={} hard_clauses={} alpha_assumptions={} unit_assumptions={} occupancy_soft_clauses={}",
                round,
                out.num_vars,
                out.num_clauses,
                model.alpha_vars.size(),
                model.unit_assumption_vars.size(),
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
                out.occupancy_vars = occupancy.u_var_by_node.size();
                out.occupancy_implication_clauses = occupancy.implication_clause_count;
                out.occupancy_soft_clauses = request.soft_negated_vars.size();
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
                if (!validation.pass) {
                    out.ok = false;
                    out.message = "PHYSICAL_VALIDATION_FAILED";
                    debug::error_fmt(
                        "unified Z3 Optimize physical validation failed: violations={} round={}",
                        validation.violations_count,
                        round);
                    stamp_timing(out);
                    log_feedback_round_end(round, FeedbackRoundStatus::SolverError);
                    return out;
                }
                if (options.enable_global_route_v17) {
                    log_final_sat_scopes(graph, global_channel_graph, state, nets, out);
                }
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
            const auto failed_units = collect_failed_unit_sources(
                model, result.failed_assumption_literals);
            debug::info_fmt(
                "unified Z3 Optimize hard-UNSAT: vars={} clauses={} core_size={} alpha_core_pairs={} unit_core_sources={} round_solve_ms={} total_solve_ms={} round={}",
                out.num_vars,
                out.num_clauses,
                result.failed_assumption_literals.size(),
                critical.size(),
                failed_units.size(),
                round_ms,
                out.solve_ms,
                round);
            if (critical.empty() && failed_units.empty()) {
                out.message = "BASE_HARD_UNSAT";
                debug::error(
                    "unified Z3 Optimize hard constraints are UNSAT without alpha/unit core");
                log_feedback_round_end(round, FeedbackRoundStatus::SolverError);
                stamp_timing(out);
                return out;
            }
            if (!critical.empty()) {
                log_failed_nets(nets, critical);
            }
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
            const auto released = release_failed_global_units(nets, failed_units);
            if (options.enable_global_route_v17) {
                const auto added_channels = apply_v17_pair_feedback(
                    state, nets, global_channel_graph, critical);
                apply_state_to_nets(state, nets);
                debug::info_fmt(
                    "V17 feedback applied: critical_pairs={} released_unit_sources={} guide_channels_added={} guides_full={}",
                    critical.size(),
                    released,
                    added_channels,
                    all_global_route_guides_full(state, global_channel_graph.channels.size()));
            }
            else if (apply_feedback_expansion(state, nets, critical)
                     == FeedbackExpansionStatus::Exhausted) {
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
    out.message = options.enable_global_route_v17 ? "SEARCH_LIMIT" : "UNSAT";
    out.feedback_rounds = options.max_feedback_rounds;
    log_feedback_round_end(options.max_feedback_rounds, FeedbackRoundStatus::MaxRoundsExceeded);
    stamp_timing(out);
    return out;
#endif
}

} // namespace PR_tool
