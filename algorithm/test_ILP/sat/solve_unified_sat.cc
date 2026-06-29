#include "sat/solve_unified_sat.hh"

#include "sat/routing_path_log.hh"
#include "sat/sat_solution_extract.hh"
#include "sat/unified_sat_encoder.hh"
#include "scope/build_routing_nets.hh"
#include "scope/scope_bbox.hh"
#include "graph/unified_routing_graph.hh"

#include <chrono>
#include <debug/debug.hh>
#include <format>

namespace PR_tool {

auto solve_unified_sat(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const UnifiedSatSolveOptions& options
) -> SatRoutingResult {
    auto out = SatRoutingResult {};
    long long solve_ms = 0;

    auto nets = build_routing_nets(basedie.nets_to_vector());
    assign_scope_bboxes(nets);
    log_scope_bboxes(nets, options.verbose_level);

    const auto graph = build_unified_graph(interposer, nets);
    debug::info_fmt(
        "unified graph: nodes={} arcs={} track_nodes={} tob_nodes={}",
        graph.nodes.size(),
        graph.arcs.size(),
        graph.track_node_count,
        graph.tob_node_count);

    auto session = CadicalSession {options.cadical};
    try {
        debug::info("streaming unified numeric SAT model into CaDiCal...");
        const auto model = build_unified_sat_model(session, graph, nets);
        debug::info_fmt(
            "unified SAT model built: vars={} clauses={}",
            session.num_vars(),
            session.num_clauses());
        debug::info("solving unified SAT with CaDiCal...");
        const auto solve_begin = std::chrono::steady_clock::now();
        const auto solve_result = session.solve_once();
        const auto solve_end = std::chrono::steady_clock::now();
        solve_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_begin)
                .count();
        out.num_vars = session.num_vars();
        out.num_clauses = session.num_clauses();
        if (solve_result.ok) {
            out = extract_sat_solution(graph, nets, model, session, solve_result);
        }
        else {
            out.message = solve_result.message;
        }
    }
    catch (const MemoryLimitExceeded&) {
        out.message = "MEMORY_LIMIT";
        out.num_vars = session.num_vars();
        out.num_clauses = session.num_clauses();
    }

    out.solve_ms = solve_ms;
    if (out.ok) {
        log_routing_paths(graph, nets, out);
        debug::info_fmt(
            "unified SAT ok: paths={} vars={} clauses={} ms={}",
            out.paths.size(),
            out.num_vars,
            out.num_clauses,
            out.solve_ms);
    }
    else {
        debug::error_fmt(
            "unified SAT failed: {} (vars={} clauses={} ms={})",
            out.message,
            out.num_vars,
            out.num_clauses,
            out.solve_ms);
    }
    return out;
}

} // namespace PR_tool
