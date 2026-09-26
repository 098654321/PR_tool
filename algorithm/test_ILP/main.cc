#include "direct_ilp/direct_scope.hh"
#include "direct_ilp/direct_validate.hh"
#include "common/route_metrics.hh"
#include "route_ilp/route_master.hh"
#include "route_ilp/route_rrr.hh"
#include "sat/solve_unified_sat.hh"
#include "scope/build_routing_nets.hh"
#include "test_ilp_cli.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace PR_tool {

auto run_main(int argc, char** argv) -> int {
    auto args = std::Vector<std::string_view>{};
    for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);
    auto cli = TestIlpCliOptions{};
    try {
        cli = parse_test_ilp_cli(args);
    } catch (const std::exception& error) {
        debug::error(error.what());
        debug::info("Usage: test_ILP <config_path> [-v|-vv] [-o DIR] [--time-limit MIN] [--m-mode MODE] [--init-SAT]");
        debug::info("M modes: default, gap-1..gap-4");
        return 1;
    }
    const auto log_dir = std::filesystem::path{cli.output_dir};
    std::filesystem::create_directories(log_dir);
    debug::initial_log(log_dir / "debug.log");
    const auto begin = std::chrono::steady_clock::now();
    try {
        auto [interposer, basedie] = parse::read_config(cli.config_path, 0, false);
        algo::build_nets(basedie.get(), interposer.get());
        auto initial_sat = SatRoutingResult{};
        auto initial_route = RoutingResult{};
        if (cli.init_sat) {
            auto sat_options = UnifiedSatSolveOptions{};
            sat_options.verbose_level = cli.verbose_level;
            initial_sat = solve_unified_sat(interposer.get(), *basedie.get(), sat_options);
            if (!initial_sat.ok) {
                debug::error_fmt("initial SAT failed: {}", initial_sat.message);
                return 1;
            }
            initial_route.paths = std::move(initial_sat.paths);
            debug::info_fmt("initial SAT: paths={} wirelength={} vars={} clauses={} rounds={} total_ms={}",
                initial_route.paths.size(), initial_sat.total_wirelength,
                initial_sat.num_vars, initial_sat.num_clauses,
                initial_sat.feedback_rounds, initial_sat.sat_total_ms);
        }
        auto nets = build_routing_nets(basedie->nets_to_vector());
        auto graph = build_unified_graph(interposer.get(), nets);
        augment_graph_for_pnnet(graph, nets);
        if (cli.init_sat)
            for (auto& path : initial_route.paths) {
                const auto& net = *std::find_if(nets.begin(), nets.end(),
                    [&](const auto& candidate) { return candidate.net_id == path.net_id; });
                if (net.kind != RoutingNetKind::PNnet) continue;
                const auto source = std::find_if(net.sources.begin(), net.sources.end(),
                    [&](const auto& candidate) {
                        return resolve_graph_node(graph, candidate) == path.physical_source_node;
                    });
                if (source == net.sources.end())
                    throw std::logic_error("SAT PNnet source is not a candidate source");
                path.source_index = static_cast<std::size_t>(source - net.sources.begin());
            }
        debug::info_fmt("route ILP graph: nodes={} arcs={} nets={}",
                        graph.nodes.size(), graph.arcs.size(), nets.size());
        const auto scopes = build_direct_scopes(graph, nets, cli.verbose_level);
        if (cli.init_sat) {
            initial_route.ok = true;
            initial_route.total_wirelength = total_wirelength(graph, initial_route);
            initial_route.used_tob_switch_ids.clear();
            initial_route.vline_mode_straight_by_group.clear();
            auto switches = std::set<int>{};
            for (const auto& path : initial_route.paths)
                for (std::size_t j = 1; j < path.node_path.size(); ++j)
                    for (int aid : graph.out_arc_ids[static_cast<std::size_t>(path.node_path[j - 1])]) {
                        const auto& arc = graph.arcs[static_cast<std::size_t>(aid)];
                        if (arc.v != path.node_path[j]) continue;
                        if (arc.physical_switch_id >= 0) switches.insert(arc.physical_switch_id);
                        if (arc.mode_group_id >= 0)
                            initial_route.vline_mode_straight_by_group[
                                static_cast<std::size_t>(arc.mode_group_id)] =
                                arc.is_vline_track_straight;
                        break;
                    }
            initial_route.used_tob_switch_ids.assign(switches.begin(), switches.end());
        }
        if (cli.init_sat && !validate_direct_route(graph, nets, scopes, initial_route))
            throw std::logic_error("SAT initial route failed current physical validation");
        const auto route_begin = std::chrono::steady_clock::now();
        auto solved = solve_route_ilp(graph, nets, scopes,
            RouteIlpOptions{cli.verbose_level, cli.time_limit_minutes,
                             (log_dir / "highs.log").string(), cli.big_m_mode,
                             cli.init_sat ? &initial_route : nullptr});
        if (!solved.has_integer_solution || !validate_partial_route(
                graph, nets, scopes, solved.route, solved.bus_lengths)) {
            debug::error_fmt("route ILP did not produce a legal partial route: {}",
                             solved.route.message);
            return 1;
        }
        debug::info_fmt("route ILP partial validation: PASS paths={} missing={} wirelength={}",
                        solved.route.paths.size(), solved.missing.size(),
                        solved.route.total_wirelength);
        const auto deadline = cli.time_limit_minutes > 0 ?
            route_begin + std::chrono::minutes(cli.time_limit_minutes) :
            std::chrono::steady_clock::time_point::max();
        auto final = optimize_route_columns_rrr(graph, nets, scopes, solved,
                                                 cli.verbose_level, deadline);
        if (!validate_partial_route(graph, nets, scopes, final,
                                    solved.bus_lengths)) {
            debug::error("RRR produced an invalid partial route");
            return 1;
        }
        if (!final.ok || !validate_direct_route(graph, nets, scopes, final)) {
            debug::error_fmt("route ILP + RRR incomplete: status={}", final.rrr_status);
            return 1;
        }
        debug::info_fmt("ILP -> RRR: wirelength={}->{} status={} accepted={}",
            solved.route.total_wirelength, final.total_wirelength,
            final.rrr_status, final.rrr_accepted);
        for (const auto& path : final.paths) {
            auto description = std::String{};
            for (int node : path.node_path) {
                if (!description.empty()) description += " -> ";
                description += format_unified_node(graph, node);
            }
            debug::info_fmt("route net={} demand={} source={} path={}",
                path.net_id, path.demand_id, path.source_index, description);
        }
        const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - begin).count();
        debug::info_fmt("route ILP + RRR complete: wirelength={} total_ms={}",
                        final.total_wirelength, total_ms);
        return 0;
    } catch (const std::exception& error) {
        debug::error_fmt("route ILP + RRR failed: {}", error.what());
        return 1;
    }
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int { return PR_tool::run_main(argc, argv); }
