#include "direct_ilp/direct_scope.hh"
#include "direct_ilp/direct_validate.hh"
#include "route_ilp/route_master.hh"
#include "route_ilp/route_rrr.hh"
#include "scope/build_routing_nets.hh"
#include "test_ilp_cli.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>

#include <chrono>
#include <filesystem>
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
        debug::info("Usage: test_ILP <config_path> [-v|-vv] [-o DIR] [--time-limit MIN] [--m-mode MODE]");
        debug::info("M modes: fixed, min-lmin, min-lmin-plus-1, max-lmin, max-lmin-plus-1, gap-1..gap-4");
        return 1;
    }
    const auto log_dir = std::filesystem::path{cli.output_dir};
    std::filesystem::create_directories(log_dir);
    debug::initial_log(log_dir / "debug.log");
    const auto begin = std::chrono::steady_clock::now();
    try {
        auto [interposer, basedie] = parse::read_config(cli.config_path, 0, false);
        algo::build_nets(basedie.get(), interposer.get());
        auto nets = build_routing_nets(basedie->nets_to_vector());
        auto graph = build_unified_graph(interposer.get(), nets);
        augment_graph_for_pnnet(graph, nets);
        debug::info_fmt("route ILP graph: nodes={} arcs={} nets={}",
                        graph.nodes.size(), graph.arcs.size(), nets.size());
        const auto scopes = build_direct_scopes(graph, nets, cli.verbose_level);
        auto solved = solve_route_ilp(graph, nets, scopes,
            RouteIlpOptions{cli.verbose_level, cli.time_limit_minutes,
                             (log_dir / "highs.log").string(), cli.big_m_mode});
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
            begin + std::chrono::minutes(cli.time_limit_minutes) :
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
