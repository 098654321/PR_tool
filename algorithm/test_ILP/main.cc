#include "direct_ilp/direct_router.hh"
#include "direct_ilp/direct_scope.hh"
#include "direct_ilp/direct_validate.hh"
#include "rrr/rrr.hh"
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
        debug::info("Usage: test_ILP <config_path> [-v|-vv] [-o DIR] [--time-limit MIN]");
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
        debug::info_fmt("direct ILP graph: nodes={} arcs={} nets={}",
                        graph.nodes.size(), graph.arcs.size(), nets.size());
        const auto direct = build_direct_graph(graph);
        debug::info_fmt("direct ILP undirected graph: edges={}", direct.edges.size());
        const auto scopes = build_direct_scopes(graph, nets, cli.verbose_level);
        auto solved = solve_direct_ilp(graph, direct, nets, scopes,
            DirectIlpOptions{cli.verbose_level, cli.time_limit_minutes,
                             (log_dir / "highs.log").string()});
        if (!solved.route.ok || !validate_direct_route(graph, nets, scopes, solved.route)) {
            debug::error_fmt("direct ILP did not produce a legal route: {}",
                             solved.route.message);
            return 1;
        }
        auto final = optimize_routes_rrr(
            graph, nets, scopes, solved.route,
            RrrOptions{.verbose_level = cli.verbose_level});
        if (!validate_direct_route(graph, nets, scopes, final)) {
            debug::error("RRR produced an invalid route");
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
        debug::info_fmt("direct ILP + RRR complete: wirelength={} total_ms={}",
                        final.total_wirelength, total_ms);
        return 0;
    } catch (const std::exception& error) {
        debug::error_fmt("direct ILP + RRR failed: {}", error.what());
        return 1;
    }
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int { return PR_tool::run_main(argc, argv); }
