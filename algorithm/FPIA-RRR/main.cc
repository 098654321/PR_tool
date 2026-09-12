#include "rrr_cli.hh"
#include "hardware_graph.hh"
#include "net_adapter.hh"
#include "route_validate.hh"
#include "rrr_router.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>
#include <utility/elapsed.hh>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace PR_tool {

namespace {

constexpr auto kUsage =
    "Usage: ./output/FPIA_RRR <config_path> [-v|-vv] [-o DIR] [--max-iterations N] [--seed N]";

} // namespace

auto run_main(int argc, char** argv) -> int {
    Elapsed::start();
    auto cli_args = std::Vector<std::string_view> {};
    cli_args.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0);
    for (int argi = 1; argi < argc; ++argi) {
        cli_args.emplace_back(argv[argi]);
    }

    auto cli = RrrCliOptions {};
    try {
        cli = parse_rrr_cli(cli_args);
    }
    catch (const std::invalid_argument& error) {
        debug::error(error.what());
        debug::info(kUsage);
        return 1;
    }

    if (cli.verbose_level >= 1) {
        debug::set_debug_level(debug::DebugLevel::Debug);
    }

    const auto log_dir = std::filesystem::path {cli.output_dir};
    std::filesystem::create_directories(log_dir);
    debug::initial_log(log_dir / "debug.log");

    auto [interposer, basedie] = PR_tool::parse::read_config(cli.config_path, 0, false);
    algo::build_nets(basedie.get(), interposer.get());
    const auto nets = build_routing_nets(basedie->nets_to_vector());
    const auto graph = build_hardware_graph(interposer.get(), nets);

    auto params = RrrParams {};
    params.max_iterations = cli.max_iterations;
    params.seed = cli.seed;
    const auto result = run_rrr(graph, nets, params, interposer.get(), cli.verbose_level);
    if (result.status != "success" || result.best_overflow != 0) {
        return 1;
    }
    if (!validate_rrr_solution(graph, nets, result, interposer.get())) {
        return 1;
    }
    return 0;
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_main(argc, argv);
}
