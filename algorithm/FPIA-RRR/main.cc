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
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <stdexcept>

namespace PR_tool {

namespace {

constexpr auto kUsage =
    "Usage: ./output/FPIA_RRR <config_path> [-v|-vv] [-o DIR] [--max-iterations N] [--seed N] [--time-budget-seconds S]";

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
    const auto begin = std::chrono::steady_clock::now();

    auto [interposer, basedie] = PR_tool::parse::read_config(cli.config_path, 0, false);
    algo::build_nets(basedie.get(), interposer.get());
    const auto nets = build_routing_nets(basedie->nets_to_vector());
    const auto graph = build_hardware_graph(interposer.get(), nets);

    auto params = RrrParams {};
    params.max_iterations = cli.max_iterations;
    params.seed = cli.seed;
    params.time_budget_seconds = cli.time_budget_seconds;
    params.budget_start = begin;
    const auto result = run_rrr(graph, nets, params, interposer.get(), cli.verbose_level);
    const bool success = result.status == "success" && result.best_overflow == 0
        && validate_rrr_solution(graph, nets, result, interposer.get());
    const auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - begin).count();
    debug::info_fmt(
        "FPIA RRR complete: status={} stop_reason={} wirelength={} total_ms={} time_budget_seconds={} over_budget_ms={}",
        success ? "success" : "failed", result.stop_reason, result.total_wirelength, total_ms,
        cli.time_budget_seconds,
        cli.time_budget_seconds > 0 ? std::max(0.0, total_ms - cli.time_budget_seconds * 1000) : 0.0);
    return success ? 0 : 1;
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_main(argc, argv);
}
