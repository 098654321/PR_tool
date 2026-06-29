// V13 unified SAT routing entry.

#include "sat/solve_unified_sat.hh"
#include "test_ilp_cli.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>

#include <chrono>
#include <cstdlib>
#include <format>
#include <stdexcept>
#include <sys/resource.h>

namespace PR_tool {

namespace {

constexpr auto kUsage =
    "Usage: xmake run test_ILP <config_path> [-v|-vv] [--sat-log] [--max-rss-mb N]";

auto get_peak_rss_mb() -> double {
    rusage usage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return 0.0;
    }
#if defined(__APPLE__)
    return static_cast<double>(usage.ru_maxrss) / (1024.0 * 1024.0);
#else
    return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
}

} // namespace

auto run_main(int argc, char** argv) -> int {
    const auto run_begin = std::chrono::steady_clock::now();
    auto cli_args = std::Vector<std::string_view> {};
    cli_args.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0);
    for (int argi = 1; argi < argc; ++argi) {
        cli_args.emplace_back(argv[argi]);
    }

    auto cli = TestIlpCliOptions {};
    try {
        cli = parse_test_ilp_cli(cli_args);
    }
    catch (const std::invalid_argument& error) {
        debug::error(error.what());
        debug::info(kUsage);
        return 1;
    }

    auto [interposer, basedie] = PR_tool::parse::read_config(cli.config_path, 0, false);
    algo::build_nets(basedie.get(), interposer.get());

    UnifiedSatSolveOptions options {};
    options.verbose_level = cli.verbose_level;
    options.cadical.enable_sat_log = cli.enable_sat_log;
    options.cadical.verbose_level = cli.verbose_level;
    options.cadical.max_rss_mb = cli.max_rss_mb;
    if (cli.enable_sat_log) {
        debug::info_fmt("CaDiCal solver logs enabled: directory={}", options.cadical.log_dir);
    }
    if (cli.max_rss_mb != 0) {
        debug::info_fmt("Process peak RSS limit enabled: {} MB", cli.max_rss_mb);
    }

    const auto result = solve_unified_sat(interposer.get(), *basedie.get(), options);
    const auto peak_rss_mb = get_peak_rss_mb();
    debug::info_fmt("Process peak RSS: {:.2f} MB", peak_rss_mb);

    const auto run_end = std::chrono::steady_clock::now();
    const auto run_ms = std::chrono::duration_cast<std::chrono::milliseconds>(run_end - run_begin).count();
    debug::info_fmt("run_main total elapsed: {} ms", run_ms);

    if (!result.ok) {
        debug::error_fmt("unified SAT routing failed: {}", result.message);
        return 1;
    }
    debug::info_fmt(
        "unified SAT routing succeeded: paths={} vars={} clauses={} solve_ms={}",
        result.paths.size(),
        result.num_vars,
        result.num_clauses,
        result.solve_ms);
    return 0;
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_main(argc, argv);
}
