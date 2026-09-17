// V13 unified SAT routing entry.

#include "sat/solve_unified_sat.hh"
#include "test_ilp_cli.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <stdexcept>
#include <sys/resource.h>

namespace PR_tool {

namespace {

constexpr auto kUsage =
    "Usage: xmake run test_ILP <config_path> [-v|-vv] [-o DIR] [--sat-log] [--max-rss-mb N] [--time-limit MIN] [-s S] [-d D] [--z3-optimize | --global-route-v17 | --global-route-v18 [--ilp-optimize | --maze-optimize]]";

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

    const auto log_dir = std::filesystem::path {cli.output_dir};
    std::filesystem::create_directories(log_dir);
    debug::initial_log(log_dir / "debug.log");

    auto [interposer, basedie] = PR_tool::parse::read_config(cli.config_path, 0, false);
    algo::build_nets(basedie.get(), interposer.get());

    UnifiedSatSolveOptions options {};
    options.verbose_level = cli.verbose_level;
    options.cadical.enable_sat_log = cli.enable_sat_log;
    options.cadical.verbose_level = cli.verbose_level;
    options.cadical.max_rss_mb = cli.max_rss_mb;
    options.initial_scope_pad = cli.initial_scope_pad;
    options.initial_delay_pad = cli.initial_delay_pad;
    options.enable_z3_optimize = cli.enable_z3_optimize;
    options.enable_global_route_v17 = cli.enable_global_route_v17;
    options.enable_global_route_v18 = cli.enable_global_route_v18;
    options.enable_post_sat_ilp = cli.enable_post_sat_ilp;
    options.enable_post_sat_maze = cli.enable_post_sat_maze;
    options.highs_log_path = (log_dir / "highs.log").string();
    options.highs_time_limit_minutes = cli.highs_time_limit_minutes;
    if (cli.highs_time_limit_minutes != 0) {
        debug::info_fmt(
            "HiGHS per-stage time limit: {} min",
            cli.highs_time_limit_minutes);
    }
    if (cli.enable_sat_log) {
        debug::info_fmt("CaDiCal solver logs enabled: directory={}", options.cadical.log_dir);
    }
    if (cli.max_rss_mb != 0) {
        if (cli.enable_z3_optimize) {
            debug::info_fmt(
                "RSS limit enabled for Z3 CNF encoding only: {} MB",
                cli.max_rss_mb);
        }
        else {
            debug::info_fmt("Process peak RSS limit enabled: {} MB", cli.max_rss_mb);
        }
    }
    if (cli.verbose_level >= 1
        && (cli.initial_scope_pad != 0 || cli.initial_delay_pad != 0)) {
        debug::info_fmt(
            "initial search padding: scope_pad={} delay_pad={}",
            cli.initial_scope_pad,
            cli.initial_delay_pad);
    }
    if (cli.enable_global_route_v18) {
        debug::info(
            "v18 flow enabled: HiGHS Channel/COBUnit capacity cuts (no W) -> guided CaDiCaL pure SAT");
    }
    if (cli.enable_post_sat_ilp) {
        debug::info("V20 post-SAT ILP refinement enabled by --ilp-optimize");
    }
    else if (cli.enable_post_sat_maze) {
        debug::info("V20 post-SAT local maze/RRR refinement enabled by --maze-optimize");
    }
    else if (cli.enable_global_route_v17) {
        debug::info(
            "v17 flow enabled: HiGHS Channel/COBUnit global routing -> guided Z3 weighted partial MaxSAT");
    }
    else if (cli.enable_z3_optimize) {
        debug::info("v16 Z3 Optimize enabled: hard constraints + alpha assumptions, node-occupancy soft objective");
    }

    const auto result = solve_unified_sat(interposer.get(), *basedie.get(), options);
    const auto peak_rss_mb = get_peak_rss_mb();
    debug::info_fmt("Process peak RSS: {:.2f} MB", peak_rss_mb);

    const auto run_end = std::chrono::steady_clock::now();
    const auto raw_run_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(run_end - run_begin).count();
    const auto run_ms = std::max(0LL, raw_run_ms - result.excluded_diagnostic_ms);
    debug::info_fmt(
        "run_main total elapsed: {} ms (raw={} excluded_diagnostic_ms={})",
        run_ms,
        raw_run_ms,
        result.excluded_diagnostic_ms);

    if (!result.ok) {
        debug::error_fmt("unified SAT routing failed: {}", result.message);
        return 1;
    }
    debug::info_fmt(
        "unified SAT: paths={} vars={} clauses={} total_ms={} pre_ms={} solve_ms={}",
        result.paths.size(),
        result.num_vars,
        result.num_clauses,
        result.sat_total_ms,
        result.sat_pre_ms,
        result.solve_ms);
    debug::info_fmt(
        "global route: status={} vars={} constraints={} objective={} estimated_wirelength={} total_ms={} build_ms={} solve_ms={}",
        result.global_route_requested ? result.global_route_status : "n/a",
        result.global_route_vars,
        result.global_route_constraints,
        result.global_route_objective,
        result.global_route_estimated_wirelength,
        result.global_route_total_ms,
        result.global_route_build_ms,
        result.global_route_solve_ms);
    debug::info_fmt(
        "post-SAT ILP: attempted={} accepted={} status={} parents={} segments={} vars={} constraints={} wirelength={}->{} objective={:.0f} bound={:.3f} gap={:.6f} total_ms={} build_ms={} solve_ms={}",
        result.post_sat_ilp_attempted,
        result.post_sat_ilp_accepted,
        result.post_sat_ilp_attempted ? result.post_sat_ilp_status : "n/a",
        result.post_sat_ilp_parents,
        result.post_sat_ilp_segments,
        result.post_sat_ilp_variables,
        result.post_sat_ilp_constraints,
        result.post_sat_ilp_baseline_wirelength,
        result.post_sat_ilp_wirelength,
        result.post_sat_ilp_objective,
        result.post_sat_ilp_bound,
        result.post_sat_ilp_gap,
        result.post_sat_ilp_total_ms,
        result.post_sat_ilp_build_ms,
        result.post_sat_ilp_solve_ms);
    debug::info_fmt(
        "post-SAT maze: attempted={} accepted={} status={} triggers={} accepted_triggers={} rrr_iterations={} rerouted_owners={} wirelength={}->{} total_ms={}",
        result.post_sat_maze_attempted,
        result.post_sat_maze_accepted,
        result.post_sat_maze_attempted ? result.post_sat_maze_status : "n/a",
        result.post_sat_maze_triggers,
        result.post_sat_maze_accepted_triggers,
        result.post_sat_maze_rrr_iterations,
        result.post_sat_maze_rerouted_owners,
        result.post_sat_maze_baseline_wirelength,
        result.post_sat_maze_wirelength,
        result.post_sat_maze_total_ms);
    debug::info_fmt(
        "routing result: total_wirelength={}",
        result.total_wirelength);
    return 0;
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_main(argc, argv);
}
