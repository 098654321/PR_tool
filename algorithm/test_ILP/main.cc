// V13 unified SAT routing entry.

#include "sat/solve_unified_sat.hh"
#include "test_ilp_cli.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <stdexcept>
#include <sys/resource.h>

namespace PR_tool {

namespace {

constexpr auto kUsage =
    "Usage: xmake run test_ILP <config_path> [-v|-vv] [-o DIR] [--sat-log] [--max-rss-mb N] [-s S] [-d D] [--ilp-optimize -L percent [-R pad] [--time-limit hours]]";

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

    auto [interposer, basedie, register_map] = PR_tool::parse::read_config(cli.config_path, 0, false);
    algo::build_nets(basedie.get(), interposer.get());

    UnifiedSatSolveOptions options {};
    options.verbose_level = cli.verbose_level;
    options.cadical.enable_sat_log = cli.enable_sat_log;
    options.cadical.verbose_level = cli.verbose_level;
    options.cadical.max_rss_mb = cli.max_rss_mb;
    options.initial_scope_pad = cli.initial_scope_pad;
    options.initial_delay_pad = cli.initial_delay_pad;
    options.ilp_optimize.enabled = cli.enable_ilp_optimize;
    options.ilp_optimize.stretch_threshold_percent =
        cli.ilp_stretch_threshold_percent.value_or(0.0);
    options.ilp_optimize.segment_bbox_pad =
        cli.ilp_segment_bbox_pad.value_or(0);
    options.ilp_optimize.time_limit_hours = cli.ilp_time_limit_hours;
    options.ilp_optimize.verbose_level = cli.verbose_level;
    options.ilp_optimize.gurobi_log_dir = (log_dir / "gurobi").string();
    if (cli.enable_sat_log) {
        debug::info_fmt("CaDiCal solver logs enabled: directory={}", options.cadical.log_dir);
    }
    if (cli.max_rss_mb != 0) {
        debug::info_fmt("Process peak RSS limit enabled: {} MB", cli.max_rss_mb);
    }
    if (cli.verbose_level >= 1
        && (cli.initial_scope_pad != 0 || cli.initial_delay_pad != 0)) {
        debug::info_fmt(
            "initial search padding: scope_pad={} delay_pad={}",
            cli.initial_scope_pad,
            cli.initial_delay_pad);
    }
    if (cli.enable_ilp_optimize) {
        if (options.ilp_optimize.time_limit_hours.has_value()) {
            debug::info_fmt(
                "v15 ILP optimization enabled: threshold={:.2f}% segment_bbox_pad={} time_limit_hours={} gurobi_log_dir={}",
                options.ilp_optimize.stretch_threshold_percent,
                options.ilp_optimize.segment_bbox_pad,
                options.ilp_optimize.time_limit_hours.value(),
                options.ilp_optimize.gurobi_log_dir);
        } else {
            debug::info_fmt(
                "v15 ILP optimization enabled: threshold={:.2f}% segment_bbox_pad={} time_limit=unlimited gurobi_log_dir={}",
                options.ilp_optimize.stretch_threshold_percent,
                options.ilp_optimize.segment_bbox_pad,
                options.ilp_optimize.gurobi_log_dir);
        }
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
        "unified SAT: paths={} vars={} clauses={} total_ms={} pre_ms={} solve_ms={}",
        result.paths.size(),
        result.num_vars,
        result.num_clauses,
        result.sat_total_ms,
        result.sat_pre_ms,
        result.solve_ms);
    debug::info_fmt(
        "unified ILP: requested={} status={} fallback_to_SAT={} vars={} constraints={} total_ms={} pre_ms={} solve_ms={}",
        result.ilp_optimization_requested,
        result.ilp_optimization_requested ? result.ilp_status : "n/a",
        result.ilp_fallback_to_sat,
        result.ilp_model_vars,
        result.ilp_model_constraints,
        result.ilp_total_ms,
        result.ilp_pre_ms,
        result.ilp_solve_ms);
    debug::info_fmt(
        "routing result: total_wirelength={}",
        result.total_wirelength);
    return 0;
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_main(argc, argv);
}
