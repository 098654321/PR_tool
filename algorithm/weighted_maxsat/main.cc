#include "wmaxsat_cli.hh"
#include "wmaxsat_router.hh"

#include <debug/debug.hh>
#include <parse/reader/module.hh>

#include <chrono>
#include <filesystem>
#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

constexpr auto kUsage =
    "Usage: xmake run weighted_maxsat <config_path> [-v|-vv] [-o DIR] [--solver PATH] [--keep-wcnf]";

auto find_project_root() -> std::filesystem::path {
    auto root = std::filesystem::current_path();
    for (;;) {
        if (std::filesystem::is_regular_file(root / "xmake.lua")) {
            return root;
        }
        const auto parent = root.parent_path();
        if (parent == root) {
            return std::filesystem::current_path();
        }
        root = parent;
    }
}

auto resolve_config_directory(const std::String& requested) -> std::filesystem::path {
    const auto path = std::filesystem::path {requested};
    if (path.is_absolute() || std::filesystem::is_directory(path)) {
        return path;
    }
    auto root = find_project_root();
    for (;;) {
        const auto candidate = root / path;
        if (std::filesystem::is_directory(candidate)) {
            return candidate;
        }
        const auto parent = root.parent_path();
        if (parent == root) {
            return path;
        }
        root = parent;
    }
}

auto resolve_solver_path(const std::String& requested) -> std::filesystem::path {
    const auto path = std::filesystem::path {requested};
    if (path.is_absolute() || std::filesystem::exists(path)) {
        return path;
    }
    auto root = find_project_root();
    for (;;) {
        const auto candidate = root / path;
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
        const auto parent = root.parent_path();
        if (parent == root) {
            return path;
        }
        root = parent;
    }
}

} // namespace

auto run_main(int argc, char** argv) -> int {
    auto args = std::Vector<std::string_view> {};
    for (int i = 1; i < argc; ++i) {
        args.push_back(argv[i]);
    }
    auto cli = WmaxsatCliOptions {};
    try {
        cli = parse_wmaxsat_cli(args);
    }
    catch (const std::invalid_argument& error) {
        debug::error(error.what());
        debug::info(kUsage);
        return 1;
    }

    const auto started = std::chrono::steady_clock::now();
    const auto output_path = std::filesystem::path {cli.output_dir};
    const auto output_dir = output_path.is_absolute() ? output_path : find_project_root() / output_path;
    std::filesystem::create_directories(output_dir);
    debug::initial_log(output_dir / "debug.log");
    const auto solver_path = resolve_solver_path(cli.solver_path);
    debug::info_fmt(
        "weighted MAXSAT: config={} solver={} scope_pad=1 delay_pad=10 feedback_expansion=off",
        cli.config_path,
        solver_path.string());

    try {
        const auto config_path = resolve_config_directory(cli.config_path);
        auto [interposer, basedie] = parse::read_config(config_path, 0, false);
        auto encoding = build_wmaxsat_encoding(interposer.get(), *basedie.get());
        const auto wcnf_path = output_dir / "wmaxsat_instance.wcnf";
        write_wcnf(encoding, wcnf_path);
        auto result = WmaxsatSolverResult {};
        try {
            result = run_evalmaxsat(solver_path, wcnf_path, encoding.num_vars);
        }
        catch (...) {
            if (!cli.keep_wcnf) {
                std::filesystem::remove(wcnf_path);
            }
            throw;
        }
        if (!cli.keep_wcnf) {
            std::filesystem::remove(wcnf_path);
        }
        if (result.hard_unsat) {
            debug::error("weighted MAXSAT hard constraints are UNSAT");
            return 1;
        }
        if (!result.has_model) {
            debug::error_fmt("EvalMaxSAT returned no usable model: status={} output={}", result.status, result.output);
            return 1;
        }
        (void)log_wmaxsat_solution(encoding, result);
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started).count();
        debug::info_fmt("weighted MAXSAT total elapsed: {} ms", elapsed);
        return 0;
    }
    catch (const std::exception& error) {
        debug::error_fmt("weighted MAXSAT failed: {}", error.what());
        return 1;
    }
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_main(argc, argv);
}
