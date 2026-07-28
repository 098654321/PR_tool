
#include "cli/cli.hh"
#ifndef PR_TOOL_CLI_ONLY
#include "gui/gui.hh"
#endif

#include "algo/router/backend/router_backend.hh"
#include "debug/console.hh"
#include "std/integer.hh"
#include "std/utility.hh"
#include <charconv>
#include <cmath>
#include <std/collection.hh>
#include <std/range.hh>
#include <std/string.hh>
#include <debug/debug.hh>
#include <std/algorithm.hh>

#ifdef _WIN32
#include "Windows.h"
#endif

namespace PR_tool {

namespace {

auto parse_non_negative_int(std::StringView value, const char* option_name) -> int {
    int parsed = 0;
    const auto [end, error] =
        std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (error != std::errc {} || end != value.data() + value.size() || parsed < 0) {
        debug::fatal_fmt("{} requires a non-negative integer argument", option_name);
    }
    return parsed;
}

auto parse_non_negative_double(std::StringView value, const char* option_name) -> double {
    double parsed = 0.0;
    const auto [end, error] =
        std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (error != std::errc {}
        || end != value.data() + value.size()
        || !std::isfinite(parsed)
        || parsed < 0.0) {
        debug::fatal_fmt("{} requires a non-negative finite number", option_name);
    }
    return parsed;
}

auto parse_positive_double(std::StringView value, const char* option_name) -> double {
    const auto parsed = parse_non_negative_double(value, option_name);
    if (parsed <= 0.0) {
        debug::fatal_fmt("{} requires a positive finite number", option_name);
    }
    return parsed;
}

auto require_next_arg(
    const std::Vector<std::String>& arguments, std::usize& index, const char* option_name
) -> std::StringView {
    if (++index >= arguments.size()) {
        debug::fatal_fmt("{} requires an argument", option_name);
    }
    const auto value = std::StringView {arguments[index]};
    if (value.empty() || value[0] == '-') {
        debug::fatal_fmt("{} requires an argument", option_name);
    }
    return value;
}

auto parse_router_and_sat_options(
    const std::Vector<std::String>& arguments,
    algo::RouterKind& router_kind,
    algo::SatRouterCliOptions& sat_opts,
    bool& any_sat_only_flag
) -> void {
    router_kind = algo::RouterKind::Maze;
    sat_opts = {};
    any_sat_only_flag = false;

    for (std::usize i = 0; i < arguments.size(); ++i) {
        const auto arg = std::StringView {arguments[i]};
        if (arg == "--router") {
            const auto value = require_next_arg(arguments, i, "--router");
            if (value == "maze") {
                router_kind = algo::RouterKind::Maze;
            } else if (value == "sat") {
#if !PR_TOOL_HAS_SAT_ROUTER
                debug::fatal("This build was configured without sat_router");
#else
                router_kind = algo::RouterKind::Sat;
#endif
            } else {
                debug::fatal("--router requires maze or sat");
            }
            continue;
        }
        if (arg == "--scope-pad") {
            any_sat_only_flag = true;
            sat_opts.initial_scope_pad =
                parse_non_negative_int(require_next_arg(arguments, i, "--scope-pad"), "--scope-pad");
            continue;
        }
        if (arg == "--delay-pad") {
            any_sat_only_flag = true;
            sat_opts.initial_delay_pad =
                parse_non_negative_int(require_next_arg(arguments, i, "--delay-pad"), "--delay-pad");
            continue;
        }
        if (arg == "-d") {
            any_sat_only_flag = true;
            sat_opts.initial_delay_pad =
                parse_non_negative_int(require_next_arg(arguments, i, "-d"), "-d");
            continue;
        }
        if (arg == "--sat-log") {
            any_sat_only_flag = true;
            sat_opts.enable_sat_log = true;
            continue;
        }
        if (arg == "--max-rss-mb") {
            any_sat_only_flag = true;
            const auto value = require_next_arg(arguments, i, "--max-rss-mb");
            std::size_t parsed = 0;
            const auto [end, error] =
                std::from_chars(value.data(), value.data() + value.size(), parsed);
            if (error != std::errc {} || end != value.data() + value.size() || parsed == 0) {
                debug::fatal("--max-rss-mb requires a positive integer argument");
            }
            sat_opts.max_rss_mb = parsed;
            continue;
        }
        if (arg == "--ilp-optimize") {
            any_sat_only_flag = true;
            sat_opts.enable_ilp_optimize = true;
            continue;
        }
        if (arg == "-L") {
            any_sat_only_flag = true;
            sat_opts.ilp_stretch_threshold_percent =
                parse_non_negative_double(require_next_arg(arguments, i, "-L"), "-L");
            continue;
        }
        if (arg == "-R") {
            any_sat_only_flag = true;
            sat_opts.ilp_segment_bbox_pad =
                parse_non_negative_int(require_next_arg(arguments, i, "-R"), "-R");
            continue;
        }
        if (arg == "--time-limit") {
            any_sat_only_flag = true;
            sat_opts.ilp_time_limit_hours =
                parse_positive_double(require_next_arg(arguments, i, "--time-limit"), "--time-limit");
            continue;
        }
        if (arg == "-v" || arg == "--verbose") {
            sat_opts.verbose_level += 1;
            continue;
        }
        if (arg.size() >= 2 && arg[0] == '-' && arg[1] == 'v') {
            bool all_v = true;
            for (std::size_t char_index = 1; char_index < arg.size(); ++char_index) {
                if (arg[char_index] != 'v') {
                    all_v = false;
                    break;
                }
            }
            if (all_v) {
                sat_opts.verbose_level += static_cast<int>(arg.size() - 1);
                continue;
            }
        }
    }

    if (sat_opts.enable_ilp_optimize && !sat_opts.ilp_stretch_threshold_percent.has_value()) {
        debug::fatal("--ilp-optimize requires -L <percent>");
    }
    if (!sat_opts.enable_ilp_optimize && sat_opts.ilp_stretch_threshold_percent.has_value()) {
        debug::fatal("-L requires --ilp-optimize");
    }
    if (!sat_opts.enable_ilp_optimize && sat_opts.ilp_segment_bbox_pad.has_value()) {
        debug::fatal("-R requires --ilp-optimize");
    }
    if (!sat_opts.enable_ilp_optimize && sat_opts.ilp_time_limit_hours.has_value()) {
        debug::fatal("--time-limit requires --ilp-optimize");
    }
}

} // namespace

    auto PR_toollogo = "\
    \t██████╗ ██████╗         ████████╗ ██████╗  ██████╗ ██╗\n\
    \t██╔══██╗██╔══██╗        ╚══██╔══╝██╔═══██╗██╔═══██╗██║\n\
    \t██████╔╝██████╔╝           ██║   ██║   ██║██║   ██║██║\n\
    \t██╔═══╝ ██╔══██╗           ██║   ██║   ██║██║   ██║██║\n\
    \t██║     ██║  ██║           ██║   ╚██████╔╝╚██████╔╝███████╗\n\
    \t╚═╝     ╚═╝  ╚═╝           ╚═╝    ╚═════╝  ╚═════╝ ╚══════╝\n";

    auto print_help() -> void {
        using console::Color;

        console::println("Place & Route tool\n");

        console::println_with_color("Usage: ", Color::Green);
        console::println_with_color("\tPR_tool <input folder path> [OPTIONS]\n", Color::Cyan);
        console::println_with_color("Options: ", Color::Green);

        console::print_with_color("\t-o, --output <OUTPUT_PATH>  ", Color::Cyan);
        console::println("Output root; writes regnamecontrolbit_4part/ under it");

        console::print_with_color("\t-g, --gui                   ", Color::Cyan);
        console::println("Work in gui");

        console::print_with_color("\t-h, --help                  ", Color::Cyan);
        console::println("Print help");

        console::print_with_color("\t-V, --version               ", Color::Cyan);
        console::println("Print version info and exit");

        console::print_with_color("\t-v, --verbose               ", Color::Cyan);
        console::println("Print lots of verbose information for users.");

        console::print_with_color("\t-p, --placement             ", Color::Cyan);
        console::println("Work in placement mode.");

        console::print_with_color("\t-s, --simplify-controlbits-file ", Color::Cyan);
        console::println("Omit default-valued registers when writing the four REG files.");

        console::print_with_color("\t--router maze|sat           ", Color::Cyan);
        console::println("Router backend (default: maze).");

        console::println_with_color("SAT router flags (--router sat):", Color::Green);
        console::print_with_color("\t--scope-pad N               ", Color::Cyan);
        console::println("Initial pair bbox padding (default 0).");
        console::print_with_color("\t--delay-pad N, -d N         ", Color::Cyan);
        console::println("Initial delay padding (default 0).");
        console::print_with_color("\t--sat-log                   ", Color::Cyan);
        console::println("Enable CaDiCal solver logs (./cadical-log).");
        console::print_with_color("\t--max-rss-mb N              ", Color::Cyan);
        console::println("Peak RSS limit in MB for CaDiCal.");
        console::print_with_color("\t--ilp-optimize -L percent     ", Color::Cyan);
        console::println("Optional v15 Gurobi wirelength optimization.");
        console::print_with_color("\t-R pad, --time-limit hours  ", Color::Cyan);
        console::println("ILP segment bbox pad / Gurobi time limit.");
    }

    auto print_verion() -> void {
        using console::Color;

        console::println_fmt("PR_tool v1.0.0 ({} {})\n", __DATE__, __TIME__);
        console::println_with_color(PR_toollogo, Color::Blue);
    }

    int main(int argc, char** argv) {
    #ifdef _WIN32
        SetConsoleOutputCP(CP_UTF8);
        if (AttachConsole(ATTACH_PARENT_PROCESS)) {
            freopen("CONOUT$", "w", stdout);
            freopen("CONOUT$", "w", stderr);
        }
    #endif

        if (argc == 1) {
            print_help();
            return 0;
        }

        auto arguments = std::Vector<std::String>{};
        for (int i = 1; i < argc; ++i) {
            arguments.emplace_back(argv[i]);
        }

        // return the position of the first argument that is arg1 or arg2. If not found, return std::nullopt
        auto argument_index = [&arguments](std::StringView arg1, std::StringView arg2) -> std::Option<std::usize> {
            for (std::usize i = 0; i < arguments.size(); ++i) {
                if (arguments[i] == arg1 || arguments[i] == arg2) {
                    return i;
                }
            }
            return std::nullopt;
        };

        if (argument_index("-v", "--verbose").has_value()) {
            debug::set_debug_level(debug::DebugLevel::Debug);
        }

        constexpr auto kIncrementalUnsupported =
            "Incremental routing and the relative functions is not supported in version 1.0.0";

        if (argument_index("-i", "--incremental").has_value()
            || argument_index("-c", "--compare").has_value()) {
            debug::fatal(kIncrementalUnsupported);
        }

        if (argument_index("-g", "--gui").has_value()) {
#ifdef PR_TOOL_CLI_ONLY
            debug::fatal("GUI is not available in PR_tool_cli; build/run PR_tool for GUI");
#else
            // gui mode
            if (arguments[0] != "-g" && arguments[0] != "--gui") {
                debug::warning_fmt("Use gui model but indicate input config '{}', it will be ignored", arguments[0]);
            }
            return gui_main(argc, argv);
#endif
        } 
        else if (arguments[0] == "-h" || arguments[0] == "--help") {
            print_help();
        }
        else if (arguments[0] == "-V" || arguments[0] == "--version") {
            print_verion();
        }
        else {
            // cli mode output path
            auto output_opt = argument_index("-o", "--output");
            auto output_path = std::Option<std::StringView>{std::nullopt};
            if (output_opt.has_value()) {
                auto index = *output_opt;
                if (index >= (arguments.size() - 1) || arguments[index + 1].at(0) == '-') {
                    debug::warning("Use '-o/--output' but not indicate the output path! Use default instead");
                } else {
                    output_path.emplace(arguments[index + 1]);
                }
            }

            // placement
            bool placement = false;
            if (argument_index("-p", "--placement").has_value()) {
                placement = true;
            }

            bool simplify_controlbits = false;
            if (argument_index("-s", "--simplify-controlbits-file").has_value()) {
                simplify_controlbits = true;
            }

            algo::RouterKind router_kind = algo::RouterKind::Maze;
            algo::SatRouterCliOptions sat_opts {};
            bool any_sat_only_flag = false;
            parse_router_and_sat_options(arguments, router_kind, sat_opts, any_sat_only_flag);
            if (router_kind == algo::RouterKind::Maze && any_sat_only_flag) {
                debug::fatal("SAT-only flags require --router sat");
            }

            return cli_main(arguments[0], std::move(output_path), /*mode=*/0, /*compare=*/std::nullopt,
                            /*try_all_modes=*/false, placement, simplify_controlbits,
                            router_kind, sat_opts);
        }

        return 0;
    }
}