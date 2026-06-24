
#include "cli/cli.hh"
#include "gui/gui.hh"

#include "debug/console.hh"
#include "std/integer.hh"
#include "std/utility.hh"
#include <std/collection.hh>
#include <std/range.hh>
#include <std/string.hh>
#include <debug/debug.hh>
#include <std/algorithm.hh>

#ifdef _WIN32
#include "Windows.h"
#endif

namespace PR_tool {

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
        console::println("Indicate output directory for controlbit file");

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

        console::print_with_color("\t-i, --incremental           ", Color::Cyan);
        console::println("Work in incremental routing mode.");
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

        if (argument_index("-g", "--gui").has_value()) {
            // gui mode
            if (arguments[0] != "-g" && arguments[0] != "--gui") {
                debug::warning_fmt("Use gui model but indicate input config '{}', it will be ignored", arguments[0]);
            }
            return gui_main(argc, argv);
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

            // command for incremental mode
            auto incre_opt = argument_index("-i", "--incremental");
            auto comp_opt = argument_index("-c", "--compare");
            int incre_mode = 0;
            bool try_all_modes = false;
            std::optional<int> compare = std::nullopt;
            if (incre_opt.has_value()) {
                auto index = incre_opt.value();
                if (index >= (arguments.size() - 1) || arguments[index + 1].at(0) == '-') {
                    try_all_modes = true;
                } 
                else {
                    incre_mode = std::stoi(arguments[index + 1]);
                    if (incre_mode <= 0) {
                        debug::fatal("incremental mode should be a positive integer");
                    }
                }

                // check if comparation is needed
                if (comp_opt.has_value()) {
                    auto index = comp_opt.value();
                    if (index >= (arguments.size() - 1) || arguments[index + 1].at(0) == '-') {
                        debug::fatal("Use '-c/--compare' but not indicate the compare target!");
                    }
                    else {
                        compare = std::stoi(arguments[index + 1]);
                        if (compare.value() <= 0) {
                            debug::fatal("compare target should be a positive integer");
                        }
                    }
                }
            }

            return cli_main(arguments[0], std::move(output_path), incre_mode, compare, try_all_modes, placement);
        }

        return 0;
    }
}