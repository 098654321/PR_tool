#include "wmaxsat_cli.hh"

#include <format>
#include <stdexcept>

namespace PR_tool {

auto parse_wmaxsat_cli(const std::span<const std::string_view> args) -> WmaxsatCliOptions {
    if (args.empty()) {
        throw std::invalid_argument("No config path given");
    }
    auto options = WmaxsatCliOptions {};
    options.config_path = args.front();
    for (std::size_t i = 1; i < args.size(); ++i) {
        const auto arg = args[i];
        if (arg == "-o" || arg == "--output") {
            if (++i >= args.size() || args[i].empty()) {
                throw std::invalid_argument("-o/--output requires an output directory");
            }
            options.output_dir = args[i];
            continue;
        }
        if (arg == "--solver") {
            if (++i >= args.size() || args[i].empty()) {
                throw std::invalid_argument("--solver requires an executable path");
            }
            options.solver_path = args[i];
            continue;
        }
        if (arg == "--keep-wcnf") {
            options.keep_wcnf = true;
            continue;
        }
        if (arg.size() >= 2 && arg[0] == '-' && arg[1] == 'v') {
            for (std::size_t j = 1; j < arg.size(); ++j) {
                if (arg[j] != 'v') {
                    throw std::invalid_argument(std::format("Unknown argument: {}", arg));
                }
            }
            options.verbose_level += static_cast<int>(arg.size() - 1);
            continue;
        }
        throw std::invalid_argument(std::format("Unknown argument: {}", arg));
    }
    return options;
}

} // namespace PR_tool
