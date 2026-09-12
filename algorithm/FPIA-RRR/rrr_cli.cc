#include "rrr_cli.hh"

#include <charconv>
#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

auto parse_non_negative_int(std::string_view value, const char* option_name) -> int {
    int parsed = 0;
    const auto [end, error] =
        std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (error != std::errc {} || end != value.data() + value.size() || parsed < 0) {
        throw std::invalid_argument(
            std::format("{} requires a non-negative integer argument", option_name));
    }
    return parsed;
}

} // namespace

auto parse_rrr_cli(std::span<std::string_view> args) -> RrrCliOptions {
    if (args.empty()) {
        throw std::invalid_argument("No config path given");
    }

    auto options = RrrCliOptions {};
    options.config_path = args.front();
    for (std::size_t i = 1; i < args.size(); ++i) {
        const auto arg = args[i];
        if (arg == "-o") {
            if (++i >= args.size() || args[i].empty() || args[i][0] == '-') {
                throw std::invalid_argument("-o requires an output directory");
            }
            options.output_dir = args[i];
            continue;
        }
        if (arg == "--max-iterations") {
            if (++i >= args.size()) {
                throw std::invalid_argument(
                    "--max-iterations requires a non-negative integer argument");
            }
            options.max_iterations = parse_non_negative_int(args[i], "--max-iterations");
            continue;
        }
        if (arg == "--seed") {
            if (++i >= args.size()) {
                throw std::invalid_argument("--seed requires a non-negative integer argument");
            }
            options.seed = parse_non_negative_int(args[i], "--seed");
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
                options.verbose_level += static_cast<int>(arg.size() - 1);
                continue;
            }
        }
        throw std::invalid_argument(std::format("Unknown argument: {}", arg));
    }
    return options;
}

} // namespace PR_tool
