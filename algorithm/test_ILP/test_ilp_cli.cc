#include "test_ilp_cli.hh"

#include <charconv>
#include <cmath>
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

auto parse_non_negative_double(std::string_view value, const char* option_name) -> double {
    double parsed = 0.0;
    const auto [end, error] =
        std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (error != std::errc {}
        || end != value.data() + value.size()
        || !std::isfinite(parsed)
        || parsed < 0.0) {
        throw std::invalid_argument(
            std::format("{} requires a non-negative finite number", option_name));
    }
    return parsed;
}

} // namespace

auto parse_test_ilp_cli(const std::span<const std::string_view> args) -> TestIlpCliOptions {
    if (args.empty()) {
        throw std::invalid_argument("No config path given");
    }

    auto options = TestIlpCliOptions {};
    options.config_path = args.front();
    for (std::size_t i = 1; i < args.size(); ++i) {
        const auto arg = args[i];
        if (arg == "--sat-log") {
            options.enable_sat_log = true;
            continue;
        }
        if (arg == "--max-rss-mb") {
            if (++i >= args.size()) {
                throw std::invalid_argument(
                    "--max-rss-mb requires a positive integer argument");
            }
            const auto value = args[i];
            std::size_t parsed = 0;
            const auto [end, error] =
                std::from_chars(value.data(), value.data() + value.size(), parsed);
            if (error != std::errc {} || end != value.data() + value.size() || parsed == 0) {
                throw std::invalid_argument(
                    "--max-rss-mb requires a positive integer argument");
            }
            options.max_rss_mb = parsed;
            continue;
        }
        if (arg == "-s") {
            if (++i >= args.size()) {
                throw std::invalid_argument("-s requires a non-negative integer argument");
            }
            options.initial_scope_pad = parse_non_negative_int(args[i], "-s");
            continue;
        }
        if (arg == "-d") {
            if (++i >= args.size()) {
                throw std::invalid_argument("-d requires a non-negative integer argument");
            }
            options.initial_delay_pad = parse_non_negative_int(args[i], "-d");
            continue;
        }
        if (arg == "--ilp-optimize") {
            options.enable_ilp_optimize = true;
            continue;
        }
        if (arg == "-L") {
            if (++i >= args.size()) {
                throw std::invalid_argument("-L requires a non-negative finite number");
            }
            options.ilp_stretch_threshold_percent =
                parse_non_negative_double(args[i], "-L");
            continue;
        }
        if (arg == "-R") {
            if (++i >= args.size()) {
                throw std::invalid_argument("-R requires a non-negative integer argument");
            }
            options.ilp_segment_bbox_pad = parse_non_negative_int(args[i], "-R");
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
    if (options.enable_ilp_optimize && !options.ilp_stretch_threshold_percent.has_value()) {
        throw std::invalid_argument("--ilp-optimize requires -L <percent>");
    }
    if (!options.enable_ilp_optimize && options.ilp_stretch_threshold_percent.has_value()) {
        throw std::invalid_argument("-L requires --ilp-optimize");
    }
    if (!options.enable_ilp_optimize && options.ilp_segment_bbox_pad.has_value()) {
        throw std::invalid_argument("-R requires --ilp-optimize");
    }
    return options;
}

} // namespace PR_tool
