#include "test_ilp_cli.hh"

#include <charconv>
#include <format>
#include <stdexcept>

namespace PR_tool {

auto parse_test_ilp_cli(std::span<const std::string_view> args)
    -> TestIlpCliOptions {
    if (args.empty() || args.front().empty() || args.front()[0] == '-')
        throw std::invalid_argument("config path is required");
    auto options = TestIlpCliOptions{};
    options.config_path = args.front();
    for (std::size_t i = 1; i < args.size(); ++i) {
        const auto arg = args[i];
        if (arg == "-o" || arg == "--output") {
            if (++i >= args.size() || args[i].empty() || args[i][0] == '-')
                throw std::invalid_argument("-o requires an output directory");
            options.output_dir = args[i];
        } else if (arg == "--time-limit") {
            if (++i >= args.size())
                throw std::invalid_argument("--time-limit requires minutes");
            const auto value = args[i];
            const auto [end, error] = std::from_chars(
                value.data(), value.data() + value.size(), options.time_limit_minutes);
            if (error != std::errc{} || end != value.data() + value.size()
                || options.time_limit_minutes <= 0)
                throw std::invalid_argument("--time-limit requires positive minutes");
        } else if (arg == "--m-mode") {
            if (++i >= args.size())
                throw std::invalid_argument("--m-mode requires a mode");
            const auto mode = parse_route_big_m_mode(args[i]);
            if (!mode)
                throw std::invalid_argument(std::format("unknown --m-mode: {}", args[i]));
            options.big_m_mode = *mode;
        } else if (arg.size() > 1 && arg[0] == '-') {
            bool verbose = true;
            for (std::size_t j = 1; j < arg.size(); ++j) verbose &= arg[j] == 'v';
            if (!verbose) throw std::invalid_argument(std::format("unknown option: {}", arg));
            options.verbose_level += static_cast<int>(arg.size() - 1);
        } else {
            throw std::invalid_argument(std::format("unexpected argument: {}", arg));
        }
    }
    return options;
}

} // namespace PR_tool
