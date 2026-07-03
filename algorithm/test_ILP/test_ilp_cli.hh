#pragma once

#include <cstddef>
#include <initializer_list>
#include <span>
#include <std/string.hh>
#include <string_view>

namespace PR_tool {

struct TestIlpCliOptions {
    std::String config_path;
    int verbose_level{0};
    bool enable_sat_log{false};
    std::size_t max_rss_mb{0};
    int initial_scope_pad{0};
    int initial_delay_pad{0};
};

auto parse_test_ilp_cli(std::span<const std::string_view> args) -> TestIlpCliOptions;

inline auto parse_test_ilp_cli(
    const std::initializer_list<std::string_view> args
) -> TestIlpCliOptions {
    return parse_test_ilp_cli(std::span<const std::string_view> {args.begin(), args.size()});
}

} // namespace PR_tool
