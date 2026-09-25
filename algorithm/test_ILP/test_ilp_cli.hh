#pragma once

#include "route_ilp/route_big_m.hh"

#include <initializer_list>
#include <span>
#include <std/string.hh>
#include <string_view>

namespace PR_tool {

struct TestIlpCliOptions {
    std::String config_path;
    std::String output_dir{"."};
    int verbose_level{0};
    int time_limit_minutes{0};
    RouteBigMMode big_m_mode{RouteBigMMode::Default};
    bool init_sat{false};
};

auto parse_test_ilp_cli(std::span<const std::string_view> args)
    -> TestIlpCliOptions;

inline auto parse_test_ilp_cli(std::initializer_list<std::string_view> args)
    -> TestIlpCliOptions {
    return parse_test_ilp_cli({args.begin(), args.size()});
}

} // namespace PR_tool
