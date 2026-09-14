#pragma once

#include <std/string.hh>

#include <span>
#include <initializer_list>
#include <string_view>

namespace PR_tool {

struct WmaxsatCliOptions {
    std::String config_path;
    std::String output_dir{"."};
    std::String solver_path{"third_party/EvalMaxSAT/build/EvalMaxSAT_bin"};
    int verbose_level{0};
    bool keep_wcnf{false};
};

auto parse_wmaxsat_cli(std::span<const std::string_view> args) -> WmaxsatCliOptions;

inline auto parse_wmaxsat_cli(
    const std::initializer_list<std::string_view> args
) -> WmaxsatCliOptions {
    return parse_wmaxsat_cli(std::span<const std::string_view> {args.begin(), args.size()});
}

} // namespace PR_tool
