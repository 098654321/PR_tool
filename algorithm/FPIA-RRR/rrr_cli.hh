#pragma once

#include "rrr_types.hh"

#include <initializer_list>
#include <span>
#include <std/collection.hh>
#include <string_view>

namespace PR_tool {

auto parse_rrr_cli(std::span<std::string_view> args) -> RrrCliOptions;

inline auto parse_rrr_cli(std::initializer_list<std::string_view> args) -> RrrCliOptions {
    auto storage = std::Vector<std::string_view>(args);
    return parse_rrr_cli(std::span<std::string_view>{storage});
}

} // namespace PR_tool
