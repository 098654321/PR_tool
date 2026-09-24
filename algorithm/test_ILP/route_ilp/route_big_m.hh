#pragma once

#include <array>
#include <optional>
#include <string_view>
#include <utility>

namespace PR_tool {

enum class RouteBigMMode {
    Fixed,
    MinLmin,
    MinLminPlusOne,
    MaxLmin,
    MaxLminPlusOne,
    GapOne,
    GapTwo,
    GapThree,
    GapFour,
};

inline constexpr auto kRouteBigMModeNames = std::array{
    std::pair{std::string_view{"fixed"}, RouteBigMMode::Fixed},
    std::pair{std::string_view{"min-lmin"}, RouteBigMMode::MinLmin},
    std::pair{std::string_view{"min-lmin-plus-1"}, RouteBigMMode::MinLminPlusOne},
    std::pair{std::string_view{"max-lmin"}, RouteBigMMode::MaxLmin},
    std::pair{std::string_view{"max-lmin-plus-1"}, RouteBigMMode::MaxLminPlusOne},
    std::pair{std::string_view{"gap-1"}, RouteBigMMode::GapOne},
    std::pair{std::string_view{"gap-2"}, RouteBigMMode::GapTwo},
    std::pair{std::string_view{"gap-3"}, RouteBigMMode::GapThree},
    std::pair{std::string_view{"gap-4"}, RouteBigMMode::GapFour},
};

inline auto parse_route_big_m_mode(std::string_view name)
    -> std::optional<RouteBigMMode> {
    for (const auto& [label, mode] : kRouteBigMModeNames)
        if (label == name) return mode;
    return std::nullopt;
}

inline auto route_big_m_mode_name(RouteBigMMode mode) -> std::string_view {
    for (const auto& [label, value] : kRouteBigMModeNames)
        if (value == mode) return label;
    return "unknown";
}

} // namespace PR_tool
