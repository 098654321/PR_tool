#pragma once

#include <cstddef>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct Bump_coord {
    std::size_t TOB;
    std::size_t Bank;
    std::size_t Group;
    std::size_t Index;

    auto operator==(const Bump_coord& other) const -> bool {
        return TOB == other.TOB && Bank == other.Bank && Group == other.Group && Index == other.Index;
    }

    auto operator<(const Bump_coord& other) const -> bool {
        return std::tie(TOB, Bank, Group, Index) < std::tie(other.TOB, other.Bank, other.Group, other.Index);
    }
};

inline auto map_track(std::size_t track) -> std::size_t {
    return track < 64 ? track % 8 : track % 8 + 8;
}

} // namespace PR_tool
