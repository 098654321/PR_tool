#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string_view>

namespace order_experiment {
// Columns are the eight concrete Net classes; only top-level Nets are sorted.
inline constexpr std::array<std::string_view, 8> classes = {
    "SyncNet", "TrackToBumpsNet", "BumpToTracksNet", "BumpToBumpsNet",
    "BumpToTrackNet", "TrackToBumpNet", "BumpToBumpNet", "TracksToBumpsNet"};
inline constexpr std::array<std::array<int, 8>, 4> priorities = {{
    {0, 1, 1, 2, 3, 3, 4, 5}, // current
    {3, 0, 0, 1, 4, 4, 5, 2}, // new1: multi -> fixed -> sync -> two-pin
    {0, 2, 2, 1, 4, 4, 3, 5}, // new2: user-corrected bump-first ordering
    {2, 3, 3, 4, 0, 0, 1, 5}, // new3: two-pin first; others retain original order
}};
inline int policy = 0;
inline float base_priority(std::size_t kind) {
    return static_cast<float>(priorities.at(policy).at(kind));
}
inline std::mt19937_64 generator;
inline double random_real() {
    return static_cast<double>(generator() >> 11) * 0x1.0p-53;
}
inline std::int64_t random_integer(std::int64_t min, std::int64_t max) {
    if (max <= min) throw std::runtime_error("Empty random interval");
    const auto bound = static_cast<std::uint64_t>(max - min);
    const auto threshold = -bound % bound;
    std::uint64_t value;
    do { value = generator(); } while (value < threshold);
    return min + static_cast<std::int64_t>(value % bound);
}
} // namespace order_experiment
