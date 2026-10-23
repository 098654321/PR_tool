#pragma once

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>

namespace sa_experiment {
inline std::mt19937_64 generator;

// Explicit mappings keep random draws independent of the standard library's
// uniform_distribution implementation. Integer intervals are [min, max).
inline double random_real() {
    return static_cast<double>(generator() >> 11) * 0x1.0p-53;
}
inline std::int64_t random_integer(std::int64_t min, std::int64_t max) {
    if (max <= min) throw std::runtime_error("Empty random integer interval");
    const auto bound = static_cast<std::uint64_t>(max - min);
    const auto threshold = -bound % bound;
    std::uint64_t value;
    do { value = generator(); } while (value < threshold);
    return min + static_cast<std::int64_t>(value % bound);
}

struct Statistics {
    std::uint64_t attempts{}, swap_attempts{}, move_attempts{};
    std::uint64_t accepted_nonworse{}, accepted_uphill{}, rejected{};
    std::size_t layers{}, stalled_layers{};
    double final_temperature{};
    std::int64_t best_cost{};
};
inline Statistics stats;
inline std::ofstream iterations;

inline void temperature_end(std::size_t layer, double temperature,
                            std::int64_t current, std::int64_t best,
                            std::size_t stalled) {
    stats.best_cost = best;
    iterations << layer << '\t' << std::setprecision(17) << temperature << '\t'
               << current << '\t' << best << '\t' << stalled << '\t'
               << stats.attempts << '\t' << stats.swap_attempts << '\t'
               << stats.move_attempts << '\t' << stats.accepted_nonworse << '\t'
               << stats.accepted_uphill << '\t' << stats.rejected << '\n';
    iterations.flush();
}
inline void finish(double temperature, std::size_t stalled, std::size_t layers) {
    stats.final_temperature = temperature;
    stats.stalled_layers = stalled;
    stats.layers = layers;
}
} // namespace sa_experiment
