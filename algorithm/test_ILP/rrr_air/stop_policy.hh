#pragma once

#include <chrono>
#include <cstddef>
#include <limits>

namespace PR_tool::air316 {

// Only complete, independently validated solutions are observations.
struct StopPolicy {
    using Clock = std::chrono::steady_clock;
    Clock::time_point deadline{Clock::time_point::max()};
    std::size_t best_length{std::numeric_limits<std::size_t>::max()};
    std::size_t legal_count{};
    int stagnant{};
    bool has_legal{false};

    auto observe(bool legal, std::size_t length) -> bool {
        if (!legal) return false;
        ++legal_count;
        if (!has_legal || length < best_length) {
            has_legal = true;
            best_length = length;
            stagnant = 0;
            return true;
        }
        ++stagnant;
        return false;
    }
    auto expired(Clock::time_point now = Clock::now()) const -> bool {
        return now >= deadline;
    }
    auto reason(Clock::time_point now = Clock::now()) const -> const char* {
        if (stagnant >= 10) return "legal_stagnation";
        return expired(now) ? "time_budget" : nullptr;
    }
};

} // namespace PR_tool::air316
