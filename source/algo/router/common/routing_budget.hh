#pragma once

#include <chrono>
#include <optional>

namespace PR_tool::algo {

// Deliberately separate from std::exception: legacy maze wrappers must not
// turn cancellation into a routing failure or retry it inside a sync group.
struct RoutingTimeout {};

class RoutingBudget {
public:
    using Clock = std::chrono::steady_clock;
    explicit RoutingBudget(std::optional<double> seconds = {},
                           Clock::time_point start = Clock::now())
        : _seconds(seconds), _start(start), _previous(_active) { _active = this; }
    ~RoutingBudget() { _active = _previous; }
    RoutingBudget(const RoutingBudget&) = delete;
    auto operator=(const RoutingBudget&) -> RoutingBudget& = delete;

    auto expired(Clock::time_point now = Clock::now()) const -> bool {
        return _seconds && std::chrono::duration<double>(now - _start).count() >= *_seconds;
    }
    static auto check() -> void {
        if (_active && _active->expired()) throw RoutingTimeout{};
    }
private:
    std::optional<double> _seconds;
    Clock::time_point _start;
    RoutingBudget* _previous;
    inline static thread_local RoutingBudget* _active{nullptr};
};

}
