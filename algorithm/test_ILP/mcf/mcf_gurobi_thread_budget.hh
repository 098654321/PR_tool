#pragma once

#include <std/collection.hh>
#include <std/string.hh>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <vector>

namespace PR_tool {

inline constexpr int kMcfGurobiThreadCap = 64;

/// Tracks active Gurobi thread slots across concurrent MCF solves (cap 64).
class McfGurobiThreadBudget {
public:
    explicit McfGurobiThreadBudget(int cap = kMcfGurobiThreadCap);

    void acquire(int threads);
    void release(int threads);

    [[nodiscard]] auto cap() const -> int;
    [[nodiscard]] auto active() const -> int;

private:
    int cap_;
    std::atomic<int> active_{0};
    std::mutex mutex_;
    std::condition_variable cv_;
};

/// Run tasks in waves so each wave's thread sum does not exceed \p budget.cap().
/// Stops launching new waves after \p cancel_flag becomes true.
auto run_mcf_parallel_waves(
    McfGurobiThreadBudget& budget,
    int threads_per_task,
    const std::Vector<std::function<void()>>& tasks,
    const std::atomic<bool>* cancel_flag
) -> void;

auto mcf_gurobi_thread_budget_instance() -> McfGurobiThreadBudget&;

} // namespace PR_tool
