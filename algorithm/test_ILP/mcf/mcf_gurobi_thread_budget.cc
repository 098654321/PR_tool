#include "mcf/mcf_gurobi_thread_budget.hh"

#include "debug/debug.hh"

#include <algorithm>
#include <future>
#include <thread>

namespace PR_tool {

McfGurobiThreadBudget::McfGurobiThreadBudget(const int cap) : cap_ {std::max(1, cap)} {}

void McfGurobiThreadBudget::acquire(const int threads) {
    const auto need = std::max(1, threads);
    std::unique_lock lock {mutex_};
    cv_.wait(lock, [&] { return active_.load() + need <= cap_; });
    active_ += need;
}

void McfGurobiThreadBudget::release(const int threads) {
    const auto give = std::max(1, threads);
    {
        const std::lock_guard lock {mutex_};
        active_ = std::max(0, active_.load() - give);
    }
    cv_.notify_all();
}

auto McfGurobiThreadBudget::cap() const -> int {
    return cap_;
}

auto McfGurobiThreadBudget::active() const -> int {
    return active_.load();
}

auto run_mcf_parallel_waves(
    McfGurobiThreadBudget& budget,
    const int threads_per_task,
    const std::Vector<std::function<void()>>& tasks,
    const std::atomic<bool>* cancel_flag
) -> void {
    if (tasks.empty()) {
        return;
    }
    const auto per = std::max(1, threads_per_task);
    const auto max_parallel = std::max(1, budget.cap() / per);
    std::size_t cursor = 0;
    int wave = 0;
    while (cursor < tasks.size()) {
        if (cancel_flag != nullptr && cancel_flag->load()) {
            break;
        }
        const auto batch_end = std::min(tasks.size(), cursor + static_cast<std::size_t>(max_parallel));
        debug::info_fmt(
            "MCF thread budget: wave={} tasks=[{},{}] threads_per_task={} budget_active={}/{}",
            wave,
            cursor,
            batch_end,
            per,
            budget.active(),
            budget.cap());
        auto futures = std::Vector<std::future<void>> {};
        futures.reserve(batch_end - cursor);
        for (std::size_t i = cursor; i < batch_end; ++i) {
            if (cancel_flag != nullptr && cancel_flag->load()) {
                break;
            }
            futures.push_back(std::async(std::launch::async, [&, i] {
            tasks[i]();
        }));
        }
        for (auto& fut : futures) {
            fut.get();
        }
        if (cancel_flag != nullptr && cancel_flag->load()) {
            break;
        }
        cursor = batch_end;
        ++wave;
    }
}

auto mcf_gurobi_thread_budget_instance() -> McfGurobiThreadBudget& {
    static McfGurobiThreadBudget budget {kMcfGurobiThreadCap};
    return budget;
}

} // namespace PR_tool
