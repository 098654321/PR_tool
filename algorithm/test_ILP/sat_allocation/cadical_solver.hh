#pragma once

#include <std/collection.hh>
#include <std/string.hh>

#include <cstddef>
#include <initializer_list>
#include <memory>
#include <span>
#include <stdexcept>

namespace PR_tool {

struct CadicalDiagnosticsOptions {
    bool enable_sat_log{false};
    int verbose_level{0};
    std::String log_dir {"./cadical-log"};
    std::size_t max_rss_mb{0};
};

struct CadicalSolveResult {
    bool ok{false};
    int status{0};
    bool memory_limit_exceeded{false};
    std::String message;
};

class MemoryLimitExceeded final : public std::runtime_error {
public:
    MemoryLimitExceeded();
};

class CadicalSession {
public:
    explicit CadicalSession(CadicalDiagnosticsOptions options = {});
    ~CadicalSession();

    CadicalSession(const CadicalSession&) = delete;
    auto operator=(const CadicalSession&) -> CadicalSession& = delete;

    auto new_var() -> int;
    auto add_clause(std::initializer_list<int> clause) -> void;
    auto add_clause(const std::Vector<int>& clause) -> void;
    auto add_clause(std::span<const int> clause) -> void;

    [[nodiscard]] auto num_vars() const -> std::size_t;
    [[nodiscard]] auto num_clauses() const -> std::size_t;
    auto solve_once() -> CadicalSolveResult;
    [[nodiscard]] auto value(int var) const -> bool;

    [[nodiscard]] auto memory_limit_exceeded() const -> bool;
    [[nodiscard]] auto peak_rss_mb() const -> double;
    [[nodiscard]] auto encoding_memory_sample_count() const -> std::size_t;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace PR_tool
