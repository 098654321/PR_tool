#include "sat_allocation/cadical_solver.hh"

#ifndef USE_CADICAL
#error "USE_CADICAL is required; build with xmake f --cadical=y"
#endif

#include <cadical.hpp>
#include <debug/debug.hh>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <format>
#include <limits>
#include <memory>
#include <stdexcept>
#include <sys/resource.h>

namespace PR_tool {

namespace {

constexpr auto kBytesPerMb = 1024.0 * 1024.0;
constexpr std::size_t kMemoryCheckInterval = 4096;

auto process_peak_rss_bytes() -> std::size_t {
    auto usage = rusage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0 || usage.ru_maxrss <= 0) {
        return 0;
    }
#if defined(__APPLE__)
    return static_cast<std::size_t>(usage.ru_maxrss);
#elif defined(__linux__)
    return static_cast<std::size_t>(usage.ru_maxrss) * 1024;
#else
    return static_cast<std::size_t>(usage.ru_maxrss) * 1024;
#endif
}

} // namespace

MemoryLimitExceeded::MemoryLimitExceeded()
    : std::runtime_error("MEMORY_LIMIT") {
}

class CadicalSession::Impl final : public CaDiCaL::Terminator {
public:
    explicit Impl(CadicalDiagnosticsOptions diagnostics)
        : options(std::move(diagnostics)) {
        solver.set("quiet", 1);
        if (options.enable_sat_log) {
            std::filesystem::create_directories(options.log_dir);
            const auto trace_path = std::format("{}/unified_sat.trace", options.log_dir);
            trace_file.reset(std::fopen(trace_path.c_str(), "w"));
            if (trace_file) {
                solver.trace_api_calls(trace_file.get());
                debug::info_fmt("CaDiCal API trace enabled: {}", trace_path);
            }
        }
    }

    auto terminate() -> bool override {
        return observe_memory_limit();
    }

    [[nodiscard]] auto memory_limit_enabled() const -> bool {
        return options.max_rss_mb != 0;
    }

    auto observe_memory_limit() -> bool {
        peak_rss_bytes = std::max(peak_rss_bytes, process_peak_rss_bytes());
        if (!memory_limit_enabled()) {
            return false;
        }
        const auto limit_bytes =
            static_cast<double>(options.max_rss_mb) * kBytesPerMb;
        if (static_cast<double>(peak_rss_bytes) > limit_bytes) {
            memory_exceeded = true;
        }
        return memory_exceeded;
    }

    auto check_encoding_memory() -> void {
        if (!memory_limit_enabled()) {
            return;
        }
        if (encoding_operation_count % kMemoryCheckInterval == 0) {
            sample_encoding_memory();
        }
        ++encoding_operation_count;
    }

    auto sample_encoding_memory() -> void {
        ++encoding_memory_samples;
        check_memory_now();
    }

    auto check_memory_now() -> void {
        if (observe_memory_limit()) {
            throw MemoryLimitExceeded {};
        }
    }

    auto ensure_encoding_open() const -> void {
        if (solve_called) {
            throw std::logic_error("CaDiCaL session has already been solved");
        }
    }

    using FilePtr = std::unique_ptr<FILE, decltype(&std::fclose)>;

    CadicalDiagnosticsOptions options;
    // Keep the trace open until after the solver emits its destructor trace.
    FilePtr trace_file {nullptr, &std::fclose};
    CaDiCaL::Solver solver;
    std::size_t variable_count{0};
    std::size_t clause_count{0};
    std::size_t encoding_operation_count{0};
    std::size_t encoding_memory_samples{0};
    std::size_t peak_rss_bytes{0};
    int solve_status{0};
    bool solve_called{false};
    bool memory_exceeded{false};
    std::Vector<int> assumed_literals;
};

CadicalSession::CadicalSession(CadicalDiagnosticsOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {
}

CadicalSession::~CadicalSession() = default;

auto CadicalSession::new_var() -> int {
    impl_->ensure_encoding_open();
    impl_->check_encoding_memory();
    if (impl_->variable_count >= static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::overflow_error("CaDiCaL variable limit exceeded");
    }
    const int var = impl_->solver.declare_one_more_variable();
    ++impl_->variable_count;
    return var;
}

auto CadicalSession::add_clause(std::initializer_list<int> clause) -> void {
    add_clause(std::span<const int> {clause.begin(), clause.size()});
}

auto CadicalSession::add_clause(const std::Vector<int>& clause) -> void {
    add_clause(std::span<const int> {clause});
}

auto CadicalSession::add_clause(std::span<const int> clause) -> void {
    impl_->ensure_encoding_open();
    impl_->check_encoding_memory();
    for (const int literal : clause) {
        if (literal == 0 || literal == std::numeric_limits<int>::min()) {
            throw std::invalid_argument("CaDiCaL clause contains an invalid literal");
        }
        const auto variable = static_cast<std::size_t>(std::abs(literal));
        if (variable > impl_->variable_count) {
            throw std::out_of_range("CaDiCaL clause references an undeclared variable");
        }
    }
    std::size_t inserted_literals = 0;
    for (const int literal : clause) {
        impl_->solver.add(literal);
        ++inserted_literals;
        if (impl_->memory_limit_enabled()
            && inserted_literals % kMemoryCheckInterval == 0) {
            impl_->sample_encoding_memory();
        }
    }
    impl_->solver.add(0);
    ++impl_->clause_count;
}

auto CadicalSession::num_vars() const -> std::size_t {
    return impl_->variable_count;
}

auto CadicalSession::num_clauses() const -> std::size_t {
    return impl_->clause_count;
}

auto CadicalSession::assume(int lit) -> void {
    impl_->ensure_encoding_open();
    if (lit == 0) {
        throw std::invalid_argument("CaDiCaL assume() requires a non-zero literal");
    }
    impl_->solver.assume(lit);
    impl_->assumed_literals.push_back(lit);
}

auto CadicalSession::solve() -> CadicalSolveResult {
    if (impl_->solve_called) {
        throw std::logic_error("CaDiCaL solve() may only be called once per session");
    }
    impl_->solve_called = true;

    auto out = CadicalSolveResult {};
    if (impl_->memory_limit_enabled() && impl_->observe_memory_limit()) {
        out.memory_limit_exceeded = true;
        out.message = "MEMORY_LIMIT";
        return out;
    }

    if (impl_->memory_limit_enabled()) {
        impl_->solver.connect_terminator(impl_.get());
        try {
            impl_->solve_status = impl_->solver.solve();
            impl_->solver.disconnect_terminator();
        }
        catch (...) {
            impl_->solver.disconnect_terminator();
            throw;
        }
    }
    else {
        impl_->solve_status = impl_->solver.solve();
    }

    out.status = impl_->solve_status;
    if (impl_->solve_status == CaDiCaL::SATISFIABLE) {
        out.ok = true;
        out.message = "SAT";
    }
    else if (impl_->solve_status == CaDiCaL::UNSATISFIABLE) {
        out.message = "UNSAT";
        for (const int lit : impl_->assumed_literals) {
            if (impl_->solver.failed(lit)) {
                out.failed_assumption_literals.push_back(lit);
            }
        }
    }
    else if (impl_->memory_exceeded) {
        out.memory_limit_exceeded = true;
        out.message = "MEMORY_LIMIT";
    }
    else {
        out.message = std::format("CaDiCal returned status={}", impl_->solve_status);
    }
    impl_->assumed_literals.clear();
    return out;
}

auto CadicalSession::solve_once() -> CadicalSolveResult {
    return solve();
}

auto CadicalSession::failed(int lit) const -> bool {
    if (!impl_->solve_called || impl_->solve_status != CaDiCaL::UNSATISFIABLE) {
        throw std::logic_error("CaDiCaL failed() requires an UNSAT result");
    }
    if (lit == 0) {
        throw std::invalid_argument("CaDiCaL failed() requires a non-zero literal");
    }
    return impl_->solver.failed(lit);
}

auto CadicalSession::value(int var) const -> bool {
    if (!impl_->solve_called || impl_->solve_status != CaDiCaL::SATISFIABLE) {
        throw std::logic_error("CaDiCaL value() requires a SAT result");
    }
    if (var <= 0 || static_cast<std::size_t>(var) > impl_->variable_count) {
        throw std::out_of_range("CaDiCaL value() variable is out of range");
    }
    return impl_->solver.val(var) > 0;
}

auto CadicalSession::memory_limit_exceeded() const -> bool {
    (void)impl_->observe_memory_limit();
    return impl_->memory_exceeded;
}

auto CadicalSession::peak_rss_mb() const -> double {
    (void)impl_->observe_memory_limit();
    return static_cast<double>(impl_->peak_rss_bytes) / kBytesPerMb;
}

auto CadicalSession::encoding_memory_sample_count() const -> std::size_t {
    return impl_->encoding_memory_samples;
}

} // namespace PR_tool
