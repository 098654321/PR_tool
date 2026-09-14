#pragma once

#include <cstddef>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

enum class Z3OptimizeStatus {
    Optimal,
    HardUnsat,
    Unknown,
};

struct Z3OptimizeRequest {
    std::size_t num_vars{0};
    std::Vector<std::Vector<int>> hard_clauses;
    std::Vector<int> soft_negated_vars;
    std::Vector<int> external_assumptions;
};

struct Z3OptimizeResult {
    Z3OptimizeStatus status{Z3OptimizeStatus::Unknown};
    std::String message;
    std::size_t objective_cost{0};
    std::Vector<int> failed_assumption_literals;
    std::Vector<bool> assignment;

    [[nodiscard]] auto value(int variable) const -> bool;
};

auto solve_z3_optimize(const Z3OptimizeRequest& request) -> Z3OptimizeResult;

} // namespace PR_tool
