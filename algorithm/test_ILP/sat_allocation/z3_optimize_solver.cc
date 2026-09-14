#include "sat_allocation/z3_optimize_solver.hh"

#include <z3++.h>

#include <cstdlib>
#include <format>
#include <limits>
#include <stdexcept>
#include <string>

namespace PR_tool {

namespace {

auto validate_literal(int literal, std::size_t num_vars, const char* context) -> void {
    if (literal == 0 || literal == std::numeric_limits<int>::min()
        || static_cast<std::size_t>(std::abs(literal)) > num_vars) {
        throw std::invalid_argument(std::format("{} contains invalid literal {}", context, literal));
    }
}

auto literal_expr(const std::Vector<z3::expr>& variables, int literal) -> z3::expr {
    const auto& variable = variables[static_cast<std::size_t>(std::abs(literal))];
    return literal > 0 ? variable : !variable;
}

} // namespace

auto Z3OptimizeResult::value(const int variable) const -> bool {
    if (variable <= 0 || static_cast<std::size_t>(variable) >= assignment.size()) {
        throw std::out_of_range("Z3 model variable is out of range");
    }
    return assignment[static_cast<std::size_t>(variable)];
}

auto solve_z3_optimize(const Z3OptimizeRequest& request) -> Z3OptimizeResult {
    auto out = Z3OptimizeResult {};
    try {
        if (request.num_vars == 0) {
            throw std::invalid_argument("Z3 Optimize requires at least one variable");
        }

        auto context = z3::context {};
        auto optimizer = z3::optimize {context};
        auto variables = std::Vector<z3::expr> {};
        variables.reserve(request.num_vars + 1);
        variables.push_back(context.bool_val(false));
        for (std::size_t index = 1; index <= request.num_vars; ++index) {
            variables.push_back(context.bool_const(std::format("x{}", index).c_str()));
        }

        for (const auto& clause : request.hard_clauses) {
            auto expression = context.bool_val(false);
            for (const int literal : clause) {
                validate_literal(literal, request.num_vars, "Z3 hard clause");
                expression = expression || literal_expr(variables, literal);
            }
            optimizer.add(expression);
        }
        for (const int variable : request.soft_negated_vars) {
            if (variable <= 0 || static_cast<std::size_t>(variable) > request.num_vars) {
                throw std::invalid_argument(std::format("Z3 soft variable contains invalid variable {}", variable));
            }
            optimizer.add_soft(!variables[static_cast<std::size_t>(variable)], 1);
        }

        auto assumptions = z3::expr_vector {context};
        for (const int literal : request.external_assumptions) {
            validate_literal(literal, request.num_vars, "Z3 external assumption");
            assumptions.push_back(literal_expr(variables, literal));
        }

        const auto status = optimizer.check(assumptions);
        if (status == z3::unsat) {
            out.status = Z3OptimizeStatus::HardUnsat;
            out.message = "UNSAT";
            const auto core = optimizer.unsat_core();
            for (const int assumption : request.external_assumptions) {
                const auto assumption_expr = literal_expr(variables, assumption);
                for (unsigned index = 0; index < core.size(); ++index) {
                    if (z3::eq(assumption_expr, core[index])) {
                        out.failed_assumption_literals.push_back(assumption);
                        break;
                    }
                }
            }
            return out;
        }
        if (status == z3::unknown) {
            const auto reason = Z3_optimize_get_reason_unknown(context, optimizer);
            out.message = reason == nullptr ? "UNKNOWN" : std::format("UNKNOWN: {}", reason);
            return out;
        }

        out.status = Z3OptimizeStatus::Optimal;
        out.message = "OPTIMUM";
        out.assignment.assign(request.num_vars + 1, false);
        const auto model = optimizer.get_model();
        for (std::size_t index = 1; index <= request.num_vars; ++index) {
            out.assignment[index] = model.eval(variables[index], true).is_true();
        }
        for (const int variable : request.soft_negated_vars) {
            if (out.value(variable)) {
                ++out.objective_cost;
            }
        }
        return out;
    }
    catch (const z3::exception& error) {
        out.message = std::format("Z3 exception: {}", error.what());
    }
    catch (const std::exception& error) {
        out.message = std::format("Z3 wrapper error: {}", error.what());
    }
    return out;
}

} // namespace PR_tool
