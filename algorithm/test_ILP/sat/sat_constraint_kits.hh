#pragma once

#include "sat_allocation/cadical_solver.hh"

#include <initializer_list>
#include <span>

namespace PR_tool {

auto add_at_least_one(CadicalSession& session, std::span<const int> literals) -> void;
auto add_at_least_one(CadicalSession& session, std::initializer_list<int> literals) -> void;
auto add_pairwise_at_most_one(CadicalSession& session, std::span<const int> literals) -> void;
auto add_pairwise_at_most_one(CadicalSession& session, std::initializer_list<int> literals) -> void;
auto add_sequential_at_most_one(CadicalSession& session, std::span<const int> literals) -> void;
auto add_sequential_at_most_one(CadicalSession& session, std::initializer_list<int> literals) -> void;
auto add_exactly_one(CadicalSession& session, std::span<const int> literals) -> void;
auto add_exactly_one(CadicalSession& session, std::initializer_list<int> literals) -> void;
auto add_equiv(CadicalSession& session, int a, int b) -> void;
auto add_implies(CadicalSession& session, int antecedent, int consequent) -> void;
auto add_or_equiv(CadicalSession& session, int out, std::span<const int> inputs) -> void;
auto add_or_equiv(CadicalSession& session, int out, std::initializer_list<int> inputs) -> void;

struct BinarySuccessorVars {
    std::Vector<int> bits;
    int overflow{0};
};

auto add_binary_successor(
    CadicalSession& session,
    std::span<const int> input_bits
) -> BinarySuccessorVars;
auto add_conditional_successor(
    CadicalSession& session,
    int condition,
    const BinarySuccessorVars& successor,
    std::span<const int> output_bits
) -> void;

} // namespace PR_tool
