#pragma once

#include "sat/sat_encoding_stats.hh"
#include "sat_allocation/cadical_solver.hh"

#include <initializer_list>
#include <span>

namespace PR_tool {

auto add_at_least_one(
    CadicalSession& session,
    std::span<const int> literals,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::Constant
) -> void;
auto add_at_least_one(
    CadicalSession& session,
    std::initializer_list<int> literals,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::Constant
) -> void;
auto add_pairwise_at_most_one(
    CadicalSession& session,
    std::span<const int> literals,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::Connectivity
) -> void;
auto add_pairwise_at_most_one(
    CadicalSession& session,
    std::initializer_list<int> literals,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::Connectivity
) -> void;
auto add_sequential_at_most_one(
    CadicalSession& session,
    std::span<const int> literals,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::Exclusivity
) -> void;
auto add_sequential_at_most_one(
    CadicalSession& session,
    std::initializer_list<int> literals,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::Exclusivity
) -> void;
auto add_exactly_one(
    CadicalSession& session,
    std::span<const int> literals,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::Constant
) -> void;
auto add_exactly_one(
    CadicalSession& session,
    std::initializer_list<int> literals,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::Constant
) -> void;
auto add_equiv(
    CadicalSession& session,
    int a,
    int b,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::VariableRelation
) -> void;
auto add_implies(
    CadicalSession& session,
    int antecedent,
    int consequent,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::VariableRelation
) -> void;
auto add_or_equiv(
    CadicalSession& session,
    int out,
    std::span<const int> inputs,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::VariableRelation
) -> void;
auto add_or_equiv(
    CadicalSession& session,
    int out,
    std::initializer_list<int> inputs,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::VariableRelation
) -> void;

struct BinarySuccessorVars {
    std::Vector<int> bits;
    int overflow{0};
};

// Legacy v13 SyncBus loop-elimination helper; retained for unit tests only.
auto add_binary_successor(
    CadicalSession& session,
    std::span<const int> input_bits,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::VariableRelation
) -> BinarySuccessorVars;
auto add_conditional_successor(
    CadicalSession& session,
    int condition,
    const BinarySuccessorVars& successor,
    std::span<const int> output_bits,
    SatEncodingStats* stats = nullptr,
    SatClauseCategory cat = SatClauseCategory::VariableRelation
) -> void;

} // namespace PR_tool
