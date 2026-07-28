#include "sat/sat_constraint_kits.hh"

namespace PR_tool {

namespace {

auto record_clause(
    SatEncodingStats* stats,
    const SatClauseCategory cat,
    const std::size_t count = 1
) -> void {
    if (stats != nullptr) {
        stats->add_clauses(cat, count);
    }
}

auto add_xor_equiv(
    CadicalSession& session,
    const int out,
    const int a,
    const int b,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    session.add_clause({-a, -b, -out});
    session.add_clause({a, b, -out});
    session.add_clause({a, -b, out});
    session.add_clause({-a, b, out});
    record_clause(stats, cat, 4);
}

auto add_and_equiv(
    CadicalSession& session,
    const int out,
    const int a,
    const int b,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    session.add_clause({-out, a});
    session.add_clause({-out, b});
    session.add_clause({out, -a, -b});
    record_clause(stats, cat, 3);
}

} // namespace

auto add_at_least_one(
    CadicalSession& session,
    const std::span<const int> literals,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    session.add_clause(literals);
    record_clause(stats, cat);
}

auto add_at_least_one(
    CadicalSession& session,
    const std::initializer_list<int> literals,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    add_at_least_one(session, std::span<const int> {literals.begin(), literals.size()}, stats, cat);
}

auto add_pairwise_at_most_one(
    CadicalSession& session,
    const std::span<const int> literals,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    const auto n = literals.size();
    if (n < 2) {
        return;
    }
    std::size_t clause_count = 0;
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            session.add_clause({-literals[i], -literals[j]});
            ++clause_count;
        }
    }
    record_clause(stats, cat, clause_count);
}

auto add_pairwise_at_most_one(
    CadicalSession& session,
    const std::initializer_list<int> literals,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    add_pairwise_at_most_one(
        session, std::span<const int> {literals.begin(), literals.size()}, stats, cat);
}

auto add_sequential_at_most_one(
    CadicalSession& session,
    const std::span<const int> literals,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    if (literals.size() <= 1) {
        return;
    }
    auto prefix = std::Vector<int> {};
    prefix.reserve(literals.size() - 1);
    for (std::size_t i = 0; i + 1 < literals.size(); ++i) {
        prefix.push_back(session.new_var());
    }

    std::size_t clause_count = 0;
    session.add_clause({-literals.front(), prefix.front()});
    ++clause_count;
    for (std::size_t i = 1; i + 1 < literals.size(); ++i) {
        session.add_clause({-literals[i], prefix[i]});
        session.add_clause({-prefix[i - 1], prefix[i]});
        session.add_clause({-literals[i], -prefix[i - 1]});
        clause_count += 3;
    }
    session.add_clause({-literals.back(), -prefix.back()});
    ++clause_count;
    record_clause(stats, cat, clause_count);
}

auto add_sequential_at_most_one(
    CadicalSession& session,
    const std::initializer_list<int> literals,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    add_sequential_at_most_one(
        session, std::span<const int> {literals.begin(), literals.size()}, stats, cat);
}

auto add_exactly_one(
    CadicalSession& session,
    const std::span<const int> literals,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    add_at_least_one(session, literals, stats, cat);
    add_pairwise_at_most_one(session, literals, stats, cat);
}

auto add_exactly_one(
    CadicalSession& session,
    const std::initializer_list<int> literals,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    add_exactly_one(session, std::span<const int> {literals.begin(), literals.size()}, stats, cat);
}

auto add_equiv(
    CadicalSession& session,
    const int a,
    const int b,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    session.add_clause({-a, b});
    session.add_clause({a, -b});
    record_clause(stats, cat, 2);
}

auto add_implies(
    CadicalSession& session,
    const int antecedent,
    const int consequent,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    session.add_clause({-antecedent, consequent});
    record_clause(stats, cat);
}

auto add_or_equiv(
    CadicalSession& session,
    const int out,
    const std::span<const int> inputs,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    for (const int input : inputs) {
        add_implies(session, input, out, stats, cat);
    }
    auto clause = std::Vector<int> {inputs.begin(), inputs.end()};
    clause.push_back(-out);
    session.add_clause(clause);
    record_clause(stats, cat);
}

auto add_or_equiv(
    CadicalSession& session,
    const int out,
    const std::initializer_list<int> inputs,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    add_or_equiv(session, out, std::span<const int> {inputs.begin(), inputs.size()}, stats, cat);
}

auto add_binary_successor(
    CadicalSession& session,
    const std::span<const int> input_bits,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> BinarySuccessorVars {
    if (input_bits.empty()) {
        throw std::invalid_argument("binary successor requires a non-empty bit vector");
    }

    auto out = BinarySuccessorVars {};
    out.bits.reserve(input_bits.size());
    const int low_bit = session.new_var();
    out.bits.push_back(low_bit);
    session.add_clause({input_bits.front(), low_bit});
    session.add_clause({-input_bits.front(), -low_bit});
    record_clause(stats, cat, 2);

    int carry = input_bits.front();
    for (std::size_t bit = 1; bit < input_bits.size(); ++bit) {
        const int successor_bit = session.new_var();
        out.bits.push_back(successor_bit);
        add_xor_equiv(session, successor_bit, input_bits[bit], carry, stats, cat);

        const int next_carry = session.new_var();
        add_and_equiv(session, next_carry, input_bits[bit], carry, stats, cat);
        carry = next_carry;
    }
    out.overflow = carry;
    return out;
}

auto add_conditional_successor(
    CadicalSession& session,
    const int condition,
    const BinarySuccessorVars& successor,
    const std::span<const int> output_bits,
    SatEncodingStats* stats,
    const SatClauseCategory cat
) -> void {
    if (successor.bits.empty() || successor.bits.size() != output_bits.size()
        || successor.overflow <= 0) {
        throw std::invalid_argument(
            "conditional successor requires equal non-empty bit vectors");
    }
    session.add_clause({-condition, -successor.overflow});
    record_clause(stats, cat);
    for (std::size_t bit = 0; bit < output_bits.size(); ++bit) {
        session.add_clause({-condition, -successor.bits[bit], output_bits[bit]});
        session.add_clause({-condition, successor.bits[bit], -output_bits[bit]});
        record_clause(stats, cat, 2);
    }
}

} // namespace PR_tool
