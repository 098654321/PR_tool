#include "sat/sat_constraint_kits.hh"

namespace PR_tool {

auto add_at_least_one(CadicalSession& session, const std::span<const int> literals) -> void {
    session.add_clause(literals);
}

auto add_at_least_one(CadicalSession& session, const std::initializer_list<int> literals) -> void {
    add_at_least_one(session, std::span<const int> {literals.begin(), literals.size()});
}

auto add_pairwise_at_most_one(CadicalSession& session, const std::span<const int> literals) -> void {
    for (std::size_t i = 0; i < literals.size(); ++i) {
        for (std::size_t j = i + 1; j < literals.size(); ++j) {
            session.add_clause({-literals[i], -literals[j]});
        }
    }
}

auto add_pairwise_at_most_one(CadicalSession& session, const std::initializer_list<int> literals) -> void {
    add_pairwise_at_most_one(session, std::span<const int> {literals.begin(), literals.size()});
}

auto add_sequential_at_most_one(
    CadicalSession& session,
    const std::span<const int> literals
) -> void {
    if (literals.size() <= 1) {
        return;
    }
    auto prefix = std::Vector<int> {};
    prefix.reserve(literals.size() - 1);
    for (std::size_t i = 0; i + 1 < literals.size(); ++i) {
        prefix.push_back(session.new_var());
    }

    session.add_clause({-literals.front(), prefix.front()});
    for (std::size_t i = 1; i + 1 < literals.size(); ++i) {
        session.add_clause({-literals[i], prefix[i]});
        session.add_clause({-prefix[i - 1], prefix[i]});
        session.add_clause({-literals[i], -prefix[i - 1]});
    }
    session.add_clause({-literals.back(), -prefix.back()});
}

auto add_sequential_at_most_one(
    CadicalSession& session,
    const std::initializer_list<int> literals
) -> void {
    add_sequential_at_most_one(
        session, std::span<const int> {literals.begin(), literals.size()});
}

auto add_exactly_one(CadicalSession& session, const std::span<const int> literals) -> void {
    add_at_least_one(session, literals);
    add_pairwise_at_most_one(session, literals);
}

auto add_exactly_one(CadicalSession& session, const std::initializer_list<int> literals) -> void {
    add_exactly_one(session, std::span<const int> {literals.begin(), literals.size()});
}

auto add_equiv(CadicalSession& session, const int a, const int b) -> void {
    session.add_clause({-a, b});
    session.add_clause({a, -b});
}

auto add_implies(CadicalSession& session, const int antecedent, const int consequent) -> void {
    session.add_clause({-antecedent, consequent});
}

auto add_or_equiv(
    CadicalSession& session,
    const int out,
    const std::span<const int> inputs
) -> void {
    for (const int input : inputs) {
        add_implies(session, input, out);
    }
    auto clause = std::Vector<int> {inputs.begin(), inputs.end()};
    clause.push_back(-out);
    session.add_clause(clause);
}

auto add_or_equiv(
    CadicalSession& session,
    const int out,
    const std::initializer_list<int> inputs
) -> void {
    add_or_equiv(session, out, std::span<const int> {inputs.begin(), inputs.size()});
}

namespace {

auto add_xor_equiv(CadicalSession& session, const int out, const int a, const int b) -> void {
    session.add_clause({-a, -b, -out});
    session.add_clause({a, b, -out});
    session.add_clause({a, -b, out});
    session.add_clause({-a, b, out});
}

auto add_and_equiv(CadicalSession& session, const int out, const int a, const int b) -> void {
    session.add_clause({-out, a});
    session.add_clause({-out, b});
    session.add_clause({out, -a, -b});
}

} // namespace

auto add_binary_successor(
    CadicalSession& session,
    const std::span<const int> input_bits
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

    int carry = input_bits.front();
    for (std::size_t bit = 1; bit < input_bits.size(); ++bit) {
        const int successor_bit = session.new_var();
        out.bits.push_back(successor_bit);
        add_xor_equiv(session, successor_bit, input_bits[bit], carry);

        const int next_carry = session.new_var();
        add_and_equiv(session, next_carry, input_bits[bit], carry);
        carry = next_carry;
    }
    out.overflow = carry;
    return out;
}

auto add_conditional_successor(
    CadicalSession& session,
    const int condition,
    const BinarySuccessorVars& successor,
    const std::span<const int> output_bits
) -> void {
    if (successor.bits.empty() || successor.bits.size() != output_bits.size()
        || successor.overflow <= 0) {
        throw std::invalid_argument(
            "conditional successor requires equal non-empty bit vectors");
    }
    session.add_clause({-condition, -successor.overflow});
    for (std::size_t bit = 0; bit < output_bits.size(); ++bit) {
        session.add_clause({-condition, -successor.bits[bit], output_bits[bit]});
        session.add_clause({-condition, successor.bits[bit], -output_bits[bit]});
    }
}

} // namespace PR_tool
