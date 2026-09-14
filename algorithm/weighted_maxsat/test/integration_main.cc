#include "wmaxsat_router.hh"

#include <parse/reader/module.hh>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace PR_tool;

auto require(const bool condition, const std::string& message) -> void {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

auto contains_literal(const std::Vector<int>& clause, const int literal) -> bool {
    return std::find(clause.begin(), clause.end(), literal) != clause.end();
}

auto find_case_directory() -> std::filesystem::path {
    auto root = std::filesystem::current_path();
    for (;;) {
        const auto candidate = root / "test/module_test/test_function/testlength/testiosimple";
        if (std::filesystem::is_directory(candidate)) {
            return candidate;
        }
        const auto parent = root.parent_path();
        if (parent == root) {
            throw std::runtime_error("cannot locate testlength/testiosimple");
        }
        root = parent;
    }
}

auto test_small_case_encodes_to_wcnf() -> void {
    const auto case_dir = find_case_directory();
    auto [interposer, basedie] = parse::read_config(case_dir, 0, false);
    const auto encoding = build_wmaxsat_encoding(interposer.get(), *basedie);

    require(!encoding.model.pair_delays.empty(), "small case must produce routing pairs");
    require(encoding.q_by_pair.size() == encoding.model.pair_delays.size(), "every pair needs q");
    require(!encoding.hard_clauses.empty(), "encoding must contain hard constraints");
    require(!encoding.soft_clauses.empty(), "encoding must contain soft constraints");
    for (const auto& [_, q] : encoding.q_by_pair) {
        bool has_negative_q_clause = false;
        bool has_positive_q_clause = false;
        for (const auto& clause : encoding.hard_clauses) {
            has_negative_q_clause = has_negative_q_clause || contains_literal(clause, -q);
            has_positive_q_clause = has_positive_q_clause || contains_literal(clause, q);
        }
        require(has_negative_q_clause && has_positive_q_clause, "q must be encoded in both directions");
    }

    const auto wcnf = std::filesystem::temp_directory_path() / "weighted_maxsat_integration.wcnf";
    write_wcnf(encoding, wcnf);
    auto input = std::ifstream {wcnf};
    auto header = std::string {};
    std::getline(input, header);
    require(header.rfind("p wcnf ", 0) == 0, "integration WCNF must have a WCNF header");
    std::filesystem::remove(wcnf);
}

} // namespace

auto main() -> int {
    try {
        test_small_case_encodes_to_wcnf();
        std::cout << "weighted_maxsat_integration: all tests passed\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "weighted_maxsat_integration: " << error.what() << '\n';
        return 1;
    }
}
