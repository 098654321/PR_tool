#pragma once

#include "common/ilp_types.hh"

#include <cstddef>
#include <functional>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct TobSatCnf {
    std::size_t num_vars{0};
    std::size_t num_clauses{0};
    std::Vector<std::Vector<int>> clauses;
    std::map<std::String, int> var_index;
    std::function<int(std::string_view name)> literal;
};

auto build_tob_sat_cnf(const std::Vector<Net_cost_record>& records) -> TobSatCnf;

auto a_var(const Bump_coord& b, std::size_t track) -> std::String;

} // namespace PR_tool
