#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"

#include <functional>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

auto build_tob_ilp_result_from_assignment(
    const std::Vector<Net_cost_record>& records,
    const std::function<bool(std::string_view var_name)>& is_true
) -> TobIlpResult;

} // namespace PR_tool
