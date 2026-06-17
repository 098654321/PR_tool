#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "ilp_allocation/gurobi_model_stats.hh"

#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

auto solve_tob_ilp_with_gurobi(
    const std::Vector<Net_cost_record>& records,
    bool enable_parallel = false,
    const GurobiDiagnosticsOptions& diag = {}
)
    -> TobIlpResult;

} // namespace PR_tool
