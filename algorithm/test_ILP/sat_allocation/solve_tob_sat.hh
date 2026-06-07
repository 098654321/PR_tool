#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "sat_allocation/cadical_solver.hh"

namespace PR_tool {

auto solve_tob_sat_with_cadical(
    std::Vector<Net_cost_record>& records,
    const CadicalDiagnosticsOptions& diag = {}
) -> TobIlpResult;

} // namespace PR_tool
