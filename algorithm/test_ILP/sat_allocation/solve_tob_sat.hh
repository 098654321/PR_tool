#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "sat_allocation/cadical_solver.hh"

namespace PR_tool {

inline constexpr std::size_t kMaxRangeLevel = 4;

auto solve_tob_sat_at_range_level(
    std::Vector<Net_cost_record>& records,
    std::size_t range_level,
    const CadicalDiagnosticsOptions& diag = {}
) -> TobIlpResult;

auto solve_tob_sat_with_cadical(
    std::Vector<Net_cost_record>& records,
    const CadicalDiagnosticsOptions& diag = {}
) -> TobIlpResult;

} // namespace PR_tool
