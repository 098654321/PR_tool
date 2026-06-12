#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "common/tob_bbox_expansion.hh"
#include "precompute/tob_path_precompute.hh"
#include "sat_allocation/cadical_solver.hh"

#include <hardware/interposer.hh>

namespace PR_tool {

auto solve_tob_sat_with_tier_state(
    std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache,
    const TobTierState& state,
    const CadicalDiagnosticsOptions& diag = {}
) -> TobIlpResult;

auto solve_tob_sat_with_cadical(
    std::Vector<Net_cost_record>& records,
    hardware::Interposer* interposer,
    const CadicalDiagnosticsOptions& diag = {},
    bool enable_presat_parallel = false
) -> TobIlpResult;

} // namespace PR_tool
