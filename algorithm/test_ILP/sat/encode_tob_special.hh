#pragma once

#include "sat/sat_encoding_stats.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat_allocation/cadical_solver.hh"

namespace PR_tool {

auto encode_tob_special_constraints(
    CadicalSession& session,
    const UnifiedGraph& graph,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void;

} // namespace PR_tool
