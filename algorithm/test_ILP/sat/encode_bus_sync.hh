#pragma once

#include "common/routing_types.hh"
#include "sat/sat_encoding_stats.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat_allocation/cadical_solver.hh"

namespace PR_tool {

auto encode_bus_sync_constraints(
    CadicalSession& session,
    const std::Vector<RoutingNet>& nets,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void;

} // namespace PR_tool
