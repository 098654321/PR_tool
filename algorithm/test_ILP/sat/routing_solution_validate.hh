#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat_allocation/cadical_solver.hh"

#include <map>

namespace PR_tool {

enum class ViolationKind {
    EndpointMismatch,
    MissingArc,
    InvalidNodeTransition,
    DelayReplayMismatch,
    TrackBumpConflict,
    TobSwitchConflict,
    PartialMatchingConflict,
    BnetUnitConflict,
    SyncBusDelayMismatch,
    PnnetTrackRule,
    NodeOutOfScope
};

struct Violation {
    ViolationKind kind {ViolationKind::EndpointMismatch};
    std::size_t net_id {0};
    std::size_t demand_id {0};
    std::size_t source_index {0};
    int node_id {-1};
    int arc_id {-1};
    std::String detail;
};

struct ValidationReport {
    bool pass {true};
    std::size_t net_count {0};
    std::size_t path_count {0};
    std::size_t track_bump_nodes {0};
    std::size_t tob_switches {0};
    std::size_t mode_groups {0};
    std::size_t violations_count {0};
    std::Vector<Violation> violations;
    std::map<ViolationKind, std::size_t> category_counts;
};

auto validate_routing_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const UnifiedSatModel& model,
    const CadicalSession& session,
    const SatRoutingResult& out
) -> ValidationReport;

auto log_validation_report(const ValidationReport& report, int verbose_level) -> void;

} // namespace PR_tool
