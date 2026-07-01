#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/sat_encoding_stats.hh"
#include "sat_allocation/cadical_solver.hh"

namespace PR_tool {

struct UnifiedSatNetScope {
    std::size_t net_id{0};
    std::Vector<int> node_ids;
    std::Vector<int> arc_ids;
    // Global ID -> compact offset, or -1 when absent.
    std::Vector<int> node_offset;
    std::Vector<int> arc_offset;
};

struct UnifiedSatLogicalSourceVars {
    std::size_t net_id{0};
    std::size_t source_index{0};
    int source_node{-1};
    std::size_t scope_index{0};
    std::Vector<int> p_vars;
};

struct UnifiedSatPairVars {
    std::size_t net_id{0};
    std::size_t demand_id{0};
    std::size_t source_index{0};
    int source_node{-1};
    int sink_node{-1};
    std::size_t scope_index{0};
    int activation{0};
    int p_sink{0};
    std::Vector<int> p_vars;
    std::Vector<int> x_vars;
    // Retained only for Sync bus result checking/debugging.
    std::Vector<int> sink_distance_bits;
};

struct UnifiedSatModel {
    std::Vector<UnifiedSatNetScope> scopes;
    std::Vector<UnifiedSatLogicalSourceVars> logical_sources;
    std::Vector<UnifiedSatPairVars> pairs;
    std::map<int, int> mode_var_by_group;
    std::map<int, int> switch_var_by_id;
};

auto build_unified_sat_model(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    SatEncodingStats* stats = nullptr
) -> UnifiedSatModel;

} // namespace PR_tool
