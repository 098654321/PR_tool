#pragma once

#include "common/routing_types.hh"
#include "delay/pair_delay_precompute.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/sat_encoding_stats.hh"
#include "sat/unified_sat_scope.hh"
#include "sat_allocation/cadical_solver.hh"
#include "scope/pair_routing_state.hh"

namespace PR_tool {

struct SourceDelayVars {
    std::size_t net_id{0};
    std::size_t source_index{0};
    int source_node{-1};
    std::size_t scope_index{0};
    std::size_t model_source_index{0};
    int d_max{0};
    // Positive literal for an active exact-delay state; -1 when pruned.
    std::Vector<std::Vector<int>> d_var;
    // Bnet only: exactly one selected physical COBUnit.
    std::Vector<int> unit_selector_var_by_unit;
};

struct TobArcDelayVars {
    int arc_global_id{-1};
    std::size_t model_source_index{0};
    int d_max{0};
    // a_var[d], 0 when not allocated
    std::Vector<int> a_var;
};

struct PairAlphaVar {
    PairKey key;
    int alpha_lit{0};
};

struct UnifiedSatModel {
    std::Vector<UnifiedSatNetScope> scopes;
    std::Vector<SourceDelayVars> sources;
    std::Vector<TobArcDelayVars> tob_arcs;
    std::map<std::pair<std::size_t, int>, std::size_t> tob_arc_index;
    std::Vector<PairDelayInfo> pair_delays;
    std::Vector<PairAlphaVar> alpha_vars;
    std::map<PairKey, int> alpha_lit_by_pair;
    std::map<int, int> mode_var_by_group;
    std::map<int, int> switch_var_by_id;
};

auto build_unified_sat_model(
    CadicalSession& session,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const DelayPrecomputeResult& delays,
    SatEncodingStats* stats = nullptr
) -> UnifiedSatModel;

auto is_tob_arc(const UnifiedArc& arc) -> bool;

auto find_tob_arc_vars(
    const UnifiedSatModel& model,
    std::size_t model_source_index,
    int arc_global_id
) -> const TobArcDelayVars*;

auto tob_a_literal(
    const UnifiedSatModel& model,
    std::size_t model_source_index,
    int arc_global_id,
    int delay
) -> int;

} // namespace PR_tool
