#pragma once

#include "ilp_allocation/gurobi.hh"
#include "common/ilp_types.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>
#include <std/string.hh>
#include <array>

namespace PR_tool::circuit {
class BaseDie;
} // namespace PR_tool::circuit

namespace PR_tool {

/// COB tile grid size for MCF graph construction (must match `hardware::Interposer::COB_ARRAY_*` when passed from CLI).
struct CobMcfGridDims {
    int rows{0};
    int cols{0};
};

struct CobMcfCobUnitSummary {
    std::size_t cob_unit{0};
    int num_commodities{0};
    bool ok{false};
    double objective{0.0};
    int solve_ms{0};
    std::String message;
};

struct CobMcfRunSummary {
    std::Vector<CobMcfCobUnitSummary> per_cob;
    bool all_ok{true};
    /// Wall time for MCF graph warm-start routing (`--enable-pre-routing`); 0 if disabled.
    int mcf_warm_start_ms{0};
    /// Wall time for BusMCF + SimpleMCF Gurobi solves only.
    int mcf_solve_ms{0};
};

struct McfPathInfo {
    std::String label;
    std::String origin_name;
    std::size_t record_id{0};
    int src{0};
    int snk{0};
    int demand{0};
    std::size_t cob_unit{0};
    std::size_t start_track{0};
    std::size_t end_track{0};
    std::Vector<std::size_t> record_indices;
    std::Vector<std::Vector<int>> unit_paths;
    std::Vector<std::Vector<std::size_t>> track_paths;
};

struct CobMcfFullResult {
    CobMcfRunSummary summary;
    std::array<std::Vector<McfPathInfo>, 16> paths_by_unit {};
    /// Per-unit SimpleMCF Gurobi solve succeeded (undefined if unit has no simple commodities).
    std::array<bool, 16> simple_mcf_ok {};
    /// Unit has at least one non-Bus SimpleMCF commodity.
    std::array<bool, 16> has_simple_commodities {};
};

/// Per-cobunit MCF: getNetinCOBUnit → merge → build_commodities → BusMCF (global) + SimpleMCF (per unit).
/// When \p enable_mcf_parallel is true, SimpleMCF runs concurrently (one Gurobi model per COBUnit).
/// SimpleMCF uses v5 undirected physical-edge x^H_e and origin-level o^H_i (multi-fanout and single 2-pin).
/// When \p enable_mcf_obj is true, SimpleMCF adds min Σ x objective; otherwise feasibility-only (zero costs).
auto run_mcf_global_routing_cob_units(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    hardware::Interposer* interposer,
    const circuit::BaseDie& basedie,
    CobMcfGridDims cob_grid,
    bool enable_mcf_parallel = false,
    bool enable_pre_routing = false,
    bool enable_mcf_obj = false,
    bool defer_interposer_suspend = false,
    bool disable_bus_mcf = false
) -> CobMcfFullResult;

} // namespace PR_tool
