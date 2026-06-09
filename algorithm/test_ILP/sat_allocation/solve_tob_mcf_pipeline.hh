#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "ilp_allocation/gurobi.hh"
#include "mcf/cob_mcf_router.hh"
#include "sat_allocation/cadical_solver.hh"

#include <hardware/interposer.hh>
#include <std/string.hh>

namespace PR_tool::circuit {
class BaseDie;
} // namespace PR_tool::circuit

namespace PR_tool {

struct TobMcfPipelineResult {
    bool ok{false};
    std::String message;
    TobIlpResult tob;
    CobMcfFullResult mcf;
    std::size_t range_level{0};
    std::size_t attempts{0};
    long long tob_sat_solve_ms{0};
    long long mcf_warm_start_ms{0};
    long long mcf_solve_ms{0};
};

auto solve_tob_mcf_with_range_iteration(
    std::Vector<Net_cost_record>& records,
    hardware::Interposer* interposer,
    const circuit::BaseDie& basedie,
    CobMcfGridDims cob_grid,
    bool enable_mcf_parallel,
    bool enable_pre_routing,
    bool enable_mcf_obj,
    bool disable_bus_mcf,
    bool enable_simple_maze,
    bool apply_interposer_suspend_on_success,
    const CadicalDiagnosticsOptions& sat_diag,
    const GurobiDiagnosticsOptions& gurobi_diag
) -> TobMcfPipelineResult;

} // namespace PR_tool
