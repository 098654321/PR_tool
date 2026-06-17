#pragma once

#include "gurobi_c++.h"

#include <format>
#include <std/string.hh>

namespace PR_tool {

/// MCF-only Gurobi MIP parameters. Edit default member values below for manual tuning.
/// Gurobi convention: -1 = automatic where applicable.
struct McfGurobiSolveParams {
    int presolve{2};       // GRB_IntParam_Presolve: -1 auto, 0 off, 1 conservative, 2 aggressive
    int pre_sparsify{1};  // GRB_IntParam_PreSparsify: -1 auto, 0 off, 1 MIP, 2 all models
    int symmetry{2};      // GRB_IntParam_Symmetry: -1 auto, 0 off, 1 conservative, 2 aggressive
    int mip_focus{2};      // GRB_IntParam_MIPFocus: 0 balance, 1 feasible, 2 optimal, 3 bound
    int cuts{-1};          // GRB_IntParam_Cuts: -1 auto, 0 off, 1 conservative, 2 aggressive, 3 very aggressive
    int threads{0};        // GRB_IntParam_Threads; set per solve (Bus 8/4, Simple 2)
};

inline auto default_mcf_gurobi_solve_params() -> McfGurobiSolveParams {
    return McfGurobiSolveParams {};
}

inline auto apply_mcf_gurobi_solve_params(GRBModel& model, const McfGurobiSolveParams& params) -> void {
    model.set(GRB_IntParam_Presolve, params.presolve);
    model.set(GRB_IntParam_PreSparsify, params.pre_sparsify);
    model.set(GRB_IntParam_Symmetry, params.symmetry);
    model.set(GRB_IntParam_MIPFocus, params.mip_focus);
    model.set(GRB_IntParam_Cuts, params.cuts);
    if (params.threads > 0) {
        model.set(GRB_IntParam_Threads, params.threads);
    }
}

inline auto format_mcf_gurobi_solve_params(const McfGurobiSolveParams& params) -> std::String {
    return std::format(
        "Presolve={} PreSparsify={} Symmetry={} MIPFocus={} Cuts={} Threads={}",
        params.presolve,
        params.pre_sparsify,
        params.symmetry,
        params.mip_focus,
        params.cuts,
        params.threads);
}

} // namespace PR_tool
