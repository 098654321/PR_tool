#pragma once

#include "ilp_allocation/gurobi.hh"
#include "mcf/cob_mcf_router.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>

namespace PR_tool::circuit {
class BaseDie;
}

namespace PR_tool {

struct MazeCheckSummary {
    int failed_units{0};
    int failed_records{0};
    int unique_origins_routed{0};
    int maze_ok{0};
    int maze_failed{0};
    int maze_skipped{0};
};

auto run_maze_check_ilp_mcf_after_mcf(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    CobMcfGridDims cob_grid
) -> MazeCheckSummary;

auto run_maze_check_mcf_after_mcf(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    CobMcfGridDims cob_grid
) -> MazeCheckSummary;

} // namespace PR_tool
