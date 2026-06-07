#pragma once

#include "sat_allocation/tob_sat_encoder.hh"

#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct CadicalDiagnosticsOptions {
    bool enable_sat_log{false};
    bool verbose_reach_endpoints{false};
    std::String log_dir {"./cadical-log"};
    std::size_t range_level{0};
};

struct CadicalSolveResult {
    bool ok{false};
    int status{0};
    std::String message;
    std::map<std::String, bool> assignment;
};

auto solve_tob_sat_cnf(const TobSatCnf& cnf, const CadicalDiagnosticsOptions& diag) -> CadicalSolveResult;

} // namespace PR_tool
