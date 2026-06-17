#include "sat_allocation/cadical_solver.hh"

#ifndef USE_CADICAL
#error "USE_CADICAL is required for SAT TOB allocation; build with xmake f --cadical=y and compile third_party/cadical"
#endif

#include <cadical.hpp>
#include <debug/debug.hh>

#include <cstdio>
#include <filesystem>
#include <format>

namespace PR_tool {

auto solve_tob_sat_cnf(const TobSatCnf& cnf, const CadicalDiagnosticsOptions& diag) -> CadicalSolveResult {
    auto out = CadicalSolveResult {};
    CaDiCaL::Solver solver;
    solver.set("quiet", 1);

    if (diag.enable_sat_log) {
        std::filesystem::create_directories(diag.log_dir);
        const auto trace_path = std::format("{}/sat_tier{}.trace", diag.log_dir, diag.max_tier);
        if (FILE* fp = std::fopen(trace_path.c_str(), "w")) {
            solver.trace_api_calls(fp);
            debug::info_fmt("CaDiCal API trace enabled: {}", trace_path);
        }
    }

    if (cnf.num_vars > 0) {
        solver.resize(static_cast<int>(cnf.num_vars));
    }

    for (const auto& clause : cnf.clauses) {
        for (const int lit : clause) {
            solver.add(lit);
        }
        solver.add(0);
    }

    const int status = solver.solve();
    out.status = status;

    if (status == CaDiCaL::SATISFIABLE) {
        out.ok = true;
        out.message = "SAT";
        for (const auto& [name, idx] : cnf.var_index) {
            const int val = solver.val(idx);
            out.assignment[name] = val > 0;
        }
        return out;
    }
    if (status == CaDiCaL::UNSATISFIABLE) {
        out.ok = false;
        out.message = "UNSAT";
        return out;
    }

    out.ok = false;
    out.message = std::format("CaDiCal returned status={}", status);
    return out;
}

} // namespace PR_tool
