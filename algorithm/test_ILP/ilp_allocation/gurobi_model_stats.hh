#pragma once

#include "gurobi_c++.h"

#include <std/collection.hh>
#include <std/string.hh>
#include <string_view>

namespace PR_tool {

inline constexpr std::StringView kGurobiLogSubdir {"gurobi-log"};
inline constexpr std::StringView kGurobiModelInfoFilename {"modelinfo.log"};

struct GurobiDiagnosticsOptions {
    std::String log_dir {"./gurobi-log"};
    int heavy_row_min_nnz{10};
    double heavy_row_factor{5.0};
    int heavy_row_top_k{10};
};

struct GurobiRowMeta {
    std::String kind;
    std::String detail;
};

struct GurobiHeavyConstraintRow {
    int row_index{0};
    int nnz{0};
    std::String constr_name;
    std::String detail;
};

struct GurobiMatrixSparsityStats {
    int rows{0};
    int cols{0};
    long long nnz{0};
    double density{0.0};
    int max_row_nnz{0};
    double avg_row_nnz{0.0};
    bool sparse{false};
    int heavy_threshold_nnz{0};
    std::Vector<GurobiHeavyConstraintRow> heavy_rows;
};

auto init_gurobi_modelinfo_log(const std::String& log_dir) -> void;

auto gurobi_modelinfo_log_path(const std::String& log_dir) -> std::String;

auto log_gurobi_modelinfo(const std::String& log_dir, const std::String& line) -> void;

auto compute_gurobi_matrix_sparsity(
    GRBModel& model,
    const GurobiDiagnosticsOptions& opts,
    const std::Vector<GurobiRowMeta>* row_meta = nullptr
) -> GurobiMatrixSparsityStats;

auto log_gurobi_matrix_diagnostics(
    GRBModel& model,
    const std::String& stage_name,
    const GurobiDiagnosticsOptions& opts,
    const std::Vector<GurobiRowMeta>* row_meta = nullptr
) -> void;

} // namespace PR_tool
