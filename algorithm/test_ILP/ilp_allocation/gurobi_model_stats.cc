#include "ilp_allocation/gurobi_model_stats.hh"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <format>
#include <fstream>
#include <mutex>
#include <optional>
#include <set>

namespace PR_tool {

namespace {

constexpr double kSparseDensityThreshold = 0.01;

auto resolve_gurobi_log_dir(const std::String& log_dir) -> std::String {
    if (log_dir.empty()) {
        return std::format("./{}", kGurobiLogSubdir);
    }
    return log_dir;
}

auto ensure_gurobi_log_dir(const std::String& log_dir) -> std::String {
    const auto dir = resolve_gurobi_log_dir(log_dir);
    std::error_code ec {};
    std::filesystem::create_directories(dir, ec);
    return dir;
}

auto sanitize_stage_name(const std::String& stage) -> std::String {
    auto out = std::String {};
    for (const char c : stage) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') {
            out += c;
        }
        else {
            out += '_';
        }
    }
    return out;
}

struct RowNnzEntry {
    int constr_index{0};
    int nnz{0};
    std::String name;
};

auto parse_mcf_row_index(const std::String& constr_name) -> std::optional<std::size_t> {
    if (!constr_name.starts_with("mcf_r_")) {
        return std::nullopt;
    }
    const auto suffix = constr_name.substr(6);
    const auto under = suffix.find('_');
    const auto num_str = under == std::String::npos ? suffix : suffix.substr(0, under);
    if (num_str.empty()) {
        return std::nullopt;
    }
    try {
        return static_cast<std::size_t>(std::stoul(num_str));
    }
    catch (...) {
        return std::nullopt;
    }
}

auto row_meta_detail(
    const std::String& constr_name,
    const std::Vector<GurobiRowMeta>* row_meta
) -> std::String {
    if (row_meta == nullptr) {
        return {};
    }
    const auto row_idx = parse_mcf_row_index(constr_name);
    if (!row_idx.has_value() || *row_idx >= row_meta->size()) {
        return {};
    }
    const auto& meta = row_meta->at(*row_idx);
    if (meta.kind.empty() && meta.detail.empty()) {
        return {};
    }
    if (meta.detail.empty()) {
        return std::format(" kind={}", meta.kind);
    }
    return std::format(" kind={} detail={}", meta.kind, meta.detail);
}

auto next_gurobi_log_seq() -> int {
    static std::atomic<int> seq {0};
    return ++seq;
}

auto modelinfo_mutex() -> std::mutex& {
    static std::mutex mutex {};
    return mutex;
}

} // namespace

auto gurobi_modelinfo_log_path(const std::String& log_dir) -> std::String {
    const auto dir = resolve_gurobi_log_dir(log_dir);
    return std::format("{}/{}", dir, kGurobiModelInfoFilename);
}

auto init_gurobi_modelinfo_log(const std::String& log_dir) -> void {
    const auto path = gurobi_modelinfo_log_path(log_dir);
    ensure_gurobi_log_dir(log_dir);
    std::ofstream out {path, std::ios::trunc};
}

auto log_gurobi_modelinfo(const std::String& log_dir, const std::String& line) -> void {
    const auto path = gurobi_modelinfo_log_path(log_dir);
    ensure_gurobi_log_dir(log_dir);
    const std::lock_guard lock {modelinfo_mutex()};
    std::ofstream out {path, std::ios::app};
    if (!out.is_open()) {
        return;
    }
    out << line << '\n';
}

auto make_gurobi_log_path(
    const std::String& log_dir,
    const std::String& stage_name
) -> std::String {
    const auto base = sanitize_stage_name(stage_name);
    const auto seq = next_gurobi_log_seq();
    const auto dir = ensure_gurobi_log_dir(log_dir);
    return std::format("{}/gurobi_{}_{}.log", dir, base, seq);
}

auto compute_gurobi_matrix_sparsity(
    GRBModel& model,
    const GurobiDiagnosticsOptions& opts,
    const std::Vector<GurobiRowMeta>* row_meta
) -> GurobiMatrixSparsityStats {
    (void)row_meta;

    model.update();

    auto out = GurobiMatrixSparsityStats {};
    out.rows = model.get(GRB_IntAttr_NumConstrs);
    out.cols = model.get(GRB_IntAttr_NumVars);
    if (out.rows <= 0 || out.cols <= 0) {
        return out;
    }

    const auto constrs = model.getConstrs();
    auto entries = std::Vector<RowNnzEntry> {};
    entries.reserve(static_cast<std::size_t>(out.rows));

    for (int ci = 0; ci < out.rows; ++ci) {
        const auto& constr = constrs[ci];
        const int row_nnz = static_cast<int>(model.getRow(constr).size());
        out.nnz += row_nnz;
        out.max_row_nnz = std::max(out.max_row_nnz, row_nnz);
        entries.push_back(RowNnzEntry {ci, row_nnz, constr.get(GRB_StringAttr_ConstrName)});
    }

    out.avg_row_nnz = static_cast<double>(out.nnz) / static_cast<double>(out.rows);
    out.density = static_cast<double>(out.nnz)
        / (static_cast<double>(out.rows) * static_cast<double>(out.cols));
    out.sparse = out.density < kSparseDensityThreshold;

    const int threshold_from_avg = static_cast<int>(std::ceil(opts.heavy_row_factor * out.avg_row_nnz));
    out.heavy_threshold_nnz = std::max(opts.heavy_row_min_nnz, threshold_from_avg);

    auto heavy_indices = std::set<int> {};
    for (const auto& entry : entries) {
        if (entry.nnz >= out.heavy_threshold_nnz) {
            heavy_indices.insert(entry.constr_index);
        }
    }

    auto sorted = entries;
    std::sort(sorted.begin(), sorted.end(), [](const RowNnzEntry& a, const RowNnzEntry& b) {
        if (a.nnz != b.nnz) {
            return a.nnz > b.nnz;
        }
        return a.constr_index < b.constr_index;
    });

    const int top_k = std::max(0, opts.heavy_row_top_k);
    for (int i = 0; i < top_k && i < static_cast<int>(sorted.size()); ++i) {
        if (sorted[static_cast<std::size_t>(i)].nnz > 1) {
            heavy_indices.insert(sorted[static_cast<std::size_t>(i)].constr_index);
        }
    }

    out.heavy_rows.reserve(heavy_indices.size());
    for (const auto& entry : entries) {
        if (!heavy_indices.contains(entry.constr_index)) {
            continue;
        }
        auto heavy = GurobiHeavyConstraintRow {};
        heavy.row_index = entry.constr_index;
        heavy.nnz = entry.nnz;
        heavy.constr_name = entry.name;
        heavy.detail = row_meta_detail(entry.name, row_meta);
        out.heavy_rows.push_back(std::move(heavy));
    }

    std::sort(out.heavy_rows.begin(), out.heavy_rows.end(), [](const GurobiHeavyConstraintRow& a, const GurobiHeavyConstraintRow& b) {
        if (a.nnz != b.nnz) {
            return a.nnz > b.nnz;
        }
        return a.row_index < b.row_index;
    });

    return out;
}

auto configure_gurobi_solver_log(
    GRBEnv& env,
    const std::String& stage_name,
    const GurobiDiagnosticsOptions& opts
) -> std::String {
    if (!opts.enable_gurobi_log) {
        env.set(GRB_IntParam_OutputFlag, 0);
        return {};
    }
    const auto log_path = make_gurobi_log_path(opts.log_dir, stage_name);
    env.set(GRB_IntParam_OutputFlag, 1);
    env.set(GRB_IntParam_LogToConsole, 0);
    env.set(GRB_StringParam_LogFile, log_path);
    log_gurobi_modelinfo(opts.log_dir, std::format("{} Gurobi log file: {}", stage_name, log_path));
    return log_path;
}

auto log_gurobi_matrix_diagnostics(
    GRBModel& model,
    const std::String& stage_name,
    const GurobiDiagnosticsOptions& opts,
    const std::Vector<GurobiRowMeta>* row_meta
) -> void {
    const auto stats = compute_gurobi_matrix_sparsity(model, opts, row_meta);
    log_gurobi_modelinfo(
        opts.log_dir,
        std::format(
            "{} constraint matrix: rows={} cols={} nnz={} density={:.6e} sparse={} max_row_nnz={} avg_row_nnz={:.2f}",
            stage_name,
            stats.rows,
            stats.cols,
            stats.nnz,
            stats.density,
            stats.sparse ? "yes" : "no",
            stats.max_row_nnz,
            stats.avg_row_nnz));

    log_gurobi_modelinfo(
        opts.log_dir,
        std::format(
            "{} heavy coupling rows: count={} threshold_nnz>={} avg_row_nnz={:.2f}",
            stage_name,
            stats.heavy_rows.size(),
            stats.heavy_threshold_nnz,
            stats.avg_row_nnz));

    for (const auto& row : stats.heavy_rows) {
        log_gurobi_modelinfo(
            opts.log_dir,
            std::format(
                "  row={} nnz={} name={}{}",
                row.row_index,
                row.nnz,
                row.constr_name,
                row.detail));
    }
}

} // namespace PR_tool
