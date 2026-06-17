#include "mcf/mcf_gurobi_log_io.hh"

#include "ilp_allocation/gurobi_model_stats.hh"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace PR_tool {

namespace {

auto resolve_log_dir(const std::String& log_dir) -> std::filesystem::path {
    if (log_dir.empty()) {
        return std::filesystem::path(std::format("./{}", kGurobiLogSubdir));
    }
    return std::filesystem::path(log_dir);
}

auto stage_index(const McfGurobiLogStage& stage) -> std::size_t {
    if (stage.kind == McfGurobiLogStageKind::Bus) {
        return 0;
    }
    return 1 + stage.unit;
}

auto format_component_meta_lines(const McfGurobiSolveMeta& meta) -> std::String {
    if (meta.component_count <= 1 && meta.component_id < 0) {
        return std::String {};
    }
    auto out = std::format(
        "component_id={}\n"
        "component_count={}\n",
        meta.component_id,
        meta.component_count > 0 ? meta.component_count : 1);
    if (!meta.component_summary.empty()) {
        out += std::format("component_summary={}\n", meta.component_summary);
    }
    return out;
}

} // namespace

McfGurobiLogSink::McfGurobiLogSink(const std::filesystem::path output_dir) : output_dir_ {output_dir} {}

auto McfGurobiLogSink::prepare_output_dir(const std::String& log_dir) -> void {
    const auto dir = resolve_log_dir(log_dir);
    std::error_code ec {};
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir / kMcfGurobiPrmDir, ec);
}

auto McfGurobiLogSink::stage_label(const McfGurobiLogStage& stage) const -> std::String {
    if (stage.kind == McfGurobiLogStageKind::Bus) {
        return "bus";
    }
    return std::format("simple-unit{}", stage.unit);
}

auto McfGurobiLogSink::stage_log_path(const McfGurobiLogStage& stage) const -> std::String {
    if (stage.kind == McfGurobiLogStageKind::Bus) {
        return std::format("{}/{}", output_dir_.string(), kMcfGurobiBusLogFile);
    }
    return std::format("{}/{}", output_dir_.string(), std::format(kMcfGurobiSimpleUnitLogFmt, stage.unit));
}

auto McfGurobiLogSink::stage_prm_basename(const McfGurobiLogStage& stage, const int solve_id) const -> std::String {
    return std::format("{}_solve{}.prm", stage_label(stage), solve_id);
}

auto McfGurobiLogSink::retry_kind_name(const McfGurobiRetryKind kind) const -> std::string_view {
    switch (kind) {
        case McfGurobiRetryKind::None:
            return "none";
        case McfGurobiRetryKind::NoWarmStart:
            return "no_warm_start";
    }
    return "none";
}

auto McfGurobiLogSink::next_solve_id(const McfGurobiLogStage& stage) -> int {
    const auto idx = stage_index(stage);
    solve_ids_[idx] += 1;
    return solve_ids_[idx];
}

auto McfGurobiLogSink::format_timestamp() const -> std::String {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf {};
#if defined(_WIN32)
    localtime_s(&tm_buf, &time);
#else
    localtime_r(&time, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
    return oss.str();
}

auto McfGurobiLogSink::append_lines(const std::String& path, const std::String& content) -> void {
    std::ofstream out {path, std::ios::app};
    if (!out.is_open()) {
        return;
    }
    out << content;
    if (!content.empty() && content.back() != '\n') {
        out << '\n';
    }
}

auto McfGurobiLogSink::begin_solve(const McfGurobiSolveMeta& meta) -> McfGurobiSolvePaths {
    const std::lock_guard lock {mutex_};
    auto out = McfGurobiSolvePaths {};
    out.solve_id = next_solve_id(meta.stage);
    out.log_path = stage_log_path(meta.stage);
    out.prm_path = std::format("{}/{}/{}", output_dir_.string(), kMcfGurobiPrmDir, stage_prm_basename(meta.stage, out.solve_id));

    const auto header = std::format(
        "=== MCF Gurobi solve begin ===\n"
        "timestamp={}\n"
        "stage={}\n"
        "solve_id={}\n"
        "tier={}\n"
        "sat_tier_attempt={}\n"
        "bbox_attempt={}\n"
        "warm_start={}\n"
        "retry_kind={}\n"
        "pass={}\n"
        "{}"
        "settings_prm={}\n",
        format_timestamp(),
        stage_label(meta.stage),
        out.solve_id,
        meta.tier,
        meta.sat_tier_attempt,
        meta.bbox_attempt,
        meta.warm_start ? "yes" : "no",
        retry_kind_name(meta.retry_kind),
        simple_mcf_solve_pass_name(meta.pass),
        format_component_meta_lines(meta),
        out.prm_path);
    append_lines(out.log_path, header);
    return out;
}

auto McfGurobiLogSink::configure_model_log(GRBModel& model, const McfGurobiSolvePaths& paths) -> void {
    model.set(GRB_IntParam_OutputFlag, 1);
    model.set(GRB_IntParam_LogToConsole, 0);
    model.set(GRB_StringParam_LogFile, paths.log_path);
    model.set(GRB_IntParam_DisplayInterval, 5);
}

auto McfGurobiLogSink::write_settings_prm(GRBModel& model, const std::String& prm_path) -> void {
    model.write(prm_path);
}

auto McfGurobiLogSink::end_solve(
    const McfGurobiSolveMeta& meta,
    const McfGurobiSolvePaths& paths,
    GRBModel& model,
    const McfSolutionClass solution_class
) -> void {
    const std::lock_guard lock {mutex_};
    double runtime_sec = 0.0;
    double obj_val = 0.0;
    double obj_bound = 0.0;
    double mip_gap = 0.0;
    double node_count = 0.0;
    int model_status = 0;
    int sol_count = 0;
    try {
        model_status = model.get(GRB_IntAttr_Status);
        runtime_sec = model.get(GRB_DoubleAttr_Runtime);
        node_count = model.get(GRB_DoubleAttr_NodeCount);
        sol_count = model.get(GRB_IntAttr_SolCount);
        if (stage_result_usable(solution_class)) {
            obj_val = model.get(GRB_DoubleAttr_ObjVal);
            obj_bound = model.get(GRB_DoubleAttr_ObjBound);
            mip_gap = model.get(GRB_DoubleAttr_MIPGap);
        }
        else if (model_status == GRB_INFEASIBLE || model_status == GRB_INF_OR_UNBD) {
            try {
                obj_bound = model.get(GRB_DoubleAttr_ObjBound);
            }
            catch (...) {
            }
        }
    }
    catch (...) {
    }

    const auto footer = std::format(
        "=== MCF Gurobi solve end ===\n"
        "timestamp={}\n"
        "stage={}\n"
        "solve_id={}\n"
        "solution_class={}\n"
        "model_status={}\n"
        "runtime_sec={:.6f}\n"
        "node_count={:.0f}\n"
        "sol_count={}\n"
        "obj_val={:.6f}\n"
        "obj_bound={:.6f}\n"
        "mip_gap={:.6f}\n",
        format_timestamp(),
        stage_label(meta.stage),
        paths.solve_id,
        solution_class_name(solution_class),
        model_status,
        runtime_sec,
        node_count,
        sol_count,
        obj_val,
        obj_bound,
        mip_gap);
    append_lines(paths.log_path, footer);
}

auto McfGurobiLogSink::end_solve_exception(
    const McfGurobiSolveMeta& meta,
    const McfGurobiSolvePaths& paths,
    const McfSolutionClass solution_class,
    const std::string_view detail
) -> void {
    const std::lock_guard lock {mutex_};
    const auto footer = std::format(
        "=== MCF Gurobi solve end ===\n"
        "timestamp={}\n"
        "stage={}\n"
        "solve_id={}\n"
        "solution_class={}\n"
        "detail={}\n",
        format_timestamp(),
        stage_label(meta.stage),
        paths.solve_id,
        solution_class_name(solution_class),
        detail);
    append_lines(paths.log_path, footer);
}

auto McfGurobiLogSink::write_stub(
    const McfGurobiSolveMeta& meta,
    const std::string_view reason,
    const std::string_view detail
) -> void {
    const std::lock_guard lock {mutex_};
    const auto solve_id = next_solve_id(meta.stage);
    const auto path = stage_log_path(meta.stage);
    auto content = std::format(
        "=== MCF Gurobi solve stub ===\n"
        "timestamp={}\n"
        "stage={}\n"
        "solve_id={}\n"
        "tier={}\n"
        "sat_tier_attempt={}\n"
        "bbox_attempt={}\n"
        "warm_start={}\n"
        "retry_kind={}\n"
        "{}"
        "reason={}\n",
        format_timestamp(),
        stage_label(meta.stage),
        solve_id,
        meta.tier,
        meta.sat_tier_attempt,
        meta.bbox_attempt,
        meta.warm_start ? "yes" : "no",
        retry_kind_name(meta.retry_kind),
        format_component_meta_lines(meta),
        reason);
    if (!detail.empty()) {
        content += std::format("detail={}\n", detail);
    }
    append_lines(path, content);
}

auto McfGurobiLogSink::write_skipped(const McfGurobiLogStage& stage, const std::string_view reason) -> void {
    const std::lock_guard lock {mutex_};
    const auto path = stage_log_path(stage);
    const auto content = std::format(
        "=== MCF Gurobi skipped ===\n"
        "timestamp={}\n"
        "stage={}\n"
        "reason={}\n",
        format_timestamp(),
        stage_label(stage),
        reason);
    append_lines(path, content);
}

} // namespace PR_tool
