#pragma once

#include "mcf/cob_mcf_router.hh"

#include "gurobi_c++.h"

#include <std/string.hh>

#include <array>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string_view>

namespace PR_tool {

inline constexpr std::string_view kMcfGurobiBusLogFile {"bus.log"};
inline constexpr std::string_view kMcfGurobiSimpleUnitLogFmt {"simple-unit{}.log"};
inline constexpr std::string_view kMcfGurobiPrmDir {"prm"};

enum class McfGurobiLogStageKind { Bus, SimpleUnit };

struct McfGurobiLogStage {
    McfGurobiLogStageKind kind{McfGurobiLogStageKind::Bus};
    std::size_t unit{0};

    static auto bus() -> McfGurobiLogStage { return McfGurobiLogStage {McfGurobiLogStageKind::Bus, 0}; }

    static auto simple_unit(const std::size_t unit_c) -> McfGurobiLogStage {
        return McfGurobiLogStage {McfGurobiLogStageKind::SimpleUnit, unit_c};
    }
};

enum class McfGurobiRetryKind { None, NoWarmStart };

struct McfGurobiSolveMeta {
    McfGurobiLogStage stage {McfGurobiLogStage::bus()};
    int tier{0};
    int sat_tier_attempt{0};
    int bbox_attempt{0};
    bool warm_start{false};
    McfGurobiRetryKind retry_kind{McfGurobiRetryKind::None};
    int component_id{-1};
    int component_count{0};
    std::String component_summary;
};

struct McfGurobiSolvePaths {
    std::String log_path;
    std::String prm_path;
    int solve_id{0};
};

class McfGurobiLogSink {
public:
    explicit McfGurobiLogSink(std::filesystem::path output_dir);

    static auto prepare_output_dir(const std::String& log_dir) -> void;

    auto begin_solve(const McfGurobiSolveMeta& meta) -> McfGurobiSolvePaths;

    static auto configure_model_log(GRBModel& model, const McfGurobiSolvePaths& paths) -> void;

    static auto write_settings_prm(GRBModel& model, const std::String& prm_path) -> void;

    auto end_solve(
        const McfGurobiSolveMeta& meta,
        const McfGurobiSolvePaths& paths,
        GRBModel& model,
        McfSolutionClass solution_class
    ) -> void;

    auto end_solve_exception(
        const McfGurobiSolveMeta& meta,
        const McfGurobiSolvePaths& paths,
        McfSolutionClass solution_class,
        std::string_view detail
    ) -> void;

    auto write_stub(const McfGurobiSolveMeta& meta, std::string_view reason, std::string_view detail = {}) -> void;

    auto write_skipped(const McfGurobiLogStage& stage, std::string_view reason) -> void;

private:
    auto stage_log_path(const McfGurobiLogStage& stage) const -> std::String;
    auto stage_prm_basename(const McfGurobiLogStage& stage, int solve_id) const -> std::String;
    auto stage_label(const McfGurobiLogStage& stage) const -> std::String;
    auto retry_kind_name(McfGurobiRetryKind kind) const -> std::string_view;
    auto next_solve_id(const McfGurobiLogStage& stage) -> int;
    auto append_lines(const std::String& path, const std::String& content) -> void;
    auto format_timestamp() const -> std::String;

    std::filesystem::path output_dir_;
    std::mutex mutex_;
    std::array<int, 17> solve_ids_ {};
};

} // namespace PR_tool
