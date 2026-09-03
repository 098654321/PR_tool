#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace PR_tool {

enum class SatClauseCategory : std::uint8_t {
    Constant,
    VariableRelation,
    Connectivity,
    Exclusivity,
    VlineTrackMode,
    SyncBusEqualLength,
    TobSwitchUniqueness,
    SourceUnitSelection,
};

constexpr std::size_t kSatClauseCategoryCount = 8;

struct SatEncodingStats {
    std::size_t d_vars{0};
    std::size_t a_vars{0};
    std::size_t mode_vars{0};
    std::size_t switch_vars{0};
    std::size_t alpha_vars{0};
    std::size_t unit_selector_vars{0};
    std::size_t dense_d_slots{0};
    std::size_t unit_eligible_d_slots{0};
    std::size_t dense_a_slots{0};
    std::size_t unit_eligible_a_slots{0};
    std::size_t encoding_aux_vars{0};

    std::array<std::size_t, kSatClauseCategoryCount> clause_counts {};

    auto add_clauses(SatClauseCategory cat, std::size_t count) -> void;
    auto finalize_variables(std::size_t total_session_vars) -> void;
    [[nodiscard]] auto total_clauses() const -> std::size_t;
    [[nodiscard]] auto known_primary_vars() const -> std::size_t;
};

auto log_sat_encoding_stats(
    const SatEncodingStats& stats,
    std::size_t total_session_vars,
    std::size_t total_session_clauses
) -> void;

} // namespace PR_tool
