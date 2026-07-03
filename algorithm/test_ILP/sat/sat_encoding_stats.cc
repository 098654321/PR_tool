#include "sat/sat_encoding_stats.hh"

#include <debug/debug.hh>
#include <format>
#include <numeric>

namespace PR_tool {

namespace {

auto category_index(const SatClauseCategory cat) -> std::size_t {
    return static_cast<std::size_t>(cat);
}

auto category_label(const SatClauseCategory cat) -> const char* {
    switch (cat) {
        case SatClauseCategory::Constant:
            return "constant constraints";
        case SatClauseCategory::VariableRelation:
            return "variable relations";
        case SatClauseCategory::Connectivity:
            return "connectivity constraints";
        case SatClauseCategory::Exclusivity:
            return "exclusivity (global D)";
        case SatClauseCategory::VlineTrackMode:
            return "vline-track mode (M_g)";
        case SatClauseCategory::SyncBusEqualLength:
            return "sync bus equal length";
        case SatClauseCategory::TobSwitchUniqueness:
            return "TOB switch uniqueness (Y)";
    }
    return "unknown";
}

} // namespace

auto SatEncodingStats::add_clauses(const SatClauseCategory cat, const std::size_t count) -> void {
    clause_counts[category_index(cat)] += count;
}

auto SatEncodingStats::known_primary_vars() const -> std::size_t {
    return d_vars + a_vars + mode_vars + switch_vars + alpha_vars;
}

auto SatEncodingStats::finalize_variables(const std::size_t total_session_vars) -> void {
    encoding_aux_vars = total_session_vars >= known_primary_vars()
        ? total_session_vars - known_primary_vars()
        : 0;
}

auto SatEncodingStats::total_clauses() const -> std::size_t {
    return std::accumulate(clause_counts.begin(), clause_counts.end(), std::size_t {0});
}

auto log_sat_encoding_stats(
    const SatEncodingStats& stats,
    const std::size_t total_session_vars,
    const std::size_t total_session_clauses
) -> void {
    debug::info("========== unified SAT encoding stats (-v) ==========");
    debug::info("Variables:");
    debug::info_fmt("  D   (source distance)          : {}", stats.d_vars);
    debug::info_fmt("  A   (TOB arc transition)       : {}", stats.a_vars);
    debug::info_fmt("  M_g (vline-track mode)         : {}", stats.mode_vars);
    debug::info_fmt("  Y   (physical switch aggregate): {}", stats.switch_vars);
    debug::info_fmt("  alpha (pair assumptions)       : {}", stats.alpha_vars);
    debug::info_fmt(
        "  reachable (source,node,d) triples: {}",
        stats.reachable_delay_triples);
    if (stats.reachable_delay_triples > 0) {
        debug::info_fmt(
            "  D var / reachable ratio        : {:.3f}",
            static_cast<double>(stats.d_vars)
                / static_cast<double>(stats.reachable_delay_triples));
    }
    debug::info_fmt("  encoding auxiliary             : {}", stats.encoding_aux_vars);
    debug::info_fmt("  total SAT variables            : {}", total_session_vars);

    debug::info("Clauses by category (CNF):");
    for (std::size_t i = 0; i < kSatClauseCategoryCount; ++i) {
        const auto cat = static_cast<SatClauseCategory>(i);
        debug::info_fmt(
            "  [{}] {:<32}: {}",
            i + 1,
            category_label(cat),
            stats.clause_counts[i]);
    }
    debug::info_fmt("  total CNF clauses              : {}", total_session_clauses);
    debug::info("=====================================================");
}

} // namespace PR_tool
