#include "sat/encode_tob_special.hh"

#include "sat/sat_constraint_kits.hh"

#include <algorithm>
#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

constexpr int kGlobalVlineModeGroupCount = 16 * 64;

auto d_var_at(
    const UnifiedSatModel& model,
    std::size_t model_source_index,
    int node,
    int delay
) -> int {
    const auto& source = model.sources[model_source_index];
    const auto& scope = model.scopes[source.scope_index];
    if (node < 0 || static_cast<std::size_t>(node) >= scope.node_offset.size()) {
        return 0;
    }
    const int node_offset = scope.node_offset[static_cast<std::size_t>(node)];
    if (node_offset < 0 || delay < 0 || delay > source.d_max) {
        return 0;
    }
    const int lit = source.d_var[static_cast<std::size_t>(node_offset)]
                        [static_cast<std::size_t>(delay)];
    return lit > 0 ? lit : 0;
}

} // namespace

auto encode_tob_special_constraints(
    CadicalSession& session,
    const UnifiedGraph& graph,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void {
    for (auto& tob_arc : model.tob_arcs) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(tob_arc.arc_global_id)];
        for (int delay = 1; delay <= tob_arc.d_max; ++delay) {
            const int a_lit = tob_arc.a_var[static_cast<std::size_t>(delay)];
            if (a_lit == 0) {
                continue;
            }
            const int d_u = d_var_at(model, tob_arc.model_source_index, arc.u, delay - 1);
            const int d_v = d_var_at(model, tob_arc.model_source_index, arc.v, delay);
            if (d_u <= 0 || d_v <= 0) {
                throw std::logic_error(std::format(
                    "TOB A var has missing D endpoint: source={} arc={} delay={}",
                    tob_arc.model_source_index,
                    tob_arc.arc_global_id,
                    delay));
            }
            add_implies(session, a_lit, d_u, stats, SatClauseCategory::VariableRelation);
            add_implies(session, a_lit, d_v, stats, SatClauseCategory::VariableRelation);
        }
    }

    auto switch_uses = std::map<int, std::Vector<int>> {};
    auto switch_endpoints = std::map<int, std::pair<int, int>> {};
    auto switch_arc_ids = std::map<int, int> {};
    for (const auto& tob_arc : model.tob_arcs) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(tob_arc.arc_global_id)];
        if (arc.physical_switch_id >= 0
            && arc.physical_switch_kind != PhysicalSwitchKind::None) {
            for (int delay = 1; delay <= tob_arc.d_max; ++delay) {
                const int a_lit = tob_arc.a_var[static_cast<std::size_t>(delay)];
                if (a_lit > 0) {
                    switch_uses[arc.physical_switch_id].push_back(a_lit);
                }
            }
            switch_endpoints.emplace(arc.physical_switch_id, std::pair {arc.u, arc.v});
            switch_arc_ids.emplace(arc.physical_switch_id, tob_arc.arc_global_id);
        }
    }

    for (auto& [switch_id, uses] : switch_uses) {
        (void)switch_id;
        std::sort(uses.begin(), uses.end());
        uses.erase(std::unique(uses.begin(), uses.end()), uses.end());
        const int y = session.new_var();
        model.switch_var_by_id.emplace(switch_id, y);
        if (stats != nullptr) {
            ++stats->switch_vars;
        }
        add_or_equiv(session, y, uses, stats, SatClauseCategory::TobSwitchUniqueness);
    }

    for (const auto& [switch_id, y] : model.switch_var_by_id) {
        const auto arc_it = switch_arc_ids.find(switch_id);
        if (arc_it == switch_arc_ids.end()) {
            continue;
        }
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_it->second)];
        if (arc.physical_switch_kind != PhysicalSwitchKind::VLineTrack) {
            continue;
        }
        const auto mode_it = model.mode_var_by_group.find(arc.mode_group_id);
        if (arc.mode_group_id < 0
            || arc.mode_group_id >= kGlobalVlineModeGroupCount
            || mode_it == model.mode_var_by_group.end()) {
            throw std::invalid_argument(std::format(
                "VLineTrack mode group {} is outside the global 0..{} range",
                arc.mode_group_id,
                kGlobalVlineModeGroupCount - 1));
        }
        if (arc.is_vline_track_straight) {
            add_implies(
                session,
                y,
                mode_it->second,
                stats,
                SatClauseCategory::VlineTrackMode);
        }
        if (arc.is_vline_track_swap) {
            add_implies(
                session,
                y,
                -mode_it->second,
                stats,
                SatClauseCategory::VlineTrackMode);
        }
    }

    auto matching = std::map<std::pair<int, int>, std::Vector<int>> {};
    for (const auto& [switch_id, y] : model.switch_var_by_id) {
        const auto endpoint_it = switch_endpoints.find(switch_id);
        if (endpoint_it == switch_endpoints.end()) {
            continue;
        }
        const auto [u, v] = endpoint_it->second;
        const auto u_kind = graph.nodes[static_cast<std::size_t>(u)].kind;
        const auto v_kind = graph.nodes[static_cast<std::size_t>(v)].kind;
        if (u_kind == UnifiedNodeKind::Bump && v_kind == UnifiedNodeKind::HLine) {
            matching[{0, u}].push_back(y);
            matching[{1, v}].push_back(y);
        }
        else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::Bump) {
            matching[{0, v}].push_back(y);
            matching[{1, u}].push_back(y);
        }
        else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::VLine) {
            matching[{2, u}].push_back(y);
            matching[{3, v}].push_back(y);
        }
        else if (u_kind == UnifiedNodeKind::VLine && v_kind == UnifiedNodeKind::HLine) {
            matching[{2, v}].push_back(y);
            matching[{3, u}].push_back(y);
        }
    }
    for (auto& [key, ys] : matching) {
        (void)key;
        std::sort(ys.begin(), ys.end());
        ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
        add_pairwise_at_most_one(
            session,
            ys,
            stats,
            SatClauseCategory::TobSwitchUniqueness);
    }
}

} // namespace PR_tool
