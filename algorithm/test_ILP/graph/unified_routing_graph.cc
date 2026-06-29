#include "graph/unified_routing_graph.hh"

#include "common/hw_map.hh"
#include "scope/scope_bbox.hh"

#include <hardware/cob/cob.hh>
#include <hardware/cob/cobunit.hh>

#include <format>

namespace PR_tool {

namespace {

using TrackNodeKey = std::tuple<std::size_t, int, int, int, std::size_t>;

auto add_node(UnifiedGraph& g, const UnifiedNode& node) -> int {
    const int id = static_cast<int>(g.nodes.size());
    g.nodes.push_back(node);
    if (node.kind == UnifiedNodeKind::Track) {
        g.track_node_by_key[TrackNodeKey {node.unit, node.track_dir, node.track_row, node.track_col, node.track_index}] =
            id;
        g.track_node_count += 1;
    }
    else {
        g.tob_node_count += 1;
    }
    g.in_arc_ids.emplace_back();
    g.out_arc_ids.emplace_back();
    return id;
}

auto get_track_node_id(
    const UnifiedGraph& g,
    std::size_t unit,
    int dir,
    int row,
    int col,
    std::size_t track
) -> int {
    const auto it = g.track_node_by_key.find(TrackNodeKey {unit, dir, row, col, track});
    if (it == g.track_node_by_key.end()) {
        return -1;
    }
    return it->second;
}

auto add_arc(
    UnifiedGraph& g,
    int u,
    int v,
    bool is_straight = false,
    bool is_swap = false,
    int mode_group_id = -1,
    int physical_switch_id = -1,
    PhysicalSwitchKind physical_switch_kind = PhysicalSwitchKind::None
) -> void {
    if (u < 0 || v < 0) {
        return;
    }
    if (g.directed_arc_set.contains({u, v})) {
        return;
    }
    g.directed_arc_set.insert({u, v});
    const int arc_id = static_cast<int>(g.arcs.size());
    g.arcs.push_back(UnifiedArc {
        u,
        v,
        is_straight,
        is_swap,
        mode_group_id,
        physical_switch_id,
        physical_switch_kind});
    g.out_arc_ids[static_cast<std::size_t>(u)].push_back(arc_id);
    g.in_arc_ids[static_cast<std::size_t>(v)].push_back(arc_id);
}

auto add_physical_connection(
    UnifiedGraph& g,
    int u,
    int v,
    int& next_switch_id,
    PhysicalSwitchKind kind,
    bool is_straight = false,
    bool is_swap = false,
    int mode_group_id = -1
) -> void {
    const int switch_id = next_switch_id++;
    add_arc(g, u, v, is_straight, is_swap, mode_group_id, switch_id, kind);
    add_arc(g, v, u, is_straight, is_swap, mode_group_id, switch_id, kind);
}

auto side_track_pos(hardware::COBDirection side, int cob_r, int cob_c) -> std::tuple<int, int, int> {
    switch (side) {
        case hardware::COBDirection::Down:  return {1, cob_r, cob_c};
        case hardware::COBDirection::Up:    return {1, cob_r + 1, cob_c};
        case hardware::COBDirection::Left:  return {0, cob_r, cob_c};
        case hardware::COBDirection::Right: return {0, cob_r, cob_c + 1};
    }
    return {0, 0, 0};
}

auto add_bump_node(
    UnifiedGraph& g,
    std::size_t tob,
    std::size_t bank,
    std::size_t group,
    std::size_t index
) -> int {
    const auto bump = Bump_coord {tob, bank, group, index};
    UnifiedNode node {};
    node.kind = UnifiedNodeKind::Bump;
    node.bump = bump;
    node.tob = tob;
    node.bank = bank;
    node.group = group;
    node.line_index = index;
    const int id = add_node(g, node);
    g.bump_node_by_key.emplace(bump, id);
    return id;
}

auto add_hline_node(UnifiedGraph& g, std::size_t tob, std::size_t bank, std::size_t group, std::size_t j) -> int {
    const auto key = std::tuple {tob, bank, group, j};
    UnifiedNode node {};
    node.kind = UnifiedNodeKind::HLine;
    node.tob = tob;
    node.bank = bank;
    node.group = group;
    node.line_index = j;
    const int id = add_node(g, node);
    g.hline_node_by_key.emplace(key, id);
    return id;
}

auto add_vline_node(UnifiedGraph& g, std::size_t tob, std::size_t global_v) -> int {
    const auto key = std::tuple {tob, global_v};
    UnifiedNode node {};
    node.kind = UnifiedNodeKind::VLine;
    node.tob = tob;
    node.bank = global_v / 64;
    node.line_index = global_v;
    const int id = add_node(g, node);
    g.vline_node_by_key.emplace(key, id);
    return id;
}

auto build_track_subgraph(UnifiedGraph& g) -> void {
    g.rows = static_cast<int>(hardware::Interposer::COB_ARRAY_HEIGHT);
    g.cols = static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH);

    for (std::size_t unit = 0; unit < 16; ++unit) {
        for (std::size_t inner = 0; inner < 8; ++inner) {
            const auto tr = track_from_unit_inner(unit, inner);
            for (int r = 0; r <= g.rows; ++r) {
                for (int c = 0; c < g.cols; ++c) {
                    UnifiedNode node {};
                    node.kind = UnifiedNodeKind::Track;
                    node.unit = unit;
                    node.track_dir = 1;
                    node.track_row = r;
                    node.track_col = c;
                    node.track_index = tr;
                    add_node(g, node);
                }
            }
            for (int r = 0; r < g.rows; ++r) {
                for (int c = 0; c <= g.cols; ++c) {
                    UnifiedNode node {};
                    node.kind = UnifiedNodeKind::Track;
                    node.unit = unit;
                    node.track_dir = 0;
                    node.track_row = r;
                    node.track_col = c;
                    node.track_index = tr;
                    add_node(g, node);
                }
            }
        }
    }

    constexpr auto dirs = std::array {
        hardware::COBDirection::Left,
        hardware::COBDirection::Right,
        hardware::COBDirection::Up,
        hardware::COBDirection::Down};

    for (int cob_r = 0; cob_r < g.rows; ++cob_r) {
        for (int cob_c = 0; cob_c < g.cols; ++cob_c) {
            for (std::size_t unit = 0; unit < 16; ++unit) {
                for (std::size_t inner = 0; inner < 8; ++inner) {
                    for (const auto from : dirs) {
                        for (const auto to : dirs) {
                            if (from == to) {
                                continue;
                            }
                            const auto mapped = static_cast<std::size_t>(hardware::COBUnit::index_map(from, inner, to));
                            const auto tr_in = track_from_unit_inner(unit, inner);
                            const auto tr_out = track_from_unit_inner(unit, mapped);
                            const auto [in_dir, in_r, in_c] = side_track_pos(from, cob_r, cob_c);
                            const auto [out_dir, out_r, out_c] = side_track_pos(to, cob_r, cob_c);
                            const int u = get_track_node_id(g, unit, in_dir, in_r, in_c, tr_in);
                            const int v = get_track_node_id(g, unit, out_dir, out_r, out_c, tr_out);
                            add_arc(g, u, v);
                            add_arc(g, v, u);
                        }
                    }
                }
            }
        }
    }
}

auto build_tob_subgraph(UnifiedGraph& g) -> void {
    for (std::size_t tob = 0; tob < hardware::Interposer::TOB_SIZE; ++tob) {
        for (std::size_t bank = 0; bank < 2; ++bank) {
            for (std::size_t group = 0; group < 8; ++group) {
                for (std::size_t index = 0; index < 8; ++index) {
                    add_bump_node(g, tob, bank, group, index);
                }
                for (std::size_t j = 0; j < 8; ++j) {
                    add_hline_node(g, tob, bank, group, j);
                }
            }
        }
        for (std::size_t global_v = 0; global_v < 128; ++global_v) {
            add_vline_node(g, tob, global_v);
        }
    }

    int next_switch_id = 0;
    for (std::size_t tob = 0; tob < hardware::Interposer::TOB_SIZE; ++tob) {
        for (std::size_t bank = 0; bank < 2; ++bank) {
            for (std::size_t group = 0; group < 8; ++group) {
                for (std::size_t index = 0; index < 8; ++index) {
                    const int bump_id = g.bump_node_by_key.at(Bump_coord {tob, bank, group, index});
                    for (std::size_t j = 0; j < 8; ++j) {
                        const int hline_id = g.hline_node_by_key.at(std::tuple {tob, bank, group, j});
                        add_physical_connection(
                            g,
                            bump_id,
                            hline_id,
                            next_switch_id,
                            PhysicalSwitchKind::BumpH);
                    }
                }

                for (std::size_t j = 0; j < 8; ++j) {
                    const int hline_id = g.hline_node_by_key.at(std::tuple {tob, bank, group, j});
                    for (std::size_t k = 0; k < 8; ++k) {
                        const std::size_t global_v = bank * 64 + j * 8 + k;
                        const int vline_id = g.vline_node_by_key.at(std::tuple {tob, global_v});
                        add_physical_connection(
                            g,
                            hline_id,
                            vline_id,
                            next_switch_id,
                            PhysicalSwitchKind::HLineVLine);
                    }
                }
            }
        }

        const auto anchor = tob_anchor_cob(tob);
        for (std::size_t k = 0; k < 64; ++k) {
            const int v0_id = g.vline_node_by_key.at(std::tuple {tob, k});
            const int v1_id = g.vline_node_by_key.at(std::tuple {tob, k + 64});
            const int track0_id = get_track_node_id(
                g,
                map_track(k),
                1,
                static_cast<int>(anchor.row),
                static_cast<int>(anchor.col),
                k);
            const int track1_id = get_track_node_id(
                g,
                map_track(k + 64),
                1,
                static_cast<int>(anchor.row),
                static_cast<int>(anchor.col),
                k + 64);
            const int mode_group_id = static_cast<int>(tob * 64 + k);

            add_physical_connection(
                g,
                v0_id,
                track0_id,
                next_switch_id,
                PhysicalSwitchKind::VLineTrack,
                true,
                false,
                mode_group_id);
            add_physical_connection(
                g,
                v1_id,
                track1_id,
                next_switch_id,
                PhysicalSwitchKind::VLineTrack,
                true,
                false,
                mode_group_id);
            add_physical_connection(
                g,
                v0_id,
                track1_id,
                next_switch_id,
                PhysicalSwitchKind::VLineTrack,
                false,
                true,
                mode_group_id);
            add_physical_connection(
                g,
                v1_id,
                track0_id,
                next_switch_id,
                PhysicalSwitchKind::VLineTrack,
                false,
                true,
                mode_group_id);
        }
    }
}

auto cob_in_scope(const UnifiedGraph& graph, int row, int col, const IlpBoundingBox& scope) -> bool {
    return row >= 0 && row < graph.rows && col >= 0 && col < graph.cols
        && row >= scope.row_min && row <= scope.row_max
        && col >= scope.col_min && col <= scope.col_max;
}

} // namespace

auto build_unified_graph(hardware::Interposer* interposer, const std::Vector<RoutingNet>& nets) -> UnifiedGraph {
    (void)interposer;
    (void)nets;
    auto graph = UnifiedGraph {};
    build_track_subgraph(graph);
    build_tob_subgraph(graph);
    graph.directed_arc_set.clear();
    return graph;
}

auto resolve_graph_node(const UnifiedGraph& graph, const GraphNodeRef& ref) -> int {
    if (ref.kind == GraphNodeRef::Kind::Bump) {
        const auto it = graph.bump_node_by_key.find(ref.bump);
        return it == graph.bump_node_by_key.end() ? -1 : it->second;
    }
    if (ref.kind == GraphNodeRef::Kind::Track) {
        const std::size_t unit = map_track(ref.track_index);
        const int dir = ref.track_coord.dir == hardware::TrackDirection::Horizontal ? 0 : 1;
        return get_track_node_id(
            graph,
            unit,
            dir,
            static_cast<int>(ref.track_coord.row),
            static_cast<int>(ref.track_coord.col),
            ref.track_index);
    }
    return -1;
}

auto node_in_scope(const UnifiedGraph& graph, int node_id, const IlpBoundingBox& scope) -> bool {
    if (node_id < 0 || node_id >= static_cast<int>(graph.nodes.size())) {
        return false;
    }
    const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
    if (node.kind == UnifiedNodeKind::Track) {
        if (node.track_dir == 0) {
            return cob_in_scope(graph, node.track_row, node.track_col - 1, scope)
                || cob_in_scope(graph, node.track_row, node.track_col, scope);
        }
        return cob_in_scope(graph, node.track_row - 1, node.track_col, scope)
            || cob_in_scope(graph, node.track_row, node.track_col, scope);
    }
    const auto [tob_row, tob_col] = tob_index_from_linear(node.tob);
    const auto [cob0, cob1] = tob_pair_cob_coords(tob_row, tob_col);
    return cob_in_scope(graph, static_cast<int>(cob0.row), static_cast<int>(cob0.col), scope)
        || cob_in_scope(graph, static_cast<int>(cob1.row), static_cast<int>(cob1.col), scope);
}

auto format_unified_node(const UnifiedGraph& graph, int node_id) -> std::String {
    if (node_id < 0 || node_id >= static_cast<int>(graph.nodes.size())) {
        return std::format("N{}", node_id);
    }
    const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
    switch (node.kind) {
        case UnifiedNodeKind::Track:
            return std::format(
                "T U{} {}({},{}) idx={}",
                node.unit,
                node.track_dir == 0 ? "H" : "V",
                node.track_row,
                node.track_col,
                node.track_index);
        case UnifiedNodeKind::Bump:
            return std::format(
                "B T{} B{} G{} I{}",
                node.bump.TOB,
                node.bump.Bank,
                node.bump.Group,
                node.bump.Index);
        case UnifiedNodeKind::HLine:
            return std::format("H T{} B{} G{} J{}", node.tob, node.bank, node.group, node.line_index);
        case UnifiedNodeKind::VLine:
            return std::format("V T{} B{} V{}", node.tob, node.bank, node.line_index);
    }
    return std::format("N{}", node_id);
}

} // namespace PR_tool
