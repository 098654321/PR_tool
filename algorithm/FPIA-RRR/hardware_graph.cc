#include "hardware_graph.hh"

#include "hw_map.hh"

#include <hardware/cob/cob.hh>
#include <hardware/cob/cobdirection.hh>
#include <hardware/cob/cobunit.hh>
#include <hardware/track/track.hh>
#include <hardware/track/trackcoord.hh>

#include <algorithm>
#include <array>
#include <format>
#include <set>

namespace PR_tool {

namespace {

using TrackNodeKey = std::tuple<std::size_t, int, int, int, std::size_t>;

auto valid_node(const UnifiedGraph& graph, int node_id) -> bool {
    return node_id >= 0 && node_id < static_cast<int>(graph.nodes.size());
}

auto is_vline_track(const UnifiedGraph& graph, const UnifiedArc& arc) -> bool {
    if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack) {
        return true;
    }
    if (!valid_node(graph, arc.u) || !valid_node(graph, arc.v)) {
        return false;
    }
    const auto ku = graph.nodes[static_cast<std::size_t>(arc.u)].kind;
    const auto kv = graph.nodes[static_cast<std::size_t>(arc.v)].kind;
    return (ku == UnifiedNodeKind::VLine && kv == UnifiedNodeKind::Track)
        || (ku == UnifiedNodeKind::Track && kv == UnifiedNodeKind::VLine);
}

auto append_resource_key(ArcResourceKeys& keys, const ResourceKey& key) -> void {
    for (std::size_t i = 0; i < keys.count; ++i) {
        if (keys.values[i] == key) {
            return;
        }
    }
    if (keys.count < keys.values.size()) {
        keys.values[keys.count++] = key;
    }
}

auto populate_arc_resource_keys(const UnifiedGraph& graph, const UnifiedArc& arc) -> void {
    if (arc.resource_keys_ready || !valid_node(graph, arc.u) || !valid_node(graph, arc.v)) {
        return;
    }
    auto& keys = arc.resource_keys;
    keys.count = 0;
    const auto ku = graph.nodes[static_cast<std::size_t>(arc.u)].kind;
    const auto kv = graph.nodes[static_cast<std::size_t>(arc.v)].kind;
    append_resource_key(keys, node_resource(arc.v));
    if (ku == UnifiedNodeKind::HLine || ku == UnifiedNodeKind::VLine) {
        append_resource_key(keys, node_resource(arc.u));
    }
    if (arc.physical_switch_id >= 0) {
        append_resource_key(keys, switch_resource(arc.physical_switch_id));
    }

    const int bump = ku == UnifiedNodeKind::Bump ? arc.u : (kv == UnifiedNodeKind::Bump ? arc.v : -1);
    const int hline = ku == UnifiedNodeKind::HLine ? arc.u : (kv == UnifiedNodeKind::HLine ? arc.v : -1);
    const int vline = ku == UnifiedNodeKind::VLine ? arc.u : (kv == UnifiedNodeKind::VLine ? arc.v : -1);
    const int track = ku == UnifiedNodeKind::Track ? arc.u : (kv == UnifiedNodeKind::Track ? arc.v : -1);
    if (bump >= 0 && hline >= 0) {
        append_resource_key(keys, matching_endpoint_key(bump, 0));
        append_resource_key(keys, matching_endpoint_key(hline, 1));
        append_resource_key(keys, tob_mux_input_key(bump, hline));
        append_resource_key(keys, tob_mux_output_key(hline, bump));
    }
    if (hline >= 0 && vline >= 0) {
        append_resource_key(keys, matching_endpoint_key(hline, 2));
        append_resource_key(keys, matching_endpoint_key(vline, 3));
        append_resource_key(keys, tob_mux_input_key(hline, vline));
        append_resource_key(keys, tob_mux_output_key(vline, hline));
    }
    if (is_vline_track(graph, arc) && vline >= 0 && track >= 0) {
        append_resource_key(keys, tob_mux_input_key(vline, track));
        append_resource_key(keys, tob_mux_output_key(track, vline));
        if (arc.mode_group_id >= 0) {
            if (arc.is_vline_track_straight) {
                append_resource_key(keys, mode_straight_key(arc.mode_group_id));
            } else if (arc.is_vline_track_swap) {
                append_resource_key(keys, mode_swap_key(arc.mode_group_id));
            }
        }
    }
    arc.resource_keys_ready = true;
}

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

auto find_out_arc(const UnifiedGraph& graph, int u, int v) -> int {
    for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
        if (graph.arcs[static_cast<std::size_t>(arc_id)].v == v) {
            return arc_id;
        }
    }
    return -1;
}

auto cob_cardinal_rank(const UnifiedNode& track, const hardware::COBCoord& cob) -> int {
    if (cob.row < track.track_row) {
        return 0;
    }
    if (cob.col > track.track_col) {
        return 1;
    }
    if (cob.row > track.track_row) {
        return 2;
    }
    if (cob.col < track.track_col) {
        return 3;
    }
    return track.track_dir == 1 ? 2 : 1;
}

auto ordered_track_arcs(
    const UnifiedGraph& graph,
    int node_id,
    hardware::Interposer* interposer
) -> std::Vector<int> {
    const auto& raw = graph.out_arc_ids[static_cast<std::size_t>(node_id)];
    const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
    const auto coord = hardware::TrackCoord {
        node.track_row,
        node.track_col,
        node.track_dir == 0 ? hardware::TrackDirection::Horizontal : hardware::TrackDirection::Vertical,
        node.track_index};
    const auto track_opt = interposer->get_track(coord);
    if (!track_opt.has_value()) {
        return raw;
    }
    auto cobs = (*track_opt)->adjacent_cob_coords();
    std::sort(cobs.begin(), cobs.end(), [&](const auto& lhs, const auto& rhs) {
        const auto& cob_l = std::get<1>(lhs);
        const auto& cob_r = std::get<1>(rhs);
        const auto rank_l = cob_cardinal_rank(node, cob_l);
        const auto rank_r = cob_cardinal_rank(node, cob_r);
        if (rank_l != rank_r) {
            return rank_l < rank_r;
        }
        if (cob_l.row != cob_r.row) {
            return cob_l.row < cob_r.row;
        }
        return cob_l.col < cob_r.col;
    });

    auto ordered = std::Vector<int> {};
    ordered.reserve(raw.size());
    auto seen = std::Set<int> {};
    for (const auto& [from_dir, cob_coord] : cobs) {
        const auto cob_opt = interposer->get_cob(cob_coord);
        if (!cob_opt.has_value()) {
            continue;
        }
        for (auto& connector : (*cob_opt)->adjacent_connectors(from_dir, coord.index, cob_coord)) {
            const auto dest_coord = (*cob_opt)->to_dir_track_coord(
                connector.to_dir(), connector.to_track_index());
            const int dest = get_track_node_id(
                graph,
                map_track(dest_coord.index),
                dest_coord.dir == hardware::TrackDirection::Horizontal ? 0 : 1,
                static_cast<int>(dest_coord.row),
                static_cast<int>(dest_coord.col),
                dest_coord.index);
            const int arc_id = dest < 0 ? -1 : find_out_arc(graph, node_id, dest);
            if (arc_id >= 0 && seen.insert(arc_id).second) {
                ordered.push_back(arc_id);
            }
        }
    }
    for (const int arc_id : raw) {
        if (seen.insert(arc_id).second) {
            ordered.push_back(arc_id);
        }
    }
    return ordered;
}

auto cache_track_adjacency(UnifiedGraph& graph, hardware::Interposer* interposer) -> void {
    graph.ordered_track_out_arc_ids.resize(graph.nodes.size());
    (void)interposer;
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
    populate_arc_resource_keys(g, g.arcs.back());
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

} // namespace

auto build_hardware_graph(hardware::Interposer* interposer, const std::Vector<RoutingNet>& nets) -> UnifiedGraph {
    (void)nets;
    auto graph = UnifiedGraph {};
    build_track_subgraph(graph);
    build_tob_subgraph(graph);
    cache_track_adjacency(graph, interposer);
    return graph;
}

auto cached_arc_resource_keys(const UnifiedGraph& graph, const UnifiedArc& arc) -> const ArcResourceKeys& {
    populate_arc_resource_keys(graph, arc);
    return arc.resource_keys;
}

auto cached_track_out_arc_ids(
    const UnifiedGraph& graph,
    int node_id,
    hardware::Interposer* interposer
) -> const std::Vector<int>& {
    const auto& raw = graph.out_arc_ids[static_cast<std::size_t>(node_id)];
    if (interposer == nullptr || graph.nodes[static_cast<std::size_t>(node_id)].kind != UnifiedNodeKind::Track
        || static_cast<std::size_t>(node_id) >= graph.ordered_track_out_arc_ids.size()) {
        return raw;
    }
    auto& cached = graph.ordered_track_out_arc_ids[static_cast<std::size_t>(node_id)];
    if (cached.empty()) {
        cached = ordered_track_arcs(graph, node_id, interposer);
    }
    return cached;
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
