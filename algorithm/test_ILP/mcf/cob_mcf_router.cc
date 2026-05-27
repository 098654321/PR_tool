#include "mcf/cob_mcf_router.hh"

#include "mcf/mcf_graph.hh"
#include "mcf/mcf_hw_map.hh"

#include "circuit/basedie.hh"
#include "debug/debug.hh"
#include "hardware/cob/cobunit.hh"
#include "hardware/track/trackcoord.hh"
#include "highs/Highs.h"
#include "highs/lp_data/HConst.h"
#include "highs/lp_data/HighsLp.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <format>
#include <future>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <tuple>
#include <utility>
#include <vector>

namespace PR_tool {

namespace {

using namespace mcf;

enum class McfClass : int {
    Plain = 0,
    P = 1,
    N = 2
};

using NodeMeta = McfNodeMeta;
using Arc = McfArc;
using NodeKey = McfNodeKey;
using GlobalGraph = McfGlobalGraph;

struct PreparedCommodity {
    std::String label;
    std::String origin_name;
    std::size_t record_index{0};
    std::size_t record_id{0};
    std::Vector<std::size_t> record_indices {};
    std::size_t cob_unit{0};
    std::size_t start_track{0};
    std::size_t end_track{0};
    int src{-1};
    int snk{-1};
    int demand{1};
    McfClass cls{McfClass::Plain};
    bool is_bus{false};
    std::String bus_key;
    std::Vector<IlpReachStep> reach_steps;
    std::Vector<int> bbox_cobs;
};

struct StageSolveResult {
    bool ok{false};
    std::String message;
    double objective{0.0};
    int model_status{static_cast<int>(HighsModelStatus::kNotset)};
    std::map<std::pair<int, int>, int> used_edges;
    std::map<int, int> used_nodes;
    std::array<std::map<std::pair<int, int>, int>, 16> unit_used_edges {};
    std::array<std::map<int, int>, 16> unit_used_nodes {};
    std::Vector<McfPathInfo> paths;
};

struct StageWarmStart {
    std::map<std::size_t, std::Vector<int>> nodes_by_record_id;
};

struct ArcVar {
    int k{0};
    int a{0};
};

struct OVar {
    int k{0};
    int node{0};
};

struct OriginArcVar {
    int h{0};
    int a{0};
};

struct OriginOVar {
    int h{0};
    int node{0};
};

struct McfOriginGroup {
    std::String origin_key;
    std::size_t cob_unit{0};
    std::Vector<int> commodity_local_indices;
    int origin_group_id{0};
    bool is_multi_fanout{false};
};

auto get_peak_rss_mb() -> double {
    rusage usage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return 0.0;
    }
#if defined(__APPLE__)
    return static_cast<double>(usage.ru_maxrss) / (1024.0 * 1024.0);
#else
    return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
}

auto unit_bank(const std::size_t unit) -> std::size_t {
    return unit < 8 ? 0 : 1;
}

auto unit_local(const std::size_t unit) -> std::size_t {
    return unit % 8;
}

auto track_from_unit_inner(const std::size_t unit, const std::size_t inner) -> std::size_t {
    return unit_bank(unit) * 64 + inner * 8 + unit_local(unit);
}

auto node_text(const GlobalGraph& g, const int node) -> std::String {
    if (node < 0 || node >= static_cast<int>(g.nodes.size())) {
        return std::format("N{}", node);
    }
    const auto& meta = g.nodes[static_cast<std::size_t>(node)];
    if (meta.is_virtual) {
        if (meta.virtual_kind == 1) {
            return std::String("V_P");
        }
        if (meta.virtual_kind == 2) {
            return std::String("V_N");
        }
    }
    const auto dir_str = meta.track_dir == 0 ? "H" : "V";
    return std::format("U{} {}({},{}) T{}", meta.unit, dir_str, meta.track_row, meta.track_col, meta.track);
}

auto add_node(GlobalGraph& g, const NodeMeta& node) -> int {
    const int id = static_cast<int>(g.nodes.size());
    g.nodes.push_back(node);
    if (!node.is_virtual) {
        g.node_id_by_key[NodeKey{node.unit, node.track_dir, node.track_row, node.track_col, node.track}] = id;
    }
    return id;
}

auto get_node_id(
    const GlobalGraph& g,
    const std::size_t unit,
    const int dir,
    const int row,
    const int col,
    const std::size_t track
) -> int {
    const auto it = g.node_id_by_key.find(NodeKey{unit, dir, row, col, track});
    if (it == g.node_id_by_key.end()) {
        return -1;
    }
    return it->second;
}

auto add_arc(
    GlobalGraph& g,
    const int u,
    const int v,
    const bool is_virtual,
    const bool is_turn,
    const std::size_t unit,
    const int cob,
    const std::size_t track_in,
    const std::size_t track_out,
    const hardware::COBDirection from_dir = hardware::COBDirection::Left,
    const hardware::COBDirection to_dir = hardware::COBDirection::Left
) -> void {
    if (u < 0 || v < 0 || u >= static_cast<int>(g.nodes.size()) || v >= static_cast<int>(g.nodes.size())) {
        return;
    }
    if (g.directed_arc_set.contains({u, v})) {
        return;
    }
    g.directed_arc_set.insert({u, v});
    g.arcs.push_back(Arc {u, v, is_virtual, is_turn, unit, cob, track_in, track_out, from_dir, to_dir});
}

auto side_track_pos(
    const hardware::COBDirection side,
    const int cob_r,
    const int cob_c
) -> std::tuple<int, int, int> {
    switch (side) {
        case hardware::COBDirection::Down:  return {1, cob_r, cob_c};
        case hardware::COBDirection::Up:    return {1, cob_r + 1, cob_c};
        case hardware::COBDirection::Left:  return {0, cob_r, cob_c};
        case hardware::COBDirection::Right: return {0, cob_r, cob_c + 1};
    }
    return {0, 0, 0};
}

auto is_straight_through(
    const hardware::COBDirection from,
    const hardware::COBDirection to
) -> bool {
    return (from == hardware::COBDirection::Left && to == hardware::COBDirection::Right)
        || (from == hardware::COBDirection::Right && to == hardware::COBDirection::Left)
        || (from == hardware::COBDirection::Up && to == hardware::COBDirection::Down)
        || (from == hardware::COBDirection::Down && to == hardware::COBDirection::Up);
}

auto char_to_cobdir(const char c) -> hardware::COBDirection {
    switch (c) {
        case 'R': return hardware::COBDirection::Right;
        case 'U': return hardware::COBDirection::Up;
        case 'D': return hardware::COBDirection::Down;
        default:  return hardware::COBDirection::Left;
    }
}

auto build_track_graph(const CobMcfGridDims& grid) -> GlobalGraph {
    GlobalGraph g {};
    g.rows = grid.rows;
    g.cols = grid.cols;
    g.num_cob = grid.rows * grid.cols;

    for (std::size_t unit = 0; unit < 16; ++unit) {
        for (std::size_t inner = 0; inner < 8; ++inner) {
            const auto tr = track_from_unit_inner(unit, inner);
            for (int r = 0; r <= g.rows; ++r) {
                for (int c = 0; c < g.cols; ++c) {
                    add_node(g, NodeMeta {false, 0, unit, 1, r, c, tr});
                }
            }
            for (int r = 0; r < g.rows; ++r) {
                for (int c = 0; c <= g.cols; ++c) {
                    add_node(g, NodeMeta {false, 0, unit, 0, r, c, tr});
                }
            }
        }
    }
    g.vp_node = add_node(g, NodeMeta {true, 1, 0, 0, 0, 0, 0});
    g.vn_node = add_node(g, NodeMeta {true, 2, 0, 0, 0, 0, 0});

    constexpr auto dirs = std::array {
        hardware::COBDirection::Left,
        hardware::COBDirection::Right,
        hardware::COBDirection::Up,
        hardware::COBDirection::Down
    };
    for (int cob_r = 0; cob_r < g.rows; ++cob_r) {
        for (int cob_c = 0; cob_c < g.cols; ++cob_c) {
            const int cob_linear = cob_r * g.cols + cob_c;
            for (std::size_t unit = 0; unit < 16; ++unit) {
                for (std::size_t inner = 0; inner < 8; ++inner) {
                    for (const auto from : dirs) {
                        for (const auto to : dirs) {
                            if (from == to) {
                                continue;
                            }
                            const auto mapped = static_cast<std::size_t>(
                                hardware::COBUnit::index_map(from, inner, to));
                            const auto tr_in = track_from_unit_inner(unit, inner);
                            const auto tr_out = track_from_unit_inner(unit, mapped);
                            const auto [in_dir, in_r, in_c] = side_track_pos(from, cob_r, cob_c);
                            const auto [out_dir, out_r, out_c] = side_track_pos(to, cob_r, cob_c);
                            const int u = get_node_id(g, unit, in_dir, in_r, in_c, tr_in);
                            const int v = get_node_id(g, unit, out_dir, out_r, out_c, tr_out);
                            const bool turn = !is_straight_through(from, to);
                            add_arc(g, u, v, false, turn, unit, cob_linear,
                                    tr_in, tr_out, from, to);
                        }
                    }
                }
            }
        }
    }
    return g;
}

auto tob_from_linear(const std::size_t tob_linear) -> hardware::TOBCoord {
    const auto width = static_cast<std::size_t>(hardware::Interposer::TOB_ARRAY_WIDTH);
    return hardware::TOBCoord {
        static_cast<std::i64>(tob_linear / width),
        static_cast<std::i64>(tob_linear % width)};
}

auto tob_anchor_cob(const std::size_t tob_linear) -> hardware::COBCoord {
    const auto tob = tob_from_linear(tob_linear);
    return hardware::COBCoord {
        static_cast<std::i64>(1 + 2 * tob.row),
        static_cast<std::i64>(3 * tob.col)};
}

auto bbox_cob_indices(
    const CobMcfGridDims& grid,
    const hardware::COBCoord& a,
    const hardware::COBCoord& b
) -> std::Vector<int> {
    std::Vector<int> out {};
    const auto r0 = std::max<std::i64>(0, std::min(a.row, b.row));
    const auto r1 = std::min<std::i64>(grid.rows - 1, std::max(a.row, b.row));
    const auto c0 = std::max<std::i64>(0, std::min(a.col, b.col));
    const auto c1 = std::min<std::i64>(grid.cols - 1, std::max(a.col, b.col));
    for (std::i64 r = r0; r <= r1; ++r) {
        for (std::i64 c = c0; c <= c1; ++c) {
            out.push_back(static_cast<int>(r * grid.cols + c));
        }
    }
    return out;
}

auto node_from_bump_track(
    const GlobalGraph& g,
    const std::size_t unit,
    const std::size_t tob_linear,
    const std::size_t track
) -> int {
    const auto tob = tob_from_linear(tob_linear);
    const int r = static_cast<int>(1 + 2 * tob.row);
    const int c = static_cast<int>(3 * tob.col);
    return get_node_id(g, unit, 1, r, c, track);
}

auto node_from_track_coord(
    const GlobalGraph& g,
    const std::size_t unit,
    const hardware::TrackCoord& tc,
    const std::size_t track
) -> int {
    const int dir = tc.dir == hardware::TrackDirection::Horizontal ? 0 : 1;
    return get_node_id(g, unit, dir, static_cast<int>(tc.row), static_cast<int>(tc.col), track);
}

constexpr std::string_view kSyncBusOriginPrefix = "SyncNet in group ";

auto is_sync_bus_origin_key(const std::String& origin_key) -> bool {
    if (!origin_key.starts_with(kSyncBusOriginPrefix)) {
        return false;
    }
    const auto suffix = origin_key.substr(kSyncBusOriginPrefix.size());
    if (suffix.empty()) {
        return false;
    }
    int group_id = 0;
    for (const char ch : suffix) {
        if (!std::isdigit(static_cast<unsigned char>(ch))) {
            return false;
        }
        group_id = group_id * 10 + (ch - '0');
    }
    return group_id > 0;
}

auto is_no_sync_group_origin_key(const std::String& origin_key) -> bool {
    return origin_key.find("in_group_-1") != std::String::npos;
}

auto simple_origin_group_key(
    const PreparedCommodity& c,
    const Net_cost_record& record
) -> std::String {
    if (record.from_track_to_bumps_split) {
        return c.origin_name;
    }
    if (record.type == Net_type::Bnet && is_no_sync_group_origin_key(c.origin_name)) {
        return c.label;
    }
    return c.origin_name;
}

auto prepare_commodities(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfGridDims& grid,
    GlobalGraph& graph
) -> std::Vector<PreparedCommodity> {
    auto out = std::Vector<PreparedCommodity> {};
    if (records.size() != ilp_result.record_track_endpoints.size()) {
        throw std::runtime_error("MCF prepare: record_track_endpoints size mismatch");
    }

    // 遍历每一条net，构造PreparedCommodity 
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto& record = records[i];
        const auto& endpoint = ilp_result.record_track_endpoints[i];
        PreparedCommodity c {};
        c.label = std::format("{}#{}", record.net_name, record.record_id);
        c.origin_name = record.origin_key.empty() ? record.net_name : record.origin_key;
        c.record_index = i;
        c.record_id = record.record_id;
        c.cob_unit = endpoint.cob_unit;
        c.start_track = endpoint.start_track;
        c.end_track = endpoint.end_track;
        c.demand = 1;
        c.bus_key = c.origin_name;
        c.is_bus = is_sync_bus_origin_key(c.origin_name);

        if (!endpoint.has_start_track) {
            debug::warning_fmt("MCF prepare: record {} has no start track", record.net_name);
            continue;
        }
        if (map_track(endpoint.start_track) != c.cob_unit) {
            debug::warning_fmt(
                "MCF prepare: start track {} not in assigned cobunit {} for record {}",
                endpoint.start_track,
                c.cob_unit,
                record.net_name);
            continue;
        }

        hardware::COBCoord bbox_start {};
        hardware::COBCoord bbox_end {};
        if (record.start_bumps.empty()) {
            continue;
        }
        c.src = node_from_bump_track(graph, c.cob_unit, record.start_bumps.front().TOB, endpoint.start_track);
        bbox_start = tob_anchor_cob(record.start_bumps.front().TOB);

        if (record.type == Net_type::PNnet) {
            if (record.power_kind == IlpPowerKind::Pose) {
                c.cls = McfClass::P;
                c.snk = graph.vp_node;
            }
            else {
                c.cls = McfClass::N;
                c.snk = graph.vn_node;
            }

            auto virtual_edges_added = 0;
            for (const auto& [end_track, start_tracks] : record.starttrack_by_endtrack) {
                if (!std::binary_search(start_tracks.begin(), start_tracks.end(), endpoint.start_track)) {
                    continue;
                }
                if (map_track(end_track) != c.cob_unit) {
                    continue;
                }
                if (!record.pn_end_track_coord_by_index.contains(end_track)) {
                    continue;
                }
                const auto& tc = record.pn_end_track_coord_by_index.at(end_track);
                const auto n_end = node_from_track_coord(graph, c.cob_unit, tc, end_track);
                if (n_end < 0) {
                    continue;
                }
                add_arc(
                    graph,
                    n_end,
                    c.snk,
                    true,
                    false,
                    c.cob_unit,
                    -1,
                    end_track,
                    end_track);
                add_arc(
                    graph,
                    c.snk,
                    n_end,
                    true,
                    false,
                    c.cob_unit,
                    -1,
                    end_track,
                    end_track);
                bbox_end = track_to_cob(tc);
                c.end_track = end_track;
                virtual_edges_added += 1;
            }
            if (virtual_edges_added == 0 && endpoint.has_end_track
                && record.pn_end_track_coord_by_index.contains(endpoint.end_track)) {
                const auto& tc = record.pn_end_track_coord_by_index.at(endpoint.end_track);
                const auto n_end = node_from_track_coord(graph, c.cob_unit, tc, endpoint.end_track);
                if (n_end >= 0) {
                    add_arc(graph, n_end, c.snk, true, false, c.cob_unit, -1, endpoint.end_track, endpoint.end_track);
                    add_arc(graph, c.snk, n_end, true, false, c.cob_unit, -1, endpoint.end_track, endpoint.end_track);
                    bbox_end = track_to_cob(tc);
                    c.end_track = endpoint.end_track;
                }
            }
        }
        else if (record.type == Net_type::Tnet) {
            c.cls = McfClass::Plain;
            if (!endpoint.has_end_track) {
                continue;
            }
            c.snk = node_from_track_coord(graph, c.cob_unit, record.mcf_end_track, endpoint.end_track);
            bbox_end = track_to_cob(record.mcf_end_track);
        }
        else {
            c.cls = McfClass::Plain;
            if (record.end_bumps.empty() || !endpoint.has_end_track) {
                continue;
            }
            c.snk = node_from_bump_track(graph, c.cob_unit, record.end_bumps.front().TOB, endpoint.end_track);
            bbox_end = tob_anchor_cob(record.end_bumps.front().TOB);
        }

        if (c.src < 0 || c.snk < 0) {
            debug::warning_fmt(
                "MCF prepare: unresolved endpoint node for record {} (src={}, snk={})",
                record.net_name,
                c.src,
                c.snk);
            continue;
        }

        c.bbox_cobs = bbox_cob_indices(grid, bbox_start, bbox_end);
        if (endpoint.has_end_track) {
            const auto it_end = record.reach_by_end_start.find(endpoint.end_track);
            if (it_end != record.reach_by_end_start.end()) {
                const auto it_start = it_end->second.find(endpoint.start_track);
                if (it_start != it_end->second.end()) {
                    c.reach_steps = it_start->second;
                }
            }
        }
        out.push_back(std::move(c));
    }
    return out;
}

auto arc_usable_for_class(
    const GlobalGraph& graph,
    const Arc& arc,
    const McfClass cls,
    const std::size_t unit,
    const int commodity_snk
) -> bool {
    if (arc.unit != unit) {
        return false;
    }
    if (arc.u == graph.vp_node || arc.v == graph.vp_node) {
        return cls == McfClass::P;
    }
    if (arc.u == graph.vn_node || arc.v == graph.vn_node) {
        return cls == McfClass::N;
    }
    return true;
}

auto extract_path(
    const int src,
    const int snk,
    std::map<std::pair<int, int>, int>& edge_count,
    const int num_nodes
) -> std::Vector<int> {
    auto prev = std::Vector<int>(static_cast<std::size_t>(num_nodes), -1);
    std::queue<int> q;
    q.push(src);
    prev[static_cast<std::size_t>(src)] = src;
    while (!q.empty() && prev[static_cast<std::size_t>(snk)] < 0) {
        const auto u = q.front();
        q.pop();
        for (const auto& [e, cnt] : edge_count) {
            if (cnt <= 0 || e.first != u) {
                continue;
            }
            const auto v = e.second;
            if (prev[static_cast<std::size_t>(v)] >= 0) {
                continue;
            }
            prev[static_cast<std::size_t>(v)] = u;
            q.push(v);
        }
    }
    if (prev[static_cast<std::size_t>(snk)] < 0) {
        return {};
    }
    auto nodes = std::Vector<int> {};
    auto cur = snk;
    while (cur != src) {
        nodes.push_back(cur);
        cur = prev[static_cast<std::size_t>(cur)];
    }
    nodes.push_back(src);
    std::reverse(nodes.begin(), nodes.end());
    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
        auto key = std::make_pair(nodes[i], nodes[i + 1]);
        if (edge_count.contains(key) && edge_count[key] > 0) {
            edge_count[key] -= 1;
        }
    }
    return nodes;
}

auto normalized_edge_key(int u, int v) -> std::pair<int, int> {
    if (u > v) {
        std::swap(u, v);
    }
    return {u, v};
}

auto route_one_mcf_warm_path(
    const GlobalGraph& graph,
    const PreparedCommodity& commodity,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    const std::map<std::pair<int, int>, int>& used_edges,
    const std::map<int, int>& used_nodes
) -> std::Vector<int> {
    auto prev_node = std::vector<int>(graph.nodes.size(), -1);
    auto q = std::queue<int> {};
    q.push(commodity.src);
    prev_node[static_cast<std::size_t>(commodity.src)] = commodity.src;

    while (!q.empty()) {
        const auto node = q.front();
        q.pop();
        if (node == commodity.snk) {
            break;
        }
        for (const auto arc_id : outgoing_arcs[static_cast<std::size_t>(node)]) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!arc_usable_for_class(graph, arc, commodity.cls, commodity.cob_unit, commodity.snk)) {
                continue;
            }
            if (!arc.is_virtual) {
                const auto edge_key = normalized_edge_key(arc.u, arc.v);
                if (const auto it = used_edges.find(edge_key); it != used_edges.end() && it->second >= 1) {
                    continue;
                }
            }
            if (!graph.nodes[static_cast<std::size_t>(arc.v)].is_virtual) {
                if (const auto it = used_nodes.find(arc.v); it != used_nodes.end() && it->second >= 1) {
                    continue;
                }
            }
            if (prev_node[static_cast<std::size_t>(arc.v)] != -1) {
                continue;
            }
            prev_node[static_cast<std::size_t>(arc.v)] = node;
            q.push(arc.v);
        }
    }

    if (prev_node[static_cast<std::size_t>(commodity.snk)] == -1) {
        return {};
    }
    auto path = std::Vector<int> {};
    auto cur = commodity.snk;
    while (cur != commodity.src) {
        path.push_back(cur);
        cur = prev_node[static_cast<std::size_t>(cur)];
    }
    path.push_back(commodity.src);
    std::reverse(path.begin(), path.end());
    return path;
}

auto mark_mcf_warm_path_used(
    const GlobalGraph& graph,
    const std::Vector<int>& path,
    std::map<std::pair<int, int>, int>& used_edges,
    std::map<int, int>& used_nodes
) -> void {
    for (const auto node : path) {
        if (!graph.nodes[static_cast<std::size_t>(node)].is_virtual) {
            used_nodes[node] = 1;
        }
    }
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const auto u = path[i];
        const auto v = path[i + 1];
        for (const auto& arc : graph.arcs) {
            if (arc.u != u || arc.v != v) {
                continue;
            }
            if (!arc.is_virtual) {
                used_edges[normalized_edge_key(u, v)] = 1;
            }
            break;
        }
    }
}

auto route_mcf_stage_warm_start(
    const std::String& stage_name,
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& commodity_ids,
    const std::Vector<std::Vector<int>>& outgoing_arcs,
    std::map<std::pair<int, int>, int>& used_edges,
    std::map<int, int>& used_nodes
) -> StageWarmStart {
    auto warm = StageWarmStart {};
    std::size_t routed = 0;
    std::size_t failed = 0;
    for (const auto cid : commodity_ids) {
        const auto& commodity = commodities[cid];
        auto path = route_one_mcf_warm_path(graph, commodity, outgoing_arcs, used_edges, used_nodes);
        if (path.empty()) {
            ++failed;
            debug::warning_fmt("pre-routing {} warm start failed for commodity {}", stage_name, commodity.label);
            continue;
        }
        mark_mcf_warm_path_used(graph, path, used_edges, used_nodes);
        warm.nodes_by_record_id.emplace(commodity.record_id, std::move(path));
        ++routed;
    }
    debug::info_fmt(
        "pre-routing {} warm start: routed_commodities={} failed_commodities={}",
        stage_name,
        routed,
        failed);
    return warm;
}

auto build_origin_groups(
    const std::Vector<PreparedCommodity>& local_com,
    const std::Vector<Net_cost_record>& records
) -> std::Vector<McfOriginGroup> {
    // 记录 commodity 信息
    auto key_to_indices = std::map<std::pair<std::size_t, std::String>, std::Vector<int>> {};
    for (int k = 0; k < static_cast<int>(local_com.size()); ++k) {
        const auto& c = local_com[static_cast<std::size_t>(k)];
        const auto& rec = records[c.record_index];
        key_to_indices[{c.cob_unit, simple_origin_group_key(c, rec)}].push_back(k);
    }

    auto groups = std::Vector<McfOriginGroup> {};
    int gid = 0;
    for (auto& [key, indices] : key_to_indices) {
        bool is_multi_fanout = false;
        if (indices.size() > 1) {
            // 是目前允许的多扇出net
            for (const auto k : indices) {
                const auto& rec = records[local_com[static_cast<std::size_t>(k)].record_index];
                if (rec.from_track_to_bumps_split || rec.type == Net_type::PNnet) {
                    is_multi_fanout = true;
                    break;
                }
            }
        }
        McfOriginGroup group {};
        group.origin_key = key.second;
        group.cob_unit = key.first;
        group.commodity_local_indices = std::move(indices);
        group.origin_group_id = gid++;
        group.is_multi_fanout = is_multi_fanout;
        groups.push_back(std::move(group));
    }
    return groups;
}

auto log_origin_groups(const std::String& stage_name, const std::Vector<McfOriginGroup>& groups) -> void {
    std::size_t multi_count = 0;
    for (const auto& g : groups) {
        if (g.is_multi_fanout) {
            ++multi_count;
        }
    }
    debug::info_fmt(
        "{} origin groups: total={} multi_fanout={}",
        stage_name,
        groups.size(),
        multi_count);
    for (const auto& g : groups) {
        if (!g.is_multi_fanout) {
            continue;
        }
        auto child_ids = std::String {};
        for (std::size_t i = 0; i < g.commodity_local_indices.size(); ++i) {
            if (i != 0) {
                child_ids += ",";
            }
            child_ids += std::format("{}", g.commodity_local_indices[i]);
        }
        debug::info_fmt(
            "  origin \"{}\" unit={} children=[{}]",
            g.origin_key,
            g.cob_unit,
            child_ids);
    }
}

auto build_local_commodities(
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& commodity_ids
) -> std::Vector<PreparedCommodity> {
    auto local_com = std::Vector<PreparedCommodity> {};
    local_com.reserve(commodity_ids.size());
    for (const auto cid : commodity_ids) {
        local_com.push_back(commodities[cid]);
    }
    return local_com;
}

auto append_paths_from_f_solution(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& local_com,
    const std::Vector<ArcVar>& f_vars,
    const std::Vector<int>& f_values,
    StageSolveResult& out
) -> void {
    const auto K = static_cast<int>(local_com.size());
    const auto N = static_cast<int>(graph.nodes.size());
    auto edge_count_by_k = std::Vector<std::map<std::pair<int, int>, int>>(static_cast<std::size_t>(K));
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto val = f_values[j];
        if (val <= 0) {
            continue;
        }
        const auto k = f_vars[j].k;
        const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[j].a)];
        edge_count_by_k[static_cast<std::size_t>(k)][{arc.u, arc.v}] += val;
    }

    out.paths.reserve(static_cast<std::size_t>(K));
    for (int k = 0; k < K; ++k) {
        const auto& c = local_com[static_cast<std::size_t>(k)];
        McfPathInfo info {};
        info.label = c.label;
        info.origin_name = c.origin_name;
        info.record_id = c.record_id;
        info.src = c.src;
        info.snk = c.snk;
        info.demand = c.demand;
        info.cob_unit = c.cob_unit;
        info.start_track = c.start_track;
        info.end_track = c.end_track;
        if (!c.record_indices.empty()) {
            info.record_indices = c.record_indices;
        }
        else {
            info.record_indices.push_back(c.record_id);
        }

        auto& flow_edges = edge_count_by_k[static_cast<std::size_t>(k)];
        for (int pi = 0; pi < c.demand; ++pi) {
            auto path = extract_path(c.src, c.snk, flow_edges, N);
            if (path.empty()) {
                break;
            }
            info.unit_paths.push_back(path);
            auto track_path = std::Vector<std::size_t> {};
            for (const auto n : path) {
                if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
                    continue;
                }
                const auto track = graph.nodes[static_cast<std::size_t>(n)].track;
                if (track_path.empty() || track_path.back() != track) {
                    track_path.push_back(track);
                }
            }
            if (!track_path.empty()) {
                info.track_paths.push_back(std::move(track_path));
            }
        }
        out.paths.push_back(std::move(info));
    }
}

auto collect_undirected_physical_edge_keys(
    const GlobalGraph& graph,
    const std::optional<std::size_t> unit_filter
) -> std::set<std::pair<int, int>> {
    auto keys = std::set<std::pair<int, int>> {};
    for (const auto& arc : graph.arcs) {
        if (arc.is_virtual) {
            continue;
        }
        if (unit_filter.has_value() && arc.unit != *unit_filter) {
            continue;
        }
        auto u = arc.u;
        auto v = arc.v;
        if (u > v) {
            std::swap(u, v);
        }
        keys.insert({u, v});
    }
    return keys;
}

struct McfGraphScopeStats {
    std::size_t nodes{0};
    std::size_t arcs{0};
    std::size_t physical_arcs{0};
    std::size_t undirected_physical_edges{0};
};

auto count_mcf_graph_scope_stats(
    const GlobalGraph& graph,
    const std::optional<std::size_t> unit_filter
) -> McfGraphScopeStats {
    McfGraphScopeStats stats {};
    for (const auto& node : graph.nodes) {
        if (node.is_virtual) {
            continue;
        }
        if (unit_filter.has_value() && node.unit != *unit_filter) {
            continue;
        }
        ++stats.nodes;
    }
    for (const auto& arc : graph.arcs) {
        if (unit_filter.has_value() && arc.unit != *unit_filter) {
            continue;
        }
        ++stats.arcs;
        if (!arc.is_virtual) {
            ++stats.physical_arcs;
        }
    }
    stats.undirected_physical_edges = collect_undirected_physical_edge_keys(graph, unit_filter).size();
    return stats;
}

auto log_mcf_model_graph(
    const std::String& stage_name,
    const GlobalGraph& graph,
    const int num_commodities,
    const std::optional<std::size_t> unit_filter = std::nullopt
) -> void {
    const auto stats = count_mcf_graph_scope_stats(graph, unit_filter);
    if (unit_filter.has_value()) {
        debug::info_fmt(
            "{} model graph: scope=COBUnit{} nodes={} arcs={} (physical_arcs={} undirected_edges={}) "
            "commodities={} [global: nodes={} arcs={}]",
            stage_name,
            *unit_filter,
            stats.nodes,
            stats.arcs,
            stats.physical_arcs,
            stats.undirected_physical_edges,
            num_commodities,
            graph.nodes.size(),
            graph.arcs.size());
    }
    else {
        debug::info_fmt(
            "{} model graph: scope=global nodes={} arcs={} (physical_arcs={} undirected_edges={}) commodities={}",
            stage_name,
            stats.nodes,
            stats.arcs,
            stats.physical_arcs,
            stats.undirected_physical_edges,
            num_commodities);
    }
}

auto log_mcf_constraint_rows(
    const std::String& stage_name,
    const std::map<std::String, int>& rows_by_constraint
) -> void {
    int total = 0;
    for (const auto& [name, count] : rows_by_constraint) {
        debug::info_fmt("{} constraint \"{}\": {} HiGHS row(s)", stage_name, name, count);
        total += count;
    }
    debug::info_fmt("{} constraint rows total: {}", stage_name, total);
}

auto solve_bus_mcf(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::Vector<std::size_t>& bus_ids,
    const StageWarmStart* warm_start
) -> StageSolveResult {
    constexpr auto stage_name = "BusMCF";
    StageSolveResult out {};
    if (bus_ids.empty()) {
        out.ok = true;
        out.message = "empty stage";
        out.model_status = static_cast<int>(HighsModelStatus::kOptimal);
        return out;
    }

    const auto K = static_cast<int>(bus_ids.size());
    const auto A = static_cast<int>(graph.arcs.size());
    const auto local_com = build_local_commodities(commodities, bus_ids);

    // BusMCF §1: f^{c,n}_{ij} variables
    auto f_vars = std::Vector<ArcVar> {};
    auto f_by_k = std::Vector<std::Vector<int>>(static_cast<std::size_t>(K));
    f_vars.reserve(static_cast<std::size_t>(K * A / 8 + 1));
    for (int k = 0; k < K; ++k) {
        for (int a = 0; a < A; ++a) {
            if (!arc_usable_for_class(
                    graph,
                    graph.arcs[static_cast<std::size_t>(a)],
                    local_com[static_cast<std::size_t>(k)].cls,
                    local_com[static_cast<std::size_t>(k)].cob_unit,
                    local_com[static_cast<std::size_t>(k)].snk)) {
                continue;
            }
            const auto var_id = static_cast<int>(f_vars.size());
            f_vars.push_back(ArcVar {k, a});
            f_by_k[static_cast<std::size_t>(k)].push_back(var_id);
        }
    }
    if (f_vars.empty()) {
        out.ok = false;
        out.message = std::format("{}: no feasible arc-variable pairs", stage_name);
        return out;
    }

    log_mcf_model_graph(stage_name, graph, K);

    auto row_lo = std::vector<double> {};
    auto row_up = std::vector<double> {};
    auto add_eq = [&](const double rhs) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(rhs);
        row_up.push_back(rhs);
        return id;
    };
    auto add_le = [&](const double rhs) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(-kHighsInf);
        row_up.push_back(rhs);
        return id;
    };

    // BusMCF §3: node flow conservation
    auto flow_row = std::map<std::pair<int, int>, int> {};
    const auto ensure_flow_row = [&](const int k, const int n) -> int {
        const auto key = std::make_pair(k, n);
        if (flow_row.contains(key)) {
            return flow_row.at(key);
        }
        const auto row = add_eq(0.0);
        flow_row[key] = row;
        return row;
    };

    // BusMCF §2: edge capacity Σ_n (f_ij + f_ji) <= capacity
    auto edge_row = std::map<std::pair<int, int>, int> {};
    for (const auto& key : collect_undirected_physical_edge_keys(graph, std::nullopt)) {
        edge_row[key] = add_le(1.0);
    }

    auto f_entries = std::Vector<std::Vector<std::pair<int, double>>>(f_vars.size());
    auto incident_f = std::map<std::pair<int, int>, std::Vector<int>> {};
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto k = f_vars[j].k;
        const auto a = f_vars[j].a;
        const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
        f_entries[j].push_back({ensure_flow_row(k, arc.u), 1.0});
        f_entries[j].push_back({ensure_flow_row(k, arc.v), -1.0});
        if (!arc.is_virtual) {
            auto u = arc.u;
            auto v = arc.v;
            if (u > v) {
                std::swap(u, v);
            }
            f_entries[j].push_back({edge_row.at({u, v}), 1.0});
        }
        if (!graph.nodes[static_cast<std::size_t>(arc.u)].is_virtual) {
            incident_f[{k, arc.u}].push_back(static_cast<int>(j));
        }
        if (!graph.nodes[static_cast<std::size_t>(arc.v)].is_virtual) {
            incident_f[{k, arc.v}].push_back(static_cast<int>(j));
        }
    }

    for (int k = 0; k < K; ++k) {
        const auto s = local_com[static_cast<std::size_t>(k)].src;
        const auto t = local_com[static_cast<std::size_t>(k)].snk;
        const auto d = local_com[static_cast<std::size_t>(k)].demand;
        const auto rs = ensure_flow_row(k, s);
        const auto rt = ensure_flow_row(k, t);
        row_lo[static_cast<std::size_t>(rs)] = static_cast<double>(d);
        row_up[static_cast<std::size_t>(rs)] = static_cast<double>(d);
        row_lo[static_cast<std::size_t>(rt)] = static_cast<double>(-d);
        row_up[static_cast<std::size_t>(rt)] = static_cast<double>(-d);
    }

    // BusMCF §5: f <= o, Σ_n o_i <= 1
    auto o_entries = std::Vector<std::Vector<std::pair<int, double>>> {};
    auto o_vars = std::Vector<OVar> {};
    auto node_row = std::map<int, int> {};
    auto physical_nodes_in_use = std::set<int> {};
    for (const auto& [kn, vars] : incident_f) {
        (void)vars;
        physical_nodes_in_use.insert(kn.second);
    }
    for (const auto n : physical_nodes_in_use) {
        if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
            continue;
        }
        node_row[n] = add_le(1.0);
    }
    for (const auto& [kn, vars] : incident_f) {
        const auto k = kn.first;
        const auto n = kn.second;
        if (vars.empty()) {
            continue;
        }
        const auto row_link = add_le(0.0);
        for (const auto j : vars) {
            f_entries[static_cast<std::size_t>(j)].push_back({row_link, 1.0});
        }
        o_vars.push_back(OVar {k, n});
        auto col = std::Vector<std::pair<int, double>> {};
        col.push_back({row_link, -2.0});
        col.push_back({node_row.at(n), 1.0});
        o_entries.push_back(std::move(col));
    }

    // BusMCF §6: sync equal length (第三版)
    // total_flow_n = Σ_{(i,j)∈E^c} f^{c,n}_{ij},  ∀n∈Bus ∧ c = n 所在 COBUnit
    // E^c 由 arc_usable_for_class(..., cob_unit) 限定；非虚拟弧求和即 total_flow_n
    // total_flow_n = total_flow_m,  ∀n,m ∈ the_same_Bus (bus_key)
    int bus_equal_length_rows = 0;
    auto by_bus = std::map<std::String, std::Vector<int>> {};
    for (int k = 0; k < K; ++k) {
        if (!local_com[static_cast<std::size_t>(k)].is_bus) {
            continue;
        }
        by_bus[local_com[static_cast<std::size_t>(k)].bus_key].push_back(k);
    }
    for (const auto& [key, group] : by_bus) {
        (void)key;
        if (group.size() <= 1) {
            continue;
        }
        const auto ref = group.front();
        for (std::size_t gi = 1; gi < group.size(); ++gi) {
            const auto row = add_eq(0.0);
            ++bus_equal_length_rows;
            const auto cur = group[gi];
            for (const auto j : f_by_k[static_cast<std::size_t>(cur)]) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[static_cast<std::size_t>(j)].a)];
                if (arc.is_virtual) {
                    continue;
                }
                f_entries[static_cast<std::size_t>(j)].push_back({row, 1.0});
            }
            for (const auto j : f_by_k[static_cast<std::size_t>(ref)]) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[static_cast<std::size_t>(j)].a)];
                if (arc.is_virtual) {
                    continue;
                }
                f_entries[static_cast<std::size_t>(j)].push_back({row, -1.0});
            }
        }
    }

    const auto num_f = static_cast<int>(f_vars.size());
    const auto num_o = static_cast<int>(o_vars.size());
    const auto num_col = num_f + num_o;
    const auto num_row = static_cast<int>(row_lo.size());

    log_mcf_constraint_rows(
        stage_name,
        {
            {"flow_conservation", static_cast<int>(flow_row.size())},
            {"edge_capacity", static_cast<int>(edge_row.size())},
            {"f_le_o_link", static_cast<int>(o_vars.size())},
            {"node_capacity", static_cast<int>(node_row.size())},
            {"bus_equal_length", bus_equal_length_rows},
        });
    debug::info_fmt(
        "{} variables: f={} o={} cols={} rows={}",
        stage_name,
        num_f,
        num_o,
        num_col,
        num_row);

    // BusMCF objective: min Σ f on non-virtual arcs
    auto col_cost = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_lo = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_up = std::vector<double>(static_cast<std::size_t>(num_col), 1.0);
    auto a_start = std::vector<HighsInt>(static_cast<std::size_t>(num_col) + 1, 0);
    auto a_index = std::vector<HighsInt> {};
    auto a_value = std::vector<double> {};
    a_index.reserve(static_cast<std::size_t>(num_col * 8));
    a_value.reserve(static_cast<std::size_t>(num_col * 8));

    for (int j = 0; j < num_f; ++j) {
        a_start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(a_index.size());
        const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[static_cast<std::size_t>(j)].a)];
        col_cost[static_cast<std::size_t>(j)] = arc.is_virtual ? 0.0 : 1.0;
        for (const auto& [r, v] : f_entries[static_cast<std::size_t>(j)]) {
            a_index.push_back(static_cast<HighsInt>(r));
            a_value.push_back(v);
        }
    }
    for (int j = 0; j < num_o; ++j) {
        const auto col = num_f + j;
        a_start[static_cast<std::size_t>(col)] = static_cast<HighsInt>(a_index.size());
        for (const auto& [r, v] : o_entries[static_cast<std::size_t>(j)]) {
            a_index.push_back(static_cast<HighsInt>(r));
            a_value.push_back(v);
        }
    }
    a_start[static_cast<std::size_t>(num_col)] = static_cast<HighsInt>(a_index.size());

    HighsLp lp {};
    lp.num_col_ = static_cast<HighsInt>(num_col);
    lp.num_row_ = static_cast<HighsInt>(num_row);
    lp.sense_ = ObjSense::kMinimize;
    lp.offset_ = 0.0;
    lp.col_cost_ = std::move(col_cost);
    lp.col_lower_ = std::move(col_lo);
    lp.col_upper_ = std::move(col_up);
    lp.row_lower_ = std::move(row_lo);
    lp.row_upper_ = std::move(row_up);
    lp.integrality_.assign(static_cast<std::size_t>(num_col), HighsVarType::kInteger);
    lp.model_name_ = stage_name;
    lp.a_matrix_.format_ = MatrixFormat::kColwise;
    lp.a_matrix_.num_col_ = lp.num_col_;
    lp.a_matrix_.num_row_ = lp.num_row_;
    lp.a_matrix_.start_ = std::move(a_start);
    lp.a_matrix_.index_ = std::move(a_index);
    lp.a_matrix_.value_ = std::move(a_value);
    lp.setMatrixDimensions();

    Highs highs {};
    highs.setOptionValue("output_flag", false);
    highs.setOptionValue("presolve", "on");
    if (highs.passModel(std::move(lp)) != HighsStatus::kOk) {
        out.ok = false;
        out.message = std::format("{}: passModel failed", stage_name);
        return out;
    }

    if (warm_start != nullptr && !warm_start->nodes_by_record_id.empty()) {
        auto warm_values_by_col = std::map<HighsInt, double> {};
        auto o_col_by_k_node = std::map<std::pair<int, int>, int> {};
        for (std::size_t oi = 0; oi < o_vars.size(); ++oi) {
            const auto& ov = o_vars[oi];
            o_col_by_k_node[{ov.k, ov.node}] = num_f + static_cast<int>(oi);
        }

        std::size_t matched_paths = 0;
        for (int k = 0; k < K; ++k) {
            const auto& commodity = local_com[static_cast<std::size_t>(k)];
            const auto path_it = warm_start->nodes_by_record_id.find(commodity.record_id);
            if (path_it == warm_start->nodes_by_record_id.end()) {
                continue;
            }
            const auto& path = path_it->second;
            if (path.size() < 2) {
                continue;
            }
            ++matched_paths;
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                const auto u = path[i];
                const auto v = path[i + 1];
                for (const auto f_col : f_by_k[static_cast<std::size_t>(k)]) {
                    const auto arc_id = f_vars[static_cast<std::size_t>(f_col)].a;
                    const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                    if (arc.u == u && arc.v == v) {
                        warm_values_by_col[static_cast<HighsInt>(f_col)] = 1.0;
                        break;
                    }
                }
            }
            for (const auto node : path) {
                if (graph.nodes[static_cast<std::size_t>(node)].is_virtual) {
                    continue;
                }
                const auto it = o_col_by_k_node.find({k, node});
                if (it != o_col_by_k_node.end()) {
                    warm_values_by_col[static_cast<HighsInt>(it->second)] = 1.0;
                }
            }
        }

        if (!warm_values_by_col.empty()) {
            auto warm_cols = std::vector<HighsInt> {};
            auto warm_values = std::vector<double> {};
            warm_cols.reserve(warm_values_by_col.size());
            warm_values.reserve(warm_values_by_col.size());
            for (const auto& [col, value] : warm_values_by_col) {
                warm_cols.push_back(col);
                warm_values.push_back(value);
            }
            (void)highs.setOptionValue("mip_max_start_nodes", static_cast<HighsInt>(0));
            const auto start_st = highs.setSolution(
                static_cast<HighsInt>(warm_cols.size()),
                warm_cols.data(),
                warm_values.data());
            if (start_st != HighsStatus::kOk) {
                debug::warning_fmt(
                    "{} warm start rejected by HiGHS (status={}, matched_paths={}, values={})",
                    stage_name,
                    static_cast<int>(start_st),
                    matched_paths,
                    warm_cols.size());
            }
            else {
                debug::info_fmt(
                    "{} warm start accepted by HiGHS: matched_paths={}, values={}",
                    stage_name,
                    matched_paths,
                    warm_cols.size());
            }
        }
    }

    if (highs.run() != HighsStatus::kOk) {
        out.ok = false;
        out.message = std::format("{}: solver run failed", stage_name);
        out.model_status = static_cast<int>(highs.getModelStatus());
        return out;
    }
    const auto status = highs.getModelStatus();
    out.model_status = static_cast<int>(status);
    if (status != HighsModelStatus::kOptimal) {
        if (warm_start != nullptr) {
            debug::warning_fmt(
                "{} warm start led to non-optimal status ({}); retrying without warm start",
                stage_name,
                static_cast<int>(status));
            return solve_bus_mcf(graph, commodities, bus_ids, nullptr);
        }
        out.ok = false;
        out.message = std::format("{}: model not optimal ({})", stage_name, static_cast<int>(status));
        return out;
    }

    out.ok = true;
    out.message = "ok";
    out.objective = highs.getObjectiveValue();

    const auto sol = highs.getSolution();
    auto f_values = std::Vector<int>(f_vars.size(), 0);
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        f_values[j] = static_cast<int>(std::lround(sol.col_value[j]));
        if (f_values[j] <= 0) {
            continue;
        }
        const auto k = f_vars[j].k;
        const auto unit = local_com[static_cast<std::size_t>(k)].cob_unit;
        const auto& arc = graph.arcs[static_cast<std::size_t>(f_vars[j].a)];
        if (!arc.is_virtual) {
            auto u = arc.u;
            auto v = arc.v;
            if (u > v) {
                std::swap(u, v);
            }
            out.used_edges[{u, v}] = 1;
            out.unit_used_edges[unit][{u, v}] = 1;
        }
    }
    for (std::size_t j = 0; j < o_vars.size(); ++j) {
        const auto col = static_cast<std::size_t>(num_f + static_cast<int>(j));
        const auto val = static_cast<int>(std::lround(sol.col_value[col]));
        if (val > 0) {
            const auto k = o_vars[j].k;
            const auto unit = local_com[static_cast<std::size_t>(k)].cob_unit;
            out.used_nodes[o_vars[j].node] = 1;
            out.unit_used_nodes[unit][o_vars[j].node] = 1;
        }
    }

    append_paths_from_f_solution(graph, local_com, f_vars, f_values, out);
    return out;
}

auto solve_simple_mcf_unit(
    const GlobalGraph& graph,
    const std::Vector<PreparedCommodity>& commodities,
    const std::size_t unit_c,
    const std::Vector<std::size_t>& simple_ids_for_unit,
    const std::Vector<Net_cost_record>& records,
    const std::map<std::pair<int, int>, int>& edge_capacity_override,
    const std::map<int, int>& node_capacity_override,
    const bool enable_mcf_obj,
    const StageWarmStart* warm_start
) -> StageSolveResult {
    const auto stage_name = std::format("SimpleMCF_unit{}", unit_c);
    StageSolveResult out {};
    if (simple_ids_for_unit.empty()) {
        out.ok = true;
        out.message = "empty stage";
        out.model_status = static_cast<int>(HighsModelStatus::kOptimal);
        return out;
    }

    const auto K = static_cast<int>(simple_ids_for_unit.size());
    const auto A = static_cast<int>(graph.arcs.size());
    const auto local_com = build_local_commodities(commodities, simple_ids_for_unit);

    auto origin_groups = build_origin_groups(local_com, records);
    log_origin_groups(stage_name, origin_groups);
    auto commodity_origin_h = std::Vector<int>(static_cast<std::size_t>(K), -1);
    for (const auto& group : origin_groups) {
        for (const auto k : group.commodity_local_indices) {
            commodity_origin_h[static_cast<std::size_t>(k)] = group.origin_group_id;
        }
    }

    // SimpleMCF §1: f^{c,n}_{ij} variables
    auto f_vars = std::Vector<ArcVar> {};
    auto f_by_k = std::Vector<std::Vector<int>>(static_cast<std::size_t>(K));
    f_vars.reserve(static_cast<std::size_t>(K * A / 8 + 1));
    for (int k = 0; k < K; ++k) {
        for (int a = 0; a < A; ++a) {
            if (!arc_usable_for_class(
                    graph,
                    graph.arcs[static_cast<std::size_t>(a)],
                    local_com[static_cast<std::size_t>(k)].cls,
                    local_com[static_cast<std::size_t>(k)].cob_unit,
                    local_com[static_cast<std::size_t>(k)].snk)) {
                continue;
            }
            const auto var_id = static_cast<int>(f_vars.size());
            f_vars.push_back(ArcVar {k, a});
            f_by_k[static_cast<std::size_t>(k)].push_back(var_id);
        }
    }
    if (f_vars.empty()) {
        out.ok = false;
        out.message = std::format("{}: no feasible arc-variable pairs", stage_name);
        return out;
    }

    log_mcf_model_graph(stage_name, graph, K, unit_c);

    auto row_lo = std::vector<double> {};
    auto row_up = std::vector<double> {};
    auto add_eq = [&](const double rhs) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(rhs);
        row_up.push_back(rhs);
        return id;
    };
    auto add_le = [&](const double rhs) -> int {
        const auto id = static_cast<int>(row_lo.size());
        row_lo.push_back(-kHighsInf);
        row_up.push_back(rhs);
        return id;
    };

    // SimpleMCF §3: node flow conservation on f
    auto flow_row = std::map<std::pair<int, int>, int> {};
    const auto ensure_flow_row = [&](const int k, const int n) -> int {
        const auto key = std::make_pair(k, n);
        if (flow_row.contains(key)) {
            return flow_row.at(key);
        }
        const auto row = add_eq(0.0);
        flow_row[key] = row;
        return row;
    };

    // SimpleMCF §5: Σ_H (x_ij + x_ji) <= capacity - used^{Bus,c} on E^c only
    auto edge_row = std::map<std::pair<int, int>, int> {};
    for (const auto& key : collect_undirected_physical_edge_keys(graph, unit_c)) {
        auto cap = 1;
        if (edge_capacity_override.contains(key)) {
            cap = edge_capacity_override.at(key);
        }
        edge_row[key] = add_le(static_cast<double>(cap));
    }

    auto f_entries = std::Vector<std::Vector<std::pair<int, double>>>(f_vars.size());
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto k = f_vars[j].k;
        const auto a = f_vars[j].a;
        const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
        f_entries[j].push_back({ensure_flow_row(k, arc.u), 1.0});
        f_entries[j].push_back({ensure_flow_row(k, arc.v), -1.0});
    }

    for (int k = 0; k < K; ++k) {
        const auto s = local_com[static_cast<std::size_t>(k)].src;
        const auto t = local_com[static_cast<std::size_t>(k)].snk;
        const auto d = local_com[static_cast<std::size_t>(k)].demand;
        const auto rs = ensure_flow_row(k, s);
        const auto rt = ensure_flow_row(k, t);
        row_lo[static_cast<std::size_t>(rs)] = static_cast<double>(d);
        row_up[static_cast<std::size_t>(rs)] = static_cast<double>(d);
        row_lo[static_cast<std::size_t>(rt)] = static_cast<double>(-d);
        row_up[static_cast<std::size_t>(rt)] = static_cast<double>(-d);
    }

    // SimpleMCF §1-2: x^{c,H}_{ij} for every Origin H; f <= x
    auto origin_x_vars = std::Vector<OriginArcVar> {};
    auto origin_x_entries = std::Vector<std::Vector<std::pair<int, double>>> {};
    auto origin_x_by_ha = std::map<std::pair<int, int>, int> {};
    auto incident_origin_x = std::map<std::pair<int, int>, std::Vector<int>> {};
    for (const auto& group : origin_groups) {
        const int h = group.origin_group_id;
        auto arc_ids = std::set<int> {};
        for (const auto k : group.commodity_local_indices) {
            for (const auto f_var : f_by_k[static_cast<std::size_t>(k)]) {
                arc_ids.insert(f_vars[static_cast<std::size_t>(f_var)].a);
            }
        }
        for (const auto a : arc_ids) {
            const auto key = std::make_pair(h, a);
            if (origin_x_by_ha.contains(key)) {
                continue;
            }
            const auto var_id = static_cast<int>(origin_x_vars.size());
            origin_x_vars.push_back(OriginArcVar {h, a});
            origin_x_by_ha[key] = var_id;
            origin_x_entries.emplace_back();

            const auto& arc = graph.arcs[static_cast<std::size_t>(a)];
            if (!arc.is_virtual) {
                auto u = arc.u;
                auto v = arc.v;
                if (u > v) {
                    std::swap(u, v);
                }
                origin_x_entries[static_cast<std::size_t>(var_id)].push_back({edge_row.at({u, v}), 1.0});
            }
            if (!graph.nodes[static_cast<std::size_t>(arc.u)].is_virtual) {
                incident_origin_x[{h, arc.u}].push_back(var_id);
            }
            if (!graph.nodes[static_cast<std::size_t>(arc.v)].is_virtual) {
                incident_origin_x[{h, arc.v}].push_back(var_id);
            }
        }
    }

    int f_le_x_rows = 0;
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        const auto k = f_vars[j].k;
        const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
        const auto a = f_vars[j].a;
        const auto it = origin_x_by_ha.find({h, a});
        if (it == origin_x_by_ha.end()) {
            continue;
        }
        const auto row = add_le(0.0);
        ++f_le_x_rows;
        f_entries[j].push_back({row, 1.0});
        origin_x_entries[static_cast<std::size_t>(it->second)].push_back({row, -1.0});
    }

    // SimpleMCF §5: x <= o^H, Σ_H o^H_i <= 1 - used^{Bus,c}_i
    auto origin_o_entries = std::Vector<std::Vector<std::pair<int, double>>> {};
    auto origin_o_vars = std::Vector<OriginOVar> {};
    auto node_row = std::map<int, int> {};
    auto physical_nodes_in_use = std::set<int> {};
    for (const auto& [hn, vars] : incident_origin_x) {
        (void)vars;
        (void)hn;
        physical_nodes_in_use.insert(hn.second);
    }
    for (const auto n : physical_nodes_in_use) {
        if (graph.nodes[static_cast<std::size_t>(n)].is_virtual) {
            continue;
        }
        auto cap = 1;
        if (node_capacity_override.contains(n)) {
            cap = node_capacity_override.at(n);
        }
        node_row[n] = add_le(static_cast<double>(cap));
    }
    for (const auto& [hn, vars] : incident_origin_x) {
        const auto h = hn.first;
        const auto n = hn.second;
        if (vars.empty()) {
            continue;
        }
        const auto row_link = add_le(0.0);
        for (const auto j : vars) {
            origin_x_entries[static_cast<std::size_t>(j)].push_back({row_link, 1.0});
        }
        origin_o_vars.push_back(OriginOVar {h, n});
        auto col = std::Vector<std::pair<int, double>> {};
        col.push_back({row_link, -2.0});
        col.push_back({node_row.at(n), 1.0});
        origin_o_entries.push_back(std::move(col));
    }

    const auto num_f = static_cast<int>(f_vars.size());
    const auto num_origin_x = static_cast<int>(origin_x_vars.size());
    const auto num_origin_o = static_cast<int>(origin_o_vars.size());
    const auto num_col = num_f + num_origin_x + num_origin_o;
    const auto num_row = static_cast<int>(row_lo.size());

    log_mcf_constraint_rows(
        stage_name,
        {
            {"flow_conservation", static_cast<int>(flow_row.size())},
            {"edge_capacity", static_cast<int>(edge_row.size())},
            {"f_le_x", f_le_x_rows},
            {"x_le_o_link", static_cast<int>(origin_o_vars.size())},
            {"node_capacity", static_cast<int>(node_row.size())},
        });
    debug::info_fmt(
        "{} variables: f={} x={} o={} origin_groups={} cols={} rows={}",
        stage_name,
        num_f,
        num_origin_x,
        num_origin_o,
        origin_groups.size(),
        num_col,
        num_row);
    debug::info_fmt(
        "{} objective min_sum_x: {}",
        stage_name,
        enable_mcf_obj ? "enabled (--enable-mcf-obj)" : "disabled (feasibility only)");

    // SimpleMCF objective: min Σ x on non-virtual arcs (optional via --enable-mcf-obj)
    auto col_cost = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_lo = std::vector<double>(static_cast<std::size_t>(num_col), 0.0);
    auto col_up = std::vector<double>(static_cast<std::size_t>(num_col), 1.0);
    auto a_start = std::vector<HighsInt>(static_cast<std::size_t>(num_col) + 1, 0);
    auto a_index = std::vector<HighsInt> {};
    auto a_value = std::vector<double> {};
    a_index.reserve(static_cast<std::size_t>(num_col * 8));
    a_value.reserve(static_cast<std::size_t>(num_col * 8));

    for (int j = 0; j < num_f; ++j) {
        a_start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(a_index.size());
        for (const auto& [r, v] : f_entries[static_cast<std::size_t>(j)]) {
            a_index.push_back(static_cast<HighsInt>(r));
            a_value.push_back(v);
        }
    }
    for (int j = 0; j < num_origin_x; ++j) {
        const auto col = num_f + j;
        a_start[static_cast<std::size_t>(col)] = static_cast<HighsInt>(a_index.size());
        const auto& arc = graph.arcs[static_cast<std::size_t>(origin_x_vars[static_cast<std::size_t>(j)].a)];
        col_cost[static_cast<std::size_t>(col)] = (enable_mcf_obj && !arc.is_virtual) ? 1.0 : 0.0;
        for (const auto& [r, v] : origin_x_entries[static_cast<std::size_t>(j)]) {
            a_index.push_back(static_cast<HighsInt>(r));
            a_value.push_back(v);
        }
    }
    for (int j = 0; j < num_origin_o; ++j) {
        const auto col = num_f + num_origin_x + j;
        a_start[static_cast<std::size_t>(col)] = static_cast<HighsInt>(a_index.size());
        for (const auto& [r, v] : origin_o_entries[static_cast<std::size_t>(j)]) {
            a_index.push_back(static_cast<HighsInt>(r));
            a_value.push_back(v);
        }
    }
    a_start[static_cast<std::size_t>(num_col)] = static_cast<HighsInt>(a_index.size());

    HighsLp lp {};
    lp.num_col_ = static_cast<HighsInt>(num_col);
    lp.num_row_ = static_cast<HighsInt>(num_row);
    lp.sense_ = ObjSense::kMinimize;
    lp.offset_ = 0.0;
    lp.col_cost_ = std::move(col_cost);
    lp.col_lower_ = std::move(col_lo);
    lp.col_upper_ = std::move(col_up);
    lp.row_lower_ = std::move(row_lo);
    lp.row_upper_ = std::move(row_up);
    lp.integrality_.assign(static_cast<std::size_t>(num_col), HighsVarType::kInteger);
    lp.model_name_ = std::string(stage_name);
    lp.a_matrix_.format_ = MatrixFormat::kColwise;
    lp.a_matrix_.num_col_ = lp.num_col_;
    lp.a_matrix_.num_row_ = lp.num_row_;
    lp.a_matrix_.start_ = std::move(a_start);
    lp.a_matrix_.index_ = std::move(a_index);
    lp.a_matrix_.value_ = std::move(a_value);
    lp.setMatrixDimensions();

    Highs highs {};
    highs.setOptionValue("output_flag", false);
    highs.setOptionValue("presolve", "on");
    if (highs.passModel(std::move(lp)) != HighsStatus::kOk) {
        out.ok = false;
        out.message = std::format("{}: passModel failed", stage_name);
        return out;
    }

    if (warm_start != nullptr && !warm_start->nodes_by_record_id.empty()) {
        auto warm_values_by_col = std::map<HighsInt, double> {};
        auto origin_o_col_by_h_node = std::map<std::pair<int, int>, int> {};
        for (std::size_t oi = 0; oi < origin_o_vars.size(); ++oi) {
            const auto& ov = origin_o_vars[oi];
            origin_o_col_by_h_node[{ov.h, ov.node}] = num_f + num_origin_x + static_cast<int>(oi);
        }

        std::size_t matched_paths = 0;
        for (int k = 0; k < K; ++k) {
            const auto& commodity = local_com[static_cast<std::size_t>(k)];
            const auto path_it = warm_start->nodes_by_record_id.find(commodity.record_id);
            if (path_it == warm_start->nodes_by_record_id.end()) {
                continue;
            }
            const auto& path = path_it->second;
            if (path.size() < 2) {
                continue;
            }
            ++matched_paths;
            const auto h = commodity_origin_h[static_cast<std::size_t>(k)];
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                const auto u = path[i];
                const auto v = path[i + 1];
                for (const auto f_col : f_by_k[static_cast<std::size_t>(k)]) {
                    const auto arc_id = f_vars[static_cast<std::size_t>(f_col)].a;
                    const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                    if (arc.u == u && arc.v == v) {
                        warm_values_by_col[static_cast<HighsInt>(f_col)] = 1.0;
                        const auto ox_it = origin_x_by_ha.find({h, arc_id});
                        if (ox_it != origin_x_by_ha.end()) {
                            warm_values_by_col[static_cast<HighsInt>(num_f + ox_it->second)] = 1.0;
                        }
                        break;
                    }
                }
            }
            for (const auto node : path) {
                if (graph.nodes[static_cast<std::size_t>(node)].is_virtual) {
                    continue;
                }
                const auto it = origin_o_col_by_h_node.find({h, node});
                if (it != origin_o_col_by_h_node.end()) {
                    warm_values_by_col[static_cast<HighsInt>(it->second)] = 1.0;
                }
            }
        }

        if (!warm_values_by_col.empty()) {
            auto warm_cols = std::vector<HighsInt> {};
            auto warm_values = std::vector<double> {};
            warm_cols.reserve(warm_values_by_col.size());
            warm_values.reserve(warm_values_by_col.size());
            for (const auto& [col, value] : warm_values_by_col) {
                warm_cols.push_back(col);
                warm_values.push_back(value);
            }
            (void)highs.setOptionValue("mip_max_start_nodes", static_cast<HighsInt>(0));
            const auto start_st = highs.setSolution(
                static_cast<HighsInt>(warm_cols.size()),
                warm_cols.data(),
                warm_values.data());
            if (start_st != HighsStatus::kOk) {
                debug::warning_fmt(
                    "{} warm start rejected by HiGHS (status={}, matched_paths={}, values={})",
                    stage_name,
                    static_cast<int>(start_st),
                    matched_paths,
                    warm_cols.size());
            }
            else {
                debug::info_fmt(
                    "{} warm start accepted by HiGHS: matched_paths={}, values={}",
                    stage_name,
                    matched_paths,
                    warm_cols.size());
            }
        }
    }

    if (highs.run() != HighsStatus::kOk) {
        out.ok = false;
        out.message = std::format("{}: solver run failed", stage_name);
        out.model_status = static_cast<int>(highs.getModelStatus());
        return out;
    }
    const auto status = highs.getModelStatus();
    out.model_status = static_cast<int>(status);
    if (status != HighsModelStatus::kOptimal) {
        if (warm_start != nullptr) {
            debug::warning_fmt(
                "{} warm start led to non-optimal status ({}); retrying without warm start",
                stage_name,
                static_cast<int>(status));
            return solve_simple_mcf_unit(
                graph,
                commodities,
                unit_c,
                simple_ids_for_unit,
                records,
                edge_capacity_override,
                node_capacity_override,
                enable_mcf_obj,
                nullptr);
        }
        out.ok = false;
        out.message = std::format("{}: model not optimal ({})", stage_name, static_cast<int>(status));
        return out;
    }

    out.ok = true;
    out.message = "ok";
    out.objective = highs.getObjectiveValue();

    const auto sol = highs.getSolution();
    auto f_values = std::Vector<int>(f_vars.size(), 0);
    for (std::size_t j = 0; j < f_vars.size(); ++j) {
        f_values[j] = static_cast<int>(std::lround(sol.col_value[j]));
    }
    for (std::size_t j = 0; j < origin_x_vars.size(); ++j) {
        const auto col = static_cast<std::size_t>(num_f + static_cast<int>(j));
        const auto val = static_cast<int>(std::lround(sol.col_value[col]));
        if (val <= 0) {
            continue;
        }
        const auto& arc = graph.arcs[static_cast<std::size_t>(origin_x_vars[j].a)];
        if (!arc.is_virtual) {
            auto u = arc.u;
            auto v = arc.v;
            if (u > v) {
                std::swap(u, v);
            }
            out.used_edges[{u, v}] = 1;
        }
    }
    for (std::size_t j = 0; j < origin_o_vars.size(); ++j) {
        const auto col = static_cast<std::size_t>(num_f + num_origin_x + static_cast<int>(j));
        const auto val = static_cast<int>(std::lround(sol.col_value[col]));
        if (val > 0) {
            out.used_nodes[origin_o_vars[j].node] = 1;
        }
    }

    append_paths_from_f_solution(graph, local_com, f_vars, f_values, out);
    return out;
}

auto path_to_text(const GlobalGraph& graph, const std::Vector<int>& path) -> std::String {
    if (path.empty()) {
        return "(empty)";
    }
    auto s = std::String {};
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i != 0) {
            s += " -> ";
        }
        s += node_text(graph, path[i]);
    }
    return s;
}

auto log_mcf_paths_by_origin_net(
    const GlobalGraph& graph,
    const std::array<std::Vector<McfPathInfo>, 16>& paths_by_unit,
    const std::Vector<Net_cost_record>& records
) -> void {
    struct PathRef {
        const McfPathInfo* info;
        std::size_t cob_unit;
    };
    auto refs = std::Vector<PathRef> {};
    for (std::size_t u = 0; u < 16; ++u) {
        for (const auto& info : paths_by_unit[u]) {
            refs.push_back(PathRef {&info, u});
        }
    }
    if (refs.empty()) {
        debug::info("MCF paths grouped by origin net (build_nets): (no paths)");
        return;
    }
    std::sort(refs.begin(), refs.end(), [&](const PathRef& a, const PathRef& b) {
        if (a.info->origin_name != b.info->origin_name) {
            return a.info->origin_name < b.info->origin_name;
        }
        const auto bit_a = (a.info->record_id < records.size()) ? records[a.info->record_id].bit_id : 0U;
        const auto bit_b = (b.info->record_id < records.size()) ? records[b.info->record_id].bit_id : 0U;
        if (bit_a != bit_b) {
            return bit_a < bit_b;
        }
        if (a.cob_unit != b.cob_unit) {
            return a.cob_unit < b.cob_unit;
        }
        return a.info->label < b.info->label;
    });

    debug::info("MCF paths grouped by origin net (logical net from build_nets / origin_key):");
    std::String current_origin {};
    for (const auto& pr : refs) {
        const auto& info = *pr.info;
        if (info.origin_name != current_origin) {
            current_origin = info.origin_name;
            debug::info_fmt("  origin net \"{}\"", current_origin);
        }
        std::size_t bit_id = 0;
        auto rec_name = std::String("(record_id out of range)");
        if (info.record_id < records.size()) {
            bit_id = records[info.record_id].bit_id;
            rec_name = records[info.record_id].net_name;
        }
        debug::info_fmt(
            "    bit={} 2pin_record=\"{}\" record_id={} COBUnit={} commodity={} start_track={} end_track={} path_count={}",
            bit_id,
            rec_name,
            info.record_id,
            pr.cob_unit,
            info.label,
            info.start_track,
            info.end_track,
            info.unit_paths.size());
        for (std::size_t pi = 0; pi < info.unit_paths.size(); ++pi) {
            debug::info_fmt("      path#{} {}", pi, path_to_text(graph, info.unit_paths[pi]));
        }
    }
}

} // namespace

auto run_mcf_global_routing_cob_units(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    hardware::Interposer* interposer,
    const circuit::BaseDie& basedie,
    const CobMcfGridDims cob_grid,
    const bool enable_mcf_parallel,
    const bool enable_pre_routing,
    const bool enable_mcf_obj,
    const bool defer_interposer_suspend
) -> CobMcfFullResult {
    (void)basedie;

    const auto mcf_start = std::chrono::steady_clock::now();
    const auto peak_before = get_peak_rss_mb();
    debug::info_fmt(
        "MCF: BusMCF (global) + SimpleMCF (per COBUnit); SimpleMCF objective={}",
        enable_mcf_obj ? "min_sum_x (--enable-mcf-obj)" : "feasibility only");

    CobMcfFullResult out {};
    out.summary.per_cob.resize(16);
    if (records.size() != ilp_result.assignments.size() || records.size() != ilp_result.record_track_endpoints.size()) {
        for (std::size_t u = 0; u < 16; ++u) {
            out.summary.per_cob[u] = CobMcfCobUnitSummary {
                u,
                0,
                false,
                0.0,
                0,
                std::String("record/ilp result size mismatch")};
        }
        out.summary.all_ok = false;
        return out;
    }

    auto graph = build_track_graph(cob_grid);
    auto commodities = prepare_commodities(records, ilp_result, cob_grid, graph);
    auto bus_ids = std::Vector<std::size_t> {};
    auto simple_ids = std::Vector<std::size_t> {};
    auto bus_count_by_unit = std::array<int, 16> {};
    auto simple_count_by_unit = std::array<int, 16> {};
    bus_count_by_unit.fill(0);
    simple_count_by_unit.fill(0);
    for (std::size_t i = 0; i < commodities.size(); ++i) {
        if (commodities[i].is_bus) {
            bus_ids.push_back(i);
            bus_count_by_unit[commodities[i].cob_unit] += 1;
        }
        else {
            simple_ids.push_back(i);
            simple_count_by_unit[commodities[i].cob_unit] += 1;
        }
    }

    auto simple_ids_by_unit = std::array<std::Vector<std::size_t>, 16> {};
    for (const auto id : simple_ids) {
        simple_ids_by_unit[commodities[id].cob_unit].push_back(id);
    }

    auto bus_warm_start = StageWarmStart {};
    auto simple_warm_start = StageWarmStart {};
    const StageWarmStart* bus_warm_start_ptr = nullptr;
    const StageWarmStart* simple_warm_start_ptr = nullptr;
    const auto mcf_warm_t0 = std::chrono::steady_clock::now();
    if (enable_pre_routing) {
        auto outgoing_arcs = std::Vector<std::Vector<int>>(graph.nodes.size());
        for (std::size_t a = 0; a < graph.arcs.size(); ++a) {
            outgoing_arcs[static_cast<std::size_t>(graph.arcs[a].u)].push_back(static_cast<int>(a));
        }
        auto warm_used_edges = std::map<std::pair<int, int>, int> {};
        auto warm_used_nodes = std::map<int, int> {};
        bus_warm_start = route_mcf_stage_warm_start(
            "BusMCF",
            graph,
            commodities,
            bus_ids,
            outgoing_arcs,
            warm_used_edges,
            warm_used_nodes);
        simple_warm_start = route_mcf_stage_warm_start(
            "SimpleMCF",
            graph,
            commodities,
            simple_ids,
            outgoing_arcs,
            warm_used_edges,
            warm_used_nodes);
        bus_warm_start_ptr = &bus_warm_start;
        simple_warm_start_ptr = &simple_warm_start;
    }
    const auto mcf_warm_t1 = std::chrono::steady_clock::now();
    out.summary.mcf_warm_start_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(mcf_warm_t1 - mcf_warm_t0).count());
    debug::info_fmt("timing phase=mcf_warm_start ms={}", out.summary.mcf_warm_start_ms);

    const auto solve_t0 = std::chrono::steady_clock::now();
    auto bus_res = solve_bus_mcf(graph, commodities, bus_ids, bus_warm_start_ptr);
    
    // 得到剩余容量
    auto build_edge_residual = [&](const std::size_t unit_c) {
        auto edge_cap = std::map<std::pair<int, int>, int> {};
        for (const auto& [e, used] : bus_res.unit_used_edges[unit_c]) {
            edge_cap[e] = std::max(0, 1 - used);
        }
        return edge_cap;
    };
    auto build_node_residual = [&](const std::size_t unit_c) {
        auto node_cap = std::map<int, int> {};
        for (const auto& [n, used] : bus_res.unit_used_nodes[unit_c]) {
            node_cap[n] = std::max(0, 1 - used);
        }
        return node_cap;
    };

    // solve
    auto simple_results = std::array<StageSolveResult, 16> {};
    auto simple_futures = std::array<std::future<StageSolveResult>, 16> {};
    auto has_simple_unit = std::array<bool, 16> {};
    has_simple_unit.fill(false);

    for (std::size_t u = 0; u < 16; ++u) {
        if (simple_ids_by_unit[u].empty()) {
            simple_results[u].ok = true;
            simple_results[u].message = "empty stage";
            continue;
        }
        has_simple_unit[u] = true;
        const auto edge_cap = build_edge_residual(u);
        const auto node_cap = build_node_residual(u);
        if (enable_mcf_parallel) {
            simple_futures[u] = std::async(
                std::launch::async,
                [&graph, &commodities, &records, &simple_ids_by_unit, u, edge_cap, node_cap, enable_mcf_obj, simple_warm_start_ptr]() {
                    return solve_simple_mcf_unit(
                        graph,
                        commodities,
                        u,
                        simple_ids_by_unit[u],
                        records,
                        edge_cap,
                        node_cap,
                        enable_mcf_obj,
                        simple_warm_start_ptr);
                });
        }
        else {
            simple_results[u] = solve_simple_mcf_unit(
                graph,
                commodities,
                u,
                simple_ids_by_unit[u],
                records,
                edge_cap,
                node_cap,
                enable_mcf_obj,
                simple_warm_start_ptr);
            debug::info_fmt(
                "SimpleMCF unit {}: ok={} objective={:.0f} paths={}",
                u,
                simple_results[u].ok,
                simple_results[u].objective,
                simple_results[u].paths.size());
        }
    }

    // 统计结果
    if (enable_mcf_parallel) {
        for (std::size_t u = 0; u < 16; ++u) {
            if (!has_simple_unit[u]) {
                continue;
            }
            simple_results[u] = simple_futures[u].get();
            debug::info_fmt(
                "SimpleMCF unit {}: ok={} objective={:.0f} paths={}",
                u,
                simple_results[u].ok,
                simple_results[u].objective,
                simple_results[u].paths.size());
        }
    }

    const auto solve_t1 = std::chrono::steady_clock::now();
    const auto solve_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(solve_t1 - solve_t0).count());
    out.summary.mcf_solve_ms = solve_ms;
    debug::info_fmt("timing phase=mcf_solve ms={}", solve_ms);

    if (!bus_res.ok) {
        debug::error_fmt("BusMCF failed: {}", bus_res.message);
    }
    bool all_simple_ok = true;
    double simple_objective = 0.0;
    for (std::size_t u = 0; u < 16; ++u) {
        out.has_simple_commodities[u] = has_simple_unit[u];
        if (!has_simple_unit[u]) {
            out.simple_mcf_ok[u] = true;
            continue;
        }
        out.simple_mcf_ok[u] = simple_results[u].ok;
        if (!simple_results[u].ok) {
            all_simple_ok = false;
            debug::error_fmt("SimpleMCF unit {} failed: {}", u, simple_results[u].message);
        }
        simple_objective += simple_results[u].objective;
    }
    out.summary.all_ok = bus_res.ok && all_simple_ok;

    for (std::size_t u = 0; u < 16; ++u) {
        const auto obj = bus_res.objective + simple_objective;
        out.summary.per_cob[u] = CobMcfCobUnitSummary {
            u,
            bus_count_by_unit[u] + simple_count_by_unit[u],
            out.summary.all_ok,
            obj,
            solve_ms,
            out.summary.all_ok ? std::String("ok") : std::String("stage failed")};
    }

    for (auto& p : bus_res.paths) {
        out.paths_by_unit[p.cob_unit].push_back(std::move(p));
    }
    for (std::size_t u = 0; u < 16; ++u) {
        for (auto& p : simple_results[u].paths) {
            out.paths_by_unit[u].push_back(std::move(p));
        }
    }

    for (std::size_t u = 0; u < 16; ++u) {
        debug::info_fmt(
            "MCF unit {}: bus_com={} simple_com={} ok={}",
            u,
            bus_count_by_unit[u],
            simple_count_by_unit[u],
            out.summary.all_ok);
        for (const auto& info : out.paths_by_unit[u]) {
            debug::info_fmt(
                "  commodity {} rec={} start_track={} end_track={} path_count={} (track path text under origin net below)",
                info.label,
                info.record_id,
                info.start_track,
                info.end_track,
                info.unit_paths.size());
        }
    }
    log_mcf_paths_by_origin_net(graph, out.paths_by_unit, records);

    const auto mcf_end = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(mcf_end - mcf_start).count();
    const auto peak_after = get_peak_rss_mb();
    const auto stage_peak_delta = std::max(0.0, peak_after - peak_before);
    debug::info_fmt(
        "MCF detailed(track-level) summary: total_elapsed={} ms (mcf_warm_start={} ms, mcf_solve={} ms), peak_rss={:.2f} MB, "
        "stage_peak_delta={:.2f} MB",
        elapsed_ms,
        out.summary.mcf_warm_start_ms,
        out.summary.mcf_solve_ms,
        peak_after,
        stage_peak_delta);
    if (interposer != nullptr && !defer_interposer_suspend) {
        suspend_mcf_paths_on_interposer(interposer, graph, out.paths_by_unit);
    }
    return out;
}

auto build_mcf_track_graph(const CobMcfGridDims grid) -> McfGlobalGraph {
    return build_track_graph(grid);
}

auto is_sync_bus_mcf_origin_key(const std::String& origin_key) -> bool {
    return is_sync_bus_origin_key(origin_key);
}

namespace {

auto track_from_node_meta_impl(hardware::Interposer* interposer, const McfNodeMeta& m) -> hardware::Track* {
    if (interposer == nullptr || m.is_virtual) {
        return nullptr;
    }
    const auto dir = m.track_dir == 0 ? hardware::TrackDirection::Horizontal : hardware::TrackDirection::Vertical;
    const auto tc = hardware::TrackCoord {
        static_cast<std::i64>(m.track_row),
        static_cast<std::i64>(m.track_col),
        dir,
        static_cast<std::usize>(m.track)};
    const auto opt = interposer->get_track(tc);
    return opt.has_value() ? opt.value() : nullptr;
}

} // namespace

auto suspend_mcf_paths_on_interposer(
    hardware::Interposer* interposer,
    const McfGlobalGraph& graph,
    const std::array<std::Vector<McfPathInfo>, 16>& paths_by_unit
) -> void {
    if (interposer == nullptr) {
        return;
    }
    int suspended = 0;
    int skipped_no_adj = 0;
    for (std::size_t u = 0; u < 16; ++u) {
        for (const auto& info : paths_by_unit[u]) {
            for (const auto& path : info.unit_paths) {
                for (std::size_t i = 1; i < path.size(); ++i) {
                    const int na = path[i - 1];
                    const int nb = path[i];
                    if (na < 0 || nb < 0 || static_cast<std::size_t>(na) >= graph.nodes.size()
                        || static_cast<std::size_t>(nb) >= graph.nodes.size()) {
                        continue;
                    }
                    const auto& ma = graph.nodes[static_cast<std::size_t>(na)];
                    const auto& mb = graph.nodes[static_cast<std::size_t>(nb)];
                    if (ma.is_virtual || mb.is_virtual) {
                        continue;
                    }
                    auto* ta = track_from_node_meta_impl(interposer, ma);
                    auto* tb = track_from_node_meta_impl(interposer, mb);
                    if (ta == nullptr || tb == nullptr) {
                        continue;
                    }
                    bool found = false;
                    for (auto [tn, conn] : interposer->adjacent_tracks(ta)) {
                        if (tn != tb) {
                            continue;
                        }
                        found = true;
                        if (!conn.is_occupied()) {
                            conn.suspend();
                            suspended += 1;
                        }
                        break;
                    }
                    if (!found) {
                        skipped_no_adj += 1;
                    }
                }
            }
        }
    }
    if (skipped_no_adj > 0) {
        debug::warning_fmt(
            "MCF→Interposer: {} hop(s) had no matching adjacent_tracks() edge (graph vs hardware mismatch?)",
            skipped_no_adj);
    }
    debug::info_fmt("MCF→Interposer: suspended {} COBConnector(s) along MCF paths", suspended);
}

} // namespace PR_tool
