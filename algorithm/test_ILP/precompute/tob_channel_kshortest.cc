#include "precompute/tob_channel_kshortest.hh"

#include "mcf/mcf_hw_map.hh"

#include <hardware/cob/cobunit.hh>
#include <hardware/interposer.hh>
#include <hardware/tob/tobcoord.hh>

#include <algorithm>
#include <cstddef>
#include <format>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <utility>

namespace PR_tool {

namespace {

using TrackKey = std::tuple<std::i64, std::i64, int, std::size_t>;

auto track_key(const hardware::TrackCoord& tc) -> TrackKey {
    return {
        tc.row,
        tc.col,
        tc.dir == hardware::TrackDirection::Horizontal ? 0 : 1,
        tc.index};
}

auto key_to_coord(const TrackKey& key) -> hardware::TrackCoord {
    const auto [row, col, dir, index] = key;
    return hardware::TrackCoord {
        row,
        col,
        dir == 0 ? hardware::TrackDirection::Horizontal : hardware::TrackDirection::Vertical,
        index};
}

auto cob_in_bbox(const hardware::COBCoord& cob, const IlpBoundingBox& bbox) -> bool {
    return cob.row >= bbox.row_min && cob.row <= bbox.row_max && cob.col >= bbox.col_min && cob.col <= bbox.col_max;
}

auto track_cob(const hardware::TrackCoord& tc) -> hardware::COBCoord {
    return mcf::track_to_cob(tc);
}

} // namespace

auto tob_channel_track_coords(const std::size_t tob_linear) -> std::Vector<hardware::TrackCoord> {
    const auto [tr, tc] = mcf::tob_index_from_linear(tob_linear);
    const auto upper = hardware::COBCoord {
        static_cast<std::i64>(1 + 2 * static_cast<std::i64>(tr)),
        static_cast<std::i64>(3 * static_cast<std::i64>(tc))};
    auto out = std::Vector<hardware::TrackCoord> {};
    out.reserve(128);
    for (std::size_t idx = 0; idx < 128; ++idx) {
        out.push_back(hardware::TrackCoord {
            upper.row,
            upper.col,
            hardware::TrackDirection::Vertical,
            idx});
    }
    return out;
}

namespace {

auto side_track_pos(
    hardware::COBDirection from,
    std::i64 cob_r,
    std::i64 cob_c
) -> std::tuple<std::i64, std::i64, hardware::TrackDirection> {
    using D = hardware::COBDirection;
    switch (from) {
        case D::Down:
            return {cob_r, cob_c, hardware::TrackDirection::Vertical};
        case D::Up:
            return {cob_r + 1, cob_c, hardware::TrackDirection::Vertical};
        case D::Left:
            return {cob_r, cob_c, hardware::TrackDirection::Horizontal};
        case D::Right:
            return {cob_r, cob_c + 1, hardware::TrackDirection::Horizontal};
    }
    return {cob_r, cob_c, hardware::TrackDirection::Vertical};
}

auto is_straight_through(hardware::COBDirection from, hardware::COBDirection to) -> bool {
    using D = hardware::COBDirection;
    return (from == D::Left && to == D::Right) || (from == D::Right && to == D::Left)
        || (from == D::Up && to == D::Down) || (from == D::Down && to == D::Up);
}

auto build_bbox_track_graph(const IlpBoundingBox& bbox) -> std::map<TrackKey, std::Vector<TrackKey>> {
    using D = hardware::COBDirection;
    auto graph = std::map<TrackKey, std::Vector<TrackKey>> {};
    const auto rows = static_cast<std::i64>(hardware::Interposer::COB_ARRAY_HEIGHT);
    const auto cols = static_cast<std::i64>(hardware::Interposer::COB_ARRAY_WIDTH);

    const auto add_edge = [&](const TrackKey& u, const TrackKey& v) {
        if (u == v) {
            return;
        }
        graph[u].push_back(v);
        graph[v].push_back(u);
    };

    for (std::i64 cob_r = bbox.row_min; cob_r <= bbox.row_max; ++cob_r) {
        for (std::i64 cob_c = bbox.col_min; cob_c <= bbox.col_max; ++cob_c) {
            const hardware::COBCoord cob {cob_r, cob_c};
            if (!cob_in_bbox(cob, bbox)) {
                continue;
            }
            for (std::usize inner = 0; inner < 8; ++inner) {
                const std::array directions {D::Down, D::Up, D::Left, D::Right};
                for (const auto from : directions) {
                    for (const auto to : directions) {
                        if (from == to) {
                            continue;
                        }
                        const auto [in_r, in_c, in_dir] = side_track_pos(from, cob_r, cob_c);
                        const auto [out_r, out_c, out_dir] = side_track_pos(to, cob_r, cob_c);
                        const auto mapped = hardware::COBUnit::index_map(from, inner, to);
                        const auto u = track_key(hardware::TrackCoord {in_r, in_c, in_dir, inner});
                        const auto v = track_key(hardware::TrackCoord {out_r, out_c, out_dir, mapped});
                        const auto u_cob = track_cob(key_to_coord(u));
                        const auto v_cob = track_cob(key_to_coord(v));
                        if (!cob_in_bbox(u_cob, bbox) || !cob_in_bbox(v_cob, bbox)) {
                            continue;
                        }
                        if (is_straight_through(from, to)) {
                            add_edge(u, v);
                        }
                        else {
                            add_edge(u, v);
                        }
                    }
                }
            }
        }
    }

    for (auto& [node, nbrs] : graph) {
        std::sort(nbrs.begin(), nbrs.end());
        nbrs.erase(std::unique(nbrs.begin(), nbrs.end()), nbrs.end());
    }
    return graph;
}

auto bfs_shortest_path(
    const std::map<TrackKey, std::Vector<TrackKey>>& graph,
    const TrackKey& src,
    const std::set<TrackKey>& targets,
    const std::set<std::pair<TrackKey, TrackKey>>& banned_edges
) -> std::Vector<TrackKey> {
    if (targets.contains(src)) {
        return {src};
    }
    auto prev = std::map<TrackKey, TrackKey> {};
    auto q = std::queue<TrackKey> {};
    auto seen = std::set<TrackKey> {src};
    q.push(src);
    while (!q.empty()) {
        const auto u = q.front();
        q.pop();
        const auto it = graph.find(u);
        if (it == graph.end()) {
            continue;
        }
        for (const auto& v : it->second) {
            const auto edge = std::pair<TrackKey, TrackKey> {u, v};
            if (banned_edges.contains(edge)) {
                continue;
            }
            if (seen.contains(v)) {
                continue;
            }
            seen.insert(v);
            prev.emplace(v, u);
            if (targets.contains(v)) {
                auto path = std::Vector<TrackKey> {v};
                auto cur = v;
                while (cur != src) {
                    cur = prev.at(cur);
                    path.push_back(cur);
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            q.push(v);
        }
    }
    return {};
}

auto yen_k_shortest(
    const std::map<TrackKey, std::Vector<TrackKey>>& graph,
    const TrackKey& src,
    const std::set<TrackKey>& targets,
    std::size_t k
) -> std::Vector<std::Vector<TrackKey>> {
    auto paths = std::Vector<std::Vector<TrackKey>> {};
    const auto first = bfs_shortest_path(graph, src, targets, {});
    if (first.empty()) {
        return paths;
    }
    paths.push_back(first);

    struct Candidate {
        std::Vector<TrackKey> path;
        std::size_t cost;
        auto operator>(const Candidate& other) const -> bool {
            return cost > other.cost;
        }
    };
    auto candidates = std::priority_queue<Candidate, std::Vector<Candidate>, std::greater<Candidate>> {};

    for (std::size_t ki = 1; ki < k; ++ki) {
        const auto& prev_path = paths[ki - 1];
        for (std::size_t i = 0; i + 1 < prev_path.size(); ++i) {
            const auto spur_node = prev_path[i];
            const auto root = std::Vector<TrackKey> {prev_path.begin(), prev_path.begin() + static_cast<std::ptrdiff_t>(i + 1)};
            auto banned = std::set<std::pair<TrackKey, TrackKey>> {};
            for (std::size_t p = 0; p < paths.size(); ++p) {
                const auto& path_p = paths[p];
                if (path_p.size() > i && std::equal(root.begin(), root.end(), path_p.begin())) {
                    if (i + 1 < path_p.size()) {
                        banned.insert({path_p[i], path_p[i + 1]});
                        banned.insert({path_p[i + 1], path_p[i]});
                    }
                }
            }
            const auto spur = bfs_shortest_path(graph, spur_node, targets, banned);
            if (spur.empty()) {
                continue;
            }
            auto total = std::Vector<TrackKey> {};
            total.insert(total.end(), root.begin(), root.end());
            if (spur.front() == spur_node) {
                total.insert(total.end(), spur.begin() + 1, spur.end());
            }
            else {
                total.insert(total.end(), spur.begin(), spur.end());
            }
            candidates.push(Candidate {total, total.size()});
        }
        if (candidates.empty()) {
            break;
        }
        paths.push_back(candidates.top().path);
        candidates.pop();
    }
    return paths;
}

} // namespace

auto kshortest_reachable_tob_tracks(
    const hardware::TrackCoord& end_track,
    const std::size_t start_tob_linear,
    const IlpBoundingBox& bbox,
    const std::size_t k
) -> std::Vector<std::size_t> {
    if (k == 0) {
        return {};
    }
    auto graph = build_bbox_track_graph(bbox);
    const auto src = track_key(end_track);
    if (!graph.contains(src)) {
        graph[src] = {};
    }

    auto targets = std::set<TrackKey> {};
    for (const auto& tc : tob_channel_track_coords(start_tob_linear)) {
        targets.insert(track_key(tc));
    }

    const auto paths = yen_k_shortest(graph, src, targets, k);
    auto out = std::Vector<std::size_t> {};
    auto seen = std::set<std::size_t> {};
    for (const auto& path : paths) {
        if (path.empty()) {
            continue;
        }
        const auto end_key = path.back();
        const auto tc = key_to_coord(end_key);
        if (seen.contains(tc.index)) {
            continue;
        }
        seen.insert(tc.index);
        out.push_back(tc.index);
    }
    return out;
}

} // namespace PR_tool
