#pragma once

#include "rrr_routing.hh"
#include "maze_search.hh"
#include "hw_map.hh"

#include <debug/debug.hh>
#include <cmath>
#include <deque>
#include <queue>

namespace PR_tool::rrr_detail {

struct AirBox {
    int r0{0}, r1{0}, c0{0}, c1{0};
    auto contains(int r, int c) const -> bool { return r >= r0 && r <= r1 && c >= c0 && c <= c1; }
    auto include(int r, int c) -> void {
        r0 = std::min(r0, r); r1 = std::max(r1, r);
        c0 = std::min(c0, c); c1 = std::max(c1, c);
    }
};

inline auto air_position(const UnifiedGraph& graph, int id) -> std::pair<int, int> {
    const auto& node = graph.nodes[id];
    if (node.kind == UnifiedNodeKind::Track) return {node.track_row, node.track_col};
    const auto coord = tob_anchor_cob(node.kind == UnifiedNodeKind::Bump ? node.bump.TOB : node.tob);
    return {static_cast<int>(coord.row), static_cast<int>(coord.col)};
}

struct AirSpatial {
    AirBox initial;
    AirBox device;
    double bin_size{1};
    std::Map<std::pair<int, int>, std::Vector<int>> bins;
    auto bin(int r, int c) const -> std::pair<int, int> {
        return {static_cast<int>(std::floor((r - initial.r0) / bin_size)),
                static_cast<int>(std::floor((c - initial.c0) / bin_size))};
    }
};

inline auto air_spatial(const UnifiedGraph& graph, const RoutingNet& net) -> AirSpatial {
    auto result = AirSpatial{};
    bool first = true;
    for (std::size_t id = 0; id < graph.nodes.size(); ++id) {
        const auto [r, c] = air_position(graph, id);
        if (first) { result.device = {r, r, c, c}; first = false; }
        else result.device.include(r, c);
    }
    first = true;
    for (const auto& terminal : net_terminals(net)) {
        const int id = resolve_graph_node(graph, terminal);
        if (id < 0) continue;
        const auto [r, c] = air_position(graph, id);
        if (first) { result.initial = {r, r, c, c}; first = false; }
        else result.initial.include(r, c);
    }
    // A collinear/point bbox still needs nonempty spatial bins.
    const double area = std::max(1, result.initial.r1 - result.initial.r0)
        * std::max(1, result.initial.c1 - result.initial.c0);
    result.bin_size = std::sqrt(4 * area / std::max<std::size_t>(1, net.demands.size()));
    return result;
}

inline auto air_root_prefix(const OwnerRecord& owner, const std::Vector<int>& roots, int target)
    -> std::Vector<int> {
    auto adjacent = std::Map<int, std::Set<int>>{};
    for (const auto& path : owner.demand_paths)
        for (std::size_t i = 1; i < path.size(); ++i) {
            adjacent[path[i-1]].insert(path[i]); adjacent[path[i]].insert(path[i-1]);
        }
    auto previous = std::Map<int, int>{};
    auto queue = std::Queue<int>{};
    for (int root : roots) { previous.emplace(root, -1); queue.push(root); }
    while (!queue.empty() && !previous.contains(target)) {
        const int node = queue.front(); queue.pop();
        for (int next : adjacent[node])
            if (previous.emplace(next, node).second) queue.push(next);
    }
    if (!previous.contains(target)) return {};
    auto path = std::Vector<int>{};
    for (int n = target; n >= 0; n = previous.at(n)) path.push_back(n);
    std::reverse(path.begin(), path.end());
    return path;
}

// Weighted Dijkstra (no lookahead). Expansion resumes at blocked frontiers when
// the current terminal bbox cannot complete the connection; it may grow to the
// entire device, so the bbox cannot permanently exclude an available route.
inline auto air_branch(const UnifiedGraph& graph, const ResourceModel& resources,
                       const OwnerRecord& owner, const std::Vector<int>& starts,
                       int sink, int unit, const AirSpatial& spatial,
                       const std::Set<std::pair<int, int>>& bans) -> std::Vector<int> {
    struct Label { double cost; std::Vector<int> path; };
    using Item = std::pair<double, std::size_t>;
    auto heap = std::priority_queue<Item, std::Vector<Item>, std::greater<Item>>{};
    auto labels = std::Vector<Label>{};
    auto best = std::Vector<double>(graph.nodes.size(), std::numeric_limits<double>::infinity());
    auto box = spatial.initial;
    for (int n : starts) {
        if (graph.nodes[n].kind == UnifiedNodeKind::Track && static_cast<int>(graph.nodes[n].unit) != unit) continue;
        const auto [r, c] = air_position(graph, n); box.include(r, c);
        if (best[n] == 0) continue;
        best[n] = 0; labels.push_back({0, {n}}); heap.emplace(0, labels.size()-1);
    }
    auto frontier = std::Set<std::size_t>{};
    bool low_r = false, high_r = false, low_c = false, high_c = false;
    int expansions = 0;
    while ((!heap.empty() || !frontier.empty()) && expansions < 300000) {
        if (heap.empty()) {
            box.r0 = low_r ? std::max(spatial.device.r0, box.r0-1) : box.r0;
            box.r1 = high_r ? std::min(spatial.device.r1, box.r1+1) : box.r1;
            box.c0 = low_c ? std::max(spatial.device.c0, box.c0-1) : box.c0;
            box.c1 = high_c ? std::min(spatial.device.c1, box.c1+1) : box.c1;
            for (auto index : frontier) heap.emplace(labels[index].cost, index);
            frontier.clear(); low_r = high_r = low_c = high_c = false;
        }
        ++expansions;
        const auto [cost, index] = heap.top(); heap.pop();
        const auto label = labels[index];
        const int node = label.path.back();
        if (cost > best[node] + 1e-9) continue;
        if (node == sink) return label.path;
        for (int aid : graph.out_arc_ids[node]) {
            const auto& arc = graph.arcs[aid];
            const auto& dest = graph.nodes[arc.v];
            if (bans.contains(std::minmax(arc.u, arc.v)) ||
                (dest.kind == UnifiedNodeKind::Bump && arc.v != sink) ||
                (dest.kind == UnifiedNodeKind::Track && static_cast<int>(dest.unit) != unit) ||
                (dest.kind == UnifiedNodeKind::VLine && static_cast<int>(dest.line_index % 8) != unit % 8) ||
                std::find(label.path.begin(), label.path.end(), arc.v) != label.path.end() ||
                owner.tree.contains(arc.v)) continue;
            const auto [r, c] = air_position(graph, arc.v);
            if (!box.contains(r, c)) {
                low_r = low_r || r < box.r0; high_r = high_r || r > box.r1;
                low_c = low_c || c < box.c0; high_c = high_c || c > box.c1;
                frontier.insert(index); continue;
            }
            const double next = cost + resources.node_cost(owner.id, arc.v);
            if (next + 1e-9 >= best[arc.v]) continue;
            best[arc.v] = next;
            auto path = label.path; path.push_back(arc.v);
            labels.push_back({next, std::move(path)}); heap.emplace(next, labels.size()-1);
        }
    }
    return {};
}

inline auto air_connection(const UnifiedGraph& graph, const RoutingNet& net,
                           const OwnerRecord& owner, std::size_t slot,
                           const ResourceModel& resources, const AirSpatial& spatial,
                           const std::Vector<int>& starts) -> std::Vector<int> {
    const auto& demand = net.demands[owner.demand_indices[slot]];
    const auto roots = demand_source_nodes(graph, net, demand);
    const int sink = resolve_graph_node(graph, demand.sink);
    if (sink < 0) return {};
    double best_cost = std::numeric_limits<double>::infinity();
    auto best_path = std::Vector<int>{};
    for (int unit = 0; unit < 16; ++unit) {
        if ((owner.is_bnet && resources.selected_unit(owner.id) >= 0
             && resources.selected_unit(owner.id) != unit) ||
            (graph.nodes[sink].kind == UnifiedNodeKind::Track && static_cast<int>(graph.nodes[sink].unit) != unit)) continue;
        using Bans = std::Set<std::pair<int, int>>;
        auto pending = std::deque<Bans>{{}};
        auto seen = std::Set<Bans>{{}};
        int attempts = 0;
        while (!pending.empty() && attempts++ < 32) {
            auto bans = std::move(pending.front()); pending.pop_front();
            auto branch = air_branch(graph, resources, owner, starts, sink, unit, spatial, bans);
            if (branch.empty()) continue;
            auto path = air_root_prefix(owner, roots, branch.front());
            if (path.empty()) continue;
            path.insert(path.end(), branch.begin()+1, branch.end());
            auto candidate = owner; candidate.demand_paths[slot] = path;
            if (owner_paths_valid(graph, candidate)) {
                double cost = 0;
                for (std::size_t i = 1; i < branch.size(); ++i) cost += resources.node_cost(owner.id, branch[i]);
                if (cost < best_cost) { best_cost = cost; best_path = std::move(path); }
                break;
            }
            for (std::size_t i = 1; i < branch.size(); ++i) {
                auto next = bans; next.insert(std::minmax(branch[i-1], branch[i]));
                if (seen.insert(next).second) pending.push_back(std::move(next));
            }
        }
    }
    return best_path;
}

} // namespace PR_tool::rrr_detail
