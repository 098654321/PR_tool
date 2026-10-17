#pragma once

#include "rrr_routing.hh"
#include "maze_search.hh"

#include <debug/debug.hh>
#include <cmath>
#include <deque>
#include <queue>

namespace PR_tool::rrr_detail {

struct AirSpatial {
    RoutingBox initial; // Unpadded terminal bbox: fixed bin origin and area.
    RoutingBox device;
    RoutingBox router_bb; // Persistent per-net search box, updated only between rounds.
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
    if (graph.rows > 0 && graph.cols > 0) {
        // Tracks/I/O on row=height and col=width are the physical outer edges.
        result.device = {0, graph.rows, 0, graph.cols};
    } else {
        // Synthetic graph seam; production always has explicit device dimensions.
        for (std::size_t id = 0; id < graph.nodes.size(); ++id) {
            const auto [r, c] = routing_node_position(graph, id);
            if (first) { result.device = {r, r, c, c}; first = false; }
            else result.device.include(r, c);
        }
    }
    first = true;
    for (const auto& terminal : net_terminals(net)) {
        const int id = resolve_graph_node(graph, terminal);
        if (id < 0) continue;
        const auto [r, c] = routing_node_position(graph, id);
        if (first) { result.initial = {r, r, c, c}; first = false; }
        else result.initial.include(r, c);
    }
    // A collinear/point bbox still needs nonempty spatial bins.
    const double area = std::max(1, result.initial.r1 - result.initial.r0)
        * std::max(1, result.initial.c1 - result.initial.c0);
    result.bin_size = 1 + std::sqrt(4 * area / std::max<std::size_t>(1, net.demands.size()));
    result.router_bb = {
        std::max(result.device.r0, result.initial.r0-1),
        std::min(result.device.r1, result.initial.r1+1),
        std::max(result.device.c0, result.initial.c0-1),
        std::min(result.device.c1, result.initial.c1+1)};
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

// Weighted Dijkstra within this round's fixed per-net box. No in-search
// expansion or full-device fallback: only a successful rerouted net may grow
// its persistent box at the end of a round.
inline auto air_branch(const UnifiedGraph& graph, const ResourceModel& resources,
                       const OwnerRecord& owner, const std::Vector<int>& starts,
                       int sink, int unit, const AirSpatial& spatial,
                       const std::Set<std::pair<int, int>>& bans) -> std::Vector<int> {
    struct Label { double cost; std::Vector<int> path; };
    using Item = std::pair<double, std::size_t>;
    auto heap = std::priority_queue<Item, std::Vector<Item>, std::greater<Item>>{};
    auto labels = std::Vector<Label>{};
    auto best = std::Vector<double>(graph.nodes.size(), std::numeric_limits<double>::infinity());
    const auto& box = spatial.router_bb;
    for (int n : starts) {
        if (graph.nodes[n].kind == UnifiedNodeKind::Track && static_cast<int>(graph.nodes[n].unit) != unit) continue;
        const auto [r, c] = routing_node_position(graph, n);
        if (!box.contains(r, c)) continue;
        if (best[n] == 0) continue;
        best[n] = 0; labels.push_back({0, {n}}); heap.emplace(0, labels.size()-1);
    }
    int expansions = 0;
    while (!heap.empty() && expansions < 300000) {
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
            const auto [r, c] = routing_node_position(graph, arc.v);
            if (!box.contains(r, c)) continue;
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
