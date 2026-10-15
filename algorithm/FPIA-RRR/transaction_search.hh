#pragma once

#include "rrr_routing.hh"
#include "maze_search.hh"

#include <deque>
#include <algorithm>
#include <queue>

namespace PR_tool::rrr_detail {

// Full-array counterpart of test_ILP/route_ilp/route_search: physical source
// paths, unit/demand/equal-cost ordering variants and at most 32 legality retries.
inline auto transaction_path(const UnifiedGraph& graph, const ResourceModel& resources,
                             OwnerId owner, int source, int sink, int unit,
                             const RrrParams& params,
                             const std::set<std::pair<int, int>>& bans)
    -> std::Vector<int> {
    struct Label { double cost; int node; std::Vector<int> path; };
    auto labels = std::Vector<Label>{{0, source, {source}}};
    using Item = std::pair<double, std::size_t>;
    auto queue = std::priority_queue<Item, std::Vector<Item>, std::greater<Item>>{};
    queue.emplace(0, 0);
    auto best = std::Vector<double>(graph.nodes.size(), std::numeric_limits<double>::infinity());
    best[source] = 0;
    int expansions = 0;
    while (!queue.empty() && expansions++ < 300000) {
        const auto [cost, index] = queue.top(); queue.pop();
        const auto label = labels[index];
        if (cost > best[label.node] + 1e-9) continue;
        if (label.node == sink) return label.path;
        const auto& outgoing = graph.out_arc_ids[label.node];
        for (std::size_t i = 0; i < outgoing.size(); ++i) {
            const auto& arc = graph.arcs[outgoing[(i + params.variant) % outgoing.size()]];
            const auto& node = graph.nodes[arc.v];
            if (bans.contains(std::minmax(arc.u, arc.v)) ||
                (node.kind == UnifiedNodeKind::Bump && arc.v != sink) ||
                (node.kind == UnifiedNodeKind::Track && static_cast<int>(node.unit) != unit) ||
                (node.kind == UnifiedNodeKind::VLine && static_cast<int>(node.line_index % 8) != unit % 8) ||
                std::find(label.path.begin(), label.path.end(), arc.v) != label.path.end()) continue;
            const auto next = cost + arc_incremental_cost(graph, resources, owner, arc, {source}, params);
            if (next + 1e-9 >= best[arc.v]) continue;
            best[arc.v] = next;
            auto path = label.path; path.push_back(arc.v);
            labels.push_back({next, arc.v, std::move(path)});
            queue.emplace(next, labels.size() - 1);
        }
    }
    return {};
}

inline auto search_transaction_owner(OwnerRecord& owner, const UnifiedGraph& graph,
                                     const std::Vector<RoutingNet>& nets,
                                     ResourceModel& resources, const RrrParams& params) -> bool {
    const auto& net = nets[owner.net_index];
    auto order = owner.demand_indices;
    if (!order.empty()) std::rotate(order.begin(), order.begin() + params.variant % order.size(), order.end());
    for (const auto demand_index : order) {
        const auto& demand = net.demands[demand_index];
        const int sink = resolve_graph_node(graph, demand.sink);
        auto best_path = std::Vector<int>{};
        auto best_cost = std::numeric_limits<double>::infinity();
        for (const auto source : demand_source_nodes(graph, net, demand)) {
            for (int u = 0; u < 16; ++u) {
                const int unit = (u + params.variant) % 16;
                if ((graph.nodes[source].kind == UnifiedNodeKind::Track &&
                     static_cast<int>(graph.nodes[source].unit) != unit) ||
                    (graph.nodes[sink].kind == UnifiedNodeKind::Track &&
                     static_cast<int>(graph.nodes[sink].unit) != unit)) continue;
                using Bans = std::set<std::pair<int, int>>;
                auto pending = std::deque<Bans>{{}};
                auto visited = std::set<Bans>{{}};
                int trials = 0;
                while (!pending.empty() && trials++ < 32) {
                    auto bans = std::move(pending.front()); pending.pop_front();
                    auto path = transaction_path(graph, resources, owner.id, source, sink, unit, params, bans);
                    if (path.empty()) continue;
                    auto candidate = owner;
                    const auto slot = std::find(owner.demand_indices.begin(), owner.demand_indices.end(), demand_index)
                        - owner.demand_indices.begin();
                    candidate.demand_paths[slot] = path;
                    if (owner_paths_valid(graph, candidate)) {
                        double score = 0;
                        for (std::size_t j = 1; j < path.size(); ++j)
                            for (int aid : graph.out_arc_ids[path[j - 1]])
                                if (graph.arcs[aid].v == path[j]) {
                                    score += arc_incremental_cost(graph, resources, owner.id,
                                        graph.arcs[aid], {source}, params);
                                    break;
                                }
                        if (score < best_cost) { best_cost = score; best_path = std::move(path); }
                        break;
                    }
                    for (std::size_t j = 1; j < path.size(); ++j) {
                        auto next = bans; next.insert(std::minmax(path[j - 1], path[j]));
                        if (visited.insert(next).second) pending.push_back(std::move(next));
                    }
                }
            }
        }
        if (best_path.empty()) return false;
        const auto slot = std::find(owner.demand_indices.begin(), owner.demand_indices.end(), demand_index)
            - owner.demand_indices.begin();
        owner.demand_paths[slot] = std::move(best_path);
        const auto keys = path_resource_keys(graph, owner.demand_paths[slot], owner.is_bnet);
        resources.claim(owner.id, keys);
        owner.claimed.insert(keys.begin(), keys.end());
        for (int node : owner.demand_paths[slot]) add_tree_node(owner, graph, node);
    }
    return true;
}

} // namespace PR_tool::rrr_detail
