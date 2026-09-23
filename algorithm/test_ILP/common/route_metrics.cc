#include "common/route_metrics.hh"

#include <map>
#include <set>

namespace PR_tool {

auto is_wirelength_resource_node(const UnifiedGraph& graph, int node) -> bool {
    if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size()) return false;
    const auto kind = graph.nodes[static_cast<std::size_t>(node)].kind;
    return kind == UnifiedNodeKind::Track || kind == UnifiedNodeKind::Bump;
}

auto net_wirelength(const UnifiedGraph& graph,
                    const std::Vector<const SourceSinkPairPath*>& paths)
    -> std::size_t {
    auto used = std::set<int>{};
    for (const auto* path : paths)
        for (int node : path->node_path)
            if (is_wirelength_resource_node(graph, node)) used.insert(node);
    return used.size();
}

auto total_wirelength(const UnifiedGraph& graph, const RoutingResult& result)
    -> std::size_t {
    auto used = std::map<std::size_t, std::set<int>>{};
    for (const auto& path : result.paths) {
        for (int node : path.node_path) {
            if (is_wirelength_resource_node(graph, node)) used[path.net_id].insert(node);
        }
    }
    std::size_t total = 0;
    for (const auto& [_, nodes] : used) total += nodes.size();
    return total;
}

} // namespace PR_tool
