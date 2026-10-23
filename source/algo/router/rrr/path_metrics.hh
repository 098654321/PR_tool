#pragma once

#include "hardware_graph.hh"
#include <limits>

namespace PR_tool::rrr {

inline auto format_path_node(const UnifiedGraph& graph, int node) -> std::String {
    return format_unified_node(graph, node);
}

inline auto path_wirelength(const UnifiedGraph& graph, const std::Vector<int>& path) -> std::size_t {
    return std::count_if(path.begin(), path.end(), [&](int n) {
        return graph.nodes.at(n).kind == UnifiedNodeKind::Track || graph.nodes.at(n).kind == UnifiedNodeKind::Bump;
    });
}

inline auto net_wirelength(const UnifiedGraph& graph, const std::Vector<std::Vector<int>>& paths) -> std::size_t {
    std::Set<int> nodes;
    for (const auto& path : paths) for (int n : path)
        if (graph.nodes.at(n).kind == UnifiedNodeKind::Track || graph.nodes.at(n).kind == UnifiedNodeKind::Bump)
            nodes.insert(n);
    return nodes.size();
}

inline auto total_wirelength(const UnifiedGraph& graph,
                            const std::Vector<std::Vector<std::Vector<int>>>& paths) -> std::size_t {
    std::size_t length = 0;
    for (const auto& net : paths) length += net_wirelength(graph, net);
    return length;
}

struct LegalConvergence {
    std::size_t best{std::numeric_limits<std::size_t>::max()};
    int streak{0};
    auto observe(std::size_t length) -> bool {
        const bool improved = streak == 0 || length < best;
        if (improved) { best = length; streak = 1; }
        else ++streak;
        return improved;
    }
    auto converged() const -> bool { return streak >= 3; }
};

}
