#include "mcf/mcf_conflict_graph.hh"

#include <algorithm>
#include <format>
#include <map>

namespace PR_tool {

namespace {

auto resources_overlap(const McfCandidateResources& a, const McfCandidateResources& b) -> bool {
    for (const auto& e : a.physical_edges) {
        if (b.physical_edges.contains(e)) {
            return true;
        }
    }
    for (const auto n : a.physical_nodes) {
        if (b.physical_nodes.contains(n)) {
            return true;
        }
    }
    return false;
}

auto make_component_summary(
    const std::Vector<int>& vertex_ids,
    const std::Vector<std::String>& vertex_labels
) -> std::String {
    auto parts = std::Vector<std::String> {};
    parts.reserve(vertex_ids.size());
    for (const auto vid : vertex_ids) {
        if (vid >= 0 && static_cast<std::size_t>(vid) < vertex_labels.size()) {
            parts.push_back(vertex_labels[static_cast<std::size_t>(vid)]);
        }
    }
    if (parts.empty()) {
        return std::String {};
    }
    auto out = parts.front();
    for (std::size_t i = 1; i < parts.size(); ++i) {
        out += std::format(",{}", parts[i]);
    }
    return out;
}

} // namespace

auto build_edge_node_conflict_components(
    const std::Vector<McfCandidateResources>& per_vertex,
    const std::Vector<std::String>& vertex_labels
) -> std::Vector<McfConflictComponent> {
    const auto n = static_cast<int>(per_vertex.size());
    if (n == 0) {
        return {};
    }
    if (n == 1) {
        return {McfConflictComponent {{0}, make_component_summary({0}, vertex_labels)}};
    }

    auto parent = std::Vector<int>(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        parent[static_cast<std::size_t>(i)] = i;
    }
    auto find = [&](int x) {
        while (parent[static_cast<std::size_t>(x)] != x) {
            parent[static_cast<std::size_t>(x)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(x)])];
            x = parent[static_cast<std::size_t>(x)];
        }
        return x;
    };
    auto unite = [&](int a, int b) {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent[static_cast<std::size_t>(b)] = a;
        }
    };

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (resources_overlap(per_vertex[static_cast<std::size_t>(i)], per_vertex[static_cast<std::size_t>(j)])) {
                unite(i, j);
            }
        }
    }

    auto groups = std::map<int, std::Vector<int>> {};
    for (int i = 0; i < n; ++i) {
        groups[find(i)].push_back(i);
    }

    auto out = std::Vector<McfConflictComponent> {};
    out.reserve(groups.size());
    for (auto& [_, ids] : groups) {
        std::sort(ids.begin(), ids.end());
        out.push_back(McfConflictComponent {ids, make_component_summary(ids, vertex_labels)});
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
        if (a.vertex_ids.empty()) {
            return false;
        }
        if (b.vertex_ids.empty()) {
            return true;
        }
        return a.vertex_ids.front() < b.vertex_ids.front();
    });
    return out;
}

} // namespace PR_tool
