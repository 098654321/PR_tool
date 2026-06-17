#include "mcf/mcf_simple_tree_refine.hh"

#include "global/debug/debug.hh"

#include <format>
#include <map>
#include <queue>
#include <set>

namespace PR_tool {

namespace {

struct TreeUnionGraph {
    std::set<std::pair<int, int>> edges;
    std::map<int, std::Vector<int>> adj;
    std::map<int, int> degree;
};

auto is_physical_node(const McfGlobalGraph& graph, const int node) -> bool {
    if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size()) {
        return false;
    }
    return !graph.nodes[static_cast<std::size_t>(node)].is_virtual;
}

auto undirected_edge_key(const int u, const int v) -> std::pair<int, int> {
    if (u <= v) {
        return {u, v};
    }
    return {v, u};
}

auto build_tree_union(
    const McfGlobalGraph& graph,
    const std::Vector<McfPathInfo>& child_paths
) -> TreeUnionGraph {
    auto out = TreeUnionGraph {};
    for (const auto& info : child_paths) {
        for (const auto& path : info.unit_paths) {
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                const auto u = path[i];
                const auto v = path[i + 1];
                if (!is_physical_node(graph, u) || !is_physical_node(graph, v)) {
                    continue;
                }
                out.edges.insert(undirected_edge_key(u, v));
            }
        }
    }
    for (const auto& edge : out.edges) {
        const auto [u, v] = edge;
        out.adj[u].push_back(v);
        out.adj[v].push_back(u);
        out.degree[u] += 1;
        out.degree[v] += 1;
    }
    return out;
}

auto prune_leaf_nodes(
    const std::set<int>& terminals,
    TreeUnionGraph& tree
) -> void {
    bool changed = true;
    while (changed) {
        changed = false;
        auto to_remove = std::Vector<std::pair<int, int>> {};
        for (const auto& edge : tree.edges) {
            const auto [u, v] = edge;
            if (terminals.contains(u) || terminals.contains(v)) {
                continue;
            }
            if (tree.degree[u] <= 1 || tree.degree[v] <= 1) {
                to_remove.push_back(edge);
            }
        }
        for (const auto& edge : to_remove) {
            if (!tree.edges.contains(edge)) {
                continue;
            }
            const auto [u, v] = edge;
            tree.edges.erase(edge);
            tree.degree[u] -= 1;
            tree.degree[v] -= 1;
            changed = true;
        }
    }
    tree.adj.clear();
    for (const auto& edge : tree.edges) {
        const auto [u, v] = edge;
        tree.adj[u].push_back(v);
        tree.adj[v].push_back(u);
    }
}

auto connected_physical_nodes(const TreeUnionGraph& tree) -> std::set<int> {
    auto nodes = std::set<int> {};
    for (const auto& edge : tree.edges) {
        nodes.insert(edge.first);
        nodes.insert(edge.second);
    }
    return nodes;
}

auto decompose_tree_to_segments(
    const TreeUnionGraph& tree,
    const std::set<int>& important_nodes
) -> std::pair<std::Vector<std::Vector<int>>, std::size_t> {
    auto segment_paths = std::Vector<std::Vector<int>> {};
    auto seen_segment = std::set<std::pair<int, int>> {};
    auto visited_edges = std::set<std::pair<int, int>> {};

    for (const auto start : important_nodes) {
        const auto adj_it = tree.adj.find(start);
        if (adj_it == tree.adj.end()) {
            continue;
        }
        for (const auto first_next : adj_it->second) {
            const auto edge_key = undirected_edge_key(start, first_next);
            if (visited_edges.contains(edge_key)) {
                continue;
            }
            visited_edges.insert(edge_key);

            auto path = std::Vector<int> {start, first_next};
            auto cur = first_next;
            auto prev = start;
            while (!important_nodes.contains(cur)) {
                const auto cur_adj_it = tree.adj.find(cur);
                if (cur_adj_it == tree.adj.end() || cur_adj_it->second.size() != 2) {
                    break;
                }
                auto next = -1;
                for (const auto cand : cur_adj_it->second) {
                    if (cand != prev) {
                        next = cand;
                        break;
                    }
                }
                if (next < 0) {
                    break;
                }
                const auto walk_edge = undirected_edge_key(cur, next);
                visited_edges.insert(walk_edge);
                path.push_back(next);
                prev = cur;
                cur = next;
            }
            if (path.size() < 2) {
                continue;
            }
            const auto seg_key = undirected_edge_key(path.front(), path.back());
            if (seen_segment.contains(seg_key)) {
                continue;
            }
            seen_segment.insert(seg_key);
            segment_paths.push_back(std::move(path));
        }
    }

    return {segment_paths, visited_edges.size()};
}

} // namespace

auto compress_origin_paths_to_segments(
    const McfGlobalGraph& graph,
    const std::String& origin_key,
    const std::Vector<McfPathInfo>& child_paths,
    const std::set<int>& physical_endpoints
) -> OriginTreeCompressResult {
    auto out = OriginTreeCompressResult {};
    out.origin_key = origin_key;
    for (const auto& info : child_paths) {
        out.child_record_indices.push_back(info.record_id);
        out.child_record_id_list.push_back(info.record_id);
    }

    if (child_paths.empty() || physical_endpoints.empty()) {
        out.fallback = true;
        return out;
    }

    auto tree = build_tree_union(graph, child_paths);
    prune_leaf_nodes(physical_endpoints, tree);

    const auto nodes = connected_physical_nodes(tree);
    for (const auto endpoint : physical_endpoints) {
        if (!nodes.contains(endpoint)) {
            debug::warning_fmt(
                "tree-compress: origin={} missing terminal node in path union; fallback",
                origin_key);
            out.fallback = true;
            return out;
        }
    }

    if (nodes.size() < physical_endpoints.size()) {
        out.fallback = true;
        return out;
    }

    auto important = std::set<int> {physical_endpoints.begin(), physical_endpoints.end()};
    for (const auto node : nodes) {
        const auto deg_it = tree.degree.find(node);
        const auto deg = deg_it != tree.degree.end() ? deg_it->second : 0;
        if (deg >= 3) {
            important.insert(node);
        }
    }

    const auto [segment_paths, visited_edge_count] = decompose_tree_to_segments(tree, important);
    if (segment_paths.empty()) {
        out.fallback = true;
        return out;
    }
    if (visited_edge_count != tree.edges.size()) {
        debug::warning_fmt(
            "tree-compress: origin={} segment decomposition covered {}/{} tree edges; fallback",
            origin_key,
            visited_edge_count,
            tree.edges.size());
        out.fallback = true;
        return out;
    }

    const auto cob_unit = child_paths.front().cob_unit;
    std::size_t accepted_segments = 0;
    for (const auto& path : segment_paths) {
        SimpleTreeSegment seg {};
        seg.src = path.front();
        seg.snk = path.back();
        seg.guide_path = path;
        seg.bbox = segment_bbox_from_guide_path(graph, path, cob_unit);
        seg.parent_origin_key = origin_key;
        seg.parent_record_indices = out.child_record_id_list;
        if (!seg.bbox.restricted) {
            debug::warning_fmt(
                "tree-compress: origin={} segment {}->{} bbox empty; skip segment",
                origin_key,
                seg.src,
                seg.snk);
            continue;
        }
        if (!guide_path_bbox_connected(graph, path, cob_unit, seg.bbox)
            && !commodity_bbox_connected(graph, seg.src, seg.snk, cob_unit, seg.bbox)) {
            debug::warning_fmt(
                "tree-compress: origin={} segment {}->{} bbox disconnected; skip segment",
                origin_key,
                seg.src,
                seg.snk);
            continue;
        }
        out.segments.push_back(std::move(seg));
        ++accepted_segments;
    }
    if (accepted_segments == 0 || accepted_segments != segment_paths.size()) {
        if (accepted_segments > 0 && accepted_segments != segment_paths.size()) {
            debug::warning_fmt(
                "tree-compress: origin={} only {}/{} segments valid; fallback",
                origin_key,
                accepted_segments,
                segment_paths.size());
        }
        out.segments.clear();
        out.fallback = true;
    }
    return out;
}

auto compress_unit_multi_fanout_origins(
    const McfGlobalGraph& graph,
    const std::Vector<McfPathInfo>& stage1_paths,
    const std::Vector<Net_cost_record>& records,
    const std::size_t unit_c,
    const std::function<std::String(const Net_cost_record&)>& origin_group_key_fn
) -> UnitTreeCompressSummary {
    (void)unit_c;
    auto summary = UnitTreeCompressSummary {};
    auto by_origin = std::map<std::String, std::Vector<McfPathInfo>> {};
    auto endpoints_by_origin = std::map<std::String, std::set<int>> {};

    for (const auto& path_info : stage1_paths) {
        if (path_info.record_id >= records.size()) {
            continue;
        }
        const auto& record = records[path_info.record_id];
        const auto key = origin_group_key_fn(record);
        by_origin[key].push_back(path_info);
        for (const auto n : {path_info.src, path_info.snk}) {
            if (is_physical_node(graph, n)) {
                endpoints_by_origin[key].insert(n);
            }
        }
    }

    for (auto& [origin_key, paths] : by_origin) {
        if (paths.size() <= 1) {
            continue;
        }
        bool is_multi = false;
        for (const auto& p : paths) {
            if (p.record_id < records.size()) {
                const auto& rec = records[p.record_id];
                if (rec.from_track_to_bumps_split || rec.type == Net_type::PNnet) {
                    is_multi = true;
                    break;
                }
            }
        }
        if (!is_multi) {
            continue;
        }
        const auto& endpoints = endpoints_by_origin[origin_key];
        auto result = compress_origin_paths_to_segments(graph, origin_key, paths, endpoints);
        if (result.fallback) {
            summary.fallback_origin_count += 1;
        }
        else {
            summary.refined_segment_count += result.segments.size();
        }
        summary.per_origin.push_back(std::move(result));
    }
    return summary;
}

auto merge_refined_paths_for_output(
    const McfGlobalGraph& graph,
    const std::Vector<McfPathInfo>& stage1_paths,
    const std::Vector<McfPathInfo>& refine_paths,
    const UnitTreeCompressSummary& compress_summary,
    const std::Vector<Net_cost_record>& records
) -> std::Vector<McfPathInfo> {
    auto refined_origin_keys = std::set<std::String> {};
    for (const auto& origin : compress_summary.per_origin) {
        if (!origin.fallback) {
            refined_origin_keys.insert(origin.origin_key);
        }
    }

    auto origin_key_for_record = [&](const std::size_t record_id) -> std::String {
        if (record_id >= records.size()) {
            return {};
        }
        const auto& rec = records[record_id];
        return rec.origin_uid.empty() ? rec.origin_key : rec.origin_uid;
    };

    auto undirected_adj = std::map<int, std::Vector<int>> {};
    for (const auto& info : refine_paths) {
        if (info.record_id < kMcfSyntheticRecordIdBase) {
            continue;
        }
        for (const auto& path : info.unit_paths) {
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                const auto u = path[i];
                const auto v = path[i + 1];
                undirected_adj[u].push_back(v);
                undirected_adj[v].push_back(u);
            }
        }
    }

    auto extract_child_path = [&](const int src, const int snk) -> std::Vector<int> {
        auto prev = std::Vector<int>(graph.nodes.size(), -1);
        std::queue<int> q;
        q.push(src);
        prev[static_cast<std::size_t>(src)] = src;
        while (!q.empty() && prev[static_cast<std::size_t>(snk)] < 0) {
            const auto u = q.front();
            q.pop();
            const auto it = undirected_adj.find(u);
            if (it == undirected_adj.end()) {
                continue;
            }
            for (const auto v : it->second) {
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
        for (auto cur = snk; cur != src; cur = prev[static_cast<std::size_t>(cur)]) {
            nodes.push_back(cur);
        }
        nodes.push_back(src);
        std::reverse(nodes.begin(), nodes.end());
        return nodes;
    };

    auto out = std::Vector<McfPathInfo> {};
    for (const auto& info : refine_paths) {
        if (info.record_id < kMcfSyntheticRecordIdBase) {
            const auto key = origin_key_for_record(info.record_id);
            if (!refined_origin_keys.contains(key) && !refined_origin_keys.contains(info.origin_name)) {
                out.push_back(info);
            }
        }
    }

    for (const auto& stage1 : stage1_paths) {
        const auto key = origin_key_for_record(stage1.record_id);
        if (!refined_origin_keys.contains(key) && !refined_origin_keys.contains(stage1.origin_name)) {
            continue;
        }
        auto merged = stage1;
        const auto extracted = extract_child_path(stage1.src, stage1.snk);
        if (!extracted.empty()) {
            merged.unit_paths = {extracted};
        }
        else if (!refined_origin_keys.empty()) {
            debug::warning_fmt(
                "tree-refine merge: failed to extract child path record_id={} origin={}",
                stage1.record_id,
                stage1.origin_name);
        }
        out.push_back(std::move(merged));
    }
    return out;
}

} // namespace PR_tool
