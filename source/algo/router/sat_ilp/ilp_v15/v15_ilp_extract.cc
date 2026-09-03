#include "ilp_v15/v15_ilp_extract.hh"

#include "sat/routing_path_log.hh"

#include <algorithm>
#include <format>
#include <map>
#include <queue>
#include <set>

namespace PR_tool {

namespace {

auto net_by_id(const std::Vector<RoutingNet>& nets, std::size_t net_id) -> const RoutingNet* {
    const auto it = std::find_if(
        nets.begin(),
        nets.end(),
        [&](const RoutingNet& net) { return net.net_id == net_id; });
    return it == nets.end() ? nullptr : &*it;
}

auto parent_solution_by_id(
    const V15IlpModelResult& result,
    std::size_t parent_id
) -> const V15ParentModelSolution* {
    const auto it = std::find_if(
        result.parents.begin(),
        result.parents.end(),
        [&](const V15ParentModelSolution& solution) {
            return solution.parent_id == parent_id;
        });
    return it == result.parents.end() ? nullptr : &*it;
}

} // namespace

auto extract_v15_routing_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const V15PrepareResult& prepared,
    const V15IlpModelResult& model_result,
    const SatRoutingResult& sat_result,
    const std::set<std::size_t>& selected_net_ids
) -> SatRoutingResult {
    auto out = sat_result;
    out.paths.clear();
    for (const auto& path : sat_result.paths) {
        if (!selected_net_ids.contains(path.net_id)) {
            out.paths.push_back(path);
        }
    }
    if (!model_result.ok) {
        out.ok = false;
        out.message = model_result.message;
        return out;
    }

    for (const auto& parent : prepared.parents) {
        const auto* solution = parent_solution_by_id(model_result, parent.parent_id);
        const auto* net = net_by_id(nets, parent.routing_net_id);
        if (solution == nullptr || net == nullptr) {
            out.ok = false;
            out.message = std::format(
                "v15 extraction is missing parent {} or net {}",
                parent.parent_id,
                parent.routing_net_id);
            return out;
        }

        auto parent_arc = std::map<int, int> {};
        auto visited = std::set<int> {parent.root_node};
        auto queue = std::queue<int> {};
        queue.push(parent.root_node);
        while (!queue.empty()) {
            const int node = queue.front();
            queue.pop();
            if (node < 0 || static_cast<std::size_t>(node) >= graph.out_arc_ids.size()) {
                continue;
            }
            for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(node)]) {
                if (!solution->selected_arc_ids.contains(arc_id)) {
                    continue;
                }
                const int next = graph.arcs[static_cast<std::size_t>(arc_id)].v;
                if (visited.insert(next).second) {
                    parent_arc.emplace(next, arc_id);
                    queue.push(next);
                }
            }
        }

        for (std::size_t sink_index = 0; sink_index < parent.origin_sinks.size(); ++sink_index) {
            const int sink = parent.origin_sinks[sink_index];
            auto reversed = std::Vector<int> {sink};
            int current = sink;
            while (current != parent.root_node) {
                const auto parent_it = parent_arc.find(current);
                if (parent_it == parent_arc.end()) {
                    out.ok = false;
                    out.message = std::format(
                        "v15 parent {} cannot reach sink {}",
                        parent.parent_id,
                        sink);
                    return out;
                }
                current = graph.arcs[static_cast<std::size_t>(parent_it->second)].u;
                reversed.push_back(current);
            }
            std::reverse(reversed.begin(), reversed.end());

            if (net->kind == RoutingNetKind::PNnet) {
                if (reversed.size() >= 2
                    && reversed[0] == parent.root_node
                    && graph.nodes[static_cast<std::size_t>(reversed[1])].kind
                        == UnifiedNodeKind::Track) {
                    const int physical_track = reversed[1];
                    reversed.erase(reversed.begin());
                    out.paths.push_back(SourceSinkPairPath {
                        parent.routing_net_id,
                        parent.source_indices[sink_index],
                        parent.demand_ids[sink_index],
                        physical_track,
                        reversed});
                    continue;
                }
            }

            out.paths.push_back(SourceSinkPairPath {
                parent.routing_net_id,
                parent.source_indices[sink_index],
                parent.demand_ids[sink_index],
                -1,
                reversed});
        }
    }

    out.ok = true;
    out.message.clear();
    out.total_wirelength = 0;
    for (const auto& net : nets) {
        auto net_paths = std::Vector<const SourceSinkPairPath*> {};
        for (const auto& path : out.paths) {
            if (path.net_id == net.net_id) {
                net_paths.push_back(&path);
            }
        }
        out.total_wirelength += net_wirelength(graph, net_paths);
    }

    out.used_tob_switch_ids.clear();
    for (const auto& path : out.paths) {
        for (std::size_t i = 1; i < path.node_path.size(); ++i) {
            const int u = path.node_path[i - 1];
            const int v = path.node_path[i];
            for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                if (arc.v != v || arc.physical_switch_id < 0) {
                    continue;
                }
                out.used_tob_switch_ids.push_back(arc.physical_switch_id);
                break;
            }
        }
    }
    std::sort(out.used_tob_switch_ids.begin(), out.used_tob_switch_ids.end());
    out.used_tob_switch_ids.erase(
        std::unique(out.used_tob_switch_ids.begin(), out.used_tob_switch_ids.end()),
        out.used_tob_switch_ids.end());

    for (const auto& [group, straight] : model_result.mode_straight) {
        out.vline_mode_straight_by_group[static_cast<std::size_t>(group)] = straight;
    }
    return out;
}

} // namespace PR_tool
