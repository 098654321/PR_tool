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

auto solution_by_id(
    const V15IlpModelResult& result,
    std::size_t commodity_id
) -> const V15CommodityModelSolution* {
    const auto it = std::find_if(
        result.commodities.begin(),
        result.commodities.end(),
        [&](const V15CommodityModelSolution& solution) {
            return solution.commodity_id == commodity_id;
        });
    return it == result.commodities.end() ? nullptr : &*it;
}

auto find_arc_id(const UnifiedGraph& graph, int u, int v) -> int {
    if (u < 0 || static_cast<std::size_t>(u) >= graph.out_arc_ids.size()) {
        return -1;
    }
    for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
        if (graph.arcs[static_cast<std::size_t>(arc_id)].v == v) {
            return arc_id;
        }
    }
    return -1;
}

} // namespace

auto extract_v15_routing_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<IlpCommodity>& commodities,
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

    for (const auto& commodity : commodities) {
        const auto* solution = solution_by_id(model_result, commodity.commodity_id);
        const auto* net = net_by_id(nets, commodity.routing_net_id);
        if (solution == nullptr || net == nullptr) {
            out.ok = false;
            out.message = std::format(
                "v15 extraction is missing commodity {} or net {}",
                commodity.commodity_id,
                commodity.routing_net_id);
            return out;
        }

        auto parent_arc = std::map<int, int> {};
        auto visited = std::set<int> {commodity.source_node};
        auto queue = std::queue<int> {};
        queue.push(commodity.source_node);
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

        for (std::size_t sink_index = 0; sink_index < commodity.sink_nodes.size(); ++sink_index) {
            const int sink = commodity.sink_nodes[sink_index];
            auto reversed = std::Vector<int> {sink};
            int current = sink;
            while (current != commodity.source_node) {
                const auto parent_it = parent_arc.find(current);
                if (parent_it == parent_arc.end()) {
                    out.ok = false;
                    out.message = std::format(
                        "v15 commodity {} cannot reach sink {}",
                        commodity.commodity_id,
                        sink);
                    return out;
                }
                current = graph.arcs[static_cast<std::size_t>(parent_it->second)].u;
                reversed.push_back(current);
            }
            std::reverse(reversed.begin(), reversed.end());
            int physical_source = -1;
            if (net->kind == RoutingNetKind::PNnet) {
                if (reversed.size() < 2 || reversed.front() != net->virtual_source_node) {
                    out.ok = false;
                    out.message = std::format(
                        "v15 PNnet {} path is missing its virtual root",
                        net->net_id);
                    return out;
                }
                physical_source = reversed[1];
                reversed.erase(reversed.begin());
            }
            out.paths.push_back(SourceSinkPairPath {
                commodity.routing_net_id,
                commodity.source_indices.at(sink_index),
                commodity.demand_ids.at(sink_index),
                physical_source,
                std::move(reversed)});
        }
    }

    std::sort(out.paths.begin(), out.paths.end(), [](const SourceSinkPairPath& lhs, const SourceSinkPairPath& rhs) {
        if (lhs.net_id != rhs.net_id) {
            return lhs.net_id < rhs.net_id;
        }
        if (lhs.demand_id != rhs.demand_id) {
            return lhs.demand_id < rhs.demand_id;
        }
        return lhs.source_index < rhs.source_index;
    });

    auto used_switches = std::set<int> {};
    auto modes = sat_result.vline_mode_straight_by_group;
    for (const auto& path : out.paths) {
        for (std::size_t i = 1; i < path.node_path.size(); ++i) {
            const int arc_id = find_arc_id(graph, path.node_path[i - 1], path.node_path[i]);
            if (arc_id < 0) {
                continue;
            }
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (arc.physical_switch_id >= 0) {
                used_switches.insert(arc.physical_switch_id);
            }
            if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack
                && arc.mode_group_id >= 0) {
                if (arc.is_vline_track_straight) {
                    modes[static_cast<std::size_t>(arc.mode_group_id)] = true;
                }
                if (arc.is_vline_track_swap) {
                    modes[static_cast<std::size_t>(arc.mode_group_id)] = false;
                }
            }
        }
    }
    out.used_tob_switch_ids.assign(used_switches.begin(), used_switches.end());
    out.vline_mode_straight_by_group = std::move(modes);
    out.total_wirelength = total_wirelength(graph, out);
    out.ok = true;
    out.message = model_result.status == V15IlpStatus::Optimal
        ? "SAT+ILP_OPTIMAL"
        : "SAT+ILP_SUBOPTIMAL";
    return out;
}

} // namespace PR_tool
