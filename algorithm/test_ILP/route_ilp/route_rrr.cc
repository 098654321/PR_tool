#include "route_ilp/route_rrr.hh"

#include "common/route_metrics.hh"
#include "direct_ilp/direct_validate.hh"
#include "rrr_air/rrr_router.hh"

#include <debug/debug.hh>
#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace PR_tool {
namespace {
namespace air = air316;

struct GraphBridge {
    air::UnifiedGraph graph;
    std::Vector<int> to_air, from_air;
};

auto bump_ref(const Bump_coord& value) -> air::Bump_coord {
    return {value.TOB, value.Bank, value.Group, value.Index};
}

auto graph_bridge(const UnifiedGraph& source) -> GraphBridge {
    auto out = GraphBridge{};
    out.graph.rows = source.rows; out.graph.cols = source.cols;
    out.to_air.resize(source.nodes.size(), -1);
    for (std::size_t i = 0; i < source.nodes.size(); ++i) {
        const auto& n = source.nodes[i];
        if (n.kind == UnifiedNodeKind::VirtualSource) continue;
        out.to_air[i] = static_cast<int>(out.graph.nodes.size());
        out.from_air.push_back(static_cast<int>(i));
        out.graph.nodes.push_back({static_cast<air::UnifiedNodeKind>(n.kind),
            n.unit, n.track_dir, n.track_row, n.track_col, n.track_index,
            bump_ref(n.bump), n.tob, n.bank, n.group, n.line_index});
    }
    auto& graph = out.graph;
    graph.in_arc_ids.resize(graph.nodes.size());
    graph.out_arc_ids.resize(graph.nodes.size());
    graph.ordered_track_out_arc_ids.resize(graph.nodes.size());
    for (const auto& arc : source.arcs) {
        if (arc.is_virtual_source_arc) continue;
        const int u = out.to_air.at(arc.u), v = out.to_air.at(arc.v);
        if (u < 0 || v < 0) throw std::logic_error("AIR physical arc touches virtual root");
        const int aid = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back({u, v, arc.is_vline_track_straight,
            arc.is_vline_track_swap, arc.mode_group_id, arc.physical_switch_id,
            static_cast<air::PhysicalSwitchKind>(arc.physical_switch_kind)});
        graph.out_arc_ids[u].push_back(aid); graph.in_arc_ids[v].push_back(aid);
        graph.directed_arc_set.insert({u,v});
    }
    for (const auto& [key, id] : source.track_node_by_key)
        graph.track_node_by_key.emplace(key, out.to_air.at(id));
    for (const auto& [key, id] : source.bump_node_by_key)
        graph.bump_node_by_key.emplace(bump_ref(key), out.to_air.at(id));
    for (const auto& [key, id] : source.hline_node_by_key)
        graph.hline_node_by_key.emplace(key, out.to_air.at(id));
    for (const auto& [key, id] : source.vline_node_by_key)
        graph.vline_node_by_key.emplace(key, out.to_air.at(id));
    graph.track_node_count = source.track_node_count;
    graph.tob_node_count = source.tob_node_count;
    return out;
}

auto node_ref(const GraphNodeRef& r) -> air::GraphNodeRef {
    return {static_cast<air::GraphNodeRef::Kind>(r.kind), r.track_coord,
        r.track_index, bump_ref(r.bump), r.tob, r.bank, r.group, r.line_index};
}

auto routing_nets(const std::Vector<RoutingNet>& source,
                  const std::Vector<std::size_t>& indices) -> std::Vector<air::RoutingNet> {
    auto out = std::Vector<air::RoutingNet>{};
    for (const auto index : indices) {
        const auto& net = source.at(index);
        auto n = air::RoutingNet{};
        n.net_id = net.net_id; n.name = net.name;
        n.kind = static_cast<air::RoutingNetKind>(net.kind);
        n.is_sync_bus = net.is_sync_bus;
        for (const auto& ref : net.sources) n.sources.push_back(node_ref(ref));
        for (const auto& d : net.demands)
            n.demands.push_back({d.demand_id, node_ref(d.sink), d.candidate_source_indices, d.fixed_pair});
        out.push_back(std::move(n));
    }
    return out;
}

} // namespace

auto rrr_mode0_indices(const std::Vector<std::Rc<circuit::Net>>& ilp_raw_nets,
                      const std::Vector<std::Rc<circuit::Net>>& mode0_raw_nets)
    -> std::Vector<std::size_t> {
    if (ilp_raw_nets.size() != mode0_raw_nets.size())
        throw std::logic_error("AIR mode-0/ILP net count mismatch");
    auto indices = std::Vector<std::size_t>{};
    auto seen = std::set<std::size_t>{};
    for (const auto& net : mode0_raw_nets) {
        const auto it = std::find(ilp_raw_nets.begin(), ilp_raw_nets.end(), net);
        const auto index = static_cast<std::size_t>(it-ilp_raw_nets.begin());
        if (it == ilp_raw_nets.end() || !seen.insert(index).second)
            throw std::logic_error("AIR mode-0/ILP net identity mismatch");
        indices.push_back(index);
    }
    return indices;
}

auto route_bus_lengths(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                       const RoutingResult& route) -> std::map<std::size_t, std::size_t> {
    auto lengths = std::map<std::size_t, std::size_t>{};
    for (const auto& path : route.paths) {
        const auto& net = net_for_owner(nets, {path.net_id, path.demand_id});
        if (!net.is_sync_bus) continue;
        std::size_t length = 0;
        for (int id : path.node_path) if (is_wirelength_resource_node(graph, id)) ++length;
        const auto [it, inserted] = lengths.emplace(net.net_id, length);
        if (!inserted && it->second != length) throw std::logic_error("AIR returned unequal bus lanes");
    }
    return lengths;
}

auto optimize_route_columns_rrr(const UnifiedGraph& graph,
                                const std::Vector<RoutingNet>& nets,
                                const std::Vector<RoutingScope>& scopes,
                                const RouteIlpResult& baseline,
                                hardware::Interposer* interposer,
                                int verbose_level,
                                std::chrono::steady_clock::time_point deadline,
                                const std::Vector<std::Rc<circuit::Net>>* raw_nets,
                                const std::Vector<std::size_t>& raw_to_ilp)
    -> RoutingResult {
    const auto begin = std::chrono::steady_clock::now();
    if (!baseline.has_integer_solution || !validate_partial_route(
        graph, nets, scopes, baseline.route, baseline.bus_lengths))
        throw std::logic_error("AIR input is not a legal ILP partial route");
    auto expected_missing = std::set<RouteOwner>{};
    for (const auto owner : route_owners(nets)) {
        const auto& net = net_for_owner(nets, owner);
        if (std::ranges::none_of(baseline.route.paths, [&](const auto& p) {
            return p.net_id == owner.net_id && (!net.is_sync_bus || p.demand_id == owner.demand_id);
        })) expected_missing.insert(owner);
    }
    if (expected_missing != baseline.missing) throw std::logic_error("AIR input slack/path mismatch");
    const auto bridge = graph_bridge(graph);
    auto indices = raw_to_ilp;
    if (indices.empty()) {
        indices.resize(nets.size());
        std::iota(indices.begin(), indices.end(), 0);
    }
    if (indices.size() != nets.size()) throw std::logic_error("AIR net permutation size mismatch");
    const auto air_nets = routing_nets(nets, indices);
    auto initial = std::Vector<std::Vector<std::Vector<int>>>(nets.size());
    for (std::size_t ni = 0; ni < nets.size(); ++ni) {
        const auto& net = nets[indices[ni]];
        initial[ni].resize(net.demands.size());
        for (const auto& path : baseline.route.paths) if (path.net_id == net.net_id) {
            auto& mapped = initial[ni].at(path.demand_id);
            for (int id : path.node_path) {
                const int mapped_id = bridge.to_air.at(id);
                if (mapped_id < 0) throw std::logic_error("AIR input contains a virtual node");
                mapped.push_back(mapped_id);
            }
        }
    }
    auto params = air::RrrParams{};
    params.budget_start = begin; params.deadline = deadline;
    params.time_budget_seconds = std::max(0.0, std::chrono::duration<double>(deadline-begin).count());
    debug::info_fmt("route RRR start: source=316e477 owners={} missing={} wirelength={} budget_ms={}",
        route_owners(nets).size(), baseline.missing.size(), baseline.route.total_wirelength,
        params.time_budget_seconds * 1000);
    const auto result = air::run_rrr(bridge.graph, air_nets, params, interposer,
                                    verbose_level, raw_nets, initial);
    auto out = RoutingResult{};
    out.ok = result.status == "success"; out.message = result.stop_reason;
    for (std::size_t ni = 0; ni < nets.size(); ++ni)
        for (std::size_t di = 0; di < result.paths[ni].size(); ++di) {
            const auto& path = result.paths[ni][di];
            if (path.empty()) continue;
            auto physical = std::Vector<int>{};
            for (int id : path) physical.push_back(bridge.from_air.at(id));
            const auto& net = nets[indices[ni]];
            const auto& demand = net.demands[di];
            const auto source = std::find_if(demand.candidate_source_indices.begin(),
                demand.candidate_source_indices.end(), [&](auto index) {
                    return resolve_graph_node(graph, net.sources[index]) == physical.front();
                });
            if (source == demand.candidate_source_indices.end()) throw std::logic_error("AIR source is not a candidate");
            out.paths.push_back({net.net_id, *source, demand.demand_id,
                net.kind == RoutingNetKind::PNnet ? physical.front() : -1, physical});
            for (std::size_t j = 1; j < physical.size(); ++j)
                for (int aid : graph.out_arc_ids[physical[j-1]]) {
                    const auto& arc = graph.arcs[aid];
                    if (arc.v != physical[j]) continue;
                    if (arc.physical_switch_id >= 0) out.used_tob_switch_ids.push_back(arc.physical_switch_id);
                    if (arc.mode_group_id >= 0) out.vline_mode_straight_by_group[arc.mode_group_id] = arc.is_vline_track_straight;
                    break;
                }
        }
    std::sort(out.used_tob_switch_ids.begin(), out.used_tob_switch_ids.end());
    const auto unique = std::unique(out.used_tob_switch_ids.begin(), out.used_tob_switch_ids.end());
    out.used_tob_switch_ids.erase(unique, out.used_tob_switch_ids.end());
    out.total_wirelength = total_wirelength(graph, out);
    if (out.total_wirelength != result.total_wirelength) throw std::logic_error("AIR wirelength mapping mismatch");
    out.rrr_attempted = true;
    out.rrr_accepted = out.ok && (!baseline.missing.empty() || out.total_wirelength < baseline.route.total_wirelength);
    out.rrr_status = out.ok ? (out.rrr_accepted ? "COMPLETE_IMPROVED" : "COMPLETE_UNCHANGED") : "TIME_LIMIT";
    out.rrr_baseline_wirelength = baseline.route.total_wirelength;
    out.rrr_wirelength = out.total_wirelength;
    out.rrr_total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-begin).count();
    out.rrr_iterations = result.iterations;
    out.rrr_triggers = result.optimization_rounds;
    debug::info_fmt("route RRR summary: status={} stop_reason={} missing={} iterations={} convergences={} wirelength={}->{} total_ms={}",
        out.rrr_status, result.stop_reason, result.missing_owners, result.iterations,
        result.convergences, baseline.route.total_wirelength, out.total_wirelength, out.rrr_total_ms);
    return out;
}

} // namespace PR_tool
