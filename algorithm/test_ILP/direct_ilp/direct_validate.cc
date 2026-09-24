#include "direct_ilp/direct_validate.hh"

#include "common/route_metrics.hh"

#include <debug/debug.hh>

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <tuple>

namespace PR_tool {
namespace {

using Owner = std::pair<std::size_t, std::size_t>;

auto arc_between(const UnifiedGraph& graph, int u, int v) -> int {
    if (u < 0 || static_cast<std::size_t>(u) >= graph.out_arc_ids.size()) return -1;
    for (int id : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
        if (graph.arcs[static_cast<std::size_t>(id)].v == v) return id;
    }
    return -1;
}

} // namespace

auto validate_route(const UnifiedGraph& graph,
                    const std::Vector<RoutingNet>& nets,
                    const std::Vector<RoutingScope>& scopes,
                    const RoutingResult& route,
                    const std::map<std::size_t, std::size_t>& bus_targets,
                    bool allow_missing) -> bool {
    auto fail = [](const std::String& reason) {
        debug::error_fmt("direct route validation: FAIL {}", reason);
        return false;
    };
    auto net_by_id = std::map<std::size_t, const RoutingNet*>{};
    auto scope_by_id = std::map<std::size_t, const RoutingScope*>{};
    for (const auto& net : nets) net_by_id[net.net_id] = &net;
    for (const auto& scope : scopes) scope_by_id[scope.net_id] = &scope;
    auto counts = std::map<std::pair<std::size_t, std::size_t>, int>{};
    auto owners = std::map<int, Owner>{};
    auto matching = std::map<std::pair<int, int>, int>{};
    auto modes = std::map<int, bool>{};
    auto switches = std::set<int>{};
    auto bus_lengths = std::map<std::size_t, std::set<std::size_t>>{};
    auto owner_edges = std::map<Owner, std::set<std::pair<int, int>>>{};
    for (const auto& path : route.paths) {
        const auto ni = net_by_id.find(path.net_id);
        const auto si = scope_by_id.find(path.net_id);
        if (ni == net_by_id.end() || si == scope_by_id.end()) return fail("unknown net or scope");
        const auto& net = *ni->second;
        const auto& scope = *si->second;
        const auto demand = std::find_if(net.demands.begin(), net.demands.end(),
            [&](const auto& value) { return value.demand_id == path.demand_id; });
        if (demand == net.demands.end() || ++counts[{path.net_id, path.demand_id}] != 1)
            return fail("missing or duplicate demand");
        if (path.source_index >= net.sources.size()
            || std::find(demand->candidate_source_indices.begin(),
                         demand->candidate_source_indices.end(), path.source_index)
                    == demand->candidate_source_indices.end())
            return fail("invalid source choice");
        if (path.node_path.empty()
            || path.node_path.front() != resolve_graph_node(graph, net.sources[path.source_index])
            || path.node_path.back() != resolve_graph_node(graph, demand->sink))
            return fail("endpoint mismatch");
        if (net.kind == RoutingNetKind::PNnet) {
            if (path.physical_source_node != path.node_path.front())
                return fail("PN physical source mismatch");
        } else if (path.physical_source_node >= 0) {
            return fail("unexpected physical source field");
        }
        const Owner owner{path.net_id,
            net.is_sync_bus ? path.demand_id : std::numeric_limits<std::size_t>::max()};
        auto seen = std::set<int>{};
        std::size_t length = 0;
        int path_unit = -1;
        for (std::size_t index = 0; index < path.node_path.size(); ++index) {
            const int node = path.node_path[index];
            if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size()
                || static_cast<std::size_t>(node) >= scope.node_offset.size()
                || scope.node_offset[static_cast<std::size_t>(node)] < 0
                || graph.nodes[static_cast<std::size_t>(node)].kind
                    == UnifiedNodeKind::VirtualSource
                || !seen.insert(node).second)
                return fail("invalid, repeated, or out-of-scope node");
            const auto& data = graph.nodes[static_cast<std::size_t>(node)];
            if (data.kind == UnifiedNodeKind::Bump && index != 0 &&
                index + 1 != path.node_path.size())
                return fail("Bump used as an internal route node");
            if (data.kind == UnifiedNodeKind::Track) {
                if (path_unit >= 0 && path_unit != static_cast<int>(data.unit))
                    return fail("path crosses Track units");
                path_unit = static_cast<int>(data.unit);
            }
            const auto [it, inserted] = owners.emplace(node, owner);
            if (!inserted && it->second != owner) return fail("physical node conflict");
            if (is_wirelength_resource_node(graph, node)) ++length;
        }
        if (net.is_sync_bus) {
            bus_lengths[net.net_id].insert(length);
            const auto target = bus_targets.find(net.net_id);
            if (target != bus_targets.end() && length != target->second)
                return fail("SyncBus differs from current target length");
        }
        for (std::size_t i = 1; i < path.node_path.size(); ++i) {
            owner_edges[owner].insert(std::minmax(path.node_path[i - 1],
                                                  path.node_path[i]));
            const int arc_id = arc_between(graph, path.node_path[i - 1], path.node_path[i]);
            if (arc_id < 0 || static_cast<std::size_t>(arc_id) >= scope.arc_offset.size()
                || scope.arc_offset[static_cast<std::size_t>(arc_id)] < 0)
                return fail("missing or out-of-scope physical arc");
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (arc.is_virtual_source_arc) return fail("virtual arc in physical route");
            if (arc.physical_switch_id < 0) continue;
            switches.insert(arc.physical_switch_id);
            const auto set_matching = [&](int stage, int node) -> bool {
                const auto [it, inserted] = matching.emplace(std::pair{stage, node},
                                                              arc.physical_switch_id);
                return inserted || it->second == arc.physical_switch_id;
            };
            const auto u_kind = graph.nodes[static_cast<std::size_t>(arc.u)].kind;
            const auto v_kind = graph.nodes[static_cast<std::size_t>(arc.v)].kind;
            if (arc.physical_switch_kind == PhysicalSwitchKind::BumpH) {
                const int bump = u_kind == UnifiedNodeKind::Bump ? arc.u : arc.v;
                const int hline = u_kind == UnifiedNodeKind::HLine ? arc.u : arc.v;
                if (!set_matching(0, bump) || !set_matching(1, hline))
                    return fail("TOB switch1 matching conflict");
            } else if (arc.physical_switch_kind == PhysicalSwitchKind::HLineVLine) {
                const int hline = u_kind == UnifiedNodeKind::HLine ? arc.u : arc.v;
                const int vline = v_kind == UnifiedNodeKind::VLine ? arc.v : arc.u;
                if (!set_matching(2, hline) || !set_matching(3, vline))
                    return fail("TOB switch2 matching conflict");
            }
            if (arc.mode_group_id >= 0) {
                const auto [it, inserted] = modes.emplace(
                    arc.mode_group_id, arc.is_vline_track_straight);
                if (!inserted && it->second != arc.is_vline_track_straight)
                    return fail("TOB straight/swap conflict");
            }
        }
    }
    for (const auto& [_, edges] : owner_edges) {
        auto parent = std::map<int, int>{};
        const auto root = [&](auto&& self, int node) -> int {
            auto [it, inserted] = parent.emplace(node, node);
            if (it->second != node) it->second = self(self, it->second);
            return it->second;
        };
        for (const auto& [u, v] : edges) {
            const int a = root(root, u), b = root(root, v);
            if (a == b) return fail("physical route union contains a cycle");
            parent[a] = b;
        }
    }
    for (const auto& net : nets) {
        int present = 0;
        for (const auto& demand : net.demands) {
            const int count = counts[{net.net_id, demand.demand_id}];
            if ((!allow_missing && count != 1) || count > 1)
                return fail("unrouted demand");
            present += count;
        }
        if (allow_missing && !net.is_sync_bus && present != 0 &&
            present != static_cast<int>(net.demands.size()))
            return fail("partially routed non-Sync owner");
    }
    for (const auto& [_, lengths] : bus_lengths)
        if (lengths.size() != 1) return fail("SyncBus Track+Bump length mismatch");
    if (total_wirelength(graph, route) != route.total_wirelength)
        return fail("wirelength metadata mismatch");
    if (!std::equal(switches.begin(), switches.end(),
                    route.used_tob_switch_ids.begin(), route.used_tob_switch_ids.end()))
        return fail("TOB switch metadata mismatch");
    for (const auto& [group, straight] : modes) {
        const auto it = route.vline_mode_straight_by_group.find(
            static_cast<std::size_t>(group));
        if (it == route.vline_mode_straight_by_group.end() || it->second != straight)
            return fail("TOB mode metadata mismatch");
    }
    if (!allow_missing)
        debug::info_fmt("direct route validation: PASS paths={} wirelength={} switches={} modes={}",
                        route.paths.size(), route.total_wirelength,
                        switches.size(), modes.size());
    return true;
}

auto validate_direct_route(const UnifiedGraph& graph,
                           const std::Vector<RoutingNet>& nets,
                           const std::Vector<RoutingScope>& scopes,
                           const RoutingResult& route) -> bool {
    return validate_route(graph, nets, scopes, route, {}, false);
}

auto validate_partial_route(const UnifiedGraph& graph,
                            const std::Vector<RoutingNet>& nets,
                            const std::Vector<RoutingScope>& scopes,
                            const RoutingResult& route,
                            const std::map<std::size_t, std::size_t>& bus_lengths) -> bool {
    return validate_route(graph, nets, scopes, route, bus_lengths, true);
}

} // namespace PR_tool
