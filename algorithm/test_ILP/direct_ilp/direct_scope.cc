#include "direct_ilp/direct_scope.hh"

#include "scope/scope_bbox.hh"

#include <debug/debug.hh>

#include <set>
#include <stdexcept>

namespace PR_tool {
namespace {

auto include_bump_access(const UnifiedGraph& graph, const Bump_coord& bump,
                         std::Vector<bool>& included) -> void {
    const auto bump_it = graph.bump_node_by_key.find(bump);
    if (bump_it == graph.bump_node_by_key.end()) {
        throw std::runtime_error("direct ILP endpoint bump is absent from graph");
    }
    included[static_cast<std::size_t>(bump_it->second)] = true;
    for (std::size_t line = 0; line < 8; ++line) {
        const auto h = graph.hline_node_by_key.find({bump.TOB, bump.Bank, bump.Group, line});
        if (h != graph.hline_node_by_key.end()) included[static_cast<std::size_t>(h->second)] = true;
    }
    for (std::size_t line = bump.Bank * 64; line < (bump.Bank + 1) * 64; ++line) {
        const auto v = graph.vline_node_by_key.find({bump.TOB, line});
        if (v != graph.vline_node_by_key.end()) included[static_cast<std::size_t>(v->second)] = true;
    }
}

} // namespace

auto build_direct_scopes(const UnifiedGraph& graph,
                         const std::Vector<RoutingNet>& nets, int verbose_level)
    -> std::Vector<RoutingScope> {
    auto result = std::Vector<RoutingScope>{};
    result.reserve(nets.size());
    for (const auto& net : nets) {
        const auto box = compute_scope_bbox_for_net(net);
        auto terminal_bumps = std::set<Bump_coord>{};
        for (const auto& demand : net.demands) {
            for (const auto index : demand.candidate_source_indices) {
                if (index >= net.sources.size()) throw std::runtime_error("invalid source index");
                const auto& source = net.sources[index];
                if (source.kind == GraphNodeRef::Kind::Bump)
                    terminal_bumps.insert(source.bump);
            }
            if (demand.sink.kind == GraphNodeRef::Kind::Bump)
                terminal_bumps.insert(demand.sink.bump);
        }
        auto included = std::Vector<bool>(graph.nodes.size(), false);
        for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
            const auto& data = graph.nodes[static_cast<std::size_t>(node)];
            if (data.kind != UnifiedNodeKind::Track) continue;
            if (node_in_scope(graph, node, box)) {
                included[static_cast<std::size_t>(node)] = true;
            }
        }
        for (const auto& bump : terminal_bumps) include_bump_access(graph, bump, included);
        for (const auto& source : net.sources) {
            const int node = resolve_graph_node(graph, source);
            if (node < 0) throw std::runtime_error("direct ILP source is absent from graph");
            included[static_cast<std::size_t>(node)] = true;
        }
        for (const auto& demand : net.demands) {
            const int node = resolve_graph_node(graph, demand.sink);
            if (node < 0) throw std::runtime_error("direct ILP sink is absent from graph");
            included[static_cast<std::size_t>(node)] = true;
        }
        if (net.kind == RoutingNetKind::PNnet) {
            if (net.virtual_source_node < 0) throw std::runtime_error("PNnet lacks virtual root");
            included[static_cast<std::size_t>(net.virtual_source_node)] = true;
        }
        auto scope = RoutingScope{};
        scope.net_id = net.net_id;
        scope.node_offset.assign(graph.nodes.size(), -1);
        scope.arc_offset.assign(graph.arcs.size(), -1);
        for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
            if (!included[static_cast<std::size_t>(node)]) continue;
            scope.node_offset[static_cast<std::size_t>(node)] = static_cast<int>(scope.node_ids.size());
            scope.node_ids.push_back(node);
        }
        for (int arc_id = 0; arc_id < static_cast<int>(graph.arcs.size()); ++arc_id) {
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (!included[static_cast<std::size_t>(arc.u)]
                || !included[static_cast<std::size_t>(arc.v)]) continue;
            if (arc.is_virtual_source_arc && arc.u != net.virtual_source_node) continue;
            scope.arc_offset[static_cast<std::size_t>(arc_id)] = static_cast<int>(scope.arc_ids.size());
            scope.arc_ids.push_back(arc_id);
        }
        debug::info_fmt("direct ILP scope net={} nodes={} arcs={}",
                        net.net_id, scope.node_ids.size(), scope.arc_ids.size());
        if (verbose_level > 0) debug::info_fmt("  bbox={}", format_bbox(box));
        result.push_back(std::move(scope));
    }
    return result;
}

} // namespace PR_tool
