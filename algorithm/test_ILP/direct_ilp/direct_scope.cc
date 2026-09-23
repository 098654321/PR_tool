#include "direct_ilp/direct_scope.hh"

#include "common/hw_map.hh"
#include "scope/scope_bbox.hh"

#include <debug/debug.hh>

#include <algorithm>
#include <array>
#include <set>
#include <stdexcept>

namespace PR_tool {
namespace {

auto patch_channels(const UnifiedGraph& graph, std::size_t tob, bool compact)
    -> std::set<GlobalChannelCoord> {
    const auto [tr, tc] = tob_index_from_linear(tob);
    const auto [a, b] = tob_pair_cob_coords(tr, tc);
    const int top = static_cast<int>(std::min(a.row, b.row));
    const int bottom = static_cast<int>(std::max(a.row, b.row));
    const int col = static_cast<int>(a.col);
    const std::array<GlobalChannelCoord, 9> candidates {{
        {1, bottom, col}, {1, top, col}, {1, bottom + 1, col},
        {0, top, col}, {0, top, col + 1}, {0, bottom, col},
        {0, bottom, col + 1}, {1, bottom, col - 1},
        {1, bottom, col + 1}}};
    auto out = std::set<GlobalChannelCoord>{};
    for (std::size_t i = 0; i < (compact ? 7U : candidates.size()); ++i) {
        const auto& c = candidates[i];
        const bool valid = c.dir == 0
            ? c.row >= 0 && c.row < graph.rows && c.col >= 0 && c.col <= graph.cols
            : c.row >= 0 && c.row <= graph.rows && c.col >= 0 && c.col < graph.cols;
        if (valid) out.insert(c);
    }
    return out;
}

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
        const auto box = expand_pair_bbox_one_cell(compute_scope_bbox_for_net(net));
        auto patch = std::set<GlobalChannelCoord>{};
        auto terminal_bumps = std::set<Bump_coord>{};
        for (const auto& demand : net.demands) {
            const bool sink_bump = demand.sink.kind == GraphNodeRef::Kind::Bump;
            bool source_bump = false;
            for (const auto index : demand.candidate_source_indices) {
                if (index >= net.sources.size()) throw std::runtime_error("invalid source index");
                const auto& source = net.sources[index];
                if (source.kind == GraphNodeRef::Kind::Bump) {
                    source_bump = true;
                    terminal_bumps.insert(source.bump);
                }
            }
            if (sink_bump) terminal_bumps.insert(demand.sink.bump);
            for (const auto index : demand.candidate_source_indices) {
                const auto& source = net.sources[index];
                if (source.kind != GraphNodeRef::Kind::Bump) continue;
                const auto extra = patch_channels(graph, source.bump.TOB,
                                                  source_bump && sink_bump);
                patch.insert(extra.begin(), extra.end());
            }
            if (sink_bump) {
                const auto extra = patch_channels(graph, demand.sink.bump.TOB,
                                                  source_bump && sink_bump);
                patch.insert(extra.begin(), extra.end());
            }
        }
        auto included = std::Vector<bool>(graph.nodes.size(), false);
        for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
            const auto& data = graph.nodes[static_cast<std::size_t>(node)];
            if (data.kind != UnifiedNodeKind::Track) continue;
            const GlobalChannelCoord channel{data.track_dir, data.track_row, data.track_col};
            if (node_in_scope(graph, node, box) || patch.contains(channel)) {
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
        debug::info_fmt("direct ILP scope net={} nodes={} arcs={} patch_channels={}",
                        net.net_id, scope.node_ids.size(), scope.arc_ids.size(), patch.size());
        if (verbose_level > 0) debug::info_fmt("  bbox+1={}", format_bbox(box));
        result.push_back(std::move(scope));
    }
    return result;
}

} // namespace PR_tool
