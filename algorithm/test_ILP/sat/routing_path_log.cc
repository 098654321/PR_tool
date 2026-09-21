#include "sat/routing_path_log.hh"

#include "common/hw_map.hh"
#include "global_route_v17/global_router.hh"
#include "scope/pair_routing_state.hh"
#include "scope/scope_bbox.hh"

#include <algorithm>
#include <debug/debug.hh>
#include <format>
#include <set>

namespace PR_tool {

namespace {

auto format_tob_linear(std::size_t tob_linear) -> std::String {
    const auto [tr, tc] = tob_index_from_linear(tob_linear);
    return std::format("TOB({},{})", tr, tc);
}

auto format_bump_coord(const Bump_coord& bump) -> std::String {
    return std::format(
        "{} B{} G{} I{}",
        format_tob_linear(bump.TOB),
        bump.Bank,
        bump.Group,
        bump.Index);
}

auto track_coord_from_node(const UnifiedNode& node) -> hardware::TrackCoord {
    return hardware::TrackCoord {
        node.track_row,
        node.track_col,
        node.track_dir == 0 ? hardware::TrackDirection::Horizontal
                            : hardware::TrackDirection::Vertical,
        node.track_index};
}

auto format_routing_kind(RoutingNetKind kind) -> std::String {
    switch (kind) {
        case RoutingNetKind::Bnet:
            return "Bnet";
        case RoutingNetKind::Tnet:
            return "Tnet";
        case RoutingNetKind::PNnet:
            return "PNnet";
    }
    return "Unknown";
}

auto source_ref_for_demand(const RoutingNet& net, const RoutingDemand& demand) -> GraphNodeRef {
    if (demand.candidate_source_indices.empty()) {
        return {};
    }
    const auto source_index = demand.candidate_source_indices.front();
    if (source_index >= net.sources.size()) {
        return {};
    }
    return net.sources[source_index];
}

auto selected_source_ref(const UnifiedGraph& graph, const RoutingNet& net,
                         const RoutingDemand& demand,
                         const SourceSinkPairPath& path) -> GraphNodeRef {
    if (path.physical_source_node >= 0
        && path.physical_source_node < static_cast<int>(graph.nodes.size())
        && graph.nodes[static_cast<std::size_t>(path.physical_source_node)].kind
            == UnifiedNodeKind::Track) {
        auto source = GraphNodeRef{};
        source.kind = GraphNodeRef::Kind::Track;
        source.track_coord =
            track_coord_from_node(graph.nodes[static_cast<std::size_t>(path.physical_source_node)]);
        source.track_index = source.track_coord.index;
        return source;
    }
    if (path.source_index < net.sources.size())
        return net.sources[path.source_index];
    return source_ref_for_demand(net, demand);
}

auto paths_for_net(
    const SatRoutingResult& result,
    std::size_t net_id
) -> std::Vector<const SourceSinkPairPath*> {
    auto out = std::Vector<const SourceSinkPairPath*> {};
    for (const auto& path : result.paths) {
        if (path.net_id == net_id) {
            out.push_back(&path);
        }
    }
    std::sort(out.begin(), out.end(), [](const SourceSinkPairPath* lhs, const SourceSinkPairPath* rhs) {
        if (lhs->demand_id != rhs->demand_id) {
            return lhs->demand_id < rhs->demand_id;
        }
        return lhs->source_index < rhs->source_index;
    });
    return out;
}

struct ScopeEndpoint {
    int row_x2{0};
    int col_x2{0};
    std::String text;
};

struct ScopeSummary {
    std::size_t channels{0};
    std::size_t cobs{0};
    std::String top{"n/a"};
    std::String bottom{"n/a"};
    std::String left{"n/a"};
    std::String right{"n/a"};
};

auto channel_text(const GlobalChannelCoord& channel) -> std::String {
    return std::format("Channel({},{},{})", channel.dir == 0 ? 'H' : 'V',
                       channel.row, channel.col);
}

auto summarize_scope(const GlobalChannelGraph& graph,
                     const std::set<GlobalChannelCoord>& channels)
    -> ScopeSummary {
    auto summary = ScopeSummary{};
    summary.channels = channels.size();
    auto points = std::Vector<ScopeEndpoint>{};
    auto cobs = std::set<std::pair<int, int>>{};
    for (const auto& channel : channels) {
        points.push_back({channel.dir == 0 ? 2 * channel.row
                                           : 2 * channel.row - 1,
                          channel.dir == 0 ? 2 * channel.col - 1
                                           : 2 * channel.col,
                          channel_text(channel)});
        const auto channel_it = graph.channel_id_by_coord.find(channel);
        if (channel_it == graph.channel_id_by_coord.end())
            continue;
        for (const int arc_id : graph.arc_ids_by_channel[channel_it->second]) {
            const auto& arc = graph.arcs[arc_id];
            for (const int node_id : {arc.u, arc.v}) {
                const auto& node = graph.nodes[node_id];
                if (node.kind == GlobalRouteNodeKind::Cob)
                    cobs.emplace(node.row, node.col);
            }
        }
    }
    for (const auto [row, col] : cobs)
        points.push_back({2 * row, 2 * col, std::format("COB({},{})", row, col)});
    summary.cobs = cobs.size();
    if (points.empty())
        return summary;
    const auto top = std::min_element(
        points.begin(), points.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.row_x2 < rhs.row_x2; });
    const auto bottom = std::max_element(
        points.begin(), points.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.row_x2 < rhs.row_x2; });
    const auto left = std::min_element(
        points.begin(), points.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.col_x2 < rhs.col_x2; });
    const auto right = std::max_element(
        points.begin(), points.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.col_x2 < rhs.col_x2; });
    summary.top = top->text;
    summary.bottom = bottom->text;
    summary.left = left->text;
    summary.right = right->text;
    return summary;
}

auto log_scope_summary(const ScopeSummary& scope, std::string_view indent) -> void {
    debug::info_fmt("{}scope_channels={} scope_cobs={} bounds: top={} bottom={} "
                    "left={} right={}",
                    indent, scope.channels, scope.cobs, scope.top, scope.bottom,
                    scope.left, scope.right);
}

} // namespace

auto is_wirelength_resource_node(const UnifiedGraph& graph, int node_id) -> bool {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= graph.nodes.size()) {
        return false;
    }
    const auto kind = graph.nodes[static_cast<std::size_t>(node_id)].kind;
    return kind == UnifiedNodeKind::Track || kind == UnifiedNodeKind::Bump;
}

auto format_track_coord(const hardware::TrackCoord& coord) -> std::String {
    const char dir =
        coord.dir == hardware::TrackDirection::Horizontal ? 'H' : 'V';
    return std::format("{{r{}, c{}, {}, i{}}}", coord.row, coord.col, dir, coord.index);
}

auto infer_net_display_kind(const RoutingNet& net) -> NetDisplayKind {
    if (net.is_sync_bus) {
        return NetDisplayKind::SyncBus;
    }
    if (net.kind == RoutingNetKind::PNnet) {
        return NetDisplayKind::TracksToBumps;
    }
    if (net.kind == RoutingNetKind::Tnet && net.demands.size() > 1) {
        return NetDisplayKind::TrackToBumps;
    }
    return NetDisplayKind::TwoPin;
}

auto net_display_kind_name(NetDisplayKind kind) -> std::String {
    switch (kind) {
        case NetDisplayKind::TwoPin:
            return "TwoPin";
        case NetDisplayKind::SyncBus:
            return "SyncBus";
        case NetDisplayKind::TrackToBumps:
            return "TrackToBumps";
        case NetDisplayKind::TracksToBumps:
            return "TracksToBumps";
    }
    return "Unknown";
}

auto format_graph_node_ref(const GraphNodeRef& ref) -> std::String {
    switch (ref.kind) {
        case GraphNodeRef::Kind::Track:
            return format_track_coord(ref.track_coord);
        case GraphNodeRef::Kind::Bump:
            return format_bump_coord(ref.bump);
        case GraphNodeRef::Kind::HLine:
            return std::format(
                "{} B{} G{} J{}",
                format_tob_linear(ref.tob),
                ref.bank,
                ref.group,
                ref.line_index);
        case GraphNodeRef::Kind::VLine:
            return std::format(
                "{} B{} V{}",
                format_tob_linear(ref.tob),
                ref.bank,
                ref.line_index);
    }
    return "Unknown";
}

auto format_path_node(const UnifiedGraph& graph, int node_id) -> std::String {
    if (node_id < 0 || node_id >= static_cast<int>(graph.nodes.size())) {
        return std::format("N{}", node_id);
    }
    const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
    switch (node.kind) {
        case UnifiedNodeKind::Track:
            return format_track_coord(track_coord_from_node(node));
        case UnifiedNodeKind::Bump:
            return format_bump_coord(node.bump);
        case UnifiedNodeKind::HLine:
            return std::format(
                "{} B{} G{} J{}",
                format_tob_linear(node.tob),
                node.bank,
                node.group,
                node.line_index);
        case UnifiedNodeKind::VLine:
            return std::format(
                "{} B{} V{}",
                format_tob_linear(node.tob),
                node.bank,
                node.line_index);
        case UnifiedNodeKind::VirtualSource:
            return std::format("R_n net={}", node.unit);
    }
    return std::format("N{}", node_id);
}

auto format_bbox_corners(const IlpBoundingBox& box) -> std::String {
    return std::format(
        "corners: ({},{}) ({},{}) ({},{}) ({},{}) bounds=({},{},{},{})",
        box.row_min,
        box.col_min,
        box.row_min,
        box.col_max,
        box.row_max,
        box.col_min,
        box.row_max,
        box.col_max,
        box.row_min,
        box.row_max,
        box.col_min,
        box.col_max);
}

auto format_path_hops(const UnifiedGraph& graph, const std::Vector<int>& node_path) -> std::String {
    if (node_path.empty()) {
        return "";
    }
    auto out = format_path_node(graph, node_path.front());
    for (std::size_t i = 1; i < node_path.size(); ++i) {
        out += " -> ";
        out += format_path_node(graph, node_path[i]);
    }
    return out;
}

auto path_wirelength(const UnifiedGraph& graph, const std::Vector<int>& node_path) -> std::size_t {
    std::size_t length = 0;
    for (const int node_id : node_path) {
        if (is_wirelength_resource_node(graph, node_id)) {
            ++length;
        }
    }
    return length;
}

auto net_wirelength(
    const UnifiedGraph& graph,
    const std::Vector<const SourceSinkPairPath*>& paths
) -> std::size_t {
    auto unique_nodes = std::set<int> {};
    for (const auto* path : paths) {
        if (path == nullptr) {
            continue;
        }
        for (const int node_id : path->node_path) {
            if (is_wirelength_resource_node(graph, node_id)) {
                unique_nodes.insert(node_id);
            }
        }
    }
    return unique_nodes.size();
}

auto total_wirelength(const UnifiedGraph& graph, const SatRoutingResult& result) -> std::size_t {
    auto net_ids = std::set<std::size_t> {};
    for (const auto& path : result.paths) {
        net_ids.insert(path.net_id);
    }
    std::size_t sum = 0;
    for (const std::size_t net_id : net_ids) {
        auto net_paths = std::Vector<const SourceSinkPairPath*> {};
        for (const auto& path : result.paths) {
            if (path.net_id == net_id) {
                net_paths.push_back(&path);
            }
        }
        sum += net_wirelength(graph, net_paths);
    }
    return sum;
}

auto log_scope_bboxes(const std::Vector<RoutingNet>& nets, int verbose_level) -> void {
    if (verbose_level < 1) {
        return;
    }
    for (const auto& net : nets) {
        debug::info_fmt(
            "scope net=\"{}\" id={} kind={} display={} {}",
            net.name,
            net.net_id,
            format_routing_kind(net.kind),
            net_display_kind_name(infer_net_display_kind(net)),
            format_bbox_corners(net.scope_bbox));
        if (verbose_level >= 3) {
            const auto child_boxes = compute_scope_child_bboxes(net);
            for (std::size_t i = 0; i < child_boxes.size(); ++i) {
                debug::info_fmt(
                    "  scope child net=\"{}\" id={} index={} {}",
                    net.name,
                    net.net_id,
                    i,
                    format_bbox_corners(child_boxes[i]));
            }
        }
    }
}

auto log_routing_paths(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const SatRoutingResult& result
) -> void {
    if (!result.ok) {
        return;
    }

    for (const auto& net : nets) {
        const auto net_paths = paths_for_net(result, net.net_id);
        if (net_paths.empty()) {
            continue;
        }

        const auto display_kind = infer_net_display_kind(net);
        debug::info_fmt(
            "route net=\"{}\" id={} kind={} display={} demands={}",
            net.name,
            net.net_id,
            format_routing_kind(net.kind),
            net_display_kind_name(display_kind),
            net.demands.size());

        if (display_kind == NetDisplayKind::TwoPin) {
            const auto* path = net_paths.front();
            const auto& demand = net.demands[path->demand_id];
            debug::info_fmt(
                "  src={} snk={}",
                format_graph_node_ref(source_ref_for_demand(net, demand)),
                format_graph_node_ref(demand.sink));
            debug::info_fmt("  path: {}", format_path_hops(graph, path->node_path));
            debug::info_fmt("  net_wirelength={}", net_wirelength(graph, net_paths));
            continue;
        }

        for (const auto* path : net_paths) {
            if (path->demand_id >= net.demands.size()) {
                continue;
            }
            const auto& demand = net.demands[path->demand_id];
            GraphNodeRef source_ref {};
            if (path->physical_source_node >= 0
                && path->physical_source_node < static_cast<int>(graph.nodes.size())
                && graph.nodes[static_cast<std::size_t>(path->physical_source_node)].kind
                    == UnifiedNodeKind::Track) {
                source_ref.kind = GraphNodeRef::Kind::Track;
                source_ref.track_coord =
                    track_coord_from_node(graph.nodes[static_cast<std::size_t>(path->physical_source_node)]);
                source_ref.track_index = source_ref.track_coord.index;
            }
            else if (path->source_index < net.sources.size()) {
                source_ref = net.sources[path->source_index];
            }
            else {
                source_ref = source_ref_for_demand(net, demand);
            }

            if (display_kind == NetDisplayKind::TracksToBumps) {
                debug::info_fmt(
                    "  demand={} selected_source={} src={} snk={}",
                    path->demand_id,
                    format_graph_node_ref(source_ref),
                    format_graph_node_ref(source_ref),
                    format_graph_node_ref(demand.sink));
            }
            else {
                debug::info_fmt(
                    "  member demand={} src={} snk={}",
                    path->demand_id,
                    format_graph_node_ref(source_ref),
                    format_graph_node_ref(demand.sink));
            }
            debug::info_fmt("    path: {}", format_path_hops(graph, path->node_path));
        }
        debug::info_fmt("  net_wirelength={}", net_wirelength(graph, net_paths));
    }
}

auto log_final_sat_scopes(
    const UnifiedGraph& graph,
    const GlobalChannelGraph& channel_graph,
    const RoutingProblemState& state,
    const std::Vector<RoutingNet>& nets,
    const SatRoutingResult& result
) -> void {
    if (!result.ok)
        return;
    debug::info("========== Final SAT pair scopes ==========");
    for (const auto& net : nets) {
        const auto net_paths = paths_for_net(result, net.net_id);
        if (net_paths.empty())
            continue;
        const auto display_kind = infer_net_display_kind(net);
        debug::info_fmt("final scope net=\"{}\" id={} kind={} display={} demands={}",
                        net.name, net.net_id, format_routing_kind(net.kind),
                        net_display_kind_name(display_kind), net.demands.size());
        for (const auto* path : net_paths) {
            if (path->demand_id >= net.demands.size())
                continue;
            const auto& demand = net.demands[path->demand_id];
            const auto* pair = find_pair_state(
                state, PairKey{path->net_id, path->demand_id, path->source_index});
            const auto scope = pair == nullptr
                ? ScopeSummary{}
                : summarize_scope(channel_graph, pair->allowed_channels);
            if (display_kind == NetDisplayKind::TwoPin) {
                debug::info_fmt("  src={} snk={}",
                                format_graph_node_ref(source_ref_for_demand(net, demand)),
                                format_graph_node_ref(demand.sink));
                log_scope_summary(scope, "  ");
            }
            else {
                const auto source = selected_source_ref(graph, net, demand, *path);
                if (display_kind == NetDisplayKind::TracksToBumps) {
                    debug::info_fmt("  demand={} selected_source={} src={} snk={}",
                                    path->demand_id, format_graph_node_ref(source),
                                    format_graph_node_ref(source),
                                    format_graph_node_ref(demand.sink));
                }
                else {
                    debug::info_fmt("  member demand={} src={} snk={}",
                                    path->demand_id, format_graph_node_ref(source),
                                    format_graph_node_ref(demand.sink));
                }
                log_scope_summary(scope, "    ");
            }
        }
    }
    debug::info("===========================================");
}

} // namespace PR_tool
