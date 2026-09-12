#include "route_log.hh"

#include "hw_map.hh"

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

auto format_track_coord(const hardware::TrackCoord& coord) -> std::String {
    const char dir =
        coord.dir == hardware::TrackDirection::Horizontal ? 'H' : 'V';
    return std::format("{{r{}, c{}, {}, i{}}}", coord.row, coord.col, dir, coord.index);
}

auto is_wirelength_resource_node(const UnifiedGraph& graph, int node_id) -> bool {
    if (node_id < 0 || static_cast<std::size_t>(node_id) >= graph.nodes.size()) {
        return false;
    }
    const auto kind = graph.nodes[static_cast<std::size_t>(node_id)].kind;
    return kind == UnifiedNodeKind::Track || kind == UnifiedNodeKind::Bump;
}

} // namespace

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
    }
    return std::format("N{}", node_id);
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
    const std::Vector<std::Vector<int>>& node_paths
) -> std::size_t {
    auto unique_nodes = std::set<int> {};
    for (const auto& node_path : node_paths) {
        for (const int node_id : node_path) {
            if (is_wirelength_resource_node(graph, node_id)) {
                unique_nodes.insert(node_id);
            }
        }
    }
    return unique_nodes.size();
}

auto total_wirelength(
    const UnifiedGraph& graph,
    const std::Vector<std::Vector<std::Vector<int>>>& paths_by_net
) -> std::size_t {
    std::size_t total = 0;
    for (const auto& paths_of_net : paths_by_net) {
        total += net_wirelength(graph, paths_of_net);
    }
    return total;
}

} // namespace PR_tool
