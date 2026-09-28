#include "sat/routing_feedback.hh"
#include "sat/routing_solution_validate.hh"
#include "scope/scope_bbox.hh"

#include <hardware/cob/cob.hh>
#include <hardware/track/track.hh>
#include <array>
#include <iostream>
#include <stdexcept>

namespace PR_tool {
namespace {
auto require(bool value, const char* message) -> void {
    if (!value) throw std::runtime_error(message);
}

auto add_pair(RoutingProblemState& state, std::size_t net, std::size_t demand,
              IlpBoundingBox bbox) -> PairKey {
    const auto key = PairKey{net, demand, 0};
    const auto index = state.pairs.size();
    state.pair_index_by_key[key] = index;
    state.pair_indices_by_net[net].push_back(index);
    state.pairs.push_back({key, {5}, bbox});
    return key;
}

auto expansion_rhythm() -> void {
    auto disconnected = PairRoutingState{}; disconnected.pair_bbox = {2, 2, 2, 2};
    apply_feedback_step_to_pair(disconnected, 1);
    require(disconnected.delays.empty() && format_bbox(disconnected.pair_bbox) == "(1,3,1,3)",
            "disconnected scope must expand without inventing a shortest delay");
    auto pair = PairRoutingState{};
    pair.pair_bbox = {2, 2, 2, 2}; pair.delays = {5};
    apply_feedback_step_to_pair(pair, 1);
    require(format_bbox(pair.pair_bbox) == "(1,3,1,3)" && max_delay(pair.delays) == 6,
            "first expansion must grow bbox and length");
    for (int step = 2; step <= 4; ++step) {
        apply_feedback_step_to_pair(pair, step);
        require(format_bbox(pair.pair_bbox) == "(1,3,1,3)" && max_delay(pair.delays) == 5 + step,
                "steps two through four must only grow length");
    }
    for (int step = 5; step <= 8; ++step) {
        apply_feedback_step_to_pair(pair, step);
        require(format_bbox(pair.pair_bbox) == "(0,4,0,4)" && max_delay(pair.delays) == 5 + step,
                "second four-step cycle must grow bbox only at step five");
    }
}

auto expanded_scope_connectivity() -> void {
    const auto graph = build_unified_graph(nullptr, {});
    const auto directions = std::array{hardware::COBDirection::Left, hardware::COBDirection::Right,
                                      hardware::COBDirection::Up, hardware::COBDirection::Down};
    const auto track_node = [&](const hardware::TrackCoord& coord) {
        auto ref = GraphNodeRef{}; ref.kind = GraphNodeRef::Kind::Track;
        ref.track_coord = coord; ref.track_index = coord.index;
        return resolve_graph_node(graph, ref);
    };
    std::size_t channels = 0, excluded = 0, boundary = 0, corner_arcs = 0;
    for (int row = 0; row < graph.rows; ++row) {
        for (int col = 0; col < graph.cols; ++col) {
            auto net = RoutingNet{};
            net.scope_bbox = expand_pair_bbox_one_cell({row, row, col, col});
            const auto scope = build_scope(graph, net, {}, {}, {});
            const auto& box = net.scope_bbox;
            const auto inside = [&](const hardware::COBCoord& cob) {
                return cob.row >= box.row_min && cob.row <= box.row_max &&
                       cob.col >= box.col_min && cob.col <= box.col_max;
            };
            for (int id = 0; id < static_cast<int>(graph.nodes.size()); ++id) {
                const auto& node = graph.nodes[id];
                if (node.kind != UnifiedNodeKind::Track) continue;
                auto track = hardware::Track(node.track_row, node.track_col,
                    node.track_dir == 0 ? hardware::TrackDirection::Horizontal :
                                         hardware::TrackDirection::Vertical, node.track_index);
                const auto adjacent = track.adjacent_cob_coords();
                const bool a = inside(std::get<1>(adjacent[0]));
                const bool b = inside(std::get<1>(adjacent[1]));
                const bool external = node.track_dir == 0 ?
                    node.track_col == 0 || node.track_col == graph.cols :
                    node.track_row == 0 || node.track_row == graph.rows;
                const bool expected = external ? a || b : a && b;
                require((scope.node_offset[id] >= 0) == expected,
                        "scope Track does not obey internal/boundary Channel rules");
                if (a && b) ++channels;
                if (!external && a != b) ++excluded;
                if (external && expected) ++boundary;
            }
            for (const auto r : {box.row_min, box.row_max}) {
                for (const auto c : {box.col_min, box.col_max}) {
                    auto cob = hardware::COB(r, c);
                    for (const auto from : directions) {
                        for (std::size_t index = 0; index < 128; ++index) {
                            const int u = track_node(cob.to_dir_track_coord(from, index));
                            require(u >= 0, "corner COB side missing from hardware graph");
                            if (scope.node_offset[u] < 0) continue;
                            for (const auto& connector : cob.adjacent_connectors(from, index, cob.coord())) {
                                const int v = track_node(cob.to_dir_track_coord(connector.to_dir(), connector.to_track_index()));
                                require(v >= 0, "corner COB target missing from hardware graph");
                                if (scope.node_offset[v] < 0) continue;
                                bool included = false;
                                for (const int aid : graph.out_arc_ids[u])
                                    if (graph.arcs[aid].v == v && scope.arc_offset[aid] >= 0) included = true;
                                require(included, "corner COB switch missing from expanded scope");
                                ++corner_arcs;
                            }
                        }
                    }
                }
            }
        }
    }
    std::cout << "expanded scope: boxes=" << graph.rows * graph.cols
              << " internal_channel_tracks=" << channels << " excluded_fringe_tracks=" << excluded
              << " boundary_tracks=" << boundary << " corner_arcs=" << corner_arcs << '\n';
}

auto endpoint_scope_exception() -> void {
    const auto graph = build_unified_graph(nullptr, {});
    auto cob = hardware::COB(2, 2);
    const auto track_ref = [](hardware::TrackCoord coord) {
        auto ref = GraphNodeRef{}; ref.kind = GraphNodeRef::Kind::Track;
        ref.track_coord = coord; ref.track_index = coord.index; return ref;
    };
    auto net = RoutingNet{}; net.has_scope_bbox = true; net.scope_bbox = {2, 2, 2, 2};
    net.sources = {track_ref(cob.to_dir_track_coord(hardware::COBDirection::Left, 0))};
    const auto connectors = cob.adjacent_connectors(hardware::COBDirection::Left, 0, cob.coord());
    net.demands = {{0, track_ref(cob.to_dir_track_coord(connectors[0].to_dir(),
                   connectors[0].to_track_index())), {0}, true}};
    const int source = resolve_graph_node(graph, net.sources[0]);
    const int sink = resolve_graph_node(graph, net.demands[0].sink);
    require(!node_in_scope(graph, source, net.scope_bbox) && !node_in_scope(graph, sink, net.scope_bbox),
            "endpoint fixture must use internal fringe Tracks");
    const auto scope = build_scope(graph, net, {source}, {sink}, {});
    require(scope.node_offset[source] >= 0 && scope.node_offset[sink] >= 0,
            "actual source/sink must remain in scope");
    auto model = UnifiedSatModel{}; model.scopes = {scope};
    auto vars = SourceDelayVars{}; vars.source_node = source; vars.d_max = 1;
    vars.d_var.assign(scope.node_ids.size(), std::Vector<int>(2, 1));
    model.sources = {vars}; model.pair_delays = {{0, 0, 0, source, sink, {1}, 1, -1}};
    auto route = SatRoutingResult{}; route.paths = {{0, 0, 0, -1, {source, sink}}};
    const ModelValue value = [](int) { return true; };
    require(validate_routing_solution(graph, {net}, model, value, route).pass,
            "SAT validation rejected actual source/sink scope exceptions");
    const int other = resolve_graph_node(graph,
        track_ref(cob.to_dir_track_coord(hardware::COBDirection::Left, 1)));
    require(scope.node_offset[other] < 0, "endpoint inclusion leaked to other Channel Tracks");
    route.paths[0].node_path = {source, other, sink};
    require(validate_routing_solution(graph, {net}, model, value, route).category_counts.contains(
                ViolationKind::NodeOutOfScope), "SAT validation accepted a non-terminal fringe Track");
}

auto mixed_failures(bool same_net) -> void {
    auto state = RoutingProblemState{};
    const auto full = add_pair(state, 0, 0, full_chip_bbox());
    const auto local = add_pair(state, same_net ? 0 : 1, 1, {2, 2, 2, 2});
    add_pair(state, 2, 0, {2, 2, 2, 2});
    require(apply_feedback_expansion(state, {}, {full, local}) == FeedbackExpansionStatus::Expanded,
            "mixed failures did not expand");
    require(is_full_chip_bbox(state.pairs[0].pair_bbox) && state.pairs[0].delays == std::Vector<int>{5},
            "full failed pair changed while another failed pair could expand");
    require(format_bbox(state.pairs[1].pair_bbox) == "(1,3,1,3)" && max_delay(state.pairs[1].delays) == 6,
            "expandable failed pair was skipped");
    require(format_bbox(state.pairs[2].pair_bbox) == "(2,2,2,2)" && max_delay(state.pairs[2].delays) == 5 &&
            !state.feedback_failure_count_by_net.contains(2), "non-failed net expanded too early");
    if (!same_net)
        require(!state.feedback_failure_count_by_net.contains(0), "frozen net consumed an expansion count");
    apply_feedback_expansion(state, {}, {full, local});
    require(format_bbox(state.pairs[1].pair_bbox) == "(1,3,1,3)" && max_delay(state.pairs[1].delays) == 7,
            "second feedback did not preserve the expansion rhythm");
}

auto fallback_and_exhaustion() -> void {
    auto state = RoutingProblemState{};
    const auto a = add_pair(state, 0, 0, full_chip_bbox());
    const auto b = add_pair(state, 1, 0, full_chip_bbox());
    add_pair(state, 2, 0, {2, 2, 2, 2});
    require(apply_feedback_expansion(state, {}, {a, b}) == FeedbackExpansionStatus::Expanded,
            "all full failures did not expand a non-failed net");
    require(max_delay(state.pairs[0].delays) == 5 && max_delay(state.pairs[1].delays) == 5,
            "fallback changed failed nets");
    require(format_bbox(state.pairs[2].pair_bbox) == "(1,3,1,3)" && max_delay(state.pairs[2].delays) == 6,
            "fallback did not apply the first expansion to the non-failed net");
    state.pairs[2].pair_bbox = full_chip_bbox();
    require(apply_feedback_expansion(state, {}, {a, b}) == FeedbackExpansionStatus::Exhausted &&
            max_delay(state.pairs[2].delays) == 6, "full-chip exhaustion changed");
}

auto sync_group_consistency() -> void {
    auto state = RoutingProblemState{};
    const auto failed = add_pair(state, 0, 0, {2, 2, 2, 2});
    add_pair(state, 0, 1, {2, 2, 2, 2});
    auto bus = RoutingNet{}; bus.net_id = 0; bus.is_sync_bus = true;
    apply_feedback_expansion(state, {bus}, {failed});
    require(format_bbox(state.pairs[0].pair_bbox) == "(1,3,1,3)" &&
            format_bbox(state.pairs[1].pair_bbox) == "(1,3,1,3)" &&
            state.pairs[0].delays == state.pairs[1].delays && max_delay(state.pairs[1].delays) == 6,
            "feedback did not synchronize the bus scope and length domain");
}
} // namespace
} // namespace PR_tool

auto main() -> int {
    using namespace PR_tool;
    expansion_rhythm(); expanded_scope_connectivity(); endpoint_scope_exception();
    mixed_failures(false); mixed_failures(true);
    fallback_and_exhaustion(); sync_group_consistency();
    std::cout << "sat_feedback_unit: PASS\n";
}
