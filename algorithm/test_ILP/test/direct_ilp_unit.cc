#include "direct_ilp/direct_router.hh"
#include "direct_ilp/direct_validate.hh"
#include "common/hw_map.hh"
#include "rrr/rrr.hh"
#include "scope/scope_bbox.hh"
#include "test_ilp_cli.hh"

#include <debug/debug.hh>

#include <stdexcept>

namespace PR_tool {
namespace {

auto require(bool condition, const char* message) -> void {
    if (!condition) throw std::runtime_error(message);
}

auto add_track(UnifiedGraph& graph, int col, int row = 0) -> int {
    const int id = static_cast<int>(graph.nodes.size());
    auto node = UnifiedNode{};
    node.kind = UnifiedNodeKind::Track;
    node.unit = 0;
    node.track_dir = 0;
    node.track_row = row;
    node.track_col = col;
    node.track_index = 0;
    graph.nodes.push_back(node);
    graph.in_arc_ids.emplace_back();
    graph.out_arc_ids.emplace_back();
    graph.track_node_by_key[{0, 0, row, col, 0}] = id;
    return id;
}

auto track_ref(int col) -> GraphNodeRef {
    auto ref = GraphNodeRef{};
    ref.kind = GraphNodeRef::Kind::Track;
    ref.track_coord = hardware::TrackCoord{0, col,
        hardware::TrackDirection::Horizontal, 0};
    ref.track_index = 0;
    return ref;
}

auto add_arc(UnifiedGraph& graph, int u, int v, bool virtual_source = false)
    -> void {
    const int id = static_cast<int>(graph.arcs.size());
    graph.arcs.push_back(UnifiedArc{u, v, false, false, virtual_source,
                                   -1, -1, PhysicalSwitchKind::None});
    graph.out_arc_ids[static_cast<std::size_t>(u)].push_back(id);
    graph.in_arc_ids[static_cast<std::size_t>(v)].push_back(id);
}

auto add_edge(UnifiedGraph& graph, int u, int v) -> void {
    add_arc(graph, u, v);
    add_arc(graph, v, u);
}

auto add_switch_edge(UnifiedGraph& graph, int u, int v, int switch_id,
                     PhysicalSwitchKind kind, int mode_group = -1,
                     bool straight = false) -> void {
    const auto append = [&](int from, int to) {
        const int id = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back(UnifiedArc{from, to, straight, !straight && mode_group >= 0,
            false, mode_group, switch_id, kind});
        graph.out_arc_ids[static_cast<std::size_t>(from)].push_back(id);
        graph.in_arc_ids[static_cast<std::size_t>(to)].push_back(id);
    };
    append(u, v);
    append(v, u);
}

auto add_bump(UnifiedGraph& graph, int index) -> int {
    const int id = static_cast<int>(graph.nodes.size());
    auto node = UnifiedNode{};
    node.kind = UnifiedNodeKind::Bump;
    node.bump = Bump_coord{0, 0, 0, static_cast<std::size_t>(index)};
    graph.nodes.push_back(node);
    graph.in_arc_ids.emplace_back(); graph.out_arc_ids.emplace_back();
    graph.bump_node_by_key[node.bump] = id;
    return id;
}

auto bump_ref(int index) -> GraphNodeRef {
    auto ref = GraphNodeRef{};
    ref.kind = GraphNodeRef::Kind::Bump;
    ref.bump = Bump_coord{0, 0, 0, static_cast<std::size_t>(index)};
    return ref;
}

auto add_hline(UnifiedGraph& graph, int line) -> int {
    const int id = static_cast<int>(graph.nodes.size());
    auto node = UnifiedNode{};
    node.kind = UnifiedNodeKind::HLine;
    node.line_index = static_cast<std::size_t>(line);
    graph.nodes.push_back(node);
    graph.in_arc_ids.emplace_back(); graph.out_arc_ids.emplace_back();
    graph.hline_node_by_key[{0, 0, 0, node.line_index}] = id;
    return id;
}

auto add_vline(UnifiedGraph& graph, int line) -> int {
    const int id = static_cast<int>(graph.nodes.size());
    auto node = UnifiedNode{};
    node.kind = UnifiedNodeKind::VLine;
    node.line_index = static_cast<std::size_t>(line);
    graph.nodes.push_back(node);
    graph.in_arc_ids.emplace_back(); graph.out_arc_ids.emplace_back();
    graph.vline_node_by_key[{0, node.line_index}] = id;
    return id;
}

auto full_scope(const UnifiedGraph& graph) -> RoutingScope {
    auto scope = RoutingScope{};
    scope.net_id = 0;
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        scope.node_offset.push_back(node);
        scope.node_ids.push_back(node);
    }
    for (int arc = 0; arc < static_cast<int>(graph.arcs.size()); ++arc) {
        scope.arc_offset.push_back(arc);
        scope.arc_ids.push_back(arc);
    }
    return scope;
}

auto solve(UnifiedGraph& graph, const RoutingNet& net)
    -> DirectIlpResult {
    const auto direct = build_direct_graph(graph);
    const auto nets = std::Vector<RoutingNet>{net};
    const auto scopes = std::Vector<RoutingScope>{full_scope(graph)};
    auto result = solve_direct_ilp(graph, direct, nets, scopes,
        DirectIlpOptions{0, 1, "/private/tmp/direct_ilp_unit_highs.log"});
    require(result.route.ok, "synthetic direct ILP failed");
    require(validate_direct_route(graph, nets, scopes, result.route), "invalid direct ILP route");
    return result;
}

auto simple_path_case() -> void {
    auto graph = UnifiedGraph{};
    const int a = add_track(graph, 0), b = add_track(graph, 1);
    add_edge(graph, a, b);
    auto net = RoutingNet{};
    net.sources.push_back(track_ref(0));
    net.demands.push_back(RoutingDemand{0, track_ref(1), {0}, true});
    const auto result = solve(graph, net);
    require(result.route.paths.size() == 1, "wrong path count");
    require(result.route.total_wirelength == 2, "wrong wirelength");
    const auto direct = build_direct_graph(graph);
    require(direct.edges.size() == 1 && direct.edges.front().arcs.size() == 2,
            "reverse arcs were not merged");
    auto missing_reverse = graph;
    missing_reverse.arcs.pop_back();
    bool rejected = false;
    try { (void)build_direct_graph(missing_reverse); }
    catch (const std::logic_error&) { rejected = true; }
    require(rejected, "missing reverse arc was accepted");
    auto mismatched_reverse = graph;
    mismatched_reverse.arcs[1].physical_switch_id = 7;
    rejected = false;
    try { (void)build_direct_graph(mismatched_reverse); }
    catch (const std::logic_error&) { rejected = true; }
    require(rejected, "mismatched reverse arc was accepted");
    const auto rerouted = optimize_routes_rrr(
        graph, {net}, {full_scope(graph)}, result.route,
        RrrOptions{.max_iterations = 1, .max_sweeps = 1});
    require(validate_direct_route(graph, {net}, {full_scope(graph)}, rerouted),
            "invalid RRR route");
    require(rerouted.total_wirelength <= result.route.total_wirelength,
            "RRR worsened the route");
}

auto pn_source_case() -> void {
    auto graph = UnifiedGraph{};
    add_track(graph, 0);
    const int selected = add_track(graph, 1);
    const int bump = static_cast<int>(graph.nodes.size());
    auto bump_node = UnifiedNode{};
    bump_node.kind = UnifiedNodeKind::Bump;
    bump_node.bump = Bump_coord{0, 0, 0, 0};
    graph.nodes.push_back(bump_node);
    graph.in_arc_ids.emplace_back(); graph.out_arc_ids.emplace_back();
    graph.bump_node_by_key[bump_node.bump] = bump;
    const int root = static_cast<int>(graph.nodes.size());
    auto root_node = UnifiedNode{};
    root_node.kind = UnifiedNodeKind::VirtualSource;
    root_node.unit = 0;
    graph.nodes.push_back(root_node);
    graph.in_arc_ids.emplace_back(); graph.out_arc_ids.emplace_back();
    add_edge(graph, selected, bump);
    add_arc(graph, root, 0, true);
    add_arc(graph, root, selected, true);
    auto net = RoutingNet{};
    net.kind = RoutingNetKind::PNnet;
    net.virtual_source_node = root;
    net.sources = {track_ref(0), track_ref(1)};
    auto bump_ref = GraphNodeRef{};
    bump_ref.kind = GraphNodeRef::Kind::Bump;
    bump_ref.bump = bump_node.bump;
    net.demands.push_back(RoutingDemand{0, bump_ref, {0, 1}, false});
    const auto result = solve(graph, net);
    require(result.route.paths.front().source_index == 1, "wrong PN source index");
    require(result.route.paths.front().physical_source_node == selected,
            "wrong PN physical source");
    require(result.route.total_wirelength == 2, "PN virtual edge counted in wirelength");
}

auto pn_multiple_demands_case() -> void {
    auto graph = UnifiedGraph{};
    const int s0 = add_track(graph, 0), s1 = add_track(graph, 1);
    const int b0 = add_bump(graph, 0), b1 = add_bump(graph, 1);
    const int root = static_cast<int>(graph.nodes.size());
    auto root_node = UnifiedNode{};
    root_node.kind = UnifiedNodeKind::VirtualSource;
    graph.nodes.push_back(root_node);
    graph.in_arc_ids.emplace_back(); graph.out_arc_ids.emplace_back();
    add_edge(graph, s0, b0);
    add_edge(graph, s1, b1);
    add_arc(graph, root, s0, true);
    add_arc(graph, root, s1, true);
    auto net = RoutingNet{};
    net.kind = RoutingNetKind::PNnet;
    net.virtual_source_node = root;
    net.sources = {track_ref(0), track_ref(1)};
    net.demands.push_back(RoutingDemand{0, bump_ref(0), {0, 1}, false});
    net.demands.push_back(RoutingDemand{1, bump_ref(1), {0, 1}, false});
    const auto result = solve(graph, net);
    require(result.route.paths.size() == 2, "PN demand was lost");
    require(result.route.paths[0].source_index == 0
            && result.route.paths[1].source_index == 1,
            "PN demands did not choose physical sources independently");
}

auto sync_equal_length_case() -> void {
    auto graph = UnifiedGraph{};
    for (int col = 0; col < 7; ++col) add_track(graph, col);
    add_edge(graph, 0, 1); add_edge(graph, 1, 2);
    add_edge(graph, 3, 4); add_edge(graph, 3, 5); add_edge(graph, 5, 4);
    auto net = RoutingNet{};
    net.is_sync_bus = true;
    net.sources = {track_ref(0), track_ref(3)};
    net.demands.push_back(RoutingDemand{0, track_ref(2), {0}, true});
    net.demands.push_back(RoutingDemand{1, track_ref(4), {1}, true});
    const auto result = solve(graph, net);
    require(result.route.paths.size() == 2, "wrong sync path count");
    require(result.route.paths[0].node_path.size() == 3, "wrong first sync length");
    require(result.route.paths[1].node_path.size() == 3, "sync equality missing");
}

auto owner_capacity_case() -> void {
    auto graph = UnifiedGraph{};
    add_track(graph, 0);
    add_track(graph, 1);
    add_edge(graph, 0, 1);
    auto first = RoutingNet{};
    first.net_id = 0;
    first.sources.push_back(track_ref(0));
    first.demands.push_back(RoutingDemand{0, track_ref(1), {0}, true});
    auto second = first;
    second.net_id = 1;
    const auto nets = std::Vector<RoutingNet>{first, second};
    auto scopes = std::Vector<RoutingScope>{full_scope(graph), full_scope(graph)};
    scopes[1].net_id = 1;
    const auto result = solve_direct_ilp(graph, build_direct_graph(graph), nets, scopes,
        DirectIlpOptions{0, 1, "/private/tmp/direct_ilp_unit_highs.log"});
    require(!result.route.ok, "two owners occupied the same physical nodes");
}

auto shared_owner_case() -> void {
    auto graph = UnifiedGraph{};
    for (int col = 0; col < 3; ++col) add_track(graph, col);
    add_edge(graph, 0, 1);
    add_edge(graph, 1, 2);
    auto net = RoutingNet{};
    net.sources.push_back(track_ref(0));
    net.demands.push_back(RoutingDemand{0, track_ref(1), {0}, true});
    net.demands.push_back(RoutingDemand{1, track_ref(2), {0}, true});
    const auto result = solve(graph, net);
    require(result.route.paths.size() == 2, "multi-sink demand was lost");
    require(result.route.total_wirelength == 3, "shared trunk was counted twice");
}

auto bump_to_tracks_bbox_case() -> void {
    auto net = RoutingNet{};
    net.kind = RoutingNetKind::Tnet;
    auto bump = GraphNodeRef{};
    bump.kind = GraphNodeRef::Kind::Bump;
    bump.bump = Bump_coord{0, 0, 0, 0};
    net.sources.push_back(bump);
    net.demands.push_back(RoutingDemand{0, track_ref(5), {0}, true});
    const auto box = compute_scope_bbox_for_net(net);
    require(box.col_max >= track_to_cob(net.demands.front().sink.track_coord).col,
            "BumpToTracks bbox omitted track sink");
}

auto bbox_plus_one_scope_case() -> void {
    auto graph = UnifiedGraph{};
    graph.rows = 9;
    graph.cols = 13;
    const int inside = add_track(graph, 0, 1);
    const int patch_track = add_track(graph, 1, 0);
    const int outside = add_track(graph, 3, 0);
    const int source = add_bump(graph, 0);
    const int sink = add_bump(graph, 1);
    auto net = RoutingNet{};
    net.kind = RoutingNetKind::Bnet;
    net.sources = {bump_ref(0)};
    net.demands.push_back(RoutingDemand{0, bump_ref(1), {0}, true});
    const auto box = compute_scope_bbox_for_net(net);
    require(box.row_min == 1 && box.row_max == 1
                && box.col_min == 0 && box.col_max == 0,
            "unexpected original Bnet bbox");
    const auto scopes = build_direct_scopes(graph, {net}, 0);
    const auto& scope = scopes.front();
    require(scope.node_offset[static_cast<std::size_t>(inside)] >= 0,
            "original bbox omitted an inside Track");
    require(scope.node_offset[static_cast<std::size_t>(patch_track)] >= 0,
            "direct scope omitted a bbox+1 Track");
    require(scope.node_offset[static_cast<std::size_t>(outside)] < 0,
            "direct scope exceeded bbox+1");
    require(scope.node_offset[static_cast<std::size_t>(source)] >= 0
                && scope.node_offset[static_cast<std::size_t>(sink)] >= 0,
            "direct scope omitted a Bump endpoint");
}

auto tob_matching_case() -> void {
    auto graph = UnifiedGraph{};
    const int b0 = add_bump(graph, 0), b1 = add_bump(graph, 1);
    const int h = add_hline(graph, 0);
    const int v = add_vline(graph, 0);
    const int t0 = add_track(graph, 0), t1 = add_track(graph, 1);
    add_switch_edge(graph, b0, h, 0, PhysicalSwitchKind::BumpH);
    add_switch_edge(graph, b1, h, 1, PhysicalSwitchKind::BumpH);
    add_switch_edge(graph, h, v, 2, PhysicalSwitchKind::HLineVLine);
    add_switch_edge(graph, v, t0, 3, PhysicalSwitchKind::VLineTrack);
    add_switch_edge(graph, v, t1, 4, PhysicalSwitchKind::VLineTrack);
    auto net = RoutingNet{};
    net.kind = RoutingNetKind::Bnet;
    net.sources = {bump_ref(0), bump_ref(1)};
    net.demands.push_back(RoutingDemand{0, track_ref(0), {0}, true});
    net.demands.push_back(RoutingDemand{1, track_ref(1), {1}, true});
    const auto route = solve_direct_ilp(graph, build_direct_graph(graph), {net},
        {full_scope(graph)}, DirectIlpOptions{0, 1,
            "/private/tmp/direct_ilp_unit_highs.log"});
    require(!route.route.ok && route.stats.status == "Infeasible",
            "TOB Bump-HLine matching conflict was accepted");
}

auto tob_second_stage_matching_case() -> void {
    auto graph = UnifiedGraph{};
    const int b0 = add_bump(graph, 0), b1 = add_bump(graph, 1);
    const int h0 = add_hline(graph, 0), h1 = add_hline(graph, 1);
    const int v = add_vline(graph, 0);
    const int t0 = add_track(graph, 0), t1 = add_track(graph, 1);
    add_switch_edge(graph, b0, h0, 0, PhysicalSwitchKind::BumpH);
    add_switch_edge(graph, b1, h1, 1, PhysicalSwitchKind::BumpH);
    add_switch_edge(graph, h0, v, 2, PhysicalSwitchKind::HLineVLine);
    add_switch_edge(graph, h1, v, 3, PhysicalSwitchKind::HLineVLine);
    add_switch_edge(graph, v, t0, 4, PhysicalSwitchKind::VLineTrack);
    add_switch_edge(graph, v, t1, 5, PhysicalSwitchKind::VLineTrack);
    auto net = RoutingNet{};
    net.kind = RoutingNetKind::Bnet;
    net.sources = {bump_ref(0), bump_ref(1)};
    net.demands.push_back(RoutingDemand{0, track_ref(0), {0}, true});
    net.demands.push_back(RoutingDemand{1, track_ref(1), {1}, true});
    const auto route = solve_direct_ilp(graph, build_direct_graph(graph), {net},
        {full_scope(graph)}, DirectIlpOptions{0, 1,
            "/private/tmp/direct_ilp_unit_highs.log"});
    require(!route.route.ok && route.stats.status == "Infeasible",
            "TOB HLine-VLine matching conflict was accepted");
}

auto tob_mode_case() -> void {
    auto graph = UnifiedGraph{};
    const int v0 = add_vline(graph, 0), v1 = add_vline(graph, 8);
    const int t0 = add_track(graph, 0), t1 = add_track(graph, 1);
    const int t2 = add_track(graph, 2), t3 = add_track(graph, 3);
    add_switch_edge(graph, v0, t0, 2, PhysicalSwitchKind::VLineTrack, 0, true);
    add_switch_edge(graph, v0, t1, 3, PhysicalSwitchKind::VLineTrack, 0, true);
    add_switch_edge(graph, v1, t2, 4, PhysicalSwitchKind::VLineTrack, 0, false);
    add_switch_edge(graph, v1, t3, 5, PhysicalSwitchKind::VLineTrack, 0, false);
    auto net = RoutingNet{};
    net.sources = {track_ref(0), track_ref(2)};
    net.demands.push_back(RoutingDemand{0, track_ref(1), {0}, true});
    net.demands.push_back(RoutingDemand{1, track_ref(3), {1}, true});
    const auto route = solve_direct_ilp(graph, build_direct_graph(graph), {net},
        {full_scope(graph)}, DirectIlpOptions{0, 1,
            "/private/tmp/direct_ilp_unit_highs.log"});
    require(!route.route.ok && route.stats.status == "Infeasible",
            "TOB straight/swap conflict was accepted");
}

} // namespace
} // namespace PR_tool

auto main() -> int {
    PR_tool::debug::initial_log("/private/tmp/direct_ilp_unit_debug.log");
    const auto options = PR_tool::parse_test_ilp_cli({"config", "--time-limit", "2", "-vv"});
    if (options.time_limit_minutes != 2 || options.verbose_level != 2)
        throw std::runtime_error("CLI parsing failed");
    PR_tool::simple_path_case();
    PR_tool::pn_source_case();
    PR_tool::pn_multiple_demands_case();
    PR_tool::sync_equal_length_case();
    PR_tool::owner_capacity_case();
    PR_tool::shared_owner_case();
    PR_tool::bump_to_tracks_bbox_case();
    PR_tool::bbox_plus_one_scope_case();
    PR_tool::tob_matching_case();
    PR_tool::tob_second_stage_matching_case();
    PR_tool::tob_mode_case();
    return 0;
}
