#include "test/rrr_budget_cases.hh"

#include "air_routing.hh"
#include "rrr_cli.hh"
#include "route_validate.hh"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

using namespace PR_tool;
using namespace PR_tool::rrr_detail;

auto require(bool condition, const char* message) -> void {
    if (!condition) throw std::runtime_error(message);
}

auto add_node(UnifiedGraph& graph, int row = -1, int col = 0, std::size_t track_index = 0) -> int {
    const int id = static_cast<int>(graph.nodes.size());
    auto node = UnifiedNode{};
    node.kind = UnifiedNodeKind::Track;
    node.track_row = row < 0 ? id : row; node.track_col = col;
    node.track_index = track_index;
    graph.nodes.push_back(node);
    graph.track_node_by_key[{0, 0, node.track_row, node.track_col, track_index}] = id;
    graph.in_arc_ids.emplace_back(); graph.out_arc_ids.emplace_back();
    return id;
}

auto add_path(UnifiedGraph& graph, std::Vector<int> path) -> void {
    for (std::size_t i = 1; i < path.size(); ++i) {
        const int arc = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back(UnifiedArc{path[i-1], path[i]});
        graph.out_arc_ids[path[i-1]].push_back(arc);
        graph.in_arc_ids[path[i]].push_back(arc);
        graph.directed_arc_set.insert({path[i-1], path[i]});
    }
}

auto ref(const UnifiedGraph& graph, int id) -> GraphNodeRef {
    auto value = GraphNodeRef{};
    value.kind = GraphNodeRef::Kind::Track;
    const auto& node = graph.nodes[id];
    value.track_coord = hardware::TrackCoord{node.track_row, node.track_col,
        hardware::TrackDirection::Horizontal, node.track_index};
    value.track_index = node.track_index;
    return value;
}

auto net(const UnifiedGraph& graph, std::size_t id, int source, int sink) -> RoutingNet {
    auto value = RoutingNet{};
    value.net_id = id; value.name = "air_net_" + std::to_string(id);
    value.sources = {ref(graph, source)};
    value.demands = {RoutingDemand{0, ref(graph, sink), {0}, true}};
    return value;
}

auto expired_params() -> RrrParams {
    auto params = RrrParams{};
    params.time_budget_seconds = 0.001;
    params.budget_start = std::chrono::steady_clock::now() - std::chrono::seconds{1};
    return params;
}

struct Fixture { UnifiedGraph graph; std::Vector<RoutingNet> nets; };

auto fixture() -> Fixture {
    auto value = Fixture{};
    // Place both alternatives inside the terminal bboxes, so initial routing
    // compares path costs rather than prioritizing a smaller spatial window.
    for (const int col : {0, 8, 1, 7, 3, 4, 2, 5, 6}) add_node(value.graph, 0, col);
    add_path(value.graph, {0, 4, 5, 1});
    add_path(value.graph, {0, 6, 7, 8, 1});
    add_path(value.graph, {2, 4, 5, 3});
    value.nets = {net(value.graph, 0, 0, 1), net(value.graph, 1, 2, 3)};
    return value;
}

auto check_legal(const Fixture& value, const RrrResult& result, hardware::Interposer* interposer) -> void {
    require(result.status == "success" && result.stop_reason == "time_budget",
            "AIR must stop successfully on the time budget");
    require(result.best_overflow == 0 && result.missing_owners == 0
                && result.unequal_sync_groups == 0, "incumbent must be complete and legal");
    require(validate_rrr_solution(value.graph, value.nets, result, interposer),
            "AIR incumbent must pass independent validation");
    require(result.first_legal_ms >= 0 && result.best_legal_ms >= result.first_legal_ms,
            "legal timestamps must be recorded");
}

auto test_budget_repair_and_incumbent() -> void {
    const auto value = fixture();
    auto interposer = hardware::Interposer{};
    auto params = expired_params();
    const auto first = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, first, &interposer);
    require(first.iterations >= 2 && first.optimization_rounds == 0,
            "expired budget without legal result must continue through overflow repair");
    require(first.total_wirelength == 9 && first.first_legal_ms >= 1000,
            "node history must separate the two conflicting shortest routes");

    params.time_budget_seconds = 0.02;
    params.budget_start = std::chrono::steady_clock::now();
    const auto optimized = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, optimized, &interposer);
    require(optimized.convergences > 1 && optimized.optimization_rounds > 0,
            "legal convergence must not stop optimization before the budget");
    require(optimized.budget_elapsed_ms >= 20 && optimized.total_wirelength == 9,
            "budgeted optimization must return the best legal incumbent");
}

auto states_for(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets)
    -> std::Vector<AirNetState> {
    auto states = std::Vector<AirNetState>{};
    for (const auto& n : nets) states.push_back({air_spatial(graph, n)});
    return states;
}

auto test_root_prefix_pruning() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 24; ++i) add_node(graph);
    auto large = net(graph, 0, 0, 5);
    large.demands.clear();
    for (int sink = 5; sink < 20; ++sink)
        large.demands.push_back({large.demands.size(), ref(graph, sink), {0}, true});
    auto nets = std::Vector<RoutingNet>{large, net(graph, 1, 20, 21)};
    auto owners = build_owners(nets);
    require(owners[0].port_count == 16, "partial-pruning threshold counts root and sinks");
    for (std::size_t slot = 0; slot < owners[0].demand_paths.size(); ++slot) {
        auto path = slot < 2 ? std::Vector<int>{0, 1, 3, 4, static_cast<int>(5+slot)}
                             : std::Vector<int>{0, 1, static_cast<int>(5+slot)};
        add_path(graph, path); owners[0].demand_paths[slot] = std::move(path);
    }
    add_path(graph, {20, 4, 21}); owners[1].demand_paths[0] = {20, 4, 21};
    auto resources = ResourceModel{};
    for (auto& owner : owners) air_refresh(graph, owner, resources);
    auto states = states_for(graph, nets);
    auto interposer = hardware::Interposer{};
    require(air_prune_illegal(graph, nets, owners, resources, states, &interposer) == 2,
            "all claimants must see the same pre-prune congestion snapshot");
    require(owners[0].demand_paths[0] == std::Vector<int>{0, 1, 3}
                && owners[0].demand_paths[1] == std::Vector<int>{0, 1, 3},
            "conflicting branch and disconnected descendants must be removed");
    require(owners[0].demand_paths[2] == std::Vector<int>{0, 1, 7}
                && resources.holds(owners[0].id, node_resource(3))
                && !resources.holds(owners[0].id, node_resource(4))
                && !resources.holds(owners[0].id, node_resource(5)),
            "legal root prefixes and unaffected sinks must stay, no downstream islands");
    require(!air_connected(graph, nets[0], owners[0], 0)
                && air_connected(graph, nets[0], owners[0], 2),
            "nonempty partial prefix is not a completed sink");
    require(owners[1].demand_paths[0].empty(), "small conflicting net must be fully ripped");

    // Lowering port count to fifteen changes the policy to a whole-net rip.
    owners = build_owners(nets);
    owners[0].port_count = 15;
    owners[0].demand_paths[0] = {0, 1, 3, 4, 5};
    owners[1].demand_paths[0] = {20, 4, 21};
    resources = ResourceModel{};
    for (auto& owner : owners) air_refresh(graph, owner, resources);
    air_prune_illegal(graph, nets, owners, resources, states, &interposer);
    require(owners[0].claimed.empty(), "nets with fewer than sixteen ports are fully ripped");
}

auto test_sync_atomic_pruning() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 8; ++i) add_node(graph);
    add_path(graph, {0, 4, 1}); add_path(graph, {2, 5, 3}); add_path(graph, {6, 4, 7});
    auto sync = net(graph, 0, 0, 1);
    sync.is_sync_bus = true; sync.sources.push_back(ref(graph, 2));
    sync.demands.push_back({1, ref(graph, 3), {1}, true});
    auto nets = std::Vector<RoutingNet>{sync, net(graph, 1, 6, 7)};
    auto owners = build_owners(nets);
    owners[0].demand_paths[0] = {0, 4, 1}; owners[1].demand_paths[0] = {2, 5, 3};
    owners[2].demand_paths[0] = {6, 4, 7};
    auto resources = ResourceModel{};
    for (auto& owner : owners) air_refresh(graph, owner, resources);
    auto states = states_for(graph, nets); auto interposer = hardware::Interposer{};
    air_prune_illegal(graph, nets, owners, resources, states, &interposer);
    require(owners[0].claimed.empty() && owners[1].claimed.empty(),
            "one conflicting sync lane must rip the entire sync group");
}

auto test_spatial_bin_bias_and_initial_box() -> void {
    auto graph = UnifiedGraph{}; graph.rows = 10; graph.cols = 10;
    add_node(graph, 2, 2); add_node(graph, 6, 6);
    auto n = net(graph, 0, 0, 1);
    n.demands.resize(16, n.demands.front());
    const auto spatial = air_spatial(graph, n);
    require(std::abs(spatial.bin_size - 3) < 1e-9, "bin side must be 1+sqrt(4*16/16)=3");
    require(spatial.initial == RoutingBox{2, 6, 2, 6}
                && spatial.router_bb == RoutingBox{1, 7, 1, 7},
            "bin bbox stays raw; search bbox is padded on all four sides");
    require(spatial.bin(2, 4) == std::pair<int,int>{0, 0}
                && spatial.bin(2, 5) == std::pair<int,int>{0, 1},
            "bin lookup must use the biased side, not just report it");
    add_node(graph, 0, 0); add_node(graph, 9, 10); add_node(graph, 99, 99);
    const auto edge = air_spatial(graph, net(graph, 1, 2, 3));
    require(edge.device == RoutingBox{0, 10, 0, 10}
                && edge.router_bb == RoutingBox{0, 10, 0, 10},
            "clip to actual device including outer Track/I/O edges, not stray graph coordinates");
}

auto test_round_boundary_bbox_updates() -> void {
    auto graph = UnifiedGraph{}; graph.rows = 10; graph.cols = 10;
    add_node(graph, 5, 5); add_node(graph, 5, 6);
    for (const auto [r,c] : {std::pair{4,5}, {6,5}, {5,4}, {5,7}}) add_node(graph, r, c);
    add_path(graph, {0, 1});
    for (int i = 2; i < 6; ++i) add_path(graph, {0, i, 1});
    const auto nets = std::Vector<RoutingNet>{net(graph, 0, 0, 1), net(graph, 1, 0, 1)};
    auto owners = build_owners(nets);
    auto interposer = hardware::Interposer{};
    const auto original = RoutingBox{4, 6, 4, 7};
    const auto expected = std::Vector<RoutingBox>{{3, 6, 4, 7}, {4, 7, 4, 7},
                                                 {4, 6, 3, 7}, {4, 6, 4, 8}};
    for (int side = 0; side < 4; ++side) {
        auto states = states_for(graph, nets);
        owners[0].demand_paths[0] = {0, side+2, 1};
        owners[1].demand_paths[0] = owners[0].demand_paths[0];
        auto resources = ResourceModel{};
        for (auto& owner : owners) air_refresh(graph, owner, resources);
        require(resources.overflow() > 0, "fixture deliberately has cross-net overflow");
        require(air_update_router_boxes(graph, nets, owners, states, {false, false}, &interposer) == 0,
                "unchanged nets do not grow even if their paths touch an edge");
        require(air_update_router_boxes(graph, nets, owners, states, {true, false}, &interposer) == 1,
                "a complete internally valid net can grow during global overflow repair");
        require(states[0].spatial.router_bb == expected[side]
                    && states[1].spatial.router_bb == original,
                "only the touched side of the rerouted net expands by exactly one");
        require(air_update_router_boxes(graph, nets, owners, states, {true, false}, &interposer) == 0,
                "one-cell positive margin must not trigger another expansion");
    }
    auto states = states_for(graph, nets);
    owners[0].demand_paths[0] = {0, 1};
    require(air_update_router_boxes(graph, nets, owners, states, {true, false}, &interposer) == 0,
            "a complete result strictly inside all edges does not expand");
    owners[0].demand_paths[0] = {0, 2};
    require(air_update_router_boxes(graph, nets, owners, states, {true, false}, &interposer) == 0,
            "a nonempty but incomplete root prefix must not grow the box");
    owners[0].demand_paths[0] = {0, 2, 0, 1};
    require(air_update_router_boxes(graph, nets, owners, states, {true, false}, &interposer) == 0,
            "an internally invalid complete path must not grow the box");
    owners[0].demand_paths[0] = {0, 2, 1};
    states[0].spatial.router_bb = {5, 6, 4, 7};
    air_update_router_boxes(graph, nets, owners, states, {true, false}, &interposer);
    require(states[0].spatial.router_bb.r0 == 5,
            "the requested criterion is exactly zero, not VPR's nonpositive margin");

    // Current tree geometry includes non-Track nodes too.
    const int h = add_node(graph); graph.nodes[h].kind = UnifiedNodeKind::HLine;
    add_path(graph, {0, h, 1});
    owners[0].demand_paths[0] = {0, h, 1};
    states[0].spatial.router_bb = {1, 6, 0, 7}; // TOB0 anchor is (1,0).
    air_update_router_boxes(graph, nets, owners, states, {true, false}, &interposer);
    require(states[0].spatial.router_bb == RoutingBox{0, 6, 0, 7},
            "bbox uses all current path nodes and clips expansion at physical edges");
}

auto test_fixed_bbox_failure_and_sync_search() -> void {
    auto graph = UnifiedGraph{}; graph.rows = 10; graph.cols = 10;
    add_node(graph, 3, 3); add_node(graph, 3, 5); add_node(graph, 6, 4);
    add_path(graph, {0, 2, 1});
    const auto n = net(graph, 0, 0, 1);
    auto owners = build_owners({n});
    auto states = states_for(graph, {n}); auto resources = ResourceModel{};
    const auto before = states[0].spatial.router_bb;
    require(!air_route_owner(graph, n, owners[0], resources, states[0].spatial),
            "a failed connection must not report a successful reroute");
    auto interposer = hardware::Interposer{};
    require(owners[0].demand_paths[0].empty()
                && air_update_router_boxes(graph, {n}, owners, states, {true}, &interposer) == 0
                && states[0].spatial.router_bb == before,
            "no in-search expansion, failed-net expansion or full-device fallback");
    auto params = RrrParams{}; params.search_box = before;
    bool rejected = false;
    try { (void)route_demand(graph, resources, owners[0].id, {0}, 1, params); }
    catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "the sync base maze must also respect the fixed per-net box");
    auto sync = n; sync.is_sync_bus = true;
    owners = build_owners({sync});
    require(!air_route_sync(0, graph, {sync}, owners, resources, params, &interposer)
                && owners[0].demand_paths[0].empty(),
            "failed boxed sync routing must release the entire group");
}

auto test_bbox_growth_used_next_round() -> void {
    auto graph = UnifiedGraph{}; graph.rows = 10; graph.cols = 10;
    add_node(graph, 3, 3); add_node(graph, 3, 6);
    add_node(graph, 4, 4); add_node(graph, 4, 5); add_node(graph, 5, 4);
    add_path(graph, {0, 2, 3, 1}); // First round: touches upper row=4.
    add_path(graph, {0, 4, 1});    // Shorter, only available once row=5 is allowed.
    const auto nets = std::Vector<RoutingNet>{net(graph, 0, 0, 1)};
    auto params = RrrParams{}; params.time_budget_seconds = 0.02;
    auto interposer = hardware::Interposer{};
    const auto result = run_rrr(graph, nets, params, &interposer);
    require(result.status == "success" && result.total_wirelength == 3 && result.iterations >= 2,
            "the expanded persistent box must affect the next round, improving the saved legal route");
    require(validate_rrr_solution(graph, nets, result, &interposer), "expanded-box result must remain legal");
}

auto test_sync_tail_box_and_group_bounds() -> void {
    auto graph = UnifiedGraph{}; graph.rows = 10; graph.cols = 10;
    for (const auto [r,c] : {std::pair{2,2}, {2,3}, {2,4}, {5,4},
                            {3,2}, {3,3}, {3,4}, {3,5}}) add_node(graph, r, c);
    add_path(graph, {0, 1, 2}); add_path(graph, {0, 1, 3, 2}); add_path(graph, {4, 5, 6, 7});
    auto lanes = std::Vector<SyncLaneState>{{{0,0}, false, {0}, 2, {0,1,2}},
                                          {{0,1}, false, {4}, 7, {4,5,6,7}}};
    auto resources = ResourceModel{};
    for (const auto& lane : lanes) resources.claim(lane.id, path_resource_keys(graph, lane.path));
    auto interposer = hardware::Interposer{};
    auto params = RrrParams{}; params.r_sequence = {1.0};
    params.search_box = RoutingBox{1, 4, 1, 6};
    require(!equalize_sync_group(graph, resources, params, &interposer, lanes),
            "sync tail must not take an exact-length detour outside the current search box");
    require(lanes[0].path == std::Vector<int>{0,1,2}, "failed boxed equalization must restore prior paths");
    params.search_box.reset();
    require(equalize_sync_group(graph, resources, params, &interposer, lanes),
            "fixture has a valid equalizing detour when no box is imposed");

    auto sync = net(graph, 0, 0, 2); sync.is_sync_bus = true;
    sync.sources.push_back(ref(graph, 4)); sync.demands.push_back({1, ref(graph, 7), {1}, true});
    auto owners = build_owners({sync});
    owners[0].demand_paths[0] = lanes[0].path; owners[1].demand_paths[0] = lanes[1].path;
    auto states = states_for(graph, {sync});
    states[0].spatial.router_bb = {1, 5, 1, 6};
    require(air_update_router_boxes(graph, {sync}, owners, states, {true}, &interposer) == 1
                && states[0].spatial.router_bb.r1 == 6,
            "a sync group's bbox includes every lane and grows as one logical net");
    owners[0].demand_paths[0] = {0,1,2}; states[0].spatial.router_bb.r1 = 3;
    require(air_update_router_boxes(graph, {sync}, owners, states, {true}, &interposer) == 0,
            "an unequal sync group is not a valid result for bbox growth");
}

auto test_sync_internal_conflict_does_not_expand() -> void {
    auto graph = UnifiedGraph{}; graph.rows = 10; graph.cols = 10;
    for (const auto [r,c] : {std::pair{3,3}, {1,0}, {4,3}, {1,0},
                            {1,0}, {1,0}, {1,0}, {1,0}}) add_node(graph, r, c);
    for (int i : {1, 3}) {
        graph.nodes[i].kind = UnifiedNodeKind::Bump;
        graph.nodes[i].bump = {0, 0, 0, static_cast<std::size_t>(i)};
        graph.bump_node_by_key[graph.nodes[i].bump] = i;
    }
    graph.nodes[4].kind = graph.nodes[5].kind = UnifiedNodeKind::VLine;
    graph.nodes[6].kind = graph.nodes[7].kind = UnifiedNodeKind::HLine;
    add_path(graph, {0,4,6,1}); add_path(graph, {2,5,7,3});
    graph.arcs[0].mode_group_id = graph.arcs[3].mode_group_id = 77;
    graph.arcs[0].is_vline_track_straight = true;
    graph.arcs[3].is_vline_track_swap = true;
    auto sync = net(graph, 0, 0, 1); sync.is_sync_bus = true;
    auto sink_a = GraphNodeRef{}, sink_b = GraphNodeRef{};
    sink_a.kind = sink_b.kind = GraphNodeRef::Kind::Bump;
    sink_a.bump = graph.nodes[1].bump; sink_b.bump = graph.nodes[3].bump;
    sync.demands[0].sink = sink_a;
    sync.sources.push_back(ref(graph, 2)); sync.demands.push_back({1, sink_b, {1}, true});
    auto owners = build_owners({sync});
    owners[0].demand_paths[0] = {0,4,6,1}; owners[1].demand_paths[0] = {2,5,7,3};
    auto resources = ResourceModel{};
    for (auto& owner : owners) {
        require(owner_paths_valid(graph, owner), "each fixture lane must be internally legal on its own");
        air_refresh(graph, owner, resources);
    }
    require(resources.overflow(mode_conflict_key(77)) != 0, "the group has an internal straight/swap conflict");
    auto states = states_for(graph, {sync}); const auto before = RoutingBox{1,5,0,7};
    states[0].spatial.router_bb = before;
    auto interposer = hardware::Interposer{};
    require(air_update_router_boxes(graph, {sync}, owners, states, {true}, &interposer) == 0
                && states[0].spatial.router_bb == before,
            "equal-length lanes with a group-internal conflict are not a valid result for expansion");
}

auto test_nearby_total_port_threshold() -> void {
    for (int sinks : {14, 15}) {
        auto graph = UnifiedGraph{};
        add_node(graph, 0, 0); add_node(graph, 0, 99);
        add_node(graph, 0, 99, 8); // A different Track in the exact same bin as the sink.
        for (int i = 3; i <= sinks; ++i) add_node(graph, 0, 100+i);
        const int branch = add_node(graph, 0, 98);
        auto n = net(graph, 0, 0, 1); n.demands.clear();
        for (int i = 2; i <= sinks; ++i) {
            n.demands.push_back({n.demands.size(), ref(graph, i), {0}, true});
            add_path(graph, {0, i});
        }
        n.demands.push_back({n.demands.size(), ref(graph, 1), {0}, true});
        add_path(graph, {0, 1}); add_path(graph, {2, branch, 1});
        auto owners = build_owners({n});
        for (int i = 0; i < sinks-1; ++i) owners[0].demand_paths[i] = {0, i+2};
        auto resources = ResourceModel{}; auto spatial = air_spatial(graph, n);
        require(air_route_owner(graph, n, owners[0], resources, spatial), "missing sink must reconnect");
        const auto expected = sinks == 15 ? std::Vector<int>{0,2,branch,1} : std::Vector<int>{0,1};
        require(owners[0].demand_paths.back() == expected,
                "nearby routing activates at total ports=16 (fifteen sinks), not sink count=16");
    }
}

auto test_nearby_fallback() -> void {
    auto graph = UnifiedGraph{};
    add_node(graph, 0, 0); add_node(graph, 0, 99); add_node(graph, 0, 98);
    for (int i = 3; i <= 16; ++i) add_node(graph, 0, 100+i);
    add_node(graph, 0, 50);
    auto n = net(graph, 0, 0, 2); n.demands.clear();
    for (int i = 2; i <= 16; ++i) {
        n.demands.push_back({n.demands.size(), ref(graph, i), {0}, true});
        add_path(graph, {0, i});
    }
    n.demands.push_back({15, ref(graph, 1), {0}, true});
    add_path(graph, {0, 17, 1});
    auto owners = build_owners({n});
    for (int i = 0; i < 15; ++i) owners[0].demand_paths[i] = {0, i+2};
    auto resources = ResourceModel{}; auto spatial = air_spatial(graph, n);
    air_refresh(graph, owners[0], resources, &spatial);
    const auto bin = spatial.bin(0, 99);
    require(spatial.bins.at(bin) == std::Vector<int>{2}, "nearby bin must contain only the dead-end leaf");
    require(air_connection(graph, n, owners[0], 15, resources, spatial, {2}).empty(),
            "nearby-only attempt must fail in this fixture");
    air_route_owner(graph, n, owners[0], resources, spatial);
    require(owners[0].demand_paths[15] == std::Vector<int>{0, 17, 1},
            "failed nearby search must fall back to the entire tree");

    // Reconnect a pruned sink through an existing branch, storing a full root path.
    add_node(graph, 0, 97); add_path(graph, {2, 18});
    n.demands.push_back({16, ref(graph, 18), {0}, true});
    owners[0].demand_indices.push_back(16); owners[0].demand_paths.push_back({0, 2});
    air_route_owner(graph, n, owners[0], resources, spatial);
    require(owners[0].demand_paths[16] == std::Vector<int>{0, 2, 18},
            "incremental branch must reconstruct full root-to-sink path");
}

auto test_wirelength_candidates() -> void {
    const auto value = fixture();
    auto owners = build_owners(value.nets);
    owners[0].demand_paths[0] = {0, 6, 7, 8, 1};
    owners[1].demand_paths[0] = {2, 4, 5, 3};
    auto states = states_for(value.graph, value.nets);
    states[0].reference = 4; states[1].reference = 4;
    require(air_wirelength_candidates(value.graph, value.nets, owners, states) == std::Vector<std::size_t>{0},
            "any strict excess over uncongested reference selects the net");
    states[0].reference = 5;
    require(air_wirelength_candidates(value.graph, value.nets, owners, states) == std::Vector<std::size_t>{0, 1},
            "without multiport nets, reference-equal routing must still retry all nets");
    auto nets = value.nets; nets[1].demands.push_back(nets[1].demands.front());
    owners[1].demand_indices.push_back(1); owners[1].demand_paths.push_back(owners[1].demand_paths.front());
    require(air_wirelength_candidates(value.graph, nets, owners, states) == std::Vector<std::size_t>{1},
            "reference-equal routing prioritizes multiport nets before all-net fallback");
    nets = value.nets;
    nets[1].sources.push_back(nets[1].sources.front());
    require(air_wirelength_candidates(value.graph, nets, owners, states) == std::Vector<std::size_t>{1},
            "a multi-source PN net is also multiport even with just one sink");
}

auto test_cli_budget() -> void {
    require(parse_rrr_cli({"case", "--time-budget-seconds", "1.25"}).time_budget_seconds == 1.25,
            "fractional positive budget must parse");
    require(parse_rrr_cli({"case"}).time_budget_seconds == 0, "zero retains disabled-budget semantics");
    for (const auto bad : {"-1", "nan", "inf", "x", "1s", "1e999", ""}) {
        bool rejected = false;
        try { (void)parse_rrr_cli({"case", "--time-budget-seconds", bad}); }
        catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "invalid time budget must be rejected");
    }
}

} // namespace

auto run_rrr_budget_unit_tests() -> void {
    test_cli_budget();
    test_budget_repair_and_incumbent();
    test_root_prefix_pruning();
    test_sync_atomic_pruning();
    test_spatial_bin_bias_and_initial_box();
    test_round_boundary_bbox_updates();
    test_fixed_bbox_failure_and_sync_search();
    test_bbox_growth_used_next_round();
    test_sync_tail_box_and_group_bounds();
    test_sync_internal_conflict_does_not_expand();
    test_nearby_total_port_threshold();
    test_nearby_fallback();
    test_wirelength_candidates();
    std::cout << "FPIA_RRR_unit: AIR budget/pruning/search tests passed\n";
}
