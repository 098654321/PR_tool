#include "test/rrr_budget_cases.hh"

#include "maze_search.hh"
#include "net_adapter.hh"
#include "rrr_cli.hh"
#include "rrr_router.hh"
#include "route_validate.hh"
#include "rrr_routing.hh"
#include "transaction.hh"
#include "initial_route.hh"
#include "sync_equalize.hh"

#include <debug/debug.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/tbsnet.hh>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

using namespace PR_tool;

auto require(bool condition, const char* message) -> void {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

auto add_node(UnifiedGraph& graph) -> int {
    const int id = static_cast<int>(graph.nodes.size());
    auto node = UnifiedNode {};
    node.kind = UnifiedNodeKind::Track;
    node.track_row = id;
    graph.nodes.push_back(node);
    graph.track_node_by_key[{0, 0, id, 0, 0}] = id;
    graph.in_arc_ids.emplace_back();
    graph.out_arc_ids.emplace_back();
    return id;
}

auto add_path(UnifiedGraph& graph, std::Vector<int> path) -> void {
    for (std::size_t i = 1; i < path.size(); ++i) {
        const int arc = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back(UnifiedArc {path[i - 1], path[i]});
        graph.out_arc_ids[path[i - 1]].push_back(arc);
        graph.in_arc_ids[path[i]].push_back(arc);
        graph.directed_arc_set.insert({path[i - 1], path[i]});
    }
}

auto ref(const UnifiedGraph& graph, int id) -> GraphNodeRef {
    auto value = GraphNodeRef {};
    value.kind = GraphNodeRef::Kind::Track;
    value.track_coord = hardware::TrackCoord{graph.nodes[id].track_row, 0,
        hardware::TrackDirection::Horizontal, 0};
    return value;
}

auto net(const UnifiedGraph& graph, std::size_t id, int source, int sink) -> RoutingNet {
    auto value = RoutingNet {};
    value.net_id = id;
    value.name = "budget_net_" + std::to_string(id);
    value.sources = {ref(graph, source)};
    value.demands = {RoutingDemand {0, ref(graph, sink), {0}, true}};
    return value;
}

struct Fixture {
    UnifiedGraph graph;
    std::Vector<RoutingNet> nets;
};

auto fixture() -> Fixture {
    auto value = Fixture {};
    for (int i = 0; i < 15; ++i) {
        add_node(value.graph);
    }
    add_path(value.graph, {0, 4, 5, 6, 7, 8, 1});
    add_path(value.graph, {2, 4, 5, 6, 7, 8, 3});
    add_path(value.graph, {0, 9, 10, 11, 12, 13, 14, 1});
    value.nets = {net(value.graph, 0, 0, 1), net(value.graph, 1, 2, 3)};
    return value;
}

auto optimization_fixture() -> Fixture {
    auto value = Fixture {};
    for (int i = 0; i < 24; ++i) {
        add_node(value.graph);
    }
    add_path(value.graph, {0, 4, 5, 6, 7, 8, 1});
    add_path(value.graph, {2, 4, 5, 6, 7, 8, 3});
    add_path(value.graph, {0, 9, 10, 11, 12, 13, 14, 1});
    add_path(value.graph, {2, 16, 17, 18, 19, 20, 21, 22, 23, 3});
    value.nets = {net(value.graph, 0, 0, 1), net(value.graph, 1, 2, 3)};
    return value;
}

auto expired_params() -> RrrParams {
    auto params = RrrParams {};
    params.time_budget_seconds = 0.001;
    params.budget_start = std::chrono::steady_clock::now() - std::chrono::seconds {1};
    params.max_iterations = 50;
    return params;
}

auto check_legal(const Fixture& value, const RrrResult& result, hardware::Interposer* interposer)
    -> void {
    require(result.status == "success", "budget result must succeed");
    require(result.best_overflow == 0 && result.unequal_sync_groups == 0,
            "budget result must have no overflow or unequal SyncNet");
    require(validate_rrr_solution(value.graph, value.nets, result, interposer),
            "budget result must pass independent validation");
    require(result.first_legal_ms >= 0 && result.best_legal_ms >= result.first_legal_ms,
            "legal solution timestamps must be recorded");
}

auto test_initial_type_order() -> void {
    const auto value = fixture();
    auto nets = std::Vector<RoutingNet> {};
    for (std::size_t i = 0; i < 7; ++i) {
        nets.push_back(net(value.graph, i, 0, 1));
    }
    nets[0].kind = RoutingNetKind::PNnet;
    nets[0].sources.resize(4, nets[0].sources.front());
    nets[1].kind = RoutingNetKind::Bnet;
    nets[3].kind = RoutingNetKind::Bnet;
    nets[3].demands.resize(4, nets[3].demands.front());
    nets[4].demands.push_back(RoutingDemand {1, ref(value.graph, 3), {0}, true});
    nets[5].is_sync_bus = true;
    nets[5].demands.push_back(RoutingDemand {1, ref(value.graph, 3), {0}, true});
    nets[6].initial_type_priority = 1; // TrackToBumpsNet can contain just one sink.
    const auto owners = rrr_detail::build_owners(nets);
    const auto old_order = rrr_detail::initial_order(owners);
    const auto typed = rrr_detail::initial_type_order(owners, nets);
    auto ids = [&](const std::Vector<std::size_t>& order) {
        auto result = std::Vector<std::size_t> {};
        for (const auto i : order) {
            result.push_back(owners[i].id.net_id);
        }
        return result;
    };
    require(ids(typed) == std::Vector<std::size_t> {5, 5, 4, 6, 3, 2, 1, 0},
            "initial routing must follow type priorities 0..5 and keep same-type ordering");
    require(owners[typed[0]].id.demand_id == 0 && owners[typed[1]].id.demand_id == 1,
            "initial type ordering must keep SyncNet lanes together in demand order");
    require(ids(old_order) == std::Vector<std::size_t> {5, 5, 0, 3, 4, 1, 2, 6}
                && rrr_detail::initial_order(owners) == old_order,
            "type ordering must not change the legacy ordering used by optimization rounds");

    auto interposer = hardware::Interposer {};
    auto name = std::String {"type_order"};
    auto uid = name;
    auto* bump = interposer.get_bump({0, 0}, 0).value();
    auto* other = interposer.get_bump({0, 0}, 1).value();
    auto* track = interposer.get_track(0, 0, hardware::TrackDirection::Horizontal, 0).value();
    const auto adapted = build_routing_nets({
        std::make_shared<circuit::BumpToBumpNet>(bump, other, std::HashSet<int> {0}, name, uid),
        std::make_shared<circuit::TrackToBumpsNet>(
            track, std::Vector<hardware::Bump*> {bump}, std::HashSet<int> {0}, name, uid)});
    require(adapted[0].initial_type_priority == 4 && adapted[1].initial_type_priority == 1,
            "adapter must preserve original type priority even for a one-sink fanout class");
}

auto test_initial_routing_uses_type_order() -> void {
    auto value = fixture();
    value.nets[0].kind = RoutingNetKind::Bnet;
    auto params = RrrParams {};
    params.s = 1; // Make the first routed net's shared node expensive enough to detour.
    auto interposer = hardware::Interposer {};
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, result, &interposer);
    require(result.iterations >= 1
                && result.paths[0][0] == std::Vector<int> {0, 9, 10, 11, 12, 13, 14, 1}
                && result.paths[1][0] == std::Vector<int> {2, 4, 5, 6, 7, 8, 3},
            "initial run must route Tnet before Bnet, not the legacy net-id order");
}

auto test_cli_budget() -> void {
    const auto options = parse_rrr_cli({"case", "--time-budget-seconds", "1.25"});
    require(options.time_budget_seconds == 1.25, "CLI must parse fractional seconds");
    require(options.max_iterations == kRrrDefaultMaxIterations, "CLI must default to large cap");
    require(parse_rrr_cli({"case"}).time_budget_seconds == 0, "budget defaults to disabled");
    require(parse_rrr_cli({"case", "--time-budget-seconds", "0"}).time_budget_seconds == 0,
            "zero disables budget");
    for (const auto bad : {"-1", "nan", "inf", "x", "1s", "1e999", ""}) {
        bool rejected = false;
        try {
            (void)parse_rrr_cli({"case", "--time-budget-seconds", bad});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "invalid budget must be rejected");
    }
    bool rejected = false;
    try {
        (void)parse_rrr_cli({"case", "--time-budget-seconds"});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "missing budget value must be rejected");
}

auto test_overflow_slope() -> void {
    auto graph = UnifiedGraph {};
    add_node(graph);
    add_node(graph);
    add_path(graph, {0, 1});
    const auto owner = OwnerId {100, 0};
    for (const int height : {4, 16}) {
        auto strong = RrrParams {};
        strong.H = height;
        strong.s = 2;
        auto weak = strong;
        weak.s = 20;
        auto resources = ResourceModel {};
        for (int u = 1; u <= 3; ++u) {
            const double strong_cost = arc_incremental_cost(
                graph, resources, owner, graph.arcs.front(), {0}, strong);
            const double weak_cost = arc_incremental_cost(
                graph, resources, owner, graph.arcs.front(), {0}, weak);
            const double common = 2.0 + height / (std::exp(strong.k * (1 - u)) + 1.0);
            const double excess = static_cast<double>(height) / 2.0 * (u - 1);
            require(std::abs(strong_cost - common - excess) < 1e-9,
                    "s=2 must apply H/2 times overflow without changing the base cost");
            require(std::abs((strong_cost - common) - 10 * (weak_cost - common)) < 1e-9,
                    "s=2 must strengthen only the overflow term tenfold compared with s=20");
            resources.claim(OwnerId {static_cast<std::size_t>(101 + u), 0}, {node_resource(1)});
        }
    }
}

auto test_additive_history_cost() -> void {
    const auto owner = OwnerId {100, 0};
    for (const auto key : {node_resource(1), switch_resource(7), matching_endpoint_key(1, 0),
                           tob_mux_input_key(1, 3), tob_mux_output_key(1, 3)}) {
        auto graph = UnifiedGraph {};
        add_node(graph);
        add_node(graph);
        add_path(graph, {0, 1});
        auto& arc = graph.arcs.front();
        arc.resource_keys_ready = true;
        arc.resource_keys.count = 1;
        arc.resource_keys.values[0] = key;
        for (const int height : {0, 4, 16}) {
            auto params = RrrParams {};
            params.H = height;
            params.history_weight = 1.75;
            auto resources = ResourceModel {params};
            auto other = key;
            if (is_mux_port_key(key)) {
                other.extra = 4;
            }
            resources.claim(OwnerId {1, 0}, {key});
            resources.claim(OwnerId {2, 0}, {other});
            resources.history_next();
            resources.release(OwnerId {1, 0});
            resources.release(OwnerId {2, 0});
            const double history = resources.history(key);
            require(history == 1, "fixture must retain congestion history after rip-up");
            for (int u = 1; u <= 3; ++u) {
                const double penalty = 1 + height / (std::exp(1.0 - u) + 1)
                    + static_cast<double>(height) / params.s * (u - 1);
                const double present = resources.type_weight(key) * penalty;
                const double actual = arc_incremental_cost(graph, resources, owner, arc, {0}, params);
                const double expected = 1 + present + params.history_weight * history;
                require(std::abs(actual - expected) < 1e-9,
                        "capacity and mux resources must use unscaled additive history at every occupancy");
                other = key;
                if (is_mux_port_key(key)) {
                    other.extra = 10 + u;
                }
                resources.claim(OwnerId {static_cast<std::size_t>(101 + u), 0}, {other});
            }
            resources.claim(owner, {key});
            require(arc_incremental_cost(graph, resources, owner, arc, {0}, params) == 1,
                    "same-owner tree and exact mux reuse must retain zero incremental resource cost");
        }
    }
}

auto test_original_mode_cost() -> void {
    const auto owner = OwnerId {100, 0};
    for (const bool straight : {true, false}) {
        auto graph = UnifiedGraph {};
        add_node(graph);
        add_node(graph);
        graph.nodes[0].kind = UnifiedNodeKind::VLine;
        graph.nodes[1].kind = UnifiedNodeKind::Track;
        add_path(graph, {0, 1});
        auto& arc = graph.arcs.front();
        arc.mode_group_id = 7;
        arc.is_vline_track_straight = straight;
        arc.is_vline_track_swap = !straight;
        const auto mode = straight ? mode_straight_key(7) : mode_swap_key(7);
        const auto opposite = straight ? mode_swap_key(7) : mode_straight_key(7);
        for (const int height : {4, 16}) {
            auto params = RrrParams {};
            params.H = height;
            params.history_weight = 1.75;
            auto resources = ResourceModel {params};
            const double baseline = arc_incremental_cost(graph, resources, owner, arc, {0}, params);
            resources.claim(OwnerId {1, 0}, {mode});
            require(arc_incremental_cost(graph, resources, owner, arc, {0}, params) == baseline,
                    "compatible mode sharing must not add congestion cost");
            resources.release(OwnerId {1, 0});
            resources.claim(OwnerId {2, 0}, {opposite});
            const double first_penalty = 1 + height / 2.0;
            const double over_penalty = 1 + height / (std::exp(-1.0) + 1)
                + static_cast<double>(height) / params.s;
            require(std::abs(arc_incremental_cost(graph, resources, owner, arc, {0}, params)
                             - baseline - 8 * first_penalty) < 1e-9,
                    "original first mode conflict must use predicted ModeConflict occupancy u=1");
            resources.claim(owner, {mode});
            resources.history_next();
            require(arc_incremental_cost(graph, resources, owner, arc, {0}, params) == baseline,
                    "original same-owner mode reuse must remain exempt from incremental mode cost");
            resources.release(owner);
            const double expected = baseline + 8 * first_penalty + params.history_weight;
            require(std::abs(arc_incremental_cost(graph, resources, owner, arc, {0}, params)
                             - expected) < 1e-9,
                    "original first mode conflict must include unscaled additive history after rip-up");
            resources.claim(OwnerId {3, 0}, {mode});
            require(std::abs(arc_incremental_cost(graph, resources, owner, arc, {0}, params)
                             - baseline - 8 * over_penalty - params.history_weight) < 1e-9,
                    "already-conflicted mode must use predicted ModeConflict occupancy u=2");
            require(route_demand(graph, resources, owner, {0}, 1, params)
                        == std::Vector<int> {0, 1},
                    "mode conflicts must remain finite-cost soft constraints during maze routing");
            resources.release(OwnerId {2, 0});
            resources.release(OwnerId {3, 0});
            resources.claim(owner, {mode});
            require(arc_incremental_cost(graph, resources, owner, arc, {0}, params) == baseline,
                    "compatible mode reuse must not charge old conflict history");
        }
    }
}

auto test_expired_initial_legal() -> void {
    auto value = fixture();
    value.nets.resize(1);
    auto interposer = hardware::Interposer {};
    auto params = expired_params();
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, result, &interposer);
    require(result.stop_reason == "time_budget" && result.iterations == 0,
            "expired budget must stop immediately after legal initial routing");
    require(result.first_legal_ms >= 1000, "budget must include time before run_rrr");
}

auto test_expired_no_legal_repairs() -> void {
    const auto value = fixture();
    auto interposer = hardware::Interposer {};
    auto params = expired_params();
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, result, &interposer);
    require(result.iterations > 0 && result.optimization_rounds == 0,
            "expired infeasible initial solution must be repaired before stopping");
    require(result.stop_reason == "time_budget" && result.first_legal_ms >= 1000,
            "stop only at first legal solution after an expired budget");
}

auto test_optimize_and_restore_best() -> void {
    const auto value = optimization_fixture();
    auto interposer = hardware::Interposer{};
    auto params = RrrParams{}; params.max_iterations = 0;
    const auto initial = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, initial, &interposer);
    params.time_budget_seconds = 3600; params.max_iterations = 2;
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, result, &interposer);
    require(result.iterations == 2 && result.optimization_rounds == 2,
            "complete initial solution must enter transactional optimization");
    require(result.stop_reason == "iteration_limit", "iteration cap must still apply");
    require(result.total_wirelength < initial.total_wirelength,
            "a displacement transaction must improve the initial legal route");
    params.max_iterations = 8;
    const auto longer = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, longer, &interposer);
    require(longer.total_wirelength <= result.total_wirelength,
            "later rejected transactions must not degrade the legal incumbent");
}

auto test_cap_without_legal_and_stagnation() -> void {
    auto value = Fixture{};
    for (int i = 0; i < 5; ++i) add_node(value.graph);
    add_path(value.graph, {0, 4, 1}); add_path(value.graph, {2, 4, 3});
    value.nets = {net(value.graph, 0, 0, 1), net(value.graph, 1, 2, 3)};
    auto params = expired_params(); params.max_iterations = 5;
    auto interposer = hardware::Interposer{};
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    require(result.status == "iteration_limit" && result.best_overflow == 0 && result.missing_owners == 1,
            "zero-overflow partial route must never be success");
    require(result.iterations == 5 && result.first_legal_ms == -1,
            "expired budget without a complete legal solution must continue to cap");
    require(result.paths[0].size() == 1 && result.paths[1].empty(),
            "failed displacement must restore the previous complete owner");
    require(validate_rrr_solution(value.graph, value.nets, result, &interposer, true),
            "retained partial solution must pass partial validation");
}

auto test_sync_optimization() -> void {
    auto value = Fixture{};
    for (int i = 0; i < 9; ++i) add_node(value.graph);
    add_path(value.graph, {0, 4, 5, 1}); add_path(value.graph, {2, 6, 3});
    add_path(value.graph, {2, 7, 8, 3});
    auto sync = net(value.graph, 0, 0, 1); sync.is_sync_bus = true;
    sync.sources.push_back(ref(value.graph, 2));
    sync.demands.push_back(RoutingDemand{1, ref(value.graph, 3), {1}, true});
    value.nets = {sync};
    auto params = RrrParams{}; params.time_budget_seconds = 3600; params.max_iterations = 4;
    auto interposer = hardware::Interposer{};
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, result, &interposer);
    require(result.iterations == 4 && result.optimization_rounds == 4,
            "SyncNet group transactions must run until the configured cap");
    for (const auto& path : result.paths[0])
        require(sync_lane_length(value.graph, path, &interposer, false) == 5,
                "independent shortest max must initialize common SyncNet target");
}

auto test_live_budget_and_legacy() -> void {
    auto value = fixture(); value.nets.resize(1);
    auto interposer = hardware::Interposer{};
    auto params = RrrParams{};
    const auto baseline = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, baseline, &interposer);
    require(baseline.iterations >= 1, "without budget, legal initial route must still try optimization");
    params.time_budget_seconds = 3600; params.max_iterations = 3;
    const auto repeated = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, repeated, &interposer);
    require(repeated.iterations == 3 && repeated.optimization_rounds == 3 && repeated.paths == baseline.paths,
            "unchanged transactions must continue while budget remains");
    params.time_budget_seconds = 0.005; params.max_iterations = 100000;
    debug::set_debug_level(debug::DebugLevel::Warning);
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    debug::set_debug_level(debug::DebugLevel::Info);
    check_legal(value, result, &interposer);
    require(result.stop_reason == "time_budget" && result.budget_elapsed_ms >= 5,
            "live budget must stop safely and retain best legal solution");
}

auto test_unreachable_initial_not_legal() -> void {
    auto value = Fixture {};
    add_node(value.graph);
    add_node(value.graph);
    value.nets = {net(value.graph, 0, 0, 1)};
    auto interposer = hardware::Interposer {};
    auto params = expired_params();
    params.max_iterations = 0;
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    require(result.status != "success" && result.first_legal_ms == -1,
            "unreachable initial routing must not become a legal incumbent");
}

auto test_original_sort_and_interruption() -> void {
    auto interposer = hardware::Interposer{};
    auto* source = interposer.get_track(0, 0, hardware::TrackDirection::Horizontal, 0).value();
    auto* b0 = interposer.get_bump(0, 0, 0).value();
    auto* b1 = interposer.get_bump(0, 0, 1).value();
    auto* b2 = interposer.get_bump(0, 0, 2).value();
    auto name0 = std::String{"two_sink"}, uid0 = name0;
    auto name1 = std::String{"one_sink"}, uid1 = name1;
    const auto raw = std::Vector<std::Rc<circuit::Net>>{
        std::make_shared<circuit::TrackToBumpsNet>(source,
            std::Vector<hardware::Bump*>{b0, b1}, std::HashSet<int>{0}, name0, uid0),
        std::make_shared<circuit::TrackToBumpsNet>(source,
            std::Vector<hardware::Bump*>{b2}, std::HashSet<int>{0}, name1, uid1)};
    const auto nets = build_routing_nets(raw);
    const auto graph = build_hardware_graph(&interposer, nets);
    auto owners = rrr_detail::build_owners(nets);
    auto resources = ResourceModel{};
    const auto idle = interposer.adjacent_idle_tracks(source).size();
    const auto order = rrr_detail::original_initial_route(graph, nets, owners,
        resources, RrrParams{}, &interposer, raw);
    require(order == std::Vector<std::size_t>{1, 0},
            "original Sort must use exact priority/port bias, not old RRR port/HPWL order");
    require(rrr_detail::owner_connected(owners[1]) && !rrr_detail::owner_connected(owners[0]),
            "initialization must keep success prefix and roll back first conflicting fanout");
    require(resources.overflow() == 0 && interposer.adjacent_idle_tracks(source).size() == idle,
            "raw failure must not leak hardware reservations into RRR");
    auto* remote_lu = interposer.get_cob({7, 7}).value()->sw_register(
        hardware::COBDirection::Left, 0, hardware::COBDirection::Up);
    remote_lu->set(hardware::COBSwState::Suspended); // Failed branch not yet in a package.
    rrr_detail::clear_initial_reservations(&interposer);
    require(!remote_lu->is_occupied(), "cleanup must also clear unrecorded remote Left-Up reservations");
}

auto test_shortest_sync_targets_are_independent() -> void {
    auto value = Fixture{};
    for (int i = 0; i < 8; ++i) add_node(value.graph);
    add_path(value.graph, {0, 4, 5, 1});
    add_path(value.graph, {2, 4, 5, 3});
    auto sync = net(value.graph, 0, 0, 1); sync.is_sync_bus = true;
    sync.sources.push_back(ref(value.graph, 2));
    sync.demands.push_back(RoutingDemand{1, ref(value.graph, 3), {1}, true});
    value.nets = {sync};
    auto owners = rrr_detail::build_owners(value.nets);
    auto interposer = hardware::Interposer{};
    rrr_detail::initialize_sync_targets(value.graph, value.nets, owners, RrrParams{}, &interposer);
    require(owners[0].sync_target == 5 && owners[1].sync_target == 5,
            "overlapping independent shortest paths must not block one another during precompute");
    require(rrr_detail::missing_owners(owners) == 2 && owners[0].claimed.empty() && owners[1].claimed.empty(),
            "shortest-length precomputation must never commit a path or occupancy");

    auto metric = Fixture{};
    for (int i = 0; i < 7; ++i) add_node(metric.graph);
    for (int i : {2, 3, 4}) metric.graph.nodes[i].kind = UnifiedNodeKind::HLine;
    add_path(metric.graph, {0, 2, 3, 4, 1}); // 2 Tracks, but more graph hops.
    add_path(metric.graph, {0, 5, 6, 1});    // Fewer hops/cost, but 4 Tracks.
    auto lane = net(metric.graph, 0, 0, 1); lane.is_sync_bus = true;
    metric.nets = {lane};
    auto metric_owners = rrr_detail::build_owners(metric.nets);
    rrr_detail::initialize_sync_targets(metric.graph, metric.nets, metric_owners, RrrParams{}, &interposer);
    require(metric_owners[0].sync_target == 3,
            "precompute must minimize Track length, not graph hops or weighted FPIA cost");
    auto model = ResourceModel{};
    auto params = RrrParams{}; params.r_sequence = {1.0};
    require(rrr_detail::route_sync_group(0, metric_owners, metric.graph, metric.nets,
        model, params, &interposer), "precomputed target must be tried during actual equalization");
    require(metric_owners[0].sync_target == 3 && metric_owners[0].demand_paths[0] == std::Vector<int>{0,2,3,4,1},
            "weighted maze's longer path must not silently override the precomputed target");

    auto name = std::String{"same_tob_sync"}, uid = name;
    auto* b0 = interposer.get_bump(0, 0, 0).value();
    auto* b1 = interposer.get_bump(0, 0, 1).value();
    auto member = std::make_shared<circuit::BumpToBumpNet>(b0, b1, std::HashSet<int>{0}, name, uid);
    auto group = std::make_shared<circuit::SyncNet>(
        std::Vector<std::Rc<circuit::BumpToBumpNet>>{member},
        std::Vector<std::Rc<circuit::BumpToTrackNet>>{},
        std::Vector<std::Rc<circuit::TrackToBumpNet>>{}, std::HashSet<int>{0}, name, uid);
    const auto real_nets = build_routing_nets({group});
    const auto real_graph = build_hardware_graph(&interposer, real_nets);
    auto real_owners = rrr_detail::build_owners(real_nets);
    rrr_detail::initialize_sync_targets(real_graph, real_nets, real_owners, RrrParams{}, &interposer);
    require(real_owners[0].sync_target > 2,
            "same-TOB shortest path must reject zero-Track Bump-HLine-Bump shorts");
}

auto test_original_sync_initial_paths_are_preserved() -> void {
    for (int kind = 0; kind < 3; ++kind) {
        auto interposer = hardware::Interposer{};
        auto name = std::String{"original_sync_initial"}, uid = name;
        auto* bump = interposer.get_bump(0, 0, 0).value();
        auto* other = interposer.get_bump(0, 1, 0).value();
        auto* track = interposer.get_track(0, 0, hardware::TrackDirection::Horizontal, 0).value();
        auto btb = std::Vector<std::Rc<circuit::BumpToBumpNet>>{};
        auto btt = std::Vector<std::Rc<circuit::BumpToTrackNet>>{};
        auto ttb = std::Vector<std::Rc<circuit::TrackToBumpNet>>{};
        std::Rc<circuit::Net> member;
        if (kind == 0) {
            btb.push_back(std::make_shared<circuit::BumpToBumpNet>(bump, other, std::HashSet<int>{0}, name, uid));
            member = btb.front();
        } else if (kind == 1) {
            btt.push_back(std::make_shared<circuit::BumpToTrackNet>(bump, track, std::HashSet<int>{0}, name, uid));
            member = btt.front();
        } else {
            ttb.push_back(std::make_shared<circuit::TrackToBumpNet>(track, bump, std::HashSet<int>{0}, name, uid));
            member = ttb.front();
        }
        auto sync = std::make_shared<circuit::SyncNet>(btb, btt, ttb, std::HashSet<int>{0}, name, uid);
        const auto raw = std::Vector<std::Rc<circuit::Net>>{sync};
        const auto nets = build_routing_nets(raw);
        const auto graph = build_hardware_graph(&interposer, nets);
        auto owners = rrr_detail::build_owners(nets);
        rrr_detail::initialize_sync_targets(graph, nets, owners, RrrParams{}, &interposer);
        sync->check_accessable_cobunit(); interposer.manage_cobunit_resources();
        const auto strategy = algo::MazeRouteStrategy{};
        sync->route(&interposer, strategy);
        auto expected = owners.front();
        rrr_detail::import_package(expected, member->pathpackage(), graph, nets.front(), &interposer);
        require(sync_lane_length(graph, expected.demand_paths[0], &interposer, expected.is_bnet)
                == expected.sync_target, "original one-lane fixture must achieve the independent target");
        sync->clear_current_package(); rrr_detail::clear_initial_reservations(&interposer);
        auto resources = ResourceModel{};
        rrr_detail::original_initial_route(graph, nets, owners, resources, RrrParams{}, &interposer, raw);
        require(owners.front().demand_paths == expected.demand_paths
                && owners.front().sync_target == expected.sync_target,
                "successful initialization must preserve original Btb/Btt/Ttb paths and target exactly");
        auto candidate = RrrResult{}; candidate.paths = rrr_detail::collect_paths_by_net(nets, owners);
        require(resources.overflow() == 0 && validate_rrr_solution(graph, nets, candidate, &interposer),
                "imported original synchronous paths must pass full independent validation");
        require(sync->pathpackage()._regular_path.empty() && member->pathpackage()._regular_path.empty()
                && interposer.is_idle_tracks(track), "sync initialization must clear temporary reservations");

        // Exercise clear/equalize/export without rerunning the first FPIA maze.
        // With no explicit target, the existing equalizer uses the snapshot maximum.
        owners = rrr_detail::build_owners(nets); resources = ResourceModel{};
        rrr_detail::original_initial_route(graph, nets, owners, resources, RrrParams{}, &interposer, raw);
        require(owners.front().sync_target == expected.sync_target
                && owners.front().demand_paths == expected.demand_paths,
                "FPIA equalization must start from the imported original snapshot, including Btt reversal");
        candidate.paths = rrr_detail::collect_paths_by_net(nets, owners);
        require(resources.overflow() == 0 && validate_rrr_solution(graph, nets, candidate, &interposer)
                && member->pathpackage()._regular_path.empty() && interposer.is_idle_tracks(track),
                "equalized sync export must remain legal and leave no temporary reservations");
    }
}

} // namespace

auto run_rrr_budget_unit_tests() -> void {
    test_initial_type_order();
    test_initial_routing_uses_type_order();
    test_original_sort_and_interruption();
    test_original_sync_initial_paths_are_preserved();
    test_shortest_sync_targets_are_independent();
    test_cli_budget();
    test_overflow_slope();
    test_additive_history_cost();
    test_original_mode_cost();
    test_unreachable_initial_not_legal();
    test_expired_initial_legal();
    test_expired_no_legal_repairs();
    test_optimize_and_restore_best();
    test_cap_without_legal_and_stagnation();
    test_sync_optimization();
    test_live_budget_and_legacy();
    std::cout << "FPIA_RRR_unit: budget regression passed\n";
}
