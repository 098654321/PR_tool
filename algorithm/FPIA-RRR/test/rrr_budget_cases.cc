#include "test/rrr_budget_cases.hh"

#include "maze_search.hh"
#include "rrr_cli.hh"
#include "rrr_router.hh"
#include "route_validate.hh"
#include "rrr_routing.hh"

#include <debug/debug.hh>

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
    node.kind = UnifiedNodeKind::Bump;
    node.bump = Bump_coord {0, 0, 0, static_cast<std::size_t>(id)};
    graph.nodes.push_back(node);
    graph.bump_node_by_key[node.bump] = id;
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
    value.kind = GraphNodeRef::Kind::Bump;
    value.bump = graph.nodes[id].bump;
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
    for (int i = 0; i < 10; ++i) {
        add_node(value.graph);
    }
    add_path(value.graph, {0, 4, 1});
    add_path(value.graph, {2, 4, 3});
    add_path(value.graph, {0, 5, 6, 1});
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
    add_path(value.graph, {0, 9, 10, 11, 12, 13, 14, 15, 1});
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
    params.H = 0;
    params.increment = 10;
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, result, &interposer);
    require(result.iterations > 0 && result.optimization_rounds == 0,
            "expired infeasible initial solution must be repaired before stopping");
    require(result.stop_reason == "time_budget" && result.first_legal_ms >= 1000,
            "stop only at first legal solution after an expired budget");
}

auto test_optimize_and_restore_best() -> void {
    const auto value = optimization_fixture();
    auto interposer = hardware::Interposer {};
    auto baseline_params = RrrParams {};
    baseline_params.H = 0;
    baseline_params.decay = 0;
    baseline_params.increment = 10;
    baseline_params.max_iterations = 50;
    const auto baseline = run_rrr(value.graph, value.nets, baseline_params, &interposer);
    check_legal(value, baseline, &interposer);

    auto params = baseline_params;
    params.time_budget_seconds = 3600;
    params.max_iterations = 2;
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, result, &interposer);
    require(result.iterations == 2 && result.optimization_rounds == 1,
            "a repaired legal solution must enter a full optimization round");
    require(result.stop_reason == "iteration_limit", "explicit iteration cap must still apply");
    require(result.total_wirelength <= baseline.total_wirelength,
            "later rounds must never degrade the returned legal incumbent");

    // Recreate the rotated round to ensure this test actually exercises a worse/illegal state.
    auto owners = rrr_detail::build_owners(value.nets);
    auto resources = ResourceModel {params};
    rrr_detail::route_owner(owners[1], value.graph, value.nets, resources, params, &interposer);
    rrr_detail::route_owner(owners[0], value.graph, value.nets, resources, params, &interposer);
    require(resources.overflow() > 0
                || rrr_detail::current_wirelength(value.graph, value.nets, owners) > baseline.total_wirelength,
            "rotated fixture must produce a worse or conflicted trial");
    require(result.paths == baseline.paths, "iteration cap must restore the earlier best legal paths");

    params.max_iterations = 12;
    const auto longer = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, longer, &interposer);
    require(longer.iterations == 12 && longer.optimization_rounds > 1,
            "optimization and repair must continue while time remains");
    require(longer.total_wirelength <= baseline.total_wirelength,
            "multiple rounds must retain best legal wirelength");
}

auto test_cap_without_legal_and_stagnation() -> void {
    auto value = fixture();
    // Remove the only alternative; both owners must use node 4.
    value.graph = UnifiedGraph {};
    for (int i = 0; i < 5; ++i) {
        add_node(value.graph);
    }
    add_path(value.graph, {0, 4, 1});
    add_path(value.graph, {2, 4, 3});
    value.nets = {net(value.graph, 0, 0, 1), net(value.graph, 1, 2, 3)};
    auto params = expired_params();
    params.max_iterations = 30;
    params.stagnation_limit = 1;
    auto interposer = hardware::Interposer {};
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    require(result.status == "iteration_limit" && result.best_overflow > 0,
            "iteration cap without feasible solution must report failure");
    require(result.iterations == 30 && result.first_legal_ms == -1,
            "time-budget mode must not exit early due to stagnation");
}

auto test_sync_optimization() -> void {
    auto value = Fixture {};
    // The normal net has no alternative; only the SyncNet can take the longer route.
    for (int i = 0; i < 8; ++i) {
        add_node(value.graph);
    }
    add_path(value.graph, {0, 4, 1});
    add_path(value.graph, {2, 4, 3});
    value.nets = {net(value.graph, 0, 0, 1)};
    auto sync = net(value.graph, 1, 2, 3);
    sync.is_sync_bus = true;
    const int s = add_node(value.graph);
    const int t = add_node(value.graph);
    add_path(value.graph, {s, t});
    add_path(value.graph, {2, 5, 6, 7, 3});
    sync.sources.push_back(ref(value.graph, s));
    sync.demands.push_back(RoutingDemand {1, ref(value.graph, t), {1}, true});
    value.nets.push_back(sync);
    auto params = RrrParams {};
    params.time_budget_seconds = 3600;
    params.max_iterations = 4;
    params.H = 0;
    params.increment = 10;
    auto interposer = hardware::Interposer {};
    const auto result = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, result, &interposer);
    require(result.iterations == 4 && result.optimization_rounds > 0,
            "SyncNet optimization must continue with whole-group scheduling");
    require(result.paths[1].size() == 2, "SyncNet group must retain both lanes");
}

auto test_live_budget_and_legacy() -> void {
    auto value = fixture();
    value.nets.resize(1);
    auto interposer = hardware::Interposer {};
    auto params = RrrParams {};
    const auto legacy = run_rrr(value.graph, value.nets, params, &interposer);
    require(legacy.iterations == 0 && legacy.stop_reason == "feasible",
            "disabled budget must preserve first-legal stopping");
    params.time_budget_seconds = 3600;
    params.max_iterations = 12;
    const auto repeated = run_rrr(value.graph, value.nets, params, &interposer);
    check_legal(value, repeated, &interposer);
    require(repeated.stop_reason == "iteration_limit"
                && repeated.iterations == 12 && repeated.optimization_rounds == 12,
            "unchanged shortest legal routes must continue full optimization while budget remains");
    require(repeated.paths == legacy.paths, "unchanged full reroutes must preserve the best legal paths");

    const auto optimizable = optimization_fixture();
    params.H = 0;
    params.increment = 10;
    params.decay = 0;
    params.time_budget_seconds = 0.005;
    params.max_iterations = 100000;
    debug::set_debug_level(debug::DebugLevel::Warning);
    const auto result = run_rrr(optimizable.graph, optimizable.nets, params, &interposer);
    debug::set_debug_level(debug::DebugLevel::Info);
    check_legal(optimizable, result, &interposer);
    require(result.stop_reason == "time_budget" && result.budget_elapsed_ms >= 5,
            "positive live budget must stop after a completed round");
    require(result.optimization_rounds > 0, "live budget must spend time on full optimization");

    params = RrrParams {};
    params.time_budget_seconds = 0.005;
    params.max_iterations = 100000;
    debug::set_debug_level(debug::DebugLevel::Warning);
    const auto flat = run_rrr(value.graph, value.nets, params, &interposer);
    debug::set_debug_level(debug::DebugLevel::Info);
    check_legal(value, flat, &interposer);
    require(flat.stop_reason == "time_budget" && flat.budget_elapsed_ms >= 5
                && flat.optimization_rounds > 1 && flat.paths == legacy.paths,
            "unchanged legal routes must keep optimizing until the live time budget expires");
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

} // namespace

auto run_rrr_budget_unit_tests() -> void {
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
