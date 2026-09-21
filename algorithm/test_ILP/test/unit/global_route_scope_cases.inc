// TOB peak objective, guide application, distance initialization, scope repair,
// scope expansion, feedback thresholds, and guide logging tests.

auto test_v21_tob_peak_objective() -> void {
    const auto make_fixture = [](const std::size_t bump_count) {
        const auto anchor = tob_anchor_cob(0);
        const auto source = track_ref(
            0,
            0,
            hardware::TrackDirection::Horizontal,
            track_from_unit_inner(0, 0));
        auto net = RoutingNet {};
        net.net_id = 0;
        net.kind = RoutingNetKind::Tnet;
        net.sources = {source};
        for (std::size_t index = 0; index < bump_count; ++index) {
            net.demands.push_back(RoutingDemand {
                index, bump_ref(0, 0, index), {0}, true});
        }
        const auto graph = synthetic_channel_graph(
            {{0, 0, 0},
             {1, static_cast<int>(anchor.row), static_cast<int>(anchor.col)}},
            {global_port_node(source, 0), global_cob_node(), global_tob_node(0, 1)},
            {{0, 1, 0}, {1, 2, 1}});
        return std::pair {graph, std::Vector<RoutingNet> {std::move(net)}};
    };

    const auto solve = [&](const std::size_t bump_count, const GlobalRouteCapacityMode mode) {
        auto [graph, nets] = make_fixture(bump_count);
        const auto route = solve_global_route_v17(UnifiedGraph {}, graph, nets, 0, mode);
        require(route.ok, "V21 TOB peak fixture must be feasible");
        require(
            route.stats.objective == 2
                && route.stats.estimated_wirelength == route.stats.objective + bump_count,
            "V21 must count a shared multi-sink Channel union once while "
                "keeping every TOB bump in estimated wirelength");
        require(
            std::abs(route.stats.full_objective
                     - (static_cast<double>(route.stats.objective) + route.stats.tob_peak_cost))
                    < 1e-6
                && std::abs(route.stats.solver_objective
                            - (route.stats.full_objective
                               - route.stats.tob_peak_constant_cost))
                    < 1e-6,
            "V21 full objective must include all TOB peaks while the HiGHS "
                "objective excludes unavoidable fixed offsets");
        require(
            route.stats.model.total_variables() == route.stats.variables
                && route.stats.model.total_constraints() == route.stats.constraints,
            "V21 TOB peak variables and rows must reconcile with the full "
                "model totals");
        return route;
    };

    for (const auto mode : {GlobalRouteCapacityMode::DenseW,
                            GlobalRouteCapacityMode::IterativeCuts}) {
        const auto below = solve(6, mode);
        require(
            below.stats.model.tob_h7_vars == 0 && below.stats.model.tob_h8_vars == 0
                && below.stats.tob_load7 == 0 && below.stats.tob_load8 == 0
                && below.stats.tob_peak_cost == 0.0 && below.stats.max_tob_unit_load == 6,
            "a TOB-unit whose possible load is below seven must not create "
                "or select H variables");

        const auto seven = solve(7, mode);
        require(
            seven.stats.model.tob_h7_vars == 1 && seven.stats.model.tob_h8_vars == 0
                && seven.stats.model.tob_peak_threshold == 2
                && seven.stats.tob_load7 == 1 && seven.stats.tob_load8 == 0
                && seven.stats.tob_load7_forced == 1
                && seven.stats.tob_load8_forced == 0
                && seven.stats.tob_peak_constant_cost == 0.5
                && seven.stats.tob_peak_cost == 0.5 && seven.stats.max_tob_unit_load == 7,
            "a load of exactly seven must select only H7 with peak cost 0.5");

        const auto eight = solve(8, mode);
        require(
            eight.stats.model.tob_h7_vars == 1 && eight.stats.model.tob_h8_vars == 1
                && eight.stats.model.tob_peak_threshold == 2
                && eight.stats.tob_load7 == 0 && eight.stats.tob_load8 == 1
                && eight.stats.tob_load7_forced == 0
                && eight.stats.tob_load8_forced == 1
                && eight.stats.tob_peak_constant_cost == 1.0
                && eight.stats.tob_peak_cost == 1.0 && eight.stats.max_tob_unit_load == 8,
            "a load of exactly eight must select only H8 with peak cost 1.0");
    }

    const auto anchor = tob_anchor_cob(0);
    const auto graph = synthetic_channel_graph(
        {{1, static_cast<int>(anchor.row), static_cast<int>(anchor.col)}},
        {global_tob_node(0, 0)},
        {});
    auto selectable_nets = std::Vector<RoutingNet> {};
    for (std::size_t net_index = 0; net_index < 2; ++net_index) {
        auto net = RoutingNet {};
        net.net_id = net_index;
        net.kind = RoutingNetKind::Bnet;
        net.sources = {bump_ref(0, 0, 4 * net_index)};
        for (std::size_t offset = 1; offset < 4; ++offset) {
            net.demands.push_back(RoutingDemand {
                offset - 1, bump_ref(0, 0, 4 * net_index + offset), {0}, true});
        }
        selectable_nets.push_back(std::move(net));
    }
    for (const auto mode : {GlobalRouteCapacityMode::DenseW,
                            GlobalRouteCapacityMode::IterativeCuts}) {
        const auto balanced = solve_global_route_v17(
            UnifiedGraph {}, graph, selectable_nets, 0, mode);
        require(
            balanced.ok
                && balanced.stats.model.tob_h7_vars == 16
                && balanced.stats.model.tob_h8_vars == 16
                && balanced.stats.objective == 2
                && balanced.stats.estimated_wirelength == 10
                && balanced.stats.tob_load7 == 0
                && balanced.stats.tob_load8 == 0
                && balanced.stats.tob_peak_cost == 0.0
                && balanced.stats.max_tob_unit_load == 4,
            "V21 TOB peak objective must separate selectable four-bump "
                "owners instead of accepting one 8/8 unit");
    }

    auto selectable_seven = RoutingNet {};
    selectable_seven.net_id = 10;
    selectable_seven.kind = RoutingNetKind::Bnet;
    selectable_seven.sources = {bump_ref(0, 0, 0)};
    for (std::size_t index = 1; index < 7; ++index) {
        selectable_seven.demands.push_back(RoutingDemand {
            index - 1, bump_ref(0, 0, index), {0}, true});
    }
    for (const auto mode : {GlobalRouteCapacityMode::DenseW,
                            GlobalRouteCapacityMode::IterativeCuts}) {
        const auto selected_peak = solve_global_route_v17(
            UnifiedGraph {}, graph, {selectable_seven}, 0, mode);
        require(
            selected_peak.ok
                && selected_peak.stats.model.tob_h7_vars == 16
                && selected_peak.stats.model.tob_h8_vars == 0
                && selected_peak.stats.tob_load7 == 1
                && selected_peak.stats.tob_load7_forced == 0
                && selected_peak.stats.tob_peak_constant_cost == 0.0
                && selected_peak.stats.tob_peak_cost == 0.5
                && selected_peak.stats.solver_objective == 1.5
                && selected_peak.stats.full_objective == 1.5,
            "V21 must charge one decision-dependent H7 for a selectable "
                "seven-bump owner");
    }
}

auto test_v17_guide_scope_unit_release() -> void {
    auto graph = synthetic_graph(3, {});
    graph.nodes[1].kind = UnifiedNodeKind::Track;
    graph.nodes[1].track_dir = 1;
    graph.nodes[1].track_row = 1;
    graph.nodes[1].track_col = 0;
    graph.nodes[1].unit = 3;
    graph.nodes[2] = graph.nodes[1];
    graph.nodes[2].unit = 4;
    auto net = two_pin_net(
        0, RoutingNetKind::Bnet, synthetic_ref(0), synthetic_ref(0));
    net.has_global_route_guide = true;
    net.global_route_channels.insert(GlobalChannelCoord {1, 1, 0});
    net.global_unit_by_source[0] = 3;
    const auto fixed = build_scope(graph, net, {0}, {0}, {});
    require(
        fixed.node_offset[1] >= 0 && fixed.node_offset[2] < 0,
        "a fixed V17 unit must expose only that unit's lanes in the guide");
    net.released_global_unit_sources.insert(0);
    const auto released = build_scope(graph, net, {0}, {0}, {});
    require(
        released.node_offset[1] >= 0 && released.node_offset[2] >= 0,
        "a released V17 unit must expose all units inside the same guide");
}

auto test_v17_unit_assumption_is_traceable() -> void {
    auto graph = synthetic_graph(2, {{0, 1}});
    auto net = synthetic_net(0, {0}, {{1, {0}}});
    net.kind = RoutingNetKind::Bnet;
    net.global_unit_by_source[0] = 3;
    const auto scopes = std::Vector<UnifiedSatNetScope> {synthetic_scope(graph, 0)};
    const auto delays = compute_pair_delays(graph, {net}, scopes);
    auto session = CadicalSession {};
    const auto model = build_unified_sat_model(session, graph, {net}, scopes, delays);
    require(
        model.unit_assumption_vars.size() == 1
            && model.unit_assumption_vars[0].unit == 3,
        "V17 must emit one traceable gamma assumption for a fixed Bnet "
            "source unit");
    const auto& source = model.sources[0];
    session.add_clause({-source.unit_selector_var_by_unit[3]});
    session.assume(model.alpha_vars[0].alpha_lit);
    session.assume(model.unit_assumption_vars[0].assumption_lit);
    const auto result = session.solve();
    require(
        !result.ok
            && std::ranges::find(
                   result.failed_assumption_literals,
                   model.unit_assumption_vars[0].assumption_lit)
                != result.failed_assumption_literals.end(),
        "a unit conflict must identify only the releasable gamma "
            "assumption in the core");
}

auto test_v17_distance_domain_starts_at_scoped_minimum() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {2, 3}, {1, 3}});
    auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].global_route_distance_cap = 3;
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes, &state);
    require(
        delays.pairs[0].delays == std::Vector<int>({2}),
        "V17 initial global-guide distance domain must contain only scoped "
            "d_min");
}

auto test_v20_track_targets_start_at_scoped_minimum() -> void {
    const auto graph = synthetic_graph(
        5, {{0, 1}, {1, 2}, {2, 3}, {3, 4}});
    auto track_to_bump = synthetic_net(0, {0}, {{2, {0}}});
    track_to_bump.name = "TrackToBumpNet";
    track_to_bump.post_sat_ilp_target = true;
    auto track_to_bumps = synthetic_net(1, {0}, {{3, {0}}, {4, {0}}});
    track_to_bumps.name = "TrackToBumpsNet";
    track_to_bumps.post_sat_ilp_target = true;
    const auto nets = std::Vector<RoutingNet> {track_to_bump, track_to_bumps};
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes);
    require(
        delays.pairs.size() == 3
            && delays.pairs[0].delays == std::Vector<int>({2})
            && delays.pairs[1].delays == std::Vector<int>({3})
            && delays.pairs[2].delays == std::Vector<int>({4}),
        "TrackToBump and every TrackToBumps demand must start with only "
            "scoped d_min");
}

auto test_v17_bus_distance_domain_starts_at_shared_minimum() -> void {
    const auto graph = synthetic_graph(
        7, {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {5, 6}});
    auto bus = synthetic_net(0, {0, 3}, {{2, {0}}, {6, {1}}});
    bus.is_sync_bus = true;
    auto nets = std::Vector<RoutingNet> {bus};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].global_route_distance_cap = 4;
    state.pairs[1].global_route_distance_cap = 5;
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes, &state);
    const auto expected = std::Vector<int>({3});
    require(
        delays.pairs[0].delays == expected && delays.pairs[1].delays == expected,
        "V17 bus members must share singleton {max member d_min}");
}

auto test_scoped_path_error_identifies_pair() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(17, {0}, {{2, {0}}})};
    const auto scopes = std::Vector<UnifiedSatNetScope> {synthetic_scope(graph, 17)};
    try {
        (void)compute_pair_delays(graph, nets, scopes);
        require(false, "disconnected scoped pair must fail delay precompute");
    }
    catch (const ScopedPathUnavailable& error) {
        require(
            error.pair_key == PairKey {17, 0, 0},
            "scoped-path failure must identify only the unreachable pair");
    }
}

auto test_v19_tob_repair_template_and_shared_guide() -> void {
    const auto detailed = build_unified_graph(nullptr, {});
    const auto channel_graph = build_global_channel_graph(detailed, {});
    constexpr std::size_t tob = 5; // TOB(1,1), between COB(2,3) and COB(3,3).
    const auto [first, second] = tob_pair_cob_coords(1, 1);
    require(
        (first == hardware::COBCoord {3, 3} && second == hardware::COBCoord {2, 3})
            || (first == hardware::COBCoord {2, 3} && second == hardware::COBCoord {3, 3}),
        "TOB(1,1) must be located between COB(2,3) and COB(3,3)");

    auto net = RoutingNet {};
    net.net_id = 31;
    net.kind = RoutingNetKind::Tnet;
    net.sources = {track_ref(0, 0)};
    net.demands = {
        RoutingDemand {0, bump_ref(tob, 0, 0), {0}, true},
        RoutingDemand {1, bump_ref(tob, 0, 1), {0}, true}};
    auto nets = std::Vector<RoutingNet> {net};
    auto state = init_routing_problem_state(nets);
    const auto central = GlobalChannelCoord {1, 3, 3};
    const auto branch = GlobalChannelCoord {0, 0, 1};
    auto route = GlobalRouteResult {};
    route.ok = true;
    route.pair_channels.emplace(state.pairs[0].key, std::set {central});
    route.pair_channels.emplace(state.pairs[1].key, std::set {branch});
    apply_global_route_v17(route, channel_graph, state, nets);

    const auto repair = std::set<GlobalChannelCoord> {
        {1, 3, 3}, {1, 2, 3}, {1, 4, 3},
        {0, 2, 3}, {0, 2, 4}, {0, 3, 3}, {0, 3, 4},
        {1, 3, 2}, {1, 3, 4}};
    auto expected = repair;
    expected.insert(branch);
    auto second_pair_expected = repair;
    second_pair_expected.insert(branch);
    require(
        state.pairs[0].allowed_channels == repair
            && state.pairs[1].allowed_channels == second_pair_expected
            && nets[0].global_route_channels == expected,
        "V19 multi-pin repair must preserve pair-local guides and expose "
            "their union to SAT");
    require(
        !expected.contains(GlobalChannelCoord {1, 1, 3}),
        "the repair template must include only its upper boundary Channel, "
            "not an extra outer COB side");

    auto tob_to_tob = two_pin_net(
        32, RoutingNetKind::Bnet, bump_ref(tob, 0, 0), bump_ref(tob, 0, 1));
    auto tob_nets = std::Vector<RoutingNet> {tob_to_tob};
    auto tob_state = init_routing_problem_state(tob_nets);
    auto tob_route = GlobalRouteResult {};
    tob_route.ok = true;
    tob_route.pair_channels.emplace(tob_state.pairs[0].key, std::set {central});
    apply_global_route_v17(tob_route, channel_graph, tob_state, tob_nets);
    const auto compact_repair = std::set<GlobalChannelCoord> {
        {1, 3, 3}, {1, 2, 3}, {1, 4, 3},
        {0, 2, 3}, {0, 2, 4}, {0, 3, 3}, {0, 3, 4}};
    require(
        net_is_tob_to_tob(tob_nets[0])
            && tob_state.pairs[0].allowed_channels == compact_repair
            && compact_repair.size() == 7
            && !compact_repair.contains(GlobalChannelCoord {1, 3, 2})
            && !compact_repair.contains(GlobalChannelCoord {1, 3, 4}),
        "V19 TOB-TOB pair must receive the compact seven-Channel TOB "
            "repair template");

    auto boundary = RoutingNet {};
    boundary.net_id = 33;
    boundary.kind = RoutingNetKind::Tnet;
    boundary.sources = {track_ref(0, 0)};
    boundary.demands = {RoutingDemand {0, bump_ref(0, 0, 0), {0}, true}};
    auto boundary_nets = std::Vector<RoutingNet> {boundary};
    auto boundary_state = init_routing_problem_state(boundary_nets);
    auto boundary_route = GlobalRouteResult {};
    boundary_route.ok = true;
    boundary_route.pair_channels.emplace(
        boundary_state.pairs[0].key, std::set {GlobalChannelCoord {1, 1, 0}});
    apply_global_route_v17(boundary_route, channel_graph, boundary_state, boundary_nets);
    const auto boundary_expected = std::set<GlobalChannelCoord> {
        {1, 1, 0}, {1, 0, 0}, {1, 2, 0}, {0, 0, 0},
        {0, 0, 1}, {0, 1, 0}, {0, 1, 1}, {1, 1, 1}};
    require(
        boundary_state.pairs[0].allowed_channels == boundary_expected,
        "V19 boundary TOB repair must retain boundary Channels and clip "
            "the nonexistent outer side");

    auto tob_bus = RoutingNet {};
    tob_bus.net_id = 34;
    tob_bus.kind = RoutingNetKind::Bnet;
    tob_bus.is_sync_bus = true;
    tob_bus.sources = {bump_ref(tob, 0, 0), bump_ref(6, 0, 0)};
    tob_bus.demands = {
        RoutingDemand {0, bump_ref(7, 0, 0), {0}, true},
        RoutingDemand {1, bump_ref(8, 0, 0), {1}, true}};
    auto bus_nets = std::Vector<RoutingNet> {tob_bus};
    auto bus_state = init_routing_problem_state(bus_nets);
    auto bus_route = GlobalRouteResult {};
    bus_route.ok = true;
    bus_route.pair_channels.emplace(bus_state.pairs[0].key, std::set {central});
    bus_route.pair_channels.emplace(bus_state.pairs[1].key, std::set {branch});
    apply_global_route_v17(bus_route, channel_graph, bus_state, bus_nets);
    require(
        net_is_tob_to_tob(bus_nets[0])
            && bus_state.pairs[0].allowed_channels != bus_state.pairs[1].allowed_channels
            && bus_state.pairs[0].allowed_channels.contains(central)
            && bus_state.pairs[1].allowed_channels.contains(branch)
            && bus_nets[0].global_route_channels.size() > 2,
        "all-TOB SyncBus must compact-repair each pair locally before "
            "exposing their union to SAT");
}

auto test_v20_target_scopes_expand_guide_and_tob_patch_one_hop() -> void {
    const auto detailed = build_unified_graph(nullptr, {});
    const auto graph = build_global_channel_graph(detailed, {});
    const auto apply = [&](const std::size_t id, const RoutingNetKind kind,
                           const bool target) {
        auto net = two_pin_net(
            id, kind, track_ref(0, 0), bump_ref(0, 0, 0));
        net.post_sat_ilp_target = target;
        auto nets = std::Vector<RoutingNet> {net};
        auto state = init_routing_problem_state(nets);
        auto route = GlobalRouteResult {};
        route.ok = true;
        route.pair_channels.emplace(
            state.pairs[0].key, std::set {GlobalChannelCoord {1, 1, 0}});
        apply_global_route_v17(route, graph, state, nets);
        return state.pairs[0];
    };
    const auto control = apply(35, RoutingNetKind::Tnet, false);
    auto expected = control.allowed_channels;
    for (const auto& channel : control.allowed_channels) {
        const auto id = graph.channel_id_by_coord.at(channel);
        for (const int adjacent : graph.adjacent_channel_ids[static_cast<std::size_t>(id)]) {
            expected.insert(graph.channels[static_cast<std::size_t>(adjacent)]);
        }
    }
    auto reached_cobs = std::set<std::pair<int, int>>{};
    const auto channel_cobs = [&](const std::size_t channel) {
        auto cobs = std::set<std::pair<int, int>>{};
        for (const int arc_id : graph.arc_ids_by_channel[channel]) {
            const auto &arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            for (const int node_id : {arc.u, arc.v}) {
                const auto &node =
                    graph.nodes[static_cast<std::size_t>(node_id)];
                if (node.kind == GlobalRouteNodeKind::Cob) {
                    cobs.emplace(node.row, node.col);
                }
            }
        }
        return cobs;
    };
    for (const auto &channel : expected) {
        const auto cobs = channel_cobs(
            static_cast<std::size_t>(graph.channel_id_by_coord.at(channel)));
        reached_cobs.insert(cobs.begin(), cobs.end());
    }
    for (std::size_t channel = 0; channel < graph.channels.size(); ++channel) {
        const auto cobs = channel_cobs(channel);
        if (cobs.size() == 2 && std::ranges::all_of(cobs, [&](const auto &cob) {
                return reached_cobs.contains(cob);
            })) {
            expected.insert(graph.channels[channel]);
        }
    }
    const auto track_target = apply(36, RoutingNetKind::Tnet, true);
    const auto pn_target = apply(37, RoutingNetKind::PNnet, false);
    require(
        !track_target.initial_expanded_channels.empty()
            && track_target.allowed_channels == expected
            && pn_target.allowed_channels == expected,
        "TrackToBump(s) targets and PNnet must expand guide plus TOB patch "
            "by one hop");
    require(
        control.initial_expanded_channels.empty()
            && control.allowed_channels != expected,
        "non-target Tnet such as normalized BumpToTrack must keep guide "
            "plus TOB patch");
}

auto test_v19_global_guide_feedback_thresholds_and_union() -> void {
    const auto c0 = GlobalChannelCoord {0, 0, 1};
    const auto c1 = GlobalChannelCoord {1, 1, 1};
    const auto c2 = GlobalChannelCoord {0, 2, 1};
    const auto channel_graph = synthetic_channel_graph(
        {c0, c1, c2},
        {global_cob_node(0, 0), global_cob_node(0, 1), global_cob_node(1, 1)},
        {{0, 1, 0}, {1, 2, 1}});

    auto net = RoutingNet {};
    net.net_id = 41;
    net.kind = RoutingNetKind::Tnet;
    net.sources = {track_ref(0, 0)};
    net.demands = {
        RoutingDemand {0, bump_ref(0, 0, 0), {0}, true},
        RoutingDemand {1, bump_ref(1, 0, 0), {0}, true}};
    auto unrelated = synthetic_net(43, {0}, {{1, {0}}});
    auto nets = std::Vector<RoutingNet> {net, unrelated};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].allowed_channels = {c0};
    state.pairs[1].allowed_channels = {c2};
    state.pairs[2].allowed_channels = {c2};
    state.pairs[0].delays = {7};
    state.pairs[1].delays = {9};
    state.pairs[2].delays = {5};
    for (int attempt = 0; attempt < 2; ++attempt) {
        const auto stats = apply_global_route_feedback_step(
            channel_graph, state, nets, {state.pairs[0].key});
        require(stats.expanded_nets.empty(), "ordinary net must wait four distance-only extensions");
    }
    for (int attempt = 0; attempt < 2; ++attempt) {
        const auto stats = apply_global_route_feedback_step(
            channel_graph, state, nets, {state.pairs[1].key});
        require(
            stats.expanded_nets.empty(),
            "different demands must not pool their distance failures");
    }
    for (int attempt = 0; attempt < 2; ++attempt) {
        const auto stats = apply_global_route_feedback_step(
            channel_graph, state, nets, {state.pairs[0].key});
        require(stats.expanded_nets.empty(), "the same pair must get four distance-only extensions");
    }
    const auto normal_stats = apply_global_route_feedback_step(
        channel_graph, state, nets, {state.pairs[0].key});
    const auto expected = std::set<GlobalChannelCoord> {c0, c1, c2};
    apply_state_to_nets(state, nets);
    require(
        normal_stats.expanded_nets == std::set<std::size_t> {41}
            && normal_stats.pair_local_added_channels == 1
            && normal_stats.added_channels == 1
            && state.pairs[0].allowed_channels == std::set<GlobalChannelCoord>({c0, c1})
            && state.pairs[1].allowed_channels == std::set<GlobalChannelCoord> {c2}
            && nets[0].global_route_channels == expected
            && state.pairs[0].delays == std::Vector<int>({7, 8, 9, 10, 11, 12})
            && state.pairs[1].delays == std::Vector<int>({9, 10, 11})
            && state.global_route_feedback_failure_count_by_pair[state.pairs[0].key] == 0
            && state.global_route_feedback_failure_count_by_pair[state.pairs[1].key] == 2
            && state.pairs[2].allowed_channels == std::set<GlobalChannelCoord> {c2}
            && state.pairs[2].delays == std::Vector<int> {5},
        "the fifth failure of one pair must expand only its guide and expose "
        "the net union");

    auto overlap_state = init_routing_problem_state({net});
    overlap_state.pairs[0].allowed_channels = {c0};
    overlap_state.pairs[1].allowed_channels = {c1};
    const auto overlap_graph = synthetic_channel_graph(
        {c0, c1, c2},
        {global_cob_node(0, 0), global_cob_node(0, 1),
         global_cob_node(0, 2), global_cob_node(0, 3)},
        {{0, 1, 0}, {1, 2, 1}, {2, 3, 2}});
    const auto overlap_first = expand_global_route_guides_one_hop(
        overlap_graph, overlap_state, {overlap_state.pairs[0].key});
    const auto overlap_second = expand_global_route_guides_one_hop(
        overlap_graph, overlap_state, {overlap_state.pairs[0].key});
    require(
        overlap_first.pair_local_added_channels == 1
            && overlap_first.added_channels == 0
            && overlap_second.pair_local_added_channels == 1
            && overlap_second.added_channels == 1
            && overlap_state.pairs[0].allowed_channels
                == std::set<GlobalChannelCoord>({c0, c1, c2}),
        "pair-local growth must continue when its first new Channel "
            "already exists in the net union");

    const auto bottom = GlobalChannelCoord{0, 1, 1};
    const auto left = GlobalChannelCoord{1, 1, 0};
    const auto right = GlobalChannelCoord{1, 1, 1};
    const auto top = GlobalChannelCoord{0, 0, 1};
    const auto closure_graph =
        synthetic_channel_graph({bottom, left, right, top},
                                {global_cob_node(0, 0), global_cob_node(0, 1),
                                 global_cob_node(1, 0), global_cob_node(1, 1)},
                                {{2, 3, 0}, {0, 2, 1}, {1, 3, 2}, {0, 1, 3}});
    require(std::ranges::find(closure_graph.adjacent_channel_ids[0], 3) ==
                closure_graph.adjacent_channel_ids[0].end(),
            "the opposite Channel must not be reachable by the raw one-hop "
            "adjacency");
    auto closure_net = two_pin_net(44, RoutingNetKind::Tnet, track_ref(0, 0),
                                   bump_ref(0, 0, 0));
    auto closure_state = init_routing_problem_state({closure_net});
    closure_state.pairs[0].allowed_channels = {bottom};
    const auto closure_stats = expand_global_route_guides_one_hop(
        closure_graph, closure_state, {closure_state.pairs[0].key});
    require(closure_stats.pair_local_added_channels == 3 &&
                closure_stats.added_channels == 3 &&
                closure_state.pairs[0].allowed_channels ==
                    std::set<GlobalChannelCoord>({bottom, left, right, top}),
            "one-hop expansion must close Channels induced by the newly "
            "reached COBs");

    auto tob_to_tob = two_pin_net(
        42, RoutingNetKind::Bnet, bump_ref(0, 0, 0), bump_ref(1, 0, 0));
    const auto tob_nets = std::Vector<RoutingNet> {tob_to_tob};
    auto tob_state = init_routing_problem_state(tob_nets);
    tob_state.pairs[0].allowed_channels = {c0};
    tob_state.pairs[0].delays = {3};
    for (int attempt = 0; attempt < 4; ++attempt) {
        const auto stats = apply_global_route_feedback_step(
            channel_graph, tob_state, tob_nets, {tob_state.pairs[0].key});
        require(stats.expanded_nets.empty(), "TOB-TOB net must wait four distance-only extensions");
    }
    const auto tob_stats = apply_global_route_feedback_step(
        channel_graph, tob_state, tob_nets, {tob_state.pairs[0].key});
    require(
        tob_stats.expanded_nets == std::set<std::size_t> {42}
            && tob_state.pairs[0].delays == std::Vector<int>({3, 4, 5, 6, 7, 8}),
        "the fifth TOB-TOB failure must expand scope after preserving its "
            "fifth distance extension");
}

auto test_v20_global_guide_logger_handles_walk_and_residual() -> void {
    const auto source = bump_ref(0, 0, 0);
    const auto sink = bump_ref(1, 0, 0);
    auto net = two_pin_net(61, RoutingNetKind::Bnet, source, sink);
    auto nets = std::Vector<RoutingNet>{net};
    auto state = init_routing_problem_state(nets);
    const auto c0 = GlobalChannelCoord{1, 1, 0};
    const auto c1 = GlobalChannelCoord{0, 1, 1};
    const auto graph =
        synthetic_channel_graph({c0, c1},
                                {global_tob_node(0, 0), global_cob_node(0, 0),
                                 global_cob_node(0, 1), global_tob_node(1, 1)},
                                {{0, 1, 0}, {1, 2, 1}, {2, 3, 1}});
    auto route = GlobalRouteResult{};
    route.ok = true;
    const auto key = state.pairs.front().key;
    route.pair_channels.emplace(key, std::set{c0, c1});
    // Arc 1 goes back over the first physical Channel and is intentionally
    // residual beside the complete forward source-to-sink walk 0,2,4.
    route.selected_arc_ids_by_pair.emplace(key, std::Vector<int>{0, 1, 2, 4});
    route.selected_unit_by_pair.emplace(key, 3);
    state.pairs.front().allowed_channels = {c0, c1};
    require(
        log_global_route_guides(route, graph, state, nets) >= 0,
        "V20 guide logger must handle a directed walk plus residual selected "
        "arc");
}
