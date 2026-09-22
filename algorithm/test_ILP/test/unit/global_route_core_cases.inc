// PN source/unit preselection and core V17/V18/V19 Global Routing model tests.

auto test_v18_pn_preselection_reserves_fixed_tnet_unit_load() -> void {
    const auto unit0_source = track_ref(
        0, 0, hardware::TrackDirection::Vertical, track_from_unit_inner(0, 0));
    const auto unit1_source = track_ref(
        1, static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH),
        hardware::TrackDirection::Horizontal,
        track_from_unit_inner(1, 0));
    auto nets = std::Vector<RoutingNet> {};
    for (std::size_t bump = 0; bump < 8; ++bump) {
        nets.push_back(two_pin_net(
            bump,
            RoutingNetKind::Tnet,
            unit0_source,
            bump_ref(0, 0, bump)));
    }
    nets.push_back(pn_preselection_net(
        8, {unit0_source, unit1_source}, {bump_ref(0, 1, 0)}));

    const auto detailed = build_unified_graph(nullptr, nets);
    const auto global = build_global_channel_graph(detailed, nets);
    const auto selected = preselect_pn_sources_v18(global, nets, 0);
    require(selected.ok, "V18 PN preselection fixed-unit fixture must be feasible");
    require(
        selected.stats.fixed_tnet_bumps == 8
            && selected.stats.candidates == 2
            && selected.stats.source_trees == 1
            && selected.stats.overflow_vars
                == hardware::Interposer::COB_ARRAY_WIDTH + 2
            && selected.stats.alpha > 0.0
            && selected.stats.lambda_a
                == static_cast<double>(hardware::Interposer::COB_ARRAY_WIDTH - 2),
        "V18 PN preselection must report fixed Tnet load and sparse RUDY "
        "variables");
    require(
        selected.nets.size() == 9
            && std::ranges::none_of(
                selected.nets,
                [](const RoutingNet& net) { return net.kind == RoutingNetKind::PNnet; })
            && selected.nets.back().kind == RoutingNetKind::Tnet
            && selected.nets.back().sources.size() == 1
            && map_track(selected.nets.back().sources.front().track_index) == 1,
        "eight fixed unit-0 Tnet bumps must force the PN bump onto unit 1");
}

auto test_v18_pn_preselection_uses_unit_aware_rudy() -> void {
    const auto unit0_source = track_ref(
        1, 0, hardware::TrackDirection::Horizontal, track_from_unit_inner(0, 0));
    const auto unit1_source = track_ref(
        1, 0, hardware::TrackDirection::Horizontal, track_from_unit_inner(1, 0));
    const auto far_tob = hardware::Interposer::TOB_SIZE - 1;
    const auto nets = std::Vector<RoutingNet> {
        two_pin_net(
            0,
            RoutingNetKind::Tnet,
            unit0_source,
            bump_ref(far_tob, 0, 0)),
        pn_preselection_net(
            1,
            {unit0_source, unit1_source},
            {bump_ref(far_tob, 1, 0)})};

    const auto detailed = build_unified_graph(nullptr, nets);
    const auto global = build_global_channel_graph(detailed, nets);
    const auto selected = preselect_pn_sources_v18(global, nets, 0);
    require(selected.ok, "V18 PN preselection RUDY fixture must be feasible");
    require(
        selected.stats.alpha > 0.0
            && selected.stats.overflow_vars > 0
            && selected.nets.size() == 2
            && selected.nets.back().kind == RoutingNetKind::Tnet
            && map_track(selected.nets.back().sources.front().track_index) == 1,
        "unit-aware RUDY must prefer the equally distant unit without "
            "fixed Tnet demand");
}

auto test_v18_pn_preselection_scales_lambda_a_by_k_hat() -> void {
    const auto source0 = track_ref(
        0, 0, hardware::TrackDirection::Horizontal, track_from_unit_inner(0, 0));
    const auto source1 = track_ref(
        0, 0, hardware::TrackDirection::Horizontal, track_from_unit_inner(1, 0));
    const auto nets = std::Vector<RoutingNet> {
        pn_preselection_net(0, {source0, source1}, {bump_ref(0, 0, 0)}),
        pn_preselection_net(1, {source0, source1}, {bump_ref(0, 0, 1)})};
    const auto detailed = build_unified_graph(nullptr, nets);
    const auto global = build_global_channel_graph(detailed, nets);
    const auto selected = preselect_pn_sources_v18(global, nets, 0);
    require(selected.ok, "V18 PN preselection k_hat fixture must be feasible");
    require(
        selected.stats.pn_bumps == 2
            && selected.stats.pn_01_ports == 2
            && selected.stats.k_hat == 2.0
            && selected.stats.lambda_a_base > 0.0
            && selected.stats.lambda_a == selected.stats.lambda_a_base * selected.stats.k_hat,
        "V18 PN lambda_A must use all PN bumps and deduplicate physical "
            "ports across PNnets");
}

auto test_v18_pn_preselection_reserves_fixed_tnet_bank_residue_load() -> void {
    const auto fixed_source = track_ref(
        1, 0, hardware::TrackDirection::Horizontal, track_from_unit_inner(0, 0));
    const auto residue0_source = track_ref(
        1, 0, hardware::TrackDirection::Horizontal, track_from_unit_inner(8, 0));
    const auto residue1_source = track_ref(
        1, static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH),
        hardware::TrackDirection::Horizontal,
        track_from_unit_inner(1, 0));
    auto sinks = std::Vector<GraphNodeRef> {};
    for (std::size_t bump = 0; bump < 8; ++bump) {
        sinks.push_back(bump_ref(0, 0, bump));
    }
    auto fixed_bump = bump_ref(0, 0, 0);
    fixed_bump.bump.Group = 1;
    auto nets = std::Vector<RoutingNet> {
        two_pin_net(0, RoutingNetKind::Tnet, fixed_source, fixed_bump),
        pn_preselection_net(1, {residue0_source, residue1_source}, sinks)};

    const auto detailed = build_unified_graph(nullptr, nets);
    const auto global = build_global_channel_graph(detailed, nets);
    const auto selected = preselect_pn_sources_v18(global, nets, 0);
    require(selected.ok, "V18 PN preselection bank-residue fixture must be feasible");
    require(
        selected.stats.source_trees == 2 && selected.nets.size() == 3,
        "bank-residue pressure must create two physical source-tree Tnets");
    std::size_t unit1_bumps = 0;
    std::size_t unit8_bumps = 0;
    for (const auto& net : selected.nets) {
        if (net.name.find("synthetic-PN#source") == std::String::npos) {
            continue;
        }
        const auto unit = map_track(net.sources.front().track_index);
        if (unit == 1) {
            unit1_bumps += net.demands.size();
        }
        if (unit == 8) {
            unit8_bumps += net.demands.size();
        }
    }
    require(
        unit1_bumps >= 1 && unit8_bumps <= 7 && unit1_bumps + unit8_bumps == 8,
        "fixed unit-0 load must leave at most seven same-bank residue-0 "
            "slots for unit 8");
}

auto test_v17_global_route_two_pin_and_fixed_unit() -> void {
    const auto a0 = tob_anchor_cob(0);
    const auto a1 = tob_anchor_cob(1);
    const auto bump_graph = synthetic_channel_graph(
        {{1, static_cast<int>(a0.row), static_cast<int>(a0.col)},
         {1, static_cast<int>(a1.row), static_cast<int>(a1.col)}},
        {global_tob_node(0, 0), global_cob_node(), global_tob_node(1, 1)},
        {{0, 1, 0}, {1, 2, 1}});
    auto bump_net = two_pin_net(
        0, RoutingNetKind::Bnet, bump_ref(0, 0, 0), bump_ref(1, 0, 1));
    const auto bump_route = solve_global_route_v17(
        UnifiedGraph {}, bump_graph, {bump_net}, 1);
    require(bump_route.ok, "V17 synthetic bump-to-bump route must be feasible");
    require(
        bump_route.unit_by_owner.size() == 1
            && bump_route.pair_channels.at(PairKey {0, 0, 0}).size() == 2
            && bump_route.selected_arc_ids_by_pair.at(PairKey {0, 0, 0}).size() == 2
            && bump_route.pair_cobs.at(PairKey {0, 0, 0}).size() == 1
            && bump_route.stats.objective == 2
            && bump_route.stats.estimated_wirelength == 4,
        "V17 must choose one unit and expose the guide arc and derived COB");
    require(
        bump_route.stats.model.q_vars == 16
            && bump_route.stats.model.x_vars == 2
            && bump_route.stats.model.w_vars == 32
            && bump_route.stats.model.f_vars == 4
            && bump_route.stats.model.total_variables() == bump_route.stats.variables
            && bump_route.stats.model.total_constraints() == bump_route.stats.constraints,
        "V17 model breakdown must reconcile every variable and constraint");

    const auto cut_route = solve_global_route_v17(
        UnifiedGraph {},
        bump_graph,
        {bump_net},
        0,
        GlobalRouteCapacityMode::IterativeCuts);
    require(
        cut_route.ok
            && cut_route.stats.capacity_cuts_enabled
            && cut_route.stats.model.w_vars == 0
            && cut_route.stats.model.w_linearization == 0
            && cut_route.stats.mip_start_owners == 1
            && cut_route.stats.mip_start_two_pin_owners == 1
            && cut_route.stats.mip_start_commodities == 1
            && cut_route.stats.mip_start_entries > 0
            && cut_route.stats.capacity_cuts == 0
            && cut_route.stats.objective == bump_route.stats.objective
            && cut_route.stats.variables < bump_route.stats.variables
            && cut_route.stats.constraints < bump_route.stats.constraints,
        "capacity-cut mode must remove dense W without changing an "
            "uncongested optimum");

    auto state = init_routing_problem_state({bump_net});
    auto bump_nets = std::Vector<RoutingNet> {bump_net};
    apply_global_route_v17(bump_route, state, bump_nets);
    require(
        state.pairs[0].global_route_distance_cap == 8
            && bump_nets[0].global_unit_by_source.contains(0),
        "V17 interface must preserve the detailed distance cap and source "
            "unit");

    const auto fixed_source = track_ref(
        0, 0, hardware::TrackDirection::Horizontal, 69);
    const auto unrelated_port = track_ref(
        0, 0, hardware::TrackDirection::Horizontal, 70);
    const auto fixed_graph = synthetic_channel_graph(
        {{0, 0, 0},
         {1, static_cast<int>(a0.row), static_cast<int>(a0.col)}},
        {global_port_node(fixed_source, 0), global_cob_node(), global_tob_node(0, 1),
         global_port_node(unrelated_port, 0)},
        {{0, 1, 0}, {1, 2, 1}, {3, 1, 0}});
    auto fixed_net = two_pin_net(
        5,
        RoutingNetKind::Tnet,
        fixed_source,
        bump_ref(0, 0, 2));
    const auto fixed_route = solve_global_route_v17(
        UnifiedGraph {}, fixed_graph, {fixed_net}, 0);
    require(
        fixed_route.ok
            && fixed_route.unit_by_owner.at(GlobalUnitOwnerKey {5, 0}) == map_track(69)
            && fixed_route.stats.model.f_vars == 4,
        "V17 must preserve a fixed COBUnit and omit another port's private "
            "F arcs");
}

auto test_v17_estimated_wirelength_deduplicates_multi_terminal_channels() -> void {
    const auto a0 = tob_anchor_cob(0);
    const auto a1 = tob_anchor_cob(1);
    const auto a2 = tob_anchor_cob(2);
    const auto graph = synthetic_channel_graph(
        {{1, static_cast<int>(a0.row), static_cast<int>(a0.col)},
         {1, static_cast<int>(a1.row), static_cast<int>(a1.col)},
         {1, static_cast<int>(a2.row), static_cast<int>(a2.col)}},
        {global_tob_node(0, 0), global_cob_node(), global_tob_node(1, 1),
         global_tob_node(2, 2)},
        {{0, 1, 0}, {1, 2, 1}, {1, 3, 2}});
    auto net = RoutingNet {};
    net.net_id = 0;
    net.kind = RoutingNetKind::Bnet;
    net.sources = {bump_ref(0, 0, 0)};
    net.demands = {
        RoutingDemand {0, bump_ref(1, 0, 1), {0}, true},
        RoutingDemand {1, bump_ref(2, 0, 2), {0}, true}};

    const auto route = solve_global_route_v17(UnifiedGraph {}, graph, {net}, 0);
    require(
        route.ok
            && route.stats.objective == 3
            && route.stats.estimated_wirelength == 6,
        "estimated wirelength must count the three-Channel owner union and "
            "three unique bumps");

    const auto warm_route = solve_global_route_v17(
        UnifiedGraph {}, graph, {net}, 0, GlobalRouteCapacityMode::IterativeCuts);
    require(
        warm_route.ok
            && warm_route.stats.objective == route.stats.objective
            && warm_route.stats.mip_start_owners == 1
            && warm_route.stats.mip_start_multi_pin_owners == 1
            && warm_route.stats.mip_start_commodities == 2
            && warm_route.stats.mip_start_skipped_owners == 0
            && warm_route.stats.mip_start_entries > 0,
        "V19 maze MIP start must build one shared single-source multi-pin "
            "tree");
}

auto test_v22_global_route_reserves_sync_channel_unit_capacity() -> void {
    const auto a0 = tob_anchor_cob(0);
    const auto a1 = tob_anchor_cob(1);
    const auto graph = synthetic_channel_graph(
        {{1, static_cast<int>(a0.row), static_cast<int>(a0.col)},
         {1, static_cast<int>(a1.row), static_cast<int>(a1.col)}},
        {global_tob_node(0, 0), global_cob_node(), global_tob_node(1, 1)},
        {{0, 1, 0}, {1, 2, 1}});
    auto bnet = two_pin_net(
        22, RoutingNetKind::Bnet, bump_ref(0, 0, 0), bump_ref(1, 0, 0));
    bnet.global_unit_by_source.emplace(0, 3);
    const auto fixed = solve_global_route_v17(
        UnifiedGraph{}, graph, {bnet}, 0, GlobalRouteCapacityMode::IterativeCuts);
    require(fixed.ok
                && fixed.unit_by_owner.at(GlobalUnitOwnerKey{22, 0}) == 3,
            "V22 post guide Global Routing must retain a Bnet SAT COBUnit");

    auto peak_net = bnet;
    for (std::size_t index = 1; index < 7; ++index) {
        peak_net.demands.push_back(RoutingDemand {
            index, bump_ref(1, 0, index), {0}, true});
    }
    auto no_reservations = GlobalRouteChannelReservations{};
    const auto post_route = solve_global_route_v17(
        UnifiedGraph{}, graph, {peak_net}, 0,
        GlobalRouteCapacityMode::IterativeCuts,
        {}, false, GlobalRouteScopeMode::FullGraph, 0, &no_reservations, false);
    require(post_route.ok
                && post_route.stats.model.tob_h7_vars == 0
                && post_route.stats.model.tob_h8_vars == 0
                && post_route.stats.model.tob_peak_threshold == 0
                && post_route.stats.tob_peak_cost == 0.0,
            "V22 fixed-unit post guide must omit constant TOB peak variables");

    auto reservations = GlobalRouteChannelReservations{};
    reservations.load_by_channel_unit.resize(graph.channels.size());
    for (auto& load : reservations.load_by_channel_unit) {
        load[3] = 8;
    }
    const auto blocked = solve_global_route_v17(
        UnifiedGraph{}, graph, {bnet}, 0, GlobalRouteCapacityMode::IterativeCuts,
        {}, false, GlobalRouteScopeMode::FullGraph, 0, &reservations);
    require(!blocked.ok,
            "fixed Sync (Channel,unit) reservations must reduce post-guide capacity");
}

auto test_v19_bbox_plus_one_global_route_scope() -> void {
    const auto a0 = tob_anchor_cob(0);
    const auto a1 = tob_anchor_cob(1);
    const auto channels = std::Vector<GlobalChannelCoord> {
        {1, static_cast<int>(a0.row), static_cast<int>(a0.col)},
        {1, static_cast<int>(a1.row), static_cast<int>(a1.col)},
        {0, 2, 4},
        {0, 8, 11}};
    const auto nodes = std::Vector<GlobalRouteNode> {
        global_tob_node(0, 0),
        global_cob_node(1, 1),
        global_tob_node(1, 1),
        global_cob_node(8, 10),
        global_cob_node(8, 11)};
    const auto graph = synthetic_channel_graph(
        channels,
        nodes,
        {{0, 1, 0}, {1, 3, 2}, {3, 2, 1}, {3, 4, 3}});
    const auto net = two_pin_net(
        0, RoutingNetKind::Bnet, bump_ref(0, 0, 0), bump_ref(1, 0, 1));
    const auto scoped = solve_global_route_v17(
        UnifiedGraph {},
        graph,
        {net},
        0,
        GlobalRouteCapacityMode::IterativeCuts,
        {},
        false,
        GlobalRouteScopeMode::BboxPlusOne);
    require(
        scoped.ok
            && scoped.stats.bbox_scope_enabled
            && scoped.stats.bbox_scope_padding == 1
            && scoped.stats.model.x_vars == 3
            && scoped.stats.model.x_dense_slots == 4
            && scoped.stats.model.f_vars == 6
            && scoped.stats.model.f_dense_slots == 8,
        "V19 bbox+1 scope must retain the required +1 ring and omit "
            "unrelated X/F variables");

    const auto detour_graph = synthetic_channel_graph(
        {channels[0], channels[1], channels[3]},
        {global_tob_node(0, 0), global_cob_node(1, 1),
         global_cob_node(8, 10), global_tob_node(1, 1)},
        {{0, 1, 0}, {1, 2, 2}, {2, 3, 1}});
    const auto full = solve_global_route_v17(
        UnifiedGraph {},
        detour_graph,
        {net},
        0,
        GlobalRouteCapacityMode::IterativeCuts);
    const auto clipped = solve_global_route_v17(
        UnifiedGraph {},
        detour_graph,
        {net},
        0,
        GlobalRouteCapacityMode::IterativeCuts,
        {},
        false,
        GlobalRouteScopeMode::BboxPlusOne);
    require(
        full.ok && !clipped.ok && clipped.message == "Infeasible",
        "V19 bbox+1 experiment must report failure directly when the only "
            "detour is outside scope");

    const auto a8 = tob_anchor_cob(8);
    const auto a3 = tob_anchor_cob(3);
    const auto fanout_graph = synthetic_channel_graph(
        {{1, static_cast<int>(a0.row), static_cast<int>(a0.col)},
         {1, static_cast<int>(a8.row), static_cast<int>(a8.col)},
         {1, static_cast<int>(a3.row), static_cast<int>(a3.col)},
         {0, static_cast<int>(a8.row), static_cast<int>(a3.col)}},
        {global_tob_node(0, 0), global_cob_node(1, 0),
         global_tob_node(8, 1), global_tob_node(3, 2),
         global_cob_node(5, 9)},
        {{0, 1, 0}, {1, 2, 1}, {1, 3, 2}, {1, 4, 3}});
    auto fanout = RoutingNet {};
    fanout.net_id = 1;
    fanout.kind = RoutingNetKind::Bnet;
    fanout.sources = {bump_ref(0, 0, 0)};
    fanout.demands = {
        RoutingDemand {0, bump_ref(8, 0, 1), {0}, true},
        RoutingDemand {1, bump_ref(3, 0, 2), {0}, true}};
    const auto fanout_scoped = solve_global_route_v17(
        UnifiedGraph {},
        fanout_graph,
        {fanout},
        0,
        GlobalRouteCapacityMode::IterativeCuts,
        {},
        false,
        GlobalRouteScopeMode::BboxPlusOne);
    require(
        fanout_scoped.ok
            && fanout_scoped.stats.model.x_vars == 3
            && fanout_scoped.stats.model.x_dense_slots == 4
            && fanout_scoped.stats.model.f_vars == 12
            && fanout_scoped.stats.model.f_dense_slots == 16,
        "V19 multi-sink scope must keep the expanded child-bbox union, not "
            "its rectangular hull");
}

auto test_v17_channel_graph_collapses_real_hardware_topology() -> void {
    const auto detailed = build_unified_graph(nullptr, {});
    const auto global = build_global_channel_graph(detailed, {});
    constexpr std::size_t cob_count = hardware::Interposer::COB_ARRAY_WIDTH
        * hardware::Interposer::COB_ARRAY_HEIGHT;
    constexpr std::size_t boundary_count = 2
        * (hardware::Interposer::COB_ARRAY_WIDTH
           + hardware::Interposer::COB_ARRAY_HEIGHT);
    constexpr std::size_t internal_channel_count =
        hardware::Interposer::COB_ARRAY_HEIGHT
            * (hardware::Interposer::COB_ARRAY_WIDTH - 1)
        + (hardware::Interposer::COB_ARRAY_HEIGHT - 1)
            * hardware::Interposer::COB_ARRAY_WIDTH;
    constexpr std::size_t channel_count = internal_channel_count + boundary_count;
    constexpr std::size_t node_count = cob_count + hardware::Interposer::TOB_SIZE
        + boundary_count;
    constexpr std::size_t arc_count = 2
        * (channel_count + hardware::Interposer::TOB_SIZE);
    require(
        !global.channels.empty() && !global.arcs.empty(),
        "V17 must derive a non-empty Channel adjacency graph from the "
            "hardware graph");
    require(
        detailed.track_node_count == global.channels.size() * 128
            && global.channels.size() == channel_count
            && global.cob_node_count == cob_count
            && global.tob_terminal_node_count == 16
            && global.boundary_terminal_node_count == boundary_count
            && global.port_terminal_node_count == 0
            && global.nodes.size() == node_count
            && global.arcs.size() == arc_count,
        "V17 hardware graph must use COB/terminal nodes and physical "
            "Channels as edges");
    for (const auto& arc : global.arcs) {
        require(
            arc.u >= 0 && arc.v >= 0 && arc.u != arc.v
                && static_cast<std::size_t>(arc.u) < global.nodes.size()
                && static_cast<std::size_t>(arc.v) < global.nodes.size()
                && arc.channel >= 0
                && static_cast<std::size_t>(arc.channel) < global.channels.size(),
            "every V17 traversal arc must identify valid nodes and one "
                "Channel resource");
    }
    for (std::size_t tob = 0; tob < hardware::Interposer::TOB_SIZE; ++tob) {
        const int node = global.tob_node_by_tob.at(tob);
        const int channel = global.nodes[static_cast<std::size_t>(node)].channel;
        const auto [cob0_coord, cob1_coord] = tob_pair_cob_coords(
            tob / hardware::Interposer::TOB_ARRAY_WIDTH,
            tob % hardware::Interposer::TOB_ARRAY_WIDTH);
        const int cob0 = global.cob_node_by_coord.at(std::pair {
            static_cast<int>(cob0_coord.row), static_cast<int>(cob0_coord.col)});
        const int cob1 = global.cob_node_by_coord.at(std::pair {
            static_cast<int>(cob1_coord.row), static_cast<int>(cob1_coord.col)});
        auto endpoints = std::set<std::pair<int, int>> {};
        for (const int arc_id : global.arc_ids_by_channel[static_cast<std::size_t>(channel)]) {
            const auto& arc = global.arcs[static_cast<std::size_t>(arc_id)];
            endpoints.emplace(arc.u, arc.v);
        }
        require(
            endpoints == std::set<std::pair<int, int>> {
                {cob0, node}, {node, cob0}, {cob1, node}, {node, cob1}},
            "a TOB Channel must connect only its midpoint and two adjacent "
                "COBs");
    }

    const auto boundary_port = track_ref(
        9, 5, hardware::TrackDirection::Vertical, 69);
    const auto with_port = build_global_channel_graph(
        detailed,
        {two_pin_net(99, RoutingNetKind::Tnet, boundary_port, bump_ref(0, 0, 0))});
    const auto port_node = with_port.port_node_by_key.at(GlobalPortKey {1, 9, 5, 69});
    require(
        with_port.nodes.size() == node_count + 1
            && with_port.port_terminal_node_count == 1
            && with_port.boundary_terminal_node_count == boundary_count
            && with_port.arcs.size() == arc_count + 2
            && std::count_if(
                with_port.arcs.begin(),
                with_port.arcs.end(),
                [&](const GlobalChannelArc& arc) {
                    return arc.restricted_port_node == port_node;
                }) == 2,
        "a real boundary port must coexist with V_B and add only two "
            "private arcs");
}

auto test_v17_tob_halves_share_channel_capacity() -> void {
    auto channels = std::Vector<GlobalChannelCoord> {};
    for (int source = 0; source < 9; ++source) {
        channels.push_back(GlobalChannelCoord {0, 0, source});
    }
    channels.push_back(GlobalChannelCoord {1, 1, 0});
    for (int sink = 0; sink < 9; ++sink) {
        const auto anchor = tob_anchor_cob(static_cast<std::size_t>(sink + 1));
        channels.push_back(GlobalChannelCoord {
            1, static_cast<int>(anchor.row), static_cast<int>(anchor.col)});
    }

    auto nodes = std::Vector<GlobalRouteNode> {};
    for (int source = 0; source < 9; ++source) {
        nodes.push_back(global_port_node(
            track_ref(0, source, hardware::TrackDirection::Horizontal, 0), source));
    }
    nodes.push_back(global_cob_node(0, 0));
    nodes.push_back(global_tob_node(0, 9));
    nodes.push_back(global_cob_node(1, 0));
    for (std::size_t sink = 0; sink < 9; ++sink) {
        nodes.push_back(global_tob_node(sink + 1, static_cast<int>(10 + sink)));
    }

    auto segments = std::Vector<std::tuple<int, int, int>> {};
    for (int source = 0; source < 9; ++source) {
        segments.emplace_back(source, source < 5 ? 9 : 11, source);
    }
    segments.emplace_back(9, 10, 9);
    segments.emplace_back(11, 10, 9);
    for (int sink = 0; sink < 9; ++sink) {
        segments.emplace_back(10, 12 + sink, 10 + sink);
    }
    const auto graph = synthetic_channel_graph(channels, nodes, segments);

    auto nets = std::Vector<RoutingNet> {};
    for (std::size_t index = 0; index < 9; ++index) {
        nets.push_back(two_pin_net(
            index,
            RoutingNetKind::Tnet,
            track_ref(
                0,
                static_cast<int>(index),
                hardware::TrackDirection::Horizontal,
                0),
            bump_ref(index + 1, 0, 0)));
    }
    const auto route = solve_global_route_v17(UnifiedGraph {}, graph, nets, 0);
    require(
        !route.ok && route.message == "Infeasible",
        "nine same-unit routes on opposite TOB halves must overflow one "
            "shared Channel");

    const auto cut_route = solve_global_route_v17(
        UnifiedGraph {},
        graph,
        nets,
        0,
        GlobalRouteCapacityMode::IterativeCuts);
    require(
        !cut_route.ok
            && cut_route.message == "Infeasible"
            && cut_route.stats.capacity_cuts_enabled
            && cut_route.stats.model.w_vars == 0
            && cut_route.stats.initial_fixed_unit_capacity_rows >= 1
            && cut_route.stats.model.channel_unit_capacity
                == cut_route.stats.initial_fixed_unit_capacity_rows
            && cut_route.stats.capacity_cut_rounds == 0
            && cut_route.stats.capacity_cuts == 0,
        "fixed-unit capacity rows must reject a known overload in the "
            "first solve");

    auto bypass_channels = channels;
    bypass_channels.push_back(GlobalChannelCoord {0, 3, 0});
    bypass_channels.push_back(GlobalChannelCoord {0, 3, 1});
    bypass_channels.push_back(GlobalChannelCoord {0, 3, 2});
    auto bypass_nodes = nodes;
    bypass_nodes.push_back(global_cob_node(2, 0));
    bypass_nodes.push_back(global_cob_node(2, 1));
    auto bypass_segments = segments;
    bypass_segments.emplace_back(8, 21, 19);
    bypass_segments.emplace_back(21, 22, 20);
    bypass_segments.emplace_back(22, 20, 21);
    const auto bypass_graph = synthetic_channel_graph(
        bypass_channels, bypass_nodes, bypass_segments);
    const auto bypass_route = solve_global_route_v17(
        UnifiedGraph {},
        bypass_graph,
        nets,
        0,
        GlobalRouteCapacityMode::IterativeCuts);
    require(
        bypass_route.ok
            && bypass_route.stats.initial_fixed_unit_capacity_rows >= 1
            && bypass_route.stats.mip_start_owners == 9
            && bypass_route.stats.mip_start_skipped_owners == 0
            && bypass_route.stats.capacity_cut_rounds == 0
            && bypass_route.stats.capacity_cuts == 0,
        "the ninth fixed-unit maze route must avoid a full Channel by "
            "using its bypass");
}

auto test_v18_dynamic_sparse_capacity_repairs_overloaded_incumbent() -> void {
    auto channels = std::Vector<GlobalChannelCoord> {};
    for (int index = 0; index < 8; ++index) {
        channels.push_back(GlobalChannelCoord {0, 0, index});
    }
    for (int index = 0; index < 8; ++index) {
        channels.push_back(GlobalChannelCoord {0, 1, index});
    }
    channels.push_back(GlobalChannelCoord {1, 10, 0});
    channels.push_back(GlobalChannelCoord {0, 2, 0});
    channels.push_back(GlobalChannelCoord {0, 2, 1});
    channels.push_back(GlobalChannelCoord {1, 10, 1});
    const auto sink_anchor = tob_anchor_cob(0);
    channels.push_back(GlobalChannelCoord {
        1, static_cast<int>(sink_anchor.row), static_cast<int>(sink_anchor.col)});

    auto nodes = std::Vector<GlobalRouteNode> {};
    for (int index = 0; index < 8; ++index) {
        nodes.push_back(global_port_node(
            track_ref(0, index, hardware::TrackDirection::Horizontal, 0), index));
    }
    for (int index = 0; index < 8; ++index) {
        nodes.push_back(global_port_node(
            track_ref(1, index, hardware::TrackDirection::Horizontal, 0), 8 + index));
    }
    nodes.push_back(global_cob_node(0, 0));
    nodes.push_back(global_cob_node(0, 1));
    nodes.push_back(global_cob_node(1, 0));
    const auto pn_source0 = track_ref(2, 0, hardware::TrackDirection::Horizontal, 0);
    const auto pn_source1 = track_ref(2, 1, hardware::TrackDirection::Horizontal, 1);
    nodes.push_back(global_port_node(pn_source0, 17));
    nodes.push_back(global_port_node(pn_source1, 18));
    nodes.push_back(global_tob_node(0, 20));

    auto segments = std::Vector<std::tuple<int, int, int>> {};
    for (int index = 0; index < 8; ++index) {
        segments.emplace_back(index, 16, index);
        segments.emplace_back(17, 8 + index, 8 + index);
    }
    segments.emplace_back(16, 17, 16);
    segments.emplace_back(19, 16, 17);
    segments.emplace_back(20, 18, 18);
    segments.emplace_back(18, 16, 19);
    segments.emplace_back(17, 21, 20);
    const auto graph = synthetic_channel_graph(channels, nodes, segments);

    auto nets = std::Vector<RoutingNet> {};
    for (std::size_t index = 0; index < 7; ++index) {
        nets.push_back(two_pin_net(
            index,
            RoutingNetKind::Tnet,
            track_ref(0, static_cast<int>(index), hardware::TrackDirection::Horizontal, 0),
            track_ref(1, static_cast<int>(index), hardware::TrackDirection::Horizontal, 0)));
    }
    auto pn = RoutingNet {};
    pn.net_id = 8;
    pn.kind = RoutingNetKind::PNnet;
    pn.sources = {pn_source0, pn_source1};
    pn.demands = {RoutingDemand {0, bump_ref(0, 0, 0), {0, 1}, false}};
    nets.push_back(std::move(pn));

    auto reservations = GlobalRouteChannelReservations {};
    reservations.load_by_channel_unit.resize(graph.channels.size());
    reservations.load_by_channel_unit[16][map_track(0)] = 1;

    const auto route = solve_global_route_v17(
        UnifiedGraph {},
        graph,
        nets,
        0,
        GlobalRouteCapacityMode::IterativeCuts,
        {}, false, GlobalRouteScopeMode::FullGraph, 0, &reservations);
    require(
        route.ok
            && route.stats.initial_fixed_unit_capacity_rows == 0
            && route.stats.capacity_cut_rounds >= 1
            && route.stats.capacity_cuts >= 1
            && route.stats.model.w_vars >= 1
            && route.stats.model.w_linearization == 3 * route.stats.model.w_vars
            && route.stats.model.channel_unit_capacity
                == route.stats.initial_fixed_unit_capacity_rows
                    + route.stats.capacity_cuts
            && route.selected_source_index_by_pair.at(PairKey {8, 0, 0}) == 1
            && route.selected_unit_by_pair.at(PairKey {8, 0, 0}) == map_track(1),
        "dynamic exact sparse-W capacity must assign the PN demand to a "
            "non-overloaded unit and converge without hotspot prebuilding");
}

auto test_v17_global_route_bus_and_pn_source() -> void {
    const auto a0 = tob_anchor_cob(0);
    const auto a1 = tob_anchor_cob(1);
    const auto a2 = tob_anchor_cob(2);
    const auto a3 = tob_anchor_cob(3);
    const auto bus_graph = synthetic_channel_graph(
        {{1, static_cast<int>(a0.row), static_cast<int>(a0.col)},
         {1, static_cast<int>(a1.row), static_cast<int>(a1.col)},
         {1, static_cast<int>(a2.row), static_cast<int>(a2.col)},
         {1, static_cast<int>(a3.row), static_cast<int>(a3.col)}},
        {global_tob_node(0, 0), global_cob_node(0, 0), global_tob_node(1, 1),
         global_tob_node(2, 2), global_cob_node(0, 1), global_tob_node(3, 3)},
        {{0, 1, 0}, {1, 2, 1}, {3, 4, 2}, {4, 5, 3}});
    auto bus = RoutingNet {};
    bus.net_id = 7;
    bus.kind = RoutingNetKind::Bnet;
    bus.is_sync_bus = true;
    bus.sources = {bump_ref(0, 0, 0), bump_ref(2, 0, 1)};
    bus.demands = {
        RoutingDemand {0, bump_ref(1, 0, 2), {0}, true},
        RoutingDemand {1, bump_ref(3, 0, 3), {1}, true}};
    const auto bus_route = solve_global_route_v17(UnifiedGraph {}, bus_graph, {bus}, 0);
    require(bus_route.ok, "V17 synthetic two-member bus must be feasible");
    require(
        bus_route.channel_count_by_owner.at(GlobalUnitOwnerKey {7, 0})
            == bus_route.channel_count_by_owner.at(GlobalUnitOwnerKey {7, 1}),
        "V17 bus members must use equal Channel counts");

    const auto sink_anchor = tob_anchor_cob(0);
    const auto pn_source0 = track_ref(0, 0, hardware::TrackDirection::Horizontal, 0);
    const auto pn_source1 = track_ref(0, 2, hardware::TrackDirection::Horizontal, 1);
    const auto pn_graph = synthetic_channel_graph(
        {{0, 0, 0},
         {1, static_cast<int>(sink_anchor.row), static_cast<int>(sink_anchor.col)},
         {0, 0, 2}},
        {global_port_node(pn_source0, 0), global_cob_node(0, 0),
         global_port_node(pn_source1, 2), global_cob_node(0, 1),
         global_tob_node(0, 1)},
        {{0, 1, 0}, {2, 3, 2}, {3, 4, 1}});
    auto pn = RoutingNet {};
    pn.net_id = 8;
    pn.kind = RoutingNetKind::PNnet;
    pn.sources = {
        pn_source0,
        pn_source1};
    pn.demands = {RoutingDemand {0, bump_ref(0, 0, 4), {0, 1}, false}};
    const auto pn_route = solve_global_route_v17(UnifiedGraph {}, pn_graph, {pn}, 0);
    require(
        pn_route.ok
            && pn_route.selected_source_index_by_pair.at(PairKey {8, 0, 0}) == 1
            && pn_route.selected_unit_by_pair.at(PairKey {8, 0, 0}) == 1,
        "V17 PN demand must select a reachable source and matching unit");
    auto pn_nets = std::Vector<RoutingNet> {pn};
    auto pn_state = init_routing_problem_state(pn_nets);
    apply_global_route_v17(pn_route, pn_state, pn_nets);
    require(
        pn_nets.front().global_selected_pn_source_indices
            == std::set<std::size_t> {1},
        "V17 PN source choice must be propagated into the detailed-routing "
            "scope");
}

auto test_v17_pn_net_z_deduplicates_per_demand_channels() -> void {
    const auto a0 = tob_anchor_cob(0);
    const auto a1 = tob_anchor_cob(1);
    const auto source = track_ref(0, 0, hardware::TrackDirection::Horizontal, 0);
    const auto graph = synthetic_channel_graph(
        {{0, 0, 0},
         {1, static_cast<int>(a0.row), static_cast<int>(a0.col)},
         {1, static_cast<int>(a1.row), static_cast<int>(a1.col)}},
        {global_port_node(source, 0), global_cob_node(), global_tob_node(0, 1),
         global_tob_node(1, 2)},
        {{0, 1, 0}, {1, 2, 1}, {1, 3, 2}});
    auto pn = RoutingNet {};
    pn.net_id = 20;
    pn.kind = RoutingNetKind::PNnet;
    pn.sources = {source};
    pn.demands = {
        RoutingDemand {0, bump_ref(0, 0, 0), {0}, false},
        RoutingDemand {1, bump_ref(1, 0, 1), {0}, false}};

    const auto route = solve_global_route_v17(UnifiedGraph {}, graph, {pn}, 0);
    require(
        route.ok
            && route.stats.owners == 2
            && route.stats.commodities == 2
            && route.stats.model.z_vars == 3
            && route.stats.model.source_choice_vars == 2
            && route.channel_count_by_owner.at(GlobalUnitOwnerKey {20, 0}) == 2
            && route.channel_count_by_owner.at(GlobalUnitOwnerKey {20, 1}) == 2
            && route.stats.objective == 3
            && route.stats.estimated_wirelength == 5,
        "PNnet Z must count the Channel union of independent demand owners");
    for (std::size_t demand = 0; demand < pn.demands.size(); ++demand) {
        require(
            route.selected_source_index_by_pair.at(PairKey {20, demand, 0}) == 0,
            "each PN demand must retain its own selected source");
    }
}

auto test_v17_pn_z_deduplicates_across_units() -> void {
    const auto anchor = tob_anchor_cob(0);
    const auto source0 = track_ref(0, 0, hardware::TrackDirection::Horizontal, 0);
    const auto source1 = track_ref(0, 0, hardware::TrackDirection::Horizontal, 1);
    const auto graph = synthetic_channel_graph(
        {{0, 0, 0},
         {1, static_cast<int>(anchor.row), static_cast<int>(anchor.col)}},
        {global_port_node(source0, 0), global_port_node(source1, 0),
         global_cob_node(), global_tob_node(0, 1)},
        {{0, 2, 0}, {1, 2, 0}, {2, 3, 1}});
    auto pn = RoutingNet {};
    pn.net_id = 21;
    pn.kind = RoutingNetKind::PNnet;
    pn.sources = {source0, source1};
    pn.demands = {
        RoutingDemand {0, bump_ref(0, 0, 0), {0}, false},
        RoutingDemand {1, bump_ref(0, 0, 1), {1}, false}};

    const auto route = solve_global_route_v17(UnifiedGraph {}, graph, {pn}, 0);
    require(
        route.ok
            && route.stats.owners == 2
            && route.selected_unit_by_pair.at(PairKey {21, 0, 0}) == 0
            && route.selected_unit_by_pair.at(PairKey {21, 1, 0}) == 1
            && route.channel_count_by_owner.at(GlobalUnitOwnerKey {21, 0}) == 2
            && route.channel_count_by_owner.at(GlobalUnitOwnerKey {21, 1}) == 2
            && route.stats.model.z_vars == 2
            && route.stats.objective == 2
            && route.stats.estimated_wirelength == 4,
        "one PNnet must count a shared Channel once even when demands select "
        "different units");
}

auto test_v17_pn_z_does_not_relax_per_demand_capacity() -> void {
    const auto source = track_ref(0, 0, hardware::TrackDirection::Horizontal, 0);
    auto channels = std::Vector<GlobalChannelCoord> {{0, 0, 0}};
    auto nodes = std::Vector<GlobalRouteNode> {global_port_node(source, 0), global_cob_node()};
    auto segments = std::Vector<std::tuple<int, int, int>> {{0, 1, 0}};
    auto pn = RoutingNet {};
    pn.net_id = 22;
    pn.kind = RoutingNetKind::PNnet;
    pn.sources = {source};
    for (std::size_t demand = 0; demand < 9; ++demand) {
        const auto anchor = tob_anchor_cob(demand);
        channels.push_back(GlobalChannelCoord {
            1, static_cast<int>(anchor.row), static_cast<int>(anchor.col)});
        nodes.push_back(global_tob_node(demand, static_cast<int>(demand + 1)));
        segments.emplace_back(1, static_cast<int>(demand + 2), static_cast<int>(demand + 1));
        pn.demands.push_back(RoutingDemand {
            demand, bump_ref(demand, 0, 0), {0}, false});
    }

    const auto route = solve_global_route_v17(
        UnifiedGraph {}, synthetic_channel_graph(channels, nodes, segments), {pn}, 0);
    require(
        !route.ok && route.stats.model.z_vars == channels.size(),
        "PNnet Z must not merge nine per-demand uses of one Channel for "
            "capacity");
}

auto test_v17_rejects_tob_necessary_condition_overflow() -> void {
    const auto anchor = tob_anchor_cob(0);
    auto channels = std::Vector<GlobalChannelCoord> {};
    auto nodes = std::Vector<GlobalRouteNode> {};
    auto segments = std::Vector<std::tuple<int, int, int>> {};
    for (int source = 0; source < 9; ++source) {
        channels.push_back(GlobalChannelCoord {0, 0, source});
        nodes.push_back(global_port_node(
            track_ref(0, source, hardware::TrackDirection::Horizontal, 0), source));
        nodes.push_back(global_port_node(
            track_ref(0, source, hardware::TrackDirection::Horizontal, 64), source));
    }
    channels.push_back(GlobalChannelCoord {
        1, static_cast<int>(anchor.row), static_cast<int>(anchor.col)});
    nodes.push_back(global_cob_node());
    nodes.push_back(global_tob_node(0, 9));
    for (int source = 0; source < 9; ++source) {
        segments.emplace_back(2 * source, 18, source);
        segments.emplace_back(2 * source + 1, 18, source);
    }
    segments.emplace_back(18, 19, 9);
    const auto graph = synthetic_channel_graph(channels, nodes, segments);

    auto make_fixed_nets = [&](const bool alternate_units, const bool alternate_banks) {
        auto nets = std::Vector<RoutingNet> {};
        for (std::size_t index = 0; index < 9; ++index) {
            const std::size_t track = alternate_units && index % 2 != 0 ? 64 : 0;
            const std::size_t bank = alternate_banks ? index % 2 : 0;
            nets.push_back(two_pin_net(
                index,
                RoutingNetKind::Tnet,
                track_ref(
                    0,
                    static_cast<int>(index),
                    hardware::TrackDirection::Horizontal,
                    track),
                bump_ref(0, bank, index)));
        }
        return nets;
    };

    const auto unit_overflow = solve_global_route_v17(
        UnifiedGraph {}, graph, make_fixed_nets(false, true), 0);
    require(
        !unit_overflow.ok,
        "V17 must reject more than eight bumps assigned to one TOB unit");

    const auto residue_overflow = solve_global_route_v17(
        UnifiedGraph {}, graph, make_fixed_nets(true, false), 0);
    require(
        !residue_overflow.ok,
        "V17 must reject more than eight same-bank "
                                  "bumps in one unit residue pair");
}
