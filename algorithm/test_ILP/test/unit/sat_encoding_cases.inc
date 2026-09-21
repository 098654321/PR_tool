// Detailed SAT encoding, unit selectors, bus equality, CLI parsing, initial
// padding, TOB switch aggregation, and encoding-statistics tests.

auto test_v14_bnet_unit_selectors_only() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});

    {
        auto bnet = synthetic_net(0, {0}, {{2, {0}}});
        bnet.kind = RoutingNetKind::Bnet;
        auto session = CadicalSession {};
        SatEncodingStats stats {};
        const auto model = build_v14_model(session, graph, {bnet}, &stats, false);
        require(
            model.sources[0].unit_selector_var_by_unit.size() == 16,
            "Bnet source must allocate one Q selector for each COBUnit");
        require(
            stats.unit_selector_vars == 16,
            "Q statistics must count all Bnet unit selectors");
        require(
            stats.clause_counts[static_cast<std::size_t>(
                SatClauseCategory::SourceUnitSelection)] > 0,
            "Bnet ExactlyOne and A-to-Q clauses need a separate statistics "
                "category");
    }

    {
        auto session = CadicalSession {};
        const auto model = build_v14_model(
            session,
            graph,
            {synthetic_net(1, {0}, {{2, {0}}})},
            nullptr,
            false);
        require(
            model.sources[0].unit_selector_var_by_unit.empty(),
            "Tnet source must not allocate dynamic unit selectors");
    }

    {
        auto pn_graph = synthetic_graph(3, {{0, 2}, {1, 2}});
        pn_graph.nodes[0].kind = UnifiedNodeKind::Track;
        pn_graph.nodes[0].unit = 0;
        pn_graph.nodes[1].kind = UnifiedNodeKind::Track;
        pn_graph.nodes[1].unit = 1;
        auto pnnet = synthetic_net(2, {0, 1}, {{2, {0, 1}}});
        pnnet.kind = RoutingNetKind::PNnet;
        auto session = CadicalSession {};
        const auto model = build_v14_model(session, pn_graph, {pnnet}, nullptr, false);
        require(
            model.sources[0].unit_selector_var_by_unit.empty(),
            "PNnet virtual source must not allocate dynamic unit selectors");
    }
}

auto bnet_two_unit_graph() -> UnifiedGraph {
    auto graph = synthetic_graph(
        6,
        {{0, 1}, {1, 2}, {2, 5}, {0, 3}, {3, 4}, {4, 5}});
    graph.nodes[1].kind = UnifiedNodeKind::VLine;
    graph.nodes[1].line_index = 0;
    graph.nodes[2].kind = UnifiedNodeKind::Track;
    graph.nodes[2].unit = 0;
    graph.nodes[3].kind = UnifiedNodeKind::VLine;
    graph.nodes[3].line_index = 1;
    graph.nodes[4].kind = UnifiedNodeKind::Track;
    graph.nodes[4].unit = 1;
    for (const int arc_id : {1, 4}) {
        auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        arc.physical_switch_kind = PhysicalSwitchKind::VLineTrack;
        arc.physical_switch_id = 200 + arc_id;
        arc.mode_group_id = arc_id == 1 ? 0 : 1;
        arc.is_vline_track_straight = true;
    }
    return graph;
}

auto test_v14_bnet_unit_selector_rejects_two_units() -> void {
    const auto graph = bnet_two_unit_graph();
    auto bnet = synthetic_net(0, {0}, {{5, {0}}});
    bnet.kind = RoutingNetKind::Bnet;

    for (const int selected_arc : {1, 4}) {
        auto session = CadicalSession {};
        const auto model = build_v14_model(session, graph, {bnet});
        session.add_clause({a_lit_at(model, selected_arc, 2)});
        require(
            session.solve_once().ok,
            "a Bnet must be able to select either individually reachable unit");
    }

    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, {bnet});
    session.add_clause({a_lit_at(model, 1, 2)});
    session.add_clause({a_lit_at(model, 4, 2)});
    require(
        !session.solve_once().ok,
        "forcing one Bnet source through two "
                                      "different track units must be UNSAT");
}

auto test_v14_pure_track_chain_sat() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {2, 3}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto session = CadicalSession {};
    (void)build_v14_model(session, graph, nets);
    require(session.solve_once().ok, "pure track chain must be SAT");
}

auto test_v14_pure_track_fork_sat() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {0, 2}, {1, 3}, {2, 3}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto session = CadicalSession {};
    (void)build_v14_model(session, graph, nets);
    require(session.solve_once().ok, "pure track fork must be SAT");
}

auto test_v14_pure_track_unreachable_unsat() -> void {
    const auto graph = synthetic_graph(3, {});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    try {
        const auto scopes = build_all_scopes(graph, nets);
        (void)compute_pair_delays(graph, nets, scopes);
        require(false, "disconnected graph must fail delay precompute");
    }
    catch (const std::runtime_error&) {
    }
}

auto test_v14_bus_equal_delay_equiv() -> void {
    const auto graph = synthetic_graph(6, {{0, 1}, {1, 2}, {3, 4}, {4, 5}});
    auto net = synthetic_net(0, {0, 3}, {{2, {0}}, {5, {1}}});
    net.is_sync_bus = true;
    auto session = CadicalSession {};
    (void)build_v14_model(session, graph, {net});
    require(session.solve_once().ok, "equal shortest bus members must be SAT");
}

auto test_v14_bus_forall_d_equiv() -> void {
    const auto graph = synthetic_graph(
        9, {{0, 1}, {1, 2}, {2, 8}, {3, 4}, {4, 5}, {5, 6}});
    auto net = synthetic_net(0, {0, 3}, {{8, {0}}, {6, {1}}});
    net.is_sync_bus = true;
    auto session = CadicalSession {};
    SatEncodingStats stats {};
    (void)build_v14_model(session, graph, {net}, &stats);
    const auto sync_clauses =
        stats.clause_counts[static_cast<std::size_t>(SatClauseCategory::SyncBusEqualLength)];
    require(
        sync_clauses >= 2,
        "bus forall-d equal length must encode at least one sink equiv layer");
}

auto test_v14_bus_sync_covers_reachable_delay_domain() -> void {
    const auto graph = synthetic_graph(
        10,
        {{0, 1}, {1, 2}, {0, 4}, {4, 5}, {5, 2}, {6, 7}, {7, 8}, {8, 9}});
    auto net = synthetic_net(0, {0, 6}, {{2, {0}}, {9, {1}}});
    net.is_sync_bus = true;
    auto session = CadicalSession {};
    SatEncodingStats stats {};
    (void)build_v14_model(session, graph, {net}, &stats);

    const auto sync_clauses =
        stats.clause_counts[static_cast<std::size_t>(SatClauseCategory::SyncBusEqualLength)];
    require(
        sync_clauses == 2,
        "bus encoding must equate the shared reachable sink delay");
}

auto test_cli_max_rss_option() -> void {
    const auto parsed = parse_test_ilp_cli({
        "test/config/case1", "-vv", "--sat-log", "--max-rss-mb", "10240"});
    require(parsed.config_path == "test/config/case1", "CLI must preserve the config path");
    require(parsed.verbose_level == 2, "CLI must preserve -vv");
    require(parsed.enable_sat_log, "CLI must preserve --sat-log");
    require(parsed.max_rss_mb == 10240, "CLI must parse a positive RSS limit");

    const auto unlimited = parse_test_ilp_cli({"test/config/case1", "-v"});
    require(unlimited.max_rss_mb == 0, "missing RSS option must mean unlimited internally");

    const auto require_invalid = [](std::initializer_list<std::string_view> args) {
        try {
            (void)parse_test_ilp_cli(args);
            require(false, "invalid --max-rss-mb input must be rejected");
        }
        catch (const std::invalid_argument&) {
        }
    };
    require_invalid({"test/config/case1", "--max-rss-mb"});
    require_invalid({"test/config/case1", "--max-rss-mb", "0"});
    require_invalid({"test/config/case1", "--max-rss-mb", "-1"});
    require_invalid({"test/config/case1", "--max-rss-mb", "12MB"});
}

auto test_cli_time_limit_option() -> void {
    const auto parsed = parse_test_ilp_cli({
        "test/config/case1", "--global-route-v18", "--time-limit", "30"});
    require(parsed.highs_time_limit_minutes == 30, "CLI must parse --time-limit in minutes");

    const auto unlimited = parse_test_ilp_cli({"test/config/case1", "--global-route-v18"});
    require(
        unlimited.highs_time_limit_minutes == 0,
        "missing --time-limit must mean no HiGHS time limit");

    const auto require_invalid = [](std::initializer_list<std::string_view> args) {
        try {
            (void)parse_test_ilp_cli(args);
            require(false, "invalid --time-limit input must be rejected");
        }
        catch (const std::invalid_argument&) {
        }
    };
    require_invalid({"test/config/case1", "--time-limit"});
    require_invalid({"test/config/case1", "--time-limit", "0"});
    require_invalid({"test/config/case1", "--time-limit", "-1"});
    require_invalid({"test/config/case1", "--time-limit", "1.5"});
    require_invalid({"test/config/case1", "--time-limit", "10min"});
}

auto test_cli_initial_padding_options() -> void {
    const auto parsed = parse_test_ilp_cli({
        "algorithm/test_ILP/test/case_2btb",
        "-v",
        "--max-rss-mb",
        "8192",
        "-s",
        "2",
        "-d",
        "3"});
    require(
        parsed.config_path == "algorithm/test_ILP/test/case_2btb",
        "CLI must preserve config path with padding options");
    require(parsed.verbose_level == 1, "CLI must preserve -v with padding options");
    require(parsed.max_rss_mb == 8192, "CLI must preserve --max-rss-mb with padding options");
    require(parsed.initial_scope_pad == 2, "CLI must parse -s");
    require(parsed.initial_delay_pad == 3, "CLI must parse -d");

    const auto zero_pads = parse_test_ilp_cli({"case_2btt", "-s", "0", "-d", "0"});
    require(zero_pads.initial_scope_pad == 0, "CLI must accept -s 0");
    require(zero_pads.initial_delay_pad == 0, "CLI must parse -d 0");

    const auto require_invalid = [](std::initializer_list<std::string_view> args) {
        try {
            (void)parse_test_ilp_cli(args);
            require(false, "invalid padding CLI input must be rejected");
        }
        catch (const std::invalid_argument&) {
        }
    };
    require_invalid({"case_2btt", "-s"});
    require_invalid({"case_2btt", "-d"});
    require_invalid({"case_2btt", "-s", "-1"});
    require_invalid({"case_2btt", "-d", "x"});
}

auto test_cli_output_dir_option() -> void {
    const auto parsed = parse_test_ilp_cli({
        "test/config/case1", "-v", "-o", "output/case1_run"});
    require(parsed.config_path == "test/config/case1", "CLI must preserve the config path");
    require(parsed.verbose_level == 1, "CLI must preserve -v with -o");
    require(parsed.output_dir == "output/case1_run", "CLI must parse -o output directory");

    const auto long_form = parse_test_ilp_cli({
        "test/config/case1", "--output", "out/exp"});
    require(
        long_form.output_dir == "out/exp",
        "CLI must accept --output as an alias of -o");

    const auto defaults = parse_test_ilp_cli({"test/config/case1", "-v"});
    require(defaults.output_dir == ".", "missing -o must keep the default output directory");

    const auto require_invalid = [](std::initializer_list<std::string_view> args) {
        try {
            (void)parse_test_ilp_cli(args);
            require(false, "invalid -o/--output input must be rejected");
        }
        catch (const std::invalid_argument&) {
        }
    };
    require_invalid({"test/config/case1", "-o"});
    require_invalid({"test/config/case1", "--output"});
    require_invalid({"test/config/case1", "-o", "-v"});
    require_invalid({"test/config/case1", "--output", "--sat-log"});
}

auto test_initial_search_padding_scope() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {2, 3}});
    auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto state = init_routing_problem_state(nets);
    const auto before = state.pairs[0].pair_bbox;
    apply_initial_search_padding(state, nets, graph, 1, 0);
    const auto expected = expand_pair_bbox_one_cell(before);
    require_bbox(
        state.pairs[0].pair_bbox,
        expected.row_min,
        expected.row_max,
        expected.col_min,
        expected.col_max,
        "initial scope padding must expand pair bbox by one cell");
    require(state.pairs[0].delays.empty(), "scope-only padding must leave delays empty");
}

auto test_initial_search_padding_delay() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {2, 3}});
    auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto state = init_routing_problem_state(nets);
    apply_initial_search_padding(state, nets, graph, 0, 2);
    require(
        state.pairs[0].delays == std::Vector<int>({3, 4, 5}),
        "initial delay padding must span d_min through d_min+d");
}

auto test_initial_search_padding_rejects_delay_overflow() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}})};
    auto state = init_routing_problem_state(nets);
    try {
        apply_initial_search_padding(
            state,
            nets,
            graph,
            0,
            std::numeric_limits<int>::max());
        require(false, "overflowing initial delay padding must be rejected");
    }
    catch (const std::overflow_error&) {
    }
}

auto test_initial_search_padding_fanout_keeps_pair_delays_independent() -> void {
    const auto graph = synthetic_graph(
        6, {{0, 1}, {1, 2}, {0, 3}, {3, 4}, {4, 5}});
    auto fanout = synthetic_net(1, {0}, {{2, {0}}, {5, {0}}});
    auto nets = std::Vector<RoutingNet> {fanout};
    auto state = init_routing_problem_state(nets);
    apply_initial_search_padding(state, nets, graph, 0, 1);
    require(
        state.pairs[0].delays == std::Vector<int>({2, 3}),
        "fanout demand 0 must pad from its own d_min");
    require(
        state.pairs[1].delays == std::Vector<int>({3, 4}),
        "fanout demand 1 must pad from its own d_min");
}

auto test_initial_search_padding_bus_uses_shared_delay_without_bbox_merge() -> void {
    const auto graph = synthetic_graph(
        7, {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {5, 6}});
    auto bus = synthetic_net(2, {0, 3}, {{2, {0}}, {6, {1}}});
    bus.is_sync_bus = true;
    auto nets = std::Vector<RoutingNet> {bus};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].pair_bbox = IlpBoundingBox {0, 0, 0, 0};
    state.pairs[1].pair_bbox = IlpBoundingBox {0, 1, 0, 1};
    const auto bbox0 = state.pairs[0].pair_bbox;
    const auto bbox1 = state.pairs[1].pair_bbox;
    apply_state_to_nets(state, nets);

    apply_initial_search_padding(state, nets, graph, 0, 1);

    require(
        state.pairs[0].delays == std::Vector<int>({3, 4})
            && state.pairs[1].delays == std::Vector<int>({3, 4}),
        "bus members must pad from the shared bus_d_min");
    require_bbox(
        state.pairs[0].pair_bbox,
        bbox0.row_min,
        bbox0.row_max,
        bbox0.col_min,
        bbox0.col_max,
        "delay padding must not change bus member 0 bbox");
    require_bbox(
        state.pairs[1].pair_bbox,
        bbox1.row_min,
        bbox1.row_max,
        bbox1.col_min,
        bbox1.col_max,
        "delay padding must not change bus member 1 bbox");
}

auto test_initial_search_padding_applies_scope_before_delay() -> void {
    auto graph = synthetic_graph(
        5, {{0, 1}, {1, 2}, {2, 3}, {0, 4}, {4, 3}});
    for (const int node : {1, 2}) {
        graph.nodes[static_cast<std::size_t>(node)].kind = UnifiedNodeKind::Track;
        graph.nodes[static_cast<std::size_t>(node)].track_dir = 0;
        graph.nodes[static_cast<std::size_t>(node)].track_row = 0;
        graph.nodes[static_cast<std::size_t>(node)].track_col = 1;
    }
    graph.nodes[4].kind = UnifiedNodeKind::Track;
    graph.nodes[4].track_dir = 0;
    graph.nodes[4].track_row = 1;
    graph.nodes[4].track_col = 1;

    auto nets = std::Vector<RoutingNet> {
        synthetic_net(3, {0}, {{3, {0}}})};
    auto state = init_routing_problem_state(nets);
    apply_state_to_nets(state, nets);
    const auto initial_scopes = build_all_scopes(graph, nets);
    require(
        bfs_shortest_delay(graph, initial_scopes[0], 0, 3) == 3,
        "fixture must have d_min=3 before scope padding");

    apply_initial_search_padding(state, nets, graph, 1, 1);
    require(
        state.pairs[0].delays == std::Vector<int>({2, 3}),
        "delay padding must use d_min from the expanded scope");
}

auto test_sequential_at_most_one_truth_table_and_scaling() -> void {
    for (int mask = 0; mask < 8; ++mask) {
        auto session = CadicalSession {};
        const auto literals =
            std::Vector<int> {session.new_var(), session.new_var(), session.new_var()};
        add_sequential_at_most_one(session, literals);
        for (std::size_t bit = 0; bit < literals.size(); ++bit) {
            session.add_clause({
                (mask & (1 << bit)) != 0 ? literals[bit] : -literals[bit]});
        }
        const auto solved = session.solve_once();
        require(
            solved.ok == (std::popcount(static_cast<unsigned>(mask)) <= 1),
            "sequential AMO truth table must allow exactly zero/one true "
                "literal");
    }

    auto forced_two = CadicalSession {};
    const auto forced_literals = std::Vector<int> {
        forced_two.new_var(), forced_two.new_var(), forced_two.new_var()};
    add_sequential_at_most_one(forced_two, forced_literals);
    forced_two.add_clause({forced_literals[0]});
    forced_two.add_clause({forced_literals[2]});
    require(!forced_two.solve_once().ok, "sequential AMO must reject two forced true literals");

    auto scaling = CadicalSession {};
    auto many_literals = std::Vector<int> {};
    for (int i = 0; i < 100; ++i) {
        many_literals.push_back(scaling.new_var());
    }
    add_sequential_at_most_one(scaling, many_literals);
    require(
        scaling.num_vars() == 199,
        "Sinz AMO must allocate n-1 auxiliary variables");
    require(
        scaling.num_clauses() <= 3 * many_literals.size(),
        "Sinz AMO clause count must scale linearly");
}

auto test_numeric_fixed_path_and_extraction() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(7, {0}, {{2, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "connected fixed demand must be SAT");
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok && result.paths.size() == 1, "SAT extraction must return every demand");
    require(result.paths[0].demand_id == 0, "extraction must preserve demand_id");
    require(
        result.paths[0].node_path == std::Vector<int>({0, 1, 2}),
        "extraction must follow the unique D distance chain");
    require(
        total_wirelength(graph, result) == 3,
        "extracted path wirelength must count bump hops only in synthetic "
            "graph");

    const auto disconnected = synthetic_graph(3, {});
    auto disconnected_session = CadicalSession {};
    try {
        (void)build_v14_model(disconnected_session, disconnected, nets);
        require(false, "disconnected fixed demand must fail delay precompute");
    }
    catch (const std::runtime_error&) {
    }
}

auto test_v14_rejects_multi_candidate_demand() -> void {
    const auto graph = synthetic_graph(4, {{0, 2}, {1, 2}, {2, 3}, {0, 3}});
    auto net = synthetic_net(1, {0, 1}, {{3, {0, 1}}});
    net.kind = RoutingNetKind::Tnet;
    net.demands[0].fixed_pair = false;
    try {
        validate_v14_routing_nets({net});
        require(false, "multi-candidate Tnet demand must be rejected in v14");
    }
    catch (const std::invalid_argument&) {
    }

    auto pnnet = synthetic_net(2, {0, 1}, {{3, {0, 1}}});
    pnnet.kind = RoutingNetKind::PNnet;
    pnnet.demands[0].fixed_pair = false;
    validate_v14_routing_nets({pnnet});
}

auto test_logical_source_exclusivity() -> void {
    const auto graph = synthetic_graph(3, {{0, 2}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}}),
        synthetic_net(1, {1}, {{2, {0}}})};
    auto session = CadicalSession {};
    (void)build_v14_model(session, graph, nets);
    require(!session.solve_once().ok, "distinct logical sources must be exclusive at a shared node");
}

auto test_logical_source_owns_its_source_node() -> void {
    const auto graph = synthetic_graph(2, {{0, 1}});
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{1, {0}}})};

    auto value_session = CadicalSession {};
    const auto value_model = build_v14_model(value_session, graph, nets);
    const int source_d0 = d_lit_at(value_model, 0, 0, 0);
    const auto solved = value_session.solve_once();
    require(
        solved.ok && value_session.value(source_d0),
        "logical D(source, source-node, 0) must be true in every SAT model");

    auto forced_false_session = CadicalSession {};
    const auto forced_false_model =
        build_v14_model(forced_false_session, graph, nets);
    const int forced_false_d0 = d_lit_at(forced_false_model, 0, 0, 0);
    forced_false_session.add_clause({-forced_false_d0});
    require(
        !forced_false_session.solve_once().ok,
        "forcing logical D(source, source-node, 0)=false must be UNSAT");
}

auto test_sat_encoding_stats_reconcile() -> void {
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}})};

    const auto source_in_graph = synthetic_graph(3, {{0, 2}, {1, 0}});
    auto source_in_session = CadicalSession {};
    SatEncodingStats source_in_stats {};
    (void)build_v14_model(
        source_in_session, source_in_graph, nets, &source_in_stats);
    require(
        source_in_stats.d_vars > 0,
        "encoding stats must count allocated D variables");
    require(
        source_in_stats.total_clauses() == source_in_session.num_clauses(),
        "encoding stats must reconcile clause categories with session total");
    require(
        source_in_stats.clause_counts[static_cast<std::size_t>(
            SatClauseCategory::SyncBusEqualLength)]
            == 0,
        "non-sync fixture must have zero sync bus equal-length clauses");
}

auto test_v16_occupancy_stats_are_not_auxiliary() -> void {
    auto stats = SatEncodingStats {};
    stats.d_vars = 4;
    stats.occupancy_vars = 3;
    stats.occupancy_implication_clauses = 9;
    stats.add_clauses(SatClauseCategory::NodeOccupancy, stats.occupancy_implication_clauses);
    stats.finalize_variables(7);

    require(stats.encoding_aux_vars == 0, "U variables must not be reported as encoding auxiliary");
    require(stats.known_primary_vars() == 7, "occupancy variables must be primary variables");
    require(stats.total_clauses() == 9, "D=>U clauses must have their own CNF category");
}

auto test_source_distance_constants() -> void {
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}})};

    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const int source_d0 = d_lit_at(model, 0, 0, 0);
    session.add_clause({-source_d0});
    require(!session.solve_once().ok, "forcing D(source,source,0)=false must be UNSAT");
}

auto test_mode_group_zero_conflict() -> void {
    auto graph = synthetic_graph(4, {{0, 2}, {1, 3}});
    graph.arcs[0].physical_switch_kind = PhysicalSwitchKind::VLineTrack;
    graph.arcs[1].physical_switch_kind = PhysicalSwitchKind::VLineTrack;
    graph.arcs[0].physical_switch_id = 70;
    graph.arcs[1].physical_switch_id = 71;
    graph.arcs[0].mode_group_id = 0;
    graph.arcs[0].is_vline_track_straight = true;
    graph.arcs[1].mode_group_id = 0;
    graph.arcs[1].is_vline_track_swap = true;
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}}),
        synthetic_net(1, {1}, {{3, {0}}})};
    auto session = CadicalSession {};
    (void)build_v14_model(session, graph, nets);
    require(!session.solve_once().ok, "group zero straight/swap uses must conflict");
}

auto test_all_mode_groups_and_group_zero_extraction() -> void {
    auto graph = synthetic_graph(2, {{0, 1}});
    graph.arcs[0].physical_switch_kind = PhysicalSwitchKind::VLineTrack;
    graph.arcs[0].physical_switch_id = 77;
    graph.arcs[0].mode_group_id = 0;
    graph.arcs[0].is_vline_track_straight = true;
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{1, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    require(
        model.mode_var_by_group.size() == 16 * 64,
        "model must allocate all 1024 global VLineTrack mode groups");
    require(
        model.switch_var_by_id.contains(77),
        "VLineTrack use must aggregate into a physical-switch Y variable");
    const auto solved = session.solve_once();
    require(solved.ok, "single group-zero straight path must be SAT");
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(
        result.ok && result.vline_mode_straight_by_group.size() == 16 * 64,
        "extraction must return all 1024 mode states");
    require(
        result.used_tob_switch_ids == std::Vector<int>({77}),
        "extraction must return the used VLineTrack physical switch ID");
    require(
        result.vline_mode_straight_by_group.at(0),
        "a used group-zero straight arc must extract M_0=true");
}

auto test_y_aggregation_and_partial_matching() -> void {
    auto graph = synthetic_graph(
        5, {{0, 1}, {1, 0}, {0, 2}, {2, 0}, {3, 0}, {0, 4}});
    graph.nodes[1].kind = UnifiedNodeKind::HLine;
    graph.nodes[2].kind = UnifiedNodeKind::HLine;
    for (std::size_t i = 0; i < 2; ++i) {
        graph.arcs[i].physical_switch_id = 5;
        graph.arcs[i].physical_switch_kind = PhysicalSwitchKind::BumpH;
    }
    for (std::size_t i = 2; i < 4; ++i) {
        graph.arcs[i].physical_switch_id = 6;
        graph.arcs[i].physical_switch_kind = PhysicalSwitchKind::BumpH;
    }
    auto session = CadicalSession {};
    const auto model = build_v14_model(
        session, graph, {synthetic_net(0, {3}, {{1, {0}}, {2, {0}}})});
    require(
        model.switch_var_by_id.size() == 2,
        "reverse arcs and all pair uses must aggregate to one Y per "
            "physical switch");
    session.add_clause({model.switch_var_by_id.at(5)});
    session.add_clause({model.switch_var_by_id.at(6)});
    require(!session.solve_once().ok, "two bump-to-h switches at one bump must violate matching");
}

auto matching_fixture(
    UnifiedNodeKind center_kind,
    UnifiedNodeKind leaf0_kind,
    UnifiedNodeKind leaf1_kind,
    PhysicalSwitchKind switch_kind
) -> std::pair<UnifiedGraph, RoutingNet> {
    auto graph = synthetic_graph(
        6, {{0, 1}, {1, 0}, {0, 2}, {2, 0}, {3, 0}, {0, 4}, {0, 5}});
    graph.nodes[0].kind = center_kind;
    graph.nodes[1].kind = leaf0_kind;
    graph.nodes[2].kind = leaf1_kind;
    for (std::size_t arc_id = 0; arc_id < 2; ++arc_id) {
        graph.arcs[arc_id].physical_switch_id = 10;
        graph.arcs[arc_id].physical_switch_kind = switch_kind;
    }
    for (std::size_t arc_id = 2; arc_id < 4; ++arc_id) {
        graph.arcs[arc_id].physical_switch_id = 11;
        graph.arcs[arc_id].physical_switch_kind = switch_kind;
    }
    return {
        std::move(graph),
        synthetic_net(0, {3}, {{1, {0}}, {2, {0}}})};
}

auto matching_side_is_unsat(
    UnifiedNodeKind center_kind,
    UnifiedNodeKind leaf0_kind,
    UnifiedNodeKind leaf1_kind,
    PhysicalSwitchKind switch_kind
) -> bool {
    auto [graph, net] = matching_fixture(
        center_kind, leaf0_kind, leaf1_kind, switch_kind);
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, {net});
    session.add_clause({model.switch_var_by_id.at(10)});
    session.add_clause({model.switch_var_by_id.at(11)});
    return !session.solve_once().ok;
}

auto test_all_four_y_partial_matching_sides() -> void {
    const bool bump_side = matching_side_is_unsat(
        UnifiedNodeKind::Bump,
        UnifiedNodeKind::HLine,
        UnifiedNodeKind::HLine,
        PhysicalSwitchKind::BumpH);
    const bool bump_hline_side = matching_side_is_unsat(
        UnifiedNodeKind::HLine,
        UnifiedNodeKind::Bump,
        UnifiedNodeKind::Bump,
        PhysicalSwitchKind::BumpH);
    const bool hline_vline_side = matching_side_is_unsat(
        UnifiedNodeKind::HLine,
        UnifiedNodeKind::VLine,
        UnifiedNodeKind::VLine,
        PhysicalSwitchKind::HLineVLine);
    const bool vline_side = matching_side_is_unsat(
        UnifiedNodeKind::VLine,
        UnifiedNodeKind::HLine,
        UnifiedNodeKind::HLine,
        PhysicalSwitchKind::HLineVLine);
    require(bump_side, "Bump->H matching must be exclusive at the bump side");
    require(bump_hline_side, "H->Bump matching must be exclusive at the hline side");
    require(hline_vline_side, "H->V matching must be exclusive at the hline side");
    require(vline_side, "V->H matching must be exclusive at the vline side");
}

auto test_y_bidirectional_equivalence() -> void {
    auto [graph, net] = matching_fixture(
        UnifiedNodeKind::Bump,
        UnifiedNodeKind::HLine,
        UnifiedNodeKind::HLine,
        PhysicalSwitchKind::BumpH);
    auto y_to_x_session = CadicalSession {};
    const auto y_to_x_model = build_v14_model(y_to_x_session, graph, {net});
    y_to_x_session.add_clause({y_to_x_model.switch_var_by_id.at(10)});
    const int a0 = a_lit_at(y_to_x_model, 0, 2);
    const int a1 = a_lit_at(y_to_x_model, 1, 2);
    if (a0 > 0) {
        y_to_x_session.add_clause({-a0});
    }
    if (a1 > 0) {
        y_to_x_session.add_clause({-a1});
    }
    require(
        !y_to_x_session.solve_once().ok,
        "Y=true must require at least one direction A use across all pairs");

    auto x_to_y_session = CadicalSession {};
    const auto x_to_y_model = build_v14_model(x_to_y_session, graph, {net});
    x_to_y_session.add_clause({-x_to_y_model.switch_var_by_id.at(10)});
    const int forward_a = a_lit_at(x_to_y_model, 0, 2);
    if (forward_a > 0) {
        x_to_y_session.add_clause({forward_a});
    }
    require(
        !x_to_y_session.solve_once().ok,
        "any directed A use must imply its aggregate physical-switch Y");
}

auto test_forward_and_reverse_switch_use_extract_same_y() -> void {
    auto graph = synthetic_graph(2, {{0, 1}, {1, 0}});
    graph.nodes[1].kind = UnifiedNodeKind::HLine;
    for (auto& arc : graph.arcs) {
        arc.physical_switch_id = 42;
        arc.physical_switch_kind = PhysicalSwitchKind::BumpH;
    }

    const auto check_direction = [&](std::size_t source, std::size_t sink, int expected_arc) {
        const auto nets = std::Vector<RoutingNet> {
            synthetic_net(0, {source}, {{sink, {0}}})};
        auto session = CadicalSession {};
        const auto model = build_v14_model(session, graph, nets);
        const auto solved = session.solve_once();
        require(solved.ok, "single physical-switch direction must be SAT");
        const int a_lit = a_lit_at(model, static_cast<int>(expected_arc), 1);
        require(a_lit > 0 && session.value(a_lit), "the expected directed TOB arc must be selected");
        require(
            session.value(model.switch_var_by_id.at(42)),
            "either physical arc direction must set the same aggregate Y");
        const auto result = extract_sat_solution(graph, nets, model, session, solved);
        require(
            result.ok && result.used_tob_switch_ids == std::Vector<int>({42}),
            "numeric extraction must return exactly the used physical "
                "switch ID");
    };

    check_direction(0, 1, 0);
    check_direction(1, 0, 1);
}
