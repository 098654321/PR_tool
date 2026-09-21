// Numeric constraint kits, distance-domain state, assumptions and UNSAT-core
// feedback, reachable-layer pruning, and COBUnit eligibility tests.

auto test_numeric_constraint_kits() -> void {
    auto session = CadicalSession {};
    const int a = session.new_var();
    const int b = session.new_var();
    const int out = session.new_var();
    add_exactly_one(session, {a, b});
    add_or_equiv(session, out, {a, b});
    add_equiv(session, out, a);
    const auto result = session.solve_once();
    require(result.ok && session.value(a) && !session.value(b), "numeric kits must encode EO/equiv/OR");
}

auto test_binary_successor_truth_table() -> void {
    for (std::size_t width = 1; width <= 4; ++width) {
        const std::size_t value_count = std::size_t {1} << width;
        for (std::size_t input_value = 0; input_value < value_count; ++input_value) {
            for (std::size_t output_value = 0; output_value < value_count; ++output_value) {
                for (const bool enabled : {false, true}) {
                    auto session = CadicalSession {};
                    auto input_bits = std::Vector<int> {};
                    auto output_bits = std::Vector<int> {};
                    for (std::size_t bit = 0; bit < width; ++bit) {
                        input_bits.push_back(session.new_var());
                        output_bits.push_back(session.new_var());
                    }
                    const int condition = session.new_var();
                    const auto successor = add_binary_successor(session, input_bits);
                    add_conditional_successor(
                        session, condition, successor, output_bits);
                    session.add_clause({enabled ? condition : -condition});
                    for (std::size_t bit = 0; bit < width; ++bit) {
                        session.add_clause({
                            (input_value & (std::size_t {1} << bit)) != 0
                                ? input_bits[bit]
                                : -input_bits[bit]});
                        session.add_clause({
                            (output_value & (std::size_t {1} << bit)) != 0
                                ? output_bits[bit]
                                : -output_bits[bit]});
                    }

                    const bool expected = !enabled
                        || (input_value + 1 < value_count
                            && output_value == input_value + 1);
                    const auto solved = session.solve_once();
                    require(
                        solved.ok == expected,
                        "shared successor must match conditional "
                            "non-overflowing increment");
                    if (solved.ok) {
                        const auto modular_successor = (input_value + 1) % value_count;
                        for (std::size_t bit = 0; bit < width; ++bit) {
                            require(
                                session.value(successor.bits[bit])
                                    == ((modular_successor
                                         & (std::size_t {1} << bit))
                                        != 0),
                                "unconditional successor bits must equal "
                                    "input+1 modulo 2^w");
                        }
                        require(
                            session.value(successor.overflow)
                                == (input_value + 1 == value_count),
                            "successor overflow literal must identify the "
                                "maximum input");
                    }
                }
            }
        }
    }
}

auto test_delay_precompute_simple_path() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto net = synthetic_net(0, {0}, {{2, {0}}});
    const auto scopes = build_all_scopes(graph, {net});
    const auto delays = compute_pair_delays(graph, {net}, scopes);
    require(delays.pairs.size() == 1, "simple path must produce one pair delay");
    require(
        delays.pairs[0].target_delay == 2,
        "chain 0->1->2 must have shortest delay 2");
    require(
        delays.pairs[0].delays == std::Vector<int>({2}),
        "simple path delays must be singleton d_min");
}

auto test_pair_state_initial_delays_bbox() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {2, 3}});
    auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto state = init_routing_problem_state(nets);
    require(state.pairs.size() == 1, "single demand must init one pair state");
    require(state.pairs[0].delays.empty(), "pair delays start empty before precompute");
    apply_state_to_nets(state, nets);
    require(nets[0].has_scope_bbox, "apply_state_to_nets must publish net scope_bbox");
    const auto scopes = build_all_scopes(graph, nets);
    (void)compute_pair_delays(graph, nets, scopes, &state);
    require(
        state.pairs[0].delays == std::Vector<int>({3}),
        "chain 0->3 must initialize pair delays to shortest delay 3");

    auto fanout = synthetic_net(1, {0}, {{2, {0}}, {3, {0}}});
    auto fanout_state = init_routing_problem_state({fanout});
    require(fanout_state.pairs.size() == 2, "fanout net must init one pair per demand");

    auto bus = synthetic_net(2, {0, 4}, {{2, {0}}, {6, {1}}});
    bus.is_sync_bus = true;
    const auto bus_graph = synthetic_graph(
        7, {{0, 1}, {1, 2}, {4, 5}, {5, 6}});
    auto bus_nets = std::Vector<RoutingNet> {bus};
    auto bus_state = init_routing_problem_state(bus_nets);
    apply_state_to_nets(bus_state, bus_nets);
    const auto bus_scopes = build_all_scopes(bus_graph, bus_nets);
    (void)compute_pair_delays(bus_graph, bus_nets, bus_scopes, &bus_state);
    require(
        bus_state.pairs[0].delays == bus_state.pairs[1].delays,
        "bus members must share aligned delays after precompute");
    require(
        bus_state.pairs[0].delays == std::Vector<int>({2}),
        "bus aligned delays must equal bus_d_min");
}

auto test_delay_set_drives_d_max() -> void {
    const auto graph = synthetic_graph(5, {{0, 1}, {1, 2}, {2, 3}, {0, 4}, {4, 3}});
    auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {2, 3};
    apply_state_to_nets(state, nets);
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes, &state);
    require(delays.sources[0].d_max == 3, "d_max must follow max(pair.delays)");
    require(
        delays.pairs[0].delays == std::Vector<int>({2, 3}),
        "expanded delay set must be preserved in pair delay info");

    auto session = CadicalSession {};
    const auto model = build_unified_sat_model(session, graph, nets, scopes, delays);
    require(d_lit_at(model, 0, 3, 2) > 0, "delay 2 must allocate a sink D variable");
    require(d_lit_at(model, 0, 3, 3) > 0, "delay 3 must allocate a sink D variable");
    require(model.alpha_vars.size() == 1, "one pair must allocate one alpha variable");
}

auto test_cadical_assume_failed() -> void {
    auto session = CadicalSession {};
    const int x = session.new_var();
    session.add_clause({-x});
    session.assume(x);
    const auto result = session.solve();
    require(!result.ok && result.message == "UNSAT", "assumed literal conflicting with unit clause must be UNSAT");
    require(
        result.failed_assumption_literals.size() == 1
            && result.failed_assumption_literals.front() == x,
        "UNSAT result must list the failed assumption literal");
    require(session.failed(x), "failed() must report the assumed literal in the unsat core");
}

auto test_z3_optimize_cli_option() -> void {
    const auto enabled = parse_test_ilp_cli({"case", "--z3-optimize"});
    require(enabled.enable_z3_optimize, "--z3-optimize must enable the Z3 backend");

    const auto v17 = parse_test_ilp_cli({"case", "--global-route-v17"});
    require(
        v17.enable_global_route_v17 && v17.enable_z3_optimize,
        "--global-route-v17 must enable global routing and the existing Z3 "
            "backend");

    const auto v18 = parse_test_ilp_cli({"case", "--global-route-v18"});
    require(
        v18.enable_global_route_v18 && !v18.enable_z3_optimize,
        "--global-route-v18 must select guided CaDiCaL without Z3 Optimize");
    require(!v18.enable_post_sat_ilp && !v18.enable_post_sat_maze,
            "V18 post-SAT optimizers must be disabled by default");

    const auto ilp = parse_test_ilp_cli(
        {"case", "--global-route-v18", "--ilp-optimize"});
    require(ilp.enable_post_sat_ilp && !ilp.enable_post_sat_maze,
            "--ilp-optimize must explicitly enable only post-SAT ILP");
    const auto maze = parse_test_ilp_cli(
        {"case", "--global-route-v18", "--maze-optimize"});
    require(maze.enable_post_sat_maze && !maze.enable_post_sat_ilp,
            "--maze-optimize must explicitly enable only post-SAT RRR");

    bool rejected_padding = false;
    try {
        (void)parse_test_ilp_cli({"case", "--global-route-v17", "-s", "1"});
    }
    catch (const std::invalid_argument&) {
        rejected_padding = true;
    }
    require(rejected_padding, "V17 must reject legacy scope/delay padding");

    bool rejected_v18_z3 = false;
    try {
        (void)parse_test_ilp_cli({"case", "--global-route-v18", "--z3-optimize"});
    }
    catch (const std::invalid_argument&) {
        rejected_v18_z3 = true;
    }
    require(rejected_v18_z3, "V18 pure SAT must reject Z3 Optimize");

    bool rejected_post_without_v18 = false;
    try {
        (void)parse_test_ilp_cli({"case", "--maze-optimize"});
    }
    catch (const std::invalid_argument&) {
        rejected_post_without_v18 = true;
    }
    require(rejected_post_without_v18,
            "post-SAT optimization must require the V18 guided flow");

    bool rejected_two_post_optimizers = false;
    try {
        (void)parse_test_ilp_cli(
            {"case", "--global-route-v18", "--ilp-optimize",
             "--maze-optimize"});
    }
    catch (const std::invalid_argument&) {
        rejected_two_post_optimizers = true;
    }
    require(rejected_two_post_optimizers,
            "post-SAT ILP and maze optimization must be mutually exclusive");
}

#ifdef USE_Z3
auto test_z3_optimize_minimizes_unit_soft_clauses() -> void {
    auto request = Z3OptimizeRequest {};
    request.num_vars = 3;
    request.hard_clauses = {{-1, 2, 3}};
    request.soft_negated_vars = {2, 3};
    request.external_assumptions = {1};

    const auto result = solve_z3_optimize(request);
    require(result.status == Z3OptimizeStatus::Optimal, "Z3 must prove the unit-soft optimum");
    require(result.objective_cost == 1, "exactly one occupancy variable must be required");
    require(
        result.value(2) != result.value(3),
        "the optimum must activate exactly one occupancy variable");
}

auto test_z3_optimize_returns_external_assumption_core() -> void {
    auto request = Z3OptimizeRequest {};
    request.num_vars = 1;
    request.hard_clauses = {{-1}};
    request.external_assumptions = {1};

    const auto result = solve_z3_optimize(request);
    require(
        result.status == Z3OptimizeStatus::HardUnsat,
        "a hard conflict under alpha must be reported as hard UNSAT");
    require(
        result.failed_assumption_literals == std::Vector<int> {1},
        "Z3 must return the conflicting external assumption, not a soft core");
}

auto test_z3_optimize_hard_unsat_has_no_alpha_core() -> void {
    auto request = Z3OptimizeRequest {};
    request.num_vars = 1;
    request.hard_clauses = {{1}, {-1}};
    request.external_assumptions = {1};

    const auto result = solve_z3_optimize(request);
    require(
        result.status == Z3OptimizeStatus::HardUnsat,
        "a contradictory hard formula must be reported as hard UNSAT");
    require(
        result.failed_assumption_literals.empty(),
        "a hard-only contradiction must not manufacture an alpha core");
}

auto test_z3_node_occupancy_matches_required_route_union() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes);
    auto session_options = CadicalDiagnosticsOptions {};
    session_options.capture_clauses = true;
    auto session = CadicalSession {session_options};
    const auto model = build_unified_sat_model(session, graph, nets, scopes, delays);
    const auto occupancy = add_node_occupancy_variables(session, graph, model);

    auto request = Z3OptimizeRequest {};
    request.num_vars = session.num_vars();
    request.hard_clauses = session.clauses();
    for (const auto& [_, variable] : occupancy.u_var_by_node) {
        request.soft_negated_vars.push_back(variable);
    }
    for (const auto& alpha : model.alpha_vars) {
        request.external_assumptions.push_back(alpha.alpha_lit);
    }

    const auto result = solve_z3_optimize(request);
    require(result.status == Z3OptimizeStatus::Optimal, "a required path must have a Z3 optimum");
    require(result.objective_cost == 3, "node occupancy must count the three-node route union");
}
#endif

auto test_alpha_implies_sink_d() -> void {
    const auto graph = synthetic_graph(5, {{0, 1}, {1, 2}, {2, 3}, {0, 4}, {4, 3}});
    auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {2, 3};
    apply_state_to_nets(state, nets);
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes, &state);
    auto session = CadicalSession {};
    SatEncodingStats stats {};
    const auto model = build_unified_sat_model(session, graph, nets, scopes, delays, &stats);
    require(model.alpha_vars.size() == 1, "single pair must allocate one alpha variable");
    require(stats.alpha_vars == 1, "encoding stats must count the alpha variable");
    const int alpha = model.alpha_vars.front().alpha_lit;
    const int sink_d2 = d_lit_at(model, 0, 3, 2);
    const int sink_d3 = d_lit_at(model, 0, 3, 3);
    require(sink_d2 > 0 && sink_d3 > 0, "fixture must allocate sink D at both target delays");
    session.assume(alpha);
    const auto result = session.solve();
    require(result.ok, "assuming alpha on a satisfiable pair must remain SAT");
    require(
        session.value(sink_d2) || session.value(sink_d3),
        "alpha assumption must imply at least one sink D literal is true");
}

auto test_alpha_gates_sink_connectivity() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    {
        auto session = CadicalSession {};
        const auto model = build_v14_model(session, graph, nets, nullptr, false);
        const int sink_d2 = d_lit_at(model, 0, 2, 2);
        require(sink_d2 > 0, "fixture must allocate sink D at target delay");
        session.assume(-sink_d2);
        require(
            session.solve().ok,
            "sink connectivity must be disabled when alpha is not assumed");
    }

    {
        auto session = CadicalSession {};
        const auto model = build_v14_model(session, graph, nets, nullptr, false);
        const int alpha = model.alpha_vars.front().alpha_lit;
        const int sink_d2 = d_lit_at(model, 0, 2, 2);
        session.assume(alpha);
        session.assume(-sink_d2);
        const auto with_alpha = session.solve();
        require(
            !with_alpha.ok && with_alpha.message == "UNSAT",
            "alpha and a disabled sink D must be UNSAT");
        require(
            session.failed(alpha),
            "alpha must be part of the failed assumption core");
    }
}

auto test_alpha_skips_unreachable_delay_for_sat() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {2, 3};
    apply_state_to_nets(state, nets);
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes, &state);
    auto session = CadicalSession {};
    const auto model = build_unified_sat_model(session, graph, nets, scopes, delays);
    require(d_lit_at(model, 0, 2, 2) > 0, "reachable delay must allocate sink D");
    require(d_lit_at(model, 0, 2, 3) <= 0, "unreachable requested delay must not allocate sink D");
    session.assume(model.alpha_vars.front().alpha_lit);
    require(
        session.solve().ok,
        "SAT must choose the reachable delay when "
                                "another allowed delay is unreachable");
}

auto test_alpha_empty_exact_delay_reports_core() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {3};
    apply_state_to_nets(state, nets);
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes, &state);
    auto session = CadicalSession {};
    const auto model = build_unified_sat_model(session, graph, nets, scopes, delays);
    require(
        d_lit_at(model, 0, 2, 3) <= 0,
        "an unreachable exact delay must not allocate a sink D variable");
    const int alpha = model.alpha_vars.front().alpha_lit;
    session.assume(alpha);
    const auto result = session.solve();
    require(
        !result.ok && result.message == "UNSAT",
        "SAT connectivity must reject an unreachable exact delay");
    require(
        session.failed(alpha),
        "the unreachable exact delay must report alpha in the failed core");
}

auto test_feedback_expands_delays_on_unsat() -> void {
    PairRoutingState pair {};
    pair.key = PairKey {0, 0, 0};
    pair.delays = {5};
    apply_feedback_step_to_pair(pair, 1);
    require(
        pair.delays == std::Vector<int>({5, 6}),
        "odd feedback failure must append only max+1 to delays");

    const auto before = pair.pair_bbox;
    apply_feedback_step_to_pair(pair, 2);
    require(
        pair.delays == std::Vector<int>({5, 6, 7}),
        "even feedback failure must append another max+1 to delays");
    require(
        pair.pair_bbox.row_min <= before.row_min
            && pair.pair_bbox.row_max >= before.row_max,
        "even feedback failure must grow pair bbox by one cell per side");
}

auto test_bus_member_delay_bbox_sync() -> void {
    auto net = synthetic_net(0, {0, 4}, {{2, {0}}, {6, {1}}});
    net.is_sync_bus = true;
    const auto nets = std::Vector<RoutingNet> {net};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {5, 6};
    state.pairs[0].pair_bbox = IlpBoundingBox {1, 2, 1, 2};
    state.pairs[1].delays = {3};
    state.pairs[1].pair_bbox = IlpBoundingBox {4, 5, 4, 5};
    sync_bus_after_expand(state, nets, 0);
    require(
        state.pairs[0].delays == state.pairs[1].delays,
        "bus sync must merge member delay sets");
    require_bbox(
        state.pairs[0].pair_bbox,
        state.pairs[1].pair_bbox.row_min,
        state.pairs[1].pair_bbox.row_max,
        state.pairs[1].pair_bbox.col_min,
        state.pairs[1].pair_bbox.col_max,
        "bus sync must merge member bboxes to a shared hull");
    require(
        state.pairs[0].delays == std::Vector<int>({3, 5, 6}),
        "bus merged delays must be the union of member delays");
}

auto test_feedback_rebuilds_after_reaching_full_bbox() -> void {
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}})};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {2};
    const auto chip = full_chip_bbox();
    state.pairs[0].pair_bbox =
        IlpBoundingBox {chip.row_min + 1, chip.row_max, chip.col_min, chip.col_max};

    const auto status_first =
        apply_feedback_expansion(state, nets, {state.pairs[0].key});
    require(
        status_first == FeedbackExpansionStatus::Expanded,
        "first odd feedback failure must still expand delays");
    require(
        !is_full_chip_bbox(state.pairs[0].pair_bbox),
        "first odd feedback failure must not expand scope yet");
    require(
        state.pairs[0].delays == std::Vector<int>({2, 3}),
        "first odd feedback failure must append only max+1");

    const auto status_second =
        apply_feedback_expansion(state, nets, {state.pairs[0].key});
    require(
        status_second == FeedbackExpansionStatus::Expanded,
        "reaching full-chip bbox must rebuild and solve once before "
            "exhaustion");
    require(
        is_full_chip_bbox(state.pairs[0].pair_bbox),
        "second even feedback failure must expand critical bbox to full chip");
    require(
        state.pairs[0].delays == std::Vector<int>({2, 3, 4}),
        "second even feedback failure must append another max+1");
}

auto test_feedback_exhausted_state_is_unchanged() -> void {
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}})};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {2};
    state.pairs[0].pair_bbox = full_chip_bbox();

    const auto status =
        apply_feedback_expansion(state, nets, {state.pairs[0].key});
    require(
        status == FeedbackExpansionStatus::Exhausted,
        "an UNSAT model already solved at full-chip bbox must be exhausted");
    require(
        state.pairs[0].delays == std::Vector<int>({2}),
        "exhausted feedback must not mutate delays");
}

auto test_feedback_expands_only_critical_fanout_pair_delay() -> void {
    const auto fanout_net = synthetic_net(0, {0}, {{2, {0}}, {3, {0}}});
    const auto nets = std::Vector<RoutingNet> {fanout_net};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {4};
    state.pairs[1].delays = {6};

    const auto status =
        apply_feedback_expansion(state, nets, {state.pairs[0].key});

    require(
        status == FeedbackExpansionStatus::Expanded,
        "a critical fanout pair must expand");
    require(
        state.pairs[0].delays == std::Vector<int>({4, 5}),
        "the critical fanout pair must append its own max+1");
    require(
        state.pairs[1].delays == std::Vector<int>({6}),
        "a non-critical fanout pair must keep its own delay set");
}

auto test_feedback_global_expand_skips_full_net_and_keeps_fanout_delays_independent() -> void {
    auto full_net = synthetic_net(0, {0}, {{2, {0}}});
    auto fanout_net = synthetic_net(1, {3}, {{4, {0}}, {5, {0}}});
    const auto nets = std::Vector<RoutingNet> {full_net, fanout_net};
    auto state = init_routing_problem_state(nets);
    state.pairs[0].delays = {2};
    state.pairs[0].pair_bbox = full_chip_bbox();
    state.pairs[1].delays = {4};
    state.pairs[1].pair_bbox = IlpBoundingBox {2, 3, 2, 3};
    state.pairs[2].delays = {6};
    state.pairs[2].pair_bbox = IlpBoundingBox {4, 5, 4, 5};

    const auto status =
        apply_feedback_expansion(state, nets, {state.pairs[0].key});
    require(
        status == FeedbackExpansionStatus::Expanded,
        "a full critical net must trigger expansion of the other nets");
    require(
        state.pairs[0].delays == std::Vector<int>({2}),
        "the already-full critical net must not expand twice");
    require(
        state.pairs[1].delays == std::Vector<int>({4, 5}),
        "global expansion must append the first fanout pair's own max+1");
    require(
        state.pairs[2].delays == std::Vector<int>({6, 7}),
        "global expansion must append the second fanout pair's own max+1");
}

auto test_bus_delay_takes_max_member() -> void {
    const auto graph = synthetic_graph(
        7, {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {5, 6}});
    auto net = synthetic_net(0, {0, 3}, {{2, {0}}, {6, {1}}});
    net.is_sync_bus = true;
    const auto scopes = build_all_scopes(graph, {net});
    const auto delays = compute_pair_delays(graph, {net}, scopes);
    require(delays.pairs.size() == 2, "bus net must produce two pair delays");
    require(
        delays.pairs[0].target_delay == 3 && delays.pairs[1].target_delay == 3,
        "bus delay must equal max(member shortest delays)");
    require(
        delays.pairs[0].delays == std::Vector<int>({3})
            && delays.pairs[1].delays == std::Vector<int>({3}),
        "bus pair delays must be singleton bus_d_min");
    require(
        delays.pairs[0].member_shortest_delay == 2
            && delays.pairs[1].member_shortest_delay == 3,
        "bus members must retain their pre-alignment shortest delays");
}

auto test_v14_d_var_exact_reachability_allocation() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {2, 3}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{3, {0}}})};
    auto session = CadicalSession {};
    SatEncodingStats stats {};
    const auto model = build_v14_model(session, graph, nets, &stats);
    require(model.sources.size() == 1, "single source net must allocate one source block");
    const auto& source = model.sources.front();
    require(source.d_max == 3, "chain length 4 must use delay 3 at sink");
    std::size_t allocated = 0;
    std::size_t omitted = 0;
    for (const auto& row : source.d_var) {
        for (int lit : row) {
            if (lit > 0) {
                ++allocated;
            }
            else {
                ++omitted;
            }
        }
    }
    require(
        allocated > 0 && omitted > 0,
        "exact reachability must omit structurally irrelevant D slots");
    require(stats.d_vars == allocated, "D statistics must count active variables");
    require(
        stats.dense_d_slots == allocated + omitted,
        "D statistics must retain the pre-pruning dense slot count");
}

auto test_v14_unreachable_tob_arc_has_no_a_var() -> void {
    auto graph = synthetic_graph(4, {{0, 1}, {2, 3}});
    graph.arcs[0].physical_switch_kind = PhysicalSwitchKind::BumpH;
    graph.arcs[0].physical_switch_id = 7;
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {2}, {{3, {0}}})};
    auto session = CadicalSession {};
    SatEncodingStats stats {};
    const auto model = build_v14_model(session, graph, nets, &stats);

    require(
        a_lit_at(model, 0, 1) == 0,
        "a TOB arc outside every source-to-sink walk must not allocate A");
    require(stats.a_vars == 0, "irrelevant TOB arcs must not contribute A variables");
    require(stats.dense_a_slots == 1, "A statistics must retain the dense slot count");
}

auto test_v14_irrelevant_base_states_are_not_allocated() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};

    {
        auto session = CadicalSession {};
        const auto model = build_v14_model(session, graph, nets, nullptr, false);
        const int source_d1 = d_lit_at(model, 0, 0, 1);
        require(
            source_d1 <= 0,
            "source D at d>0 must be omitted from the exact useful domain");
    }

    {
        auto session = CadicalSession {};
        const auto model = build_v14_model(session, graph, nets, nullptr, false);
        const int non_source_d0 = d_lit_at(model, 0, 1, 0);
        require(
            non_source_d0 <= 0,
            "non-source D at delay zero must be omitted");
    }
}

auto test_v14_d_without_predecessor_is_not_allocated() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets, nullptr, false);
    const int isolated_d1 = d_lit_at(model, 0, 3, 1);
    require(
        isolated_d1 <= 0,
        "a D state without any connectivity predecessor must be omitted");
}

auto test_v14_reverse_reachability_prunes_dead_branch() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {0, 3}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets, nullptr, false);
    require(d_lit_at(model, 0, 1, 1) > 0, "source-to-sink branch must retain its D state");
    require(
        d_lit_at(model, 0, 3, 1) <= 0,
        "forward-reachable branch that cannot reach the sink must be omitted");
}

auto test_v14_feedback_delay_rebuilds_active_mask() -> void {
    const auto graph = synthetic_graph(
        5,
        {{0, 1}, {1, 4}, {0, 2}, {2, 3}, {3, 4}});
    auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{4, {0}}})};
    auto state = init_routing_problem_state(nets);
    apply_state_to_nets(state, nets);

    {
        const auto scopes = build_all_scopes(graph, nets);
        const auto delays = compute_pair_delays(graph, nets, scopes, &state);
        auto session = CadicalSession {};
        const auto model =
            build_unified_sat_model(session, graph, nets, scopes, delays);
        require(
            d_lit_at(model, 0, 2, 1) <= 0,
            "a longer branch must be absent before its delay is allowed");
    }

    state.pairs[0].delays = {2, 3};
    apply_state_to_nets(state, nets);
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes, &state);
    auto session = CadicalSession {};
    const auto model =
        build_unified_sat_model(session, graph, nets, scopes, delays);
    require(
        d_lit_at(model, 0, 2, 1) > 0 && d_lit_at(model, 0, 4, 3) > 0,
        "feedback delay expansion must rebuild and expose newly valid states");
}

auto test_v14_fanout_active_mask_unions_sinks() -> void {
    const auto graph =
        synthetic_graph(5, {{0, 1}, {1, 2}, {0, 3}, {3, 4}});
    const auto net = synthetic_net(0, {0}, {{2, {0}}, {4, {0}}});
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, {net}, nullptr, false);
    require(
        d_lit_at(model, 0, 1, 1) > 0
            && d_lit_at(model, 0, 3, 1) > 0,
        "fanout source domain must union states useful to different sinks");
}

auto test_v14_source_unit_masks() -> void {
    {
        auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
        graph.nodes[0].kind = UnifiedNodeKind::Track;
        graph.nodes[0].unit = 3;
        const auto net = synthetic_net(0, {0}, {{2, {0}}});
        const auto scopes = build_all_scopes(graph, {net});
        const auto delays = compute_pair_delays(graph, {net}, scopes);
        require(
            delays.sources[0].source_unit_mask == (std::uint16_t {1} << 3),
            "track source must expose exactly its physical COBUnit");
    }

    {
        auto graph = synthetic_graph(4, {{0, 2}, {1, 2}, {2, 3}});
        graph.nodes[0].kind = UnifiedNodeKind::Track;
        graph.nodes[0].unit = 2;
        graph.nodes[1].kind = UnifiedNodeKind::Track;
        graph.nodes[1].unit = 10;
        auto pnnet = synthetic_net(1, {0, 1}, {{3, {0, 1}}});
        pnnet.kind = RoutingNetKind::PNnet;
        auto nets = std::Vector<RoutingNet> {pnnet};
        augment_graph_for_pnnet(graph, nets);
        const auto scopes = build_all_scopes(graph, nets);
        const auto delays = compute_pair_delays(graph, nets, scopes);
        require(
            delays.sources[0].source_unit_mask
                == ((std::uint16_t {1} << 2) | (std::uint16_t {1} << 10)),
            "PNnet virtual source must expose the union of candidate track "
                "units");
        auto session = CadicalSession {};
        const auto model =
            build_unified_sat_model(session, graph, nets, scopes, delays);
        require(
            d_lit_at(model, 0, 0, 1) > 0
                && d_lit_at(model, 0, 1, 1) > 0,
            "PNnet active mask must retain every useful candidate source "
                "track");
    }

    {
        const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
        auto net = synthetic_net(2, {0}, {{2, {0}}});
        net.kind = RoutingNetKind::Bnet;
        const auto scopes = build_all_scopes(graph, {net});
        const auto delays = compute_pair_delays(graph, {net}, scopes);
        require(
            delays.sources[0].source_unit_mask == std::uint16_t {0xffff},
            "Bnet bump source must keep all 16 units statically eligible");
    }
}

auto test_cob_unit_mask_helpers() -> void {
    {
        auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
        graph.nodes[0].kind = UnifiedNodeKind::Track;
        graph.nodes[0].unit = 5;
        graph.nodes[1].kind = UnifiedNodeKind::Track;
        graph.nodes[1].unit = 5;
        graph.nodes[2].kind = UnifiedNodeKind::Track;
        graph.nodes[2].unit = 7;
        const auto net = synthetic_net(0, {0}, {{2, {0}}});
        const auto mask = compute_source_unit_mask(graph, net, 0);
        require(mask == (std::uint16_t {1} << 5), "track source mask must be a single unit bit");
        require(
            node_unit_eligible(graph.nodes[1], mask),
            "same-unit track must be eligible");
        require(
            !node_unit_eligible(graph.nodes[2], mask),
            "different-unit track must be ineligible");
        require(
            arc_unit_eligible(graph, graph.arcs[0], mask),
            "same-unit arc must be eligible");
    }

    {
        auto graph = synthetic_graph(4, {{0, 2}, {1, 2}, {2, 3}});
        graph.nodes[0].kind = UnifiedNodeKind::Track;
        graph.nodes[0].unit = 1;
        graph.nodes[1].kind = UnifiedNodeKind::Track;
        graph.nodes[1].unit = 9;
        auto pnnet = synthetic_net(1, {0, 1}, {{3, {0, 1}}});
        pnnet.kind = RoutingNetKind::PNnet;
        auto nets = std::Vector<RoutingNet> {pnnet};
        augment_graph_for_pnnet(graph, nets);
        const auto mask = compute_source_unit_mask(graph, nets[0], nets[0].virtual_source_node);
        require(
            mask == ((std::uint16_t {1} << 1) | (std::uint16_t {1} << 9)),
            "PNnet mask must union candidate track units");
    }

    {
        const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
        auto net = synthetic_net(2, {0}, {{2, {0}}});
        net.kind = RoutingNetKind::Bnet;
        require(
            compute_source_unit_mask(graph, net, 0) == std::uint16_t {0xffff},
            "bump source mask must keep all units");
    }
}

auto test_v14_unit_mask_prunes_track_and_vline_states() -> void {
    {
        auto graph = synthetic_graph(4, {{0, 1}, {1, 3}, {0, 2}, {2, 3}});
        for (const int node : {0, 1, 2}) {
            graph.nodes[static_cast<std::size_t>(node)].kind = UnifiedNodeKind::Track;
        }
        graph.nodes[0].unit = 3;
        graph.nodes[1].unit = 3;
        graph.nodes[2].unit = 4;
        auto session = CadicalSession {};
        SatEncodingStats stats {};
        const auto model = build_v14_model(
            session,
            graph,
            {synthetic_net(0, {0}, {{3, {0}}})},
            &stats,
            false);
        require(d_lit_at(model, 0, 1, 1) > 0, "same-unit track state must remain active");
        require(d_lit_at(model, 0, 2, 1) <= 0, "different-unit track state must be omitted");
        require(
            stats.unit_eligible_d_slots < stats.dense_d_slots,
            "unit eligibility statistics must count pruned track slots");
    }

    {
        auto graph = synthetic_graph(4, {{0, 1}, {1, 3}, {0, 2}, {2, 3}});
        graph.nodes[0].kind = UnifiedNodeKind::Track;
        graph.nodes[0].unit = 3;
        graph.nodes[1].kind = UnifiedNodeKind::VLine;
        graph.nodes[1].line_index = 3;
        graph.nodes[2].kind = UnifiedNodeKind::VLine;
        graph.nodes[2].line_index = 4;
        for (const int arc_id : {0, 2}) {
            graph.arcs[static_cast<std::size_t>(arc_id)].physical_switch_kind =
                PhysicalSwitchKind::VLineTrack;
            graph.arcs[static_cast<std::size_t>(arc_id)].physical_switch_id = 100 + arc_id;
            graph.arcs[static_cast<std::size_t>(arc_id)].mode_group_id = arc_id / 2;
            graph.arcs[static_cast<std::size_t>(arc_id)].is_vline_track_straight = true;
        }
        auto session = CadicalSession {};
        const auto model = build_v14_model(
            session,
            graph,
            {synthetic_net(0, {0}, {{3, {0}}})},
            nullptr,
            false);
        require(d_lit_at(model, 0, 1, 1) > 0, "matching local-k VLine must remain active");
        require(d_lit_at(model, 0, 2, 1) <= 0, "mismatched local-k VLine must be omitted");
        require(a_lit_at(model, 0, 1) > 0, "eligible VLine-track A must remain allocated");
        require(a_lit_at(model, 2, 1) == 0, "ineligible VLine-track A must be omitted");
    }
}
