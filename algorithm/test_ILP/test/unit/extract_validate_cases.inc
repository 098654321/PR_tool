// SAT solution extraction, path formatting/logging, independent physical
// validation, ideal-wirelength helpers, PN virtual-source, and golden cases.

auto test_extraction_selects_one_valid_predecessor() -> void {
    const auto graph = synthetic_graph(4, {{0, 1}, {0, 2}, {1, 3}, {2, 3}});
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{3, {0}}})};
    auto session = CadicalSession {};
    auto model = UnifiedSatModel {};
    auto scope = UnifiedSatNetScope {};
    scope.net_id = 0;
    scope.node_ids = {0, 1, 2, 3};
    scope.arc_ids = {0, 1, 2, 3};
    scope.node_offset = {0, 1, 2, 3};
    scope.arc_offset = {0, 1, 2, 3};
    model.scopes.push_back(scope);
    model.pair_delays.push_back(PairDelayInfo {
        0, 0, 0, 0, 3, std::Vector<int>({2}), 2, -1});

    auto source = SourceDelayVars {};
    source.net_id = 0;
    source.source_index = 0;
    source.source_node = 0;
    source.scope_index = 0;
    source.model_source_index = 0;
    source.d_max = 2;
    source.d_var = {
        {session.new_var(), -1, -1},
        {-1, session.new_var(), -1},
        {-1, session.new_var(), -1},
        {-1, -1, session.new_var()}};
    model.sources.push_back(source);

    session.add_clause({source.d_var[0][0]});
    session.add_clause({source.d_var[1][1]});
    session.add_clause({source.d_var[2][1]});
    session.add_clause({source.d_var[3][2]});
    const auto solved = session.solve_once();
    require(solved.ok, "manual ambiguous next-arc assignment must be SAT");
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(
        result.ok && result.paths.size() == 1,
        "extraction must select one valid predecessor when the D tree "
            "branches");
    const auto& path = result.paths[0].node_path;
    require(
        path == std::Vector<int>({0, 1, 3})
            || path == std::Vector<int>({0, 2, 3}),
        "extraction must return one valid source-to-sink path");
}

auto test_extraction_handles_shared_source_fanout() -> void {
    const auto graph = synthetic_graph(
        5, {{0, 1}, {0, 2}, {1, 3}, {2, 4}});
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{3, {0}}, {4, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "shared-source fanout fixture must be SAT");

    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok && result.paths.size() == 2, "fanout extraction must return both demands");
    require(
        result.paths[0].node_path == std::Vector<int>({0, 1, 3}),
        "first fanout demand must follow its source-to-sink branch");
    require(
        result.paths[1].node_path == std::Vector<int>({0, 2, 4}),
        "second fanout demand must follow its source-to-sink branch");
}

auto test_extraction_error_branches() -> void {
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}})};

    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    auto missing_pair_session = CadicalSession {};
    const auto missing_pair_solved = missing_pair_session.solve_once();
    const auto missing_pair = extract_sat_solution(
        graph, nets, UnifiedSatModel {}, missing_pair_session, missing_pair_solved);
    require(
        !missing_pair.ok && std::string {missing_pair.message}.find("no delay pair") != std::string::npos,
        "extraction must reject a demand with no delay pair");

    auto inactive_sink_session = CadicalSession {};
    auto inactive_model = UnifiedSatModel {};
    auto inactive_scope = UnifiedSatNetScope {};
    inactive_scope.node_ids = {0, 1, 2};
    inactive_scope.node_offset = {0, 1, 2};
    inactive_model.scopes.push_back(inactive_scope);
    inactive_model.pair_delays.push_back(PairDelayInfo {
        0, 0, 0, 0, 2, std::Vector<int>({2}), 2, -1});
    auto inactive_source = SourceDelayVars {};
    inactive_source.scope_index = 0;
    inactive_source.source_node = 0;
    inactive_source.d_max = 2;
    inactive_source.d_var = {
        {inactive_sink_session.new_var(), -1, -1},
        {-1, -1, -1},
        {-1, -1, inactive_sink_session.new_var()}};
    inactive_model.sources.push_back(inactive_source);
    inactive_sink_session.add_clause({inactive_source.d_var[0][0]});
    const auto inactive_solved = inactive_sink_session.solve_once();
    const auto inactive = extract_sat_solution(
        graph, nets, inactive_model, inactive_sink_session, inactive_solved);
    require(
        !inactive.ok
            && (std::string {inactive.message}.find("not active at any target delay") != std::string::npos
                || std::string {inactive.message}.find("no next arc") != std::string::npos),
        "extraction must reject an inactive sink delay");

    const auto disconnected = synthetic_graph(3, {});
    try {
        const auto scopes = build_all_scopes(disconnected, nets);
        (void)compute_pair_delays(disconnected, nets, scopes);
        require(false, "disconnected graph must fail before extraction");
    }
    catch (const std::runtime_error&) {
    }
}

auto test_format_path_node_track_bump_hline_vline() -> void {
    auto graph = UnifiedGraph {};
    graph.nodes.resize(4);

    graph.nodes[0].kind = UnifiedNodeKind::Track;
    graph.nodes[0].track_row = 4;
    graph.nodes[0].track_col = 6;
    graph.nodes[0].track_dir = 1;
    graph.nodes[0].track_index = 12;

    graph.nodes[1].kind = UnifiedNodeKind::Bump;
    graph.nodes[1].bump = Bump_coord {0, 1, 2, 3};

    graph.nodes[2].kind = UnifiedNodeKind::HLine;
    graph.nodes[2].tob = 0;
    graph.nodes[2].bank = 1;
    graph.nodes[2].group = 2;
    graph.nodes[2].line_index = 4;

    graph.nodes[3].kind = UnifiedNodeKind::VLine;
    graph.nodes[3].tob = 0;
    graph.nodes[3].bank = 1;
    graph.nodes[3].line_index = 17;

    require(
        format_path_node(graph, 0) == "{r4, c6, V, i12}",
        "track path node must use compact TrackCoord text");
    require(
        format_path_node(graph, 1) == "TOB(0,0) B1 G2 I3",
        "bump path node must use TOB/bank/group/index");
    require(
        format_path_node(graph, 2) == "TOB(0,0) B1 G2 J4",
        "hline path node must use J index");
    require(
        format_path_node(graph, 3) == "TOB(0,0) B1 V17",
        "vline path node must use V index");
}

auto test_infer_net_display_kind() -> void {
    auto two_pin = RoutingNet {};
    two_pin.kind = RoutingNetKind::Bnet;
    two_pin.demands.emplace_back(RoutingDemand {0, bump_ref(1), {0}, true});
    require(
        infer_net_display_kind(two_pin) == NetDisplayKind::TwoPin,
        "single-demand Bnet must be TwoPin");

    auto ttb = RoutingNet {};
    ttb.kind = RoutingNetKind::Tnet;
    ttb.demands.push_back(fixed_demand(0, bump_ref(1)));
    ttb.demands.push_back(fixed_demand(1, bump_ref(2)));
    require(
        infer_net_display_kind(ttb) == NetDisplayKind::TrackToBumps,
        "multi-demand Tnet must be TrackToBumps");

    auto tsbs = RoutingNet {};
    tsbs.kind = RoutingNetKind::PNnet;
    tsbs.demands.push_back(RoutingDemand {0, bump_ref(1), {0, 1}, false});
    require(
        infer_net_display_kind(tsbs) == NetDisplayKind::TracksToBumps,
        "PNnet must be TracksToBumps");

    auto sync = RoutingNet {};
    sync.kind = RoutingNetKind::Bnet;
    sync.is_sync_bus = true;
    sync.demands.push_back(fixed_demand(0, bump_ref(1)));
    sync.demands.push_back(fixed_demand(1, bump_ref(2)));
    require(
        infer_net_display_kind(sync) == NetDisplayKind::SyncBus,
        "sync bus must be SyncBus");
}

auto test_format_bbox_corners() -> void {
    const auto text = format_bbox_corners(IlpBoundingBox {1, 3, 2, 5});
    require(
        text == "corners: (1,2) (1,5) (3,2) (3,5) bounds=(1,3,2,5)",
        "bbox corners must list all four rectangle corners");
}

auto test_path_wirelength_counts_bump_and_track_only() -> void {
    auto graph = UnifiedGraph {};
    graph.nodes.resize(5);
    graph.in_arc_ids.resize(5);
    graph.out_arc_ids.resize(5);
    graph.nodes[0].kind = UnifiedNodeKind::Track;
    graph.nodes[1].kind = UnifiedNodeKind::HLine;
    graph.nodes[2].kind = UnifiedNodeKind::Bump;
    graph.nodes[3].kind = UnifiedNodeKind::VLine;
    graph.nodes[4].kind = UnifiedNodeKind::Track;

    require(
        path_wirelength(graph, std::Vector<int> {0, 1, 2, 3, 4}) == 3,
        "wirelength must count only bump and track nodes");

    auto result = SatRoutingResult {};
    result.paths.push_back(SourceSinkPairPath {0, 0, 0, -1, std::Vector<int> {0, 1, 2}});
    result.paths.push_back(SourceSinkPairPath {1, 0, 0, -1, std::Vector<int> {4}});
    require(
        total_wirelength(graph, result) == 3,
        "total wirelength must sum deduplicated bump and track resources "
            "per net");

    result.paths.clear();
    result.paths.push_back(SourceSinkPairPath {0, 0, 0, -1, std::Vector<int> {0, 1, 2}});
    result.paths.push_back(SourceSinkPairPath {0, 0, 1, -1, std::Vector<int> {0, 4}});
    require(
        path_wirelength(graph, result.paths[0].node_path) == 2,
        "per-pair wirelength must still count every hop on one path");
    require(
        path_wirelength(graph, result.paths[1].node_path) == 2,
        "per-pair wirelength must count the second fanout path independently");
    const auto fanout_paths = std::Vector<const SourceSinkPairPath*> {
        &result.paths[0],
        &result.paths[1]};
    require(
        net_wirelength(graph, fanout_paths) == 3,
        "net wirelength must deduplicate shared track nodes across fanout "
            "paths");
    require(
        total_wirelength(graph, result) == 3,
        "total wirelength must use net-level deduplication");
}

auto test_format_path_hops_and_graph_node_ref() -> void {
    auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto hops = format_path_hops(graph, std::Vector<int> {0, 1, 2});
    require(
        hops.find("TOB(0,0) B0 G0 I0") != std::string::npos,
        "path hops must format bump nodes in the chain");

    GraphNodeRef track {};
    track.kind = GraphNodeRef::Kind::Track;
    track.track_coord = hardware::TrackCoord {
        6, 3, hardware::TrackDirection::Vertical, 4};
    require(
        format_graph_node_ref(track) == "{r6, c3, V, i4}",
        "track endpoint ref must use compact TrackCoord text");
}

auto synthetic_pnnet(
    std::size_t net_id,
    const std::Vector<std::size_t>& sources,
    const std::Vector<std::pair<std::size_t, std::Vector<std::size_t>>>& demands
) -> RoutingNet {
    auto net = synthetic_net(net_id, sources, demands);
    net.kind = RoutingNetKind::PNnet;
    return net;
}

auto test_initial_search_padding_pnnet_keeps_sink_delays_independent() -> void {
    const auto graph = synthetic_graph(
        7,
        {{0, 2}, {1, 2}, {2, 3}, {0, 4}, {1, 4}, {4, 5}, {5, 6}});
    auto nets = std::Vector<RoutingNet> {
        synthetic_pnnet(4, {0, 1}, {{3, {0, 1}}, {6, {0, 1}}})};
    auto state = init_routing_problem_state(nets);
    auto working_graph = graph;
    augment_graph_for_pnnet(working_graph, nets);

    apply_initial_search_padding(state, nets, working_graph, 0, 1);

    require(
        state.pairs[0].delays == std::Vector<int>({3, 4}),
        "PNnet demand 0 must pad from its own virtual-source d_min");
    require(
        state.pairs[1].delays == std::Vector<int>({4, 5}),
        "PNnet demand 1 must pad from its own virtual-source d_min");
}

auto test_pnnet_virtual_delay_shift() -> void {
    const auto graph = synthetic_graph(4, {{0, 2}, {1, 2}, {2, 3}});
    auto nets = std::Vector<RoutingNet> {synthetic_pnnet(0, {0, 1}, {{3, {0, 1}}})};
    auto working_graph = graph;
    augment_graph_for_pnnet(working_graph, nets);
    const auto scopes = build_all_scopes(working_graph, nets);
    const auto delays = compute_pair_delays(working_graph, nets, scopes);
    require(delays.pairs.size() == 1, "PNnet fixture must have one pair delay");
    const int physical_shortest = bfs_shortest_delay(graph, scopes[0], 0, 3);
    require(physical_shortest == 2, "PNnet fixture physical shortest path must be 2 hops");
    require(
        delays.pairs[0].delays.size() == 1 && delays.pairs[0].delays[0] == physical_shortest + 1,
        "PNnet initial delay must be 1 + min physical shortest path");
    require(
        delays.pairs[0].source_node == nets[0].virtual_source_node,
        "PNnet pair delay must reference virtual source node");
}

auto test_pnnet_forbid_track_delay_ne_1() -> void {
    const auto graph = synthetic_graph(4, {{0, 2}, {1, 2}, {2, 3}});
    const auto nets = std::Vector<RoutingNet> {synthetic_pnnet(0, {0, 1}, {{3, {0, 1}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    require(!model.sources.empty(), "PNnet model must allocate a logical source block");
    const int track_d2 = d_lit_at(model, 0, 0, 2);
    if (track_d2 > 0) {
        auto forced_session = CadicalSession {};
        const auto forced_model = build_v14_model(forced_session, graph, nets);
        const int lit = d_lit_at(forced_model, 0, 0, 2);
        forced_session.add_clause({lit});
        require(
            !forced_session.solve_once().ok,
            "forcing D(r_n, track, d!=1) must be UNSAT for PNnet");
    }
}

auto test_pnnet_virtual_arc_sat_extract() -> void {
    const auto graph = synthetic_graph(4, {{0, 2}, {1, 2}, {2, 3}});
    const auto nets = std::Vector<RoutingNet> {synthetic_pnnet(0, {0, 1}, {{3, {0, 1}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "PNnet fork fixture must be SAT");
    auto ext_graph = graph;
    auto ext_nets = nets;
    augment_graph_for_pnnet(ext_graph, ext_nets);
    const auto result = extract_sat_solution(ext_graph, ext_nets, model, session, solved);
    require(result.ok, "PNnet extraction must succeed");
    require(result.paths.size() == 1, "PNnet fixture must extract one path");
    const auto& extracted = result.paths[0];
    require(
        extracted.physical_source_node == 0 || extracted.physical_source_node == 1,
        "PNnet path must record selected physical track");
    require(
        extracted.node_path.front() == extracted.physical_source_node,
        "PNnet extracted path must start at selected track");
    require(
        extracted.node_path.back() == 3,
        "PNnet extracted path must end at sink");
    require(
        std::find(extracted.node_path.begin(), extracted.node_path.end(), ext_nets[0].virtual_source_node)
            == extracted.node_path.end(),
        "PNnet extracted path must not include virtual source node");
}

auto test_log_routing_paths_two_pin() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "two-pin fixture must be SAT");
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok, "two-pin extraction must succeed");
    log_routing_paths(graph, nets, result);
}

auto test_validate_accepts_valid_two_pin() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "validator SAT fixture must solve");
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok, "validator fixture extraction must succeed");

    const auto validation = validate_routing_solution(graph, nets, model, session, result);
    require(validation.pass, "valid two-pin route must pass validation");
    require(validation.violations_count == 0, "valid route must not report violations");
}

auto test_validate_uses_encoded_guide_scope_instead_of_legacy_bbox() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    auto net = synthetic_net(0, {0}, {{2, {0}}});
    net.has_global_route_guide = true;
    net.scope_bbox = IlpBoundingBox {8, 8, 11, 11};
    require(
        !node_in_scope(graph, 1, net.scope_bbox),
        "guide-scope fixture must place its route outside the legacy bbox");
    const auto nets = std::Vector<RoutingNet> {net};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "guide-scope validator fixture must solve");
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok, "guide-scope validator fixture extraction must succeed");

    const auto validation = validate_routing_solution(graph, nets, model, session, result);
    require(
        validation.pass,
        "a path inside the encoded guide scope must not "
                             "be rejected by the legacy bbox");
}

auto test_validate_detects_node_outside_encoded_scope() -> void {
    auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {0, 3}, {3, 2}});
    graph.nodes[3].kind = UnifiedNodeKind::Track;
    graph.nodes[3].track_dir = 0;
    graph.nodes[3].track_row = 8;
    graph.nodes[3].track_col = 11;
    auto net = synthetic_net(0, {0}, {{2, {0}}});
    net.has_global_route_guide = true;
    const auto nets = std::Vector<RoutingNet> {net};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    require(model.scopes[0].node_offset[3] < 0, "fixture track must be outside encoded scope");
    const auto solved = session.solve_once();
    require(solved.ok, "encoded-scope rejection fixture must solve before tampering");
    auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok && result.paths.size() == 1, "fixture must extract one path");
    result.paths[0].node_path = {0, 3, 2};

    const auto validation = validate_routing_solution(graph, nets, model, session, result);
    require(!validation.pass, "a path outside encoded scope must fail validation");
    require(
        validation.category_counts.contains(ViolationKind::NodeOutOfScope)
            && validation.category_counts.at(ViolationKind::NodeOutOfScope) > 0,
        "an encoded-scope escape must be reported under NodeOutOfScope");
}

auto test_validate_detects_missing_arc() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "missing-arc fixture must solve before tampering");
    auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok && result.paths.size() == 1, "missing-arc fixture must extract one path");
    result.paths[0].node_path = {0, 2};

    const auto validation = validate_routing_solution(graph, nets, model, session, result);
    require(!validation.pass, "missing arc must fail validation");
    require(
        validation.category_counts.contains(ViolationKind::MissingArc)
            && validation.category_counts.at(ViolationKind::MissingArc) > 0,
        "missing arc must be reported under MissingArc");
}

auto test_validate_detects_endpoint_mismatch() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {synthetic_net(0, {0}, {{2, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_v14_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "endpoint-mismatch fixture must solve before tampering");
    auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok && result.paths.size() == 1, "endpoint-mismatch fixture must extract one path");
    result.paths[0].node_path = {1, 2};

    const auto validation = validate_routing_solution(graph, nets, model, session, result);
    require(!validation.pass, "endpoint mismatch must fail validation");
    require(
        validation.category_counts.contains(ViolationKind::EndpointMismatch)
            && validation.category_counts.at(ViolationKind::EndpointMismatch) > 0,
        "endpoint mismatch must be reported under EndpointMismatch");
}

auto test_unique_failed_net_ids_deduplicates() -> void {
    const auto critical = std::Vector<PairKey> {
        PairKey {3, 0, 0},
        PairKey {3, 1, 0},
        PairKey {7, 0, 0},
    };
    const auto ids = unique_failed_net_ids(critical);
    require(ids.size() == 2, "failed net ids must deduplicate by net_id");
    require(ids[0] == 3 && ids[1] == 7, "failed net ids must be sorted ascending");
}

auto test_ideal_two_pin_wirelength_matches_shortest_path() -> void {
    auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
    graph.nodes[0].kind = UnifiedNodeKind::Bump;
    graph.nodes[1].kind = UnifiedNodeKind::Track;
    graph.nodes[2].kind = UnifiedNodeKind::Bump;
    auto net = synthetic_net(0, {0}, {{2, {0}}});
    auto delays = DelayPrecomputeResult {};
    delays.pairs.push_back(PairDelayInfo {
        0, 0, 0, 0, 2, std::Vector<int> {2}, 2, -1});
    const auto path = shortest_unified_node_path(graph, 0, 2);
    require(path == std::Vector<int>({0, 1, 2}), "fixture must expose a unique shortest path");
    const auto expected = unified_path_wirelength(graph, path);
    require(
        ideal_net_wirelength(nullptr, graph, net, delays) == expected,
        "two-pin ideal wirelength must match bump+track count on shortest "
            "path");
}

auto test_ideal_sync_bus_wirelength_scales_by_members() -> void {
    auto graph = synthetic_graph(5, {{0, 1}, {1, 2}, {3, 4}});
    graph.nodes[0].kind = UnifiedNodeKind::Bump;
    graph.nodes[1].kind = UnifiedNodeKind::Track;
    graph.nodes[2].kind = UnifiedNodeKind::Bump;
    graph.nodes[3].kind = UnifiedNodeKind::Bump;
    graph.nodes[4].kind = UnifiedNodeKind::Bump;
    auto net = synthetic_net(0, {0, 3}, {{2, {0}}, {4, {1}}});
    net.is_sync_bus = true;
    auto delays = DelayPrecomputeResult {};
    delays.pairs.push_back(PairDelayInfo {
        0, 0, 0, 0, 2, std::Vector<int> {2}, 2, 2});
    delays.pairs.push_back(PairDelayInfo {
        0, 1, 1, 3, 4, std::Vector<int> {2}, 2, 3});
    const auto reference_path = shortest_unified_node_path(graph, 3, 4);
    const auto reference_wirelength = unified_path_wirelength(graph, reference_path);
    require(
        ideal_net_wirelength(nullptr, graph, net, delays) == reference_wirelength * net.demands.size(),
        "sync-bus ideal wirelength must scale by member count");
}

auto read_wirelength_golden(const std::string& golden_path) -> std::size_t {
    std::ifstream input {golden_path};
    if (!input.is_open()) {
        throw std::runtime_error("failed to open golden wirelength file: " + golden_path);
    }
    std::size_t expected = 0;
    input >> expected;
    if (!input || expected == 0) {
        throw std::runtime_error("invalid golden wirelength in: " + golden_path);
    }
    return expected;
}

auto solve_testlength_case(const std::string& case_dir) -> SatRoutingResult {
    auto [interposer, basedie] = parse::read_config(case_dir, 0, false);
    algo::build_nets(basedie.get(), interposer.get());
    UnifiedSatSolveOptions options {};
    options.verbose_level = 0;
    return solve_unified_sat(interposer.get(), *basedie.get(), options);
}

auto test_wirelength_matches_testlength_golden() -> void {
    constexpr auto kTestlengthRoot = "test/module_test/test_function/testlength";
    const auto strict_cases = std::Vector<const char*> {
        "testiosimple",
        "testchipletsimple",
        "testchipletbus",
        "testiobus",
    };
    for (const auto* case_name : strict_cases) {
        const std::string case_dir = std::string {kTestlengthRoot} + "/" + case_name;
        const auto expected = read_wirelength_golden(case_dir + "/golden.txt");
        const auto result = solve_testlength_case(case_dir);
        require(result.ok, std::string {case_name} + " must solve with unified SAT");
        require(
            result.total_wirelength == expected,
            std::string {case_name} + " wirelength mismatch: got "
                + std::to_string(result.total_wirelength) + " expected "
                + std::to_string(expected));
    }

    const std::string pn_dir = std::string {kTestlengthRoot} + "/testpn";
    const auto pn_result = solve_testlength_case(pn_dir);
    require(pn_result.ok, "testpn must solve with unified SAT");
}
