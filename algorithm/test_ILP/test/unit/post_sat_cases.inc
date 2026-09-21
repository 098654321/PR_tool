// V22 whole-route candidate ILP, V20 post-SAT Maze/RRR, guided V18 smoke
// tests, and shared builders used by the lower-level SAT suites.

auto make_bidirectional_edges(
    std::initializer_list<std::pair<int, int>> edges)
    -> std::Vector<std::pair<int, int>> {
  auto out = std::Vector<std::pair<int, int>>{};
  for (const auto [u, v] : edges) {
    out.emplace_back(u, v);
    out.emplace_back(v, u);
  }
  return out;
}

auto make_all_track_graph(
    const std::size_t node_count,
    std::initializer_list<std::pair<int, int>> edges) -> UnifiedGraph {
  auto graph = synthetic_graph(node_count, make_bidirectional_edges(edges));
  for (std::size_t node = 0; node < node_count; ++node) {
    (void)synthetic_track_ref(
        graph, static_cast<int>(node), 0, 0, static_cast<int>(node), node);
  }
  return graph;
}

auto test_v22_candidate_ilp_uses_full_fixed_unit_graph() -> void {
  auto graph = make_all_track_graph(
      4, {{0, 1}, {1, 2}, {2, 3}, {0, 3}});
  const auto nets = std::Vector<RoutingNet>{
      synthetic_net(0, {0}, {{3, {0}}})};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2, 3}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  const auto refined = optimize_post_sat_candidate_routes(
      graph, nets, baseline, {});
  require(refined.post_sat_ilp_attempted && refined.post_sat_ilp_accepted
              && refined.total_wirelength == 2
              && refined.paths.front().node_path == std::Vector<int>({0, 3}),
          "V22 candidate ILP must search the complete fixed-unit graph rather "
          "than the final SAT scope");
}

auto test_v22_two_pin_second_candidate_length_policy() -> void {
  {
    auto graph = make_all_track_graph(
        8, {{0, 1}, {1, 3}, {0, 2}, {2, 4}, {4, 3},
            {0, 5}, {5, 6}, {6, 7}, {7, 3}});
    const auto nets = std::Vector<RoutingNet>{
        synthetic_net(0, {0}, {{3, {0}}})};
    auto baseline = SatRoutingResult{};
    baseline.ok = true;
    baseline.paths = {
        SourceSinkPairPath{0, 0, 0, -1, {0, 5, 6, 7, 3}}};
    baseline.total_wirelength = total_wirelength(graph, baseline);

    const auto refined = optimize_post_sat_candidate_routes(
        graph, nets, baseline, {});
    require(refined.post_sat_ilp_accepted
                && refined.post_sat_ilp_segments == 3
                && refined.paths.front().node_path
                    == std::Vector<int>({0, 1, 3}),
            "V22 two-pin generation must retain a physical-wirelength "
            "Lmin+1 route in addition to the shortest and SAT candidates");
  }
  {
    auto graph = make_all_track_graph(
        7, {{0, 1}, {0, 2}, {2, 3}, {3, 1},
            {0, 4}, {4, 5}, {5, 6}, {6, 1}});
    const auto nets = std::Vector<RoutingNet>{
        synthetic_net(0, {0}, {{1, {0}}})};
    auto baseline = SatRoutingResult{};
    baseline.ok = true;
    baseline.paths = {
        SourceSinkPairPath{0, 0, 0, -1, {0, 4, 5, 6, 1}}};
    baseline.total_wirelength = total_wirelength(graph, baseline);

    const auto refined = optimize_post_sat_candidate_routes(
        graph, nets, baseline, {});
    require(refined.post_sat_ilp_accepted
                && refined.post_sat_ilp_segments == 3
                && refined.paths.front().node_path == std::Vector<int>({0, 1}),
            "V22 two-pin generation must use the true next-shortest route "
            "when no Lmin+1 route exists");
  }
}

auto test_v22_candidate_ilp_selects_nonconflicting_routes() -> void {
  auto graph = make_all_track_graph(
      11, {{0, 1}, {1, 3}, {0, 4}, {4, 5}, {5, 10}, {10, 3},
           {6, 1}, {1, 7}, {6, 8}, {8, 9}, {9, 7}});
  const auto nets = std::Vector<RoutingNet>{
      synthetic_net(0, {0}, {{3, {0}}}),
      synthetic_net(1, {6}, {{7, {0}}})};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {
      SourceSinkPairPath{0, 0, 0, -1, {0, 4, 5, 10, 3}},
      SourceSinkPairPath{1, 0, 0, -1, {6, 8, 9, 7}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  const auto refined = optimize_post_sat_candidate_routes(
      graph, nets, baseline, {});
  require(refined.post_sat_ilp_accepted
              && refined.total_wirelength < baseline.total_wirelength,
          "V22 candidate ILP must choose a shorter combination without sharing "
          "the common shortest-path node");
  auto owners = std::map<int, std::set<std::size_t>>{};
  for (const auto& path : refined.paths) {
    for (const int node : path.node_path) {
      owners[node].insert(path.net_id);
    }
  }
  require(std::ranges::none_of(owners, [](const auto& entry) {
            return entry.second.size() > 1;
          }),
          "V22 selected candidates must obey physical node capacity one");
}

auto test_v22_multi_terminal_candidate_is_a_whole_tree() -> void {
  auto graph = make_all_track_graph(
      7, {{0, 1}, {1, 2}, {2, 3}, {2, 4}, {0, 5}, {5, 6}, {6, 4}});
  const auto nets = std::Vector<RoutingNet>{
      synthetic_net(0, {0}, {{3, {0}}, {4, {0}}})};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {
      SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2, 3}},
      SourceSinkPairPath{0, 0, 1, -1, {0, 5, 6, 4}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  const auto refined = optimize_post_sat_candidate_routes(
      graph, nets, baseline, {});
  require(refined.post_sat_ilp_accepted && refined.paths.size() == 2
              && refined.total_wirelength == 5,
          "V22 multi-terminal candidate must select one complete shared tree "
          "and charge its Track+Bump union once");
}

auto test_v22_ilp_optimize_chains_maze_then_full_space_ilp() -> void {
  auto graph = make_all_track_graph(
      7, {{0, 1}, {1, 2}, {2, 6}, {6, 3},
          {0, 4}, {4, 5}, {5, 3}, {0, 3}});
  const auto nets = std::Vector<RoutingNet>{
      synthetic_net(0, {0}, {{3, {0}}})};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {
      SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2, 6, 3}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  auto scope_nodes = std::Vector<int>{0, 1, 2, 3, 4, 5, 6};
  auto scope_arcs = std::Vector<int>{};
  for (std::size_t arc_id = 0; arc_id < graph.arcs.size(); ++arc_id) {
    const auto& arc = graph.arcs[arc_id];
    if (!((arc.u == 0 && arc.v == 3)
          || (arc.u == 3 && arc.v == 0))) {
      scope_arcs.push_back(static_cast<int>(arc_id));
    }
  }
  const auto scope = synthetic_scope(graph, 0, scope_nodes, scope_arcs);
  const auto refined = optimize_post_sat_routes(
      graph, nets, {scope}, baseline,
      PostSatIlpOptions{.verbose_level = 0});

  require(refined.post_sat_maze_attempted
              && refined.post_sat_maze_accepted
              && refined.post_sat_maze_baseline_wirelength == 5
              && refined.post_sat_maze_wirelength == 4,
          "--ilp-optimize must first retain the shorter route found inside "
          "the final SAT scope by Maze/RRR");
  require(refined.post_sat_ilp_attempted
              && refined.post_sat_ilp_accepted
              && refined.post_sat_ilp_baseline_wirelength == 4
              && refined.post_sat_ilp_wirelength == 2
              && refined.paths.front().node_path == std::Vector<int>({0, 3}),
          "candidate ILP must use the Maze route as incumbent, discard scope, "
          "and accept a further full-space improvement");
}

auto test_v20_post_sat_rrr_improves_within_final_scope() -> void {
  const auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {2, 3}, {0, 3}});
  const auto nets = std::Vector<RoutingNet>{synthetic_net(0, {0}, {{3, {0}}})};
  const auto scopes =
      std::Vector<UnifiedSatNetScope>{synthetic_scope(graph, 0)};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2, 3}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  const auto refined = optimize_post_sat_routes_rrr(
      graph, nets, scopes, baseline,
      PostSatRrrOptions{
          .max_iterations = 8, .stagnation_limit = 4, .max_sweeps = 1});
  require(refined.ok, "post-SAT RRR must preserve the SAT success state");
  require(refined.post_sat_maze_attempted && refined.post_sat_maze_accepted,
          "post-SAT RRR must accept a strictly shorter legal route");
  require(refined.total_wirelength == 2 && refined.paths.size() == 1 &&
              refined.paths.front().node_path == std::Vector<int>({0, 3}),
          "post-SAT RRR must stay in scope and select the shorter path");
}

auto test_v20_post_sat_rrr_keeps_sync_as_hard_obstacle() -> void {
  const auto graph = synthetic_graph(
      7, {{0, 1}, {1, 2}, {2, 3}, {0, 4}, {4, 3}, {5, 4}, {4, 6}});
  auto ordinary = synthetic_net(0, {0}, {{3, {0}}});
  auto sync = synthetic_net(1, {5}, {{6, {0}}});
  sync.is_sync_bus = true;
  const auto nets = std::Vector<RoutingNet>{ordinary, sync};
  const auto scopes = std::Vector<UnifiedSatNetScope>{
      synthetic_scope(graph, 0), synthetic_scope(graph, 1)};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2, 3}},
                    SourceSinkPairPath{1, 0, 0, -1, {5, 4, 6}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  const auto refined = optimize_post_sat_routes_rrr(
      graph, nets, scopes, baseline,
      PostSatRrrOptions{
          .max_iterations = 8, .stagnation_limit = 4, .max_sweeps = 1});
  require(!refined.post_sat_maze_accepted &&
              refined.total_wirelength == baseline.total_wirelength,
          "a shorter route through a Sync resource must not be accepted");
  const auto sync_path =
      std::find_if(refined.paths.begin(), refined.paths.end(),
                   [](const auto &path) { return path.net_id == 1; });
  require(sync_path != refined.paths.end() &&
              sync_path->node_path == std::Vector<int>({5, 4, 6}),
          "post-SAT RRR must keep every Sync path byte-for-byte unchanged");
}

auto test_v20_post_sat_rrr_repairs_non_sync_overflow_cascade() -> void {
  const auto graph = synthetic_graph(16, {{0, 1},
                                          {1, 2},
                                          {2, 3},
                                          {3, 4},
                                          {4, 5},
                                          {5, 6},
                                          {6, 7},
                                          {7, 8},
                                          {8, 9},
                                          {9, 10},
                                          {0, 11},
                                          {11, 10},
                                          {12, 11},
                                          {11, 13},
                                          {12, 14},
                                          {14, 15},
                                          {15, 13}});
  const auto nets = std::Vector<RoutingNet>{
      synthetic_net(0, {0}, {{10, {0}}}), synthetic_net(1, {12}, {{13, {0}}})};
  const auto scopes = std::Vector<UnifiedSatNetScope>{
      synthetic_scope(graph, 0), synthetic_scope(graph, 1)};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {
      SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
      SourceSinkPairPath{1, 0, 0, -1, {12, 11, 13}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  const auto refined = optimize_post_sat_routes_rrr(
      graph, nets, scopes, baseline,
      PostSatRrrOptions{
          .max_iterations = 8, .stagnation_limit = 4, .max_sweeps = 1});
  require(refined.post_sat_maze_accepted &&
              refined.total_wirelength < baseline.total_wirelength,
          "a seed conflict with another non-Sync net must be repairable by "
          "dirty-owner RRR");
  require(refined.post_sat_maze_rrr_iterations >= 1 &&
              refined.post_sat_maze_rerouted_owners >= 2,
          "overflow repair must rip the full dirty set before rerouting");
}

auto test_v22_full_post_guide_maze_rebuilds_all_non_sync() -> void {
  auto graph = synthetic_graph(
      8, {{0, 1}, {1, 2}, {2, 3}, {0, 3},
          {4, 5}, {5, 6}, {6, 7}, {4, 7}});
  const auto source0 = synthetic_track_ref(graph, 0, 0, 0, 0);
  const auto source1 = synthetic_track_ref(graph, 4, 0, 0, 4);
  for (const int node : {1, 2, 3, 5, 6, 7}) {
    (void)synthetic_track_ref(graph, node, 0, 0, node);
  }
  auto net0 = RoutingNet{};
  net0.net_id = 0;
  net0.kind = RoutingNetKind::Tnet;
  net0.sources = {source0};
  net0.demands = {RoutingDemand{0, synthetic_ref(3), {0}, true}};
  auto net1 = RoutingNet{};
  net1.net_id = 1;
  net1.kind = RoutingNetKind::Tnet;
  net1.sources = {source1};
  net1.demands = {RoutingDemand{0, synthetic_ref(7), {0}, true}};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2, 3}},
                    SourceSinkPairPath{1, 0, 0, -1, {4, 5, 6, 7}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  const auto rebuilt = rebuild_all_non_sync_routes_rrr(
      graph, {net0, net1}, {synthetic_scope(graph, 0), synthetic_scope(graph, 1)},
      baseline, PostSatRrrOptions{.max_iterations = 8, .stagnation_limit = 4});
  require(rebuilt.has_value() && rebuilt->total_wirelength == 4,
          "V22 full maze must rebuild every non-Sync owner from the Sync-only state");
  require(rebuilt->paths.size() == 2
              && rebuilt->paths[0].node_path == std::Vector<int>({0, 3})
              && rebuilt->paths[1].node_path == std::Vector<int>({4, 7}),
          "V22 full maze must not retain an old non-Sync SAT path outside the guide");
}

auto test_v22_full_post_guide_maze_preserves_pn_physical_source() -> void {
  auto graph = synthetic_graph(3, {{0, 1}, {1, 2}, {0, 2}});
  const auto source0 = synthetic_track_ref(graph, 0, 2, 0, 0);
  const auto source1 = synthetic_track_ref(graph, 1, 2, 0, 1);
  (void)synthetic_track_ref(graph, 2, 2, 0, 2);
  auto pn = RoutingNet{};
  pn.net_id = 9;
  pn.kind = RoutingNetKind::PNnet;
  pn.sources = {source0, source1};
  pn.demands = {RoutingDemand{0, synthetic_ref(2), {0, 1}, false}};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{9, 0, 0, 0, {0, 1, 2}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  const auto rebuilt = rebuild_all_non_sync_routes_rrr(
      graph, {pn}, {synthetic_scope(graph, 9)}, baseline,
      PostSatRrrOptions{.max_iterations = 8, .stagnation_limit = 4});
  require(rebuilt.has_value() && rebuilt->paths.size() == 1
              && rebuilt->paths.front().physical_source_node == 0
              && rebuilt->paths.front().node_path == std::Vector<int>({0, 2}),
          "V22 full maze must retain the SAT-selected PN physical source");
}

auto test_v17_pn_scope_retains_selected_union_only() -> void {
  auto graph = build_unified_graph(nullptr, {});
  auto pn = RoutingNet{};
  pn.net_id = 9;
  pn.kind = RoutingNetKind::PNnet;
  pn.sources = {track_ref(0, 0, hardware::TrackDirection::Horizontal, 0),
                track_ref(0, 0, hardware::TrackDirection::Horizontal, 8),
                track_ref(0, 1, hardware::TrackDirection::Horizontal, 1)};
  pn.demands = {RoutingDemand{0, bump_ref(0, 0, 0), {0, 1, 2}, false},
                RoutingDemand{1, bump_ref(0, 0, 1), {0, 1, 2}, false}};
  pn.global_selected_pn_source_indices = {0, 2};
  pn.global_route_channels = {
      GlobalChannelCoord{0, 0, 0}, GlobalChannelCoord{0, 0, 1},
      GlobalChannelCoord{1, static_cast<int>(tob_anchor_cob(0).row),
                         static_cast<int>(tob_anchor_cob(0).col)}};
  pn.has_global_route_guide = true;
  auto nets = std::Vector<RoutingNet>{pn};
  augment_graph_for_pnnet(graph, nets);
  const auto scopes = build_all_scopes(graph, nets);
  const int selected0 = resolve_graph_node(graph, nets[0].sources[0]);
  const int rejected = resolve_graph_node(graph, nets[0].sources[1]);
  const int selected2 = resolve_graph_node(graph, nets[0].sources[2]);
  bool selected0_arc_in_scope = false;
  bool selected2_arc_in_scope = false;
  bool rejected_arc_in_scope = false;
  for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(
           nets[0].virtual_source_node)]) {
    const auto &arc = graph.arcs[static_cast<std::size_t>(arc_id)];
    const bool included =
        scopes[0].arc_offset[static_cast<std::size_t>(arc_id)] >= 0;
    selected0_arc_in_scope |= arc.v == selected0 && included;
    selected2_arc_in_scope |= arc.v == selected2 && included;
    rejected_arc_in_scope |= arc.v == rejected && included;
  }
  require(selected0_arc_in_scope && selected2_arc_in_scope &&
              !rejected_arc_in_scope,
          "V17 multi-sink PN scope must retain the selected-source union and "
          "mask an unselected same-Channel/unit virtual arc");
}

auto test_v18_guided_cadical_smoke() -> void {
  auto [interposer, basedie] =
      parse::read_config("algorithm/test_ILP/test/case_2btb", 0, false);
  algo::build_nets(basedie.get(), interposer.get());
  auto options = UnifiedSatSolveOptions{};
  options.enable_global_route_v18 = true;
  options.enable_z3_optimize = false;
  options.max_feedback_rounds = 4;
  const auto result = solve_unified_sat(interposer.get(), *basedie, options);
  require(result.ok && result.global_route_requested &&
              result.global_route_status == "OPTIMAL" &&
              result.global_route_capacity_cuts_enabled &&
              result.paths.size() == 2 && result.occupancy_vars == 0 &&
              result.occupancy_implication_clauses == 0 &&
              result.occupancy_soft_clauses == 0 && result.total_wirelength > 0,
          "V18 must route through HiGHS and guided CaDiCaL without occupancy "
          "objective support");
}

auto build_v14_model(CadicalSession &session, const UnifiedGraph &graph_in,
                     const std::Vector<RoutingNet> &nets_in,
                     SatEncodingStats *stats = nullptr,
                     bool assume_pairs = true) -> UnifiedSatModel {
  auto graph = graph_in;
  auto nets = nets_in;
  augment_graph_for_pnnet(graph, nets);
  const auto scopes = build_all_scopes(graph, nets);
  const auto delays = compute_pair_delays(graph, nets, scopes);
  auto model =
      build_unified_sat_model(session, graph, nets, scopes, delays, stats);
  if (assume_pairs) {
    for (const auto &alpha : model.alpha_vars) {
      session.assume(alpha.alpha_lit);
    }
  }
  return model;
}

auto d_lit_at(const UnifiedSatModel &model, std::size_t model_source_index,
              int node, int delay) -> int {
  const auto &source = model.sources[model_source_index];
  const auto &scope = model.scopes[source.scope_index];
  const int node_offset = scope.node_offset[static_cast<std::size_t>(node)];
  if (node_offset < 0 || delay < 0 || delay > source.d_max) {
    return 0;
  }
  return source.d_var[static_cast<std::size_t>(node_offset)]
                     [static_cast<std::size_t>(delay)];
}

auto a_lit_at(const UnifiedSatModel &model, int arc_id, int delay,
              std::size_t model_source_index = 0) -> int {
  for (const auto &tob_arc : model.tob_arcs) {
    if (tob_arc.arc_global_id == arc_id &&
        tob_arc.model_source_index == model_source_index &&
        delay <= tob_arc.d_max) {
      return tob_arc.a_var[static_cast<std::size_t>(delay)];
    }
  }
  return 0;
}
