// V22 post-SAT pair-flow ILP, V20 post-SAT Maze/RRR, guided V18 smoke tests,
// and shared builders used by the lower-level SAT suites.

auto test_v22_post_sat_ilp_target_filter() -> void {
  auto target = RoutingNet{};
  target.kind = RoutingNetKind::Tnet;
  target.post_sat_ilp_target = true;
  target.sources = {track_ref(0, 0, hardware::TrackDirection::Horizontal, 0)};
  target.demands = {RoutingDemand{0, bump_ref(0, 0, 0), {0}, true}};
  require(is_post_sat_ilp_target(target),
          "non-Sync Track-to-Bump must be selected");

  auto normalized_bump_to_track = target;
  normalized_bump_to_track.post_sat_ilp_target = false;
  require(
      is_post_sat_ilp_target(normalized_bump_to_track),
      "normalized BumpToTrack must be selected without a legacy target tag");

  auto sync = target;
  sync.is_sync_bus = true;
  require(!is_post_sat_ilp_target(sync),
          "Sync must remain frozen even if accidentally tagged");
}

auto test_v20_post_sat_ilp_improves_fixed_source_path() -> void {
  auto graph = synthetic_graph(4, {{0, 1}, {1, 2}, {2, 3}, {0, 3}});
  graph.bump_node_by_key.erase(graph.nodes[0].bump);
  graph.nodes[0].kind = UnifiedNodeKind::Track;
  graph.nodes[0].unit = map_track(0);
  graph.nodes[0].track_dir = 0;
  graph.nodes[0].track_row = 0;
  graph.nodes[0].track_col = 0;
  graph.nodes[0].track_index = 0;
  graph.track_node_by_key.emplace(
      std::tuple{map_track(0), 0, 0, 0, std::size_t{0}}, 0);
  for (int node : {1, 2}) {
    graph.bump_node_by_key.erase(
        graph.nodes[static_cast<std::size_t>(node)].bump);
    graph.nodes[static_cast<std::size_t>(node)].kind = UnifiedNodeKind::Track;
    graph.nodes[static_cast<std::size_t>(node)].unit = map_track(0);
    graph.nodes[static_cast<std::size_t>(node)].track_dir = 0;
    graph.nodes[static_cast<std::size_t>(node)].track_row = 0;
    graph.nodes[static_cast<std::size_t>(node)].track_col = node;
    graph.nodes[static_cast<std::size_t>(node)].track_index = 0;
  }

  auto net = RoutingNet{};
  net.net_id = 0;
  net.name = "TrackToBumpNet_synthetic";
  net.kind = RoutingNetKind::Tnet;
  net.post_sat_ilp_target = true;
  net.sources = {track_ref(0, 0, hardware::TrackDirection::Horizontal, 0)};
  net.demands = {RoutingDemand{0, synthetic_ref(3), {0}, true}};
  const auto nets = std::Vector<RoutingNet>{net};
  const auto scopes =
      std::Vector<UnifiedSatNetScope>{synthetic_scope(graph, 0)};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2, 3}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  const auto refined = optimize_post_sat_routes(graph, nets, scopes, baseline,
                                                PostSatIlpOptions{});
  require(refined.ok, "post-SAT ILP must preserve a successful SAT result");
  require(refined.post_sat_ilp_accepted,
          "synthetic shorter incumbent must be accepted");
  require(refined.total_wirelength == 2 && refined.paths.size() == 1 &&
              refined.paths.front().node_path == std::Vector<int>({0, 3}),
          "post-SAT ILP must choose the shorter fixed-source path");
}

auto test_v22_post_sat_ilp_supports_bump_root() -> void {
  auto graph = synthetic_graph(3, {{0, 1}, {1, 2}});
  (void)synthetic_track_ref(graph, 1, 5, 0, 1);

  auto net = RoutingNet{};
  net.net_id = 0;
  net.name = "BumpToBump_synthetic";
  net.kind = RoutingNetKind::Bnet;
  net.sources = {synthetic_ref(2)};
  net.demands = {RoutingDemand{0, synthetic_ref(0), {0}, true}};

  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  // The synthetic graph stores only 0->1 and 1->2.  Routing 2->0
  // verifies that V22 collapses the resource graph to undirected edges.
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {2, 1, 0}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  const auto refined = optimize_post_sat_routes(
      graph, {net}, {synthetic_scope(graph, 0)}, baseline, PostSatIlpOptions{});

  require(refined.post_sat_ilp_accepted,
          "V22 must optimize a Bump-root net on undirected resources");
  require(refined.paths.size() == 1 &&
              refined.paths.front().node_path == std::Vector<int>({2, 1, 0}),
          "V22 must preserve endpoints while traversing stored arcs in "
          "reverse");
}

auto test_v22_post_sat_ilp_moves_multiterminal_branch() -> void {
  auto graph = synthetic_graph(
      6, {{0, 1}, {1, 3}, {0, 2}, {2, 4}, {0, 5}, {5, 3}, {5, 4}});
  graph.arcs[4].physical_switch_id = 42;
  graph.arcs[4].physical_switch_kind = PhysicalSwitchKind::BumpH;
  const auto source = synthetic_track_ref(graph, 0, 0, 0, 0);
  (void)synthetic_track_ref(graph, 1, 0, 0, 1);
  (void)synthetic_track_ref(graph, 2, 0, 0, 2);
  (void)synthetic_track_ref(graph, 5, 0, 0, 5);

  auto net = RoutingNet{};
  net.net_id = 0;
  net.name = "TrackToBumps_synthetic";
  net.kind = RoutingNetKind::Tnet;
  net.sources = {source};
  net.demands = {RoutingDemand{0, synthetic_ref(3), {0}, true},
                 RoutingDemand{1, synthetic_ref(4), {0}, true}};

  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 3}},
                    SourceSinkPairPath{0, 0, 1, -1, {0, 2, 4}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  const auto refined = optimize_post_sat_routes(
      graph, {net}, {synthetic_scope(graph, 0)}, baseline, PostSatIlpOptions{});

  require(refined.post_sat_ilp_accepted && refined.total_wirelength == 4,
          "pair flows with a global edge union must reduce physical "
          "wirelength");
  require(refined.paths.size() == 2 &&
              refined.paths[0].node_path == std::Vector<int>({0, 5, 3}) &&
              refined.paths[1].node_path == std::Vector<int>({0, 5, 4}),
          "pair flows must share a better trunk in their scope intersection");
  require(refined.used_tob_switch_ids == std::Vector<int>({42}),
          "same-net pairs must share one physical-edge Y variable");
}

auto test_v22_post_sat_ilp_uses_pair_local_scope_and_union_warm_start()
    -> void {
  auto graph = synthetic_graph(
      6, {{0, 1}, {1, 3}, {0, 2}, {2, 4}, {0, 5}, {5, 3}, {5, 4}});
  const auto source = synthetic_track_ref(graph, 0, 0, 0, 0);
  for (const int node : {1, 2, 5})
    (void)synthetic_track_ref(graph, node, 0, 0, node);
  auto net = RoutingNet{};
  net.net_id = 31;
  net.kind = RoutingNetKind::Tnet;
  net.sources = {source};
  net.demands = {RoutingDemand{0, synthetic_ref(3), {0}, true},
                 RoutingDemand{1, synthetic_ref(4), {0}, true}};
  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{31, 0, 0, -1, {0, 1, 3}},
                    SourceSinkPairPath{31, 0, 1, -1, {0, 2, 4}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  auto state = init_routing_problem_state({net});
  state.pairs[0].allowed_channels = {GlobalChannelCoord{0, 0, 0},
                                     GlobalChannelCoord{0, 0, 1},
                                     GlobalChannelCoord{0, 0, 3}};
  state.pairs[1].allowed_channels = {GlobalChannelCoord{0, 0, 0},
                                     GlobalChannelCoord{0, 0, 2},
                                     GlobalChannelCoord{0, 0, 4}};
  const auto refined =
      optimize_post_sat_routes(graph, {net}, {synthetic_scope(graph, 31)},
                               baseline, PostSatIlpOptions{}, &state);
  require(refined.post_sat_ilp_accepted && refined.total_wirelength == 5,
          "pair-local domains must not borrow another pair's route-only node");
  require(std::ranges::none_of(refined.paths,
                               [](const auto &path) {
                                 return std::ranges::find(path.node_path, 5) !=
                                        path.node_path.end();
                               }),
          "pair-local post-SAT ILP must exclude a net-union-only shortcut");
}

auto test_v22_post_sat_ilp_exact_components_follow_scope_overlap() -> void {
  auto graph = synthetic_graph(
      7, {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {0, 6}, {6, 2}, {3, 6}, {6, 5}});
  const auto source0 = synthetic_track_ref(graph, 0, 0, 0, 0);
  (void)synthetic_track_ref(graph, 1, 0, 0, 1);
  const auto source1 = synthetic_track_ref(graph, 3, 0, 0, 3);
  (void)synthetic_track_ref(graph, 4, 0, 0, 4);
  (void)synthetic_track_ref(graph, 6, 0, 0, 6);

  auto net0 = RoutingNet{};
  net0.net_id = 0;
  net0.kind = RoutingNetKind::Tnet;
  net0.sources = {source0};
  net0.demands = {RoutingDemand{0, synthetic_ref(2), {0}, true}};
  auto net1 = RoutingNet{};
  net1.net_id = 1;
  net1.kind = RoutingNetKind::Tnet;
  net1.sources = {source1};
  net1.demands = {RoutingDemand{0, synthetic_ref(5), {0}, true}};

  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2}},
                    SourceSinkPairPath{1, 0, 0, -1, {3, 4, 5}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  const auto split =
      optimize_post_sat_routes(graph, {net0, net1},
                               {synthetic_scope(graph, 0, {0, 1, 2}, {0, 1}),
                                synthetic_scope(graph, 1, {3, 4, 5}, {2, 3})},
                               baseline, PostSatIlpOptions{});
  require(split.post_sat_ilp_accepted && split.post_sat_ilp_components == 2,
          "resource-disjoint scopes must form two exact components");

  const auto merged = optimize_post_sat_routes(
      graph, {net0, net1},
      {synthetic_scope(graph, 0), synthetic_scope(graph, 1)}, baseline,
      PostSatIlpOptions{});
  require(merged.post_sat_ilp_accepted && merged.post_sat_ilp_components == 1,
          "scopes sharing a candidate physical node must share one component");
}

auto test_v22_post_sat_ilp_jointly_exchanges_nonbus_resources() -> void {
  auto graph = synthetic_graph(
      8,
      {{0, 1}, {1, 7}, {7, 2}, {3, 4}, {4, 5}, {0, 4}, {4, 2}, {3, 6}, {6, 5}});
  const auto source0 = synthetic_track_ref(graph, 0, 0, 0, 0);
  (void)synthetic_track_ref(graph, 1, 0, 0, 1);
  const auto source1 = synthetic_track_ref(graph, 3, 0, 0, 3);
  (void)synthetic_track_ref(graph, 4, 0, 0, 4);
  (void)synthetic_track_ref(graph, 6, 0, 0, 6);
  (void)synthetic_track_ref(graph, 7, 0, 0, 7);

  auto net0 = RoutingNet{};
  net0.net_id = 0;
  net0.kind = RoutingNetKind::Tnet;
  net0.sources = {source0};
  net0.demands = {RoutingDemand{0, synthetic_ref(2), {0}, true}};
  auto net1 = RoutingNet{};
  net1.net_id = 1;
  net1.kind = RoutingNetKind::Tnet;
  net1.sources = {source1};
  net1.demands = {RoutingDemand{0, synthetic_ref(5), {0}, true}};

  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 7, 2}},
                    SourceSinkPairPath{1, 0, 0, -1, {3, 4, 5}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  const auto refined = optimize_post_sat_routes(
      graph, {net0, net1},
      {synthetic_scope(graph, 0), synthetic_scope(graph, 1)}, baseline,
      PostSatIlpOptions{});

  require(refined.post_sat_ilp_accepted && refined.total_wirelength == 6,
          "all non-bus nets must be optimized jointly rather than locked");
  require(refined.paths.size() == 2 &&
              refined.paths[0].node_path == std::Vector<int>({0, 4, 2}) &&
              refined.paths[1].node_path == std::Vector<int>({3, 6, 5}),
          "one non-bus net must be able to take another net's SAT resource "
          "after that net moves");
}

auto test_v22_post_sat_ilp_enforces_shared_mode_group() -> void {
  auto graph = synthetic_graph(11, {{0, 1},
                                    {1, 2},
                                    {0, 7},
                                    {7, 8},
                                    {8, 2},
                                    {3, 4},
                                    {4, 5},
                                    {3, 9},
                                    {9, 10},
                                    {10, 5}});
  const auto source0 = synthetic_track_ref(graph, 0, 0, 0, 0);
  const auto source1 = synthetic_track_ref(graph, 3, 0, 0, 3);
  for (const int node : {7, 8, 9, 10})
    (void)synthetic_track_ref(graph, node, 0, 0, node);
  for (const int node : {1, 4}) {
    graph.bump_node_by_key.erase(
        graph.nodes[static_cast<std::size_t>(node)].bump);
    graph.nodes[static_cast<std::size_t>(node)].kind = UnifiedNodeKind::VLine;
    graph.nodes[static_cast<std::size_t>(node)].line_index = 0;
  }
  graph.arcs[0].physical_switch_id = 10;
  graph.arcs[0].physical_switch_kind = PhysicalSwitchKind::VLineTrack;
  graph.arcs[0].mode_group_id = 7;
  graph.arcs[0].is_vline_track_straight = true;
  graph.arcs[5].physical_switch_id = 11;
  graph.arcs[5].physical_switch_kind = PhysicalSwitchKind::VLineTrack;
  graph.arcs[5].mode_group_id = 7;
  graph.arcs[5].is_vline_track_swap = true;

  auto net0 = RoutingNet{};
  net0.net_id = 0;
  net0.kind = RoutingNetKind::Tnet;
  net0.sources = {source0};
  net0.demands = {RoutingDemand{0, synthetic_ref(2), {0}, true}};
  auto net1 = RoutingNet{};
  net1.net_id = 1;
  net1.kind = RoutingNetKind::Tnet;
  net1.sources = {source1};
  net1.demands = {RoutingDemand{0, synthetic_ref(5), {0}, true}};

  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2}},
                    SourceSinkPairPath{1, 0, 0, -1, {3, 9, 10, 5}}};
  baseline.vline_mode_straight_by_group[7] = true;
  baseline.used_tob_switch_ids = {10};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  const auto refined = optimize_post_sat_routes(
      graph, {net0, net1},
      {synthetic_scope(graph, 0), synthetic_scope(graph, 1)}, baseline,
      PostSatIlpOptions{});

  const auto switches = std::set<int>(refined.used_tob_switch_ids.begin(),
                                      refined.used_tob_switch_ids.end());
  require(refined.post_sat_ilp_accepted && refined.total_wirelength == 6,
          "shared straight/swap mode must keep the legal SAT objective");
  require(switches.contains(10) != switches.contains(11),
          "one mode group must not select straight and swap switches together");
  require(refined.vline_mode_straight_by_group.at(7) == switches.contains(10),
          "reported mode must agree with the selected VLineTrack switch");
}

auto test_v22_post_sat_ilp_fixes_complete_sync_bus() -> void {
  auto graph = synthetic_graph(6, {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {3, 5}});
  const auto bus_source = synthetic_track_ref(graph, 0, 0, 0, 0);
  graph.bump_node_by_key.erase(graph.nodes[1].bump);
  graph.nodes[1].kind = UnifiedNodeKind::VLine;
  graph.nodes[1].line_index = 0;
  const auto data_source = synthetic_track_ref(graph, 3, 0, 0, 3);
  (void)synthetic_track_ref(graph, 4, 0, 0, 4);
  graph.arcs[0].physical_switch_id = 10;
  graph.arcs[0].physical_switch_kind = PhysicalSwitchKind::VLineTrack;
  graph.arcs[0].mode_group_id = 7;
  graph.arcs[0].is_vline_track_straight = true;

  auto bus = RoutingNet{};
  bus.net_id = 0;
  bus.kind = RoutingNetKind::Tnet;
  bus.is_sync_bus = true;
  bus.sources = {bus_source};
  bus.demands = {RoutingDemand{0, synthetic_ref(2), {0}, true}};
  auto data = RoutingNet{};
  data.net_id = 1;
  data.kind = RoutingNetKind::Tnet;
  data.sources = {data_source};
  data.demands = {RoutingDemand{0, synthetic_ref(5), {0}, true}};

  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2}},
                    SourceSinkPairPath{1, 0, 0, -1, {3, 4, 5}}};
  baseline.vline_mode_straight_by_group[7] = true;
  baseline.used_tob_switch_ids = {10};
  baseline.total_wirelength = total_wirelength(graph, baseline);
  const auto refined = optimize_post_sat_routes(
      graph, {bus, data},
      {synthetic_scope(graph, 0), synthetic_scope(graph, 1)}, baseline,
      PostSatIlpOptions{});

  require(refined.post_sat_ilp_accepted && refined.total_wirelength == 4,
          "V22 must optimize the non-bus net around a fixed bus");
  require(refined.paths.size() == 2 &&
              refined.paths[0].node_path == std::Vector<int>({0, 1, 2}) &&
              refined.paths[1].node_path == std::Vector<int>({3, 5}),
          "V22 must preserve the complete bus path while rerouting non-bus");
  require(refined.vline_mode_straight_by_group.at(7) &&
              refined.used_tob_switch_ids == std::Vector<int>({10}),
          "V22 must preserve bus mode and physical-switch configuration");
}

auto test_v22_post_sat_ilp_excludes_fixed_bus_nodes() -> void {
  auto graph = synthetic_graph(
      7, {{0, 1}, {1, 2}, {3, 4}, {4, 6}, {6, 5}, {3, 1}, {1, 5}});
  const auto bus_source = synthetic_track_ref(graph, 0, 0, 0, 0);
  (void)synthetic_track_ref(graph, 1, 0, 0, 1);
  const auto data_source = synthetic_track_ref(graph, 3, 0, 0, 3);
  (void)synthetic_track_ref(graph, 4, 0, 0, 4);
  (void)synthetic_track_ref(graph, 6, 0, 0, 6);

  auto bus = RoutingNet{};
  bus.net_id = 0;
  bus.kind = RoutingNetKind::Tnet;
  bus.is_sync_bus = true;
  bus.sources = {bus_source};
  bus.demands = {RoutingDemand{0, synthetic_ref(2), {0}, true}};

  auto data = RoutingNet{};
  data.net_id = 1;
  data.kind = RoutingNetKind::Tnet;
  data.sources = {data_source};
  data.demands = {RoutingDemand{0, synthetic_ref(5), {0}, true}};

  auto baseline = SatRoutingResult{};
  baseline.ok = true;
  baseline.paths = {SourceSinkPairPath{0, 0, 0, -1, {0, 1, 2}},
                    SourceSinkPairPath{1, 0, 0, -1, {3, 4, 6, 5}}};
  baseline.total_wirelength = total_wirelength(graph, baseline);

  const auto refined = optimize_post_sat_routes(
      graph, {bus, data},
      {synthetic_scope(graph, 0), synthetic_scope(graph, 1)}, baseline,
      PostSatIlpOptions{});
  const auto data_path = std::ranges::find_if(
      refined.paths, [](const auto &path) { return path.net_id == 1; });
  require(refined.post_sat_ilp_accepted,
          "a fixed bus node must be filtered before solving, not rejected "
          "only by post-validation");
  require(data_path != refined.paths.end() &&
              data_path->node_path == std::Vector<int>({3, 4, 6, 5}),
          "non-bus pair flow must not take a shorter path through a fixed "
          "SyncBus node");
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
