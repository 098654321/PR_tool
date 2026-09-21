// Shared synthetic fixtures plus routing-net normalization, geometry, unified
// graph, and CaDiCaL session tests. Included by unit_main.cc inside its private
// namespace so helpers remain local to the unit-test executable.

auto require(bool condition, const std::string& message) -> void {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

auto require_bbox(
    const IlpBoundingBox& actual,
    std::i64 row_min,
    std::i64 row_max,
    std::i64 col_min,
    std::i64 col_max,
    const std::string& context
) -> void {
    require(
        actual.row_min == row_min && actual.row_max == row_max
            && actual.col_min == col_min && actual.col_max == col_max,
        context + ": expected (" + std::to_string(row_min) + "," + std::to_string(row_max)
            + "," + std::to_string(col_min) + "," + std::to_string(col_max)
            + "), got (" + std::to_string(actual.row_min) + "," + std::to_string(actual.row_max)
            + "," + std::to_string(actual.col_min) + "," + std::to_string(actual.col_max) + ")");
}

auto bump_ref(std::size_t tob) -> GraphNodeRef {
    GraphNodeRef ref {};
    ref.kind = GraphNodeRef::Kind::Bump;
    ref.bump = Bump_coord {tob, 0, 0, 0};
    return ref;
}

auto track_ref(std::i64 row, std::i64 col) -> GraphNodeRef {
    GraphNodeRef ref {};
    ref.kind = GraphNodeRef::Kind::Track;
    ref.track_coord = hardware::TrackCoord {
        row,
        col,
        hardware::TrackDirection::Vertical,
        0};
    return ref;
}

auto fixed_demand(std::size_t id, GraphNodeRef sink, std::size_t source_index = 0) -> RoutingDemand {
    return RoutingDemand {id, sink, std::Vector<std::size_t> {source_index}, true};
}

auto lookup_vline(const UnifiedGraph& graph, std::size_t tob, std::size_t global_v) -> int {
    const auto it = graph.vline_node_by_key.find(std::tuple {tob, global_v});
    return it == graph.vline_node_by_key.end() ? -1 : it->second;
}

auto lookup_track(
    const UnifiedGraph& graph,
    std::size_t unit,
    int dir,
    int row,
    int col,
    std::size_t track
) -> int {
    const auto it = graph.track_node_by_key.find(std::tuple {unit, dir, row, col, track});
    return it == graph.track_node_by_key.end() ? -1 : it->second;
}

auto find_arc(const UnifiedGraph& graph, int u, int v) -> const UnifiedArc* {
    for (const int arc_id : graph.out_arc_ids.at(static_cast<std::size_t>(u))) {
        const auto& arc = graph.arcs.at(static_cast<std::size_t>(arc_id));
        if (arc.v == v) {
            return &arc;
        }
    }
    return nullptr;
}

auto require_bidirectional_physical_connection(
    const UnifiedGraph& graph,
    int a,
    int b,
    PhysicalSwitchKind expected_kind,
    const std::string& context
) -> int {
    const auto* forward = find_arc(graph, a, b);
    const auto* reverse = find_arc(graph, b, a);
    require(forward != nullptr && reverse != nullptr, context + ": missing reverse arc pair");
    require(
        forward->physical_switch_id >= 0,
        context + ": physical switch ID must use -1, not 0, as the sentinel");
    require(
        forward->physical_switch_id == reverse->physical_switch_id,
        context + ": reverse arcs must share one physical switch ID");
    require(
        forward->physical_switch_kind == expected_kind
            && reverse->physical_switch_kind == expected_kind,
        context + ": unexpected physical switch kind");
    return forward->physical_switch_id;
}

auto test_unified_graph_fixed_hardware_inventory() -> void {
    const auto graph = build_unified_graph(nullptr, {});
    require(
        graph.directed_arc_set.empty(),
        "construction-only directed-arc dedup storage must be released "
            "before returning the graph");
    require(
        graph.rows == static_cast<int>(hardware::Interposer::COB_ARRAY_HEIGHT)
            && graph.cols == static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH),
        "unified graph dimensions must match the hardware inventory");
    const auto expected_track_nodes = std::size_t {128}
        * (static_cast<std::size_t>(graph.rows + 1) * static_cast<std::size_t>(graph.cols)
           + static_cast<std::size_t>(graph.rows) * static_cast<std::size_t>(graph.cols + 1));
    require(
        graph.track_node_count == expected_track_nodes,
        "unified graph track-node count must match the configured hardware "
            "dimensions");
    require(graph.tob_node_count == 6144, "unified graph must contain exactly 6,144 TOB nodes");
    require(
        graph.nodes.size() == expected_track_nodes + graph.tob_node_count,
        "unified graph total node count must equal track plus TOB nodes");
    require(graph.bump_node_by_key.size() == 16 * 128, "every TOB must contain all 128 bump nodes");
    require(graph.hline_node_by_key.size() == 16 * 128, "every TOB must contain all 128 h-line nodes");
    require(graph.vline_node_by_key.size() == 16 * 128, "every TOB must contain all 128 v-line nodes");

    auto per_tob = std::map<std::size_t, std::array<std::size_t, 3>> {};
    for (const auto& node : graph.nodes) {
        if (node.kind == UnifiedNodeKind::Bump) {
            per_tob[node.tob][0] += 1;
        }
        else if (node.kind == UnifiedNodeKind::HLine) {
            per_tob[node.tob][1] += 1;
        }
        else if (node.kind == UnifiedNodeKind::VLine) {
            per_tob[node.tob][2] += 1;
        }
    }
    require(per_tob.size() == 16, "unified graph must materialize all 16 TOBs");
    for (std::size_t tob = 0; tob < 16; ++tob) {
        require(
            per_tob.at(tob) == std::array<std::size_t, 3> {128, 128, 128},
            "each TOB must contain 128 bump, 128 h-line, and 128 v-line nodes");
        for (std::size_t global_v = 0; global_v < 128; ++global_v) {
            require(
                lookup_vline(graph, tob, global_v) >= 0,
                "v-line lookup must be keyed by (TOB, global v-line index)");
        }
    }

    RoutingNet active_net {};
    active_net.sources = {bump_ref(0)};
    const auto graph_with_net = build_unified_graph(nullptr, {active_net});
    require(
        graph_with_net.nodes.size() == graph.nodes.size()
            && graph_with_net.arcs.size() == graph.arcs.size(),
        "unified graph inventory must not depend on active nets");
}

auto test_unified_graph_tob_connections() -> void {
    const auto graph = build_unified_graph(nullptr, {});
    static_assert(std::is_signed_v<decltype(UnifiedArc {}.mode_group_id)>);
    static_assert(std::is_signed_v<decltype(UnifiedArc {}.physical_switch_id)>);

    const int bump = graph.bump_node_by_key.at(Bump_coord {0, 0, 0, 0});
    auto connected_hlines = std::set<int> {};
    for (const int arc_id : graph.out_arc_ids.at(static_cast<std::size_t>(bump))) {
        const auto& arc = graph.arcs.at(static_cast<std::size_t>(arc_id));
        if (graph.nodes.at(static_cast<std::size_t>(arc.v)).kind == UnifiedNodeKind::HLine) {
            connected_hlines.insert(arc.v);
        }
    }
    require(connected_hlines.size() == 8, "each bump must connect to all 8 h-lines in its group");
    for (std::size_t j = 0; j < 8; ++j) {
        const int hline = graph.hline_node_by_key.at(std::tuple {std::size_t {0}, std::size_t {0}, std::size_t {0}, j});
        require(connected_hlines.contains(hline), "bump connected to an h-line outside its bank/group");
        const int switch_id = require_bidirectional_physical_connection(
            graph,
            bump,
            hline,
            PhysicalSwitchKind::BumpH,
            "bump/h-line connection");
        if (j == 0) {
            require(switch_id == 0, "physical switch ID 0 must be a valid bump/h-line switch");
        }
    }
    const auto* bump_h_arc = find_arc(graph, bump, *connected_hlines.begin());
    require(
        bump_h_arc != nullptr && bump_h_arc->mode_group_id == -1,
        "non-mode physical connections must use mode-group sentinel -1");

    const int hline = graph.hline_node_by_key.at(
        std::tuple {std::size_t {0}, std::size_t {1}, std::size_t {3}, std::size_t {7}});
    auto connected_vlines = std::set<int> {};
    for (const int arc_id : graph.out_arc_ids.at(static_cast<std::size_t>(hline))) {
        const auto& arc = graph.arcs.at(static_cast<std::size_t>(arc_id));
        if (graph.nodes.at(static_cast<std::size_t>(arc.v)).kind == UnifiedNodeKind::VLine) {
            connected_vlines.insert(arc.v);
        }
    }
    require(connected_vlines.size() == 8, "each h-line must connect to 8 v-lines");
    for (std::size_t k = 0; k < 8; ++k) {
        const int vline = lookup_vline(graph, 0, 64 + 7 * 8 + k);
        require(connected_vlines.contains(vline), "h-line connected to the wrong global v-line group");
        (void)require_bidirectional_physical_connection(
            graph,
            hline,
            vline,
            PhysicalSwitchKind::HLineVLine,
            "h-line/v-line connection");
    }
}

auto test_unified_graph_straight_swap_groups() -> void {
    const auto graph = build_unified_graph(nullptr, {});
    const auto* wilton_arc = find_arc(
        graph,
        lookup_track(graph, map_track(0), 0, 0, 0, 0),
        lookup_track(graph, map_track(0), 0, 0, 1, 0));
    require(
        wilton_arc != nullptr
            && wilton_arc->physical_switch_id == -1
            && wilton_arc->physical_switch_kind == PhysicalSwitchKind::None,
        "non-TOB arcs must use physical-switch sentinel -1 and kind None");

    auto groups_by_tob = std::map<std::size_t, std::set<int>> {};
    for (const auto& arc : graph.arcs) {
        if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack) {
            const auto& u = graph.nodes.at(static_cast<std::size_t>(arc.u));
            const auto& v = graph.nodes.at(static_cast<std::size_t>(arc.v));
            const auto tob = u.kind == UnifiedNodeKind::VLine ? u.tob : v.tob;
            groups_by_tob[tob].insert(arc.mode_group_id);
        }
    }
    for (std::size_t tob = 0; tob < 16; ++tob) {
        require(groups_by_tob.at(tob).size() == 64, "each TOB must have exactly 64 straight/swap mode groups");
        require(
            groups_by_tob.at(tob).contains(static_cast<int>(tob * 64))
                && groups_by_tob.at(tob).contains(static_cast<int>(tob * 64 + 63)),
            "TOB mode-group IDs must be tob*64+k, including group 0");
    }

    for (const std::size_t tob : {std::size_t {0}, std::size_t {15}}) {
        const auto anchor = tob_anchor_cob(tob);
        for (const std::size_t k : {std::size_t {0}, std::size_t {63}}) {
            const int v0 = lookup_vline(graph, tob, k);
            const int v1 = lookup_vline(graph, tob, k + 64);
            const int t0 = lookup_track(
                graph,
                map_track(k),
                1,
                static_cast<int>(anchor.row),
                static_cast<int>(anchor.col),
                k);
            const int t1 = lookup_track(
                graph,
                map_track(k + 64),
                1,
                static_cast<int>(anchor.row),
                static_cast<int>(anchor.col),
                k + 64);
            require(v0 >= 0 && v1 >= 0 && t0 >= 0 && t1 >= 0, "representative TOB endpoints must exist");

            const auto check = [&](
                                   int vline,
                                   int track,
                                   bool straight,
                                   const std::string& context) {
                const auto* arc = find_arc(graph, vline, track);
                require(arc != nullptr, context + ": missing v-line/track arc");
                require(
                    arc->is_vline_track_straight == straight
                        && arc->is_vline_track_swap != straight,
                    context + ": wrong straight/swap classification");
                require(
                    arc->mode_group_id == static_cast<int>(tob * 64 + k),
                    context + ": wrong mode-group ID");
                (void)require_bidirectional_physical_connection(
                    graph,
                    vline,
                    track,
                    PhysicalSwitchKind::VLineTrack,
                    context);
            };
            check(v0, t0, true, "v0 straight");
            check(v1, t1, true, "v1 straight");
            check(v0, t1, false, "v0 swap");
            check(v1, t0, false, "v1 swap");
        }
    }

    auto physical_arc_count = std::map<int, std::size_t> {};
    for (const auto& arc : graph.arcs) {
        const int switch_id = arc.physical_switch_id;
        if (switch_id < 0) {
            continue;
        }
        physical_arc_count[switch_id] += 1;
        const auto* reverse = find_arc(graph, arc.v, arc.u);
        require(
            reverse != nullptr && reverse->physical_switch_id == switch_id,
            "every bidirectional physical connection must share its switch "
                "ID with its reverse arc");
    }
    for (const auto& [switch_id, arc_count] : physical_arc_count) {
        (void)switch_id;
        require(arc_count == 2, "each physical switch ID must identify exactly "
                                "one reverse arc pair");
    }
    require(
        physical_arc_count.size() == 36864,
        "full TOB graph must contain 36,864 distinct physical connections");
}

auto test_unified_graph_boundary_scope() -> void {
    const auto graph = build_unified_graph(nullptr, {});
    const auto one_cob = [](std::i64 row, std::i64 col) {
        return IlpBoundingBox {row, row, col, col};
    };

    const int horizontal_internal = lookup_track(graph, map_track(0), 0, 4, 6, 0);
    require(node_in_scope(graph, horizontal_internal, one_cob(4, 5)), "horizontal track must see its left COB");
    require(node_in_scope(graph, horizontal_internal, one_cob(4, 6)), "horizontal track must see its right COB");

    const int vertical_internal = lookup_track(graph, map_track(0), 1, 4, 6, 0);
    require(node_in_scope(graph, vertical_internal, one_cob(3, 6)), "vertical track must see its lower-index adjacent COB");
    require(node_in_scope(graph, vertical_internal, one_cob(4, 6)), "vertical track must see its higher-index adjacent COB");

    const int right_external = lookup_track(graph, map_track(0), 0, 4, graph.cols, 0);
    const int top_external = lookup_track(graph, map_track(0), 1, graph.rows, 6, 0);
    require(node_in_scope(graph, right_external, one_cob(4, graph.cols - 1)), "right external track must see the boundary COB");
    require(node_in_scope(graph, top_external, one_cob(graph.rows - 1, 6)), "top external track must see the boundary COB");

    const int tob0_bump = graph.bump_node_by_key.at(Bump_coord {0, 0, 0, 0});
    require(node_in_scope(graph, tob0_bump, one_cob(0, 0)), "TOB node must see the first adjacent COB");
    require(node_in_scope(graph, tob0_bump, one_cob(1, 0)), "TOB node must see the second adjacent COB");
    require(!node_in_scope(graph, tob0_bump, one_cob(2, 0)), "TOB node must not see a non-adjacent COB");
}

auto test_bnet_geometry() -> void {
    RoutingNet diagonal {};
    diagonal.kind = RoutingNetKind::Bnet;
    diagonal.sources = {bump_ref(0)};
    diagonal.demands = {fixed_demand(0, bump_ref(5))};
    require_bbox(compute_scope_bbox_for_net(diagonal), 1, 3, 0, 3, "Bnet diagonal");

    RoutingNet horizontal {};
    horizontal.kind = RoutingNetKind::Bnet;
    horizontal.sources = {bump_ref(4)};
    horizontal.demands = {fixed_demand(0, bump_ref(6))};
    require_bbox(compute_scope_bbox_for_net(horizontal), 2, 3, 0, 6, "Bnet horizontal");

    RoutingNet vertical {};
    vertical.kind = RoutingNetKind::Bnet;
    vertical.sources = {bump_ref(1)};
    vertical.demands = {fixed_demand(0, bump_ref(9))};
    require_bbox(compute_scope_bbox_for_net(vertical), 1, 5, 3, 3, "Bnet vertical");
}

auto test_tnet_geometry() -> void {
    auto check = [](
                     GraphNodeRef source,
                     GraphNodeRef sink,
                     IlpBoundingBox expected_child,
                     IlpBoundingBox expected_scope,
                     const std::string& context) {
        RoutingNet net {};
        net.kind = RoutingNetKind::Tnet;
        net.sources = {source};
        net.demands = {fixed_demand(0, sink)};
        const auto children = compute_scope_child_bboxes(net);
        require(children.size() == 1, context + ": expected one child bbox");
        require_bbox(
            children.front(),
            expected_child.row_min,
            expected_child.row_max,
            expected_child.col_min,
            expected_child.col_max,
            context + " child");
        require_bbox(
            compute_scope_bbox_for_net(net),
            expected_scope.row_min,
            expected_scope.row_max,
            expected_scope.col_min,
            expected_scope.col_max,
            context + " scope");
    };

    check(track_ref(6, 6), bump_ref(5), {3, 5, 3, 6}, {3, 5, 3, 6}, "Tnet diagonal above");
    check(track_ref(2, 6), bump_ref(5), {1, 2, 3, 6}, {1, 3, 3, 6}, "Tnet diagonal below");
    check(track_ref(6, 3), bump_ref(5), {3, 5, 3, 3}, {3, 5, 3, 3}, "Tnet same column above");
    check(track_ref(2, 3), bump_ref(5), {1, 2, 3, 3}, {1, 3, 3, 3}, "Tnet same column below");

    check(
        track_ref(6, 3),
        bump_ref(7),
        {3, 5, 3, 9},
        {3, 5, 3, 9},
        "Tnet TOB column 3 uses anchor COB column");
}

auto test_pn_child_and_original_union() -> void {
    RoutingNet net {};
    net.kind = RoutingNetKind::PNnet;
    net.sources = {track_ref(6, 0), track_ref(2, 9)};
    net.demands = {
        RoutingDemand {0, bump_ref(5), {0, 1}, false},
        RoutingDemand {1, bump_ref(10), {0, 1}, false}};

    const auto children = compute_scope_child_bboxes(net);
    require(children.size() == 2, "PNnet should have one bbox child per demand");
    const auto demand0 = compute_pnnet_demand_pair_bbox(net, 0);
    const auto demand1 = compute_pnnet_demand_pair_bbox(net, 1);
    require_bbox(children[0], demand0.row_min, demand0.row_max, demand0.col_min, demand0.col_max, "PNnet demand 0 pair union");
    require_bbox(children[1], demand1.row_min, demand1.row_max, demand1.col_min, demand1.col_max, "PNnet demand 1 pair union");
    require_bbox(compute_scope_bbox_for_net(net), 1, 5, 0, 9, "PNnet original child union");
}

auto test_original_net_aggregation() -> void {
    hardware::TOB tob0 {0, 0};
    hardware::TOB tob1 {1, 1};
    hardware::Bump bump0 {hardware::BumpCoord {0, 0, 0}, &tob0};
    hardware::Bump bump1 {hardware::BumpCoord {0, 0, 1}, &tob1};
    hardware::Track track0 {6, 0, hardware::TrackDirection::Vertical, 4};
    hardware::Track track1 {2, 9, hardware::TrackDirection::Vertical, 7};

    std::String tbs_name {"aggregate_tbs"};
    std::String tbs_uid {"aggregate_tbs_uid"};
    auto tbs = std::make_shared<circuit::TrackToBumpsNet>(
        &track0,
        std::Vector<hardware::Bump*> {&bump0, &bump1},
        std::HashSet<int> {0},
        tbs_name,
        tbs_uid);
    auto tbs_result = build_routing_nets({tbs});
    require(tbs_result.size() == 1, "TrackToBumpsNet must remain one RoutingNet");
    require(tbs_result[0].sources.size() == 1, "TrackToBumpsNet must keep one source");
    require(tbs_result[0].demands.size() == 2, "TrackToBumpsNet must have one demand per bump");
    require(tbs_result[0].demands[0].fixed_pair, "TrackToBumpsNet demands must be fixed");
    require(
        tbs_result[0].demands[1].candidate_source_indices == std::Vector<std::size_t> {0},
        "TrackToBumpsNet fixed demand must reference source 0");

    std::String tsbs_name {"aggregate_tsbs"};
    std::String tsbs_uid {"aggregate_tsbs_uid"};
    auto tsbs = std::make_shared<circuit::TracksToBumpsNet>(
        std::Vector<hardware::Track*> {&track0, &track1},
        std::Vector<hardware::Bump*> {&bump0, &bump1},
        std::HashSet<int> {0},
        tsbs_name,
        tsbs_uid);
    auto tsbs_result = build_routing_nets({tsbs});
    require(tsbs_result.size() == 1, "TracksToBumpsNet must remain one RoutingNet");
    require(tsbs_result[0].kind == RoutingNetKind::PNnet, "TracksToBumpsNet must map to PNnet");
    require(tsbs_result[0].sources.size() == 2, "TracksToBumpsNet must keep all candidate tracks");
    require(tsbs_result[0].demands.size() == 2, "TracksToBumpsNet must have one demand per bump");
    require(
        tsbs_result[0].demands[0].candidate_source_indices.size() == 2,
        "TracksToBumpsNet demands must list all candidate sources");
    validate_v14_routing_nets(tsbs_result);
}

auto test_sync_pairing_and_normalization() -> void {
    hardware::TOB tob0 {0, 0};
    hardware::TOB tob1 {1, 1};
    hardware::Bump bump0 {hardware::BumpCoord {0, 0, 0}, &tob0};
    hardware::Bump bump1 {hardware::BumpCoord {0, 0, 1}, &tob1};
    hardware::Track track0 {6, 3, hardware::TrackDirection::Vertical, 4};
    hardware::Track track1 {2, 9, hardware::TrackDirection::Vertical, 7};

    std::String member0_name {"sync_member_0"};
    std::String member0_uid {"sync_member_0_uid"};
    std::String member1_name {"sync_member_1"};
    std::String member1_uid {"sync_member_1_uid"};
    auto member0 = std::make_shared<circuit::BumpToTrackNet>(
        &bump0,
        &track0,
        std::HashSet<int> {0},
        member0_name,
        member0_uid);
    auto member1 = std::make_shared<circuit::BumpToTrackNet>(
        &bump1,
        &track1,
        std::HashSet<int> {0},
        member1_name,
        member1_uid);

    std::String sync_name {"SyncNet in group 2"};
    std::String sync_uid {"sync_uid"};
    auto sync = std::make_shared<circuit::SyncNet>(
        std::Vector<std::Rc<circuit::BumpToBumpNet>> {},
        std::Vector<std::Rc<circuit::BumpToTrackNet>> {member0, member1},
        std::Vector<std::Rc<circuit::TrackToBumpNet>> {},
        std::HashSet<int> {0},
        sync_name,
        sync_uid);
    const auto result = build_routing_nets({sync});
    require(result.size() == 1, "SyncNet must remain one RoutingNet");
    require(result[0].is_sync_bus, "SyncNet must retain sync-bus metadata");
    require(result[0].sources.size() == 2, "SyncNet must retain distinct physical track sources");
    require(result[0].sources[0].kind == GraphNodeRef::Kind::Track, "BumpToTrack must normalize to track source");
    require(result[0].demands.size() == 2, "SyncNet must retain every member demand");
    for (std::size_t i = 0; i < result[0].demands.size(); ++i) {
        const auto& demand = result[0].demands[i];
        require(demand.fixed_pair, "SyncNet member pairings must remain fixed");
        require(
            demand.candidate_source_indices == std::Vector<std::size_t> {i},
            "SyncNet fixed pair must reference its own normalized track "
                "source");
        require(demand.sink.kind == GraphNodeRef::Kind::Bump, "BumpToTrack must normalize to bump sink");
    }

    std::String btb_name {"mixed_btb"};
    std::String btb_uid {"mixed_btb_uid"};
    auto btb = std::make_shared<circuit::BumpToBumpNet>(
        &bump0,
        &bump1,
        std::HashSet<int> {0},
        btb_name,
        btb_uid);
    std::String mixed_name {"mixed_sync"};
    std::String mixed_uid {"mixed_sync_uid"};
    auto mixed = std::make_shared<circuit::SyncNet>(
        std::Vector<std::Rc<circuit::BumpToBumpNet>> {btb},
        std::Vector<std::Rc<circuit::BumpToTrackNet>> {member0},
        std::Vector<std::Rc<circuit::TrackToBumpNet>> {},
        std::HashSet<int> {0},
        mixed_name,
        mixed_uid);
    try {
        (void)build_routing_nets({mixed});
        require(false, "mixed Bnet/Tnet SyncNet must be rejected");
    }
    catch (const std::runtime_error& error) {
        require(
            std::string {error.what()}.find("mixed Bnet/Tnet SyncNet 'mixed_sync'") != std::string::npos,
            "mixed SyncNet error must include its type and name");
    }
}

auto test_cadical_session_sat_and_counts() -> void {
    auto session = CadicalSession {};
    const int x = session.new_var();
    session.add_clause({x});
    const auto clause = std::Vector<int> {x};
    session.add_clause(clause);
    session.add_clause(std::span<const int> {clause});

    require(session.num_vars() == 1, "CaDiCaL session must count variables");
    require(session.num_clauses() == 3, "CaDiCaL session must count every clause input form");

    const auto result = session.solve_once();
    require(result.ok && result.message == "SAT", "unit clause x must be satisfiable");
    require(session.value(x), "SAT model must assign x=true");
}

auto test_cadical_session_unsat() -> void {
    auto session = CadicalSession {};
    const int x = session.new_var();
    session.add_clause({x});
    session.add_clause(std::Vector<int> {-x});

    const auto result = session.solve_once();
    require(!result.ok && result.message == "UNSAT", "x and !x must be unsatisfiable");
    require(session.num_vars() == 1, "UNSAT session variable count must remain exact");
    require(session.num_clauses() == 2, "UNSAT session clause count must remain exact");
}

auto test_cadical_session_large_clause_sampling() -> void {
    auto options = CadicalDiagnosticsOptions {};
    options.max_rss_mb = 1'000'000;
    auto session = CadicalSession {options};
    const int x = session.new_var();
    for (std::size_t i = 0; i < 4095; ++i) {
        session.add_clause({x});
    }
    require(
        session.encoding_memory_sample_count() == 1,
        "small clauses must not trigger a getrusage sample after every clause");
    session.add_clause({x});
    require(
        session.encoding_memory_sample_count() == 2,
        "global encoding operations must sample at the 4096-operation "
            "boundary");

    const auto clause = std::Vector<int>(4097, x);
    session.add_clause(clause);

    require(
        session.encoding_memory_sample_count() == 3,
        "a huge clause must sample after each 4096 inserted literals");
    require(session.num_clauses() == 4097, "large clause must be terminated and counted once");
    const auto result = session.solve_once();
    require(result.ok && session.value(x), "large clause sampling must preserve solver state");
}

auto test_cadical_session_state_guards() -> void {
    auto session = CadicalSession {};
    const int x = session.new_var();
    session.add_clause({x});

    try {
        (void)session.value(x);
        require(false, "value() before a SAT solve must fail");
    }
    catch (const std::logic_error&) {
    }

    const auto result = session.solve_once();
    require(result.ok, "state guard fixture must be satisfiable");
    try {
        (void)session.solve_once();
        require(false, "solve_once() must reject a second call");
    }
    catch (const std::logic_error&) {
    }
}

auto test_cadical_session_memory_limit() -> void {
    auto options = CadicalDiagnosticsOptions {};
    options.max_rss_mb = 1;
    auto session = CadicalSession {options};
    try {
        (void)session.new_var();
        require(false, "1 MB RSS limit must fail on the first encoding operation");
    }
    catch (const MemoryLimitExceeded& error) {
        require(
            std::string {error.what()} == "MEMORY_LIMIT",
            "memory limit exception must use the stable MEMORY_LIMIT message");
    }
    require(session.memory_limit_exceeded(), "session must retain the memory-limit flag");
    require(session.peak_rss_mb() > 1, "reported peak RSS must exceed the 1 MB limit");

    auto solve_session = CadicalSession {options};
    const auto result = solve_session.solve_once();
    require(
        !result.ok && result.memory_limit_exceeded && result.message == "MEMORY_LIMIT",
        "solve-time memory rejection must be distinguishable from ordinary "
            "UNKNOWN");
}

auto synthetic_graph(std::size_t node_count, const std::Vector<std::pair<int, int>>& edges) -> UnifiedGraph {
    auto graph = UnifiedGraph {};
    graph.rows = 9;
    graph.cols = 12;
    graph.nodes.resize(node_count);
    graph.in_arc_ids.resize(node_count);
    graph.out_arc_ids.resize(node_count);
    for (std::size_t i = 0; i < node_count; ++i) {
        graph.nodes[i].kind = UnifiedNodeKind::Bump;
        graph.nodes[i].bump = Bump_coord {0, 0, 0, i};
        graph.bump_node_by_key.emplace(graph.nodes[i].bump, static_cast<int>(i));
    }
    for (const auto [u, v] : edges) {
        const int arc_id = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back(UnifiedArc {u, v});
        graph.out_arc_ids[static_cast<std::size_t>(u)].push_back(arc_id);
        graph.in_arc_ids[static_cast<std::size_t>(v)].push_back(arc_id);
    }
    return graph;
}

auto synthetic_track_ref(UnifiedGraph &graph, int node_id, std::size_t unit,
                         int row, int col, std::size_t track_index = 0)
    -> GraphNodeRef {
    auto &node = graph.nodes.at(static_cast<std::size_t>(node_id));
    graph.bump_node_by_key.erase(node.bump);
    node.kind = UnifiedNodeKind::Track;
    node.unit = unit;
    node.track_dir = static_cast<int>(hardware::TrackDirection::Horizontal);
    node.track_row = row;
    node.track_col = col;
    node.track_index = track_index;
    graph.track_node_by_key.emplace(
        std::tuple{unit, node.track_dir, row, col, track_index}, node_id);

    auto ref = GraphNodeRef{};
    ref.kind = GraphNodeRef::Kind::Track;
    ref.track_coord = hardware::TrackCoord{
        row, col, hardware::TrackDirection::Horizontal, track_index};
    ref.track_index = track_index;
    return ref;
}

auto synthetic_ref(std::size_t node) -> GraphNodeRef {
    GraphNodeRef ref {};
    ref.kind = GraphNodeRef::Kind::Bump;
    ref.bump = Bump_coord {0, 0, 0, node};
    return ref;
}

auto synthetic_net(
    std::size_t net_id,
    const std::Vector<std::size_t>& sources,
    const std::Vector<std::pair<std::size_t, std::Vector<std::size_t>>>& demands
) -> RoutingNet {
    auto net = RoutingNet {};
    net.net_id = net_id;
    net.has_scope_bbox = true;
    net.scope_bbox = IlpBoundingBox {0, 8, 0, 11};
    for (const auto source : sources) {
        net.sources.push_back(synthetic_ref(source));
    }
    for (std::size_t i = 0; i < demands.size(); ++i) {
        net.demands.push_back(RoutingDemand {
            i,
            synthetic_ref(demands[i].first),
            demands[i].second,
            demands[i].second.size() == 1});
    }
    return net;
}

auto synthetic_scope(const UnifiedGraph& graph, std::size_t net_id) -> UnifiedSatNetScope {
    auto scope = UnifiedSatNetScope {};
    scope.net_id = net_id;
    scope.node_offset.assign(graph.nodes.size(), -1);
    scope.arc_offset.assign(graph.arcs.size(), -1);
    for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
        scope.node_offset[i] = static_cast<int>(scope.node_ids.size());
        scope.node_ids.push_back(static_cast<int>(i));
    }
    for (std::size_t i = 0; i < graph.arcs.size(); ++i) {
        scope.arc_offset[i] = static_cast<int>(scope.arc_ids.size());
        scope.arc_ids.push_back(static_cast<int>(i));
    }
    return scope;
}

auto synthetic_scope(const UnifiedGraph &graph, std::size_t net_id,
                     const std::Vector<int> &node_ids,
                     const std::Vector<int> &arc_ids) -> UnifiedSatNetScope {
    auto scope = UnifiedSatNetScope{};
    scope.net_id = net_id;
    scope.node_offset.assign(graph.nodes.size(), -1);
    scope.arc_offset.assign(graph.arcs.size(), -1);
    for (const int node_id : node_ids) {
        scope.node_offset.at(static_cast<std::size_t>(node_id)) =
            static_cast<int>(scope.node_ids.size());
        scope.node_ids.push_back(node_id);
    }
    for (const int arc_id : arc_ids) {
        scope.arc_offset.at(static_cast<std::size_t>(arc_id)) =
            static_cast<int>(scope.arc_ids.size());
        scope.arc_ids.push_back(arc_id);
    }
    return scope;
}

auto synthetic_channel_graph(
    const std::Vector<GlobalChannelCoord>& channels,
    const std::Vector<GlobalRouteNode>& nodes,
    const std::Vector<std::tuple<int, int, int>>& segments
) -> GlobalChannelGraph {
    auto graph = GlobalChannelGraph {};
    graph.channels = channels;
    graph.nodes = nodes;
    graph.in_arc_ids.resize(nodes.size());
    graph.out_arc_ids.resize(nodes.size());
    graph.arc_ids_by_channel.resize(channels.size());
    graph.adjacent_channel_ids.resize(channels.size());
    for (std::size_t index = 0; index < channels.size(); ++index) {
        graph.channel_id_by_coord.emplace(channels[index], static_cast<int>(index));
    }
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        const auto& node = nodes[index];
        if (node.kind == GlobalRouteNodeKind::Cob) {
            graph.cob_node_by_coord.emplace(std::pair {node.row, node.col}, index);
            ++graph.cob_node_count;
        }
        else if (node.kind == GlobalRouteNodeKind::TobTerminal) {
            graph.tob_node_by_tob.emplace(node.tob, index);
            ++graph.tob_terminal_node_count;
        }
        else if (node.kind == GlobalRouteNodeKind::PortTerminal) {
            graph.port_node_by_key.emplace(node.port, index);
            ++graph.port_terminal_node_count;
        }
        else {
            graph.boundary_node_by_channel.emplace(node.channel, index);
            ++graph.boundary_terminal_node_count;
        }
    }
    const auto add_arc = [&](const int u, const int v, const int channel, const int port) {
        const int arc = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back(GlobalChannelArc {u, v, channel, port});
        graph.out_arc_ids[static_cast<std::size_t>(u)].push_back(arc);
        graph.in_arc_ids[static_cast<std::size_t>(v)].push_back(arc);
        graph.arc_ids_by_channel[static_cast<std::size_t>(channel)].push_back(arc);
    };
    auto incident_channels = std::Vector<std::set<int>>(nodes.size());
    for (const auto [u, v, channel] : segments) {
        int port = -1;
        if (nodes[static_cast<std::size_t>(u)].kind == GlobalRouteNodeKind::PortTerminal) {
            port = u;
        }
        if (nodes[static_cast<std::size_t>(v)].kind == GlobalRouteNodeKind::PortTerminal) {
            port = v;
        }
        add_arc(u, v, channel, port);
        add_arc(v, u, channel, port);
        if (nodes[static_cast<std::size_t>(u)].kind == GlobalRouteNodeKind::Cob) {
            incident_channels[static_cast<std::size_t>(u)].insert(channel);
        }
        if (nodes[static_cast<std::size_t>(v)].kind == GlobalRouteNodeKind::Cob) {
            incident_channels[static_cast<std::size_t>(v)].insert(channel);
        }
    }
    for (const auto& incident : incident_channels) {
        for (const int lhs : incident) {
            for (const int rhs : incident) {
                if (lhs != rhs) {
                    graph.adjacent_channel_ids[static_cast<std::size_t>(lhs)].push_back(rhs);
                }
            }
        }
    }
    for (auto& adjacent : graph.adjacent_channel_ids) {
        std::sort(adjacent.begin(), adjacent.end());
        adjacent.erase(std::unique(adjacent.begin(), adjacent.end()), adjacent.end());
    }
    return graph;
}

auto bump_ref(std::size_t tob, std::size_t bank, std::size_t index) -> GraphNodeRef {
    auto ref = GraphNodeRef {};
    ref.kind = GraphNodeRef::Kind::Bump;
    ref.bump = Bump_coord {tob, bank, 0, index};
    return ref;
}

auto track_ref(
    int row,
    int col,
    hardware::TrackDirection direction,
    std::size_t track
) -> GraphNodeRef {
    auto ref = GraphNodeRef {};
    ref.kind = GraphNodeRef::Kind::Track;
    ref.track_coord = hardware::TrackCoord {row, col, direction, track};
    ref.track_index = track;
    return ref;
}

auto global_cob_node(int row = 0, int col = 0) -> GlobalRouteNode {
    auto node = GlobalRouteNode {};
    node.kind = GlobalRouteNodeKind::Cob;
    node.row = row;
    node.col = col;
    return node;
}

auto global_tob_node(std::size_t tob, int channel) -> GlobalRouteNode {
    auto node = GlobalRouteNode {};
    node.kind = GlobalRouteNodeKind::TobTerminal;
    node.tob = tob;
    node.channel = channel;
    return node;
}

auto global_port_node(const GraphNodeRef& ref, int channel) -> GlobalRouteNode {
    auto node = GlobalRouteNode {};
    node.kind = GlobalRouteNodeKind::PortTerminal;
    node.channel = channel;
    node.port = GlobalPortKey {
        ref.track_coord.dir == hardware::TrackDirection::Horizontal ? 0 : 1,
        static_cast<int>(ref.track_coord.row),
        static_cast<int>(ref.track_coord.col),
        ref.track_index};
    return node;
}

auto two_pin_net(
    std::size_t net_id,
    RoutingNetKind kind,
    const GraphNodeRef& source,
    const GraphNodeRef& sink
) -> RoutingNet {
    auto net = RoutingNet {};
    net.net_id = net_id;
    net.kind = kind;
    net.sources = {source};
    net.demands = {RoutingDemand {0, sink, {0}, true}};
    return net;
}

auto pn_preselection_net(
    std::size_t net_id,
    const std::Vector<GraphNodeRef>& sources,
    const std::Vector<GraphNodeRef>& sinks
) -> RoutingNet {
    auto net = RoutingNet {};
    net.net_id = net_id;
    net.name = "synthetic-PN";
    net.origin_uid = "synthetic-PN";
    net.origin_key = "synthetic-PN";
    net.kind = RoutingNetKind::PNnet;
    net.sources = sources;
    auto candidates = std::Vector<std::size_t> {};
    for (std::size_t source = 0; source < sources.size(); ++source) {
        candidates.push_back(source);
    }
    for (std::size_t demand = 0; demand < sinks.size(); ++demand) {
        net.demands.push_back(RoutingDemand {demand, sinks[demand], candidates, false});
    }
    return net;
}
