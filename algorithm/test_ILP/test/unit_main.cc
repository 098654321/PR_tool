#include "common/hw_map.hh"
#include "delay/pair_delay_precompute.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/ideal_shortest_wirelength.hh"
#include "sat/routing_path_log.hh"
#include "sat/routing_feedback.hh"
#include "sat/routing_round_diagnostics.hh"
#include "sat/routing_solution_validate.hh"
#include "sat/sat_constraint_kits.hh"
#include "sat/sat_encoding_stats.hh"
#include "sat/sat_solution_extract.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat/unified_sat_scope.hh"
#include "sat_allocation/cadical_solver.hh"
#include "scope/build_routing_nets.hh"
#include "scope/pair_routing_state.hh"
#include "scope/scope_bbox.hh"
#include "sat/solve_unified_sat.hh"
#include "test_ilp_cli.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <hardware/bump/bump.hh>
#include <hardware/tob/tob.hh>
#include <hardware/track/track.hh>
#include <parse/reader/module.hh>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <bit>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>

namespace {

using namespace PR_tool;

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
        "construction-only directed-arc dedup storage must be released before returning the graph");
    require(graph.rows == 9 && graph.cols == 12, "unified graph dimensions must be 9x12");
    require(graph.track_node_count == 30336, "unified graph must contain exactly 30,336 track nodes");
    require(graph.tob_node_count == 6144, "unified graph must contain exactly 6,144 TOB nodes");
    require(graph.nodes.size() == 36480, "unified graph must contain exactly 36,480 nodes");
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
            "every bidirectional physical connection must share its switch ID with its reverse arc");
    }
    for (const auto& [switch_id, arc_count] : physical_arc_count) {
        (void)switch_id;
        require(arc_count == 2, "each physical switch ID must identify exactly one reverse arc pair");
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
            "SyncNet fixed pair must reference its own normalized track source");
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
        "global encoding operations must sample at the 4096-operation boundary");

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
        "solve-time memory rejection must be distinguishable from ordinary UNKNOWN");
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

auto build_v14_model(
    CadicalSession& session,
    const UnifiedGraph& graph_in,
    const std::Vector<RoutingNet>& nets_in,
    SatEncodingStats* stats = nullptr,
    bool assume_pairs = true
) -> UnifiedSatModel {
    auto graph = graph_in;
    auto nets = nets_in;
    augment_graph_for_pnnet(graph, nets);
    const auto scopes = build_all_scopes(graph, nets);
    const auto delays = compute_pair_delays(graph, nets, scopes);
    auto model = build_unified_sat_model(session, graph, nets, scopes, delays, stats);
    if (assume_pairs) {
        for (const auto& alpha : model.alpha_vars) {
            session.assume(alpha.alpha_lit);
        }
    }
    return model;
}

auto d_lit_at(
    const UnifiedSatModel& model,
    std::size_t model_source_index,
    int node,
    int delay
) -> int {
    const auto& source = model.sources[model_source_index];
    const auto& scope = model.scopes[source.scope_index];
    const int node_offset = scope.node_offset[static_cast<std::size_t>(node)];
    if (node_offset < 0 || delay < 0 || delay > source.d_max) {
        return 0;
    }
    return source.d_var[static_cast<std::size_t>(node_offset)][static_cast<std::size_t>(delay)];
}

auto a_lit_at(
    const UnifiedSatModel& model,
    int arc_id,
    int delay,
    std::size_t model_source_index = 0
) -> int {
    for (const auto& tob_arc : model.tob_arcs) {
        if (tob_arc.arc_global_id == arc_id
            && tob_arc.model_source_index == model_source_index
            && delay <= tob_arc.d_max) {
            return tob_arc.a_var[static_cast<std::size_t>(delay)];
        }
    }
    return 0;
}

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
                        "shared successor must match conditional non-overflowing increment");
                    if (solved.ok) {
                        const auto modular_successor = (input_value + 1) % value_count;
                        for (std::size_t bit = 0; bit < width; ++bit) {
                            require(
                                session.value(successor.bits[bit])
                                    == ((modular_successor
                                         & (std::size_t {1} << bit))
                                        != 0),
                                "unconditional successor bits must equal input+1 modulo 2^w");
                        }
                        require(
                            session.value(successor.overflow)
                                == (input_value + 1 == value_count),
                            "successor overflow literal must identify the maximum input");
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
        "SAT must choose the reachable delay when another allowed delay is unreachable");
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
    expand_pair_delays(pair);
    require(
        pair.delays == std::Vector<int>({5, 6, 7}),
        "feedback expansion must append max+1 and max+2 to delays");

    auto state = RoutingProblemState {};
    state.pairs.push_back(pair);
    state.pair_index_by_key.emplace(pair.key, 0);
    state.pair_indices_by_net[0] = {0};
    const auto before = pair.pair_bbox;
    state.pairs[0].pair_bbox = expand_pair_bbox_one_cell(before);
    require(
        state.pairs[0].pair_bbox.row_min <= before.row_min
            && state.pairs[0].pair_bbox.row_max >= before.row_max,
        "feedback expansion must grow pair bbox by one cell per side");
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

    const auto status =
        apply_feedback_expansion(state, nets, {state.pairs[0].key});
    require(
        status == FeedbackExpansionStatus::Expanded,
        "reaching full-chip bbox must rebuild and solve once before exhaustion");
    require(
        is_full_chip_bbox(state.pairs[0].pair_bbox),
        "critical bbox must expand to full chip");
    require(
        state.pairs[0].delays == std::Vector<int>({2, 3, 4}),
        "critical delays must expand exactly once");
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

auto test_feedback_global_expand_skips_full_net_and_syncs_others() -> void {
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
        state.pairs[1].delays == state.pairs[2].delays,
        "global expansion must synchronize fanout delays");
    require(
        state.pairs[1].delays == std::Vector<int>({4, 5, 6, 7, 8}),
        "global fanout synchronization must merge expanded delay sets");
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
            "PNnet virtual source must expose the union of candidate track units");
        auto session = CadicalSession {};
        const auto model =
            build_unified_sat_model(session, graph, nets, scopes, delays);
        require(
            d_lit_at(model, 0, 0, 1) > 0
                && d_lit_at(model, 0, 1, 1) > 0,
            "PNnet active mask must retain every useful candidate source track");
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
            "Bnet ExactlyOne and A-to-Q clauses need a separate statistics category");
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
        "forcing one Bnet source through two different track units must be UNSAT");
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
            "sequential AMO truth table must allow exactly zero/one true literal");
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
        "extracted path wirelength must count bump hops only in synthetic graph");

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
        "reverse arcs and all pair uses must aggregate to one Y per physical switch");
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
            "numeric extraction must return exactly the used physical switch ID");
    };

    check_direction(0, 1, 0);
    check_direction(1, 0, 1);
}

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
        "extraction must select one valid predecessor when the D tree branches");
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
        "total wirelength must sum deduplicated bump and track resources per net");

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
        "net wirelength must deduplicate shared track nodes across fanout paths");
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
        "two-pin ideal wirelength must match bump+track count on shortest path");
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

} // namespace

auto main() -> int {
    try {
        test_bnet_geometry();
        test_tnet_geometry();
        test_pn_child_and_original_union();
        test_original_net_aggregation();
        test_sync_pairing_and_normalization();
        test_unified_graph_fixed_hardware_inventory();
        test_unified_graph_tob_connections();
        test_unified_graph_straight_swap_groups();
        test_unified_graph_boundary_scope();
        test_cadical_session_sat_and_counts();
        test_cadical_session_unsat();
        test_cadical_session_large_clause_sampling();
        test_cadical_session_state_guards();
        test_cadical_session_memory_limit();
        test_numeric_constraint_kits();
        test_binary_successor_truth_table();
        test_delay_precompute_simple_path();
        test_pair_state_initial_delays_bbox();
        test_delay_set_drives_d_max();
        test_cadical_assume_failed();
        test_alpha_implies_sink_d();
        test_alpha_gates_sink_connectivity();
        test_alpha_skips_unreachable_delay_for_sat();
        test_alpha_empty_exact_delay_reports_core();
        test_feedback_expands_delays_on_unsat();
        test_bus_member_delay_bbox_sync();
        test_feedback_rebuilds_after_reaching_full_bbox();
        test_feedback_exhausted_state_is_unchanged();
        test_feedback_global_expand_skips_full_net_and_syncs_others();
        test_bus_delay_takes_max_member();
        test_v14_d_var_exact_reachability_allocation();
        test_v14_unreachable_tob_arc_has_no_a_var();
        test_v14_irrelevant_base_states_are_not_allocated();
        test_v14_d_without_predecessor_is_not_allocated();
        test_v14_reverse_reachability_prunes_dead_branch();
        test_v14_feedback_delay_rebuilds_active_mask();
        test_v14_fanout_active_mask_unions_sinks();
        test_v14_source_unit_masks();
        test_v14_unit_mask_prunes_track_and_vline_states();
        test_v14_bnet_unit_selectors_only();
        test_v14_bnet_unit_selector_rejects_two_units();
        test_v14_pure_track_chain_sat();
        test_v14_pure_track_fork_sat();
        test_v14_pure_track_unreachable_unsat();
        test_v14_bus_equal_delay_equiv();
        test_v14_bus_forall_d_equiv();
        test_v14_bus_sync_covers_reachable_delay_domain();
        test_cli_max_rss_option();
        test_cli_initial_padding_options();
        test_initial_search_padding_scope();
        test_initial_search_padding_delay();
        test_initial_search_padding_rejects_delay_overflow();
        test_initial_search_padding_fanout_keeps_pair_delays_independent();
        test_initial_search_padding_bus_uses_shared_delay_without_bbox_merge();
        test_initial_search_padding_applies_scope_before_delay();
        test_sequential_at_most_one_truth_table_and_scaling();
        test_numeric_fixed_path_and_extraction();
        test_v14_rejects_multi_candidate_demand();
        test_initial_search_padding_pnnet_keeps_sink_delays_independent();
        test_pnnet_virtual_delay_shift();
        test_pnnet_forbid_track_delay_ne_1();
        test_pnnet_virtual_arc_sat_extract();
        test_logical_source_exclusivity();
        test_logical_source_owns_its_source_node();
        test_sat_encoding_stats_reconcile();
        test_source_distance_constants();
        test_mode_group_zero_conflict();
        test_all_mode_groups_and_group_zero_extraction();
        test_y_aggregation_and_partial_matching();
        test_all_four_y_partial_matching_sides();
        test_y_bidirectional_equivalence();
        test_forward_and_reverse_switch_use_extract_same_y();
        test_extraction_selects_one_valid_predecessor();
        test_extraction_handles_shared_source_fanout();
        test_extraction_error_branches();
        test_format_path_node_track_bump_hline_vline();
        test_infer_net_display_kind();
        test_format_bbox_corners();
        test_path_wirelength_counts_bump_and_track_only();
        test_format_path_hops_and_graph_node_ref();
        test_log_routing_paths_two_pin();
        test_validate_accepts_valid_two_pin();
        test_validate_detects_missing_arc();
        test_validate_detects_endpoint_mismatch();
        test_unique_failed_net_ids_deduplicates();
        test_ideal_two_pin_wirelength_matches_shortest_path();
        test_ideal_sync_bus_wirelength_scales_by_members();
        test_wirelength_matches_testlength_golden();
        std::cout << "test_ILP_unit: all tests passed\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "test_ILP_unit: " << error.what() << '\n';
        return 1;
    }
}
