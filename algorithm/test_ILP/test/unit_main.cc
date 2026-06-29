#include "common/hw_map.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/sat_constraint_kits.hh"
#include "sat/sat_solution_extract.hh"
#include "sat/unified_sat_encoder.hh"
#include "sat_allocation/cadical_solver.hh"
#include "scope/build_routing_nets.hh"
#include "scope/scope_bbox.hh"
#include "test_ilp_cli.hh"

#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <hardware/bump/bump.hh>
#include <hardware/tob/tob.hh>
#include <hardware/track/track.hh>

#include <iostream>
#include <bit>
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
    require(children.size() == 2, "PNnet should have one bbox child per source");
    require_bbox(children[0], 3, 5, 0, 6, "PNnet source 0 child union");
    require_bbox(children[1], 1, 4, 3, 9, "PNnet source 1 child union");
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
    require(tsbs_result[0].sources.size() == 2, "TracksToBumpsNet must keep all unique sources");
    require(tsbs_result[0].demands.size() == 2, "TracksToBumpsNet must have one demand per bump");
    for (const auto& demand : tsbs_result[0].demands) {
        require(!demand.fixed_pair, "TracksToBumpsNet demands must allow source choice");
        require(
            demand.candidate_source_indices == std::Vector<std::size_t> {0, 1},
            "TracksToBumpsNet demands must list every source candidate");
    }
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

auto test_sync_bus_successor_is_shared_per_node() -> void {
    const auto graph = synthetic_graph(
        4, {{0, 1}, {0, 2}, {1, 2}, {1, 3}, {2, 3}});
    auto ordinary = synthetic_net(0, {0}, {{3, {0}}});
    auto ordinary_session = CadicalSession {};
    (void)build_unified_sat_model(ordinary_session, graph, {ordinary});

    ordinary.is_sync_bus = true;
    auto bus_session = CadicalSession {};
    (void)build_unified_sat_model(bus_session, graph, {ordinary});

    constexpr std::size_t width = 3;
    constexpr std::size_t node_count = 4;
    const std::size_t expected_extra_vars =
        node_count * width + node_count * (2 * width - 1);
    require(
        bus_session.num_vars() - ordinary_session.num_vars() == expected_extra_vars,
        "Sync distance encoding must allocate one distance and one successor circuit per node");
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
    const auto model = build_unified_sat_model(session, graph, nets);
    const auto solved = session.solve_once();
    require(solved.ok, "connected fixed demand must be SAT");
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(result.ok && result.paths.size() == 1, "SAT extraction must return every demand");
    require(result.paths[0].demand_id == 0, "extraction must preserve demand_id");
    require(
        result.paths[0].node_path == std::Vector<int>({0, 1, 2}),
        "extraction must follow the unique true x chain");

    const auto disconnected = synthetic_graph(3, {});
    auto disconnected_session = CadicalSession {};
    (void)build_unified_sat_model(disconnected_session, disconnected, nets);
    require(!disconnected_session.solve_once().ok, "disconnected fixed demand must be UNSAT");
}

auto test_candidate_pairs_and_forced_flow_amo() -> void {
    const auto graph = synthetic_graph(4, {{0, 2}, {1, 2}, {2, 3}, {0, 3}});
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(1, {0, 1}, {{3, {0, 1}}})};
    auto session = CadicalSession {};
    const auto model = build_unified_sat_model(session, graph, nets);
    require(model.pairs.size() == 2, "candidate pairs must come only from demand candidate indices");
    const auto solved = session.solve_once();
    require(solved.ok, "one of two candidate sources must route");
    int active = 0;
    for (const auto& pair : model.pairs) {
        active += session.value(pair.activation) ? 1 : 0;
    }
    require(active == 1, "demand candidate activations must be ExactlyOne");

    auto amo_session = CadicalSession {};
    const auto amo_model = build_unified_sat_model(amo_session, graph, {
        synthetic_net(2, {0}, {{3, {0}}})});
    const auto& pair = amo_model.pairs.front();
    amo_session.add_clause({pair.x_vars[0]});
    amo_session.add_clause({pair.x_vars[3]});
    require(!amo_session.solve_once().ok, "forcing two source outgoing arcs must violate AMO");

    const auto sink_graph = synthetic_graph(
        4, {{0, 1}, {0, 2}, {1, 3}, {2, 3}});
    auto sink_session = CadicalSession {};
    const auto sink_model = build_unified_sat_model(
        sink_session, sink_graph, {synthetic_net(3, {0}, {{3, {0}}})});
    sink_session.add_clause({sink_model.pairs.front().x_vars[2]});
    sink_session.add_clause({sink_model.pairs.front().x_vars[3]});
    require(!sink_session.solve_once().ok, "forcing two sink incoming arcs must violate AMO");

    const auto intermediate_in_graph = synthetic_graph(
        6, {{0, 5}, {0, 2}, {0, 3}, {2, 1}, {3, 1}, {1, 4}, {4, 2}});
    auto intermediate_in_session = CadicalSession {};
    const auto intermediate_in_model = build_unified_sat_model(
        intermediate_in_session,
        intermediate_in_graph,
        {synthetic_net(4, {0}, {{5, {0}}})});
    const auto& intermediate_in_pair = intermediate_in_model.pairs.front();
    intermediate_in_session.add_clause({intermediate_in_pair.x_vars[3]});
    intermediate_in_session.add_clause({intermediate_in_pair.x_vars[4]});
    require(
        !intermediate_in_session.solve_once().ok,
        "forcing two incoming arcs at an intermediate node must violate AMO");

    const auto intermediate_out_graph = synthetic_graph(
        6, {{0, 5}, {0, 4}, {4, 1}, {1, 2}, {1, 3}, {2, 4}, {3, 4}});
    auto intermediate_out_session = CadicalSession {};
    const auto intermediate_out_model = build_unified_sat_model(
        intermediate_out_session,
        intermediate_out_graph,
        {synthetic_net(5, {0}, {{5, {0}}})});
    const auto& intermediate_out_pair = intermediate_out_model.pairs.front();
    intermediate_out_session.add_clause({intermediate_out_pair.x_vars[3]});
    intermediate_out_session.add_clause({intermediate_out_pair.x_vars[4]});
    require(
        !intermediate_out_session.solve_once().ok,
        "forcing two outgoing arcs at an intermediate node must violate AMO");
}

auto test_logical_source_exclusivity() -> void {
    const auto graph = synthetic_graph(3, {{0, 2}, {1, 2}});
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}}),
        synthetic_net(1, {1}, {{2, {0}}})};
    auto session = CadicalSession {};
    (void)build_unified_sat_model(session, graph, nets);
    require(!session.solve_once().ok, "distinct logical sources must be exclusive at a shared node");
}

auto test_logical_source_owns_its_source_node() -> void {
    const auto graph = synthetic_graph(2, {{0, 1}});
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{1, {0}}})};

    auto value_session = CadicalSession {};
    const auto value_model = build_unified_sat_model(value_session, graph, nets);
    const int source_p = value_model.logical_sources[0].p_vars[
        static_cast<std::size_t>(value_model.scopes[0].node_offset[0])];
    const auto solved = value_session.solve_once();
    require(
        solved.ok && value_session.value(source_p),
        "logical P(source, source-node) must be true in every SAT model");

    auto forced_false_session = CadicalSession {};
    const auto forced_false_model =
        build_unified_sat_model(forced_false_session, graph, nets);
    const int forced_false_p = forced_false_model.logical_sources[0].p_vars[
        static_cast<std::size_t>(forced_false_model.scopes[0].node_offset[0])];
    forced_false_session.add_clause({-forced_false_p});
    require(
        !forced_false_session.solve_once().ok,
        "forcing logical P(source, source-node)=false must be UNSAT");
}

auto test_source_incoming_and_sink_outgoing_are_zero() -> void {
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}})};

    const auto source_in_graph = synthetic_graph(3, {{0, 2}, {1, 0}});
    auto source_in_session = CadicalSession {};
    const auto source_in_model =
        build_unified_sat_model(source_in_session, source_in_graph, nets);
    require(
        source_in_session.num_clauses() == 23,
        "source-incoming fixture must include its explicit zero unit clause");
    source_in_session.add_clause({source_in_model.pairs[0].x_vars[1]});
    require(
        !source_in_session.solve_once().ok,
        "forcing a selected arc into the pair source must be UNSAT");

    const auto sink_out_graph = synthetic_graph(3, {{0, 2}, {2, 1}});
    auto sink_out_session = CadicalSession {};
    const auto sink_out_model =
        build_unified_sat_model(sink_out_session, sink_out_graph, nets);
    require(
        sink_out_session.num_clauses() == 23,
        "sink-outgoing fixture must include its explicit zero unit clause");
    sink_out_session.add_clause({sink_out_model.pairs[0].x_vars[1]});
    require(
        !sink_out_session.solve_once().ok,
        "forcing a selected arc out of the pair sink must be UNSAT");
}

auto test_mode_group_zero_conflict() -> void {
    auto graph = synthetic_graph(4, {{0, 2}, {1, 3}});
    graph.arcs[0].mode_group_id = 0;
    graph.arcs[0].is_vline_track_straight = true;
    graph.arcs[1].mode_group_id = 0;
    graph.arcs[1].is_vline_track_swap = true;
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}}),
        synthetic_net(1, {1}, {{3, {0}}})};
    auto session = CadicalSession {};
    (void)build_unified_sat_model(session, graph, nets);
    require(!session.solve_once().ok, "group zero straight/swap uses must conflict");
}

auto test_all_mode_groups_and_group_zero_extraction() -> void {
    auto graph = synthetic_graph(2, {{0, 1}});
    graph.arcs[0].mode_group_id = 0;
    graph.arcs[0].is_vline_track_straight = true;
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{1, {0}}})};
    auto session = CadicalSession {};
    const auto model = build_unified_sat_model(session, graph, nets);
    require(
        model.mode_var_by_group.size() == 16 * 64,
        "model must allocate all 1024 global VLineTrack mode groups");
    const auto solved = session.solve_once();
    require(solved.ok, "single group-zero straight path must be SAT");
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(
        result.ok && result.vline_mode_straight_by_group.size() == 16 * 64,
        "extraction must return all 1024 mode states");
    require(
        result.vline_mode_straight_by_group.at(0),
        "a used group-zero straight arc must extract M_0=true");
}

auto test_sync_bus_binary_distance() -> void {
    const auto equal_graph = synthetic_graph(6, {{0, 1}, {1, 2}, {3, 4}, {4, 5}});
    auto equal_net = synthetic_net(0, {0, 3}, {{2, {0}}, {5, {1}}});
    equal_net.is_sync_bus = true;
    auto equal_session = CadicalSession {};
    (void)build_unified_sat_model(equal_session, equal_graph, {equal_net});
    require(equal_session.solve_once().ok, "equal-length fixed Sync bus members must be SAT");

    const auto unequal_graph = synthetic_graph(
        7, {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {5, 6}});
    auto unequal_net = synthetic_net(0, {0, 3}, {{2, {0}}, {6, {1}}});
    unequal_net.is_sync_bus = true;
    auto unequal_session = CadicalSession {};
    (void)build_unified_sat_model(unequal_session, unequal_graph, {unequal_net});
    require(!unequal_session.solve_once().ok, "unequal fixed Sync bus members must be UNSAT");
}

auto test_sync_bus_distance_starts_at_one() -> void {
    const auto graph = synthetic_graph(2, {{0, 1}});
    auto bus = synthetic_net(0, {0}, {{1, {0}}});
    bus.is_sync_bus = true;
    auto session = CadicalSession {};
    const auto model = build_unified_sat_model(session, graph, {bus});
    const auto solved = session.solve_once();
    require(solved.ok, "one-edge Sync bus distance fixture must be SAT");
    const auto& sink_bits = model.pairs[0].sink_distance_bits;
    require(sink_bits.size() == 2, "two scoped nodes require a two-bit strict distance width");
    require(
        !session.value(sink_bits[0]) && session.value(sink_bits[1]),
        "d_source=1 must make a one-edge sink distance equal binary 2");
}

auto test_sync_bus_forbids_forced_cycle() -> void {
    const auto graph = synthetic_graph(
        5, {{0, 1}, {1, 2}, {3, 4}, {4, 3}});
    auto bus = synthetic_net(0, {0}, {{2, {0}}});
    bus.is_sync_bus = true;
    auto bus_session = CadicalSession {};
    const auto bus_model = build_unified_sat_model(bus_session, graph, {bus});
    const auto& bus_pair = bus_model.pairs.front();
    bus_session.add_clause({bus_pair.x_vars[2]});
    bus_session.add_clause({bus_pair.x_vars[3]});
    require(!bus_session.solve_once().ok, "binary distance must reject a forced bus cycle");

    bus.is_sync_bus = false;
    auto ordinary_session = CadicalSession {};
    const auto ordinary_model = build_unified_sat_model(ordinary_session, graph, {bus});
    const auto& ordinary_pair = ordinary_model.pairs.front();
    ordinary_session.add_clause({ordinary_pair.x_vars[2]});
    ordinary_session.add_clause({ordinary_pair.x_vars[3]});
    const auto ordinary_solved = ordinary_session.solve_once();
    require(ordinary_solved.ok, "non-bus redundant disjoint cycles may remain satisfiable");
    const auto extracted = extract_sat_solution(
        graph, {bus}, ordinary_model, ordinary_session, ordinary_solved);
    require(
        extracted.ok && extracted.paths[0].node_path == std::Vector<int>({0, 1, 2}),
        "non-bus extraction must ignore a redundant disjoint cycle");
}

auto test_y_aggregation_and_partial_matching() -> void {
    auto graph = synthetic_graph(
        5, {{0, 1}, {1, 0}, {0, 2}, {2, 0}, {3, 4}});
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
    const auto model = build_unified_sat_model(
        session, graph, {synthetic_net(0, {3}, {{4, {0}}})});
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
        6, {{0, 1}, {1, 0}, {0, 2}, {2, 0}, {3, 4}, {3, 5}});
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
        synthetic_net(0, {3}, {{4, {0}}, {5, {0}}})};
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
    const auto model = build_unified_sat_model(session, graph, {net});
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
    const auto y_to_x_model = build_unified_sat_model(y_to_x_session, graph, {net});
    y_to_x_session.add_clause({y_to_x_model.switch_var_by_id.at(10)});
    for (const auto& pair : y_to_x_model.pairs) {
        y_to_x_session.add_clause({-pair.x_vars[0]});
        y_to_x_session.add_clause({-pair.x_vars[1]});
    }
    require(
        !y_to_x_session.solve_once().ok,
        "Y=true must require at least one direction x use across all pairs");

    auto x_to_y_session = CadicalSession {};
    const auto x_to_y_model = build_unified_sat_model(x_to_y_session, graph, {net});
    x_to_y_session.add_clause({-x_to_y_model.switch_var_by_id.at(10)});
    x_to_y_session.add_clause({x_to_y_model.pairs[0].x_vars[0]});
    x_to_y_session.add_clause({x_to_y_model.pairs[0].x_vars[1]});
    require(
        !x_to_y_session.solve_once().ok,
        "any directed x use must imply its aggregate physical-switch Y");
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
        const auto model = build_unified_sat_model(session, graph, nets);
        const auto solved = session.solve_once();
        require(solved.ok, "single physical-switch direction must be SAT");
        require(
            session.value(model.pairs[0].x_vars[static_cast<std::size_t>(expected_arc)]),
            "the expected directed physical arc must be selected");
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

auto test_extraction_rejects_multiple_next_arcs() -> void {
    const auto graph = synthetic_graph(3, {{0, 1}, {0, 2}});
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{1, {0}}})};
    auto session = CadicalSession {};
    const int active = session.new_var();
    const int x0 = session.new_var();
    const int x1 = session.new_var();
    session.add_clause({active});
    session.add_clause({x0});
    session.add_clause({x1});
    auto model = UnifiedSatModel {};
    auto scope = UnifiedSatNetScope {};
    scope.net_id = 0;
    scope.node_ids = {0, 1, 2};
    scope.arc_ids = {0, 1};
    scope.node_offset = {0, 1, 2};
    scope.arc_offset = {0, 1};
    model.scopes.push_back(scope);
    auto pair = UnifiedSatPairVars {};
    pair.net_id = 0;
    pair.demand_id = 0;
    pair.source_node = 0;
    pair.sink_node = 1;
    pair.activation = active;
    pair.x_vars = {x0, x1};
    model.pairs.push_back(pair);
    const auto solved = session.solve_once();
    const auto result = extract_sat_solution(graph, nets, model, session, solved);
    require(
        !result.ok && std::string {result.message}.find("multiple next arcs") != std::string::npos,
        "numeric extraction must report a malformed branching assignment");
}

struct ExtractionFixture {
    UnifiedSatModel model;
    std::unique_ptr<CadicalSession> session;
    CadicalSolveResult solved;
};

auto extraction_fixture(
    const UnifiedGraph& graph,
    int active_count,
    const std::Vector<int>& true_arc_offsets
) -> ExtractionFixture {
    auto session = std::make_unique<CadicalSession>();
    auto model = UnifiedSatModel {};
    auto scope = UnifiedSatNetScope {};
    scope.net_id = 0;
    scope.node_ids = {0, 1, 2};
    scope.arc_ids.resize(graph.arcs.size());
    scope.node_offset = {0, 1, 2};
    scope.arc_offset.resize(graph.arcs.size());
    for (std::size_t i = 0; i < graph.arcs.size(); ++i) {
        scope.arc_ids[i] = static_cast<int>(i);
        scope.arc_offset[i] = static_cast<int>(i);
    }
    model.scopes.push_back(scope);
    for (int pair_index = 0; pair_index < active_count; ++pair_index) {
        auto pair = UnifiedSatPairVars {};
        pair.net_id = 0;
        pair.demand_id = 0;
        pair.source_node = 0;
        pair.sink_node = 2;
        pair.activation = session->new_var();
        session->add_clause({pair.activation});
        pair.x_vars.resize(graph.arcs.size());
        for (std::size_t i = 0; i < graph.arcs.size(); ++i) {
            pair.x_vars[i] = session->new_var();
            session->add_clause({
                std::find(true_arc_offsets.begin(), true_arc_offsets.end(), static_cast<int>(i))
                        != true_arc_offsets.end()
                    ? pair.x_vars[i]
                    : -pair.x_vars[i]});
        }
        model.pairs.push_back(pair);
    }
    const auto solved = session->solve_once();
    return {std::move(model), std::move(session), solved};
}

auto test_extraction_error_branches() -> void {
    const auto nets = std::Vector<RoutingNet> {
        synthetic_net(0, {0}, {{2, {0}}})};

    const auto no_next_graph = synthetic_graph(3, {});
    auto no_active_session = CadicalSession {};
    const auto no_active_solved = no_active_session.solve_once();
    const auto no_active = extract_sat_solution(
        no_next_graph, nets, UnifiedSatModel {}, no_active_session, no_active_solved);
    require(
        !no_active.ok && std::string {no_active.message}.find("no active") != std::string::npos,
        "extraction must reject a demand with no active pair");

    const auto direct_graph = synthetic_graph(3, {{0, 2}});
    auto multiple_fixture = extraction_fixture(direct_graph, 2, {0});
    const auto multiple = extract_sat_solution(
        direct_graph,
        nets,
        multiple_fixture.model,
        *multiple_fixture.session,
        multiple_fixture.solved);
    require(
        !multiple.ok && std::string {multiple.message}.find("multiple active") != std::string::npos,
        "extraction must reject multiple active candidate pairs");

    auto stuck_fixture = extraction_fixture(no_next_graph, 1, {});
    const auto stuck = extract_sat_solution(
        no_next_graph,
        nets,
        stuck_fixture.model,
        *stuck_fixture.session,
        stuck_fixture.solved);
    require(
        !stuck.ok && std::string {stuck.message}.find("no next arc") != std::string::npos,
        "extraction must reject a path that does not reach its sink");

    const auto revisit_graph = synthetic_graph(3, {{0, 1}, {1, 0}});
    auto revisit_fixture = extraction_fixture(revisit_graph, 1, {0, 1});
    const auto revisit = extract_sat_solution(
        revisit_graph,
        nets,
        revisit_fixture.model,
        *revisit_fixture.session,
        revisit_fixture.solved);
    require(
        !revisit.ok && std::string {revisit.message}.find("revisits node") != std::string::npos,
        "extraction must reject a revisited node");
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
        test_sync_bus_successor_is_shared_per_node();
        test_cli_max_rss_option();
        test_sequential_at_most_one_truth_table_and_scaling();
        test_numeric_fixed_path_and_extraction();
        test_candidate_pairs_and_forced_flow_amo();
        test_logical_source_exclusivity();
        test_logical_source_owns_its_source_node();
        test_source_incoming_and_sink_outgoing_are_zero();
        test_mode_group_zero_conflict();
        test_all_mode_groups_and_group_zero_extraction();
        test_sync_bus_binary_distance();
        test_sync_bus_distance_starts_at_one();
        test_sync_bus_forbids_forced_cycle();
        test_y_aggregation_and_partial_matching();
        test_all_four_y_partial_matching_sides();
        test_y_bidirectional_equivalence();
        test_forward_and_reverse_switch_use_extract_same_y();
        test_extraction_rejects_multiple_next_arcs();
        test_extraction_error_branches();
        std::cout << "test_ILP_unit: all tests passed\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "test_ILP_unit: " << error.what() << '\n';
        return 1;
    }
}
