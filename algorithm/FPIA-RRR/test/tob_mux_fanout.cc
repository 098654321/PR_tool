#include "tob_mux_fanout.hh"

#include "hardware_graph.hh"
#include "hw_map.hh"
#include "maze_search.hh"
#include "net_adapter.hh"
#include "resource_model.hh"
#include "route_validate.hh"
#include "rrr_router.hh"
#include "rrr_types.hh"

#include <hardware/interposer.hh>
#include <hardware/track/trackcoord.hh>

#include <stdexcept>
#include <string>
#include <utility>

namespace PR_tool {

namespace {

auto require(bool condition, const std::string& message) -> void {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

auto add_synth_node(UnifiedGraph& graph, UnifiedNodeKind kind = UnifiedNodeKind::Track) -> int {
    UnifiedNode node {};
    node.kind = kind;
    const int id = static_cast<int>(graph.nodes.size());
    graph.nodes.push_back(node);
    graph.in_arc_ids.emplace_back();
    graph.out_arc_ids.emplace_back();
    return id;
}

auto add_synth_arc(UnifiedGraph& graph, int u, int v, PhysicalSwitchKind kind = PhysicalSwitchKind::None)
    -> void {
    const int arc_id = static_cast<int>(graph.arcs.size());
    UnifiedArc arc {};
    arc.u = u;
    arc.v = v;
    arc.physical_switch_kind = kind;
    arc.physical_switch_id = arc_id;
    graph.arcs.push_back(arc);
    graph.out_arc_ids[static_cast<std::size_t>(u)].push_back(arc_id);
    graph.in_arc_ids[static_cast<std::size_t>(v)].push_back(arc_id);
    graph.directed_arc_set.insert({u, v});
}

auto add_undirected(UnifiedGraph& graph, int u, int v, PhysicalSwitchKind kind = PhysicalSwitchKind::None)
    -> void {
    add_synth_arc(graph, u, v, kind);
    add_synth_arc(graph, v, u, kind);
}

auto add_mapped_track(UnifiedGraph& graph, int row, int col, std::size_t track_index) -> int {
    const int id = add_synth_node(graph, UnifiedNodeKind::Track);
    auto& node = graph.nodes[static_cast<std::size_t>(id)];
    node.track_row = row;
    node.track_col = col;
    node.track_dir = 1;
    node.track_index = track_index;
    node.unit = map_track(track_index);
    graph.track_node_by_key[std::tuple {
        node.unit,
        node.track_dir,
        node.track_row,
        node.track_col,
        node.track_index}] = id;
    return id;
}

auto add_synth_bump(UnifiedGraph& graph, std::size_t tob, std::size_t group, std::size_t index) -> int {
    const int id = add_synth_node(graph, UnifiedNodeKind::Bump);
    const auto bump = Bump_coord {tob, 0, group, index};
    graph.nodes[static_cast<std::size_t>(id)].bump = bump;
    graph.bump_node_by_key[bump] = id;
    return id;
}

auto bump_ref(std::size_t tob, std::size_t group, std::size_t index) -> GraphNodeRef {
    GraphNodeRef ref {};
    ref.kind = GraphNodeRef::Kind::Bump;
    ref.bump = Bump_coord {tob, 0, group, index};
    return ref;
}

auto track_ref(int row, int col, std::size_t track_index) -> GraphNodeRef {
    GraphNodeRef ref {};
    ref.kind = GraphNodeRef::Kind::Track;
    ref.track_coord = hardware::TrackCoord {
        row,
        col,
        hardware::TrackDirection::Vertical,
        track_index};
    ref.track_index = track_index;
    return ref;
}

auto make_pnnet(std::size_t net_id, GraphNodeRef source, const std::Vector<GraphNodeRef>& sinks)
    -> RoutingNet {
    RoutingNet net {};
    net.net_id = net_id;
    net.name = "synth_pnnet";
    net.kind = RoutingNetKind::PNnet;
    net.sources.push_back(source);
    for (std::size_t i = 0; i < sinks.size(); ++i) {
        RoutingDemand demand {};
        demand.demand_id = i;
        demand.sink = sinks[i];
        demand.candidate_source_indices = {0};
        demand.fixed_pair = false;
        net.demands.push_back(demand);
    }
    return net;
}

auto test_mux_same_owner_two_peers_overflow() -> void {
    auto model = ResourceModel {};
    const auto owner = OwnerId {1, 0};
    const auto first = tob_mux_output_key(10, 20);
    const auto second = tob_mux_output_key(10, 21);
    model.claim(owner, {first});
    require(model.overflow() == 0, "one mux peer must not overflow");
    require(model.mux_distinct_peers(first) == 1, "one claimed extra is one peer");
    require(!model.mux_has_other_peer(owner, first), "the exact mux peer must remain reusable");
    require(model.mux_has_other_peer(owner, second), "a second peer on one owner mux port must be blocked");
    model.claim(owner, {second});
    require(model.overflow() == 1, "same owner with two mux peers must overflow by 1");
    require(model.overflow(first) == 1, "port overflow is visible on either extra key");
    require(model.type_weight(first) == 2, "mux port type_weight must match matching endpoints");
    model.claim(owner, {first});
    require(model.overflow() == 1, "reclaiming the same connection must not add another peer");
    model.history_next();
    const auto third = tob_mux_output_key(10, 22);
    require(model.history(first) > 0, "overflowing mux port must accrue history");
    require(model.history(second) == model.history(first), "mux history must be shared across extras of one port");
    require(model.history(third) == model.history(first), "a new extra on a hot mux port must see the same history");
}

auto test_illegal_four_bump_tails() -> void {
    auto graph = UnifiedGraph {};
    const int track = add_synth_node(graph, UnifiedNodeKind::Track);
    const int vline = add_synth_node(graph, UnifiedNodeKind::VLine);
    auto hlines = std::Vector<int> {};
    auto bumps = std::Vector<int> {};
    for (std::size_t i = 0; i < 4; ++i) {
        hlines.push_back(add_synth_node(graph, UnifiedNodeKind::HLine));
        bumps.push_back(add_synth_bump(graph, 0, i, 0));
        add_undirected(graph, vline, hlines.back(), PhysicalSwitchKind::HLineVLine);
        add_undirected(graph, hlines.back(), bumps.back(), PhysicalSwitchKind::BumpH);
    }
    add_undirected(graph, track, vline, PhysicalSwitchKind::VLineTrack);

    auto paths = std::Vector<std::Vector<int>> {
        {track, vline, hlines[0], bumps[0]},
        {vline, hlines[1], bumps[1]},
        {vline, hlines[2], bumps[2]},
        {vline, hlines[3], bumps[3]}};
    const auto hits = collect_illegal_tob_fanout(graph, paths);
    require(!hits.empty(), "four VLine-HLine tails must be reported as illegal fanout");
    bool found_vline = false;
    for (const auto& hit : hits) {
        if (hit.node_id == vline && hit.side == "HLine") {
            found_vline = true;
            require(hit.peer_count == 4, "illegal fixture VLine must have four HLine peers");
        }
    }
    require(found_vline, "illegal fixture must flag the shared VLine");

    auto sinks = std::Vector<GraphNodeRef> {};
    for (std::size_t i = 0; i < 4; ++i) {
        sinks.push_back(bump_ref(0, i, 0));
    }
    auto nets = std::Vector<RoutingNet> {make_pnnet(0, track_ref(0, 0, 0), sinks)};
    graph.nodes[static_cast<std::size_t>(track)].track_row = 0;
    graph.nodes[static_cast<std::size_t>(track)].track_col = 0;
    graph.nodes[static_cast<std::size_t>(track)].track_dir = 1;
    graph.nodes[static_cast<std::size_t>(track)].track_index = 0;
    graph.nodes[static_cast<std::size_t>(track)].unit = map_track(0);
    graph.track_node_by_key[std::tuple {map_track(0), 1, 0, 0, std::size_t {0}}] = track;

    RrrResult result {};
    result.status = "success";
    result.paths = {paths};
    require(!validate_rrr_solution(graph, nets, result), "illegal four-bump tails must fail validation");
}

auto test_legal_track_steiner_still_ok() -> void {
    auto graph = UnifiedGraph {};
    const int source = add_mapped_track(graph, 0, 0, 0);
    const int trunk = add_mapped_track(graph, 0, 1, 1);
    const int leaf_a = add_mapped_track(graph, 1, 1, 2);
    const int leaf_b = add_mapped_track(graph, 2, 1, 3);
    add_undirected(graph, source, trunk);
    add_undirected(graph, trunk, leaf_a);
    add_undirected(graph, trunk, leaf_b);

    const int v0 = add_synth_node(graph, UnifiedNodeKind::VLine);
    const int v1 = add_synth_node(graph, UnifiedNodeKind::VLine);
    const int h0 = add_synth_node(graph, UnifiedNodeKind::HLine);
    const int h1 = add_synth_node(graph, UnifiedNodeKind::HLine);
    const int b0 = add_synth_bump(graph, 0, 0, 0);
    const int b1 = add_synth_bump(graph, 0, 1, 0);
    add_undirected(graph, leaf_a, v0, PhysicalSwitchKind::VLineTrack);
    add_undirected(graph, v0, h0, PhysicalSwitchKind::HLineVLine);
    add_undirected(graph, h0, b0, PhysicalSwitchKind::BumpH);
    add_undirected(graph, leaf_b, v1, PhysicalSwitchKind::VLineTrack);
    add_undirected(graph, v1, h1, PhysicalSwitchKind::HLineVLine);
    add_undirected(graph, h1, b1, PhysicalSwitchKind::BumpH);

    const auto paths = std::Vector<std::Vector<int>> {
        {source, trunk, leaf_a, v0, h0, b0},
        {trunk, leaf_b, v1, h1, b1}};
    require(
        collect_illegal_tob_fanout(graph, paths).empty(),
        "shared Track trunk with disjoint TOB accesses must be legal");

    auto nets = std::Vector<RoutingNet> {
        make_pnnet(0, track_ref(0, 0, 0), {bump_ref(0, 0, 0), bump_ref(0, 1, 0)})};
    RrrResult result {};
    result.status = "success";
    result.paths = {paths};
    require(validate_rrr_solution(graph, nets, result), "legal Track Steiner fanout must pass validation");
}

auto test_run_rrr_assigns_disjoint_tob_access() -> void {
    auto graph = UnifiedGraph {};
    const int source = add_mapped_track(graph, 0, 0, 8);
    const int shared_vline = add_synth_node(graph, UnifiedNodeKind::VLine);
    add_undirected(graph, source, shared_vline, PhysicalSwitchKind::VLineTrack);
    auto sinks = std::Vector<GraphNodeRef> {};
    for (std::size_t i = 0; i < 4; ++i) {
        const int leaf = add_mapped_track(graph, 0, static_cast<int>(i + 1), i);
        const int vline = add_synth_node(graph, UnifiedNodeKind::VLine);
        const int hline = add_synth_node(graph, UnifiedNodeKind::HLine);
        const int bump = add_synth_bump(graph, 0, i, 0);
        add_undirected(graph, source, leaf);
        add_undirected(graph, leaf, vline, PhysicalSwitchKind::VLineTrack);
        add_undirected(graph, shared_vline, hline, PhysicalSwitchKind::HLineVLine);
        add_undirected(graph, vline, hline, PhysicalSwitchKind::HLineVLine);
        add_undirected(graph, hline, bump, PhysicalSwitchKind::BumpH);
        sinks.push_back(bump_ref(0, i, 0));
    }

    auto nets = std::Vector<RoutingNet> {make_pnnet(0, track_ref(0, 0, 8), sinks)};
    auto interposer = hardware::Interposer {};
    auto params = RrrParams {};
    params.max_iterations = 8;
    const auto result = run_rrr(graph, nets, params, &interposer);
    require(result.status == "success", "four-bump synthetic PNnet must route");
    require(result.best_overflow == 0, "four-bump synthetic PNnet must finish overflow-free");
    require(result.paths.size() == 1, "synthetic PNnet result must have one net");
    require(result.paths[0].size() == 4, "synthetic PNnet must keep four demand paths");
    require(
        collect_illegal_tob_fanout(graph, result.paths[0]).empty(),
        "run_rrr must not emit VLine-HLine parasitic tails");
    require(
        validate_rrr_solution(graph, nets, result, &interposer),
        "run_rrr four-bump result must pass independent validation");
    for (const auto& path : result.paths[0]) {
        require(path.size() >= 4, "each legal TOB access must include Track and TOB nodes");
        bool has_track = false;
        for (const int node : path) {
            if (graph.nodes[static_cast<std::size_t>(node)].kind == UnifiedNodeKind::Track) {
                has_track = true;
            }
        }
        require(has_track, "no demand may be a VLine-HLine-Bump tail without a Track access");
    }
}

} // namespace

auto run_tob_mux_fanout_unit_tests() -> void {
    test_mux_same_owner_two_peers_overflow();
    test_illegal_four_bump_tails();
    test_legal_track_steiner_still_ok();
    test_run_rrr_assigns_disjoint_tob_access();
}

} // namespace PR_tool
