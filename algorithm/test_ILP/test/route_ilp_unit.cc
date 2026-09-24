#include "direct_ilp/direct_validate.hh"
#include "route_ilp/route_master.hh"
#include "route_ilp/route_rrr.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace PR_tool {
namespace {

auto require(bool value, const char* message) -> void {
    if (!value) throw std::runtime_error(message);
}

auto add_node(UnifiedGraph& graph, UnifiedNodeKind kind, int col,
              std::size_t unit = 0) -> int {
    const int id = static_cast<int>(graph.nodes.size());
    auto node = UnifiedNode{};
    node.kind = kind;
    node.unit = unit;
    node.track_col = col;
    node.track_index = unit;
    node.bump = Bump_coord{0, 0, 0, static_cast<std::size_t>(col)};
    graph.nodes.push_back(node);
    graph.out_arc_ids.emplace_back();
    graph.in_arc_ids.emplace_back();
    if (kind == UnifiedNodeKind::Track)
        graph.track_node_by_key[{unit, 0, 0, col, unit}] = id;
    if (kind == UnifiedNodeKind::Bump) graph.bump_node_by_key[node.bump] = id;
    return id;
}

auto ref(UnifiedNodeKind kind, int col, std::size_t unit = 0) -> GraphNodeRef {
    auto result = GraphNodeRef{};
    result.kind = kind == UnifiedNodeKind::Bump ? GraphNodeRef::Kind::Bump :
        GraphNodeRef::Kind::Track;
    result.bump = Bump_coord{0, 0, 0, static_cast<std::size_t>(col)};
    result.track_coord = hardware::TrackCoord{0, col,
        hardware::TrackDirection::Horizontal, unit};
    result.track_index = unit;
    return result;
}

auto edge(UnifiedGraph& graph, int a, int b) -> void {
    for (const auto [u, v] : {std::pair{a, b}, std::pair{b, a}}) {
        const int id = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back(UnifiedArc{u, v});
        graph.out_arc_ids[static_cast<std::size_t>(u)].push_back(id);
        graph.in_arc_ids[static_cast<std::size_t>(v)].push_back(id);
    }
}

auto mode_edge(UnifiedGraph& graph, int a, int b, int switch_id,
               bool straight) -> void {
    for (const auto [u, v] : {std::pair{a, b}, std::pair{b, a}}) {
        const int id = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back(UnifiedArc{u, v, straight, !straight, false,
            7, switch_id, PhysicalSwitchKind::VLineTrack});
        graph.out_arc_ids[static_cast<std::size_t>(u)].push_back(id);
        graph.in_arc_ids[static_cast<std::size_t>(v)].push_back(id);
    }
}

auto full_scope(const UnifiedGraph& graph, std::size_t net_id = 0)
    -> RoutingScope {
    auto scope = RoutingScope{}; scope.net_id = net_id;
    for (int n = 0; n < static_cast<int>(graph.nodes.size()); ++n) {
        scope.node_offset.push_back(n); scope.node_ids.push_back(n);
    }
    for (int a = 0; a < static_cast<int>(graph.arcs.size()); ++a) {
        scope.arc_offset.push_back(a); scope.arc_ids.push_back(a);
    }
    return scope;
}

auto simple_master() -> void {
    auto graph = UnifiedGraph{};
    const int a = add_node(graph, UnifiedNodeKind::Track, 0);
    const int b = add_node(graph, UnifiedNodeKind::Track, 1);
    edge(graph, a, b);
    auto net = RoutingNet{};
    net.sources.push_back(ref(UnifiedNodeKind::Track, 0));
    net.demands.push_back({0, ref(UnifiedNodeKind::Track, 1), {0}, true});
    const auto nets = std::Vector<RoutingNet>{net};
    const auto scopes = std::Vector<RoutingScope>{full_scope(graph)};
    const auto result = solve_route_ilp(graph, nets, scopes,
        RouteIlpOptions{0, 1, "/private/tmp/route_ilp_unit_highs.log"});
    require(result.route.ok && result.missing.empty(), "master failed simple net");
    require(result.big_m == 10000.0, "default route ILP M changed");
    require(validate_direct_route(graph, nets, scopes, result.route),
            "master produced invalid route");
    bool rejected = false;
    try {
        (void)solve_route_ilp(graph, nets, scopes,
            RouteIlpOptions{0, 1, "/private/tmp/route_ilp_unit_highs.log",
                            RouteBigMMode::MinLmin});
    } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "SyncBus M mode accepted an instance without SyncBus");
}

auto sync_master_length() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 7; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    edge(graph, 0, 1); edge(graph, 0, 2); edge(graph, 2, 1);
    edge(graph, 4, 5); edge(graph, 5, 6);
    auto net = RoutingNet{}; net.is_sync_bus = true;
    net.sources = {ref(UnifiedNodeKind::Track, 0),
                   ref(UnifiedNodeKind::Track, 4)};
    net.demands = {{0, ref(UnifiedNodeKind::Track, 1), {0}, true},
                   {1, ref(UnifiedNodeKind::Track, 6), {1}, true}};
    const auto nets = std::Vector<RoutingNet>{net};
    const auto scopes = std::Vector<RoutingScope>{full_scope(graph)};
    const auto result = solve_route_ilp(graph, nets, scopes,
        RouteIlpOptions{0, 1, "/private/tmp/route_ilp_unit_highs.log"});
    require(result.route.ok && result.bus_lengths.at(0) == 3,
            "master did not use common SyncBus length");
    require(validate_direct_route(graph, nets, scopes, result.route),
            "master SyncBus invalid");
    const auto gap = solve_route_ilp(graph, nets, scopes,
        RouteIlpOptions{0, 1, "/private/tmp/route_ilp_unit_highs.log",
                        RouteBigMMode::GapTwo});
    require(std::abs(gap.big_m - 3.5) < 1e-9,
            "negative initial max-owner gap was subtracted incorrectly");
}

auto sync_master_length_increase() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 6; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    mode_edge(graph, 0, 1, 10, true);
    mode_edge(graph, 2, 3, 11, false);
    edge(graph, 0, 4); edge(graph, 4, 1);
    edge(graph, 2, 5); edge(graph, 5, 3);
    auto net = RoutingNet{}; net.is_sync_bus = true;
    net.sources = {ref(UnifiedNodeKind::Track, 0),
                   ref(UnifiedNodeKind::Track, 2)};
    net.demands = {{0, ref(UnifiedNodeKind::Track, 1), {0}, true},
                   {1, ref(UnifiedNodeKind::Track, 3), {1}, true}};
    const auto nets = std::Vector<RoutingNet>{net};
    const auto scopes = std::Vector<RoutingScope>{full_scope(graph)};
    const auto result = solve_route_ilp(graph, nets, scopes,
        RouteIlpOptions{0, 1, "/private/tmp/route_ilp_unit_highs.log"});
    require(result.route.ok && result.bus_lengths.at(0) == 3,
            "master did not raise SyncBus length after mode conflict");
    require(validate_direct_route(graph, nets, scopes, result.route),
            "length-increased SyncBus invalid");
}

auto big_m_modes() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 18; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    edge(graph, 0, 1); edge(graph, 2, 3);
    edge(graph, 4, 5); edge(graph, 5, 6);
    edge(graph, 7, 8); edge(graph, 8, 9);
    for (int i = 10; i < 17; ++i) edge(graph, i, i + 1);
    auto nets = std::Vector<RoutingNet>(3);
    for (std::size_t bus = 0; bus < 2; ++bus) {
        auto& net = nets[bus];
        net.net_id = bus;
        net.is_sync_bus = true;
        const int first = bus == 0 ? 0 : 4;
        const int second = bus == 0 ? 2 : 7;
        const int offset = bus == 0 ? 1 : 2;
        net.sources = {ref(UnifiedNodeKind::Track, first),
                       ref(UnifiedNodeKind::Track, second)};
        net.demands = {{0, ref(UnifiedNodeKind::Track, first + offset), {0}, true},
                       {1, ref(UnifiedNodeKind::Track, second + offset), {1}, true}};
    }
    nets[2].net_id = 2;
    nets[2].sources = {ref(UnifiedNodeKind::Track, 10)};
    nets[2].demands = {{0, ref(UnifiedNodeKind::Track, 17), {0}, true}};
    const auto scopes = std::Vector<RoutingScope>{
        full_scope(graph, 0), full_scope(graph, 1), full_scope(graph, 2)};
    const auto expected = std::array{
        std::pair{RouteBigMMode::MinLmin, 2.0},
        std::pair{RouteBigMMode::MinLminPlusOne, 3.0},
        std::pair{RouteBigMMode::MaxLmin, 3.0},
        std::pair{RouteBigMMode::MaxLminPlusOne, 4.0},
        std::pair{RouteBigMMode::GapOne, 8.0},
        std::pair{RouteBigMMode::GapTwo, 6.0},
        std::pair{RouteBigMMode::GapThree, 4.0 + 4.0 / 3.0},
        std::pair{RouteBigMMode::GapFour, 5.0},
    };
    for (const auto [mode, value] : expected) {
        const auto result = solve_route_ilp(graph, nets, scopes,
            RouteIlpOptions{0, 1, "/private/tmp/route_ilp_unit_highs.log", mode});
        require(std::abs(result.big_m - value) < 1e-9,
                "route ILP selected the wrong M for a SyncBus experiment mode");
        if (mode == RouteBigMMode::MinLmin)
            require(result.missing.contains({2, 0}),
                    "MIP selected an isolated owner whose route costs more than M");
    }
}

auto missing_sync_lane() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 6; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    edge(graph, 0, 1); edge(graph, 1, 2);
    edge(graph, 3, 4); edge(graph, 4, 5);
    auto net = RoutingNet{}; net.is_sync_bus = true;
    net.sources = {ref(UnifiedNodeKind::Track, 0),
                   ref(UnifiedNodeKind::Track, 3)};
    net.demands = {{0, ref(UnifiedNodeKind::Track, 2), {0}, true},
                   {1, ref(UnifiedNodeKind::Track, 5), {1}, true}};
    auto baseline = RouteIlpResult{};
    baseline.has_integer_solution = true;
    baseline.route.paths.push_back({0, 0, 0, -1, {0, 1, 2}});
    baseline.route.total_wirelength = 3;
    baseline.missing.insert({0, 1});
    baseline.bus_lengths[0] = 3;
    const auto nets = std::Vector<RoutingNet>{net};
    const auto scopes = std::Vector<RoutingScope>{full_scope(graph)};
    const auto result = optimize_route_columns_rrr(graph, nets, scopes, baseline);
    require(result.ok && result.paths.size() == 2, "RRR did not complete SyncBus");
    require(validate_direct_route(graph, nets, scopes, result),
            "RRR SyncBus invalid");
}

auto sync_lane_displacement() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 6; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    edge(graph, 0, 4); edge(graph, 4, 1);
    edge(graph, 0, 5); edge(graph, 5, 1);
    edge(graph, 2, 4); edge(graph, 4, 3);
    auto net = RoutingNet{}; net.is_sync_bus = true;
    net.sources = {ref(UnifiedNodeKind::Track, 0),
                   ref(UnifiedNodeKind::Track, 2)};
    net.demands = {{0, ref(UnifiedNodeKind::Track, 1), {0}, true},
                   {1, ref(UnifiedNodeKind::Track, 3), {1}, true}};
    auto baseline = RouteIlpResult{};
    baseline.has_integer_solution = true;
    baseline.route.paths.push_back({0, 0, 0, -1, {0, 4, 1}});
    baseline.route.total_wirelength = 3;
    baseline.missing.insert({0, 1});
    baseline.bus_lengths[0] = 3;
    const auto nets = std::Vector<RoutingNet>{net};
    const auto scopes = std::Vector<RoutingScope>{full_scope(graph)};
    const auto result = optimize_route_columns_rrr(graph, nets, scopes, baseline);
    require(result.ok && result.paths.size() == 2,
            "RRR did not displace routed SyncBus lane");
    const auto old_lane = std::find_if(result.paths.begin(), result.paths.end(),
        [](const auto& path) { return path.demand_id == 0; });
    require(old_lane != result.paths.end() && old_lane->node_path[1] == 5,
            "RRR kept SyncBus as hard obstacle");
    require(validate_direct_route(graph, nets, scopes, result),
            "RRR displaced SyncBus invalid");
}

auto sync_cut_tail() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 5; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    edge(graph, 0, 1); edge(graph, 1, 2); edge(graph, 2, 3);
    edge(graph, 1, 4); edge(graph, 4, 3);
    auto net = RoutingNet{}; net.is_sync_bus = true;
    net.sources = {ref(UnifiedNodeKind::Track, 0)};
    net.demands = {{0, ref(UnifiedNodeKind::Track, 3), {0}, true}};
    const auto former = route_column_from_paths(graph, net, {0, 0},
        {{0, 0, 0, -1, {0, 1, 2, 3}}});
    auto options = RouteSearchOptions{};
    options.exact_length = 4;
    options.discouraged_nodes.insert(2);
    const auto rerouted = find_sync_prefix_column(graph, net,
        full_scope(graph), {0, 0}, options, former, 50);
    require(!rerouted.paths.empty() && rerouted.paths.front().node_path ==
        std::Vector<int>({0, 1, 4, 3}),
        "SyncBus cut-tail search did not preserve prefix and equal length");
}

auto chained_rrr_displacement() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 9; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    edge(graph, 0, 6); edge(graph, 6, 1);
    edge(graph, 2, 6); edge(graph, 6, 3);
    edge(graph, 2, 7); edge(graph, 7, 3);
    edge(graph, 4, 7); edge(graph, 7, 5);
    edge(graph, 4, 8); edge(graph, 8, 5);
    auto nets = std::Vector<RoutingNet>(3);
    for (std::size_t i = 0; i < nets.size(); ++i) {
        nets[i].net_id = i;
        nets[i].sources = {ref(UnifiedNodeKind::Track, static_cast<int>(2 * i))};
        nets[i].demands = {{0, ref(UnifiedNodeKind::Track,
            static_cast<int>(2 * i + 1)), {0}, true}};
    }
    auto scopes = std::Vector<RoutingScope>{};
    for (std::size_t i = 0; i < nets.size(); ++i)
        scopes.push_back(full_scope(graph, i));
    auto baseline = RouteIlpResult{}; baseline.has_integer_solution = true;
    baseline.route.paths = {{1, 0, 0, -1, {2, 6, 3}},
                            {2, 0, 0, -1, {4, 7, 5}}};
    baseline.route.total_wirelength = 6;
    baseline.missing.insert({0, 0});
    const auto result = optimize_route_columns_rrr(graph, nets, scopes, baseline);
    require(result.ok && result.paths.size() == 3,
            "RRR failed chained displacement");
    require(validate_direct_route(graph, nets, scopes, result),
            "chained displacement produced invalid route");
}

auto mode_resources() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 6; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    mode_edge(graph, 0, 1, 10, true);
    mode_edge(graph, 2, 3, 11, false);
    mode_edge(graph, 4, 5, 12, true);
    auto nets = std::Vector<RoutingNet>(3);
    for (std::size_t i = 0; i < nets.size(); ++i) {
        nets[i].net_id = i;
        nets[i].sources = {ref(UnifiedNodeKind::Track, static_cast<int>(2 * i))};
        nets[i].demands = {{0, ref(UnifiedNodeKind::Track,
                                     static_cast<int>(2 * i + 1)), {0}, true}};
    }
    const auto a = route_column_from_paths(graph, nets[0], {0, 0},
        {{0, 0, 0, -1, {0, 1}}});
    const auto b = route_column_from_paths(graph, nets[1], {1, 0},
        {{1, 0, 0, -1, {2, 3}}});
    const auto c = route_column_from_paths(graph, nets[2], {2, 0},
        {{2, 0, 0, -1, {4, 5}}});
    require(route_columns_conflict(a, b), "opposite TOB modes did not conflict");
    require(!route_columns_conflict(a, c), "same TOB mode falsely conflicted");
}

auto invalid_shortest_detour() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 5; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    mode_edge(graph, 0, 1, 20, true);
    mode_edge(graph, 1, 2, 21, false);
    edge(graph, 0, 3); edge(graph, 3, 4); edge(graph, 4, 2);
    auto net = RoutingNet{};
    net.sources = {ref(UnifiedNodeKind::Track, 0)};
    net.demands = {{0, ref(UnifiedNodeKind::Track, 2), {0}, true}};
    const auto column = find_route_column(graph, net, full_scope(graph),
                                           {0, 0}, {});
    require(!column.paths.empty() && column.paths.front().node_path ==
        std::Vector<int>({0, 3, 4, 2}),
        "search missed a legal detour after invalid shortest path");
}

auto independent_validation_rejects_bad_topology() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 6; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    edge(graph, 0, 1); edge(graph, 1, 2); edge(graph, 2, 3);
    edge(graph, 0, 4); edge(graph, 4, 2); edge(graph, 2, 5);
    auto net = RoutingNet{};
    net.sources = {ref(UnifiedNodeKind::Track, 0)};
    net.demands = {{0, ref(UnifiedNodeKind::Track, 3), {0}, true},
                   {1, ref(UnifiedNodeKind::Track, 5), {0}, true}};
    auto route = RoutingResult{};
    route.paths = {{0, 0, 0, -1, {0, 1, 2, 3}},
                   {0, 0, 1, -1, {0, 4, 2, 5}}};
    route.total_wirelength = 6;
    require(!validate_direct_route(graph, {net}, {full_scope(graph)}, route),
            "independent validator accepted a physical union cycle");
    auto bump_graph = UnifiedGraph{};
    add_node(bump_graph, UnifiedNodeKind::Track, 0);
    add_node(bump_graph, UnifiedNodeKind::Bump, 1);
    add_node(bump_graph, UnifiedNodeKind::Track, 2);
    edge(bump_graph, 0, 1); edge(bump_graph, 1, 2);
    auto bump_net = RoutingNet{};
    bump_net.sources = {ref(UnifiedNodeKind::Track, 0)};
    bump_net.demands = {{0, ref(UnifiedNodeKind::Track, 2), {0}, true}};
    auto bump_route = RoutingResult{};
    bump_route.paths = {{0, 0, 0, -1, {0, 1, 2}}};
    bump_route.total_wirelength = 3;
    require(!validate_direct_route(bump_graph, {bump_net},
                                   {full_scope(bump_graph)}, bump_route),
            "independent validator accepted an internal Bump");
}

auto pn_source_change() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 4; ++i) add_node(graph, UnifiedNodeKind::Track, i);
    const int bump = add_node(graph, UnifiedNodeKind::Bump, 4);
    edge(graph, 0, 2); edge(graph, 2, 3); edge(graph, 3, bump);
    edge(graph, 1, bump);
    auto net = RoutingNet{}; net.kind = RoutingNetKind::PNnet;
    net.sources = {ref(UnifiedNodeKind::Track, 0),
                   ref(UnifiedNodeKind::Track, 1)};
    net.demands = {{0, ref(UnifiedNodeKind::Bump, 4), {0, 1}, false}};
    auto baseline = RouteIlpResult{}; baseline.has_integer_solution = true;
    baseline.route.ok = true;
    baseline.route.paths.push_back({0, 0, 0, 0, {0, 2, 3, bump}});
    baseline.route.total_wirelength = 4;
    const auto nets = std::Vector<RoutingNet>{net};
    const auto scopes = std::Vector<RoutingScope>{full_scope(graph)};
    const auto result = optimize_route_columns_rrr(graph, nets, scopes, baseline);
    require(result.ok && result.paths.front().source_index == 1,
            "RRR locked PN source");
    require(result.total_wirelength == 2, "RRR did not improve PN wirelength");
}

auto unit_change() -> void {
    auto graph = UnifiedGraph{};
    const int a = add_node(graph, UnifiedNodeKind::Bump, 0);
    const int b = add_node(graph, UnifiedNodeKind::Bump, 1);
    const int u0 = add_node(graph, UnifiedNodeKind::Track, 2, 0);
    const int u0b = add_node(graph, UnifiedNodeKind::Track, 3, 0);
    const int u1 = add_node(graph, UnifiedNodeKind::Track, 4, 1);
    edge(graph, a, u0); edge(graph, u0, u0b); edge(graph, u0b, b);
    edge(graph, a, u1); edge(graph, u1, b);
    auto net = RoutingNet{}; net.kind = RoutingNetKind::Bnet;
    net.sources = {ref(UnifiedNodeKind::Bump, 0)};
    net.demands = {{0, ref(UnifiedNodeKind::Bump, 1), {0}, true}};
    auto baseline = RouteIlpResult{}; baseline.has_integer_solution = true;
    baseline.route.ok = true;
    baseline.route.paths.push_back({0, 0, 0, -1, {a, u0, u0b, b}});
    baseline.route.total_wirelength = 4;
    const auto nets = std::Vector<RoutingNet>{net};
    const auto scopes = std::Vector<RoutingScope>{full_scope(graph)};
    const auto result = optimize_route_columns_rrr(graph, nets, scopes, baseline);
    require(result.ok && result.paths.front().node_path[1] == u1,
            "RRR locked Bnet unit");
}

} // namespace
} // namespace PR_tool

auto main() -> int {
    PR_tool::simple_master();
    PR_tool::sync_master_length();
    PR_tool::sync_master_length_increase();
    PR_tool::big_m_modes();
    PR_tool::missing_sync_lane();
    PR_tool::sync_lane_displacement();
    PR_tool::sync_cut_tail();
    PR_tool::chained_rrr_displacement();
    PR_tool::mode_resources();
    PR_tool::invalid_shortest_detour();
    PR_tool::independent_validation_rejects_bad_topology();
    PR_tool::pn_source_change();
    PR_tool::unit_change();
    return 0;
}
