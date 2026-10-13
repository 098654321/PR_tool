#include "route_ilp/rrr_cost.hh"

#include <cmath>
#include <stdexcept>

namespace PR_tool {
namespace {
auto check_cost(bool value, const char* message) -> void {
    if (!value) throw std::runtime_error(message);
}
auto close(double a, double b) -> bool { return std::abs(a - b) < 1e-9; }
auto penalty(int u) -> double {
    return 1 + 4 / (std::exp(1.0 - u) + 1) + 0.2 * std::max(0, u - 1);
}
auto node(UnifiedGraph& graph, UnifiedNodeKind kind) -> int {
    const int id = static_cast<int>(graph.nodes.size());
    auto item = UnifiedNode{}; item.kind = kind; item.track_col = id;
    graph.nodes.push_back(item);
    graph.out_arc_ids.emplace_back(); graph.in_arc_ids.emplace_back();
    if (kind == UnifiedNodeKind::Track) graph.track_node_by_key[{0, 0, 0, id, 0}] = id;
    return id;
}
auto connect(UnifiedGraph& graph, int a, int b, int sw = -1,
             PhysicalSwitchKind kind = PhysicalSwitchKind::None,
             int group = -1, bool straight = false) -> int {
    const int first = static_cast<int>(graph.arcs.size());
    for (const auto [u, v] : {std::pair{a,b}, std::pair{b,a}}) {
        const int id = static_cast<int>(graph.arcs.size());
        graph.arcs.push_back({u, v, straight, group >= 0 && !straight, false, group, sw, kind});
        graph.out_arc_ids[u].push_back(id); graph.in_arc_ids[v].push_back(id);
    }
    return first;
}
auto column(RouteOwner owner, std::initializer_list<int> nodes) -> RouteColumn {
    auto out = RouteColumn{}; out.owner = owner;
    out.paths.push_back({owner.net_id, 0, owner.demand_id, -1, std::Vector<int>(nodes)});
    return out;
}

auto occupancy_and_history() -> void {
    auto graph = UnifiedGraph{};
    for (int i = 0; i < 3; ++i) node(graph, UnifiedNodeKind::Track);
    const int aid = connect(graph, 0, 1); connect(graph, 2, 1);
    auto cost = RrrCostModel{graph};
    const auto& arc = graph.arcs[aid];
    check_cost(close(cost.arc_cost(arc, {}), 1 + penalty(1)), "free-node FPIA cost mismatch");
    cost.set_selection({{{1,0}, column({1,0}, {2,1})}});
    check_cost(close(cost.arc_cost(arc, {}), 1 + penalty(2)), "present did not reflect another owner");
    check_cost(close(cost.arc_cost(arc, {{0,1,0}}), 1), "same-owner tree reuse was charged");
    cost.set_selection({});
    check_cost(close(cost.arc_cost(arc, {}), 1 + penalty(1)), "rip-up/rollback leaked occupancy");
    cost.set_selection({{{0,0}, column({0,0}, {0,1})}, {{1,0}, column({1,0}, {2,1})}});
    const auto update = cost.history_next();
    check_cost(update.overflow == 1 && close(update.total, 1), "history did not use resource overflow");
    cost.set_selection({});
    check_cost(close(cost.arc_cost(arc, {}), 2 + penalty(1)), "history was scaled by present cost");
    const auto decayed = cost.history_next();
    check_cost(decayed.overflow == 0 && close(decayed.total, 0.9), "history decay mismatch");
    check_cost(close(cost.arc_cost(arc, {}), 1.9 + penalty(1)), "unused resource lost history");
    auto fresh = RrrCostModel{graph};
    fresh.set_selection({{{0,0}, column({0,0}, {0,1})}});
    check_cost(close(fresh.history_next().total, 0), "legal committed state accrued history");
}

auto tob_mux_cost() -> void {
    auto graph = UnifiedGraph{};
    node(graph, UnifiedNodeKind::Bump);
    for (int i = 0; i < 3; ++i) node(graph, UnifiedNodeKind::HLine);
    const int a = connect(graph, 0, 1, 10, PhysicalSwitchKind::BumpH);
    const int b = connect(graph, 0, 2, 11, PhysicalSwitchKind::BumpH);
    const int c = connect(graph, 0, 3, 12, PhysicalSwitchKind::BumpH);
    auto cost = RrrCostModel{graph};
    // Node weight 1, switch + two matching + two mux weights 2 each.
    check_cost(close(cost.arc_cost(graph.arcs[a], {}), 34), "TOB resource type weights mismatch");
    check_cost(close(cost.arc_cost(graph.arcs[a+1], {}), 34), "reverse arc double-charged its origin");
    const auto own = std::set<RouteResource>{{0,0,0}, {0,1,0}, {1,10,0}, {2,0,0}, {2,1,1}};
    cost.set_own(column({0,0}, {0,1}).paths);
    check_cost(close(cost.arc_cost(graph.arcs[a], own), 1), "exact TOB reuse was charged");
    check_cost(close(cost.arc_cost(graph.arcs[b], own), 22 + 2 * penalty(2)),
               "new mux peer did not include the owner's retained tree");
    cost.set_own({});
    check_cost(close(cost.arc_cost(graph.arcs[b], {}), 34), "retained-tree claims leaked between searches");
    auto shared = column({0,0}, {0,1}); shared.paths.push_back({0,0,1,-1,{0,2}});
    cost.set_selection({{{0,0}, shared}});
    const auto update = cost.history_next();
    check_cost(update.overflow == 1 && close(update.total, 1), "same-owner mux fanout was not overflow");
    cost.set_selection({});
    check_cost(close(cost.arc_cost(graph.arcs[c], {}), 35), "new peer did not share mux-port history");
}

auto mode_cost() -> void {
    auto graph = UnifiedGraph{};
    node(graph, UnifiedNodeKind::VLine); node(graph, UnifiedNodeKind::Track);
    node(graph, UnifiedNodeKind::VLine); node(graph, UnifiedNodeKind::Track);
    const int a = connect(graph, 0, 1, 10, PhysicalSwitchKind::VLineTrack, 7, true);
    connect(graph, 2, 3, 11, PhysicalSwitchKind::VLineTrack, 7, false);
    auto cost = RrrCostModel{graph};
    check_cost(close(cost.arc_cost(graph.arcs[a], {}), 22), "free VLineTrack cost mismatch");
    cost.set_selection({{{1,0}, column({1,0}, {2,3})}});
    check_cost(close(cost.arc_cost(graph.arcs[a], {}), 46), "first mode conflict used the wrong predicted count");
    cost.set_selection({{{0,0}, column({0,0}, {0,1})}, {{1,0}, column({1,0}, {2,3})}});
    check_cost(cost.history_next().overflow == 1, "mode-conflict history was not group-based");
    cost.set_selection({{{1,0}, column({1,0}, {2,3})}});
    check_cost(close(cost.arc_cost(graph.arcs[a], {}), 47), "mode history was missing or scaled");
    check_cost(close(cost.arc_cost(graph.arcs[a], {{1,10,0}}), 4), "same-owner mode reuse charged old conflict history");
}

auto candidate_scoring_and_ilp_isolation() -> void {
    auto graph = UnifiedGraph{};
    node(graph, UnifiedNodeKind::Track); node(graph, UnifiedNodeKind::HLine);
    for (int i = 0; i < 3; ++i) node(graph, UnifiedNodeKind::Track);
    connect(graph, 0, 1, 20); connect(graph, 1, 3, 21);
    connect(graph, 0, 2); connect(graph, 2, 4); connect(graph, 4, 3);
    auto source = GraphNodeRef{}, sink = GraphNodeRef{};
    source.track_coord = {0,0,hardware::TrackDirection::Horizontal,0};
    sink.track_coord = {0,3,hardware::TrackDirection::Horizontal,0};
    auto net = RoutingNet{}; net.sources = {source}; net.demands = {{0,sink,{0},true}};
    auto scope = RoutingScope{};
    for (int i = 0; i < static_cast<int>(graph.nodes.size()); ++i) {
        scope.node_ids.push_back(i); scope.node_offset.push_back(i);
    }
    for (int i = 0; i < static_cast<int>(graph.arcs.size()); ++i) {
        scope.arc_ids.push_back(i); scope.arc_offset.push_back(i);
    }
    const auto ilp = find_route_column(graph, net, scope, {0,0}, {});
    check_cost(ilp.paths.front().node_path == std::Vector<int>({0,1,3}), "ILP pricing cost changed");
    auto cost = RrrCostModel{graph};
    auto options = RouteSearchOptions{}; options.rrr_cost = &cost;
    const auto rrr = find_route_column(graph, net, scope, {0,0}, options);
    check_cost(rrr.paths.front().node_path == std::Vector<int>({0,2,4,3}),
               "RRR search/candidate scoring retained the old wirelength cost");
}
} // namespace

auto run_rrr_cost_tests() -> void {
    occupancy_and_history(); tob_mux_cost(); mode_cost(); candidate_scoring_and_ilp_isolation();
}
} // namespace PR_tool
