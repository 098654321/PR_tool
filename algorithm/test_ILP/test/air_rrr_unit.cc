#include "rrr_air/air_routing.hh"
#include "rrr_air/stop_policy.hh"
#include "graph/unified_routing_graph.hh"

#include <cmath>
#include <stdexcept>

namespace PR_tool::air316 {
namespace {
auto require(bool ok, const char* text) -> void {
    if (!ok) throw std::runtime_error(text);
}
auto close(double a, double b) -> bool { return std::abs(a-b) < 1e-9; }

auto stop_conditions() -> void {
    using namespace std::chrono;
    const auto start = StopPolicy::Clock::now();
    auto policy = StopPolicy{start + seconds(1)};
    require(policy.observe(true, 100), "initial legal input did not establish incumbent");
    require(policy.stagnant == 0 && policy.legal_count == 1, "initial input consumed stagnation count");
    for (int i = 0; i < 9; ++i) {
        policy.observe(false, 1);
        policy.observe(true, i % 2 ? 100 : 110);
        require(policy.reason(start) == nullptr, "stopped before ten nonimproving legal solutions");
    }
    require(policy.stagnant == 9, "illegal rounds changed legal stagnation");
    require(policy.observe(true, 99) && policy.stagnant == 0, "improvement did not reset legal stagnation");
    for (int i = 0; i < 10; ++i) policy.observe(true, 100);
    require(std::string_view(policy.reason(start)) == "legal_stagnation", "ten legal solutions did not stop");
    auto empty = StopPolicy{start};
    require(std::string_view(empty.reason(start)) == "time_budget", "no-legal timeout was ignored");
    require(empty.reason(start-nanoseconds(1)) == nullptr, "deadline comparison stopped too early");
}

auto openroad_cost() -> void {
    auto resources = ResourceModel{};
    const auto key = node_resource(5);
    const auto expected = [](double q) {
        return 1 + 4/(1+std::exp(1-q)) + 0.2*std::max(0.0,q-1);
    };
    require(close(resources.node_cost({0,0}, 5), expected(0)), "OpenROAD used candidate occupancy +1");
    resources.claim({1,0}, {key, switch_resource(7)});
    require(close(resources.node_cost({0,0}, 5), expected(1)), "OpenROAD current usage mismatch");
    require(close(resources.node_cost({1,0}, 5), 0), "retained tree node was charged");
    resources.claim({2,0}, {key, switch_resource(7)});
    resources.history_next();
    require(close(resources.history(key), 1), "OpenROAD history did not accumulate overflow");
    require(close(resources.history(switch_resource(7)), 0), "non-node resource accumulated history");
    resources.release({1,0}); resources.release({2,0});
    require(close(resources.node_cost({0,0},5), expected(0.9)), "history was not inside nonlinear input");
    resources.history_next();
    require(close(resources.history(key),1), "history decayed or was reset");
}

auto physical_graph_parity() -> void {
    auto interposer = hardware::Interposer{};
    const auto ilp = PR_tool::build_unified_graph(&interposer, {});
    const auto air = build_hardware_graph(&interposer, {});
    require(ilp.nodes.size() == air.nodes.size(), "AIR/ILP physical node count mismatch");
    require(ilp.arcs.size() == air.arcs.size(), "AIR/ILP physical arc count mismatch");
    for (std::size_t i = 0; i < ilp.nodes.size(); ++i) {
        const auto& a = ilp.nodes[i]; const auto& b = air.nodes[i];
        require(static_cast<int>(a.kind) == static_cast<int>(b.kind) &&
            std::tie(a.unit,a.track_dir,a.track_row,a.track_col,a.track_index,a.tob,a.bank,a.group,a.line_index) ==
            std::tie(b.unit,b.track_dir,b.track_row,b.track_col,b.track_index,b.tob,b.bank,b.group,b.line_index) &&
            std::tie(a.bump.TOB,a.bump.Bank,a.bump.Group,a.bump.Index) ==
            std::tie(b.bump.TOB,b.bump.Bank,b.bump.Group,b.bump.Index),
            "AIR/ILP physical node mapping changed");
    }
    for (std::size_t i = 0; i < ilp.arcs.size(); ++i) {
        const auto& a = ilp.arcs[i]; const auto& b = air.arcs[i];
        require(std::tie(a.u,a.v,a.is_vline_track_straight,a.is_vline_track_swap,a.mode_group_id,a.physical_switch_id) ==
            std::tie(b.u,b.v,b.is_vline_track_straight,b.is_vline_track_swap,b.mode_group_id,b.physical_switch_id) &&
            static_cast<int>(a.physical_switch_kind) == static_cast<int>(b.physical_switch_kind),
            "AIR/ILP physical arc ordering or attributes differ");
    }
}

auto fixture(int count) -> UnifiedGraph {
    auto g = UnifiedGraph{};
    for (int i = 0; i < count; ++i) {
        auto node = UnifiedNode{}; node.track_col = i;
        g.track_node_by_key[{0,0,0,i,0}] = i;
        g.nodes.push_back(node); g.in_arc_ids.emplace_back(); g.out_arc_ids.emplace_back();
    }
    g.ordered_track_out_arc_ids.resize(count);
    return g;
}
auto edge(UnifiedGraph& g, int a, int b) -> void {
    for (const auto [u,v] : {std::pair{a,b}, std::pair{b,a}}) {
        const int aid = g.arcs.size(); g.arcs.push_back({u,v});
        g.out_arc_ids[u].push_back(aid); g.in_arc_ids[v].push_back(aid);
        g.directed_arc_set.insert({u,v});
    }
}
auto ref(int id) -> GraphNodeRef {
    auto r = GraphNodeRef{};
    r.track_coord = {0,id,hardware::TrackDirection::Horizontal,0};
    return r;
}

auto snapshot_pruning_and_bounds() -> void {
    using namespace rrr_detail;
    auto g = fixture(18); edge(g,0,1);
    auto net = RoutingNet{}; net.sources = {ref(0)};
    for (int i = 2; i < 18; ++i) {
        edge(g,1,i);
        net.demands.push_back({static_cast<std::size_t>(i-2), ref(i), {0}, true});
    }
    auto nets = std::Vector<RoutingNet>{net};
    auto owners = build_owners(nets);
    auto states = std::Vector<AirNetState>{{air_spatial(g,net)}};
    auto resources = ResourceModel{};
    for (int i = 2; i < 18; ++i) owners[0].demand_paths[i-2] = {0,1,i};
    air_refresh(g, owners[0], resources);
    resources.claim({99,0}, {node_resource(1)});
    auto interposer = hardware::Interposer{};
    require(air_prune_illegal(g,nets,owners,resources,states,&interposer) == 1,
            "large net conflict was not pruned");
    for (const auto& p : owners[0].demand_paths)
        require(p == std::Vector<int>{0}, "conflict pruning left an orphan subtree");
    require(!air_connected(g,net,owners[0],0), "retained prefix was mistaken for routed sink");
    auto spatial = air_spatial(g,net);
    require(close(spatial.bin_size, 1 + std::sqrt(4.0*17/16)), "snapshot bin formula changed");
    auto disconnected = fixture(2);
    auto dnet = RoutingNet{}; dnet.sources = {ref(0)}; dnet.demands = {{0,ref(1),{0},true}};
    auto downers = build_owners({dnet});
    auto dstates = std::Vector<AirNetState>{{air_spatial(disconnected,dnet)}};
    const auto before = dstates[0].spatial.router_bb;
    require(air_update_router_boxes(disconnected,{dnet},downers,dstates,{false},&interposer)==0 &&
            dstates[0].spatial.router_bb==before, "failed/unrouted net grew its bbox");
}

auto imported_round_and_timeout() -> void {
    using namespace std::chrono;
    auto g = fixture(2); edge(g,0,1);
    auto net = RoutingNet{}; net.sources = {ref(0)}; net.demands = {{0,ref(1),{0},true}};
    auto interposer = hardware::Interposer{};
    auto params = RrrParams{};
    params.deadline = steady_clock::now() + seconds(1);
    const auto complete = run_rrr(g,{net},params,&interposer,0,nullptr,{{{0,1}}});
    require(complete.status == "success" && complete.stop_reason == "legal_stagnation",
            "imported complete solution did not optimize until legal stagnation");
    require(complete.convergences == 11 && complete.iterations == 11,
            "initial legal input or ten later convergences counted incorrectly");
    params.deadline = steady_clock::now()-nanoseconds(1);
    const auto partial = run_rrr(g,{net},params,&interposer,0,nullptr,{{{}}});
    require(partial.status == "incomplete" && partial.stop_reason == "time_budget" &&
            partial.missing_owners == 1 && partial.iterations == 1,
            "expired partial input continued searching for first legal solution");
    const auto saved = run_rrr(g,{net},params,&interposer,0,nullptr,{{{0,1}}});
    require(saved.status == "success" && saved.paths[0][0] == std::Vector<int>({0,1}),
            "expired budget lost the initial legal incumbent");
}
} // namespace
} // namespace PR_tool::air316

namespace PR_tool {
auto run_air_rrr_tests() -> void {
    air316::stop_conditions(); air316::openroad_cost(); air316::physical_graph_parity();
    air316::snapshot_pruning_and_bounds(); air316::imported_round_and_timeout();
}
} // namespace PR_tool
