#include <algo/router/rrr/resource_model.hh>
#include "test_support.hh"
#include "tob_mux_fanout.hh"
#include <algo/router/rrr/path_metrics.hh>
#include <algo/router/common/routing_budget.hh>
#include <stdexcept>

extern void test_rrr_core();
extern void test_rrr_air();
extern void test_rrr_adapters();
extern void test_rrr_handoff();

namespace {
auto require(bool value, const char* message) -> void {
    if (!value) throw std::runtime_error(message);
}
}

void test_rrr_main() {
    using namespace PR_tool;
    using namespace PR_tool::rrr;
    LegalConvergence c;
    require(c.observe(100) && c.streak == 1,"first legal counts as one");
    require(!c.observe(100) && c.streak == 2,"second equal legal");
    require(c.observe(90) && c.streak == 1,"improvement starts a new streak at one");
    require(!c.observe(100) && c.streak == 2,"worse legal retains best");
    require(!c.observe(90) && c.converged() && c.best == 90,"third legal converges");
    {
        ResourceModel shared;
        const OwnerId a{0,0},b{1,0},independent{2,0};
        shared.set_signal_owners({{a,a},{b,a}});
        shared.claim(a,{node_resource(7)});shared.claim(b,{node_resource(7)});
        require(shared.overflow()==0,"same driver may share capacity");
        shared.release(a);
        require(shared.holds(b,node_resource(7)) && shared.occupancy_count(node_resource(7))==1,
                "releasing one related branch must retain the other's claims");
        shared.claim(independent,{node_resource(7)});
        require(shared.overflow()==1,"different drivers must still conflict");
        shared.release(b);shared.release(independent);
        require(shared.occupancy_count(node_resource(7))==0,"all shared claims release cleanly");
    }
    const auto now = algo::RoutingBudget::Clock::now();
    {
        algo::RoutingBudget b(2.0,now);
        require(!b.expired(now+std::chrono::seconds(1)),"budget before deadline");
        require(b.expired(now+std::chrono::seconds(2)),"budget at deadline");
    }
    {
        algo::RoutingBudget b(0.01,now-std::chrono::seconds(1));
        bool cancelled = false;
        try { algo::RoutingBudget::check(); } catch (const algo::RoutingTimeout&) { cancelled = true; }
        require(cancelled,"expired search must cancel");
        { algo::RoutingBudget cleanup; algo::RoutingBudget::check(); }
    }
    algo::RoutingBudget::check();
    UnifiedGraph graph;
    hardware::Interposer interposer;
    RrrParams params; params.time_budget_seconds=0.01;
    params.budget_start=now-std::chrono::seconds(1);
    RoutingNet net; net.net_id=0;
    GraphNodeRef source; source.kind=GraphNodeRef::Kind::Track;
    source.track_coord={0,0,hardware::TrackDirection::Horizontal,0};
    auto sink=source; sink.track_coord.col=1;
    net.sources={source}; net.demands={{0,sink,{0},true}};
    for (int col=0;col<2;++col) {
        UnifiedNode n; n.track_col=col; graph.nodes.push_back(n);
        graph.track_node_by_key[{0,0,0,col,0}]=col;
        graph.in_arc_ids.emplace_back();graph.out_arc_ids.emplace_back();
    }
    graph.arcs.push_back({0,1});graph.out_arc_ids[0]={0};graph.in_arc_ids[1]={0};
    graph.directed_arc_set.insert({0,1});
    const auto failed=run_empty(graph,{net},params,&interposer);
    require(failed.status=="no_legal_solution" && failed.iterations==0,"expired budget without incumbent must not search");
    const auto saved=run_rrr(graph,{net},params,&interposer,{{{0,1}}},{0});
    require(saved.status=="success" && saved.paths==NetPaths{{{0,1}}} && saved.convergences==1,
            "expired budget returns the first-pass incumbent");
    test_rrr_handoff();
    test_rrr_adapters();
    test_rrr_core();
    test_rrr_air();
    PR_tool::rrr::run_tob_mux_fanout_unit_tests();
}
