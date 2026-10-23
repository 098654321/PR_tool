#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/command_mode/invoker.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/rrr/net_adapter.hh>
#include <algo/router/rrr/path_metrics.hh>
#include <algo/router/rrr/route_validate.hh>
#include <algo/router/rrr/rrr_router.hh>
#include <algo/router/rrr/signal_owners.hh>
#include <parse/reader/module.hh>
#include <fstream>
#include <numeric>
#include <queue>

namespace {
// Independent relaxed shortest-path bound for disjoint Track/Bump sync lanes.
// Ignore congestion, mux constraints and COB directions, so this can only
// underestimate the attainable length. Other net types have no bound here.
auto sync_lower_bound(const PR_tool::rrr::UnifiedGraph& graph,
                      const std::vector<PR_tool::rrr::RoutingNet>& nets) -> std::optional<std::size_t> {
    using namespace PR_tool::rrr;
    std::set<OwnerId> signals;
    const auto aliases=signal_owners(graph,nets);
    std::size_t total=0;
    for(const auto& net:nets) {
        if(!net.is_sync_bus)return {};
        int maximum=0;
        for(const auto& demand:net.demands) {
            if(demand.candidate_source_indices.size()!=1)return {};
            auto signal=demand_owner(net,demand);
            if(const auto it=aliases.find(signal);it!=aliases.end())signal=it->second;
            if(!signals.insert(signal).second)return {};
            const int source=resolve_graph_node(graph,net.sources.at(demand.candidate_source_indices[0]));
            const int sink=resolve_graph_node(graph,demand.sink);
            if(graph.nodes[source].kind!=UnifiedNodeKind::Track || graph.nodes[sink].kind!=UnifiedNodeKind::Bump)return {};
            std::vector<int> distance(graph.nodes.size(),1000000);
            using Item=std::pair<int,int>;
            std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
            distance[source]=1;queue.emplace(1,source);
            while(!queue.empty()) {
                const auto [cost,node]=queue.top();queue.pop();
                if(cost!=distance[node])continue;
                if(node==sink)break;
                for(int id:graph.out_arc_ids[node]) {
                    const int next=graph.arcs[id].v;
                    const int value=cost+(graph.nodes[next].kind==UnifiedNodeKind::Track);
                    if(value<distance[next]){distance[next]=value;queue.emplace(value,next);}
                }
            }
            maximum=std::max(maximum,distance[sink]);
        }
        total+=(maximum+1)*net.demands.size(); // one Bump per lane
    }
    return total;
}
}

// Exercise the unchanged command chain directly, then continue from its exact
// paths and sorted order. No short routing deadline is an acceptance criterion.
void test_rrr_benchmark_main(int argc, char** argv) {
    using namespace PR_tool;
    if(argc<4)throw std::runtime_error("rrr_benchmark CONFIG OUTPUT [first-pass|optimize]");
    const std::filesystem::path output=argv[3];
    std::filesystem::create_directories(output);
    debug::initial_log(output/"benchmark.log");
    const auto start=std::chrono::steady_clock::now();
    auto [interposer,basedie,register_map]=parse::read_config(argv[2],0,false);
    algo::build_nets(basedie.get(),interposer.get());
    algo::MazeRouteStrategy strategy;
    algo::HK allocator;
    algo::RouteEngine engine(basedie->nets(),strategy,allocator,0,false,false,false,interposer.get());
    engine.stop_on_first_failure=true;
    algo::Invoker invoker;
    invoker.set_route_commands(false,false,false);
    invoker.invoke(interposer.get(),engine);
    const auto raw=engine.nets();
    const bool complete=engine.position()==raw.size() && engine.failed_net_names().empty();
    std::size_t legacy_length=0;
    for(auto* net:raw)legacy_length+=net->length();
    debug::info_fmt("First maze pass: complete={} routed={}/{} wirelength={}",
        complete,engine.position(),raw.size(),legacy_length);
    if(!complete)throw std::runtime_error("original first maze pass did not complete all nets");
    std::Vector<std::Rc<circuit::Net>> sorted;
    for(auto* net:raw)for(const auto& shared:basedie->nets().at(0))
        if(shared.get()==net){sorted.push_back(shared);break;}
    const auto nets=rrr::build_routing_nets(sorted);
    const auto graph=rrr::build_hardware_graph(interposer.get(),nets);
    const auto initial=rrr::import_paths(interposer.get(),graph,nets,raw);
    rrr::RrrResult warm;warm.paths=initial;
    const bool physical_legal=rrr::validate_rrr_solution(graph,nets,warm,interposer.get());
    const auto initial_length=rrr::total_wirelength(graph,initial);
    std::ofstream report(output/"metrics.tsv");
    report<<"first_complete\tfirst_physical_legal\tfirst_length\tfirst_legal_length\tbest_length\tlegal_solutions\telapsed_ms\n";
    report<<complete<<'\t'<<physical_legal<<'\t'<<initial_length<<std::flush;
    debug::info_fmt("First maze pass validation: physical_legal={} wirelength={}",physical_legal,initial_length);
    const auto lower_bound=sync_lower_bound(graph,nets);
    if(lower_bound)debug::info_fmt("Independent relaxed Sync wirelength lower bound: {}",*lower_bound);
    if(argc>4 && std::string(argv[4])=="first-pass") {report<<"\t-\t-\t-\t-\n";return;}
    std::vector<std::size_t> order(nets.size());
    std::iota(order.begin(),order.end(),0);
    const auto result=rrr::run_rrr(graph,nets,rrr::RrrParams{},interposer.get(),initial,order);
    const auto elapsed=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
    report<<'\t'<<result.first_legal_wirelength<<'\t'<<result.total_wirelength<<'\t'<<result.convergences<<'\t'<<elapsed<<'\n';
    report.flush();
    if(result.status!="success" || !rrr::validate_rrr_solution(graph,nets,result,interposer.get()))
        throw std::runtime_error("RRR benchmark has no physically legal solution");
    if(result.convergences<3)throw std::runtime_error("RRR benchmark needs at least three legal solutions");
    // An electrically invalid warm path is not a legal length baseline.
    const bool proven_minimum=lower_bound && result.total_wirelength==*lower_bound;
    if(result.total_wirelength>=result.first_legal_wirelength && !proven_minimum)
        throw std::runtime_error("RRR did not shorten its first physically legal solution");
    rrr::commit_paths(interposer.get(),graph,nets,result.paths,raw);
    if(rrr::import_paths(interposer.get(),graph,nets,raw)!=result.paths)
        throw std::runtime_error("RRR benchmark commit round trip changed paths");
    debug::info(proven_minimum
        ? "RRR benchmark PASS: first pass completed; legal result reaches independent lower bound"
        : "RRR benchmark PASS: first pass completed; legal iterations shortened wirelength");
}
