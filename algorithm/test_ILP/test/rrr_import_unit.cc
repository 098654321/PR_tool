#include "route_ilp/route_rrr.hh"
#include "direct_ilp/direct_validate.hh"
#include "rrr_air/import_bbox.hh"
#include <circuit/net/types/bbnet.hh>

#include <stdexcept>

namespace PR_tool {
namespace {
namespace air = air316;
auto require_import(bool ok, const char* text) -> void {
    if (!ok) throw std::runtime_error(text);
}

auto mode0_order_and_path_identity() -> void {
    auto interposer = hardware::Interposer{};
    auto ilp_raw = std::Vector<std::Rc<circuit::Net>>{};
    auto nets = std::Vector<RoutingNet>{};
    auto graph = UnifiedGraph{};
    auto scope = RoutingScope{};
    auto baseline = RouteIlpResult{}; baseline.has_integer_solution = true;
    // Equal priorities and duplicate names: identity, not a label, must map paths.
    for (int i = 0; i < 18; ++i) {
        auto name = std::String{"same-name"}; auto uid = std::to_string(i);
        ilp_raw.push_back(std::make_shared<circuit::BumpToBumpNet>(
            *interposer.get_bump(0,0,2*i), *interposer.get_bump(0,0,2*i+1),
            std::HashSet<int>{0}, name, uid));
        auto net = RoutingNet{}; net.net_id = i; net.name = name;
        for (int j = 0; j < 2; ++j) {
            const int id = 2*i+j;
            auto node = UnifiedNode{}; node.track_col = id;
            graph.nodes.push_back(node); graph.out_arc_ids.emplace_back(); graph.in_arc_ids.emplace_back();
            graph.track_node_by_key[{0,0,0,id,0}] = id;
            scope.node_ids.push_back(id); scope.node_offset.push_back(id);
        }
        auto source = GraphNodeRef{}, sink = GraphNodeRef{};
        source.track_coord = {0,2*i,hardware::TrackDirection::Horizontal,0};
        sink.track_coord = {0,2*i+1,hardware::TrackDirection::Horizontal,0};
        net.sources = {source}; net.demands = {{0,sink,{0},true}}; nets.push_back(net);
        for (const auto [u,v] : {std::pair{2*i,2*i+1},std::pair{2*i+1,2*i}}) {
            const int id = graph.arcs.size(); graph.arcs.push_back({u,v});
            graph.out_arc_ids[u].push_back(id); graph.in_arc_ids[v].push_back(id);
            scope.arc_ids.push_back(id); scope.arc_offset.push_back(id);
        }
        baseline.route.paths.push_back({static_cast<std::size_t>(i),0,0,-1,{2*i,2*i+1}});
    }
    baseline.route.total_wirelength = 36;
    auto mode0 = ilp_raw; std::reverse(mode0.begin(),mode0.end());
    const auto indices = rrr_mode0_indices(ilp_raw,mode0);
    for (std::size_t i = 0; i < indices.size(); ++i)
        require_import(indices[i] == indices.size()-1-i, "mode-0 raw order was not preserved by identity");
    auto scopes = std::Vector<RoutingScope>{};
    for (std::size_t i = 0; i < nets.size(); ++i) { auto s = scope; s.net_id = i; scopes.push_back(s); }
    const auto result = optimize_route_columns_rrr(graph,nets,scopes,baseline,&interposer,0,
        std::chrono::steady_clock::now()-std::chrono::seconds(1),&mode0,indices);
    require_import(result.ok && result.paths.size() == nets.size(), "mode-0 import lost a net");
    for (const auto& path : result.paths)
        require_import(path.node_path == std::Vector<int>({static_cast<int>(2*path.net_id),static_cast<int>(2*path.net_id+1)}),
                       "mode-0 reorder attached a path to another ILP net ID");
    require_import(validate_direct_route(graph,nets,scopes,result), "mode-0 reordered output was invalid");
}

auto imported_bbox() -> void {
    using namespace air::rrr_detail;
    auto graph = air::UnifiedGraph{};
    // Leave device room on every side so independent expansion is observable.
    graph.rows = 12; graph.cols = 14;
    for (int i = 0; i < 4; ++i) {
        auto n = air::UnifiedNode{};
        n.track_row = i == 1 ? 1 : i == 2 ? 10 : 5;
        n.track_col = i == 1 ? 1 : i == 2 ? 11 : i == 0 ? 5 : 7;
        graph.nodes.push_back(n); graph.out_arc_ids.emplace_back(); graph.in_arc_ids.emplace_back();
        graph.track_node_by_key[{0,0,n.track_row,n.track_col,0}] = i;
    }
    for (const auto [u,v] : {std::pair{0,1},std::pair{1,2},std::pair{2,3}}) {
        const int id = graph.arcs.size(); graph.arcs.push_back({u,v});
        graph.out_arc_ids[u].push_back(id); graph.in_arc_ids[v].push_back(id); graph.directed_arc_set.insert({u,v});
    }
    auto source = air::GraphNodeRef{}, sink = air::GraphNodeRef{};
    source.track_coord = {5,5,hardware::TrackDirection::Horizontal,0};
    sink.track_coord = {5,7,hardware::TrackDirection::Horizontal,0};
    auto net = air::RoutingNet{}; net.sources = {source}; net.demands = {{0,sink,{0},true}};
    auto owners = build_owners({net}); owners[0].demand_paths[0] = {0,1,2,3};
    auto states = std::Vector<AirNetState>{{air_spatial(graph,net)}};
    const auto initial = states[0].spatial;
    require_import(initial.router_bb == air::RoutingBox{4,6,4,8}, "initial box was not terminal bbox+1");
    air_import_boxes(graph,{net},owners,states);
    require_import(states[0].spatial.router_bb == air::RoutingBox{1,10,1,11}, "import box did not contain all ILP nodes");
    require_import(states[0].spatial.bin_size == initial.bin_size && states[0].spatial.initial == initial.initial,
                   "import adjustment changed spatial bins");
    auto resources = air::ResourceModel{}; air_refresh(graph,owners[0],resources);
    auto interposer = hardware::Interposer{};
    require_import(air_update_router_boxes(graph,{net},owners,states,{true},&interposer) == 1 &&
        states[0].spatial.router_bb == air::RoutingBox{0,11,0,12}, "imported route did not use original d==0 expansion");
    // Missing sinks do not prevent containing an existing legal prefix.
    owners[0].demand_paths[0] = {0,1}; states[0].spatial = initial;
    air_import_boxes(graph,{net},owners,states);
    require_import(states[0].spatial.router_bb == air::RoutingBox{1,6,1,8}, "partial input path was cropped");
    states[0].spatial = initial; owners[0].demand_paths[0] = {0,3};
    air_import_boxes(graph,{net},owners,states);
    require_import(states[0].spatial.router_bb == initial.router_bb, "contained import unnecessarily enlarged box");
}
} // namespace

auto run_rrr_import_tests() -> void {
    mode0_order_and_path_identity(); imported_bbox();
}
} // namespace PR_tool
