#include "hardware_graph.hh"
#include "maze_search.hh"
#include "net_adapter.hh"
#include "resource_model.hh"
#include "route_log.hh"
#include "route_validate.hh"
#include "rrr_cli.hh"
#include "rrr_router.hh"
#include "sync_equalize.hh"
#include "test/tob_mux_fanout.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <hardware/bump/bump.hh>
#include <hardware/tob/tob.hh>
#include <hardware/track/track.hh>
#include <parse/reader/module.hh>

#include <cmath>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

using namespace PR_tool;

auto require(bool condition, const std::string& message) -> void {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

auto require_invalid(std::initializer_list<std::string_view> args, const std::string& context)
    -> void {
    try {
        (void)parse_rrr_cli(args);
        require(false, context);
    }
    catch (const std::invalid_argument&) {
    }
}

auto test_cli_missing_args_fail() -> void {
    require_invalid({}, "missing args must fail");
}

auto test_cli_output_dir_option() -> void {
    const auto parsed = parse_rrr_cli({"test/config/case1", "-o", "output/rrr_run"});
    require(parsed.config_path == "test/config/case1", "CLI must preserve the config path");
    require(parsed.output_dir == "output/rrr_run", "CLI must parse -o output directory");

    const auto defaults = parse_rrr_cli({"test/config/case1"});
    require(defaults.output_dir == ".", "missing -o must keep the default output directory");

    require_invalid({"test/config/case1", "-o"}, "missing -o directory must fail");
    require_invalid({"test/config/case1", "-o", "-v"}, "CLI must reject -o values that look like flags");
}

auto test_cli_max_iterations_option() -> void {
    const auto parsed = parse_rrr_cli({"test/config/case1", "--max-iterations", "8"});
    require(parsed.max_iterations == 8, "CLI must parse --max-iterations");

    const auto defaults = parse_rrr_cli({"test/config/case1"});
    require(defaults.max_iterations == kRrrDefaultMaxIterations, "missing --max-iterations must keep large safety cap");

    const auto zero = parse_rrr_cli({"test/config/case1", "--max-iterations", "0"});
    require(zero.max_iterations == 0, "CLI must accept --max-iterations 0");

    require_invalid(
        {"test/config/case1", "--max-iterations"},
        "missing --max-iterations value must fail");
    require_invalid(
        {"test/config/case1", "--max-iterations", "-1"},
        "negative --max-iterations must fail");
    require_invalid(
        {"test/config/case1", "--max-iterations", "x"},
        "non-integer --max-iterations must fail");
}

auto test_cli_seed_option() -> void {
    const auto parsed = parse_rrr_cli({"test/config/case1", "--seed", "7"});
    require(parsed.seed == 7, "CLI must parse --seed");

    const auto defaults = parse_rrr_cli({"test/config/case1"});
    require(defaults.seed == 1, "missing --seed must keep default 1");

    require_invalid({"test/config/case1", "--seed"}, "missing --seed value must fail");
    require_invalid({"test/config/case1", "--seed", "-2"}, "negative --seed must fail");
}

auto test_cli_verbose_options() -> void {
    const auto one = parse_rrr_cli({"test/config/case1", "-v"});
    require(one.verbose_level == 1, "CLI must parse -v");

    const auto two = parse_rrr_cli({"test/config/case1", "-vv"});
    require(two.verbose_level == 2, "CLI must parse -vv");

    const auto combined = parse_rrr_cli({
        "algorithm/test_ILP/test/case_2btb",
        "-vv",
        "-o",
        "/tmp/fpia_rrr_t1",
        "--max-iterations",
        "16",
        "--seed",
        "3"});
    require(
        combined.config_path == "algorithm/test_ILP/test/case_2btb",
        "CLI must preserve config path with combined options");
    require(combined.verbose_level == 2, "CLI must preserve -vv with other options");
    require(combined.output_dir == "/tmp/fpia_rrr_t1", "CLI must parse -o with other options");
    require(combined.max_iterations == 16, "CLI must parse --max-iterations with other options");
    require(combined.seed == 3, "CLI must parse --seed with other options");

    require_invalid({"test/config/case1", "--unknown"}, "unknown arguments must fail");
}

auto load_routing_nets(const std::string& config_path) -> std::Vector<RoutingNet> {
    auto [interposer, basedie] = parse::read_config(config_path, 0, false);
    algo::build_nets(basedie.get(), interposer.get());
    return build_routing_nets(basedie->nets_to_vector());
}

auto require_single_source_demands(const RoutingNet& net, const std::string& context) -> void {
    require(!net.demands.empty(), context + " must have at least one demand");
    for (const auto& demand : net.demands) {
        require(
            demand.candidate_source_indices.size() == 1,
            context + " non-PNnet demand must have exactly one candidate source");
    }
}

auto test_case_2btb_bnets() -> void {
    const auto nets = load_routing_nets("algorithm/test_ILP/test/case_2btb");
    require(nets.size() == 2, "case_2btb must produce two RoutingNets");
    for (const auto& net : nets) {
        require(net.kind == RoutingNetKind::Bnet, "case_2btb nets must be Bnet");
        require(!net.is_sync_bus, "case_2btb nets must not be sync buses");
        require(net.sources.size() == 1, "case_2btb Bnet must have one bump source");
        require(net.demands.size() == 1, "case_2btb Bnet must have one demand");
        require(net.sources[0].kind == GraphNodeRef::Kind::Bump, "case_2btb source must be a bump");
        require(net.demands[0].sink.kind == GraphNodeRef::Kind::Bump, "case_2btb sink must be a bump");
        require_single_source_demands(net, "case_2btb");
    }
}

auto test_case_2btt_tnets() -> void {
    const auto nets = load_routing_nets("algorithm/test_ILP/test/case_2btt");
    require(nets.size() == 2, "case_2btt must produce two RoutingNets");
    for (const auto& net : nets) {
        require(net.kind == RoutingNetKind::Tnet, "case_2btt nets must be Tnet");
        require(!net.is_sync_bus, "case_2btt nets must not be sync buses");
        require(net.sources.size() == 1, "case_2btt Tnet must have one track source");
        require(net.demands.size() == 1, "case_2btt Tnet must have one demand");
        require(net.sources[0].kind == GraphNodeRef::Kind::Track, "case_2btt source must be a track");
        require(net.demands[0].sink.kind == GraphNodeRef::Kind::Bump, "case_2btt sink must be a bump");
        require_single_source_demands(net, "case_2btt");
    }
}

auto test_case_2fanout_tnet() -> void {
    const auto nets = load_routing_nets("algorithm/test_ILP/test/case_2fanout");
    require(nets.size() == 1, "case_2fanout must produce one RoutingNet");
    const auto& net = nets[0];
    require(net.kind == RoutingNetKind::Tnet, "case_2fanout net must be Tnet");
    require(!net.is_sync_bus, "case_2fanout net must not be a sync bus");
    require(net.sources.size() == 1, "case_2fanout Tnet must keep one source");
    require(net.demands.size() == 2, "case_2fanout Tnet must have one demand per sink");
    require(net.sources[0].kind == GraphNodeRef::Kind::Track, "case_2fanout source must be a track");
    require_single_source_demands(net, "case_2fanout");
    for (const auto& demand : net.demands) {
        require(demand.fixed_pair, "case_2fanout demands must be fixed pairs");
        require(demand.sink.kind == GraphNodeRef::Kind::Bump, "case_2fanout sink must be a bump");
    }
}

auto test_case_bus2btb_sync_bus() -> void {
    const auto nets = load_routing_nets("algorithm/test_ILP/test/case_bus2btb");
    require(nets.size() == 1, "case_bus2btb must produce one RoutingNet");
    const auto& net = nets[0];
    require(net.kind == RoutingNetKind::Bnet, "case_bus2btb SyncNet must map to Bnet");
    require(net.is_sync_bus, "case_bus2btb SyncNet must set is_sync_bus");
    require(net.sources.size() == 2, "case_bus2btb SyncNet must keep one source per member");
    require(net.demands.size() == 2, "case_bus2btb SyncNet must keep one demand per member");
    require_single_source_demands(net, "case_bus2btb");
    for (const auto& demand : net.demands) {
        require(demand.fixed_pair, "case_bus2btb member pairings must remain fixed");
        require(demand.sink.kind == GraphNodeRef::Kind::Bump, "case_bus2btb sink must be a bump");
    }
}

auto test_case5_pnnet_and_kinds() -> void {
    const auto nets = load_routing_nets("test/config/case5");
    std::size_t bnet = 0;
    std::size_t tnet = 0;
    std::size_t pnnet = 0;
    std::size_t sync_bus = 0;
    std::size_t bus8 = 0;
    std::size_t bus4 = 0;
    std::size_t regular_bnet = 0;
    std::size_t io_fanout = 0;

    for (const auto& net : nets) {
        if (net.is_sync_bus) {
            ++sync_bus;
            require(net.kind == RoutingNetKind::Bnet, "case5 buses must be Bnet SyncNets");
            if (net.demands.size() == 8) {
                ++bus8;
            } else if (net.demands.size() == 4) {
                ++bus4;
            } else {
                require(false, "case5 bus demand count must be 4 or 8");
            }
            require_single_source_demands(net, "case5 sync bus");
            continue;
        }
        if (net.kind == RoutingNetKind::Bnet) {
            ++bnet;
            ++regular_bnet;
            require(net.demands.size() == 1, "case5 regular Bnet must have one demand");
            require_single_source_demands(net, "case5 regular Bnet");
        } else if (net.kind == RoutingNetKind::Tnet) {
            ++tnet;
            require(net.sources.size() == 1, "case5 Tnet must have one track source");
            require(net.demands.size() == 4, "case5 IO Tnet must fan out to four bumps");
            ++io_fanout;
            require_single_source_demands(net, "case5 Tnet");
        } else if (net.kind == RoutingNetKind::PNnet) {
            ++pnnet;
            require(!net.is_sync_bus, "case5 PNnet must not be a sync bus");
            require(net.sources.size() > 1, "case5 PNnet must keep multiple candidate sources");
            require(net.sources.size() == 12, "case5 PNnet must keep all twelve 0/1 tracks");
            require(net.demands.size() == 1, "case5 pose/nege PNnet must have one sink bump");
            require(
                net.demands[0].candidate_source_indices.size() == net.sources.size(),
                "case5 PNnet demand must list every candidate source");
            require(!net.demands[0].fixed_pair, "case5 PNnet demand must not be a fixed pair");
            require(net.sources[0].kind == GraphNodeRef::Kind::Track, "case5 PNnet sources must be tracks");
            require(net.demands[0].sink.kind == GraphNodeRef::Kind::Bump, "case5 PNnet sink must be a bump");
        } else {
            require(false, "case5 produced an unexpected RoutingNetKind");
        }
    }

    require(regular_bnet == 32, "case5 must have 32 regular Bnets");
    require(bnet == 32, "case5 non-sync Bnet count must be 32");
    require(tnet == 16, "case5 must have 16 IO Tnets");
    require(io_fanout == 16, "case5 must have 16 four-sink IO fanouts");
    require(pnnet == 2, "case5 must have pose and nege PNnets");
    require(sync_bus == 16, "case5 must have 16 SyncNets");
    require(bus8 == 4, "case5 must have four 8-bit buses");
    require(bus4 == 12, "case5 must have twelve 4-bit buses");
    require(nets.size() == 66, "case5 must produce 66 RoutingNets");
}

auto test_mixed_sync_net_is_error() -> void {
    hardware::TOB tob0 {0, 0};
    hardware::TOB tob1 {1, 1};
    hardware::Bump bump0 {hardware::BumpCoord {0, 0, 0}, &tob0};
    hardware::Bump bump1 {hardware::BumpCoord {0, 0, 1}, &tob1};
    hardware::Track track0 {6, 3, hardware::TrackDirection::Vertical, 4};

    std::String btb_name {"mixed_btb"};
    std::String btb_uid {"mixed_btb_uid"};
    auto btb = std::make_shared<circuit::BumpToBumpNet>(
        &bump0,
        &bump1,
        std::HashSet<int> {0},
        btb_name,
        btb_uid);
    std::String btt_name {"mixed_btt"};
    std::String btt_uid {"mixed_btt_uid"};
    auto btt = std::make_shared<circuit::BumpToTrackNet>(
        &bump0,
        &track0,
        std::HashSet<int> {0},
        btt_name,
        btt_uid);
    std::String mixed_name {"mixed_sync"};
    std::String mixed_uid {"mixed_sync_uid"};
    auto mixed = std::make_shared<circuit::SyncNet>(
        std::Vector<std::Rc<circuit::BumpToBumpNet>> {btb},
        std::Vector<std::Rc<circuit::BumpToTrackNet>> {btt},
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

auto test_case_2btb_hardware_graph() -> void {
    auto [interposer, basedie] = parse::read_config("algorithm/test_ILP/test/case_2btb", 0, false);
    algo::build_nets(basedie.get(), interposer.get());
    const auto nets = build_routing_nets(basedie->nets_to_vector());
    const auto graph = build_hardware_graph(interposer.get(), nets);

    require(!graph.nodes.empty(), "case_2btb graph must contain nodes");
    require(!graph.arcs.empty(), "case_2btb graph must contain arcs");

    for (const auto& node : graph.nodes) {
        const bool physical = node.kind == UnifiedNodeKind::Track
            || node.kind == UnifiedNodeKind::Bump
            || node.kind == UnifiedNodeKind::HLine
            || node.kind == UnifiedNodeKind::VLine;
        require(physical, "case_2btb graph must have Track/Bump/HLine/VLine only");
    }

    for (const auto& arc : graph.arcs) {
        require(
            graph.directed_arc_set.contains({arc.u, arc.v}),
            "every listed arc must exist in directed_arc_set");
    }

    for (const auto& net : nets) {
        for (const auto& source : net.sources) {
            require(resolve_graph_node(graph, source) >= 0, "case_2btb source must resolve on the graph");
        }
        for (const auto& demand : net.demands) {
            require(resolve_graph_node(graph, demand.sink) >= 0, "case_2btb sink must resolve on the graph");
        }
    }
}

auto test_synthetic_track_bump_wirelength() -> void {
    auto graph = UnifiedGraph {};
    graph.nodes.resize(4);
    graph.nodes[0].kind = UnifiedNodeKind::Track;
    graph.nodes[1].kind = UnifiedNodeKind::HLine;
    graph.nodes[2].kind = UnifiedNodeKind::Bump;
    graph.nodes[3].kind = UnifiedNodeKind::VLine;

    const auto path = std::Vector<int> {0, 1, 2, 3};
    auto unique_track_bump = std::set<int> {};
    for (const int node_id : path) {
        const auto kind = graph.nodes[static_cast<std::size_t>(node_id)].kind;
        if (kind == UnifiedNodeKind::Track || kind == UnifiedNodeKind::Bump) {
            unique_track_bump.insert(node_id);
        }
    }

    require(
        path_wirelength(graph, path) == unique_track_bump.size(),
        "synthetic Track+Bump path wirelength must equal unique Track+Bump node count");
    require(
        net_wirelength(graph, {path}) == unique_track_bump.size(),
        "net wirelength must unique-count Track+Bump nodes");
    require(
        total_wirelength(graph, std::Vector<std::Vector<std::Vector<int>>> {{path}})
            == unique_track_bump.size(),
        "total wirelength must unique-count Track+Bump nodes");
}

auto test_total_wirelength_sums_per_net_unique() -> void {
    auto graph = UnifiedGraph {};
    graph.nodes.resize(3);
    graph.nodes[0].kind = UnifiedNodeKind::Track;
    graph.nodes[1].kind = UnifiedNodeKind::Bump;
    graph.nodes[2].kind = UnifiedNodeKind::Bump;

    const auto path_a = std::Vector<int> {0, 1};
    const auto path_b = std::Vector<int> {0, 2};

    require(net_wirelength(graph, {path_a}) == 2, "net A unique Track+Bump count must be 2");
    require(net_wirelength(graph, {path_b}) == 2, "net B unique Track+Bump count must be 2");
    require(
        total_wirelength(
            graph,
            std::Vector<std::Vector<std::Vector<int>>> {{path_a}, {path_b}})
            == 4,
        "total wirelength must sum per-net unique counts (4), not global unique (3)");

    const auto repeated = std::Vector<int> {0, 1, 0};
    require(
        net_wirelength(graph, {repeated}) == 2,
        "net wirelength must unique-count a repeated Track once");
    require(
        path_wirelength(graph, repeated) == 3,
        "path wirelength must count a repeated Track with multiplicity");
}

} // namespace

auto run_rrr_adapter_unit_tests() -> void {
    test_cli_missing_args_fail();
    test_cli_output_dir_option();
    test_cli_max_iterations_option();
    test_cli_seed_option();
    test_cli_verbose_options();
    test_case_2btb_bnets();
    test_case_2btt_tnets();
    test_case_2fanout_tnet();
    test_case_bus2btb_sync_bus();
    test_case5_pnnet_and_kinds();
    test_mixed_sync_net_is_error();
    test_case_2btb_hardware_graph();
    test_synthetic_track_bump_wirelength();
    test_total_wirelength_sums_per_net_unique();
}
