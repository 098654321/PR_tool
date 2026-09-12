#include "hardware_graph.hh"
#include "maze_search.hh"
#include "net_adapter.hh"
#include "resource_model.hh"
#include "route_log.hh"
#include "route_validate.hh"
#include "rrr_cli.hh"
#include "rrr_router.hh"
#include "sync_equalize.hh"

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
    require(defaults.max_iterations == 64, "missing --max-iterations must keep default 64");

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

auto require_near(double actual, double expected, const std::string& message) -> void {
    require(std::abs(actual - expected) < 1e-9, message);
}

auto contains_owner(const std::Vector<OwnerId>& owners, OwnerId owner) -> bool {
    for (const auto& item : owners) {
        if (item == owner) {
            return true;
        }
    }
    return false;
}

auto test_resource_claim_release() -> void {
    const auto defaults = RrrParams {};
    require(defaults.H == 4, "RrrParams H default must be 4");
    require(defaults.k == 1.0, "RrrParams k default must be 1.0");
    require(defaults.s == 20, "RrrParams s default must be 20");
    require(defaults.decay == 0.9, "RrrParams decay default must be 0.9");
    require(defaults.increment == 1, "RrrParams increment default must be 1");
    require(defaults.history_weight == 1, "RrrParams history_weight default must be 1");
    require(defaults.detour_bias == 0, "RrrParams detour_bias default must be 0");
    require(defaults.max_iterations == 64, "RrrParams max_iterations default must be 64");
    require(defaults.stagnation_limit == 8, "RrrParams stagnation_limit default must be 8");
    require(defaults.sync_tail_extra_tracks == 64, "RrrParams sync tail slack must be 64 tracks");
    require(defaults.seed == 1, "RrrParams seed default must be 1");
    require(defaults.r_sequence.size() == 3, "RrrParams r sequence must have three values");
    require(defaults.r_sequence[0] == 0.5, "RrrParams r sequence must start at 0.5");
    require(defaults.r_sequence[1] == 0.75, "RrrParams r sequence must include 0.75");
    require(defaults.r_sequence[2] == 1.0, "RrrParams r sequence must end at 1.0");

    auto model = ResourceModel {};
    const auto owner = OwnerId {1, 0};
    const auto node = node_resource(10);
    const auto sw = switch_resource(3);
    model.claim(owner, {node, sw});
    require(model.owners_of(node).size() == 1, "claim must record the node owner");
    require(model.owners_of(node)[0] == owner, "claimed node owner must match");
    require(model.owners_of(sw).size() == 1, "claim must record the switch owner");
    require(model.overflow() == 0, "single owner must not overflow");
    require(model.type_weight(node) == 1, "node type_weight must be 1");
    require(model.type_weight(sw) == 2, "switch type_weight must be 2");
    require(model.type_weight(matching_endpoint_key(10, 0)) == 2, "matching type_weight must be 2");
    require(model.type_weight(mode_conflict_key(1)) == 8, "mode-conflict type_weight must be 8");

    model.claim(owner, {node});
    require(model.owners_of(node).size() == 1, "same owner claiming twice occupies once");
    require(model.overflow() == 0, "same-owner double claim must not overflow");

    model.release(owner);
    require(model.owners_of(node).empty(), "release(owner) must drop all of the owner's keys");
    require(model.owners_of(sw).empty(), "release(owner) must drop switch occupancy");
    require(model.overflow() == 0, "released resources must not overflow");
}

auto test_resource_same_owner_branch_share() -> void {
    auto model = ResourceModel {};
    const auto owner = OwnerId {4, 0};
    const auto shared = node_resource(1);
    const auto branch_a = node_resource(2);
    const auto branch_b = node_resource(3);

    model.claim(owner, {shared, branch_a});
    model.claim(owner, {shared, branch_b});
    require(model.overflow() == 0, "fanout share by one owner must not overflow");
    require(model.owners_of(shared).size() == 1, "shared key must stay occupied once");

    model.release(owner, {shared, branch_a});
    require(
        model.owners_of(shared).size() == 1,
        "releasing one branch must keep a shared key held by the other branch");
    require(model.owners_of(branch_a).empty(), "released branch-only key must be free");
    require(model.owners_of(branch_b).size() == 1, "remaining branch key must stay occupied");

    model.release(owner, {shared, branch_b});
    require(model.owners_of(shared).empty(), "shared key must drop after the last branch release");
    require(model.owners_of(branch_b).empty(), "last branch key must be free");
}

auto test_resource_cross_owner_overflow() -> void {
    auto model = ResourceModel {};
    const auto node = node_resource(8);
    const auto a = OwnerId {1, 0};
    const auto b = OwnerId {2, 0};
    model.claim(a, {node});
    model.claim(b, {node});
    require(model.overflow() == 1, "two owners on one node must overflow by 1");
    require(model.overflow(node) == 1, "node overflow must be owner_count-1");
    require(model.owners_of(node).size() == 2, "both owners must be visible on the node");
    require(contains_owner(model.owners_of(node), a), "owners_of must include the first owner");
    require(contains_owner(model.owners_of(node), b), "owners_of must include the second owner");

    const auto sync_a = OwnerId {9, 0};
    const auto sync_b = OwnerId {9, 1};
    const auto sync_node = node_resource(11);
    model.claim(sync_a, {sync_node});
    model.claim(sync_b, {sync_node});
    require(
        model.overflow(sync_node) == 1,
        "SyncNet members with distinct demand_id must be different owners");

    require_near(model.history(node), 0.0, "history starts at 0");
    model.history_next();
    require_near(model.history(node), 1.0, "history_next must add increment * overflow");
    model.history_next();
    require_near(model.history(node), 1.9, "history_next must decay previous history then add overflow");
}

auto test_resource_mode_conflict() -> void {
    auto model = ResourceModel {};
    const auto straight = mode_straight_key(7);
    const auto swap = mode_swap_key(7);
    const auto conflict = mode_conflict_key(7);
    const auto a = OwnerId {1, 0};
    const auto b = OwnerId {2, 0};
    const auto c = OwnerId {3, 0};

    model.claim(a, {straight});
    model.claim(b, {straight});
    require(model.overflow() == 0, "same-mode owners must not create mode overflow");
    require(model.overflow(conflict) == 0, "same-mode group must not mark the conflict key");
    require(model.owners_of(conflict).empty(), "unconflicted mode group must not list conflict owners");

    model.claim(c, {swap});
    require(model.overflow(conflict) > 0, "straight+swap must overflow the mode-conflict key");
    require(model.overflow() > 0, "mode conflict must contribute to total overflow");
    require(model.owners_of(conflict).size() == 3, "conflict owners_of must list every mode user");
    require(contains_owner(model.owners_of(conflict), a), "conflict owners must include straight user A");
    require(contains_owner(model.owners_of(conflict), b), "conflict owners must include straight user B");
    require(contains_owner(model.owners_of(conflict), c), "conflict owners must include swap user C");
    require(model.type_weight(conflict) == 8, "mode-conflict type_weight must be 8");
}

auto test_resource_partial_matching() -> void {
    auto model = ResourceModel {};
    const auto bump_side = matching_endpoint_key(100, 0);
    const auto hline_side = matching_endpoint_key(200, 1);
    const auto a = OwnerId {1, 0};
    const auto b = OwnerId {2, 0};
    const auto c = OwnerId {3, 0};

    model.claim(a, {bump_side});
    model.claim(b, {hline_side});
    require(model.overflow(bump_side) == 0, "different switch endpoints must not overflow matching keys");
    require(model.overflow(hline_side) == 0, "the other matching endpoint must also stay at overflow 0");
    require(model.overflow() == 0, "distinct matching endpoints must not add total overflow");

    model.claim(c, {bump_side});
    require(model.overflow(bump_side) == 1, "two owners on the same matching endpoint must overflow it");
    require(model.overflow(hline_side) == 0, "the unused matching endpoint must not inherit overflow");
}

auto test_resource_hline_vline_node_exclusive() -> void {
    auto model = ResourceModel {};
    const auto hline_node = node_resource(50);
    const auto bump_h_endpoint = matching_endpoint_key(50, 1);
    const auto h_v_endpoint = matching_endpoint_key(50, 2);
    const auto a = OwnerId {1, 0};
    const auto b = OwnerId {2, 0};

    model.claim(a, {hline_node, bump_h_endpoint});
    model.claim(b, {hline_node, h_v_endpoint});
    require(model.overflow(bump_h_endpoint) == 0, "different matching endpoints must not overflow");
    require(model.overflow(h_v_endpoint) == 0, "the other matching endpoint must not overflow");
    require(model.overflow(hline_node) == 1, "two owners on the same HLine node must overflow the node");
    require(model.overflow() >= 1, "HLine node exclusivity must contribute to total overflow");
}

auto test_resource_bnet_unit_lock() -> void {
    auto model = ResourceModel {};
    const auto owner = OwnerId {5, 0};
    const auto unit_a = bnet_unit_key(3);
    const auto unit_b = bnet_unit_key(4);

    model.claim(owner, {unit_a});
    require(model.selected_unit(owner) == 3, "first Bnet unit claim must lock the unit");
    require(model.overflow() == 0, "a single selected unit must not overflow");

    model.claim(owner, {unit_b});
    require(model.selected_unit(owner) == 3, "a later different unit must not replace the locked unit");
    require(model.overflow() > 0, "claiming a second Bnet unit for the same owner must overflow");
    require(
        contains_owner(model.owners_of(unit_a), owner) || contains_owner(model.owners_of(unit_b), owner),
        "Bnet unit lock conflict must be visible through owners_of a unit key");

    model.release(owner, {unit_b});
    require(model.overflow() == 0, "releasing the extra unit must clear the lock overflow");
    require(model.selected_unit(owner) == 3, "locked unit must remain after the extra unit is released");
}

auto add_synth_node(UnifiedGraph& graph, UnifiedNodeKind kind = UnifiedNodeKind::Track, std::size_t unit = 0)
    -> int {
    UnifiedNode node {};
    node.kind = kind;
    node.unit = unit;
    const int id = static_cast<int>(graph.nodes.size());
    graph.nodes.push_back(node);
    graph.in_arc_ids.emplace_back();
    graph.out_arc_ids.emplace_back();
    return id;
}

auto add_synth_track(
    UnifiedGraph& graph,
    int row,
    int col,
    std::size_t track_index
) -> int {
    const int id = add_synth_node(graph, UnifiedNodeKind::Track);
    auto& node = graph.nodes[static_cast<std::size_t>(id)];
    node.track_row = row;
    node.track_col = col;
    node.track_dir = 0;
    node.track_index = track_index;
    return id;
}

auto add_synth_arc(UnifiedGraph& graph, int u, int v) -> void {
    const int arc_id = static_cast<int>(graph.arcs.size());
    graph.arcs.push_back(UnifiedArc {u, v});
    graph.out_arc_ids[static_cast<std::size_t>(u)].push_back(arc_id);
    graph.in_arc_ids[static_cast<std::size_t>(v)].push_back(arc_id);
    graph.directed_arc_set.insert({u, v});
}

auto add_undirected(UnifiedGraph& graph, int u, int v) -> void {
    add_synth_arc(graph, u, v);
    add_synth_arc(graph, v, u);
}

auto require_path(const std::Vector<int>& path, const std::Vector<int>& expected, const std::string& message)
    -> void {
    require(path == expected, message);
}

auto make_short_long_graph() -> UnifiedGraph {
    auto graph = UnifiedGraph {};
    const int s = add_synth_node(graph);
    const int a = add_synth_node(graph);
    const int t = add_synth_node(graph);
    const int b = add_synth_node(graph);
    const int c = add_synth_node(graph);
    const int d = add_synth_node(graph);
    require(s == 0 && a == 1 && t == 2 && b == 3 && c == 4 && d == 5, "short/long graph node ids");
    add_undirected(graph, s, a);
    add_undirected(graph, a, t);
    add_undirected(graph, s, b);
    add_undirected(graph, b, c);
    add_undirected(graph, c, d);
    add_undirected(graph, d, t);
    return graph;
}

auto test_maze_empty_occupancy_shortest_path() -> void {
    const auto graph = make_short_long_graph();
    auto resources = ResourceModel {};
    const auto owner = OwnerId {1, 0};
    const auto path = route_demand(graph, resources, owner, {0}, 2, RrrParams {});
    require_path(path, {0, 1, 2}, "empty occupancy must take the 2-hop path, not the 4-hop path");
    require(resources.owners_of(node_resource(1)).empty(), "maze must not claim the path into ResourceModel");
    require(resources.overflow() == 0, "empty maze must leave occupancy empty");
}

auto test_maze_history_diverts() -> void {
    const auto graph = make_short_long_graph();
    auto resources = ResourceModel {};
    const auto blocker_a = OwnerId {8, 0};
    const auto blocker_b = OwnerId {9, 0};
    const auto short_node = node_resource(1);
    resources.claim(blocker_a, {short_node});
    resources.claim(blocker_b, {short_node});
    for (int i = 0; i < 40; ++i) {
        resources.history_next();
    }
    resources.release(blocker_a);
    resources.release(blocker_b);
    require(resources.owners_of(short_node).empty(), "history test must leave the short node unoccupied");
    require(resources.history(short_node) > 8.0, "history on the short path must be raised");

    const auto path = route_demand(graph, resources, OwnerId {1, 0}, {0}, 2, RrrParams {});
    require_path(path, {0, 3, 4, 5, 2}, "raised history on the short path must divert maze onto the long path");
}

auto test_maze_ripup_then_reuse() -> void {
    auto graph = UnifiedGraph {};
    const int s = add_synth_node(graph);
    const int a = add_synth_node(graph);
    const int b = add_synth_node(graph);
    const int t = add_synth_node(graph);
    require(s == 0 && a == 1 && b == 2 && t == 3, "rip-up graph node ids");
    add_undirected(graph, s, a);
    add_undirected(graph, a, t);
    add_undirected(graph, s, b);
    add_undirected(graph, b, t);

    auto resources = ResourceModel {};
    const auto owner = OwnerId {1, 0};
    const auto blocker = OwnerId {2, 0};
    const auto params = RrrParams {};

    const auto empty = route_demand(graph, resources, owner, {0}, 3, params);
    require_path(empty, {0, 1, 3}, "equal-hop empty maze must prefer the first out-arc path");

    resources.claim(blocker, {node_resource(1)});
    const auto diverted = route_demand(graph, resources, owner, {0}, 3, params);
    require_path(diverted, {0, 2, 3}, "occupying the preferred path must divert maze to the other equal-hop path");

    resources.release(blocker);
    const auto reused = route_demand(graph, resources, owner, {0}, 3, params);
    require_path(reused, {0, 1, 3}, "after release, maze must reuse the original preferred path");
}

auto test_maze_pnnet_multi_source() -> void {
    auto graph = UnifiedGraph {};
    const int a = add_synth_node(graph);
    const int b = add_synth_node(graph);
    const int x = add_synth_node(graph);
    const int t = add_synth_node(graph);
    require(a == 0 && b == 1 && x == 2 && t == 3, "PNnet graph node ids");
    add_undirected(graph, a, x);
    add_undirected(graph, x, t);
    add_undirected(graph, b, t);

    auto resources = ResourceModel {};
    const auto path = route_demand(graph, resources, OwnerId {3, 0}, {0, 1}, 3, RrrParams {});
    require(!path.empty(), "PNnet maze must return a path");
    require(path.front() == 1, "PNnet maze must start at the nearer source B, not A");
    require_path(path, {1, 3}, "PNnet maze must take the 1-hop path from source B");
}

auto test_maze_fanout_tree_growth() -> void {
    auto graph = UnifiedGraph {};
    const int s = add_synth_node(graph);
    const int a = add_synth_node(graph);
    const int t1 = add_synth_node(graph);
    const int t2 = add_synth_node(graph);
    require(s == 0 && a == 1 && t1 == 2 && t2 == 3, "fanout graph node ids");
    add_undirected(graph, s, a);
    add_undirected(graph, a, t1);
    add_undirected(graph, a, t2);

    auto resources = ResourceModel {};
    const auto owner = OwnerId {4, 0};
    const auto params = RrrParams {};
    const auto first = route_demand(graph, resources, owner, {0}, 2, params);
    require_path(first, {0, 1, 2}, "first fanout branch must be the source-to-sink1 path");
    resources.claim(owner, {node_resource(0), node_resource(1), node_resource(2)});

    const auto second = route_demand(
        graph,
        resources,
        owner,
        {0},
        3,
        params,
        std::Vector<int> {0, 1, 2});
    require(!second.empty(), "second fanout demand must return a path");
    require(second.front() == 1, "second fanout branch must start at a tree node, not only the original source");
    require_path(second, {1, 3}, "second fanout branch must reuse the shared prefix and grow A->sink2");
}

auto contains_key_local(const std::Vector<ResourceKey>& keys, const ResourceKey& key) -> bool {
    for (const auto& existing : keys) {
        if (existing == key) {
            return true;
        }
    }
    return false;
}

auto test_path_resource_keys_match_maze_and_bnet_unit() -> void {
    auto graph = UnifiedGraph {};
    const int vline = add_synth_node(graph, UnifiedNodeKind::VLine, 5);
    const int track = add_synth_node(graph, UnifiedNodeKind::Track, 5);
    add_synth_arc(graph, vline, track);
    graph.arcs.back().physical_switch_kind = PhysicalSwitchKind::VLineTrack;
    graph.arcs.back().mode_group_id = 3;
    graph.arcs.back().is_vline_track_straight = true;
    graph.arcs.back().physical_switch_id = 9;

    const auto keys = path_resource_keys(graph, {vline, track}, true);
    require(contains_key_local(keys, node_resource(vline)), "path keys must include the VLine node");
    require(contains_key_local(keys, node_resource(track)), "path keys must include the Track node");
    require(contains_key_local(keys, switch_resource(9)), "path keys must include the physical switch");
    require(contains_key_local(keys, mode_straight_key(3)), "path keys must include the straight mode key");
    require(contains_key_local(keys, bnet_unit_key(5)), "Bnet path keys must claim bnet_unit_key");
    const auto arc_keys_only = arc_resource_keys(graph, graph.arcs.back());
    require(!arc_keys_only.empty(), "maze arc projection must be non-empty");
    for (const auto& key : arc_keys_only) {
        require(contains_key_local(keys, key), "path keys must include every maze arc key");
    }
}

auto test_rrr_best_save_lex_order() -> void {
    require(rrr_is_better(0, 100, 1, 10), "smaller overflow must beat a shorter overflowing solution");
    require(rrr_is_better(1, 5, 1, 10), "equal overflow must keep the shorter wirelength");
    require(!rrr_is_better(1, 10, 1, 5), "equal overflow must reject a longer wirelength");
    require(!rrr_is_better(2, 1, 1, 100), "higher overflow must not replace the best");
    require(!rrr_is_better(0, 8, 0, 8), "an identical solution is not an improvement");
}

auto add_synth_bump_node(
    UnifiedGraph& graph,
    std::size_t tob,
    std::size_t bank,
    std::size_t group,
    std::size_t index
) -> int {
    const int id = add_synth_node(graph, UnifiedNodeKind::Bump);
    const auto bump = Bump_coord {tob, bank, group, index};
    graph.nodes[static_cast<std::size_t>(id)].bump = bump;
    graph.bump_node_by_key[bump] = id;
    return id;
}

auto bump_ref(std::size_t tob, std::size_t bank, std::size_t group, std::size_t index) -> GraphNodeRef {
    GraphNodeRef ref {};
    ref.kind = GraphNodeRef::Kind::Bump;
    ref.bump = Bump_coord {tob, bank, group, index};
    return ref;
}

auto make_tiny_tnet(std::size_t net_id, GraphNodeRef src, GraphNodeRef sink) -> RoutingNet {
    RoutingNet net {};
    net.net_id = net_id;
    net.kind = RoutingNetKind::Tnet;
    net.sources.push_back(src);
    RoutingDemand demand {};
    demand.demand_id = 0;
    demand.sink = sink;
    demand.candidate_source_indices = {0};
    demand.fixed_pair = true;
    net.demands.push_back(demand);
    return net;
}

auto make_sync_tnet(
    std::size_t net_id,
    const std::Vector<GraphNodeRef>& sources,
    const std::Vector<GraphNodeRef>& sinks
) -> RoutingNet {
    require(sources.size() == sinks.size(), "synthetic SyncNet must have matched sources and sinks");
    auto net = RoutingNet {};
    net.net_id = net_id;
    net.kind = RoutingNetKind::Tnet;
    net.sources = sources;
    net.is_sync_bus = true;
    for (std::size_t i = 0; i < sinks.size(); ++i) {
        RoutingDemand demand {};
        demand.demand_id = i;
        demand.sink = sinks[i];
        demand.candidate_source_indices = {i};
        demand.fixed_pair = true;
        net.demands.push_back(demand);
    }
    return net;
}

auto print_path_log(
    const std::string& label,
    const UnifiedGraph& graph,
    const std::Vector<int>& path,
    hardware::Interposer* interposer,
    bool is_bnet
) -> void {
    std::cout << "FPIA_RRR_unit: " << label
              << " tracks=" << (sync_lane_length(graph, path, interposer, is_bnet)
                                      - sync_tob_length_constant(is_bnet))
              << " N_i=" << sync_lane_length(graph, path, interposer, is_bnet)
              << " path=[";
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (i != 0) {
            std::cout << " -> ";
        }
        std::cout << format_path_node(graph, path[i]);
    }
    std::cout << "]\n";
}

auto make_tiny_graph_and_nets() -> std::pair<UnifiedGraph, std::Vector<RoutingNet>> {
    auto graph = UnifiedGraph {};
    const int src = add_synth_bump_node(graph, 0, 0, 0, 0);
    const int snk = add_synth_bump_node(graph, 1, 0, 0, 0);
    require(src == 0 && snk == 1, "tiny graph node ids");
    add_synth_arc(graph, src, snk);
    auto nets = std::Vector<RoutingNet> {};
    nets.push_back(make_tiny_tnet(0, bump_ref(0, 0, 0, 0), bump_ref(1, 0, 0, 0)));
    return {graph, nets};
}

auto test_validate_empty_path_fails() -> void {
    auto [graph, nets] = make_tiny_graph_and_nets();
    RrrResult result {};
    result.status = "success";
    result.paths = {{{}}};
    require(
        !validate_rrr_solution(graph, nets, result),
        "empty demand path must fail independent validation");
}

auto test_validate_illegal_overflow_fails() -> void {
    auto graph = UnifiedGraph {};
    const int src = add_synth_bump_node(graph, 0, 0, 0, 0);
    const int snk = add_synth_bump_node(graph, 1, 0, 0, 0);
    add_synth_arc(graph, src, snk);
    auto nets = std::Vector<RoutingNet> {};
    nets.push_back(make_tiny_tnet(0, bump_ref(0, 0, 0, 0), bump_ref(1, 0, 0, 0)));
    nets.push_back(make_tiny_tnet(1, bump_ref(0, 0, 0, 0), bump_ref(1, 0, 0, 0)));
    RrrResult result {};
    result.status = "success";
    result.paths = {{{0, 1}}, {{0, 1}}};
    require(
        !validate_rrr_solution(graph, nets, result),
        "two owners claiming the same nodes must fail independent validation");
}

auto test_validate_legal_tiny_path_passes() -> void {
    auto [graph, nets] = make_tiny_graph_and_nets();
    RrrResult result {};
    result.status = "success";
    result.paths = {{{0, 1}}};
    require(
        validate_rrr_solution(graph, nets, result),
        "a connected overflow-free tiny path must pass independent validation");
}

auto test_run_rrr_requires_interposer() -> void {
    auto [graph, nets] = make_tiny_graph_and_nets();
    try {
        (void)run_rrr(graph, nets, RrrParams {}, nullptr);
        require(false, "run_rrr must reject a null Interposer");
    }
    catch (const std::invalid_argument&) {
    }
}

auto test_sync_track_cut_index() -> void {
    require(sync_track_cut_index(10, 0.5) == 5, "r=0.5 must keep floor(10*(1-0.5))=5 tracks");
    require(sync_track_cut_index(10, 0.75) == 2, "r=0.75 must keep floor(10*(1-0.75))=2 tracks");
    require(sync_track_cut_index(10, 1.0) == 0, "r=1.0 must keep floor(10*(1-1.0))=0 tracks");
    require(sync_track_cut_index(7, 0.5) == 3, "r=0.5 must keep floor(7*0.5)=3 tracks");
    require(sync_track_cut_index(7, 0.75) == 1, "r=0.75 must keep floor(7*0.25)=1 track");
    require(sync_track_cut_index(7, 1.0) == 0, "r=1.0 must keep floor(7*0)=0 tracks");
    require(sync_track_cut_index(1, 0.5) == 0, "single-track r=0.5 must keep floor(0.5)=0");
    require(sync_track_cut_index(0, 0.5) == 0, "empty track list cut index is 0");
}

auto test_sync_tail_equalize_replaces_short_lane() -> void {
    auto graph = UnifiedGraph {};
    const int long0 = add_synth_track(graph, 1, 0, 0);
    const int long1 = add_synth_track(graph, 1, 1, 0);
    const int long2 = add_synth_track(graph, 1, 2, 0);
    const int long_sink = add_synth_track(graph, 1, 3, 0);
    const int short0 = add_synth_track(graph, 3, 0, 1);
    const int short1 = add_synth_track(graph, 3, 1, 1);
    const int short_sink = add_synth_track(graph, 3, 2, 1);
    const int detour = add_synth_track(graph, 4, 1, 1);

    add_synth_arc(graph, long0, long1);
    add_synth_arc(graph, long1, long2);
    add_synth_arc(graph, long2, long_sink);
    add_synth_arc(graph, short0, short1);
    add_synth_arc(graph, short1, short_sink);
    add_synth_arc(graph, short1, detour);
    add_synth_arc(graph, detour, short_sink);

    auto interposer = hardware::Interposer {};
    auto resources = ResourceModel {};
    auto lanes = std::Vector<SyncLaneState> {
        SyncLaneState {OwnerId {20, 0}, false, {long0}, long_sink, {long0, long1, long2, long_sink}},
        SyncLaneState {OwnerId {20, 1}, false, {short0}, short_sink, {short0, short1, short_sink}}};
    for (const auto& lane : lanes) {
        resources.claim(lane.id, path_resource_keys(graph, lane.path, lane.is_bnet));
    }

    std::cout << "FPIA_RRR_unit: tail-equalize initial paths\n";
    for (std::size_t i = 0; i < lanes.size(); ++i) {
        print_path_log(
            "tail-equalize initial lane=" + std::to_string(i),
            graph,
            lanes[i].path,
            &interposer,
            lanes[i].is_bnet);
    }

    auto params = RrrParams {};
    params.r_sequence = {1.0};
    require(
        equalize_sync_group(graph, resources, params, &interposer, lanes),
        "tail equalization must replace a shorter SyncNet lane");
    require_path(
        lanes[1].path,
        {short0, short1, detour, short_sink},
        "tail equalization must choose the exact-length detour");
    require(
        sync_lane_length(graph, lanes[0].path, &interposer, false)
            == sync_lane_length(graph, lanes[1].path, &interposer, false),
        "tail equalization must leave equal Track-node lengths");
    require(resources.overflow() == 0, "equalized synthetic lanes must remain overflow-free");
    std::cout << "FPIA_RRR_unit: tail-equalize final paths\n";
    for (std::size_t i = 0; i < lanes.size(); ++i) {
        print_path_log(
            "tail-equalize final lane=" + std::to_string(i),
            graph,
            lanes[i].path,
            &interposer,
            lanes[i].is_bnet);
    }
}

auto add_directed_path(UnifiedGraph& graph, const std::Vector<int>& path) -> void {
    require(path.size() >= 2, "synthetic path must contain at least two nodes");
    for (std::size_t i = 1; i < path.size(); ++i) {
        add_synth_arc(graph, path[i - 1], path[i]);
    }
}

auto test_rrr_sync_bus_ripup_and_normal_detour() -> void {
    constexpr std::size_t kLaneCount = 5;
    const std::Vector<int> initial_tracks {9, 7, 6, 5, 3};
    auto graph = UnifiedGraph {};
    auto sources = std::Vector<GraphNodeRef> {};
    auto sinks = std::Vector<GraphNodeRef> {};
    auto initial_paths = std::Vector<std::Vector<int>> {};
    auto alternative_paths = std::Vector<std::Vector<int>> {};

    int lane0_shared_track = -1;
    for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
        const int source = add_synth_bump_node(graph, 0, 0, lane, 0);
        const int sink = add_synth_bump_node(graph, 1, 0, lane, 0);
        sources.push_back(bump_ref(0, 0, lane, 0));
        sinks.push_back(bump_ref(1, 0, lane, 0));

        auto initial = std::Vector<int> {source};
        for (int i = 0; i < initial_tracks[lane]; ++i) {
            initial.push_back(add_synth_track(graph, static_cast<int>(lane), i, lane));
        }
        initial.push_back(sink);
        add_directed_path(graph, initial);
        initial_paths.push_back(initial);
        if (lane == 0) {
            lane0_shared_track = initial[1 + initial_tracks[lane] / 2];
            alternative_paths.push_back(initial);
            continue;
        }

        auto alternative = std::Vector<int> {source};
        for (int i = 0; i < initial_tracks[0]; ++i) {
            alternative.push_back(add_synth_track(graph, static_cast<int>(lane + 5), i, lane));
        }
        alternative.push_back(sink);
        add_directed_path(graph, alternative);
        alternative_paths.push_back(std::move(alternative));
    }
    require(lane0_shared_track >= 0, "synthetic congested SyncNet must expose a shared track");

    const int normal_source = add_synth_bump_node(graph, 2, 0, 0, 0);
    const int normal_sink = add_synth_bump_node(graph, 3, 0, 0, 0);
    const int normal_detour0 = add_synth_track(graph, 8, 0, 6);
    const int normal_detour1 = add_synth_track(graph, 8, 1, 6);
    add_synth_arc(graph, normal_source, lane0_shared_track);
    add_synth_arc(graph, lane0_shared_track, normal_sink);
    const auto normal_primary = std::Vector<int> {normal_source, lane0_shared_track, normal_sink};
    const auto normal_detour = std::Vector<int> {
        normal_source,
        normal_detour0,
        normal_detour1,
        normal_sink};
    add_directed_path(graph, normal_detour);

    auto nets = std::Vector<RoutingNet> {};
    nets.push_back(make_sync_tnet(0, sources, sinks));
    nets.push_back(make_tiny_tnet(1, bump_ref(2, 0, 0, 0), bump_ref(3, 0, 0, 0)));

    auto interposer = hardware::Interposer {};
    std::cout << "FPIA_RRR_unit: sync-rrr initial shortest candidates\n";
    for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
        print_path_log(
            "sync-rrr initial lane=" + std::to_string(lane),
            graph,
            initial_paths[lane],
            &interposer,
            false);
        if (lane != 0) {
            print_path_log(
                "sync-rrr equal-length alternative lane=" + std::to_string(lane),
                graph,
                alternative_paths[lane],
                &interposer,
                false);
        }
    }
    print_path_log("sync-rrr normal initial", graph, normal_primary, &interposer, false);
    print_path_log("sync-rrr normal detour", graph, normal_detour, &interposer, false);

    auto params = RrrParams {};
    params.H = 0;
    params.increment = 10;
    params.decay = 1;
    params.history_weight = 1;
    params.max_iterations = 4;
    params.stagnation_limit = 4;
    params.r_sequence = {1.0};
    const auto result = run_rrr(graph, nets, params, &interposer);

    require(result.status == "success", "synthetic 5-lane SyncNet RRR must finish successfully");
    require(result.best_overflow == 0, "synthetic 5-lane SyncNet RRR must clear local congestion");
    require(result.iterations >= 1, "synthetic congestion must trigger at least one RRR iteration");
    require(result.paths.size() == 2, "synthetic result must contain the SyncNet and normal net");
    require(result.paths[0].size() == kLaneCount, "synthetic SyncNet result must retain five lanes");
    require(result.paths[1].size() == 1, "synthetic normal net must retain one demand");
    for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
        require_path(
            result.paths[0][lane],
            alternative_paths[lane],
            "synthetic SyncNet lane must take its expected equal-length path");
        require(
            sync_lane_length(graph, result.paths[0][lane], &interposer, false) == 10,
            "synthetic SyncNet lanes must finish at nine Tracks plus one TOB constant");
    }
    require_path(
        result.paths[1][0],
        normal_detour,
        "history-driven RRR must move the normal net off the congested SyncNet track");
    require(
        validate_rrr_solution(graph, nets, result, &interposer),
        "synthetic 5-lane RRR result must pass independent validation");

    std::cout << "FPIA_RRR_unit: sync-rrr final paths status=" << result.status
              << " iterations=" << result.iterations
              << " overflow=" << result.best_overflow << '\n';
    for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
        print_path_log(
            "sync-rrr final lane=" + std::to_string(lane),
            graph,
            result.paths[0][lane],
            &interposer,
            false);
    }
    print_path_log("sync-rrr final normal", graph, result.paths[1][0], &interposer, false);
}

auto test_rrr_sort_keys() -> void {
    const auto high = OwnerId {2, 0};
    const auto low = OwnerId {1, 0};
    require(
        rrr_dirty_before(8, 1, 3, high, 3, 9, 9, low),
        "dirty sort must prefer higher congestion exposure");
    require(
        rrr_dirty_before(4, 5, 1, high, 4, 2, 9, low),
        "dirty sort must prefer higher retry when exposure ties");
    require(
        rrr_dirty_before(4, 2, 9, high, 4, 2, 3, low),
        "dirty sort must prefer higher HPWL when exposure and retry tie");
    require(
        rrr_dirty_before(4, 2, 9, low, 4, 2, 9, high),
        "dirty sort must break remaining ties by ascending owner id");
    require(
        rrr_initial_before(true, 2, 1, OwnerId {9, 0}, false, 8, 20, OwnerId {0, 0}),
        "initial sort must place SyncNet bus owners before ordinary nets");
    require(
        rrr_initial_before(false, 6, 2, OwnerId {1, 0}, false, 3, 9, OwnerId {0, 0}),
        "initial ordinary sort must prefer higher port count");
}

} // namespace

auto main() -> int {
    try {
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
        test_resource_claim_release();
        test_resource_same_owner_branch_share();
        test_resource_cross_owner_overflow();
        test_resource_mode_conflict();
        test_resource_partial_matching();
        test_resource_hline_vline_node_exclusive();
        test_resource_bnet_unit_lock();
        test_maze_empty_occupancy_shortest_path();
        test_maze_history_diverts();
        test_maze_ripup_then_reuse();
        test_maze_pnnet_multi_source();
        test_maze_fanout_tree_growth();
        test_path_resource_keys_match_maze_and_bnet_unit();
        test_rrr_best_save_lex_order();
        test_rrr_sort_keys();
        test_sync_track_cut_index();
        test_sync_tail_equalize_replaces_short_lane();
        test_rrr_sync_bus_ripup_and_normal_detour();
        test_validate_empty_path_fails();
        test_validate_illegal_overflow_fails();
        test_validate_legal_tiny_path_passes();
        test_run_rrr_requires_interposer();
        std::cout << "FPIA_RRR_unit: all tests passed\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "FPIA_RRR_unit: " << error.what() << '\n';
        return 1;
    }
}
