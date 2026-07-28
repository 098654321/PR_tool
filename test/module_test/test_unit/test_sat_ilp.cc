#include "./utilty.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/path/pathpackage.hh>
#include <debug/debug.hh>
#include <hardware/interposer.hh>
#include <hardware/tob/tob.hh>
#include <parse/reader/module.hh>
#include <sat/solve_unified_sat.hh>
#include <scope/build_routing_nets.hh>

#include <iostream>

using namespace PR_tool;
using namespace PR_tool::hardware;
using namespace PR_tool::circuit;
using namespace PR_tool::algo;

namespace {

constexpr auto kCase1Config = "../test/config/case1";
constexpr auto kCase2BtbConfig = "../algorithm/test_ILP/test/case_2btb";

auto pathpackage_has_content(const PathPackage& pkg) -> bool {
    return !pkg._regular_path.empty() || !pkg._tob_to_track.empty() || !pkg._track_to_tob.empty();
}

auto solve_and_commit_config(const char* config_path) -> SatRoutingResult {
    auto [interposer, basedie, register_map] = parse::read_config(config_path, 0, false);
    (void)register_map;
    build_nets(basedie.get(), interposer.get());

    UnifiedSatSolveOptions options {};
    options.verbose_level = 0;
    return solve_unified_sat_and_commit(interposer.get(), *basedie.get(), options);
}

} // namespace

static void test_build_routing_nets_bump_to_bump() {
    debug::debug("test_build_routing_nets_bump_to_bump");

    TOB tob0 {0, 0};
    TOB tob1 {1, 1};
    Bump bump0 {BumpCoord {0, 0, 0}, &tob0};
    Bump bump1 {BumpCoord {0, 0, 1}, &tob1};

    std::String name {"sat_ilp_unit_btb"};
    std::String uid {"sat_ilp_unit_btb_uid"};
    auto net = std::make_shared<BumpToBumpNet>(
        &bump0,
        &bump1,
        std::HashSet<int> {0},
        name,
        uid);

    const auto routing_nets = build_routing_nets({net});
    ASSERT(routing_nets.size() == 1);
    ASSERT(routing_nets[0].sources.size() == 1);
    ASSERT(routing_nets[0].demands.size() == 1);
    validate_v14_routing_nets(routing_nets);
}

static void test_sat_ilp_case1_commit_smoke() {
    debug::debug("test_sat_ilp_case1_commit_smoke");

    auto [interposer, basedie, register_map] = parse::read_config(kCase1Config, 0, false);
    (void)register_map;
    build_nets(basedie.get(), interposer.get());

    UnifiedSatSolveOptions options {};
    options.verbose_level = 0;
    const auto result = solve_unified_sat_and_commit(interposer.get(), *basedie.get(), options);

    ASSERT(result.ok);
    ASSERT(!result.paths.empty());

    auto committed_nets = 0;
    for (const auto& net : basedie->nets_to_vector()) {
        if (pathpackage_has_content(net->pathpackage())) {
            ++committed_nets;
        }
    }
    ASSERT(committed_nets > 0);

    std::cout << "sat_ilp case1 ok: paths=" << result.paths.size()
              << " total_wirelength=" << result.total_wirelength
              << " committed_nets=" << committed_nets << std::endl;
}

static void test_sat_ilp_case_2btb_optional() {
    if (Interposer::COB_ARRAY_WIDTH != 13) {
        std::cout << "sat_ilp: skip case_2btb (COB_ARRAY_WIDTH="
                  << Interposer::COB_ARRAY_WIDTH << ", need 13)" << std::endl;
        return;
    }

    debug::debug("test_sat_ilp_case_2btb_optional");
    const auto result = solve_and_commit_config(kCase2BtbConfig);
    ASSERT(result.ok);
    ASSERT(!result.paths.empty());

    std::cout << "sat_ilp case_2btb ok: paths=" << result.paths.size()
              << " total_wirelength=" << result.total_wirelength << std::endl;
}

void test_sat_ilp_main() {
    test_build_routing_nets_bump_to_bump();
    test_sat_ilp_case1_commit_smoke();
    test_sat_ilp_case_2btb_optional();
}
