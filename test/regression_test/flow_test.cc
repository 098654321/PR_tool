#include "algo/netbuilder/netbuilder.hh"
#include <algo/route_data.hh>
#include <catch2/catch_test_macros.hpp>
#include <std/string.hh>
#include <std/file.hh>
#include <utility/file.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>
#include <algo/router/route_nets.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <algo/placer/place.hh>
#include <algo/placer/sa/saplacestrategy.hh>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/bbnet.hh>
#include <hardware/tob/tob.hh>
#include <parse/comparator/controlbits_parser.hh>
#include <parse/writer/module.hh>

namespace PR_tool::test {

    // Helper to extract topdie instances from basedie
    std::Vector<circuit::TopDieInstance*> get_topdie_insts(circuit::BaseDie* basedie) {
        std::Vector<circuit::TopDieInstance*> insts;
        for (const auto& pair : basedie->topdie_insts()) {
            insts.push_back(pair.second.get());
        }
        return insts;
    }

    void PLEASE_DO_NOT_FAIL_FLOW(std::usize id, std::String info) {
        WHEN("Case " + std::to_string(id) + " (Placement + Routing): " + info) {
            std::FilePath config_path{"../test/config/case" + std::to_string(id)};
            debug::initial_log("debug_flow_case" + std::to_string(id) + ".log");

            // 1. Read config
            auto [interposer, basedie, register_map] = PR_tool::parse::read_config(config_path, 0, false);
            
            // 2. Build nets
            algo::build_nets(basedie.get(), interposer.get());

            // 3. Run Placement
            debug::info("Starting Placement...");
            auto topdies = get_topdie_insts(basedie.get());
            algo::SAPlaceStrategy place_strategy{}; 
            algo::place(interposer.get(), topdies, basedie.get(), place_strategy);

            // DEBUG: Check TOB locations
            for (auto& [mode, net_list] : basedie->nets()) {
                for (auto& net : net_list) {
                    if (auto sync_net = dynamic_cast<circuit::SyncNet*>(net.get())) {
                        for (auto& btb : sync_net->btbnets()) {
                             if (btb->begin_bump()->tob()->coord() == btb->end_bump()->tob()->coord()) {
                                 auto c = btb->begin_bump()->tob()->coord();
                                 debug::error("Net " + btb->name() + " has same TOB: " + std::to_string(c.row) + "," + std::to_string(c.col));
                             }
                        }
                    }
                }
            }

            // 4. Run Routing (Standard)
            debug::info("Starting Routing...");
            auto data = algo::route_nets(interposer.get(), basedie.get(), algo::MazeRouteStrategy{}, algo::HK{}, 0, false, false);

            parse::output_from_routing_results(interposer.get(), ".", basedie.get(), 0, false, false, register_map);
            
            // 5. Verify
            THEN("Routing should succeed"){
                 CHECK(data._failed_net == 0);
            }
        }
    }

    void PLEASE_DO_NOT_FAIL_FLOW_INCRE(std::usize id, std::usize mode, std::String info) {
        WHEN("Case " + std::to_string(id) + " Mode " + std::to_string(mode) + " (Placement + Incremental Routing): " + info) {
            std::FilePath config_path{"../test/config/case" + std::to_string(id)};
            debug::initial_log("debug_flow_incre_case" + std::to_string(id) + ".log");

            // 1. Read config
            auto [interposer, basedie, register_map] = PR_tool::parse::read_config(config_path, mode, false);
            
            // 2. Build nets
            algo::build_nets(basedie.get(), interposer.get());
            basedie->merge_same_mode_nets();

            // 3. Run Placement
            debug::info("Starting Placement...");
            auto topdies = get_topdie_insts(basedie.get());
            algo::SAPlaceStrategy place_strategy{};
            algo::place(interposer.get(), topdies, basedie.get(), place_strategy);

            // 4. Run Incremental Routing
            auto [has_bits, has_other_bits] = parse::read_controlbits(config_path, interposer.get(), basedie.get(), mode, false);
            
            debug::info("Starting Incremental Routing...");
            // mode = 2, incremental = true, try_all_modes = false
            auto data_per_cycle = algo::route_nets(interposer.get(), basedie.get(), algo::MazeRouteStrategy{true}, algo::HK{}, mode, true, false, has_other_bits);
            parse::output_from_routing_results(interposer.get(), ".", basedie.get(), mode, false, false, register_map);
            
            // 5. Verify
            THEN("Routing should succeed"){
                 CHECK(data_per_cycle._failed_net == 0);
            }
        }
    }

    SCENARIO("Regression test for Placement -> Routing Flow", "[flow]"){
        PLEASE_DO_NOT_FAIL_FLOW(10, "Placement + Standard Routing (Case 10)");
        // PLEASE_DO_NOT_FAIL_FLOW_INCRE(20, 2, "Placement + Incremental Routing (Case 20, Mode 2)");
    }
}
