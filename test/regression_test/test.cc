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
#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>


namespace PR_tool::test{

    void PLEASE_DO_NOT_FAIL(std::usize id, std::String info) {
        WHEN("Case " + std::to_string(id) + ": " + info) {
            std::FilePath config_path{"../test/config/case" + std::to_string(id)};
            debug::initial_log("debug.log");

            auto [interposer, basedie, register_map] = PR_tool::parse::read_config(config_path, 0, false);
            (void)register_map;
            algo::build_nets(basedie.get(), interposer.get());
            auto data = algo::route_nets(interposer.get(), basedie.get(), algo::MazeRouteStrategy{}, algo::HK{}, 0, false, false);
            THEN("The total length should be within a limit"){
                std::ifstream golden_file(config_path / "golden.txt");
                if (!golden_file.is_open()){
                    debug::exception_in("regression test case " + std::to_string(id), "golden file open failure");
                }
                std::String golden_length;
                if (!(golden_file >> golden_length)) { 
                    debug::info("golden file is empty in regression test case " + std::to_string(id));
                }
                else {
                    CHECK(data._total_length <= std::stoi(golden_length));
                }
            }
        }

        // utility::append_logs("debug.log", "regression_test.log");
    }

    SCENARIO("Regression test for basic PR_tool functions", "[basic]"){
        
        GIVEN("Configs, describing connections, external_ports, topdies and topdie_insts"){
            // std::ofstream file("regression_test.log", std::ios::trunc);

            //! notice: cob array here is 9*12
            PLEASE_DO_NOT_FAIL(1, "Muyan topdie with synchroinzed nets only");
            PLEASE_DO_NOT_FAIL(2, "Muyan topdie with both synchroinzed and unsynchronized nets");
            PLEASE_DO_NOT_FAIL(3, "Muyan topdie with unsynchronized nets only");
            // PLEASE_DO_NOT_FAIL(4, "A complete case with more nets and net types");  
            PLEASE_DO_NOT_FAIL(5, "a case for midterm test");
            
        }
    }

    SCENARIO("CPU-MEM-AI circuit test", "[CPU_MEM_AI]"){
        
        GIVEN("config.json & a txt file from xl"){
            // std::ofstream file("regression_test.log", std::ios::trunc);
            //! notice: cob array here is 9*13, and available pose/nege port is different
            PLEASE_DO_NOT_FAIL(7, "a case with the least number of bus");
            PLEASE_DO_NOT_FAIL(8, "a case with a middle scale of bus");         // fail
            PLEASE_DO_NOT_FAIL(9, "a case with the most number of bus");        // fail
        }
    }

    SCENARIO("CPU-MEM circuit test", "[CPU_MEM]"){
        
        GIVEN("config.json & a txt file from xl"){
            // std::ofstream file("regression_test.log", std::ios::trunc);
            //! notice: cob array here is 9*13, and available pose/nege port is the same with CPU_MEM_AI
            PLEASE_DO_NOT_FAIL(10, "a case with the least number of bus");
            PLEASE_DO_NOT_FAIL(11, "a case with a middle scale of bus");
            PLEASE_DO_NOT_FAIL(12, "a case with the most number of bus");
        }
    }

    SCENARIO("AI-core circuit test", "[AI_core]"){
        
        GIVEN("config.json & a txt file from xl"){
            // std::ofstream file("regression_test.log", std::ios::trunc);
            //! notice: cob array here is 9*13, and available pose/nege port is the same with CPU_MEM_AI
            PLEASE_DO_NOT_FAIL(13, "a case with the least number of bus");
            PLEASE_DO_NOT_FAIL(14, "a case with a middle scale of bus");        // fail
            PLEASE_DO_NOT_FAIL(15, "a case with more number of bus");           // fail
            PLEASE_DO_NOT_FAIL(16, "a case with the most number of bus");       // fail
        }
    }

    

}

