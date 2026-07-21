#include <algo/route_data.hh>
#include "algo/netbuilder/netbuilder.hh"
#include <catch2/catch_test_macros.hpp>
#include <std/string.hh>
#include <std/file.hh>
#include <utility/file.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>
#include <parse/writer/module.hh>
#include <parse/comparator/controlbits_parser.hh>
#include <algo/router/route_nets.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>
#include <std/collection.hh>
#include <std/format.hh>
#include <std/file.hh>
#include <algorithm>
#include <exception>


namespace PR_tool::test {

    void analyze_route_data(const PR_tool::algo::RouteData& data, std::FilePath output_file);

    void test_case(std::usize id, std::usize mode, bool try_all_modes, algo::RouteData& data, std::usize cycle);

    void PLEASE_DO_NOT_FAIL_INCRE(std::usize id, std::String info, std::usize mode, bool try_all_modes, std::usize total_cycle) {
        WHEN("Case " + std::to_string(id) + ": " + info) {
            PR_tool::algo::RouteData data{};
            debug::initial_log("debug.log");

            for (auto i = 0; i < total_cycle;) {
            try {
                test_case(id, mode, try_all_modes, data, i);
                i++;
            }
            catch (const std::exception& e) {
                debug::info("Exception caught: " + std::string(e.what()));
            }
            }

            std::String output_file{"incremental_regression_test.log"};
            analyze_route_data(data, output_file);
        }
    }

    
    SCENARIO("Regression test for incremental routing", "[incremental]"){
        
        GIVEN("Configs, describing connections, external_ports, topdies and topdie_insts"){
            //! notice: cob array here is 9*12
            PLEASE_DO_NOT_FAIL_INCRE(20, "", 1, false, 1);
            PLEASE_DO_NOT_FAIL_INCRE(20, "", 2, false, 1);
        }
    }


    void analyze_route_data(const PR_tool::algo::RouteData& data, std::FilePath output_file) {
        std::Vector<std::String> message {
            "*****************************Loop Test Result*****************************\n"
        };

        // collector
        float ave_total_length{0.0}, ave_sync_length{0.0}, ave_max_length{0.0}, ave_fail_rate{0.0}, total_sync_net{0.0};
        std::Vector<float> total_length_vec{};
        std::Vector<float> ave_sync_l_vec{};
        std::Vector<std::usize> max_length_vec{};
        std::Vector<std::usize> sync_net_num_vec{};
        std::Vector<std::usize> fail_net_vec{};
        std::Vector<float> monopolized_by_reuse_data_vec{};
        std::Vector<float> has_nonreuse_data_vec{};

        // collect data in each cycle
        for (const auto& [cycle, data_per_cycle]: data.data()) {
            ave_total_length += data_per_cycle._total_length;
            ave_sync_length += data_per_cycle._ave_sync_length * data_per_cycle._sync_net_number;
            ave_max_length += data_per_cycle._max_length;
            ave_fail_rate += data_per_cycle._failed_net == 0 ? 0 : 1;
            total_sync_net += data_per_cycle._sync_net_number;

            total_length_vec.emplace_back(data_per_cycle._total_length);
            ave_sync_l_vec.emplace_back(data_per_cycle._ave_sync_length);
            max_length_vec.emplace_back(data_per_cycle._max_length);
            sync_net_num_vec.emplace_back(data_per_cycle._sync_net_number);
            fail_net_vec.emplace_back(data_per_cycle._failed_net);
            monopolized_by_reuse_data_vec.emplace_back(std::get<0>(data_per_cycle._reg_data));
            has_nonreuse_data_vec.emplace_back(std::get<1>(data_per_cycle._reg_data));
        }

        // print global message
        message.emplace_back(std::format("Average Total Length: {}\n", ave_total_length / (float)data.data().size()));
        message.emplace_back(std::format("Average Sync Length: {}\n", ave_sync_length / total_sync_net));
        message.emplace_back(std::format("Average Max Length: {}\n", ave_max_length / (float)data.data().size()));
        message.emplace_back(std::format("Average Fail Rate: {}\n", ave_fail_rate / (float)data.data().size()));
        message.emplace_back(std::format("Final Monopolized by Reuse: {}\n", monopolized_by_reuse_data_vec.back()));
        message.emplace_back(std::format("Final Has Nonreuse: {}\n", has_nonreuse_data_vec.back()));
        message.emplace_back(
            "**************************End of Loop Test Result**************************\n"
        );

        std::String m{};
        for (const auto& msg: message) {
            m += msg + "\n";
        }
        debug::info(m);

        // write to file
        std::ofstream output_file_stream(output_file);
        if (!output_file_stream.is_open()) {
            debug::exception_in("analyze_route_data", "output file open failure");
        }
        auto output = [&]<typename T>(const std::Vector<T>& vec) {
            for (const auto& v: vec) {
                output_file_stream << std::format("{} ", v);
            }
            output_file_stream << "\n";
        };

        output_file_stream << "// cycle, total_length, ave_sync_length, max_length, sync_net_number, failed_net, monopolized_by_reuse, has_nonreuse\n";
        output_file_stream << data.data().size() << "\n";
        output(total_length_vec);
        output(ave_sync_l_vec);
        output(max_length_vec);
        output(sync_net_num_vec);
        output(fail_net_vec);
        output(monopolized_by_reuse_data_vec);
        output(has_nonreuse_data_vec);
    }

    void test_case(std::usize id, std::usize mode, bool try_all_modes, algo::RouteData& data, std::usize cycle) {
        std::FilePath config_path{"../test/config/case" + std::to_string(id)};
                
        auto [interposer, basedie, register_map] = PR_tool::parse::read_config(config_path, mode, try_all_modes);
        algo::build_nets(basedie.get(), interposer.get());
        basedie->merge_same_mode_nets();
        auto [has_bits, has_other_bits] = parse::read_controlbits(config_path, interposer.get(), basedie.get(), mode, try_all_modes);
        if (!has_bits) {
            auto data_per_cycle = algo::route_nets(interposer.get(), basedie.get(), algo::MazeRouteStrategy{true}, algo::HK{}, mode, true, try_all_modes, has_other_bits);
            data.collect_data_in_cycle(cycle, data_per_cycle);

            std::string controlbits_file{"./" + std::to_string(cycle + 1)};
            parse::output_from_routing_results(interposer.get(), controlbits_file, basedie.get(), mode, try_all_modes, false, register_map);
        }
        else {
            debug::info("Already has control bits, skip the routing process");
        }
    }

}



