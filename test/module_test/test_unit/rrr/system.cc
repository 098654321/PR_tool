#include "controlbits_check.hh"
#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/route_nets.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <algo/router/rrr/net_adapter.hh>
#include <algo/placer/place.hh>
#include <algo/placer/sa/saplacestrategy.hh>
#include <parse/reader/module.hh>
#include <parse/writer/module.hh>
#include <fstream>
#include <sstream>

namespace {
auto require(bool ok,const char* message) -> void {if(!ok)throw std::runtime_error(message);}
}
void test_rrr_system_main(int argc,char** argv) {
    using namespace PR_tool;
    require(argc>=4,"rrr_system CONFIG OUTPUT [place|verify|verify-sparse] [seconds]");
    const std::filesystem::path output=argv[3];
    const std::string mode=argc>4?argv[4]:"route";
    std::filesystem::create_directories(output);
    debug::initial_log(output/"system-check.log");
    auto [interposer,basedie,register_map]=parse::read_config(argv[2],0,false);
    algo::build_nets(basedie.get(),interposer.get());
    if(mode=="place") {
        std::vector<circuit::TopDieInstance*> instances;
        for(const auto& [name,instance]:basedie->topdie_insts())instances.push_back(instance.get());
        algo::SAPlaceStrategy strategy;
        algo::place(interposer.get(),instances,basedie.get(),strategy);
        require(strategy.is_valid_placement(interposer.get(),instances),"invalid placement");
    }
    const bool verify=mode=="verify" || mode=="verify-sparse";
    if(!verify) {
        std::optional<double> seconds;
        if(argc>5)seconds=std::stod(argv[5]);
        auto result=algo::route_nets(interposer.get(),basedie.get(),algo::MazeRouteStrategy{},algo::HK{},0,false,false,false,{},seconds);
        require(result.failed_net_names.empty(),"system router found no legal solution");
    }
    const auto& shared=basedie->nets().at(0);
    const auto nets=rrr::build_routing_nets(shared);
    const auto graph=rrr::build_hardware_graph(interposer.get(),nets);
    std::vector<circuit::Net*> raw;for(const auto& net:shared)raw.push_back(net.get());
    if(verify) {
        test::check_routed_controlbits(interposer.get(),graph,nets,raw,register_map,
            output/"regnamecontrolbit_4part",mode=="verify-sparse");
    } else {
        const auto expected=rrr::import_paths(interposer.get(),graph,nets,raw);
        parse::output_from_routing_results(interposer.get(),output,basedie.get(),0,false,false,register_map);
        test::check_routed_controlbits(interposer.get(),graph,nets,raw,register_map,
            output/"regnamecontrolbit_4part",false,&expected);
        parse::output_from_routing_results(interposer.get(),output/"sparse",basedie.get(),0,false,true,register_map);
        test::check_routed_controlbits(interposer.get(),graph,nets,raw,register_map,
            output/"sparse/regnamecontrolbit_4part",true,&expected);
        // Deliberately sever an active COB switch in the actual artifact. Restore
        // the fixture after checking, even if verification unexpectedly passes.
        bool mutated=false;
        for(const auto& [file,addresses]:register_map) {
            const auto path=output/"regnamecontrolbit_4part"/file;
            std::ifstream in(path);std::string original((std::istreambuf_iterator<char>(in)),{});
            std::istringstream rows(original);std::ostringstream changed;std::string line;
            while(std::getline(rows,line)) {
                std::istringstream row(line);std::string hex,address,name;row>>hex>>address>>name;
                if(!mutated && name.starts_with("cob_") && name.find("_sw_")!=std::string::npos && std::stoul(hex,nullptr,16)!=0) {
                    hex="00000000";line=hex+" "+address+" "+name;mutated=true;
                }
                changed<<line<<'\n';
            }
            if(!mutated)continue;
            {std::ofstream out(path);out<<changed.str();}
            bool rejected=false;
            try {test::check_routed_controlbits(interposer.get(),graph,nets,raw,register_map,
                output/"regnamecontrolbit_4part",false,&expected);} catch(const std::exception&) {rejected=true;}
            {std::ofstream out(path);out<<original;}
            require(rejected,"decoder accepted a severed routed switch");
            // Missing active hardware addresses must fail before creating files.
            auto incomplete=register_map;
            std::istringstream original_rows(original);
            while(std::getline(original_rows,line)) {
                std::istringstream row(line);std::string hex,address,name;row>>hex>>address>>name;
                if(name.starts_with("cob_") && name.find("_sw_")!=std::string::npos && std::stoul(hex,nullptr,16)!=0) {
                    incomplete.at(file).erase(name);break;
                }
            }
            bool map_rejected=false;
            const auto rejected_output=output/"rejected-map";
            try {parse::output_from_routing_results(interposer.get(),rejected_output,basedie.get(),0,false,false,incomplete);}
            catch(const std::exception& error) {map_rejected=std::string(error.what()).find("missing from register_map")!=std::string::npos;}
            interposer->reset_regs();
            require(map_rejected && !std::filesystem::exists(rejected_output/"regnamecontrolbit_4part"),
                    "writer must reject missing active addresses without partial output");
            break;
        }
        require(mutated,"fixture must contain an active COB switch for decoder negative test");
    }
    debug::info("RRR system controlbits validation PASS");
}
