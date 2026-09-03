#include "./cli.hh"
#include "algo/netbuilder/netbuilder.hh"

#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

#include <algo/router/backend/router_backend.hh>
#include <algo/placer/place.hh>
#include <algo/placer/placestrategy.hh>
#include <algo/placer/sa/saplacestrategy.hh>

#include <parse/reader/module.hh>
#include <parse/writer/module.hh>
#include <std/utility.hh>
#include <std/range.hh>
#include <std/string.hh>
#include <debug/debug.hh>
#include <debug/exception.hh>
#include <std/algorithm.hh>
#include <filesystem>

namespace PR_tool {

    auto cli_main(
        std::StringView config_path, std::Option<std::StringView> output_path, 
        int mode, std::optional<int> compare, bool try_all_modes, bool placement,
        bool simplify_controlbits,
        algo::RouterKind router_kind,
        const algo::SatRouterCliOptions& sat_opts
    ) -> int {
    try {
        std::FilePath output_file = std::FilePath(output_path.has_value() ? *output_path : ".");
        std::filesystem::create_directories(output_file.string());
        debug::initial_log(output_file / "debug.log");

        algo::SatRouterCliOptions effective_sat_opts = sat_opts;
        effective_sat_opts.gurobi_log_dir = (output_file / "gurobi").string();

        auto [interposer, basedie, register_map] = PR_tool::parse::read_config(config_path, mode, try_all_modes); 
        algo::build_nets(basedie.get(), interposer.get());

        if (placement) {
            std::Vector<circuit::TopDieInstance*> topdies;
            for (auto& [name, topdie_inst] : basedie->topdie_insts()) {
                topdies.push_back(topdie_inst.get());
            }
            if (topdies.empty()) {
                debug::warning("No chip instances require layout");
            }
            
            place(interposer.get(), basedie.get(), topdies);
        }
        
        auto route_status = route(
            interposer.get(), basedie.get(), config_path, mode, compare, try_all_modes,
            router_kind, effective_sat_opts
        );
        if (route_status == RouteStatus::Failed) {
            return 1;
        }
        if (route_status == RouteStatus::Ok) {
            parse::output_from_routing_results(interposer.get(), output_file, basedie.get(), mode, try_all_modes, simplify_controlbits, register_map);
        }

        return 0;
    }
    catch (const Exception& err){
        debug::exception("Unexpected exception");
        return 1;
    }
    }

    auto place(PR_tool::hardware::Interposer* interposer, PR_tool::circuit::BaseDie* basedie, std::vector<PR_tool::circuit::TopDieInstance*>& topdies) -> void {
        debug::debug("Start layout ...");
// start time
auto start_time = std::chrono::high_resolution_clock::now();
        auto strategy = algo::SAPlaceStrategy();
        place(interposer, topdies, basedie, strategy);
        assert(strategy.is_valid_placement(interposer, topdies));
        auto end_time1 = std::chrono::high_resolution_clock::now();

        debug::info("Layout result:");
        for (const auto& topdie : topdies) {
            if (topdie->tob()) {
                debug::info_fmt("Chip {} is located in {}", topdie->name(), topdie->tob()->coord());
            } else {
                debug::warning_fmt("Chip {} has not been assigned a TOB", topdie->name());
            }
        }
// end time
auto end_time = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
debug::info_fmt("Layout time: {} milliseconds", duration.count());
    }

    auto route(
        PR_tool::hardware::Interposer* interposer, PR_tool::circuit::BaseDie* basedie,
        std::StringView config_path,
        int mode, std::optional<int> compare, bool try_all_modes,
        algo::RouterKind router_kind,
        const algo::SatRouterCliOptions& sat_opts
    ) -> RouteStatus {
        algo::RouterOptions options;
        options.kind = router_kind;
        options.sat = sat_opts;
        options.mode = mode;
        options.try_all_modes = try_all_modes;
        options.compare = compare;
        options.config_path = config_path;
        return algo::make_router(options.kind)->run(interposer, basedie, options);
    }
    

}