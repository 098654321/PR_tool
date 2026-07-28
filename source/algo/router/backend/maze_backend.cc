#include "router_backend.hh"

#include <algo/router/route_nets.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>

#include <parse/reader/module.hh>
#include <parse/comparator/controlbits_parser.hh>

#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

#include <debug/debug.hh>
#include <std/string.hh>

#include <memory>
#include <stdexcept>

namespace PR_tool::algo {

namespace {

class MazeRouterBackend final : public RouterBackend {
public:
    auto run(
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        const RouterOptions& options
    ) -> RouteStatus override {
        debug::debug("Start routing ...");
        const auto mode = options.mode;
        const auto try_all_modes = options.try_all_modes;

        if (!try_all_modes && mode == 0) {
            auto result = route_nets(
                interposer, basedie, MazeRouteStrategy{false}, HK{},
                mode, false, try_all_modes);
            if (!result.failed_net_names.empty()) {
                debug::error("Routing failed for:");
                for (const auto& name : result.failed_net_names) {
                    debug::error(name);
                }
                return RouteStatus::Failed;
            }
            return RouteStatus::Ok;
        }

        basedie->merge_same_mode_nets();
        auto [has_bits, has_other_bits] = parse::read_controlbits(
            options.config_path, interposer, basedie, mode, try_all_modes);
        if (!has_bits) {
            auto result = route_nets(
                interposer, basedie, MazeRouteStrategy{true}, HK{},
                mode, true, try_all_modes, has_other_bits);
            if (!result.failed_net_names.empty()) {
                debug::error("Routing failed for:");
                for (const auto& name : result.failed_net_names) {
                    debug::error(name);
                }
                return RouteStatus::Failed;
            }
        } else {
            debug::info("Already has control bits, skip the routing process");
        }

        if (!try_all_modes && options.compare.has_value()) {
            std::string current_file {"controlbits_" + std::to_string(mode) + ".txt"};
            std::string target_file {
                "controlbits_" + std::to_string(options.compare.value()) + ".txt"};
            parse::compare(current_file, target_file);
        }

        return has_bits ? RouteStatus::Skipped : RouteStatus::Ok;
    }
};

} // namespace

auto make_router(RouterKind kind) -> std::unique_ptr<RouterBackend> {
    switch (kind) {
    case RouterKind::Maze:
        return std::make_unique<MazeRouterBackend>();
    case RouterKind::Sat:
#if PR_TOOL_HAS_SAT_ROUTER
        throw std::runtime_error("sat backend not linked yet");
#else
        throw std::runtime_error("sat router unsupported");
#endif
    }
    throw std::runtime_error("unknown router kind");
}

}
