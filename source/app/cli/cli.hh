#pragma once

#include "algo/router/backend/route_status.hh"
#include "algo/router/backend/router_backend.hh"
#include "std/string.hh"
#include "std/utility.hh"
#include "std/file.hh"
#include <vector>


namespace PR_tool::hardware {
    class Interposer;
}


namespace PR_tool::circuit {
    class BaseDie;
    class TopDieInstance;
}


namespace PR_tool {

    auto cli_main(
        std::StringView config_path, std::Option<std::StringView> output_path, 
        int mode, std::optional<int> compare, bool try_all_modes, bool placement,
        bool simplify_controlbits,
        algo::RouterKind router_kind,
        const algo::SatRouterCliOptions& sat_opts
    ) -> int;

    auto place(PR_tool::hardware::Interposer*, PR_tool::circuit::BaseDie*, std::vector<PR_tool::circuit::TopDieInstance*>&) -> void;

    auto route(
        PR_tool::hardware::Interposer*, PR_tool::circuit::BaseDie*,
        std::StringView,
        int mode, std::optional<int> compare, bool try_all_modes,
        algo::RouterKind router_kind,
        const algo::SatRouterCliOptions& sat_opts
    ) -> RouteStatus;
}