#pragma once
#include <algo/router/routeengine.hh>
#include "rrr_types.hh"

namespace PR_tool::rrr {
auto finish_routing(hardware::Interposer* interposer, circuit::BaseDie* basedie,
                    algo::RouteEngine& engine, const RrrParams& params) -> bool;
}
