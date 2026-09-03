#pragma once

#include "route_status.hh"
#include "router_backend.hh"

namespace PR_tool::algo {

class SatRouterBackend final : public RouterBackend {
public:
    auto run(
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        const RouterOptions& options
    ) -> RouteStatus override;
};

} // namespace PR_tool::algo
