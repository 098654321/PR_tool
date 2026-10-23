#pragma once

#include "hardware_graph.hh"
#include "rrr_types.hh"
#include "path_adapter.hh"

#include <hardware/interposer.hh>
#include <circuit/net/net.hh>
#include <std/collection.hh>

namespace PR_tool::rrr {

auto rrr_initial_before(
    bool bus_a,
    int primary_a,
    int hpwl_a,
    OwnerId id_a,
    bool bus_b,
    int primary_b,
    int hpwl_b,
    OwnerId id_b
) -> bool;

auto run_rrr(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const RrrParams& params,
    hardware::Interposer* interposer,
    const NetPaths& initial,
    const std::Vector<std::size_t>& order
) -> RrrResult;

} // namespace PR_tool
