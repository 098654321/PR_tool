#pragma once

#include "hardware_graph.hh"
#include "rrr_types.hh"

#include <hardware/interposer.hh>
#include <circuit/net/net.hh>
#include <std/collection.hh>

namespace PR_tool::air316 {

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
    int verbose_level = 0,
    const std::Vector<std::Rc<circuit::Net>>* raw_nets = nullptr,
    const std::Vector<std::Vector<std::Vector<int>>>& initial_paths = {}
) -> RrrResult;

} // namespace PR_tool::air316
