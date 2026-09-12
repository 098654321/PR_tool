#pragma once

#include "hardware_graph.hh"
#include "rrr_types.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>

namespace PR_tool {

auto rrr_is_better(
    int overflow,
    std::size_t wirelength,
    int best_overflow,
    std::size_t best_wirelength
) -> bool;

auto rrr_dirty_before(
    int exposure_a,
    int retry_a,
    int hpwl_a,
    OwnerId id_a,
    int exposure_b,
    int retry_b,
    int hpwl_b,
    OwnerId id_b
) -> bool;

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
    int verbose_level = 0
) -> RrrResult;

} // namespace PR_tool
