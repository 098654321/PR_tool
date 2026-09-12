#pragma once

#include "hardware_graph.hh"
#include "rrr_types.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>

namespace PR_tool {

auto validate_rrr_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const RrrResult& result,
    hardware::Interposer* interposer = nullptr
) -> bool;

} // namespace PR_tool
