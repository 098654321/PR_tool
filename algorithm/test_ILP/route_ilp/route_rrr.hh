#pragma once

#include "route_ilp/route_master.hh"

#include <chrono>
#include <circuit/net/net.hh>

namespace PR_tool {

// Preserve mode-0 input order without changing ILP net IDs or using net names.
auto rrr_mode0_indices(const std::Vector<std::Rc<circuit::Net>>& ilp_raw_nets,
                      const std::Vector<std::Rc<circuit::Net>>& mode0_raw_nets)
    -> std::Vector<std::size_t>;

auto optimize_route_columns_rrr(const UnifiedGraph& graph,
                                const std::Vector<RoutingNet>& nets,
                                const std::Vector<RoutingScope>& scopes,
                                const RouteIlpResult& baseline,
                                hardware::Interposer* interposer,
                                int verbose_level = 0,
                                std::chrono::steady_clock::time_point deadline =
                                    std::chrono::steady_clock::time_point::max(),
                                const std::Vector<std::Rc<circuit::Net>>* raw_nets = nullptr,
                                const std::Vector<std::size_t>& raw_to_ilp = {})
    -> RoutingResult;

// RRR may change the common bus length; ILP's fixed targets remain unchanged.
auto route_bus_lengths(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                       const RoutingResult& route) -> std::map<std::size_t, std::size_t>;

} // namespace PR_tool
