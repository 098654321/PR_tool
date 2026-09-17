#pragma once

#include "common/routing_types.hh"
#include "global_route_v17/global_router.hh"

#include <cstddef>
#include <std/collection.hh>
#include <std/string.hh>
#include <string_view>

namespace PR_tool {

struct PnSourcePreselectionStats {
    std::size_t pn_nets{0};
    std::size_t pn_bumps{0};
    std::size_t pn_01_ports{0};
    std::size_t candidates{0};
    std::size_t source_activations{0};
    std::size_t overflow_vars{0};
    std::size_t constraints{0};
    std::size_t source_trees{0};
    std::size_t fixed_tnet_bumps{0};
    double lambda_a_base{1.0};
    double k_hat{1.0};
    double lambda_a{1.0};
    double lambda_r{1.0};
    double alpha{0.0};
    double objective{0.0};
    long long build_ms{0};
    long long solve_ms{0};
    long long total_ms{0};
};

struct PnSourcePreselectionResult {
    bool ok{false};
    std::String message;
    std::Vector<RoutingNet> nets;
    PnSourcePreselectionStats stats;
};

// V18-only endpoint preselection.  Every PNnet is replaced by one fixed-source,
// fixed-unit multi-sink Tnet per selected physical source.
auto preselect_pn_sources_v18(
    const GlobalChannelGraph& graph,
    const std::Vector<RoutingNet>& nets,
    int verbose_level,
    std::string_view highs_log_path = {}
) -> PnSourcePreselectionResult;

} // namespace PR_tool
