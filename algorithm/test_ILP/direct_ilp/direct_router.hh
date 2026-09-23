#pragma once

#include "direct_ilp/direct_scope.hh"
#include "direct_ilp/undirected_graph.hh"

namespace PR_tool {

struct DirectIlpOptions {
    int verbose_level{0};
    int time_limit_minutes{0};
    std::String highs_log_path;
};

struct DirectIlpStats {
    std::size_t commodities{0};
    std::size_t owners{0};
    std::size_t edge_vars{0};
    std::size_t node_vars{0};
    std::size_t owner_vars{0};
    std::size_t switch_vars{0};
    std::size_t mode_vars{0};
    std::size_t variables{0};
    std::size_t rows{0};
    std::size_t nonzeros{0};
    long long build_ms{0};
    long long solve_ms{0};
    double objective{0};
    double bound{0};
    double gap{0};
    std::String status;
};

struct DirectIlpResult {
    RoutingResult route;
    DirectIlpStats stats;
};

auto solve_direct_ilp(const UnifiedGraph& graph, const DirectGraph& direct,
                      const std::Vector<RoutingNet>& nets,
                      const std::Vector<RoutingScope>& scopes,
                      const DirectIlpOptions& options) -> DirectIlpResult;

} // namespace PR_tool
