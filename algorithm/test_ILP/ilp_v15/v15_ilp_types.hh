#pragma once

#include "common/routing_types.hh"

#include <cstddef>
#include <map>
#include <set>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct V15IlpOptimizeOptions {
    bool enabled{false};
    double stretch_threshold_percent{0.0};
    int verbose_level{0};
    std::String gurobi_log_dir{"./gurobi"};
};

struct IlpCommodity {
    std::size_t commodity_id{0};
    std::size_t routing_net_id{0};
    std::size_t scope_index{0};
    int source_node{-1};
    std::Vector<int> sink_nodes;
    std::Vector<std::size_t> demand_ids;
    std::Vector<std::size_t> source_indices;
    int k{0};
    bool is_bus_member{false};
};

struct V15LockedResources {
    std::Vector<bool> node_used;
    std::set<int> switch_used;
};

struct V15MipStart {
    bool available{false};
    std::map<std::size_t, std::set<int>> selected_arc_ids;
    std::map<std::pair<std::size_t, int>, int> flow_by_arc;
    std::map<std::size_t, std::set<int>> used_node_ids;
    std::map<int, bool> mode_straight;
};

enum class V15IlpStatus {
    SkippedNoCandidates,
    Optimal,
    Suboptimal,
    Failed,
};

struct V15IlpStats {
    std::size_t selected_nets{0};
    std::size_t locked_nets{0};
    std::size_t commodities{0};
    std::size_t f_vars{0};
    std::size_t x_vars{0};
    std::size_t y_vars{0};
    std::size_t mode_vars{0};
    std::size_t constraints{0};
    std::size_t nonzeros{0};
    long long model_build_ms{0};
    long long solve_ms{0};
    double objective{0.0};
    double best_bound{0.0};
    double mip_gap{0.0};
    int solution_count{0};
};

struct V15IlpOptimizeResult {
    V15IlpStatus status{V15IlpStatus::Failed};
    std::String message;
    SatRoutingResult routing;
    V15IlpStats stats;
    std::set<std::size_t> selected_net_ids;
};

} // namespace PR_tool
