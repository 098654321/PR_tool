#pragma once

#include "route_ilp/route_search.hh"

namespace PR_tool {

// These cost resources are separate from the route master's capacity rows.
enum class RrrCostKind { Node, Switch, Matching, Straight, Swap, ModeConflict,
                         MuxInput, MuxOutput };
struct RrrCostKey {
    RrrCostKind kind{};
    int id{};
    int extra{-1};
    auto operator<=>(const RrrCostKey&) const = default;
};
struct RrrHistoryUpdate {
    int overflow{};
    std::size_t nonzero_resources{};
    double total{};
};

class RrrCostModel {
public:
    explicit RrrCostModel(const UnifiedGraph& graph);
    // Rebuild from the current transaction, so failed variants cannot leak claims.
    auto set_selection(const std::map<RouteOwner, RouteColumn>& selected) -> void;
    auto set_own(const std::Vector<SourceSinkPairPath>& paths) -> void;
    auto arc_cost(const UnifiedArc& arc, const std::set<RouteResource>& own) const -> double;
    auto path_cost(const std::Vector<int>& path, const std::set<RouteResource>& own) const -> double;
    auto history_next() -> RrrHistoryUpdate;

private:
    using Keys = std::Vector<RrrCostKey>;
    const UnifiedGraph& _graph;
    std::Vector<Keys> _arc_keys;
    std::map<std::pair<int, int>, int> _arc_ids;
    std::map<std::pair<int, bool>, std::set<int>> _mode_switches;
    std::map<RrrCostKey, int> _counts;
    std::map<RrrCostKey, std::set<int>> _mux_peers;
    std::map<RrrCostKey, double> _history;
    std::set<RrrCostKey> _own;
    auto path_keys(const std::Vector<SourceSinkPairPath>& paths) const -> std::set<RrrCostKey>;
    auto owns(const RrrCostKey& key, const UnifiedArc& arc,
              const std::set<RouteResource>& own) const -> bool;
    auto count(RrrCostKey key) const -> int;
    auto overflow(RrrCostKey key) const -> int;
    auto incremental_cost(RrrCostKey key) const -> double;
};

} // namespace PR_tool
