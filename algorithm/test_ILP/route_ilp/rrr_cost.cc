#include "route_ilp/rrr_cost.hh"

#include <algorithm>
#include <cmath>

namespace PR_tool {
namespace {

auto is_mux(RrrCostKey key) -> bool {
    return key.kind == RrrCostKind::MuxInput || key.kind == RrrCostKind::MuxOutput;
}
auto history_key(RrrCostKey key) -> RrrCostKey {
    if (is_mux(key)) key.extra = -1;
    return key;
}
auto weight(RrrCostKind kind) -> int {
    switch (kind) {
    case RrrCostKind::Node: return 1;
    case RrrCostKind::Switch:
    case RrrCostKind::Matching:
    case RrrCostKind::MuxInput:
    case RrrCostKind::MuxOutput: return 2;
    case RrrCostKind::ModeConflict: return 8;
    default: return 0;
    }
}
auto present(int u) -> double {
    return 1.0 + 4.0 / (std::exp(1.0 - u) + 1.0) +
        4.0 / 20.0 * std::max(0, u - 1);
}

} // namespace

RrrCostModel::RrrCostModel(const UnifiedGraph& graph) : _graph(graph) {
    _arc_keys.reserve(graph.arcs.size());
    for (const auto& arc : graph.arcs) {
        _arc_ids.emplace(std::pair{arc.u, arc.v}, static_cast<int>(_arc_keys.size()));
        auto keys = Keys{};
        if (arc.is_virtual_source_arc) { _arc_keys.push_back({}); continue; }
        const auto ku = graph.nodes[arc.u].kind, kv = graph.nodes[arc.v].kind;
        keys.push_back({RrrCostKind::Node, arc.v});
        if (ku == UnifiedNodeKind::HLine || ku == UnifiedNodeKind::VLine)
            keys.push_back({RrrCostKind::Node, arc.u});
        if (arc.physical_switch_id >= 0)
            keys.push_back({RrrCostKind::Switch, arc.physical_switch_id});
        const int bump = ku == UnifiedNodeKind::Bump ? arc.u :
            kv == UnifiedNodeKind::Bump ? arc.v : -1;
        const int hline = ku == UnifiedNodeKind::HLine ? arc.u :
            kv == UnifiedNodeKind::HLine ? arc.v : -1;
        const int vline = ku == UnifiedNodeKind::VLine ? arc.u :
            kv == UnifiedNodeKind::VLine ? arc.v : -1;
        const int track = ku == UnifiedNodeKind::Track ? arc.u :
            kv == UnifiedNodeKind::Track ? arc.v : -1;
        if (bump >= 0 && hline >= 0) {
            keys.push_back({RrrCostKind::Matching, bump, 0});
            keys.push_back({RrrCostKind::Matching, hline, 1});
            keys.push_back({RrrCostKind::MuxInput, bump, hline});
            keys.push_back({RrrCostKind::MuxOutput, hline, bump});
        }
        if (hline >= 0 && vline >= 0) {
            keys.push_back({RrrCostKind::Matching, hline, 2});
            keys.push_back({RrrCostKind::Matching, vline, 3});
            keys.push_back({RrrCostKind::MuxInput, hline, vline});
            keys.push_back({RrrCostKind::MuxOutput, vline, hline});
        }
        if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack &&
            vline >= 0 && track >= 0) {
            keys.push_back({RrrCostKind::MuxInput, vline, track});
            keys.push_back({RrrCostKind::MuxOutput, track, vline});
            if (arc.mode_group_id >= 0)
                keys.push_back({arc.is_vline_track_straight ? RrrCostKind::Straight :
                    RrrCostKind::Swap, arc.mode_group_id});
        }
        if (arc.mode_group_id >= 0) {
            const auto mode = arc.is_vline_track_straight ? RrrCostKind::Straight : RrrCostKind::Swap;
            if (std::ranges::none_of(keys, [&](auto key) { return key.kind == mode; }))
                keys.push_back({mode, arc.mode_group_id});
            _mode_switches[{arc.mode_group_id, arc.is_vline_track_straight}]
                .insert(arc.physical_switch_id);
        }
        _arc_keys.push_back(std::move(keys));
    }
}

auto RrrCostModel::set_selection(const std::map<RouteOwner, RouteColumn>& selected) -> void {
    _counts.clear(); _mux_peers.clear(); _own.clear();
    for (const auto& [owner, column] : selected) {
        (void)owner;
        for (const auto key : path_keys(column.paths)) {
            ++_counts[key];
            if (is_mux(key)) _mux_peers[history_key(key)].insert(key.extra);
        }
    }
}

auto RrrCostModel::path_keys(const std::Vector<SourceSinkPairPath>& paths) const
    -> std::set<RrrCostKey> {
    auto keys = std::set<RrrCostKey>{};
    for (const auto& path : paths) {
        for (int node : path.node_path) keys.insert({RrrCostKind::Node, node});
        for (std::size_t j = 1; j < path.node_path.size(); ++j) {
            const auto& arc_keys = _arc_keys[_arc_ids.at({path.node_path[j-1], path.node_path[j]})];
            keys.insert(arc_keys.begin(), arc_keys.end());
        }
    }
    return keys;
}

auto RrrCostModel::set_own(const std::Vector<SourceSinkPairPath>& paths) -> void {
    for (const auto key : _own)
        if (--_counts.at(key) == 0) {
            _counts.erase(key);
            if (is_mux(key)) _mux_peers[history_key(key)].erase(key.extra);
        }
    _own = path_keys(paths);
    for (const auto key : _own) {
        ++_counts[key];
        if (is_mux(key)) _mux_peers[history_key(key)].insert(key.extra);
    }
}

auto RrrCostModel::count(RrrCostKey key) const -> int {
    if (key.kind == RrrCostKind::ModeConflict)
        return count({RrrCostKind::Straight, key.id}) > 0 &&
               count({RrrCostKind::Swap, key.id}) > 0 ? 1 : 0;
    const auto it = _counts.find(key);
    return it == _counts.end() ? 0 : it->second;
}

auto RrrCostModel::owns(const RrrCostKey& key, const UnifiedArc& arc,
                        const std::set<RouteResource>& own) const -> bool {
    if (key.kind == RrrCostKind::Node)
        return key.id == arc.u || own.contains({0, key.id, 0});
    if (key.kind == RrrCostKind::Switch) return own.contains({1, key.id, 0});
    if (key.kind == RrrCostKind::Matching) return own.contains({2, key.id, key.extra});
    if (is_mux(key)) return own.contains({1, arc.physical_switch_id, 0});
    if (key.kind == RrrCostKind::Straight || key.kind == RrrCostKind::Swap) {
        const auto it = _mode_switches.find({key.id, key.kind == RrrCostKind::Straight});
        return it != _mode_switches.end() && std::ranges::any_of(it->second,
            [&](int id) { return own.contains({1, id, 0}); });
    }
    return false;
}

auto RrrCostModel::incremental_cost(RrrCostKey key) const -> double {
    int u = count(key) + 1;
    if (is_mux(key)) {
        const auto peers = _mux_peers.find(history_key(key));
        u = (peers == _mux_peers.end() ? 0 : static_cast<int>(peers->second.size())) +
            (count(key) == 0 ? 1 : 0);
    }
    const auto old = _history.find(history_key(key));
    return weight(key.kind) * present(u) + (old == _history.end() ? 0.0 : old->second);
}

auto RrrCostModel::arc_cost(const UnifiedArc& arc,
                            const std::set<RouteResource>& own) const -> double {
    double cost = 1.0;
    for (const auto key : _arc_keys[_arc_ids.at({arc.u, arc.v})]) {
        if (owns(key, arc, own)) continue;
        cost += incremental_cost(key);
        if (key.kind == RrrCostKind::Straight || key.kind == RrrCostKind::Swap) {
            const auto opposite = key.kind == RrrCostKind::Straight ?
                RrrCostKind::Swap : RrrCostKind::Straight;
            if (count({opposite, key.id}) > 0)
                cost += incremental_cost({RrrCostKind::ModeConflict, key.id});
        }
    }
    return cost;
}

auto RrrCostModel::path_cost(const std::Vector<int>& path,
                             const std::set<RouteResource>& own) const -> double {
    double cost = 0.0;
    for (std::size_t j = 1; j < path.size(); ++j)
        cost += arc_cost(_graph.arcs[_arc_ids.at({path[j-1], path[j]})], own);
    return cost;
}

auto RrrCostModel::overflow(RrrCostKey key) const -> int {
    if (is_mux(key)) {
        const auto it = _mux_peers.find(history_key(key));
        return it == _mux_peers.end() ? 0 : std::max(0, static_cast<int>(it->second.size()) - 1);
    }
    if (key.kind == RrrCostKind::ModeConflict) return count(key);
    if (key.kind == RrrCostKind::Straight || key.kind == RrrCostKind::Swap) return 0;
    return std::max(0, count(key) - 1);
}

auto RrrCostModel::history_next() -> RrrHistoryUpdate {
    auto keys = std::set<RrrCostKey>{};
    for (const auto& [key, count] : _counts) {
        (void)count;
        if (key.kind == RrrCostKind::Straight || key.kind == RrrCostKind::Swap)
            keys.insert({RrrCostKind::ModeConflict, key.id});
        else keys.insert(history_key(key));
    }
    for (const auto& [key, value] : _history) { (void)value; keys.insert(key); }
    auto result = RrrHistoryUpdate{};
    for (const auto key : keys) {
        const int over = overflow(key);
        auto& value = _history[key];
        value = 0.9 * value + over;
        result.overflow += over;
        result.total += value;
        if (value > 0.0) ++result.nonzero_resources;
    }
    return result;
}

} // namespace PR_tool
