#include "resource_model.hh"
#include "hardware_graph.hh"

#include <algorithm>
#include <cmath>

namespace PR_tool::air316 {

// Cost projection from 0d6e86f:test_ILP/route_ilp/rrr_cost.cc. Keep it
// independent of the AIR capacity projection: pricing must not change legality.
struct RouteCostProjection {
    std::Map<std::pair<int, int>, std::Vector<ResourceKey>> arcs;
    std::Map<std::pair<int, bool>, std::Set<int>> mode_switches;

    explicit RouteCostProjection(const UnifiedGraph& graph) {
        for (const auto& arc : graph.arcs) {
            auto keys = std::Vector<ResourceKey>{node_resource(arc.v)};
            const auto ku = graph.nodes[arc.u].kind, kv = graph.nodes[arc.v].kind;
            if (ku == UnifiedNodeKind::HLine || ku == UnifiedNodeKind::VLine)
                keys.push_back(node_resource(arc.u));
            if (arc.physical_switch_id >= 0)
                keys.push_back(switch_resource(arc.physical_switch_id));
            const int bump = ku == UnifiedNodeKind::Bump ? arc.u :
                kv == UnifiedNodeKind::Bump ? arc.v : -1;
            const int h = ku == UnifiedNodeKind::HLine ? arc.u :
                kv == UnifiedNodeKind::HLine ? arc.v : -1;
            const int v = ku == UnifiedNodeKind::VLine ? arc.u :
                kv == UnifiedNodeKind::VLine ? arc.v : -1;
            const int t = ku == UnifiedNodeKind::Track ? arc.u :
                kv == UnifiedNodeKind::Track ? arc.v : -1;
            if (bump >= 0 && h >= 0) {
                keys.push_back(matching_endpoint_key(bump, 0));
                keys.push_back(matching_endpoint_key(h, 1));
                keys.push_back(tob_mux_input_key(bump, h));
                keys.push_back(tob_mux_output_key(h, bump));
            }
            if (h >= 0 && v >= 0) {
                keys.push_back(matching_endpoint_key(h, 2));
                keys.push_back(matching_endpoint_key(v, 3));
                keys.push_back(tob_mux_input_key(h, v));
                keys.push_back(tob_mux_output_key(v, h));
            }
            if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack && v >= 0 && t >= 0) {
                keys.push_back(tob_mux_input_key(v, t));
                keys.push_back(tob_mux_output_key(t, v));
            }
            if (arc.mode_group_id >= 0) {
                keys.push_back(arc.is_vline_track_straight ?
                    mode_straight_key(arc.mode_group_id) : mode_swap_key(arc.mode_group_id));
                mode_switches[{arc.mode_group_id, arc.is_vline_track_straight}]
                    .insert(arc.physical_switch_id);
            }
            arcs.emplace(std::pair{arc.u, arc.v}, std::move(keys));
        }
    }
};

auto ResourceModel::arc_cost(const UnifiedGraph& graph, OwnerId owner, const UnifiedArc& arc) const
    -> double {
    if (!graph.route_cost_projection)
        graph.route_cost_projection = std::make_shared<RouteCostProjection>(graph);
    const auto& projection = *graph.route_cost_projection;
    const auto owns = [&](const ResourceKey& key) {
        if (key.kind == ResourceKind::Node)
            return key.id == arc.u || holds(owner, key);
        if (is_mux_port_key(key))
            return holds(owner, switch_resource(arc.physical_switch_id));
        if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
            const auto it = projection.mode_switches.find({key.id, key.kind == ResourceKind::ModeStraight});
            return it != projection.mode_switches.end() && std::ranges::any_of(it->second,
                [&](int id) { return holds(owner, switch_resource(id)); });
        }
        return holds(owner, key);
    };
    const auto incremental = [&](const ResourceKey& key) {
        int u = occupancy_count(key) + 1;
        if (is_mux_port_key(key))
            u = mux_distinct_peers(key) + (has_any_owner(key) ? 0 : 1);
        const double present = 1.0 + 4.0 / (std::exp(1.0 - u) + 1.0) +
            4.0 / 20.0 * std::max(0, u - 1);
        return type_weight(key) * present + history(key);
    };
    double cost = 1.0;
    for (const auto& key : projection.arcs.at({arc.u, arc.v})) {
        if (owns(key)) continue;
        cost += incremental(key);
        if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
            const auto opposite = key.kind == ResourceKind::ModeStraight ?
                mode_swap_key(key.id) : mode_straight_key(key.id);
            if (has_any_owner(opposite)) cost += incremental(mode_conflict_key(key.id));
        }
    }
    return cost;
}

} // namespace PR_tool::air316
