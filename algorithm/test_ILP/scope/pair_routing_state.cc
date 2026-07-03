#include "scope/pair_routing_state.hh"

#include "delay/pair_delay_precompute.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_scope.hh"
#include "scope/scope_bbox.hh"

#include <algorithm>
#include <format>
#include <limits>
#include <stdexcept>

namespace PR_tool {

namespace {

auto pair_bbox_for_demand(const RoutingNet& net, std::size_t demand_index) -> IlpBoundingBox {
    if (net.kind == RoutingNetKind::PNnet) {
        return compute_pnnet_demand_pair_bbox(net, demand_index);
    }
    const auto child_boxes = compute_scope_child_bboxes(net);
    if (demand_index >= child_boxes.size()) {
        return clamp_bbox_to_cob_array(IlpBoundingBox {0, 0, 0, 0});
    }
    return child_boxes[demand_index];
}

} // namespace

auto init_routing_problem_state(const std::Vector<RoutingNet>& nets) -> RoutingProblemState {
    auto state = RoutingProblemState {};
    for (const auto& net : nets) {
        for (std::size_t demand_index = 0; demand_index < net.demands.size(); ++demand_index) {
            const auto& demand = net.demands[demand_index];
            if (demand.candidate_source_indices.empty()) {
                throw std::invalid_argument("pair routing state requires at least one candidate source");
            }
            std::size_t source_index = 0;
            if (net.kind != RoutingNetKind::PNnet) {
                if (demand.candidate_source_indices.size() != 1) {
                    throw std::invalid_argument("pair routing state requires exactly one candidate source");
                }
                source_index = demand.candidate_source_indices.front();
            }
            auto pair = PairRoutingState {};
            pair.key = PairKey {net.net_id, demand.demand_id, source_index};
            pair.pair_bbox = pair_bbox_for_demand(net, demand_index);
            const auto pair_index = state.pairs.size();
            state.pairs.push_back(std::move(pair));
            state.pair_index_by_key.emplace(pair.key, pair_index);
            state.pair_indices_by_net[net.net_id].push_back(pair_index);
        }
    }
    return state;
}

auto apply_state_to_nets(const RoutingProblemState& state, std::Vector<RoutingNet>& nets) -> void {
    for (auto& net : nets) {
        const auto net_it = state.pair_indices_by_net.find(net.net_id);
        if (net_it == state.pair_indices_by_net.end() || net_it->second.empty()) {
            continue;
        }
        auto boxes = std::Vector<IlpBoundingBox> {};
        boxes.reserve(net_it->second.size());
        for (const std::size_t pair_index : net_it->second) {
            boxes.push_back(state.pairs[pair_index].pair_bbox);
        }
        net.scope_bbox = rect_hull_boxes(boxes);
        net.has_scope_bbox = true;
    }
}

auto find_pair_state(RoutingProblemState& state, const PairKey& key) -> PairRoutingState* {
    const auto it = state.pair_index_by_key.find(key);
    if (it == state.pair_index_by_key.end()) {
        return nullptr;
    }
    return &state.pairs[it->second];
}

auto find_pair_state(const RoutingProblemState& state, const PairKey& key) -> const PairRoutingState* {
    const auto it = state.pair_index_by_key.find(key);
    if (it == state.pair_index_by_key.end()) {
        return nullptr;
    }
    return &state.pairs[it->second];
}

auto append_delays(std::Vector<int>& delays, int d1, int d2) -> void {
    delays.push_back(d1);
    delays.push_back(d2);
    std::sort(delays.begin(), delays.end());
    delays.erase(std::unique(delays.begin(), delays.end()), delays.end());
}

auto max_delay(const std::Vector<int>& delays) -> int {
    if (delays.empty()) {
        return 0;
    }
    return *std::max_element(delays.begin(), delays.end());
}

auto expand_pair_delays(PairRoutingState& pair) -> void {
    const int max_d = max_delay(pair.delays);
    append_delays(pair.delays, max_d + 1, max_d + 2);
}

auto merge_delays_union(std::Vector<int>& target, const std::Vector<int>& extra) -> void {
    target.insert(target.end(), extra.begin(), extra.end());
    std::sort(target.begin(), target.end());
    target.erase(std::unique(target.begin(), target.end()), target.end());
}

auto sync_fanout_delays(RoutingProblemState& state, std::size_t net_id) -> void {
    const auto net_it = state.pair_indices_by_net.find(net_id);
    if (net_it == state.pair_indices_by_net.end()) {
        return;
    }
    auto merged = std::Vector<int> {};
    for (const std::size_t pair_index : net_it->second) {
        merge_delays_union(merged, state.pairs[pair_index].delays);
    }
    for (const std::size_t pair_index : net_it->second) {
        state.pairs[pair_index].delays = merged;
    }
}

auto sync_bus_after_expand(
    RoutingProblemState& state,
    const std::Vector<RoutingNet>& nets,
    std::size_t net_id
) -> void {
    const auto net_it = state.pair_indices_by_net.find(net_id);
    if (net_it == state.pair_indices_by_net.end()) {
        return;
    }
    const auto net_ref = std::find_if(
        nets.begin(),
        nets.end(),
        [&](const RoutingNet& net) { return net.net_id == net_id; });
    if (net_ref == nets.end() || !net_ref->is_sync_bus) {
        return;
    }

    auto merged_delays = std::Vector<int> {};
    auto merged_bbox = IlpBoundingBox {};
    bool has_bbox = false;
    for (const std::size_t pair_index : net_it->second) {
        merge_delays_union(merged_delays, state.pairs[pair_index].delays);
        if (!has_bbox) {
            merged_bbox = state.pairs[pair_index].pair_bbox;
            has_bbox = true;
        }
        else {
            merged_bbox = rect_hull_boxes({merged_bbox, state.pairs[pair_index].pair_bbox});
        }
    }
    for (const std::size_t pair_index : net_it->second) {
        state.pairs[pair_index].delays = merged_delays;
        state.pairs[pair_index].pair_bbox = merged_bbox;
    }
}

auto all_pair_bboxes_full(const RoutingProblemState& state) -> bool {
    if (state.pairs.empty()) {
        return false;
    }
    for (const auto& pair : state.pairs) {
        if (!is_full_chip_bbox(pair.pair_bbox)) {
            return false;
        }
    }
    return true;
}

auto apply_initial_search_padding(
    RoutingProblemState& state,
    std::Vector<RoutingNet>& nets,
    const UnifiedGraph& graph,
    int scope_pad,
    int delay_pad
) -> void {
    if (scope_pad == 0 && delay_pad == 0) {
        return;
    }

    if (scope_pad > 0) {
        for (auto& pair : state.pairs) {
            for (int step = 0; step < scope_pad; ++step) {
                pair.pair_bbox = expand_pair_bbox_one_cell(pair.pair_bbox);
            }
        }
        apply_state_to_nets(state, nets);
    }

    if (delay_pad <= 0) {
        return;
    }

    const auto scopes = build_all_scopes(graph, nets);
    (void)compute_pair_delays(graph, nets, scopes, &state);

    for (auto& pair : state.pairs) {
        if (pair.delays.empty()) {
            throw std::runtime_error(std::format(
                "initial delay padding failed for net {} demand {}: no d_min",
                pair.key.net_id,
                pair.key.demand_id));
        }
        const int d_min = pair.delays.front();
        if (delay_pad > std::numeric_limits<int>::max() - d_min) {
            throw std::overflow_error(std::format(
                "initial delay padding overflows for net {} demand {}: d_min={} pad={}",
                pair.key.net_id,
                pair.key.demand_id,
                d_min,
                delay_pad));
        }
        auto expanded = std::Vector<int> {};
        expanded.reserve(static_cast<std::size_t>(delay_pad) + 1);
        for (int delay = d_min; delay <= d_min + delay_pad; ++delay) {
            expanded.push_back(delay);
        }
        pair.delays = std::move(expanded);
    }
}

} // namespace PR_tool
