#include "post_sat_ilp/candidate_router.hh"

#include "common/cob_unit_mask.hh"
#include "sat/routing_path_log.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <debug/debug.hh>
#include <format>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>

#ifdef USE_HIGHS
#include "global_route_v17/highs_log_sink.hh"
#include <Highs.h>
#endif

namespace PR_tool {
namespace {

using PathKey = std::tuple<std::size_t, std::size_t, std::size_t>;
using Edge = std::pair<int, int>;

auto edge_key(const int u, const int v) -> Edge {
    return std::minmax(u, v);
}

auto find_arc(const UnifiedGraph& graph, const int u, const int v) -> int {
    if (u < 0 || static_cast<std::size_t>(u) >= graph.out_arc_ids.size()) {
        return -1;
    }
    for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
        if (graph.arcs[static_cast<std::size_t>(arc_id)].v == v) {
            return arc_id;
        }
    }
    return -1;
}

auto switch_stage(const PhysicalSwitchKind kind) -> int {
    switch (kind) {
    case PhysicalSwitchKind::BumpH: return 0;
    case PhysicalSwitchKind::HLineVLine: return 1;
    case PhysicalSwitchKind::VLineTrack: return 2;
    case PhysicalSwitchKind::None: return -1;
    }
    return -1;
}

auto is_physical_node(const UnifiedGraph& graph, const int node) -> bool {
    return node >= 0 && static_cast<std::size_t>(node) < graph.nodes.size()
        && graph.nodes[static_cast<std::size_t>(node)].kind
            != UnifiedNodeKind::VirtualSource;
}

struct PortKey {
    int node{-1};
    int stage{-1};
    auto operator<=>(const PortKey&) const = default;
};

struct FixedUsage {
    std::set<int> nodes;
    std::set<int> switches;
    std::map<PortKey, int> peer_by_port;
    std::map<int, bool> mode_by_group;
};

struct RouteResources {
    std::set<int> nodes;
    std::set<int> wire_nodes;
    std::set<int> switches;
    std::set<Edge> edges;
    std::map<PortKey, int> peer_by_port;
    std::map<int, bool> mode_by_group;
};

auto add_arc_resources(const UnifiedArc& arc, RouteResources& resources)
    -> bool {
    resources.edges.insert(edge_key(arc.u, arc.v));
    if (arc.physical_switch_id >= 0) {
        resources.switches.insert(arc.physical_switch_id);
        const int stage = switch_stage(arc.physical_switch_kind);
        for (const auto [node, peer] :
             {std::pair{arc.u, arc.v}, std::pair{arc.v, arc.u}}) {
            const auto [it, inserted] =
                resources.peer_by_port.emplace(PortKey{node, stage}, peer);
            if (!inserted && it->second != peer) {
                return false;
            }
        }
    }
    if (arc.mode_group_id >= 0) {
        const bool straight = arc.is_vline_track_straight;
        const auto [it, inserted] =
            resources.mode_by_group.emplace(arc.mode_group_id, straight);
        if (!inserted && it->second != straight) {
            return false;
        }
    }
    return true;
}

auto add_path_resources(const UnifiedGraph& graph,
                        const std::Vector<int>& path,
                        RouteResources& resources) -> bool {
    auto path_nodes = std::set<int>{};
    for (const int node : path) {
        if (!is_physical_node(graph, node) || !path_nodes.insert(node).second) {
            return false;
        }
        resources.nodes.insert(node);
        if (is_wirelength_resource_node(graph, node)) {
            resources.wire_nodes.insert(node);
        }
    }
    for (std::size_t index = 1; index < path.size(); ++index) {
        const int arc_id = find_arc(graph, path[index - 1], path[index]);
        if (arc_id < 0
            || !add_arc_resources(
                graph.arcs[static_cast<std::size_t>(arc_id)], resources)) {
            return false;
        }
    }
    return true;
}

auto collect_fixed_sync(const UnifiedGraph& graph,
                        const std::Vector<RoutingNet>& nets,
                        const SatRoutingResult& sat) -> std::optional<FixedUsage> {
    auto sync_ids = std::set<std::size_t>{};
    for (const auto& net : nets) {
        if (net.is_sync_bus) {
            sync_ids.insert(net.net_id);
        }
    }
    auto resources = RouteResources{};
    for (const auto& path : sat.paths) {
        if (sync_ids.contains(path.net_id)
            && !add_path_resources(graph, path.node_path, resources)) {
            return std::nullopt;
        }
    }
    return FixedUsage{std::move(resources.nodes),
                      std::move(resources.switches),
                      std::move(resources.peer_by_port),
                      std::move(resources.mode_by_group)};
}

auto conflicts_with_fixed(const FixedUsage& fixed, const UnifiedArc& arc)
    -> bool {
    if (fixed.nodes.contains(arc.u) || fixed.nodes.contains(arc.v)
        || (arc.physical_switch_id >= 0
            && fixed.switches.contains(arc.physical_switch_id))) {
        return true;
    }
    if (arc.physical_switch_id >= 0) {
        const int stage = switch_stage(arc.physical_switch_kind);
        for (const auto [node, peer] :
             {std::pair{arc.u, arc.v}, std::pair{arc.v, arc.u}}) {
            const auto it = fixed.peer_by_port.find({node, stage});
            if (it != fixed.peer_by_port.end() && it->second != peer) {
                return true;
            }
        }
    }
    if (arc.mode_group_id >= 0) {
        const auto it = fixed.mode_by_group.find(arc.mode_group_id);
        if (it != fixed.mode_by_group.end()
            && it->second != arc.is_vline_track_straight) {
            return true;
        }
    }
    return false;
}

auto infer_path_unit(const UnifiedGraph& graph,
                     const std::Vector<int>& path) -> std::optional<std::size_t> {
    auto unit = std::optional<std::size_t>{};
    for (const int node_id : path) {
        if (!is_physical_node(graph, node_id)) {
            return std::nullopt;
        }
        const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
        if (node.kind != UnifiedNodeKind::Track) {
            continue;
        }
        if (unit.has_value() && *unit != node.unit) {
            return std::nullopt;
        }
        unit = node.unit;
    }
    return unit;
}

struct PairSpec {
    std::size_t source_index{0};
    std::size_t demand_id{0};
    int physical_source_node{-1};
    int source{-1};
    int sink{-1};
};

struct NetSpec {
    const RoutingNet* net{nullptr};
    std::size_t unit{0};
    std::Vector<PairSpec> pairs;
    std::Vector<SourceSinkPairPath> incumbent_paths;
};

auto build_net_specs(const UnifiedGraph& graph,
                     const std::Vector<RoutingNet>& nets,
                     const SatRoutingResult& sat)
    -> std::optional<std::Vector<NetSpec>> {
    auto net_by_id = std::map<std::size_t, const RoutingNet*>{};
    for (const auto& net : nets) {
        net_by_id[net.net_id] = &net;
    }
    auto paths_by_net = std::map<std::size_t, std::Vector<SourceSinkPairPath>>{};
    for (const auto& path : sat.paths) {
        const auto it = net_by_id.find(path.net_id);
        if (it != net_by_id.end() && !it->second->is_sync_bus) {
            paths_by_net[path.net_id].push_back(path);
        }
    }
    auto out = std::Vector<NetSpec>{};
    for (const auto& net : nets) {
        if (net.is_sync_bus) {
            continue;
        }
        auto found = paths_by_net.find(net.net_id);
        if (found == paths_by_net.end()
            || found->second.size() != net.demands.size()) {
            return std::nullopt;
        }
        std::sort(found->second.begin(), found->second.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return std::tie(lhs.demand_id, lhs.source_index)
                           < std::tie(rhs.demand_id, rhs.source_index);
                  });
        auto spec = NetSpec{};
        spec.net = &net;
        spec.incumbent_paths = found->second;
        auto fixed_unit = std::optional<std::size_t>{};
        auto seen_demands = std::set<std::size_t>{};
        for (const auto& path : found->second) {
            if (path.node_path.empty() || !seen_demands.insert(path.demand_id).second) {
                return std::nullopt;
            }
            const auto unit = infer_path_unit(graph, path.node_path);
            if (!unit.has_value()
                || (fixed_unit.has_value() && *fixed_unit != *unit)) {
                return std::nullopt;
            }
            fixed_unit = *unit;
            spec.pairs.push_back(PairSpec{
                path.source_index, path.demand_id, path.physical_source_node,
                path.node_path.front(), path.node_path.back()});
        }
        if (!fixed_unit.has_value()) {
            return std::nullopt;
        }
        spec.unit = *fixed_unit;
        out.push_back(std::move(spec));
    }
    return out;
}

struct AccessOption {
    int track{-1};
    std::Vector<int> endpoint_to_track;
};

auto enumerate_access_options(const UnifiedGraph& graph,
                              const FixedUsage& fixed,
                              const int endpoint,
                              const std::size_t unit)
    -> std::Vector<AccessOption> {
    auto out = std::Vector<AccessOption>{};
    if (!is_physical_node(graph, endpoint) || fixed.nodes.contains(endpoint)) {
        return out;
    }
    const auto& endpoint_node = graph.nodes[static_cast<std::size_t>(endpoint)];
    if (endpoint_node.kind == UnifiedNodeKind::Track) {
        if (endpoint_node.unit == unit) {
            out.push_back({endpoint, {endpoint}});
        }
        return out;
    }
    if (endpoint_node.kind != UnifiedNodeKind::Bump) {
        return out;
    }
    const auto unit_mask = unit_bit(unit);
    auto seen_tracks = std::set<int>{};
    for (const int a0_id : graph.out_arc_ids[static_cast<std::size_t>(endpoint)]) {
        const auto& a0 = graph.arcs[static_cast<std::size_t>(a0_id)];
        if (a0.physical_switch_kind != PhysicalSwitchKind::BumpH
            || conflicts_with_fixed(fixed, a0)) {
            continue;
        }
        for (const int a1_id : graph.out_arc_ids[static_cast<std::size_t>(a0.v)]) {
            const auto& a1 = graph.arcs[static_cast<std::size_t>(a1_id)];
            if (a1.physical_switch_kind != PhysicalSwitchKind::HLineVLine
                || !arc_unit_eligible(graph, a1, unit_mask)
                || conflicts_with_fixed(fixed, a1)) {
                continue;
            }
            for (const int a2_id : graph.out_arc_ids[static_cast<std::size_t>(a1.v)]) {
                const auto& a2 = graph.arcs[static_cast<std::size_t>(a2_id)];
                if (a2.physical_switch_kind != PhysicalSwitchKind::VLineTrack
                    || !arc_unit_eligible(graph, a2, unit_mask)
                    || conflicts_with_fixed(fixed, a2)) {
                    continue;
                }
                const auto& track = graph.nodes[static_cast<std::size_t>(a2.v)];
                if (track.kind != UnifiedNodeKind::Track || track.unit != unit
                    || fixed.nodes.contains(a0.v) || fixed.nodes.contains(a1.v)
                    || fixed.nodes.contains(a2.v) || !seen_tracks.insert(a2.v).second) {
                    continue;
                }
                auto resources = RouteResources{};
                auto path = std::Vector<int>{endpoint, a0.v, a1.v, a2.v};
                if (add_path_resources(graph, path, resources)) {
                    out.push_back({a2.v, std::move(path)});
                }
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.track < rhs.track;
    });
    return out;
}

struct FabricGraph {
    std::Vector<std::Vector<int>> neighbors;
};

auto build_fabric_graph(const UnifiedGraph& graph, const FixedUsage& fixed,
                        const std::size_t unit) -> FabricGraph {
    auto out = FabricGraph{};
    out.neighbors.resize(graph.nodes.size());
    for (const auto& arc : graph.arcs) {
        const auto& u = graph.nodes[static_cast<std::size_t>(arc.u)];
        const auto& v = graph.nodes[static_cast<std::size_t>(arc.v)];
        if (arc.physical_switch_kind == PhysicalSwitchKind::None
            && !arc.is_virtual_source_arc
            && u.kind == UnifiedNodeKind::Track
            && v.kind == UnifiedNodeKind::Track
            && u.unit == unit && v.unit == unit
            && !fixed.nodes.contains(arc.u) && !fixed.nodes.contains(arc.v)) {
            out.neighbors[static_cast<std::size_t>(arc.u)].push_back(arc.v);
        }
    }
    for (auto& neighbors : out.neighbors) {
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()),
                        neighbors.end());
    }
    return out;
}

auto shortest_path(const FabricGraph& graph, const int source, const int sink,
                   const std::set<int>& banned_nodes = {},
                   const std::set<std::pair<int, int>>& banned_arcs = {})
    -> std::Vector<int> {
    if (source < 0 || sink < 0
        || static_cast<std::size_t>(source) >= graph.neighbors.size()
        || static_cast<std::size_t>(sink) >= graph.neighbors.size()
        || banned_nodes.contains(source) || banned_nodes.contains(sink)) {
        return {};
    }
    auto predecessor = std::Vector<int>(graph.neighbors.size(), -2);
    auto pending = std::queue<int>{};
    predecessor[static_cast<std::size_t>(source)] = -1;
    pending.push(source);
    while (!pending.empty() && predecessor[static_cast<std::size_t>(sink)] == -2) {
        const int node = pending.front();
        pending.pop();
        for (const int next : graph.neighbors[static_cast<std::size_t>(node)]) {
            if (banned_nodes.contains(next) || banned_arcs.contains({node, next})
                || predecessor[static_cast<std::size_t>(next)] != -2) {
                continue;
            }
            predecessor[static_cast<std::size_t>(next)] = node;
            pending.push(next);
        }
    }
    if (predecessor[static_cast<std::size_t>(sink)] == -2) {
        return {};
    }
    auto out = std::Vector<int>{};
    for (int node = sink; node >= 0;
         node = predecessor[static_cast<std::size_t>(node)]) {
        out.push_back(node);
    }
    std::reverse(out.begin(), out.end());
    return out;
}

constexpr std::size_t kMaxStructuralPathsPerClass = 32;
constexpr std::size_t kMaxMultiStructuralCandidates =
    1U + 2U * kMaxStructuralPathsPerClass;

struct StructuralPathPool {
    std::Vector<std::Vector<int>> shortest;
    std::Vector<std::Vector<int>> longer;
    bool exact_plus_one{false};
    std::size_t zero_excess_detours{0};
    std::size_t positive_excess_detours{0};
};

using CandidateDeadline =
    std::optional<std::chrono::steady_clock::time_point>;

auto check_candidate_deadline(const CandidateDeadline& deadline) -> void {
    if (deadline.has_value()
        && std::chrono::steady_clock::now() >= *deadline) {
        throw std::runtime_error("V22 candidate generation time limit reached");
    }
}

// Build a bounded pool by replacing one contiguous segment of a base shortest
// path.  Zero-excess replacements expose equal-length L/Z-like alternatives;
// the minimum positive-excess replacements expose Lmin+1, or the true next
// strictly-longer length when no +1 path exists.  The pool is intentionally
// bounded because the number of equal shortest paths may be exponential.
auto structural_path_pool_from_base(const FabricGraph& graph,
                                    const std::Vector<int>& first,
                                    const CandidateDeadline& deadline)
    -> StructuralPathPool {
    if (first.empty()) {
        return {};
    }
    auto out = StructuralPathPool{};
    auto shortest_seen = std::set<std::Vector<int>>{first};
    auto longer_seen = std::set<std::Vector<int>>{};
    out.shortest.push_back(first);
    std::size_t best_longer_size = std::numeric_limits<std::size_t>::max();
    auto path_position = std::map<int, std::size_t>{};
    for (std::size_t index = 0; index < first.size(); ++index) {
        path_position.emplace(first[index], index);
    }
    for (std::size_t start_index = 0;
         start_index + 1 < first.size(); ++start_index) {
        check_candidate_deadline(deadline);
        const int start = first[start_index];
        auto predecessor = std::Vector<int>(graph.neighbors.size(), -2);
        auto pending = std::queue<int>{};
        predecessor[static_cast<std::size_t>(start)] = -1;
        pending.push(start);
        std::size_t visited = 0;
        while (!pending.empty()) {
            const int node = pending.front();
            pending.pop();
            if ((++visited & 4095U) == 0U) {
                check_candidate_deadline(deadline);
            }
            for (const int next :
                 graph.neighbors[static_cast<std::size_t>(node)]) {
                if (const auto position = path_position.find(next);
                    position != path_position.end() && next != start) {
                    const std::size_t end_index = position->second;
                    if (end_index <= start_index) {
                        continue;
                    }
                    auto detour = std::Vector<int>{next};
                    for (int cursor = node; cursor >= 0;
                         cursor = predecessor[static_cast<std::size_t>(cursor)]) {
                        detour.push_back(cursor);
                    }
                    std::reverse(detour.begin(), detour.end());
                    auto candidate = std::Vector<int>(
                        first.begin(), first.begin() + start_index + 1);
                    candidate.insert(candidate.end(), detour.begin() + 1,
                                     detour.end());
                    candidate.insert(candidate.end(),
                                     first.begin() + end_index + 1,
                                     first.end());
                    if (candidate.size() == first.size()) {
                        if (shortest_seen.insert(candidate).second) {
                            ++out.zero_excess_detours;
                            if (out.shortest.size()
                                < kMaxStructuralPathsPerClass) {
                                out.shortest.push_back(std::move(candidate));
                            }
                        }
                    } else if (candidate.size() > first.size()) {
                        if (candidate.size() < best_longer_size) {
                            best_longer_size = candidate.size();
                            longer_seen.clear();
                            out.longer.clear();
                            out.positive_excess_detours = 0;
                        }
                        if (candidate.size() == best_longer_size
                            && longer_seen.insert(candidate).second) {
                            ++out.positive_excess_detours;
                            if (out.longer.size()
                                < kMaxStructuralPathsPerClass) {
                                out.longer.push_back(std::move(candidate));
                            }
                        }
                    }
                    continue;
                }
                if (path_position.contains(next)
                    || predecessor[static_cast<std::size_t>(next)] != -2) {
                    continue;
                }
                predecessor[static_cast<std::size_t>(next)] = node;
                pending.push(next);
            }
        }
    }
    out.exact_plus_one = best_longer_size == first.size() + 1;
    return out;
}

auto structural_path_pool(const FabricGraph& graph, const int source,
                          const int sink,
                          const CandidateDeadline& deadline)
    -> StructuralPathPool {
    return structural_path_pool_from_base(
        graph, shortest_path(graph, source, sink), deadline);
}

auto combine_two_pin(const PairSpec& spec,
                     const AccessOption& source_access,
                     const std::Vector<int>& fabric,
                     const AccessOption& sink_access)
    -> SourceSinkPairPath {
    auto path = source_access.endpoint_to_track;
    path.insert(path.end(), fabric.begin() + 1, fabric.end());
    if (sink_access.endpoint_to_track.size() > 1) {
        path.insert(path.end(),
                    std::next(sink_access.endpoint_to_track.rbegin()),
                    sink_access.endpoint_to_track.rend());
    }
    return SourceSinkPairPath{0, spec.source_index, spec.demand_id,
                              spec.physical_source_node, std::move(path)};
}

struct RouteCandidate {
    std::Vector<SourceSinkPairPath> paths;
    RouteResources resources;
    bool incumbent{false};
};

using CongestionKey =
    std::tuple<int, int, int, std::size_t>; // dir, row, col, unit
using CongestionMap = std::map<CongestionKey, double>;

auto track_channel_load(const UnifiedGraph& graph,
                        const std::set<int>& nodes)
    -> CongestionMap {
    auto demand = CongestionMap{};
    for (const int node_id : nodes) {
        const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
        if (node.kind == UnifiedNodeKind::Track) {
            demand[{node.track_dir, node.track_row, node.track_col, node.unit}]
                += 1.0;
        }
    }
    return demand;
}

auto candidate_channel_demand(const UnifiedGraph& graph,
                              const RouteCandidate& candidate)
    -> CongestionMap {
    return track_channel_load(graph, candidate.resources.nodes);
}

auto add_scaled_load(CongestionMap& target, const CongestionMap& contribution,
                     const double scale) -> void {
    constexpr double epsilon = 1e-9;
    for (const auto& [key, value] : contribution) {
        const double updated = target[key] + scale * value;
        if (std::abs(updated) < epsilon) {
            target.erase(key);
        } else {
            target[key] = updated;
        }
    }
}

auto peak_load(const CongestionMap& load) -> double {
    double peak = 0.0;
    for (const auto& [_, value] : load) {
        peak = std::max(peak, value);
    }
    return peak;
}

struct CongestionScore {
    std::size_t overflow_groups{0};
    double overflow_total{0.0};
    double min_residual{std::numeric_limits<double>::infinity()};
    double total_residual{0.0};
    std::uint64_t tie_break{0};
};

auto stable_candidate_hash(const RouteCandidate& candidate,
                           const std::uint64_t salt) -> std::uint64_t {
    auto hash = std::uint64_t{1469598103934665603ULL} ^ salt;
    const auto mix = [&](const std::uint64_t value) {
        hash ^= value + 0x9e3779b97f4a7c15ULL + (hash << 6U) + (hash >> 2U);
    };
    for (const int node : candidate.resources.nodes) {
        mix(static_cast<std::uint64_t>(static_cast<std::uint32_t>(node)));
    }
    for (const auto [u, v] : candidate.resources.edges) {
        mix(static_cast<std::uint64_t>(static_cast<std::uint32_t>(u)) << 32U
            | static_cast<std::uint32_t>(v));
    }
    return hash;
}

auto congestion_score(const UnifiedGraph& graph,
                      const RouteCandidate& candidate,
                      const CongestionMap& load,
                      const std::uint64_t salt) -> CongestionScore {
    auto score = CongestionScore{};
    const auto demand = candidate_channel_demand(graph, candidate);
    for (const auto& [key, amount] : demand) {
        const auto found = load.find(key);
        const double occupied = found == load.end() ? 0.0 : found->second;
        const double residual = 8.0 - occupied - amount;
        score.min_residual = std::min(score.min_residual, residual);
        score.total_residual += residual;
        if (residual < 0.0) {
            ++score.overflow_groups;
            score.overflow_total -= residual;
        }
    }
    score.tie_break = stable_candidate_hash(candidate, salt);
    return score;
}

auto better_congestion_score(const CongestionScore& lhs,
                             const CongestionScore& rhs) -> bool {
    constexpr double epsilon = 1e-9;
    if (lhs.overflow_groups != rhs.overflow_groups) {
        return lhs.overflow_groups < rhs.overflow_groups;
    }
    if (std::abs(lhs.overflow_total - rhs.overflow_total) > epsilon) {
        return lhs.overflow_total < rhs.overflow_total;
    }
    if (std::abs(lhs.min_residual - rhs.min_residual) > epsilon) {
        return lhs.min_residual > rhs.min_residual;
    }
    if (std::abs(lhs.total_residual - rhs.total_residual) > epsilon) {
        return lhs.total_residual > rhs.total_residual;
    }
    return lhs.tie_break < rhs.tie_break;
}

auto best_candidate_index(const UnifiedGraph& graph,
                          const std::Vector<RouteCandidate>& candidates,
                          const CongestionMap& load,
                          const std::uint64_t salt,
                          const bool compare_wirelength = false)
    -> std::optional<std::size_t> {
    auto best = std::optional<std::size_t>{};
    auto best_score = CongestionScore{};
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (best.has_value() && compare_wirelength
            && candidates[index].resources.wire_nodes.size()
                != candidates[*best].resources.wire_nodes.size()) {
            if (candidates[index].resources.wire_nodes.size()
                < candidates[*best].resources.wire_nodes.size()) {
                best = index;
                best_score = congestion_score(
                    graph, candidates[index], load, salt);
            }
            continue;
        }
        const auto score = congestion_score(
            graph, candidates[index], load, salt);
        if (!best.has_value()
            || better_congestion_score(score, best_score)) {
            best = index;
            best_score = score;
        }
    }
    return best;
}

auto analyze_candidate(const UnifiedGraph& graph, const FixedUsage& fixed,
                       const NetSpec& spec,
                       std::Vector<SourceSinkPairPath> paths,
                       const bool incumbent) -> std::optional<RouteCandidate> {
    if (paths.size() != spec.pairs.size()) {
        return std::nullopt;
    }
    std::sort(paths.begin(), paths.end(), [](const auto& lhs, const auto& rhs) {
        return std::tie(lhs.demand_id, lhs.source_index)
             < std::tie(rhs.demand_id, rhs.source_index);
    });
    auto resources = RouteResources{};
    for (std::size_t index = 0; index < paths.size(); ++index) {
        auto& path = paths[index];
        const auto& pair = spec.pairs[index];
        path.net_id = spec.net->net_id;
        if (path.source_index != pair.source_index
            || path.demand_id != pair.demand_id
            || path.physical_source_node != pair.physical_source_node
            || path.node_path.empty() || path.node_path.front() != pair.source
            || path.node_path.back() != pair.sink
            || !add_path_resources(graph, path.node_path, resources)) {
            return std::nullopt;
        }
        const auto unit = infer_path_unit(graph, path.node_path);
        if (!unit.has_value() || *unit != spec.unit) {
            return std::nullopt;
        }
    }
    for (const int node : resources.nodes) {
        if (fixed.nodes.contains(node)) {
            return std::nullopt;
        }
    }
    for (const int physical_switch : resources.switches) {
        if (fixed.switches.contains(physical_switch)) {
            return std::nullopt;
        }
    }
    for (const auto& [port, peer] : resources.peer_by_port) {
        const auto it = fixed.peer_by_port.find(port);
        if (it != fixed.peer_by_port.end() && it->second != peer) {
            return std::nullopt;
        }
    }
    for (const auto& [group, straight] : resources.mode_by_group) {
        const auto it = fixed.mode_by_group.find(group);
        if (it != fixed.mode_by_group.end() && it->second != straight) {
            return std::nullopt;
        }
    }
    return RouteCandidate{std::move(paths), std::move(resources), incumbent};
}

auto candidate_signature(const RouteCandidate& candidate)
    -> std::tuple<std::set<int>, std::set<Edge>, std::map<int, bool>> {
    return {candidate.resources.nodes, candidate.resources.edges,
            candidate.resources.mode_by_group};
}

auto add_candidate_deduplicated(
    std::Vector<RouteCandidate>& candidates,
    std::set<std::tuple<std::set<int>, std::set<Edge>, std::map<int, bool>>>& seen,
    RouteCandidate candidate) -> void {
    const auto signature = candidate_signature(candidate);
    const auto [_, inserted] = seen.insert(signature);
    if (inserted) {
        candidates.push_back(std::move(candidate));
    } else if (candidate.incumbent) {
        const auto it = std::find_if(candidates.begin(), candidates.end(),
            [&](const auto& existing) {
                return candidate_signature(existing) == signature;
            });
        if (it != candidates.end()) {
            it->incumbent = true;
        }
    }
}

struct CandidateStats {
    std::size_t two_pin_combinations{0};
    std::size_t two_pin_shortest{0};
    std::size_t two_pin_plus_one{0};
    std::size_t two_pin_second_shortest{0};
    std::size_t zero_excess_detours{0};
    std::size_t positive_excess_detours{0};
    std::size_t c1_selected{0};
    std::size_t c2_selected{0};
    std::size_t multi_base_runs{0};
    std::size_t multi_forced_runs{0};
    std::size_t multi_segment_variants{0};
    std::size_t multi_failed_runs{0};
    std::size_t incumbent_candidates{0};
};

struct GeneratedCandidates {
    std::Vector<RouteCandidate> candidates;
    std::Vector<RouteCandidate> c1_representative_pool;
    std::optional<RouteCandidate> c1_short_representative;
    std::optional<RouteCandidate> c1_long_representative;
};

auto finalize_c1_representatives(const UnifiedGraph& graph,
                                 const NetSpec& spec,
                                 const CongestionMap& c1_without_net,
                                 GeneratedCandidates& generated) -> void {
    auto unique = std::Vector<RouteCandidate>{};
    auto seen = std::set<std::tuple<std::set<int>, std::set<Edge>,
                                    std::map<int, bool>>>{};
    for (auto& candidate : generated.c1_representative_pool) {
        const auto signature = candidate_signature(candidate);
        if (seen.insert(signature).second) {
            unique.push_back(std::move(candidate));
        }
    }
    generated.c1_representative_pool.clear();
    if (unique.empty()) {
        return;
    }
    std::size_t shortest_length = std::numeric_limits<std::size_t>::max();
    std::size_t longer_length = std::numeric_limits<std::size_t>::max();
    for (const auto& candidate : unique) {
        const std::size_t length = candidate.resources.wire_nodes.size();
        if (length < shortest_length) {
            longer_length = shortest_length;
            shortest_length = length;
        } else if (length > shortest_length && length < longer_length) {
            longer_length = length;
        }
    }
    auto shortest = std::Vector<RouteCandidate>{};
    auto longer = std::Vector<RouteCandidate>{};
    for (const auto& candidate : unique) {
        const std::size_t length = candidate.resources.wire_nodes.size();
        if (length == shortest_length) {
            shortest.push_back(candidate);
        } else if (length == longer_length) {
            longer.push_back(candidate);
        }
    }
    const auto salt =
        (static_cast<std::uint64_t>(spec.net->net_id) << 32U)
        ^ 0xc1c1c1c1ULL;
    if (const auto best = best_candidate_index(
            graph, shortest, c1_without_net, salt)) {
        generated.c1_short_representative = shortest[*best];
    }
    if (const auto best = best_candidate_index(
            graph, longer, c1_without_net, salt + 1U)) {
        generated.c1_long_representative = longer[*best];
    }
}

auto select_from_length_class(
    const UnifiedGraph& graph,
    const std::Vector<RouteCandidate>& pool,
    const CongestionMap& load,
    const std::uint64_t salt,
    std::Vector<RouteCandidate>& out,
    std::set<std::tuple<std::set<int>, std::set<Edge>,
                        std::map<int, bool>>>& seen,
    CandidateStats& stats,
    const bool from_c1,
    std::Vector<RouteCandidate>* representative_pool) -> void {
    const auto best = best_candidate_index(graph, pool, load, salt);
    if (!best.has_value()) {
        return;
    }
    const auto selected = pool[*best];
    if (from_c1) {
        ++stats.c1_selected;
        if (representative_pool != nullptr) {
            representative_pool->push_back(selected);
        }
    } else {
        ++stats.c2_selected;
    }
    add_candidate_deduplicated(out, seen, selected);
}

auto generate_two_pin_candidates(const UnifiedGraph& graph,
                                 const FixedUsage& fixed,
                                 const NetSpec& spec,
                                 const FabricGraph& fabric,
                                 const CongestionMap& c1_without_net,
                                 const CongestionMap& c2,
                                 CandidateStats& stats,
                                 const CandidateDeadline& deadline)
    -> GeneratedCandidates {
    auto generated = GeneratedCandidates{};
    auto seen = std::set<std::tuple<std::set<int>, std::set<Edge>,
                                    std::map<int, bool>>>{};
    const auto& pair = spec.pairs.front();
    const auto sources = enumerate_access_options(
        graph, fixed, pair.source, spec.unit);
    const auto sinks = enumerate_access_options(
        graph, fixed, pair.sink, spec.unit);
    debug::info_fmt(
        "V22 selectable_tracks: net=\"{}\" id={} demand={} source_tracks={} sink_tracks={} combinations={}",
        spec.net->name, spec.net->net_id, pair.demand_id, sources.size(),
        sinks.size(), sources.size() * sinks.size());
    for (const auto& source : sources) {
        for (const auto& sink : sinks) {
            check_candidate_deadline(deadline);
            const std::size_t combination = stats.two_pin_combinations++;
            const auto structural = structural_path_pool(
                fabric, source.track, sink.track, deadline);
            stats.zero_excess_detours += structural.zero_excess_detours;
            stats.positive_excess_detours +=
                structural.positive_excess_detours;
            if (structural.shortest.empty()) {
                continue;
            }
            auto shortest_pool = std::Vector<RouteCandidate>{};
            auto longer_pool = std::Vector<RouteCandidate>{};
            for (const auto& path : structural.shortest) {
                auto pair_path = combine_two_pin(pair, source, path, sink);
                if (auto candidate = analyze_candidate(
                        graph, fixed, spec, {std::move(pair_path)}, false)) {
                    shortest_pool.push_back(std::move(*candidate));
                }
            }
            for (const auto& path : structural.longer) {
                auto pair_path = combine_two_pin(pair, source, path, sink);
                if (auto candidate = analyze_candidate(
                        graph, fixed, spec, {std::move(pair_path)}, false)) {
                    longer_pool.push_back(std::move(*candidate));
                }
            }
            const auto salt_base =
                (static_cast<std::uint64_t>(spec.net->net_id) << 32U)
                ^ static_cast<std::uint64_t>(combination * 4U);
            select_from_length_class(
                graph, shortest_pool, c2, salt_base,
                generated.candidates, seen, stats, false, nullptr);
            select_from_length_class(
                graph, longer_pool, c2, salt_base + 1U,
                generated.candidates, seen, stats, false, nullptr);
            select_from_length_class(
                graph, shortest_pool, c1_without_net, salt_base + 2U,
                generated.candidates, seen, stats, true,
                &generated.c1_representative_pool);
            select_from_length_class(
                graph, longer_pool, c1_without_net, salt_base + 3U,
                generated.candidates, seen, stats, true,
                &generated.c1_representative_pool);
            stats.two_pin_shortest += shortest_pool.empty() ? 0U : 1U;
            if (!longer_pool.empty()) {
                if (structural.exact_plus_one) {
                    ++stats.two_pin_plus_one;
                } else {
                    ++stats.two_pin_second_shortest;
                }
            }
        }
    }
    finalize_c1_representatives(graph, spec, c1_without_net, generated);
    if (auto incumbent = analyze_candidate(
            graph, fixed, spec, spec.incumbent_paths, true)) {
        ++stats.incumbent_candidates;
        add_candidate_deduplicated(
            generated.candidates, seen, std::move(*incumbent));
    }
    return generated;
}

struct TerminalGroup {
    int endpoint{-1};
    std::Vector<AccessOption> access;
};

auto build_union_path(const std::map<int, std::set<int>>& adjacency,
                      const int source, const int sink) -> std::Vector<int> {
    auto predecessor = std::map<int, int>{{source, -1}};
    auto pending = std::queue<int>{};
    pending.push(source);
    while (!pending.empty() && !predecessor.contains(sink)) {
        const int node = pending.front();
        pending.pop();
        const auto found = adjacency.find(node);
        if (found == adjacency.end()) {
            continue;
        }
        for (const int next : found->second) {
            if (!predecessor.contains(next)) {
                predecessor[next] = node;
                pending.push(next);
            }
        }
    }
    if (!predecessor.contains(sink)) {
        return {};
    }
    auto path = std::Vector<int>{};
    for (int node = sink; node >= 0; node = predecessor.at(node)) {
        path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

auto add_union_path(const std::Vector<int>& path,
                    std::map<int, std::set<int>>& adjacency) -> void {
    for (std::size_t index = 1; index < path.size(); ++index) {
        adjacency[path[index - 1]].insert(path[index]);
        adjacency[path[index]].insert(path[index - 1]);
    }
    if (path.size() == 1) {
        adjacency[path.front()];
    }
}

auto access_compatible(const UnifiedGraph& graph,
                       const AccessOption& access,
                       const RouteResources& existing) -> bool {
    auto trial = existing;
    return add_path_resources(graph, access.endpoint_to_track, trial);
}

auto generate_multi_tree(const UnifiedGraph& graph,
                         const FixedUsage& fixed,
                         const NetSpec& spec,
                         const FabricGraph& fabric,
                         const std::Vector<TerminalGroup>& groups,
                         const int forced_group,
                         const std::size_t forced_option,
                         const CandidateDeadline& deadline)
    -> std::optional<RouteCandidate> {
    if (groups.empty()) {
        return std::nullopt;
    }
    const std::size_t start_group = forced_group >= 0
        ? static_cast<std::size_t>(forced_group) : 0;
    const std::size_t start_option = forced_group >= 0 ? forced_option : 0;
    if (start_group >= groups.size()
        || start_option >= groups[start_group].access.size()) {
        return std::nullopt;
    }
    auto connected = std::Vector<bool>(groups.size(), false);
    auto selected = std::Vector<std::size_t>(groups.size(), 0);
    auto tree_tracks = std::set<int>{};
    auto local_resources = RouteResources{};
    auto adjacency = std::map<int, std::set<int>>{};
    const auto select_access = [&](const std::size_t group,
                                   const std::size_t option) -> bool {
        const auto& access = groups[group].access[option];
        if (!add_path_resources(graph, access.endpoint_to_track,
                                local_resources)) {
            return false;
        }
        selected[group] = option;
        connected[group] = true;
        tree_tracks.insert(access.track);
        add_union_path(access.endpoint_to_track, adjacency);
        return true;
    };
    if (!select_access(start_group, start_option)) {
        return std::nullopt;
    }
    for (std::size_t connected_count = 1;
         connected_count < groups.size(); ++connected_count) {
        check_candidate_deadline(deadline);
        auto distance = std::Vector<int>(graph.nodes.size(), -1);
        auto predecessor = std::Vector<int>(graph.nodes.size(), -1);
        auto pending = std::queue<int>{};
        for (const int root : tree_tracks) {
            distance[static_cast<std::size_t>(root)] = 0;
            pending.push(root);
        }
        while (!pending.empty()) {
            const int node = pending.front();
            pending.pop();
            for (const int next : fabric.neighbors[static_cast<std::size_t>(node)]) {
                if (distance[static_cast<std::size_t>(next)] < 0) {
                    distance[static_cast<std::size_t>(next)] =
                        distance[static_cast<std::size_t>(node)] + 1;
                    predecessor[static_cast<std::size_t>(next)] = node;
                    pending.push(next);
                }
            }
        }
        auto best = std::optional<std::tuple<int, std::size_t, std::size_t>>{};
        for (std::size_t group = 0; group < groups.size(); ++group) {
            if (connected[group]) {
                continue;
            }
            for (std::size_t option = 0;
                 option < groups[group].access.size(); ++option) {
                const auto& access = groups[group].access[option];
                const int dist = distance[static_cast<std::size_t>(access.track)];
                if (dist >= 0 && access_compatible(
                        graph, access, local_resources)) {
                    const auto value = std::tuple{dist, group, option};
                    if (!best.has_value() || value < *best) {
                        best = value;
                    }
                }
            }
        }
        if (!best.has_value()) {
            return std::nullopt;
        }
        const auto [_, group, option] = *best;
        const auto& access = groups[group].access[option];
        auto fabric_path = std::Vector<int>{access.track};
        while (!tree_tracks.contains(fabric_path.back())) {
            const int previous = predecessor[static_cast<std::size_t>(fabric_path.back())];
            if (previous < 0) {
                return std::nullopt;
            }
            fabric_path.push_back(previous);
        }
        std::reverse(fabric_path.begin(), fabric_path.end());
        add_union_path(fabric_path, adjacency);
        tree_tracks.insert(fabric_path.begin(), fabric_path.end());
        if (!select_access(group, option)) {
            return std::nullopt;
        }
    }
    auto paths = std::Vector<SourceSinkPairPath>{};
    paths.reserve(spec.pairs.size());
    for (const auto& pair : spec.pairs) {
        auto node_path = build_union_path(adjacency, pair.source, pair.sink);
        if (node_path.empty()) {
            return std::nullopt;
        }
        paths.push_back(SourceSinkPairPath{
            spec.net->net_id, pair.source_index, pair.demand_id,
            pair.physical_source_node, std::move(node_path)});
    }
    return analyze_candidate(graph, fixed, spec, std::move(paths), false);
}

auto terminal_accesses_from_tree(
    const UnifiedGraph& graph, const NetSpec& spec,
    const RouteCandidate& candidate)
    -> std::optional<std::map<int, AccessOption>> {
    auto accesses = std::map<int, AccessOption>{};
    const auto record = [&](const int endpoint,
                            std::Vector<int> endpoint_to_track) -> bool {
        if (endpoint_to_track.empty()) {
            return false;
        }
        const int track = endpoint_to_track.back();
        if (graph.nodes[static_cast<std::size_t>(track)].kind
            != UnifiedNodeKind::Track) {
            return false;
        }
        const auto found = accesses.find(endpoint);
        if (found != accesses.end()) {
            return found->second.endpoint_to_track == endpoint_to_track;
        }
        accesses.emplace(
            endpoint, AccessOption{track, std::move(endpoint_to_track)});
        return true;
    };
    for (std::size_t index = 0; index < candidate.paths.size(); ++index) {
        const auto& nodes = candidate.paths[index].node_path;
        const auto& pair = spec.pairs[index];
        const auto first_track = std::find_if(
            nodes.begin(), nodes.end(), [&](const int node) {
                return graph.nodes[static_cast<std::size_t>(node)].kind
                    == UnifiedNodeKind::Track;
            });
        const auto last_track_reverse = std::find_if(
            nodes.rbegin(), nodes.rend(), [&](const int node) {
                return graph.nodes[static_cast<std::size_t>(node)].kind
                    == UnifiedNodeKind::Track;
            });
        if (first_track == nodes.end() || last_track_reverse == nodes.rend()) {
            return std::nullopt;
        }
        auto source_access = std::Vector<int>(
            nodes.begin(), std::next(first_track));
        auto sink_access = std::Vector<int>(
            nodes.rbegin(), std::next(last_track_reverse));
        if (!record(pair.source, std::move(source_access))
            || !record(pair.sink, std::move(sink_access))) {
            return std::nullopt;
        }
    }
    return accesses;
}

auto track_tree_adjacency(const UnifiedGraph& graph,
                          const RouteCandidate& candidate)
    -> std::map<int, std::set<int>> {
    auto adjacency = std::map<int, std::set<int>>{};
    for (const auto& path : candidate.paths) {
        for (std::size_t index = 1; index < path.node_path.size(); ++index) {
            const int u = path.node_path[index - 1];
            const int v = path.node_path[index];
            if (graph.nodes[static_cast<std::size_t>(u)].kind
                    == UnifiedNodeKind::Track
                && graph.nodes[static_cast<std::size_t>(v)].kind
                    == UnifiedNodeKind::Track) {
                adjacency[u].insert(v);
                adjacency[v].insert(u);
            }
        }
    }
    return adjacency;
}

auto tree_segments(const std::map<int, std::set<int>>& adjacency,
                   const std::map<int, AccessOption>& accesses)
    -> std::Vector<std::Vector<int>> {
    auto key_nodes = std::set<int>{};
    for (const auto& [_, access] : accesses) {
        key_nodes.insert(access.track);
    }
    for (const auto& [node, neighbors] : adjacency) {
        if (neighbors.size() != 2U) {
            key_nodes.insert(node);
        }
    }
    auto seen = std::set<Edge>{};
    auto segments = std::Vector<std::Vector<int>>{};
    for (const int start : key_nodes) {
        const auto found = adjacency.find(start);
        if (found == adjacency.end()) {
            continue;
        }
        for (const int next : found->second) {
            if (seen.contains(edge_key(start, next))) {
                continue;
            }
            auto segment = std::Vector<int>{start, next};
            seen.insert(edge_key(start, next));
            int previous = start;
            int current = next;
            while (!key_nodes.contains(current)) {
                const auto& neighbors = adjacency.at(current);
                if (neighbors.size() != 2U) {
                    break;
                }
                const int following = *std::find_if(
                    neighbors.begin(), neighbors.end(),
                    [&](const int node) { return node != previous; });
                seen.insert(edge_key(current, following));
                segment.push_back(following);
                previous = current;
                current = following;
            }
            segments.push_back(std::move(segment));
        }
    }
    return segments;
}

auto rebuild_tree_candidate(
    const UnifiedGraph& graph, const FixedUsage& fixed,
    const NetSpec& spec, const std::map<int, AccessOption>& accesses,
    const std::map<int, std::set<int>>& adjacency)
    -> std::optional<RouteCandidate> {
    auto paths = std::Vector<SourceSinkPairPath>{};
    paths.reserve(spec.pairs.size());
    for (const auto& pair : spec.pairs) {
        const auto source = accesses.find(pair.source);
        const auto sink = accesses.find(pair.sink);
        if (source == accesses.end() || sink == accesses.end()) {
            return std::nullopt;
        }
        const auto fabric = build_union_path(
            adjacency, source->second.track, sink->second.track);
        if (fabric.empty()) {
            return std::nullopt;
        }
        auto nodes = source->second.endpoint_to_track;
        nodes.insert(nodes.end(), std::next(fabric.begin()), fabric.end());
        if (sink->second.endpoint_to_track.size() > 1U) {
            nodes.insert(
                nodes.end(),
                std::next(sink->second.endpoint_to_track.rbegin()),
                sink->second.endpoint_to_track.rend());
        }
        paths.push_back(SourceSinkPairPath{
            spec.net->net_id, pair.source_index, pair.demand_id,
            pair.physical_source_node, std::move(nodes)});
    }
    return analyze_candidate(graph, fixed, spec, std::move(paths), false);
}

auto multi_tree_structural_pool(
    const UnifiedGraph& graph, const FixedUsage& fixed,
    const NetSpec& spec, const FabricGraph& fabric,
    RouteCandidate base, CandidateStats& stats,
    const CandidateDeadline& deadline) -> std::Vector<RouteCandidate> {
    auto out = std::Vector<RouteCandidate>{};
    auto seen = std::set<std::tuple<std::set<int>, std::set<Edge>,
                                    std::map<int, bool>>>{};
    add_candidate_deduplicated(out, seen, base);
    const auto accesses = terminal_accesses_from_tree(graph, spec, base);
    if (!accesses.has_value()) {
        return out;
    }
    const auto adjacency = track_tree_adjacency(graph, base);
    for (const auto& segment : tree_segments(adjacency, *accesses)) {
        if (out.size() >= kMaxMultiStructuralCandidates) {
            break;
        }
        check_candidate_deadline(deadline);
        if (segment.size() < 2U) {
            continue;
        }
        const auto alternatives = structural_path_pool_from_base(
            fabric, segment, deadline);
        stats.zero_excess_detours += alternatives.zero_excess_detours;
        stats.positive_excess_detours +=
            alternatives.positive_excess_detours;
        auto remaining_nodes = std::set<int>{};
        for (const auto& [node, _] : adjacency) {
            remaining_nodes.insert(node);
        }
        for (std::size_t index = 1; index + 1 < segment.size(); ++index) {
            remaining_nodes.erase(segment[index]);
        }
        const auto try_alternative = [&](const std::Vector<int>& alternative) {
            if (out.size() >= kMaxMultiStructuralCandidates
                || alternative == segment
                || std::ranges::any_of(
                    std::next(alternative.begin()),
                    std::prev(alternative.end()),
                    [&](const int node) { return remaining_nodes.contains(node); })) {
                return;
            }
            auto replaced = adjacency;
            for (std::size_t index = 1; index < segment.size(); ++index) {
                replaced[segment[index - 1]].erase(segment[index]);
                replaced[segment[index]].erase(segment[index - 1]);
            }
            for (std::size_t index = 1; index < alternative.size(); ++index) {
                replaced[alternative[index - 1]].insert(alternative[index]);
                replaced[alternative[index]].insert(alternative[index - 1]);
            }
            for (auto it = replaced.begin(); it != replaced.end();) {
                if (it->second.empty()) {
                    it = replaced.erase(it);
                } else {
                    ++it;
                }
            }
            if (auto candidate = rebuild_tree_candidate(
                    graph, fixed, spec, *accesses, replaced)) {
                const std::size_t before = out.size();
                add_candidate_deduplicated(
                    out, seen, std::move(*candidate));
                if (out.size() != before) {
                    ++stats.multi_segment_variants;
                }
            }
        };
        for (const auto& alternative : alternatives.shortest) {
            try_alternative(alternative);
        }
        for (const auto& alternative : alternatives.longer) {
            try_alternative(alternative);
        }
    }
    return out;
}

auto split_tree_length_classes(const std::Vector<RouteCandidate>& pool)
    -> std::pair<std::Vector<RouteCandidate>, std::Vector<RouteCandidate>> {
    auto shortest = std::Vector<RouteCandidate>{};
    auto longer = std::Vector<RouteCandidate>{};
    if (pool.empty()) {
        return {shortest, longer};
    }
    std::size_t shortest_length = std::numeric_limits<std::size_t>::max();
    std::size_t longer_length = std::numeric_limits<std::size_t>::max();
    for (const auto& candidate : pool) {
        const std::size_t length = candidate.resources.wire_nodes.size();
        if (length < shortest_length) {
            longer_length = shortest_length;
            shortest_length = length;
        } else if (length > shortest_length && length < longer_length) {
            longer_length = length;
        }
    }
    for (const auto& candidate : pool) {
        const std::size_t length = candidate.resources.wire_nodes.size();
        if (length == shortest_length) {
            shortest.push_back(candidate);
        } else if (length == longer_length) {
            longer.push_back(candidate);
        }
    }
    return {std::move(shortest), std::move(longer)};
}

auto generate_multi_candidates(const UnifiedGraph& graph,
                               const FixedUsage& fixed,
                               const NetSpec& spec,
                               const FabricGraph& fabric,
                               const CongestionMap& c1_without_net,
                               const CongestionMap& c2,
                               CandidateStats& stats,
                               const CandidateDeadline& deadline)
    -> GeneratedCandidates {
    auto endpoint_set = std::set<int>{};
    auto endpoints = std::Vector<int>{};
    for (const auto& pair : spec.pairs) {
        for (const int endpoint : {pair.source, pair.sink}) {
            if (endpoint_set.insert(endpoint).second) {
                endpoints.push_back(endpoint);
            }
        }
    }
    auto groups = std::Vector<TerminalGroup>{};
    for (const int endpoint : endpoints) {
        auto access = enumerate_access_options(graph, fixed, endpoint, spec.unit);
        debug::info_fmt(
            "V22 selectable_tracks: net=\"{}\" id={} terminal={} count={}",
            spec.net->name, spec.net->net_id,
            format_unified_node(graph, endpoint), access.size());
        if (access.empty()) {
            groups.clear();
            break;
        }
        groups.push_back({endpoint, std::move(access)});
    }
    auto generated = GeneratedCandidates{};
    auto seen = std::set<std::tuple<std::set<int>, std::set<Edge>,
                                    std::map<int, bool>>>{};
    std::size_t seed_index = 0;
    const auto consume_seed = [&](RouteCandidate base) {
        const auto structural = multi_tree_structural_pool(
            graph, fixed, spec, fabric, std::move(base), stats, deadline);
        const auto [shortest_pool, longer_pool] =
            split_tree_length_classes(structural);
        const auto salt_base =
            (static_cast<std::uint64_t>(spec.net->net_id) << 32U)
            ^ static_cast<std::uint64_t>(seed_index++ * 4U);
        select_from_length_class(
            graph, shortest_pool, c2, salt_base,
            generated.candidates, seen, stats, false, nullptr);
        select_from_length_class(
            graph, longer_pool, c2, salt_base + 1U,
            generated.candidates, seen, stats, false, nullptr);
        select_from_length_class(
            graph, shortest_pool, c1_without_net, salt_base + 2U,
            generated.candidates, seen, stats, true,
            &generated.c1_representative_pool);
        select_from_length_class(
            graph, longer_pool, c1_without_net, salt_base + 3U,
            generated.candidates, seen, stats, true,
            &generated.c1_representative_pool);
    };
    if (!groups.empty()) {
        ++stats.multi_base_runs;
        if (auto base = generate_multi_tree(
                graph, fixed, spec, fabric, groups, -1, 0, deadline)) {
            consume_seed(std::move(*base));
        } else {
            ++stats.multi_failed_runs;
        }
        for (std::size_t group = 0; group < groups.size(); ++group) {
            for (std::size_t option = 0; option < groups[group].access.size();
                 ++option) {
                check_candidate_deadline(deadline);
                ++stats.multi_forced_runs;
                if (auto candidate = generate_multi_tree(
                        graph, fixed, spec, fabric, groups,
                        static_cast<int>(group), option, deadline)) {
                    consume_seed(std::move(*candidate));
                } else {
                    ++stats.multi_failed_runs;
                }
            }
        }
    }
    finalize_c1_representatives(graph, spec, c1_without_net, generated);
    if (auto incumbent = analyze_candidate(
            graph, fixed, spec, spec.incumbent_paths, true)) {
        ++stats.incumbent_candidates;
        add_candidate_deduplicated(
            generated.candidates, seen, std::move(*incumbent));
    }
    return generated;
}

struct CandidateProblem {
    std::Vector<NetSpec> specs;
    std::Vector<std::Vector<RouteCandidate>> candidates;
    CandidateStats stats;
};

auto build_candidate_problem(const UnifiedGraph& graph,
                             const std::Vector<RoutingNet>& nets,
                             const SatRoutingResult& sat,
                             const FixedUsage& fixed,
                             const CandidateDeadline& deadline)
    -> std::optional<CandidateProblem> {
    const auto specs = build_net_specs(graph, nets, sat);
    if (!specs.has_value()) {
        return std::nullopt;
    }
    auto out = CandidateProblem{};
    out.specs = *specs;
    out.candidates.resize(out.specs.size());
    const auto c2 = track_channel_load(graph, fixed.nodes);
    auto c1 = c2;
    auto contributions = std::Vector<CongestionMap>(out.specs.size());
    for (std::size_t index = 0; index < out.specs.size(); ++index) {
        const auto incumbent = analyze_candidate(
            graph, fixed, out.specs[index],
            out.specs[index].incumbent_paths, true);
        if (!incumbent.has_value()) {
            return std::nullopt;
        }
        contributions[index] = candidate_channel_demand(graph, *incumbent);
        add_scaled_load(c1, contributions[index], 1.0);
    }
    debug::info_fmt(
        "V22 congestion maps initialized: C1_channel_units={} C1_peak={:.3f} C2_sync_channel_units={} C2_peak={:.3f} sequential_update=yes short_weight=0.9 long_weight=0.1",
        c1.size(), peak_load(c1), c2.size(), peak_load(c2));
    auto fabric_by_unit = std::array<std::optional<FabricGraph>, 16>{};
    for (std::size_t index = 0; index < out.specs.size(); ++index) {
        check_candidate_deadline(deadline);
        const auto& spec = out.specs[index];
        if (spec.unit >= fabric_by_unit.size()) {
            return std::nullopt;
        }
        auto& fabric_slot = fabric_by_unit[spec.unit];
        if (!fabric_slot.has_value()) {
            fabric_slot = build_fabric_graph(graph, fixed, spec.unit);
        }
        const auto& fabric = *fabric_slot;
        auto c1_without_net = c1;
        add_scaled_load(c1_without_net, contributions[index], -1.0);
        auto generated = spec.pairs.size() == 1
            ? generate_two_pin_candidates(
                graph, fixed, spec, fabric, c1_without_net, c2,
                out.stats, deadline)
            : generate_multi_candidates(
                graph, fixed, spec, fabric, c1_without_net, c2,
                out.stats, deadline);
        auto replacement = CongestionMap{};
        if (generated.c1_short_representative.has_value()) {
            const auto short_demand = candidate_channel_demand(
                graph, *generated.c1_short_representative);
            if (generated.c1_long_representative.has_value()
                && candidate_signature(*generated.c1_short_representative)
                    != candidate_signature(*generated.c1_long_representative)) {
                add_scaled_load(replacement, short_demand, 0.9);
                add_scaled_load(
                    replacement,
                    candidate_channel_demand(
                        graph, *generated.c1_long_representative),
                    0.1);
            } else {
                add_scaled_load(replacement, short_demand, 1.0);
            }
        } else {
            replacement = contributions[index];
        }
        const double peak_before = peak_load(c1);
        c1 = std::move(c1_without_net);
        add_scaled_load(c1, replacement, 1.0);
        contributions[index] = std::move(replacement);
        out.candidates[index] = std::move(generated.candidates);
        if (out.candidates[index].empty()
            || std::ranges::none_of(out.candidates[index],
                                    [](const auto& candidate) {
                                        return candidate.incumbent;
                                    })) {
            return std::nullopt;
        }
        debug::info_fmt(
            "V22 candidate net: name=\"{}\" id={} order={} pairs={} unit={} candidates={} C1_peak={:.3f}->{:.3f} representative_short={} representative_long={}",
            spec.net->name, spec.net->net_id, index, spec.pairs.size(),
            spec.unit, out.candidates[index].size(), peak_before, peak_load(c1),
            generated.c1_short_representative.has_value()
                ? static_cast<long long>(generated.c1_short_representative
                      ->resources.wire_nodes.size())
                : -1LL,
            generated.c1_long_representative.has_value()
                ? static_cast<long long>(generated.c1_long_representative
                      ->resources.wire_nodes.size())
                : -1LL);
    }
    return out;
}

auto rebuild_metadata(const UnifiedGraph& graph, SatRoutingResult& result) -> bool {
    auto resources = RouteResources{};
    for (const auto& path : result.paths) {
        if (!add_path_resources(graph, path.node_path, resources)) {
            return false;
        }
    }
    result.used_tob_switch_ids.assign(resources.switches.begin(),
                                      resources.switches.end());
    result.vline_mode_straight_by_group.clear();
    for (const auto& [group, straight] : resources.mode_by_group) {
        result.vline_mode_straight_by_group[
            static_cast<std::size_t>(group)] = straight;
    }
    result.total_wirelength = total_wirelength(graph, result);
    return true;
}

auto validate_complete_result(const UnifiedGraph& graph,
                              const std::Vector<RoutingNet>& nets,
                              const SatRoutingResult& baseline,
                              const SatRoutingResult& candidate) -> bool {
    auto net_by_id = std::map<std::size_t, const RoutingNet*>{};
    for (const auto& net : nets) {
        net_by_id[net.net_id] = &net;
    }
    auto baseline_by_key = std::map<PathKey, const SourceSinkPairPath*>{};
    for (const auto& path : baseline.paths) {
        baseline_by_key.emplace(
            PathKey{path.net_id, path.demand_id, path.source_index}, &path);
    }
    auto seen = std::set<PathKey>{};
    auto node_owners = std::map<int, std::set<std::size_t>>{};
    auto switch_owners = std::map<int, std::set<std::size_t>>{};
    auto peer_by_port = std::map<PortKey, int>{};
    auto mode_by_group = std::map<int, bool>{};
    for (const auto& path : candidate.paths) {
        const auto key = PathKey{path.net_id, path.demand_id, path.source_index};
        const auto expected = baseline_by_key.find(key);
        if (expected == baseline_by_key.end() || !seen.insert(key).second
            || path.node_path.empty()
            || path.node_path.front() != expected->second->node_path.front()
            || path.node_path.back() != expected->second->node_path.back()
            || path.physical_source_node
                != expected->second->physical_source_node) {
            return false;
        }
        const auto net = net_by_id.find(path.net_id);
        if (net == net_by_id.end()) {
            return false;
        }
        if (net->second->is_sync_bus
            && path.node_path != expected->second->node_path) {
            return false;
        }
        const auto expected_unit = infer_path_unit(
            graph, expected->second->node_path);
        const auto actual_unit = infer_path_unit(graph, path.node_path);
        if (!expected_unit.has_value() || actual_unit != expected_unit) {
            return false;
        }
        auto local_nodes = std::set<int>{};
        for (const int node : path.node_path) {
            if (!is_physical_node(graph, node)
                || !local_nodes.insert(node).second) {
                return false;
            }
            node_owners[node].insert(path.net_id);
        }
        for (std::size_t index = 1; index < path.node_path.size(); ++index) {
            const int arc_id = find_arc(
                graph, path.node_path[index - 1], path.node_path[index]);
            if (arc_id < 0) {
                return false;
            }
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (arc.physical_switch_id >= 0) {
                switch_owners[arc.physical_switch_id].insert(path.net_id);
                const int stage = switch_stage(arc.physical_switch_kind);
                for (const auto [node, peer] :
                     {std::pair{arc.u, arc.v}, std::pair{arc.v, arc.u}}) {
                    const auto [it, inserted] =
                        peer_by_port.emplace(PortKey{node, stage}, peer);
                    if (!inserted && it->second != peer) {
                        return false;
                    }
                }
            }
            if (arc.mode_group_id >= 0) {
                const bool straight = arc.is_vline_track_straight;
                const auto [it, inserted] =
                    mode_by_group.emplace(arc.mode_group_id, straight);
                if (!inserted && it->second != straight) {
                    return false;
                }
            }
        }
    }
    if (seen.size() != baseline_by_key.size()
        || std::ranges::any_of(node_owners, [](const auto& item) {
               return item.second.size() > 1;
           })
        || std::ranges::any_of(switch_owners, [](const auto& item) {
               return item.second.size() > 1;
           })
        || total_wirelength(graph, candidate) != candidate.total_wirelength) {
        return false;
    }
    return true;
}

#ifdef USE_HIGHS

class CandidateMip {
public:
    CandidateMip(const int verbose_level, const std::string_view log_path,
                 const double time_limit_seconds)
        : log_sink_(log_path, verbose_level >= 2, true) {
        log_sink_.attach(highs_);
        check(highs_.setOptionValue("mip_rel_gap", 0.015), "mip_rel_gap");
        if (time_limit_seconds > 0.0) {
            check(highs_.setOptionValue("time_limit", time_limit_seconds),
                  "time_limit");
        }
    }

    auto add_binary(const double cost) -> int {
        const int column = static_cast<int>(variables_++);
        check(highs_.addCol(cost, 0.0, 1.0, 0, nullptr, nullptr), "addCol");
        check(highs_.changeColIntegrality(column, HighsVarType::kInteger),
              "integrality");
        return column;
    }

    auto add_row(const double lower, const double upper,
                 const std::Vector<std::pair<int, double>>& terms) -> void {
        auto indices = std::Vector<HighsInt>{};
        auto values = std::Vector<double>{};
        for (const auto& [column, value] : terms) {
            if (value != 0.0) {
                indices.push_back(static_cast<HighsInt>(column));
                values.push_back(value);
            }
        }
        check(highs_.addRow(lower, upper,
                            static_cast<HighsInt>(indices.size()),
                            indices.empty() ? nullptr : indices.data(),
                            values.empty() ? nullptr : values.data()),
              "addRow");
        ++constraints_;
    }

    auto set_start(const std::map<int, double>& entries) -> void {
        auto indices = std::Vector<HighsInt>{};
        auto values = std::Vector<double>{};
        for (const auto& [column, value] : entries) {
            indices.push_back(static_cast<HighsInt>(column));
            values.push_back(value);
        }
        check(highs_.setSolution(static_cast<HighsInt>(indices.size()),
                                 indices.data(), values.data()), "setSolution");
    }

    auto solve() -> bool {
        check(highs_.run(), "run");
        const auto status = highs_.getModelStatus();
        return status == HighsModelStatus::kOptimal
            || (status == HighsModelStatus::kTimeLimit
                && highs_.getInfo().primal_solution_status
                    == kSolutionStatusFeasible);
    }

    auto values() const -> const std::Vector<double>& {
        return highs_.getSolution().col_value;
    }
    auto objective() const -> double {
        return highs_.getInfo().objective_function_value;
    }
    auto bound() const -> double { return highs_.getInfo().mip_dual_bound; }
    auto gap() const -> double { return highs_.getInfo().mip_gap; }
    auto variables() const -> std::size_t { return variables_; }
    auto constraints() const -> std::size_t { return constraints_; }

private:
    static auto check(const HighsStatus status, const char* operation) -> void {
        if (status == HighsStatus::kError) {
            throw std::runtime_error(
                std::format("HiGHS {} failed", operation));
        }
    }
    HighsLogSink log_sink_;
    Highs highs_;
    std::size_t variables_{0};
    std::size_t constraints_{0};
};

struct MasterResult {
    bool ok{false};
    std::Vector<std::size_t> selected;
    std::size_t variables{0};
    std::size_t constraints{0};
    double objective{0.0};
    double bound{0.0};
    double gap{0.0};
    long long build_ms{0};
    long long solve_ms{0};
};

auto solve_master(const CandidateProblem& problem,
                  const FixedUsage& fixed,
                  const PostSatIlpOptions& options,
                  const double time_limit_seconds) -> MasterResult {
    const auto build_begin = std::chrono::steady_clock::now();
    auto mip = CandidateMip(options.verbose_level, options.highs_log_path,
                            time_limit_seconds);
    auto x = std::Vector<std::Vector<int>>(problem.candidates.size());
    auto node_terms = std::map<int, std::Vector<std::pair<int, double>>>{};
    auto node_nets = std::map<int, std::set<std::size_t>>{};
    auto mode_groups = std::set<int>{};
    for (const auto& net_candidates : problem.candidates) {
        for (const auto& candidate : net_candidates) {
            for (const auto& [group, _] : candidate.resources.mode_by_group) {
                if (!fixed.mode_by_group.contains(group)) {
                    mode_groups.insert(group);
                }
            }
        }
    }
    auto mode_var = std::map<int, int>{};
    for (const int group : mode_groups) {
        mode_var[group] = mip.add_binary(0.0);
    }
    auto start = std::map<int, double>{};
    std::size_t mode_rows = 0;
    for (std::size_t net = 0; net < problem.candidates.size(); ++net) {
        auto exactly_one = std::Vector<std::pair<int, double>>{};
        bool has_start = false;
        for (std::size_t candidate_index = 0;
             candidate_index < problem.candidates[net].size(); ++candidate_index) {
            const auto& candidate = problem.candidates[net][candidate_index];
            const int variable = mip.add_binary(
                static_cast<double>(candidate.resources.wire_nodes.size()));
            x[net].push_back(variable);
            exactly_one.emplace_back(variable, 1.0);
            start[variable] = candidate.incumbent ? 1.0 : 0.0;
            has_start = has_start || candidate.incumbent;
            for (const int node : candidate.resources.nodes) {
                node_terms[node].emplace_back(variable, 1.0);
                node_nets[node].insert(net);
            }
            for (const auto& [group, straight] :
                 candidate.resources.mode_by_group) {
                if (fixed.mode_by_group.contains(group)) {
                    continue;
                }
                if (straight) {
                    mip.add_row(-kHighsInf, 0.0,
                                {{variable, 1.0}, {mode_var.at(group), -1.0}});
                } else {
                    mip.add_row(-kHighsInf, 1.0,
                                {{variable, 1.0}, {mode_var.at(group), 1.0}});
                }
                ++mode_rows;
            }
        }
        if (!has_start) {
            return {};
        }
        mip.add_row(1.0, 1.0, exactly_one);
    }
    std::size_t capacity_rows = 0;
    for (const auto& [node, terms] : node_terms) {
        if (node_nets[node].size() > 1) {
            mip.add_row(-kHighsInf, 1.0, terms);
            ++capacity_rows;
        }
    }
    for (const auto& [group, variable] : mode_var) {
        bool value = false;
        if (const auto it = fixed.mode_by_group.find(group);
            it != fixed.mode_by_group.end()) {
            value = it->second;
        } else {
            for (std::size_t net = 0; net < problem.candidates.size(); ++net) {
                const auto incumbent = std::find_if(
                    problem.candidates[net].begin(),
                    problem.candidates[net].end(),
                    [](const auto& candidate) { return candidate.incumbent; });
                if (incumbent == problem.candidates[net].end()) {
                    continue;
                }
                const auto mode = incumbent->resources.mode_by_group.find(group);
                if (mode != incumbent->resources.mode_by_group.end()) {
                    value = mode->second;
                    break;
                }
            }
        }
        start[variable] = value ? 1.0 : 0.0;
    }
    mip.set_start(start);
    const auto build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - build_begin).count();
    debug::info_fmt(
        "V22 candidate ILP model: nets={} candidates={} mode_vars={} vars={} constraints={} net_choice_rows={} node_capacity_rows={} mode_binding_rows={} build_ms={}",
        problem.specs.size(),
        std::accumulate(problem.candidates.begin(), problem.candidates.end(),
            std::size_t{0}, [](const auto sum, const auto& values) {
                return sum + values.size();
            }),
        mode_var.size(), mip.variables(), mip.constraints(), problem.specs.size(),
        capacity_rows, mode_rows, build_ms);
    const auto solve_begin = std::chrono::steady_clock::now();
    const bool ok = mip.solve();
    const auto solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - solve_begin).count();
    auto out = MasterResult{};
    out.ok = ok;
    out.variables = mip.variables();
    out.constraints = mip.constraints();
    out.objective = mip.objective();
    out.bound = mip.bound();
    out.gap = mip.gap();
    out.build_ms = build_ms;
    out.solve_ms = solve_ms;
    if (!ok) {
        return out;
    }
    const auto& values = mip.values();
    for (std::size_t net = 0; net < x.size(); ++net) {
        const auto selected = std::find_if(
            x[net].begin(), x[net].end(), [&](const int variable) {
                return static_cast<std::size_t>(variable) < values.size()
                    && values[static_cast<std::size_t>(variable)] > 0.5;
            });
        if (selected == x[net].end()) {
            out.ok = false;
            return out;
        }
        out.selected.push_back(
            static_cast<std::size_t>(std::distance(x[net].begin(), selected)));
    }
    return out;
}

#endif

} // namespace

auto optimize_post_sat_candidate_routes(
    const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
    const SatRoutingResult& sat_result,
    const PostSatIlpOptions& options) -> SatRoutingResult {
    const auto begin = std::chrono::steady_clock::now();
    const auto deadline = options.highs_time_limit_minutes > 0
        ? CandidateDeadline{begin + std::chrono::minutes(
              options.highs_time_limit_minutes)}
        : CandidateDeadline{};
    auto fallback = sat_result;
    fallback.post_sat_ilp_attempted = true;
    fallback.post_sat_ilp_baseline_wirelength = sat_result.total_wirelength;
    fallback.post_sat_ilp_wirelength = sat_result.total_wirelength;
    const auto finish = [&](SatRoutingResult result, const std::String& status) {
        result.post_sat_ilp_attempted = true;
        result.post_sat_ilp_status = status;
        result.post_sat_ilp_wirelength = result.total_wirelength;
        result.post_sat_ilp_total_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - begin).count();
        debug::info_fmt(
            "V22 candidate ILP summary: status={} accepted={} nets={} candidates={} vars={} constraints={} objective={} bound={} gap={} wirelength={}->{} total_ms={} build_ms={} solve_ms={}",
            status, result.post_sat_ilp_accepted, result.post_sat_ilp_parents,
            result.post_sat_ilp_segments, result.post_sat_ilp_variables,
            result.post_sat_ilp_constraints, result.post_sat_ilp_objective,
            result.post_sat_ilp_bound, result.post_sat_ilp_gap,
            result.post_sat_ilp_baseline_wirelength,
            result.post_sat_ilp_wirelength, result.post_sat_ilp_total_ms,
            result.post_sat_ilp_build_ms, result.post_sat_ilp_solve_ms);
        return result;
    };
    if (!sat_result.ok) {
        return finish(std::move(fallback), "SKIPPED_NO_SAT_SOLUTION");
    }
    try {
        const auto fixed = collect_fixed_sync(graph, nets, sat_result);
        if (!fixed.has_value()) {
            return finish(std::move(fallback), "SYNC_RESOURCE_INVALID");
        }
        debug::info_fmt(
            "V22 fixed Sync resources: nodes={} switches={} matching_ports={} modes={}",
            fixed->nodes.size(), fixed->switches.size(),
            fixed->peer_by_port.size(), fixed->mode_by_group.size());
        const auto generation_begin = std::chrono::steady_clock::now();
        const auto problem = build_candidate_problem(
            graph, nets, sat_result, *fixed, deadline);
        if (!problem.has_value()) {
            return finish(std::move(fallback), "CANDIDATE_GENERATION_FAILED");
        }
        if (problem->specs.empty()) {
            return finish(std::move(fallback), "SKIPPED_NO_NON_SYNC_NETS");
        }
        const auto generation_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - generation_begin).count();
        const std::size_t candidate_count = std::accumulate(
            problem->candidates.begin(), problem->candidates.end(),
            std::size_t{0}, [](const auto sum, const auto& values) {
                return sum + values.size();
            });
        const std::size_t two_pin_nets = static_cast<std::size_t>(
            std::ranges::count_if(problem->specs, [](const auto& spec) {
                return spec.pairs.size() == 1;
            }));
        const std::size_t pn_source_trees = static_cast<std::size_t>(
            std::ranges::count_if(problem->specs, [](const auto& spec) {
                return spec.net->pn_source_tree;
            }));
        fallback.post_sat_ilp_parents = problem->specs.size();
        fallback.post_sat_ilp_segments = candidate_count;
        fallback.post_sat_ilp_components = problem->specs.empty() ? 0 : 1;
        debug::info_fmt(
            "V22 candidate generation: nets={} two_pin_nets={} multi_terminal_nets={} pn_source_trees={} candidates={} two_pin_combinations={} shortest={} plus_one={} second_shortest={} zero_excess_detours={} positive_excess_detours={} C1_selected={} C2_selected={} multi_base={} multi_forced={} multi_segment_variants={} multi_failed={} incumbents={} generation_ms={}",
            problem->specs.size(), two_pin_nets,
            problem->specs.size() - two_pin_nets, pn_source_trees,
            candidate_count,
            problem->stats.two_pin_combinations,
            problem->stats.two_pin_shortest,
            problem->stats.two_pin_plus_one,
            problem->stats.two_pin_second_shortest,
            problem->stats.zero_excess_detours,
            problem->stats.positive_excess_detours,
            problem->stats.c1_selected,
            problem->stats.c2_selected,
            problem->stats.multi_base_runs,
            problem->stats.multi_forced_runs,
            problem->stats.multi_segment_variants,
            problem->stats.multi_failed_runs,
            problem->stats.incumbent_candidates, generation_ms);
#ifndef USE_HIGHS
        return finish(std::move(fallback), "HIGHS_NOT_AVAILABLE");
#else
        check_candidate_deadline(deadline);
        const double master_time_limit_seconds = deadline.has_value()
            ? std::max(0.001,
                  std::chrono::duration<double>(
                      *deadline - std::chrono::steady_clock::now()).count())
            : 0.0;
        const auto master = solve_master(
            *problem, *fixed, options, master_time_limit_seconds);
        fallback.post_sat_ilp_variables = master.variables;
        fallback.post_sat_ilp_constraints = master.constraints;
        fallback.post_sat_ilp_objective = master.objective;
        fallback.post_sat_ilp_bound = master.bound;
        fallback.post_sat_ilp_gap = master.gap;
        fallback.post_sat_ilp_build_ms = generation_ms + master.build_ms;
        fallback.post_sat_ilp_solve_ms = master.solve_ms;
        if (!master.ok) {
            return finish(std::move(fallback), "ILP_FAILED");
        }
        auto selected = sat_result;
        selected.paths.erase(
            std::remove_if(selected.paths.begin(), selected.paths.end(),
                [&](const auto& path) {
                    const auto it = std::find_if(
                        nets.begin(), nets.end(), [&](const auto& net) {
                            return net.net_id == path.net_id;
                        });
                    return it != nets.end() && !it->is_sync_bus;
                }),
            selected.paths.end());
        for (std::size_t net = 0; net < master.selected.size(); ++net) {
            const auto& candidate =
                problem->candidates[net][master.selected[net]];
            selected.paths.insert(selected.paths.end(), candidate.paths.begin(),
                                  candidate.paths.end());
        }
        if (!rebuild_metadata(graph, selected)
            || !validate_complete_result(
                graph, nets, sat_result, selected)) {
            return finish(std::move(fallback), "PHYSICAL_VALIDATION_FAILED");
        }
        if (selected.total_wirelength >= sat_result.total_wirelength) {
            return finish(std::move(fallback), "NO_IMPROVEMENT");
        }
        selected.post_sat_ilp_attempted = true;
        selected.post_sat_ilp_accepted = true;
        selected.post_sat_ilp_baseline_wirelength = sat_result.total_wirelength;
        selected.post_sat_ilp_parents = problem->specs.size();
        selected.post_sat_ilp_segments = candidate_count;
        selected.post_sat_ilp_components = problem->specs.empty() ? 0 : 1;
        selected.post_sat_ilp_variables = master.variables;
        selected.post_sat_ilp_constraints = master.constraints;
        selected.post_sat_ilp_objective = master.objective;
        selected.post_sat_ilp_bound = master.bound;
        selected.post_sat_ilp_gap = master.gap;
        selected.post_sat_ilp_build_ms = generation_ms + master.build_ms;
        selected.post_sat_ilp_solve_ms = master.solve_ms;
        return finish(std::move(selected), "IMPROVED");
#endif
    } catch (const std::exception& error) {
        debug::warning_fmt("V22 candidate ILP exception: {}", error.what());
        return finish(std::move(fallback),
                      std::format("EXCEPTION: {}", error.what()));
    }
}

} // namespace PR_tool
