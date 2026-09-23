#include "rrr/rrr.hh"

#include "common/cob_unit_mask.hh"
#include "common/route_metrics.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <debug/debug.hh>
#include <format>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>

namespace PR_tool {

namespace {

using Owner = std::size_t;

enum class ConflictKind { Node, Switch, Port, Mode };

struct ConflictKey {
  ConflictKind kind{ConflictKind::Node};
  int id{-1};
  int stage{-1};

  auto operator<=>(const ConflictKey &) const = default;
};

struct PortKey {
  int node{-1};
  int stage{-1};

  auto operator<=>(const PortKey &) const = default;
};

struct RouteSpec {
  std::size_t net_id{0};
  std::size_t demand_id{0};
  std::size_t source_index{0};
  int physical_source_node{-1};
  int source{-1};
  int sink{-1};
  std::uint16_t unit_mask{0xffff};
};

struct ConflictReport {
  int overflow{0};
  std::set<Owner> owners;
  std::map<Owner, int> exposure;
  std::set<ConflictKey> keys;
};

auto find_arc(const UnifiedGraph &graph, const int u, const int v) -> int {
  if (u < 0 || static_cast<std::size_t>(u) >= graph.out_arc_ids.size()) {
    return -1;
  }
  for (const int arc : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
    if (graph.arcs[static_cast<std::size_t>(arc)].v == v) {
      return arc;
    }
  }
  return -1;
}

auto is_physical_node(const UnifiedGraph &graph, const int node) -> bool {
  return node >= 0 && static_cast<std::size_t>(node) < graph.nodes.size() &&
         graph.nodes[static_cast<std::size_t>(node)].kind !=
             UnifiedNodeKind::VirtualSource;
}

auto switch_stage(const PhysicalSwitchKind kind) -> int {
  switch (kind) {
  case PhysicalSwitchKind::BumpH:
    return 0;
  case PhysicalSwitchKind::HLineVLine:
    return 1;
  case PhysicalSwitchKind::VLineTrack:
    return 2;
  case PhysicalSwitchKind::None:
    return -1;
  }
  return -1;
}

class Usage {
public:
  Usage(const UnifiedGraph &graph, const std::Vector<RoutingNet> &nets,
        const RoutingResult &result)
      : graph_(graph) {
    for (const auto &net : nets) {
      if (net.is_sync_bus) {
        sync_.insert(net.net_id);
      }
    }
    for (const auto &path : result.paths) {
      claim_path(path.net_id, path.node_path);
    }
  }

  auto claim_path(const Owner owner, const std::Vector<int> &path) -> void {
    for (const int node : path) {
      if (is_physical_node(graph_, node)) {
        nodes_[node].insert(owner);
      }
    }
    for (std::size_t index = 1; index < path.size(); ++index) {
      const int arc_id = find_arc(graph_, path[index - 1], path[index]);
      if (arc_id < 0) {
        continue;
      }
      const auto &arc = graph_.arcs[static_cast<std::size_t>(arc_id)];
      if (arc.physical_switch_id >= 0) {
        switches_[arc.physical_switch_id].insert(owner);
        const int stage = switch_stage(arc.physical_switch_kind);
        ports_[PortKey{arc.u, stage}][arc.v].insert(owner);
        ports_[PortKey{arc.v, stage}][arc.u].insert(owner);
      }
      if (arc.mode_group_id >= 0) {
        auto &uses = arc.is_vline_track_straight ? straight_ : swap_;
        uses[arc.mode_group_id].insert(owner);
      }
    }
  }

  auto is_sync(const Owner owner) const -> bool {
    return sync_.contains(owner);
  }

  auto owner_uses_node(const Owner owner, const int node) const -> bool {
    const auto it = nodes_.find(node);
    return it != nodes_.end() && it->second.contains(owner);
  }

  auto hard_blocks_node(const Owner owner, const int node) const -> bool {
    const auto it = nodes_.find(node);
    if (it == nodes_.end()) {
      return false;
    }
    return std::ranges::any_of(it->second, [&](const Owner other) {
      return other != owner && is_sync(other);
    });
  }

  auto node_conflicts(const Owner owner, const int node) const -> int {
    const auto it = nodes_.find(node);
    if (it == nodes_.end()) {
      return 0;
    }
    return static_cast<int>(std::ranges::count_if(
        it->second, [&](const Owner other) { return other != owner; }));
  }

  auto hard_blocks_arc(const Owner owner, const UnifiedArc &arc) const -> bool {
    if (arc.physical_switch_id >= 0) {
      const auto switch_it = switches_.find(arc.physical_switch_id);
      if (switch_it != switches_.end() &&
          std::ranges::any_of(switch_it->second, [&](const Owner other) {
            return other != owner && is_sync(other);
          })) {
        return true;
      }
      const int stage = switch_stage(arc.physical_switch_kind);
      for (const auto [node, peer] :
           {std::pair{arc.u, arc.v}, std::pair{arc.v, arc.u}}) {
        const auto port_it = ports_.find(PortKey{node, stage});
        if (port_it == ports_.end()) {
          continue;
        }
        for (const auto &[used_peer, owners] : port_it->second) {
          if (used_peer == peer) {
            continue;
          }
          if (std::ranges::any_of(owners, [&](const Owner other) {
                return other != owner && is_sync(other);
              })) {
            return true;
          }
        }
      }
    }
    if (arc.mode_group_id >= 0) {
      const auto &opposite = arc.is_vline_track_straight ? swap_ : straight_;
      const auto it = opposite.find(arc.mode_group_id);
      if (it != opposite.end() &&
          std::ranges::any_of(it->second, [&](const Owner other) {
            return other != owner && is_sync(other);
          })) {
        return true;
      }
    }
    return false;
  }

  auto arc_conflicts(const Owner owner, const UnifiedArc &arc) const -> int {
    int count = 0;
    if (arc.physical_switch_id >= 0) {
      const auto switch_it = switches_.find(arc.physical_switch_id);
      if (switch_it != switches_.end()) {
        count += static_cast<int>(
            std::ranges::count_if(switch_it->second, [&](const Owner other) {
              return other != owner;
            }));
      }
      const int stage = switch_stage(arc.physical_switch_kind);
      for (const auto [node, peer] :
           {std::pair{arc.u, arc.v}, std::pair{arc.v, arc.u}}) {
        const auto port_it = ports_.find(PortKey{node, stage});
        if (port_it == ports_.end()) {
          continue;
        }
        for (const auto &[used_peer, owners] : port_it->second) {
          if (used_peer != peer) {
            count += static_cast<int>(owners.size());
          }
        }
      }
    }
    if (arc.mode_group_id >= 0) {
      const auto &opposite = arc.is_vline_track_straight ? swap_ : straight_;
      const auto it = opposite.find(arc.mode_group_id);
      if (it != opposite.end()) {
        count += static_cast<int>(it->second.size());
      }
    }
    return count;
  }

  auto report() const -> ConflictReport {
    auto out = ConflictReport{};
    const auto add_owner_set = [&](const std::set<Owner> &owners,
                                   const ConflictKey key, const int amount) {
      out.overflow += amount;
      out.keys.insert(key);
      for (const Owner owner : owners) {
        if (!is_sync(owner)) {
          out.owners.insert(owner);
          out.exposure[owner] += amount;
        }
      }
    };
    for (const auto &[node, owners] : nodes_) {
      if (owners.size() > 1) {
        add_owner_set(owners, {ConflictKind::Node, node, -1},
                      static_cast<int>(owners.size() - 1));
      }
    }
    for (const auto &[physical_switch, owners] : switches_) {
      if (owners.size() > 1) {
        add_owner_set(owners, {ConflictKind::Switch, physical_switch, -1},
                      static_cast<int>(owners.size() - 1));
      }
    }
    for (const auto &[port, peers] : ports_) {
      if (peers.size() <= 1) {
        continue;
      }
      auto owners = std::set<Owner>{};
      for (const auto &[_, peer_owners] : peers) {
        owners.insert(peer_owners.begin(), peer_owners.end());
      }
      add_owner_set(owners, {ConflictKind::Port, port.node, port.stage},
                    static_cast<int>(peers.size() - 1));
    }
    auto mode_groups = std::set<int>{};
    for (const auto &[group, _] : straight_)
      mode_groups.insert(group);
    for (const auto &[group, _] : swap_)
      mode_groups.insert(group);
    for (const int group : mode_groups) {
      if (!straight_.contains(group) || !swap_.contains(group)) {
        continue;
      }
      auto owners = straight_.at(group);
      owners.insert(swap_.at(group).begin(), swap_.at(group).end());
      add_owner_set(owners, {ConflictKind::Mode, group, -1}, 1);
    }
    return out;
  }

private:
  const UnifiedGraph &graph_;
  std::set<Owner> sync_;
  std::map<int, std::set<Owner>> nodes_;
  std::map<int, std::set<Owner>> switches_;
  std::map<PortKey, std::map<int, std::set<Owner>>> ports_;
  std::map<int, std::set<Owner>> straight_;
  std::map<int, std::set<Owner>> swap_;
};

auto scope_for(const std::Vector<RoutingScope> &scopes,
               const std::size_t net_id) -> const RoutingScope * {
  const auto it =
      std::find_if(scopes.begin(), scopes.end(),
                   [&](const auto &scope) { return scope.net_id == net_id; });
  return it == scopes.end() ? nullptr : &*it;
}

auto net_for(const std::Vector<RoutingNet> &nets, const std::size_t net_id)
    -> const RoutingNet * {
  const auto it = std::find_if(nets.begin(), nets.end(), [&](const auto &net) {
    return net.net_id == net_id;
  });
  return it == nets.end() ? nullptr : &*it;
}

auto path_wire_nodes(const UnifiedGraph &graph, const std::Vector<int> &path)
    -> std::size_t {
  auto nodes = std::set<int>{};
  for (const int node : path) {
    if (is_wirelength_resource_node(graph, node)) {
      nodes.insert(node);
    }
  }
  return nodes.size();
}

auto infer_unit_mask(const UnifiedGraph &graph, const std::Vector<int> &path)
    -> std::uint16_t {
  std::uint16_t mask = 0;
  for (const int node : path) {
    if (node >= 0 && static_cast<std::size_t>(node) < graph.nodes.size() &&
        graph.nodes[static_cast<std::size_t>(node)].kind ==
            UnifiedNodeKind::Track) {
      mask = static_cast<std::uint16_t>(
          mask | unit_bit(graph.nodes[static_cast<std::size_t>(node)].unit));
    }
  }
  return mask == 0 ? std::uint16_t{0xffff} : mask;
}

auto build_specs(const UnifiedGraph &graph, const std::Vector<RoutingNet> &nets,
                 const RoutingResult &baseline)
    -> std::map<Owner, std::Vector<RouteSpec>> {
  auto out = std::map<Owner, std::Vector<RouteSpec>>{};
  for (const auto &path : baseline.paths) {
    const auto *net = net_for(nets, path.net_id);
    if (net == nullptr || net->is_sync_bus || path.node_path.empty()) {
      continue;
    }
    out[path.net_id].push_back(RouteSpec{
        path.net_id, path.demand_id, path.source_index,
        path.physical_source_node, path.node_path.front(),
        path.node_path.back(), infer_unit_mask(graph, path.node_path)});
  }
  for (auto &[_, specs] : out) {
    std::sort(specs.begin(), specs.end(), [](const auto &lhs, const auto &rhs) {
      return lhs.demand_id < rhs.demand_id;
    });
  }
  return out;
}

auto history_value(const std::map<ConflictKey, double> &history,
                   const ConflictKey &key) -> double {
  const auto it = history.find(key);
  return it == history.end() ? 0.0 : it->second;
}

auto route_one(const UnifiedGraph &graph, const RoutingScope &scope,
               const Usage &usage, const RouteSpec &spec,
               const std::map<ConflictKey, double> &history,
               const int congestion_height) -> std::Vector<int> {
  if (spec.source < 0 || spec.sink < 0 ||
      static_cast<std::size_t>(spec.source) >= graph.nodes.size() ||
      static_cast<std::size_t>(spec.sink) >= graph.nodes.size()) {
    return {};
  }
  constexpr double infinity = std::numeric_limits<double>::infinity();
  auto distance = std::Vector<double>(graph.nodes.size(), infinity);
  auto predecessor = std::Vector<int>(graph.nodes.size(), -1);
  using QueueItem = std::pair<double, int>;
  auto pending = std::priority_queue<QueueItem, std::Vector<QueueItem>,
                                     std::greater<QueueItem>>{};
  distance[static_cast<std::size_t>(spec.source)] = 0.0;
  pending.emplace(0.0, spec.source);
  while (!pending.empty()) {
    const auto [cost, node] = pending.top();
    pending.pop();
    if (cost != distance[static_cast<std::size_t>(node)]) {
      continue;
    }
    if (node == spec.sink) {
      break;
    }
    for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(node)]) {
      if (static_cast<std::size_t>(arc_id) >= scope.arc_offset.size() ||
          scope.arc_offset[static_cast<std::size_t>(arc_id)] < 0) {
        continue;
      }
      const auto &arc = graph.arcs[static_cast<std::size_t>(arc_id)];
      if (arc.is_virtual_source_arc || !is_physical_node(graph, arc.v) ||
          !arc_unit_eligible(graph, arc, spec.unit_mask) ||
          usage.hard_blocks_node(spec.net_id, arc.v) ||
          usage.hard_blocks_arc(spec.net_id, arc)) {
        continue;
      }
      double incremental = 0.02;
      if (is_wirelength_resource_node(graph, arc.v) &&
          !usage.owner_uses_node(spec.net_id, arc.v)) {
        incremental += 1.0;
      }
      incremental += history_value(history, {ConflictKind::Node, arc.v, -1});
      const int node_conflicts = usage.node_conflicts(spec.net_id, arc.v);
      if (node_conflicts > 0) {
        incremental += congestion_height * node_conflicts;
      }
      const int arc_conflicts = usage.arc_conflicts(spec.net_id, arc);
      if (arc.physical_switch_id >= 0) {
        incremental += history_value(
            history, {ConflictKind::Switch, arc.physical_switch_id, -1});
        const int stage = switch_stage(arc.physical_switch_kind);
        incremental +=
            history_value(history, {ConflictKind::Port, arc.u, stage});
        incremental +=
            history_value(history, {ConflictKind::Port, arc.v, stage});
      }
      if (arc.mode_group_id >= 0) {
        incremental +=
            history_value(history, {ConflictKind::Mode, arc.mode_group_id, -1});
      }
      if (arc_conflicts > 0) {
        incremental += 2.0 * congestion_height * arc_conflicts;
      }
      const double next = cost + incremental;
      if (next + 1e-9 < distance[static_cast<std::size_t>(arc.v)]) {
        distance[static_cast<std::size_t>(arc.v)] = next;
        predecessor[static_cast<std::size_t>(arc.v)] = arc_id;
        pending.emplace(next, arc.v);
      }
    }
  }
  if (!std::isfinite(distance[static_cast<std::size_t>(spec.sink)])) {
    return {};
  }
  auto path = std::Vector<int>{spec.sink};
  for (int node = spec.sink; node != spec.source;) {
    const int arc_id = predecessor[static_cast<std::size_t>(node)];
    if (arc_id < 0) {
      return {};
    }
    node = graph.arcs[static_cast<std::size_t>(arc_id)].u;
    path.push_back(node);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

auto erase_owner_paths(RoutingResult &result, const Owner owner) -> void {
  result.paths.erase(
      std::remove_if(result.paths.begin(), result.paths.end(),
                     [&](const auto &path) { return path.net_id == owner; }),
      result.paths.end());
}

auto append_owner_routes(
    const UnifiedGraph &graph, const std::Vector<RoutingScope> &scopes,
    const std::map<Owner, std::Vector<RouteSpec>> &specs_by_owner,
    const Owner owner, const std::map<ConflictKey, double> &history,
    const int congestion_height, Usage &usage, RoutingResult &result)
    -> bool {
  const auto specs_it = specs_by_owner.find(owner);
  const auto *scope = scope_for(scopes, owner);
  if (specs_it == specs_by_owner.end() || scope == nullptr) {
    return false;
  }
  for (const auto &spec : specs_it->second) {
    auto path =
        route_one(graph, *scope, usage, spec, history, congestion_height);
    if (path.empty()) {
      erase_owner_paths(result, owner);
      return false;
    }
    result.paths.push_back(
        SourceSinkPairPath{spec.net_id, spec.source_index, spec.demand_id,
                           spec.physical_source_node, std::move(path)});
    usage.claim_path(owner, result.paths.back().node_path);
  }
  return true;
}

auto reroute_owner(
    const UnifiedGraph &graph, const std::Vector<RoutingNet> &nets,
    const std::Vector<RoutingScope> &scopes,
    const std::map<Owner, std::Vector<RouteSpec>> &specs_by_owner,
    const Owner owner, const std::map<ConflictKey, double> &history,
    const int congestion_height, RoutingResult &result) -> bool {
  erase_owner_paths(result, owner);
  auto usage = Usage{graph, nets, result};
  return append_owner_routes(graph, scopes, specs_by_owner, owner, history,
                             congestion_height, usage, result);
}

auto rebuild_metadata(const UnifiedGraph &graph, RoutingResult &result) -> void {
  auto switches = std::set<int>{};
  auto modes = result.vline_mode_straight_by_group;
  for (const auto &path : result.paths) {
    for (std::size_t index = 1; index < path.node_path.size(); ++index) {
      const int arc_id =
          find_arc(graph, path.node_path[index - 1], path.node_path[index]);
      if (arc_id < 0) {
        continue;
      }
      const auto &arc = graph.arcs[static_cast<std::size_t>(arc_id)];
      if (arc.physical_switch_id >= 0) {
        switches.insert(arc.physical_switch_id);
      }
      if (arc.mode_group_id >= 0) {
        modes[static_cast<std::size_t>(arc.mode_group_id)] =
            arc.is_vline_track_straight;
      }
    }
  }
  result.used_tob_switch_ids.assign(switches.begin(), switches.end());
  result.vline_mode_straight_by_group = std::move(modes);
  result.total_wirelength = total_wirelength(graph, result);
}

auto validate_candidate(const UnifiedGraph &graph,
                        const std::Vector<RoutingNet> &nets,
                        const std::Vector<RoutingScope> &scopes,
                        const std::map<Owner, std::Vector<RouteSpec>> &specs,
                        const RoutingResult &baseline,
                        const RoutingResult &candidate) -> bool {
  if (Usage{graph, nets, candidate}.report().overflow != 0) {
    return false;
  }
  auto baseline_sync =
      std::map<std::tuple<std::size_t, std::size_t, std::size_t>,
               const SourceSinkPairPath *>{};
  for (const auto &path : baseline.paths) {
    const auto *net = net_for(nets, path.net_id);
    if (net != nullptr && net->is_sync_bus) {
      baseline_sync.emplace(
          std::tuple{path.net_id, path.demand_id, path.source_index}, &path);
    }
  }
  auto counts = std::map<std::pair<std::size_t, std::size_t>, int>{};
  for (const auto &path : candidate.paths) {
    ++counts[{path.net_id, path.demand_id}];
    const auto *scope = scope_for(scopes, path.net_id);
    if (scope == nullptr || path.node_path.empty()) {
      return false;
    }
    const auto spec_it = specs.find(path.net_id);
    if (spec_it != specs.end()) {
      const auto expected =
          std::find_if(spec_it->second.begin(), spec_it->second.end(),
                       [&](const auto &value) {
                         return value.demand_id == path.demand_id &&
                                value.source_index == path.source_index;
                       });
      if (expected == spec_it->second.end() ||
          path.node_path.front() != expected->source ||
          path.node_path.back() != expected->sink ||
          path.physical_source_node != expected->physical_source_node) {
        return false;
      }
      for (const int node : path.node_path) {
        if (!is_physical_node(graph, node) ||
            static_cast<std::size_t>(node) >= scope->node_offset.size() ||
            scope->node_offset[static_cast<std::size_t>(node)] < 0 ||
            !node_unit_eligible(graph.nodes[static_cast<std::size_t>(node)],
                                expected->unit_mask)) {
          return false;
        }
      }
      for (std::size_t index = 1; index < path.node_path.size(); ++index) {
        const int arc =
            find_arc(graph, path.node_path[index - 1], path.node_path[index]);
        if (arc < 0 ||
            static_cast<std::size_t>(arc) >= scope->arc_offset.size() ||
            scope->arc_offset[static_cast<std::size_t>(arc)] < 0 ||
            !arc_unit_eligible(graph, graph.arcs[static_cast<std::size_t>(arc)],
                               expected->unit_mask)) {
          return false;
        }
      }
    }
    const auto *net = net_for(nets, path.net_id);
    if (net != nullptr && net->is_sync_bus) {
      const auto key =
          std::tuple{path.net_id, path.demand_id, path.source_index};
      const auto it = baseline_sync.find(key);
      if (it == baseline_sync.end() ||
          it->second->physical_source_node != path.physical_source_node ||
          it->second->node_path != path.node_path) {
        return false;
      }
    }
  }
  for (const auto &net : nets) {
    for (const auto &demand : net.demands) {
      if (counts[{net.net_id, demand.demand_id}] != 1) {
        return false;
      }
    }
  }
  return total_wirelength(graph, candidate) == candidate.total_wirelength;
}

auto shortest_wirelength(const UnifiedGraph &graph,
                         const RoutingScope &scope, const RouteSpec &spec)
    -> std::size_t {
  auto empty_result = RoutingResult{};
  auto empty_nets = std::Vector<RoutingNet>{};
  const auto usage = Usage{graph, empty_nets, empty_result};
  const auto path = route_one(graph, scope, usage, spec, {}, 0);
  return path.empty() ? std::numeric_limits<std::size_t>::max()
                      : path_wire_nodes(graph, path);
}

struct Trigger {
  Owner owner{0};
  std::size_t stretch{0};
  std::size_t wirelength{0};
};

auto trigger_order(const UnifiedGraph &graph,
                   const std::Vector<RoutingScope> &scopes,
                   const std::map<Owner, std::Vector<RouteSpec>> &specs,
                   const RoutingResult &result) -> std::Vector<Trigger> {
  auto out = std::Vector<Trigger>{};
  for (const auto &[owner, owner_specs] : specs) {
    const auto *scope = scope_for(scopes, owner);
    if (scope == nullptr) {
      continue;
    }
    auto paths = std::Vector<const SourceSinkPairPath *>{};
    for (const auto &path : result.paths) {
      if (path.net_id == owner)
        paths.push_back(&path);
    }
    const std::size_t wirelength = net_wirelength(graph, paths);
    std::size_t lower_bound = 0;
    bool reachable = true;
    for (const auto &spec : owner_specs) {
      const auto shortest = shortest_wirelength(graph, *scope, spec);
      if (shortest == std::numeric_limits<std::size_t>::max()) {
        reachable = false;
        break;
      }
      lower_bound = std::max(lower_bound, shortest);
    }
    if (reachable && wirelength > lower_bound) {
      out.push_back({owner, wirelength - lower_bound, wirelength});
    }
  }
  std::sort(out.begin(), out.end(), [](const auto &lhs, const auto &rhs) {
    return std::tie(rhs.stretch, rhs.wirelength, lhs.owner) <
           std::tie(lhs.stretch, lhs.wirelength, rhs.owner);
  });
  return out;
}

auto update_history(std::map<ConflictKey, double> &history,
                    const ConflictReport &report) -> void {
  for (auto &[_, value] : history) {
    value *= 0.9;
  }
  for (const auto &key : report.keys) {
    history[key] += 1.0;
  }
}

} // namespace

auto optimize_routes_rrr(const UnifiedGraph &graph,
                         const std::Vector<RoutingNet> &nets,
                         const std::Vector<RoutingScope> &scopes,
                         const RoutingResult &baseline,
                         const RrrOptions &options)
    -> RoutingResult {
  const auto begin = std::chrono::steady_clock::now();
  auto incumbent = baseline;
  incumbent.rrr_attempted = true;
  incumbent.rrr_baseline_wirelength = baseline.total_wirelength;
  incumbent.rrr_wirelength = baseline.total_wirelength;
  const auto finish = [&](RoutingResult result,
                          const std::String &status) -> RoutingResult {
    result.rrr_status = status;
    result.rrr_wirelength = result.total_wirelength;
    result.rrr_total_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - begin)
            .count();
    debug::info_fmt(
        "direct RRR summary: status={} accepted={} triggers={} "
        "accepted_triggers={} rrr_iterations={} rerouted_owners={} "
        "wirelength={}->{} total_ms={}",
        result.rrr_status, result.rrr_accepted,
        result.rrr_triggers, result.rrr_accepted_triggers,
        result.rrr_iterations,
        result.rrr_rerouted_owners,
        result.rrr_baseline_wirelength,
        result.rrr_wirelength, result.rrr_total_ms);
    return result;
  };

  if (!baseline.ok) {
    return finish(std::move(incumbent), "SKIPPED_NO_ILP_SOLUTION");
  }
  const auto specs = build_specs(graph, nets, baseline);
  if (specs.empty()) {
    return finish(std::move(incumbent), "SKIPPED_NO_NON_SYNC_NETS");
  }
  debug::info_fmt("direct RRR start: non_sync_owners={} "
                  "baseline_wirelength={} max_sweeps={} max_rrr_iterations={} "
                  "stagnation_limit={} sync_policy=hard-obstacle",
                  specs.size(), baseline.total_wirelength, options.max_sweeps,
                  options.max_iterations, options.stagnation_limit);

  for (int sweep = 0; sweep < options.max_sweeps; ++sweep) {
    bool sweep_improved = false;
    const auto triggers = trigger_order(graph, scopes, specs, incumbent);
    for (const auto &trigger : triggers) {
      ++incumbent.rrr_triggers;
      auto transaction = incumbent;
      auto history = std::map<ConflictKey, double>{};
      int congestion_height = 4;
      if (!reroute_owner(graph, nets, scopes, specs, trigger.owner, history,
                         congestion_height, transaction)) {
        debug::info_fmt("direct RRR trigger: sweep={} owner={} "
                        "stretch={} status=UNROUTABLE rollback=true",
                        sweep, trigger.owner, trigger.stretch);
        continue;
      }
      ++incumbent.rrr_rerouted_owners;

      auto best_overflow = std::numeric_limits<int>::max();
      auto best_wirelength = std::numeric_limits<std::size_t>::max();
      int stagnant = 0;
      bool converged = false;
      for (int iteration = 0; iteration <= options.max_iterations;
           ++iteration) {
        const auto usage = Usage{graph, nets, transaction};
        const auto report = usage.report();
        rebuild_metadata(graph, transaction);
        if (options.verbose_level >= 1) {
          debug::info_fmt("direct RRR iter: sweep={} trigger={} iter={} "
                          "overflow={} dirty_owners={} total_wirelength={}",
                          sweep, trigger.owner, iteration, report.overflow,
                          report.owners.size(), transaction.total_wirelength);
        }
        if (report.overflow == 0) {
          converged = validate_candidate(graph, nets, scopes, specs, baseline,
                                         transaction);
          break;
        }
        if (iteration == options.max_iterations || report.owners.empty()) {
          break;
        }
        const auto score =
            std::pair{report.overflow, transaction.total_wirelength};
        const auto best = std::pair{best_overflow, best_wirelength};
        if (score < best) {
          best_overflow = report.overflow;
          best_wirelength = transaction.total_wirelength;
          stagnant = 0;
        } else if (++stagnant >= options.stagnation_limit) {
          break;
        }
        if (stagnant > 0 && stagnant % 4 == 0) {
          congestion_height = std::min(congestion_height + 4, 16);
        }
        update_history(history, report);
        auto dirty =
            std::Vector<Owner>(report.owners.begin(), report.owners.end());
        std::sort(
            dirty.begin(), dirty.end(), [&](const Owner lhs, const Owner rhs) {
              const int lhs_exposure =
                  report.exposure.contains(lhs) ? report.exposure.at(lhs) : 0;
              const int rhs_exposure =
                  report.exposure.contains(rhs) ? report.exposure.at(rhs) : 0;
              return std::tie(rhs_exposure, lhs) < std::tie(lhs_exposure, rhs);
            });
        for (const Owner owner : dirty) {
          erase_owner_paths(transaction, owner);
        }
        auto reroute_usage = Usage{graph, nets, transaction};
        bool routed = true;
        for (const Owner owner : dirty) {
          if (!append_owner_routes(graph, scopes, specs, owner, history,
                                   congestion_height, reroute_usage,
                                   transaction)) {
            routed = false;
            break;
          }
          ++incumbent.rrr_rerouted_owners;
        }
        ++incumbent.rrr_iterations;
        if (!routed) {
          break;
        }
      }

      if (converged &&
          transaction.total_wirelength < incumbent.total_wirelength) {
        const auto before = incumbent.total_wirelength;
        const auto persistent =
            std::tuple{incumbent.rrr_triggers,
                       incumbent.rrr_accepted_triggers,
                       incumbent.rrr_iterations,
                       incumbent.rrr_rerouted_owners};
        incumbent = std::move(transaction);
        incumbent.rrr_attempted = true;
        incumbent.rrr_accepted = true;
        incumbent.rrr_baseline_wirelength =
            baseline.total_wirelength;
        std::tie(incumbent.rrr_triggers,
                 incumbent.rrr_accepted_triggers,
                 incumbent.rrr_iterations,
                 incumbent.rrr_rerouted_owners) = persistent;
        ++incumbent.rrr_accepted_triggers;
        sweep_improved = true;
        debug::info_fmt("direct RRR trigger: sweep={} owner={} "
                        "stretch={} status=ACCEPT wirelength={}->{}",
                        sweep, trigger.owner, trigger.stretch, before,
                        incumbent.total_wirelength);
      } else {
        debug::info_fmt(
            "direct RRR trigger: sweep={} owner={} stretch={} status={} "
            "rollback=true candidate_wirelength={} incumbent_wirelength={}",
            sweep, trigger.owner, trigger.stretch,
            converged ? "NO_IMPROVEMENT" : "RRR_NOT_CONVERGED",
            transaction.total_wirelength, incumbent.total_wirelength);
      }
    }
    if (!sweep_improved) {
      break;
    }
  }
  const auto status = incumbent.rrr_accepted
                          ? std::String{"IMPROVED"}
                          : std::String{"NO_IMPROVEMENT"};
  return finish(std::move(incumbent), status);
}

} // namespace PR_tool
