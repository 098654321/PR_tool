#include "post_sat_ilp/post_sat_ilp.hh"

#include "common/cob_unit_mask.hh"
#include "global_route_v17/highs_log_sink.hh"
#include "sat/routing_path_log.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <debug/debug.hh>
#include <format>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>

#ifdef USE_HIGHS
#include <Highs.h>
#endif

namespace PR_tool {
namespace {
constexpr double kMipRelativeGap = 0.015;

struct UndirectedEdge {
  int u{-1}, v{-1};
  std::Vector<int> directed_arcs;
  int representative_arc{-1};
  int physical_switch_id{-1};
  PhysicalSwitchKind physical_switch_kind{PhysicalSwitchKind::None};
  int mode_group_id{-1};
  bool is_straight{false}, is_swap{false};
};

struct UndirectedGraph {
  std::Vector<UndirectedEdge> edges;
  std::Vector<std::Vector<int>> incident_edge_ids;
  std::Vector<int> edge_id_by_arc;
  std::map<std::pair<int, int>, int> edge_id_by_nodes;
};

struct Pair {
  std::size_t id{}, net{}, net_id{}, demand{}, source_index{};
  int source{-1}, sink{-1}, physical_source{-1};
  std::uint16_t unit{0xffff};
  std::Vector<int> nodes, edges;
  std::set<int> incumbent;
};

struct Net {
  std::size_t id{}, net_id{};
  std::uint16_t unit{0xffff};
  std::Vector<std::size_t> pairs;
};

struct Locked {
  std::Vector<bool> nodes;
  std::set<int> switches;
  std::map<int, bool> modes;
  std::map<std::pair<int, int>, int> matching;
  std::set<std::size_t> nets;
};

struct Prepared {
  std::Vector<Net> nets;
  std::Vector<Pair> pairs;
  std::set<std::size_t> selected;
  std::Vector<std::Vector<std::size_t>> components;
  std::Vector<std::Vector<std::size_t>> pairs_by_node;
  std::Vector<std::Vector<std::size_t>> pairs_by_edge;
};

struct PairVars {
  std::map<int, int> f, d;
};

struct Stats {
  std::size_t f{}, d{}, y{}, m{}, rows{}, nz{}, start{};
  std::size_t endpoint_degree{}, internal_degree{}, f_implies_d{},
      f_implies_y{}, edge_pair_exclusivity{}, node_pair_exclusivity{},
      tob_matching{}, mode_binding{};
  long long build{}, solve{};
  double objective{}, bound{}, gap{};
};

struct PairSolution {
  std::set<int> edges, nodes;
};

struct Result {
  bool ok{};
  std::String status;
  Stats stats;
  std::Vector<PairSolution> pairs;
  std::map<int, bool> modes;
};

auto physical(const UnifiedGraph &g, int v) -> bool {
  return v >= 0 && static_cast<std::size_t>(v) < g.nodes.size() &&
         g.nodes[v].kind != UnifiedNodeKind::VirtualSource;
}

auto edge_key(int u, int v) -> std::pair<int, int> { return std::minmax(u, v); }

auto make_undirected_graph(const UnifiedGraph &g) -> UndirectedGraph {
  auto out = UndirectedGraph{};
  out.incident_edge_ids.resize(g.nodes.size());
  out.edge_id_by_arc.assign(g.arcs.size(), -1);
  for (std::size_t arc_id = 0; arc_id < g.arcs.size(); ++arc_id) {
    const auto &arc = g.arcs[arc_id];
    if (arc.is_virtual_source_arc || !physical(g, arc.u) || !physical(g, arc.v))
      continue;
    const auto key = edge_key(arc.u, arc.v);
    auto [it, inserted] =
        out.edge_id_by_nodes.emplace(key, static_cast<int>(out.edges.size()));
    if (inserted) {
      auto edge = UndirectedEdge{};
      edge.u = key.first;
      edge.v = key.second;
      edge.representative_arc = static_cast<int>(arc_id);
      edge.physical_switch_id = arc.physical_switch_id;
      edge.physical_switch_kind = arc.physical_switch_kind;
      edge.mode_group_id = arc.mode_group_id;
      edge.is_straight = arc.is_vline_track_straight;
      edge.is_swap = arc.is_vline_track_swap;
      out.edges.push_back(std::move(edge));
      out.incident_edge_ids[key.first].push_back(it->second);
      out.incident_edge_ids[key.second].push_back(it->second);
    } else {
      auto &edge = out.edges[it->second];
      if (edge.physical_switch_id >= 0 && arc.physical_switch_id >= 0 &&
          edge.physical_switch_id != arc.physical_switch_id)
        throw std::logic_error(
            "V22 parallel physical switches need distinct endpoints");
      if (edge.physical_switch_id < 0 && arc.physical_switch_id >= 0) {
        edge.physical_switch_id = arc.physical_switch_id;
        edge.physical_switch_kind = arc.physical_switch_kind;
        edge.mode_group_id = arc.mode_group_id;
        edge.is_straight = arc.is_vline_track_straight;
        edge.is_swap = arc.is_vline_track_swap;
        edge.representative_arc = static_cast<int>(arc_id);
      }
      if ((edge.mode_group_id >= 0 || arc.mode_group_id >= 0) &&
          (edge.mode_group_id != arc.mode_group_id ||
           edge.is_straight != arc.is_vline_track_straight ||
           edge.is_swap != arc.is_vline_track_swap))
        throw std::logic_error(
            "V22 reverse arcs disagree on straight/swap mode");
    }
    out.edges[it->second].directed_arcs.push_back(static_cast<int>(arc_id));
    out.edge_id_by_arc[arc_id] = it->second;
  }
  return out;
}

auto other_end(const UndirectedEdge &edge, int node) -> int {
  if (edge.u == node)
    return edge.v;
  if (edge.v == node)
    return edge.u;
  throw std::logic_error("V22 edge is not incident to node");
}

auto path_edges(const UndirectedGraph &ug, const std::Vector<int> &path)
    -> std::Vector<int> {
  auto out = std::Vector<int>{};
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto it = ug.edge_id_by_nodes.find(edge_key(path[i - 1], path[i]));
    if (it == ug.edge_id_by_nodes.end())
      throw std::logic_error("V22 path lacks undirected graph edge");
    out.push_back(it->second);
  }
  return out;
}

auto net_for(const std::Vector<RoutingNet> &nets, std::size_t id)
    -> const RoutingNet * {
  const auto it = std::find_if(nets.begin(), nets.end(),
                               [&](const auto &n) { return n.net_id == id; });
  return it == nets.end() ? nullptr : &*it;
}

auto demand_for(const RoutingNet &net, std::size_t id)
    -> const RoutingDemand * {
  const auto it =
      std::find_if(net.demands.begin(), net.demands.end(),
                   [&](const auto &d) { return d.demand_id == id; });
  return it == net.demands.end() ? nullptr : &*it;
}

auto scope_for(const std::Vector<UnifiedSatNetScope> &scopes, std::size_t id)
    -> const UnifiedSatNetScope * {
  const auto it = std::find_if(scopes.begin(), scopes.end(),
                               [&](const auto &s) { return s.net_id == id; });
  return it == scopes.end() ? nullptr : &*it;
}

auto paths_for(const SatRoutingResult &result, std::size_t net_id)
    -> std::Vector<const SourceSinkPairPath *> {
  auto out = std::Vector<const SourceSinkPairPath *>{};
  for (const auto &path : result.paths)
    if (path.net_id == net_id)
      out.push_back(&path);
  std::sort(out.begin(), out.end(), [](auto a, auto b) {
    return std::tie(a->demand_id, a->source_index) <
           std::tie(b->demand_id, b->source_index);
  });
  return out;
}

auto matching_endpoints(const UnifiedGraph &g, const UndirectedEdge &edge)
    -> std::Vector<std::pair<int, int>> {
  const auto u = g.nodes[edge.u].kind, v = g.nodes[edge.v].kind;
  if (u == UnifiedNodeKind::Bump && v == UnifiedNodeKind::HLine)
    return {{0, edge.u}, {1, edge.v}};
  if (v == UnifiedNodeKind::Bump && u == UnifiedNodeKind::HLine)
    return {{0, edge.v}, {1, edge.u}};
  if (u == UnifiedNodeKind::HLine && v == UnifiedNodeKind::VLine)
    return {{2, edge.u}, {3, edge.v}};
  if (v == UnifiedNodeKind::HLine && u == UnifiedNodeKind::VLine)
    return {{2, edge.v}, {3, edge.u}};
  return {};
}

auto locked_resources(const UnifiedGraph &g, const UndirectedGraph &ug,
                      const std::Vector<RoutingNet> &nets,
                      const SatRoutingResult &sat) -> Locked {
  auto out = Locked{};
  auto counted_matching_switches = std::set<int>{};
  out.nodes.assign(g.nodes.size(), false);
  for (const auto &path : sat.paths) {
    const auto *net = net_for(nets, path.net_id);
    if (net == nullptr || !net->is_sync_bus)
      continue;
    out.nets.insert(path.net_id);
    for (int v : path.node_path)
      if (physical(g, v))
        out.nodes[v] = true;
    for (int edge_id : path_edges(ug, path.node_path)) {
      const auto &edge = ug.edges[edge_id];
      if (edge.physical_switch_id >= 0) {
        out.switches.insert(edge.physical_switch_id);
        if (counted_matching_switches.insert(edge.physical_switch_id).second)
          for (const auto key : matching_endpoints(g, edge))
            ++out.matching[key];
      }
      if (edge.mode_group_id >= 0) {
        const auto [it, ok] =
            out.modes.emplace(edge.mode_group_id, edge.is_straight);
        if (!ok && it->second != edge.is_straight)
          throw std::logic_error("V22 fixed bus mode conflict");
      }
    }
  }
  for (const auto &[_, count] : out.matching)
    if (count > 1)
      throw std::logic_error("V22 fixed bus matching conflict");
  return out;
}

auto infer_unit(const UnifiedGraph &g,
                const std::Vector<const SourceSinkPairPath *> &paths)
    -> std::uint16_t {
  auto units = std::set<std::size_t>{};
  for (const auto *path : paths)
    for (int v : path->node_path)
      if (physical(g, v) && g.nodes[v].kind == UnifiedNodeKind::Track)
        units.insert(g.nodes[v].unit);
  if (units.size() != 1)
    throw std::logic_error("V22 incumbent does not determine a unique COBUnit");
  return unit_bit(*units.begin());
}

auto edge_unit_eligible(const UnifiedGraph &g, const UndirectedEdge &edge,
                        std::uint16_t unit) -> bool {
  for (int arc_id : edge.directed_arcs)
    if (arc_unit_eligible(g, g.arcs[arc_id], unit))
      return true;
  return false;
}

auto make_pair_domain(const UnifiedGraph &g, const UndirectedGraph &ug,
                      const UnifiedSatNetScope &fallback_scope,
                      const PairRoutingState *state, const Locked &locked,
                      Pair &pair, const SourceSinkPairPath &incumbent) -> void {
  auto allowed = std::Vector<bool>(g.nodes.size(), false);
  if (state == nullptr) {
    for (int v : fallback_scope.node_ids)
      if (physical(g, v) && node_unit_eligible(g.nodes[v], pair.unit))
        allowed[v] = true;
  } else {
    const auto source_tob = g.nodes[pair.source].tob,
               sink_tob = g.nodes[pair.sink].tob;
    const bool source_is_tob =
        g.nodes[pair.source].kind != UnifiedNodeKind::Track;
    const bool sink_is_tob = g.nodes[pair.sink].kind != UnifiedNodeKind::Track;
    for (int v : fallback_scope.node_ids) {
      const auto &node = g.nodes[v];
      if (!physical(g, v) || !node_unit_eligible(node, pair.unit))
        continue;
      if (node.kind == UnifiedNodeKind::Track)
        allowed[v] = state->allowed_channels.contains(
            GlobalChannelCoord{node.track_dir, node.track_row, node.track_col});
      else
        allowed[v] = (source_is_tob && node.tob == source_tob) ||
                     (sink_is_tob && node.tob == sink_tob);
    }
  }
  for (int v : incumbent.node_path)
    if (physical(g, v))
      allowed[v] = true;
  if (!allowed[pair.source] || !allowed[pair.sink])
    throw std::logic_error("V22 pair scope dropped endpoint");

  auto candidates = std::set<int>{};
  for (int arc_id : fallback_scope.arc_ids) {
    if (arc_id < 0 ||
        static_cast<std::size_t>(arc_id) >= ug.edge_id_by_arc.size())
      continue;
    const int edge_id = ug.edge_id_by_arc[arc_id];
    if (edge_id < 0)
      continue;
    const auto &edge = ug.edges[edge_id];
    if (!allowed[edge.u] || !allowed[edge.v] ||
        !edge_unit_eligible(g, edge, pair.unit) || locked.nodes[edge.u] ||
        locked.nodes[edge.v])
      continue;
    if (edge.physical_switch_id >= 0 &&
        locked.switches.contains(edge.physical_switch_id))
      continue;
    if (edge.mode_group_id >= 0 && locked.modes.contains(edge.mode_group_id) &&
        locked.modes.at(edge.mode_group_id) != edge.is_straight)
      continue;
    candidates.insert(edge_id);
  }
  for (int edge_id : path_edges(ug, incumbent.node_path)) {
    const auto &edge = ug.edges[edge_id];
    if (locked.nodes[edge.u] || locked.nodes[edge.v] ||
        (edge.physical_switch_id >= 0 &&
         locked.switches.contains(edge.physical_switch_id)))
      throw std::logic_error("V22 incumbent conflicts with fixed bus");
    candidates.insert(edge_id);
    pair.incumbent.insert(edge_id);
  }

  auto reachable = std::Vector<bool>(g.nodes.size(), false);
  auto q = std::queue<int>{};
  reachable[pair.source] = true;
  q.push(pair.source);
  while (!q.empty()) {
    const int v = q.front();
    q.pop();
    for (int edge_id : ug.incident_edge_ids[v])
      if (candidates.contains(edge_id)) {
        const int next = other_end(ug.edges[edge_id], v);
        if (!reachable[next]) {
          reachable[next] = true;
          q.push(next);
        }
      }
  }
  if (!reachable[pair.sink])
    throw std::logic_error("V22 pair-local scope disconnects sink");

  auto nodes = std::set<int>{pair.source, pair.sink};
  for (int edge_id : candidates) {
    const auto &edge = ug.edges[edge_id];
    if (reachable[edge.u] && reachable[edge.v]) {
      pair.edges.push_back(edge_id);
      nodes.insert(edge.u);
      nodes.insert(edge.v);
    }
  }
  pair.nodes.assign(nodes.begin(), nodes.end());
  const auto active = std::set<int>(pair.edges.begin(), pair.edges.end());
  for (int edge_id : pair.incumbent)
    if (!active.contains(edge_id))
      throw std::logic_error("V22 pair domain dropped incumbent edge");
}

auto prepare(const UnifiedGraph &g, const UndirectedGraph &ug,
             const std::Vector<RoutingNet> &routing_nets,
             const std::Vector<UnifiedSatNetScope> &scopes,
             const SatRoutingResult &sat, const RoutingProblemState *state)
    -> std::pair<Prepared, Locked> {
  auto prepared = Prepared{};
  for (const auto &net : routing_nets)
    if (is_post_sat_ilp_target(net))
      prepared.selected.insert(net.net_id);
  const auto locked = locked_resources(g, ug, routing_nets, sat);

  for (const auto &routing_net : routing_nets) {
    if (!is_post_sat_ilp_target(routing_net))
      continue;
    const auto *scope = scope_for(scopes, routing_net.net_id);
    const auto paths = paths_for(sat, routing_net.net_id);
    if (scope == nullptr || paths.empty() ||
        paths.size() != routing_net.demands.size())
      throw std::logic_error("V22 missing final scope or incumbent path");
    auto net = Net{};
    net.id = prepared.nets.size();
    net.net_id = routing_net.net_id;
    net.unit = infer_unit(g, paths);
    for (const auto *path : paths) {
      const auto *demand = demand_for(routing_net, path->demand_id);
      if (demand == nullptr ||
          path->source_index >= routing_net.sources.size() ||
          path->node_path.empty() ||
          path->node_path.front() !=
              resolve_graph_node(g, routing_net.sources[path->source_index]) ||
          path->node_path.back() != resolve_graph_node(g, demand->sink))
        throw std::logic_error("V22 invalid pair endpoints");
      auto pair = Pair{};
      pair.id = prepared.pairs.size();
      pair.net = net.id;
      pair.net_id = net.net_id;
      pair.demand = path->demand_id;
      pair.source_index = path->source_index;
      pair.source = path->node_path.front();
      pair.sink = path->node_path.back();
      pair.physical_source = path->physical_source_node;
      pair.unit = net.unit;
      const auto *local =
          state == nullptr
              ? nullptr
              : find_pair_state(*state, PairKey{pair.net_id, pair.demand,
                                                pair.source_index});
      if (state != nullptr && local == nullptr)
        throw std::logic_error("V22 missing final PairRoutingState");
      make_pair_domain(g, ug, *scope, local, locked, pair, *path);
      net.pairs.push_back(pair.id);
      prepared.pairs.push_back(std::move(pair));
    }
    prepared.nets.push_back(std::move(net));
  }

  prepared.pairs_by_node.resize(g.nodes.size());
  prepared.pairs_by_edge.resize(ug.edges.size());
  for (const auto &pair : prepared.pairs) {
    for (int v : pair.nodes)
      prepared.pairs_by_node[v].push_back(pair.id);
    for (int edge_id : pair.edges)
      prepared.pairs_by_edge[edge_id].push_back(pair.id);
  }

  auto adj = std::Vector<std::set<std::size_t>>(prepared.nets.size());
  auto owner = std::map<std::tuple<int, int, int>, std::size_t>{};
  const auto record = [&](std::tuple<int, int, int> key, std::size_t net) {
    const auto [it, inserted] = owner.emplace(key, net);
    if (!inserted && it->second != net) {
      adj[net].insert(it->second);
      adj[it->second].insert(net);
    }
  };
  for (const auto &pair : prepared.pairs) {
    for (int v : pair.nodes)
      record({0, v, 0}, pair.net);
    for (int edge_id : pair.edges) {
      const auto &edge = ug.edges[edge_id];
      record({1, edge_id, 0}, pair.net);
      if (edge.mode_group_id >= 0)
        record({2, edge.mode_group_id, 0}, pair.net);
      for (const auto [stage, node] : matching_endpoints(g, edge))
        record({3, stage, node}, pair.net);
    }
  }
  auto seen = std::Vector<bool>(prepared.nets.size());
  for (std::size_t i = 0; i < prepared.nets.size(); ++i)
    if (!seen[i]) {
      auto component = std::Vector<std::size_t>{};
      auto q = std::queue<std::size_t>{};
      seen[i] = true;
      q.push(i);
      while (!q.empty()) {
        const auto n = q.front();
        q.pop();
        component.push_back(n);
        for (auto next : adj[n])
          if (!seen[next]) {
            seen[next] = true;
            q.push(next);
          }
      }
      prepared.components.push_back(std::move(component));
    }
  return {std::move(prepared), locked};
}

auto log_model_stats(const Prepared &p, const Locked &locked,
                     const std::Vector<std::size_t> &component,
                     std::size_t unique_edges, const Stats &stats) -> void {
  std::size_t node_slots = 0, edge_slots = 0, pairs = 0;
  for (const auto net_id : component)
    for (const auto pair_id : p.nets[net_id].pairs) {
      node_slots += p.pairs[pair_id].nodes.size();
      edge_slots += p.pairs[pair_id].edges.size();
      ++pairs;
    }
  debug::info("========== V22 undirected post-SAT ILP stats (-v) ==========");
  debug::info_fmt("  nets / pairs                    : {} / {}",
                  component.size(), pairs);
  debug::info_fmt("  fixed SyncBus nets              : {}", locked.nets.size());
  debug::info_fmt("  pair-local node slots           : {}", node_slots);
  debug::info_fmt("  pair-local undirected edge slots: {}", edge_slots);
  debug::info_fmt("  component physical-edge union   : {}", unique_edges);
  debug::info_fmt("  variables F/D/Y/M               : {}/{}/{}/{}", stats.f,
                  stats.d, stats.y, stats.m);
  debug::info_fmt("  MIP-start entries               : {}", stats.start);
  debug::info_fmt("  endpoint/internal degree rows   : {}/{}",
                  stats.endpoint_degree, stats.internal_degree);
  debug::info_fmt("  F=>D / F=>Y rows                : {} / {}",
                  stats.f_implies_d, stats.f_implies_y);
  debug::info_fmt("  pairwise edge/node conflicts    : {} / {}",
                  stats.edge_pair_exclusivity, stats.node_pair_exclusivity);
  debug::info_fmt("  TOB matching / mode rows        : {} / {}",
                  stats.tob_matching, stats.mode_binding);
  debug::info_fmt("  total variables / constraints   : {} / {}",
                  stats.f + stats.d + stats.y + stats.m, stats.rows);
  debug::info("============================================================");
}

#ifdef USE_HIGHS
class Mip {
public:
  Mip(int verbose, std::string_view log, int minutes)
      : log_(log, verbose >= 2, true) {
    log_.attach(h_);
    check(h_.setOptionValue("mip_rel_gap", kMipRelativeGap));
    if (minutes > 0)
      check(h_.setOptionValue("time_limit", 60. * minutes));
  }
  auto bin(double cost = 0.) -> int {
    const int x = vars_++;
    check(h_.addCol(cost, 0, 1, 0, nullptr, nullptr));
    check(h_.changeColIntegrality(x, HighsVarType::kInteger));
    return x;
  }
  auto row(double lo, double hi, const std::Vector<std::pair<int, double>> &in)
      -> void {
    auto sums = std::map<int, double>{};
    for (const auto [x, value] : in)
      sums[x] += value;
    auto ids = std::Vector<HighsInt>{};
    auto values = std::Vector<double>{};
    for (const auto [x, value] : sums)
      if (value != 0) {
        ids.push_back(x);
        values.push_back(value);
      }
    check(h_.addRow(lo, hi, ids.size(), ids.empty() ? nullptr : ids.data(),
                    values.empty() ? nullptr : values.data()));
    ++rows_;
    nz_ += ids.size();
  }
  auto start(const std::map<int, double> &x) -> void {
    auto ids = std::Vector<HighsInt>{};
    auto values = std::Vector<double>{};
    for (const auto [id, value] : x) {
      ids.push_back(id);
      values.push_back(value);
    }
    check(h_.setSolution(ids.size(), ids.data(), values.data()));
  }
  auto run() -> void { check(h_.run()); }
  auto feasible() const -> bool {
    return h_.getInfo().primal_solution_status == kSolutionStatusFeasible;
  }
  auto values() const -> const std::Vector<double> & {
    return h_.getSolution().col_value;
  }
  auto info() const -> const HighsInfo & { return h_.getInfo(); }
  auto status() const -> std::String {
    return h_.modelStatusToString(h_.getModelStatus());
  }
  auto vars() const -> std::size_t { return vars_; }
  auto rows() const -> std::size_t { return rows_; }
  auto nz() const -> std::size_t { return nz_; }

private:
  static auto check(HighsStatus status) -> void {
    if (status == HighsStatus::kError)
      throw std::runtime_error("V22 HiGHS API failure");
  }
  HighsLogSink log_;
  Highs h_;
  std::size_t vars_{}, rows_{}, nz_{};
};

auto solve_component(const UnifiedGraph &g, const UndirectedGraph &ug,
                     const Prepared &p, const Locked &locked,
                     const std::Vector<std::size_t> &ids,
                     const SatRoutingResult &sat,
                     const PostSatIlpOptions &options) -> Result {
  const auto begin = std::chrono::steady_clock::now();
  auto result = Result{};
  auto mip = Mip(options.verbose_level, options.highs_log_path,
                 options.highs_time_limit_minutes);
  auto pv = std::map<std::size_t, PairVars>{};
  auto y = std::map<int, int>{};
  auto component_nets = std::set<std::size_t>(ids.begin(), ids.end());

  for (auto net_id : ids)
    for (auto pair_id : p.nets[net_id].pairs) {
      for (int edge_id : p.pairs[pair_id].edges) {
        pv[pair_id].f.emplace(edge_id, mip.bin());
        ++result.stats.f;
        if (!y.contains(edge_id)) {
          y.emplace(edge_id, mip.bin(1.));
          ++result.stats.y;
        }
      }
      for (int node : p.pairs[pair_id].nodes) {
        pv[pair_id].d.emplace(node, mip.bin());
        ++result.stats.d;
      }
    }

  for (auto net_id : ids)
    for (auto pair_id : p.nets[net_id].pairs) {
      const auto &pair = p.pairs[pair_id];
      const auto &vars = pv.at(pair_id);
      for (int edge_id : pair.edges) {
        const auto &edge = ug.edges[edge_id];
        mip.row(-kHighsInf, 0,
                {{vars.f.at(edge_id), 1}, {vars.d.at(edge.u), -1}});
        mip.row(-kHighsInf, 0,
                {{vars.f.at(edge_id), 1}, {vars.d.at(edge.v), -1}});
        result.stats.f_implies_d += 2;
        mip.row(-kHighsInf, 0, {{vars.f.at(edge_id), 1}, {y.at(edge_id), -1}});
        ++result.stats.f_implies_y;
      }
      for (int node : pair.nodes) {
        auto degree = std::Vector<std::pair<int, double>>{};
        for (int edge_id : ug.incident_edge_ids[node])
          if (vars.f.contains(edge_id))
            degree.emplace_back(vars.f.at(edge_id), 1);
        if (node == pair.source || node == pair.sink) {
          mip.row(1, 1, degree);
          ++result.stats.endpoint_degree;
        } else {
          degree.emplace_back(vars.d.at(node), -2);
          mip.row(0, 0, degree);
          ++result.stats.internal_degree;
        }
      }
    }

  for (const auto &[edge_id, _] : y) {
    const auto &pairs = p.pairs_by_edge[edge_id];
    for (std::size_t i = 0; i < pairs.size(); ++i)
      for (std::size_t j = i + 1; j < pairs.size(); ++j) {
        const auto &a = p.pairs[pairs[i]], &b = p.pairs[pairs[j]];
        if (a.net == b.net || !component_nets.contains(a.net) ||
            !component_nets.contains(b.net))
          continue;
        mip.row(
            -kHighsInf, 1,
            {{pv.at(a.id).f.at(edge_id), 1}, {pv.at(b.id).f.at(edge_id), 1}});
        ++result.stats.edge_pair_exclusivity;
      }
  }
  for (std::size_t node = 0; node < p.pairs_by_node.size(); ++node) {
    const auto &pairs = p.pairs_by_node[node];
    for (std::size_t i = 0; i < pairs.size(); ++i)
      for (std::size_t j = i + 1; j < pairs.size(); ++j) {
        const auto &a = p.pairs[pairs[i]], &b = p.pairs[pairs[j]];
        if (a.net == b.net || !component_nets.contains(a.net) ||
            !component_nets.contains(b.net))
          continue;
        mip.row(-kHighsInf, 1,
                {{pv.at(a.id).d.at(static_cast<int>(node)), 1},
                 {pv.at(b.id).d.at(static_cast<int>(node)), 1}});
        ++result.stats.node_pair_exclusivity;
      }
  }

  auto matching = std::map<std::pair<int, int>, std::set<int>>{};
  auto modes = std::map<int, std::set<std::pair<int, bool>>>{};
  for (const auto &[edge_id, y_var] : y) {
    const auto &edge = ug.edges[edge_id];
    for (const auto key : matching_endpoints(g, edge))
      matching[key].insert(y_var);
    if (edge.mode_group_id >= 0)
      modes[edge.mode_group_id].insert({y_var, edge.is_straight});
  }
  for (const auto &[key, vars] : matching) {
    auto row = std::Vector<std::pair<int, double>>{};
    for (int var : vars)
      row.emplace_back(var, 1);
    mip.row(-kHighsInf,
            1 - (locked.matching.contains(key) ? locked.matching.at(key) : 0),
            row);
    ++result.stats.tob_matching;
  }
  auto mvars = std::map<int, int>{};
  for (const auto &[group, _] : modes)
    if (!locked.modes.contains(group)) {
      mvars.emplace(group, mip.bin());
      ++result.stats.m;
    }
  for (const auto &[group, uses] : modes) {
    if (!mvars.contains(group))
      continue;
    for (const auto [y_var, straight] : uses) {
      if (straight)
        mip.row(-kHighsInf, 0, {{y_var, 1}, {mvars.at(group), -1}});
      else
        mip.row(-kHighsInf, 1, {{y_var, 1}, {mvars.at(group), 1}});
      ++result.stats.mode_binding;
    }
  }

  auto start = std::map<int, double>{};
  for (int var = 0; var < static_cast<int>(mip.vars()); ++var)
    start[var] = 0.;
  for (auto net_id : ids)
    for (auto pair_id : p.nets[net_id].pairs) {
      const auto &pair = p.pairs[pair_id];
      for (int edge_id : pair.incumbent) {
        start[pv.at(pair_id).f.at(edge_id)] = 1;
        const auto &edge = ug.edges[edge_id];
        start[pv.at(pair_id).d.at(edge.u)] = 1;
        start[pv.at(pair_id).d.at(edge.v)] = 1;
        start[y.at(edge_id)] = 1;
      }
    }
  for (const auto [group, var] : mvars)
    if (sat.vline_mode_straight_by_group.contains(group))
      start[var] = sat.vline_mode_straight_by_group.at(group);
  mip.start(start);
  result.stats.start = start.size();
  result.stats.build = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - begin)
                           .count();
  result.stats.rows = mip.rows();
  result.stats.nz = mip.nz();
  if (options.verbose_level >= 1)
    log_model_stats(p, locked, ids, y.size(), result.stats);

  std::size_t pairs = 0;
  for (auto net_id : ids)
    pairs += p.nets[net_id].pairs.size();
  debug::info_fmt("V22 undirected ILP component: nets={} pairs={} vars={} "
                  "constraints={} F={} D={} Y={} M={} edge_conflicts={} "
                  "node_conflicts={} warm_start={}",
                  ids.size(), pairs, mip.vars(), mip.rows(), result.stats.f,
                  result.stats.d, result.stats.y, result.stats.m,
                  result.stats.edge_pair_exclusivity,
                  result.stats.node_pair_exclusivity, result.stats.start);

  const auto solve_begin = std::chrono::steady_clock::now();
  mip.run();
  result.stats.solve = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - solve_begin)
                           .count();
  result.status = mip.status();
  if (!mip.feasible())
    return result;
  const auto &values = mip.values();
  result.pairs.resize(p.pairs.size());
  for (auto net_id : ids)
    for (auto pair_id : p.nets[net_id].pairs) {
      for (const auto [edge_id, var] : pv.at(pair_id).f)
        if (values[var] > .5)
          result.pairs[pair_id].edges.insert(edge_id);
      for (const auto [node, var] : pv.at(pair_id).d)
        if (values[var] > .5)
          result.pairs[pair_id].nodes.insert(node);
    }
  for (const auto [group, var] : mvars)
    result.modes[group] = values[var] > .5;
  result.stats.objective = mip.info().objective_function_value;
  result.stats.bound = mip.info().mip_dual_bound;
  result.stats.gap = mip.info().mip_gap;
  result.ok = true;
  return result;
}
#else
auto solve_component(const UnifiedGraph &, const UndirectedGraph &,
                     const Prepared &, const Locked &,
                     const std::Vector<std::size_t> &, const SatRoutingResult &,
                     const PostSatIlpOptions &) -> Result {
  auto result = Result{};
  result.status = "HIGHS_UNAVAILABLE";
  return result;
}
#endif

auto valid(const UndirectedGraph &ug, const Prepared &p, const Result &result)
    -> bool {
  if (result.pairs.size() != p.pairs.size())
    return false;
  for (const auto &pair : p.pairs) {
    const auto edges = std::set<int>(pair.edges.begin(), pair.edges.end());
    const auto nodes = std::set<int>(pair.nodes.begin(), pair.nodes.end());
    if (!result.pairs[pair.id].nodes.contains(pair.source) ||
        !result.pairs[pair.id].nodes.contains(pair.sink))
      return false;
    for (int edge_id : result.pairs[pair.id].edges) {
      const auto &edge = ug.edges[edge_id];
      if (!edges.contains(edge_id) ||
          !result.pairs[pair.id].nodes.contains(edge.u) ||
          !result.pairs[pair.id].nodes.contains(edge.v))
        return false;
    }
    auto seen = std::set<int>{pair.source};
    auto q = std::queue<int>{};
    q.push(pair.source);
    while (!q.empty()) {
      const int node = q.front();
      q.pop();
      for (int edge_id : ug.incident_edge_ids[node])
        if (result.pairs[pair.id].edges.contains(edge_id)) {
          const int next = other_end(ug.edges[edge_id], node);
          if (seen.insert(next).second)
            q.push(next);
        }
    }
    if (!seen.contains(pair.sink))
      return false;
    for (int node : result.pairs[pair.id].nodes)
      if (!nodes.contains(node))
        return false;
  }
  return true;
}

auto extract(const UnifiedGraph &g, const UndirectedGraph &ug,
             const Prepared &p, const Result &result,
             const SatRoutingResult &sat) -> SatRoutingResult {
  auto out = sat;
  out.paths.erase(std::remove_if(out.paths.begin(), out.paths.end(),
                                 [&](const auto &path) {
                                   return p.selected.contains(path.net_id);
                                 }),
                  out.paths.end());
  for (const auto &pair : p.pairs) {
    auto predecessor = std::map<int, int>{};
    auto seen = std::set<int>{pair.source};
    auto q = std::queue<int>{};
    q.push(pair.source);
    while (!q.empty()) {
      const int node = q.front();
      q.pop();
      for (int edge_id : ug.incident_edge_ids[node])
        if (result.pairs[pair.id].edges.contains(edge_id)) {
          const int next = other_end(ug.edges[edge_id], node);
          if (seen.insert(next).second) {
            predecessor[next] = edge_id;
            q.push(next);
          }
        }
    }
    auto path = std::Vector<int>{pair.sink};
    for (int node = pair.sink; node != pair.source;) {
      const int edge_id = predecessor.at(node);
      node = other_end(ug.edges[edge_id], node);
      path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
    out.paths.push_back({pair.net_id, pair.source_index, pair.demand,
                         pair.physical_source, std::move(path)});
  }
  auto switches = std::set<int>{}, used_modes = std::set<int>{};
  for (const auto &path : out.paths)
    for (int edge_id : path_edges(ug, path.node_path)) {
      const auto &edge = ug.edges[edge_id];
      if (edge.physical_switch_id >= 0) {
        switches.insert(edge.physical_switch_id);
        if (edge.mode_group_id >= 0)
          used_modes.insert(edge.mode_group_id);
      }
    }
  for (const auto [group, mode] : result.modes)
    if (used_modes.contains(group))
      out.vline_mode_straight_by_group[group] = mode;
  out.used_tob_switch_ids.assign(switches.begin(), switches.end());
  out.total_wirelength = total_wirelength(g, out);
  return out;
}

auto validate_routes(const UnifiedGraph &g, const UndirectedGraph &ug,
                     const std::Vector<RoutingNet> &nets, const Prepared &p,
                     const SatRoutingResult &baseline,
                     const SatRoutingResult &result, double objective)
    -> std::Vector<std::String> {
  auto errors = std::Vector<std::String>{};
  auto by_key = std::map<std::tuple<std::size_t, std::size_t, std::size_t>,
                         const Pair *>{};
  for (const auto &pair : p.pairs)
    by_key[{pair.net_id, pair.demand, pair.source_index}] = &pair;
  using ResourceOwner = std::tuple<bool, std::size_t, std::size_t>;
  auto node_owner = std::map<int, ResourceOwner>{};
  auto edge_owner = std::map<int, ResourceOwner>{};
  auto switch_owner = std::map<int, ResourceOwner>{};
  auto used_edges = std::set<int>{};
  auto counted_matching_switches = std::set<int>{};
  auto matching = std::map<std::pair<int, int>, int>{};
  for (const auto &path : result.paths) {
    const auto *net = net_for(nets, path.net_id);
    if (net == nullptr) {
      errors.push_back("unknown net");
      continue;
    }
    const auto key = std::tuple{path.net_id, path.demand_id, path.source_index};
    const auto *pair = by_key.contains(key) ? by_key.at(key) : nullptr;
    if (pair != nullptr) {
      for (int v : path.node_path)
        if (std::ranges::find(pair->nodes, v) == pair->nodes.end())
          errors.push_back("pair path leaves local node scope");
    }
    const auto owner = net->is_sync_bus
                           ? ResourceOwner{true, path.net_id, path.demand_id}
                           : ResourceOwner{false, path.net_id, 0};
    for (int v : path.node_path)
      if (physical(g, v)) {
        const auto [it, ok] = node_owner.emplace(v, owner);
        if (!ok && it->second != owner)
          errors.push_back("physical node multiple net owners");
      }
    for (int edge_id : path_edges(ug, path.node_path)) {
      const auto &edge = ug.edges[edge_id];
      if (pair != nullptr &&
          std::ranges::find(pair->edges, edge_id) == pair->edges.end())
        errors.push_back("pair path leaves local edge scope");
      const auto [edge_it, edge_ok] = edge_owner.emplace(edge_id, owner);
      if (!edge_ok && edge_it->second != owner)
        errors.push_back("physical edge multiple net owners");
      if (!net->is_sync_bus)
        used_edges.insert(edge_id);
      if (edge.physical_switch_id >= 0) {
        const auto [it, ok] =
            switch_owner.emplace(edge.physical_switch_id, owner);
        if (!ok && it->second != owner)
          errors.push_back("physical switch multiple net owners");
        if (counted_matching_switches.insert(edge.physical_switch_id).second)
          for (const auto endpoint : matching_endpoints(g, edge))
            ++matching[endpoint];
      }
      if (edge.mode_group_id >= 0) {
        const auto mode =
            result.vline_mode_straight_by_group.find(edge.mode_group_id);
        if (mode == result.vline_mode_straight_by_group.end() ||
            mode->second != edge.is_straight)
          errors.push_back("straight/swap mode conflict");
      }
    }
  }
  for (const auto &[_, count] : matching)
    if (count > 1)
      errors.push_back("partial matching conflict");
  for (const auto &net : nets)
    for (const auto &demand : net.demands) {
      auto count = 0;
      for (const auto &path : result.paths)
        if (path.net_id == net.net_id && path.demand_id == demand.demand_id)
          ++count;
      if (count != 1)
        errors.push_back("missing/duplicate demand path");
    }
  for (const auto &path : baseline.paths) {
    const auto *net = net_for(nets, path.net_id);
    const auto it = std::find_if(
        result.paths.begin(), result.paths.end(), [&](const auto &candidate) {
          return candidate.net_id == path.net_id &&
                 candidate.demand_id == path.demand_id &&
                 candidate.source_index == path.source_index;
        });
    if (it == result.paths.end()) {
      errors.push_back("baseline pair missing");
      continue;
    }
    if ((net != nullptr && net->is_sync_bus &&
         it->node_path != path.node_path) ||
        it->physical_source_node != path.physical_source_node)
      errors.push_back("fixed path/source changed");
  }
  if (std::abs(static_cast<double>(used_edges.size()) - objective) > .5)
    errors.push_back("physical-edge union objective mismatch");
  return errors;
}

auto stamp(SatRoutingResult &result, const Stats &stats) -> void {
  result.post_sat_ilp_build_ms = stats.build;
  result.post_sat_ilp_solve_ms = stats.solve;
  result.post_sat_ilp_variables = stats.f + stats.d + stats.y + stats.m;
  result.post_sat_ilp_constraints = stats.rows;
  result.post_sat_ilp_objective = stats.objective;
  result.post_sat_ilp_bound = stats.bound;
  result.post_sat_ilp_gap = stats.gap;
}
} // namespace

auto is_post_sat_ilp_target(const RoutingNet &net) -> bool {
  return !net.is_sync_bus;
}

auto optimize_post_sat_routes(const UnifiedGraph &g,
                              const std::Vector<RoutingNet> &nets,
                              const std::Vector<UnifiedSatNetScope> &scopes,
                              const SatRoutingResult &sat,
                              const PostSatIlpOptions &options,
                              const RoutingProblemState *state)
    -> SatRoutingResult {
  auto fallback = sat;
  fallback.post_sat_ilp_attempted = true;
  fallback.post_sat_ilp_baseline_wirelength = sat.total_wirelength;
  fallback.post_sat_ilp_wirelength = sat.total_wirelength;
  const auto begin = std::chrono::steady_clock::now();
  const auto finish = [&](SatRoutingResult &result) {
    result.post_sat_ilp_total_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - begin)
            .count();
  };
  try {
    const auto ug = make_undirected_graph(g);
    auto [prepared, locked] = prepare(g, ug, nets, scopes, sat, state);
    fallback.post_sat_ilp_parents = prepared.nets.size();
    fallback.post_sat_ilp_segments = prepared.pairs.size();
    fallback.post_sat_ilp_components = prepared.components.size();
    if (prepared.nets.empty()) {
      fallback.post_sat_ilp_status = "SKIPPED_NO_NONBUS_NETS";
      finish(fallback);
      return fallback;
    }
    std::size_t node_slots = 0, edge_slots = 0;
    for (const auto &pair : prepared.pairs) {
      node_slots += pair.nodes.size();
      edge_slots += pair.edges.size();
    }
    debug::info_fmt(
        "V22 undirected post-SAT ILP prepare: nonbus_nets={} pairs={} "
        "components={} fixed_bus_nets={} pair_local_node_slots={} "
        "pair_local_edge_slots={} physical_edges={} locked_nodes={} "
        "locked_switches={}",
        prepared.nets.size(), prepared.pairs.size(), prepared.components.size(),
        locked.nets.size(), node_slots, edge_slots, ug.edges.size(),
        std::count(locked.nodes.begin(), locked.nodes.end(), true),
        locked.switches.size());

    auto all = Result{};
    all.pairs.resize(prepared.pairs.size());
    for (std::size_t i = 0; i < prepared.components.size(); ++i) {
      const auto &component = prepared.components[i];
      const auto result =
          solve_component(g, ug, prepared, locked, component, sat, options);
      all.stats.f += result.stats.f;
      all.stats.d += result.stats.d;
      all.stats.y += result.stats.y;
      all.stats.m += result.stats.m;
      all.stats.rows += result.stats.rows;
      all.stats.nz += result.stats.nz;
      all.stats.start += result.stats.start;
      all.stats.endpoint_degree += result.stats.endpoint_degree;
      all.stats.internal_degree += result.stats.internal_degree;
      all.stats.f_implies_d += result.stats.f_implies_d;
      all.stats.f_implies_y += result.stats.f_implies_y;
      all.stats.edge_pair_exclusivity += result.stats.edge_pair_exclusivity;
      all.stats.node_pair_exclusivity += result.stats.node_pair_exclusivity;
      all.stats.tob_matching += result.stats.tob_matching;
      all.stats.mode_binding += result.stats.mode_binding;
      all.stats.build += result.stats.build;
      all.stats.solve += result.stats.solve;
      all.stats.objective += result.stats.objective;
      all.stats.bound += result.stats.bound;
      all.stats.gap = std::max(all.stats.gap, result.stats.gap);
      if (!result.ok) {
        fallback.post_sat_ilp_status =
            std::format("COMPONENT_{}_{}", i, result.status);
        stamp(fallback, all.stats);
        finish(fallback);
        return fallback;
      }
      for (const auto &pair : prepared.pairs)
        if (std::ranges::find(component, pair.net) != component.end())
          all.pairs[pair.id] = result.pairs[pair.id];
      all.modes.insert(result.modes.begin(), result.modes.end());
      all.status = result.status;
    }
    if (!valid(ug, prepared, all)) {
      fallback.post_sat_ilp_status = "MODEL_VALIDATION_FAILED";
      stamp(fallback, all.stats);
      finish(fallback);
      return fallback;
    }
    auto candidate = extract(g, ug, prepared, all, sat);
    const auto errors = validate_routes(g, ug, nets, prepared, sat, candidate,
                                        all.stats.objective);
    if (!errors.empty()) {
      fallback.post_sat_ilp_status = "ROUTE_VALIDATION_FAILED";
      stamp(fallback, all.stats);
      finish(fallback);
      for (const auto &error : errors)
        debug::warning_fmt("  V22 route validation: {}", error);
      return fallback;
    }
    if (candidate.total_wirelength > sat.total_wirelength) {
      fallback.post_sat_ilp_status = "DEGRADED_INCUMBENT_REJECTED";
      stamp(fallback, all.stats);
      finish(fallback);
      return fallback;
    }
    candidate.post_sat_ilp_attempted = true;
    candidate.post_sat_ilp_accepted = true;
    candidate.post_sat_ilp_status = all.status;
    candidate.post_sat_ilp_baseline_wirelength = sat.total_wirelength;
    candidate.post_sat_ilp_wirelength = candidate.total_wirelength;
    candidate.post_sat_ilp_parents = prepared.nets.size();
    candidate.post_sat_ilp_segments = prepared.pairs.size();
    candidate.post_sat_ilp_components = prepared.components.size();
    stamp(candidate, all.stats);
    finish(candidate);
    debug::info_fmt(
        "V22 undirected post-SAT ILP accepted: components={} pairs={} "
        "physical_edges={:.0f} track_bump_wirelength={}->{} "
        "build_ms={} solve_ms={}",
        prepared.components.size(), prepared.pairs.size(), all.stats.objective,
        sat.total_wirelength, candidate.total_wirelength,
        candidate.post_sat_ilp_build_ms, candidate.post_sat_ilp_solve_ms);
    return candidate;
  } catch (const std::exception &error) {
    fallback.post_sat_ilp_status = std::format("EXCEPTION: {}", error.what());
    finish(fallback);
    debug::warning_fmt("V22 ILP fallback: {}", error.what());
    return fallback;
  }
}
} // namespace PR_tool
