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
#include <numeric>
#include <queue>
#include <set>
#include <stdexcept>

#ifdef USE_HIGHS
#include <Highs.h>
#endif

namespace PR_tool {
namespace {
constexpr double kMipRelativeGap = 0.015;

struct Pair {
    std::size_t id{}, net{}, net_id{}, demand{}, source_index{};
    int source{-1}, sink{-1}, physical_source{-1};
    std::uint16_t unit{0xffff};
    std::Vector<int> nodes, arcs;
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
};
struct PairVars {
    std::map<int, int> f, d;
};
struct NetVars {
    std::map<int, int> y, s;
};
struct Stats {
    std::size_t f{}, d{}, y{}, s{}, m{}, rows{}, nz{}, start{};
    std::size_t f_implies_d{}, f_implies_s{}, flow_balance{},
        source_no_incoming{}, node_incoming{}, sink_no_outgoing{},
        endpoint_d_fixed{}, d_implies_y{}, y_support{}, s_support{},
        node_exclusivity{}, switch_exclusivity{}, tob_matching{},
        mode_binding{};
    long long build{}, solve{};
    double objective{}, bound{}, gap{};
};
struct PairSolution {
    std::set<int> arcs, nodes;
};
struct Result {
    bool ok{};
    std::String status;
    Stats stats;
    std::Vector<PairSolution> pairs;
    std::map<int, bool> modes;
};

auto log_post_sat_ilp_model_stats(const Prepared &p, const Locked &locked,
                                  const std::Vector<std::size_t> &component,
                                  const Stats &stats) -> void {
    std::size_t node_slots = 0, arc_slots = 0;
    std::size_t pairs = 0;
    for (const auto net_id : component)
        for (const auto pair_id : p.nets[net_id].pairs) {
            const auto &pair = p.pairs[pair_id];
            node_slots += pair.nodes.size();
            arc_slots += pair.arcs.size();
            ++pairs;
        }
    const auto listed_rows =
        stats.f_implies_d + stats.f_implies_s + stats.flow_balance +
        stats.source_no_incoming + stats.node_incoming +
        stats.sink_no_outgoing + stats.endpoint_d_fixed + stats.d_implies_y +
        stats.y_support + stats.s_support + stats.node_exclusivity +
        stats.switch_exclusivity + stats.tob_matching + stats.mode_binding;
    debug::info("========== V22 post-SAT ILP component model stats (-v) ==========");
    debug::info("Model dimensions:");
    debug::info_fmt("  nets in this component          : {}", component.size());
    debug::info_fmt("  non-SyncBus nets (all)          : {}", p.nets.size());
    debug::info_fmt("  fixed SyncBus nets              : {}", locked.nets.size());
    debug::info_fmt("  source/sink pairs               : {}", pairs);
    debug::info_fmt("  interaction components (all)    : {}", p.components.size());
    debug::info_fmt("  pair-local node slots           : {}", node_slots);
    debug::info_fmt("  pair-local directed arc slots   : {}", arc_slots);
    debug::info_fmt("  locked physical nodes           : {}",
                    std::count(locked.nodes.begin(), locked.nodes.end(), true));
    debug::info_fmt("  locked physical switches        : {}", locked.switches.size());

    debug::info("Binary variables:");
    debug::info_fmt("  F   (pair directed flow)        : {}", stats.f);
    debug::info_fmt("  D   (pair node use)             : {}", stats.d);
    debug::info_fmt("  Y   (net physical node union)   : {}", stats.y);
    debug::info_fmt("  S   (net physical switch union) : {}", stats.s);
    debug::info_fmt("  M   (VLine--Track mode)         : {}", stats.m);
    debug::info_fmt("  total MIP variables             : {}",
                    stats.f + stats.d + stats.y + stats.s + stats.m);
    debug::info_fmt("  MIP-start entries               : {}", stats.start);

    debug::info("Linear constraints by category:");
    debug::info_fmt("  [1]  F implies endpoint D       : {}", stats.f_implies_d);
    debug::info_fmt("  [2]  F implies net switch S    : {}", stats.f_implies_s);
    debug::info_fmt("  [3]  pair flow balance          : {}", stats.flow_balance);
    debug::info_fmt("  [4]  source has no incoming F  : {}", stats.source_no_incoming);
    debug::info_fmt("  [5]  D equals incoming F        : {}", stats.node_incoming);
    debug::info_fmt("  [6]  sink has no outgoing F    : {}", stats.sink_no_outgoing);
    debug::info_fmt("  [7]  source/sink D fixed       : {}", stats.endpoint_d_fixed);
    debug::info_fmt("  [8]  pair D implies net Y       : {}", stats.d_implies_y);
    debug::info_fmt("  [9]  net Y support              : {}", stats.y_support);
    debug::info_fmt("  [10] net S support              : {}", stats.s_support);
    debug::info_fmt("  [11] cross-net node exclusivity : {}", stats.node_exclusivity);
    debug::info_fmt("  [12] cross-net switch exclusive : {}", stats.switch_exclusivity);
    debug::info_fmt("  [13] TOB partial matching       : {}", stats.tob_matching);
    debug::info_fmt("  [14] straight/swap mode binding : {}", stats.mode_binding);
    debug::info_fmt("  listed / total MIP constraints  : {} / {}", listed_rows,
                    stats.rows);
    debug::info("==========================================================");
}

auto physical(const UnifiedGraph &g, int v) -> bool {
    return v >= 0 && static_cast<std::size_t>(v) < g.nodes.size() &&
           g.nodes[v].kind != UnifiedNodeKind::VirtualSource;
}
auto length_node(const UnifiedGraph &g, int v) -> bool {
    return physical(g, v) && (g.nodes[v].kind == UnifiedNodeKind::Track ||
                              g.nodes[v].kind == UnifiedNodeKind::Bump);
}
auto find_arc(const UnifiedGraph &g, int u, int v) -> int {
    if (u < 0 || static_cast<std::size_t>(u) >= g.out_arc_ids.size())
        return -1;
    for (int a : g.out_arc_ids[u])
        if (g.arcs[a].v == v)
            return a;
    return -1;
}
auto path_arcs(const UnifiedGraph &g, const std::Vector<int> &path)
    -> std::Vector<int> {
    auto out = std::Vector<int>{};
    for (std::size_t i = 1; i < path.size(); ++i) {
        const int a = find_arc(g, path[i - 1], path[i]);
        if (a < 0)
            throw std::logic_error("V22 path lacks graph arc");
        out.push_back(a);
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
auto endpoints(const UnifiedGraph &g, const UnifiedArc &a)
    -> std::Vector<std::pair<int, int>> {
    if (a.physical_switch_id < 0)
        return {};
    const auto u = g.nodes[a.u].kind, v = g.nodes[a.v].kind;
    if (u == UnifiedNodeKind::Bump && v == UnifiedNodeKind::HLine)
        return {{0, a.u}, {1, a.v}};
    if (u == UnifiedNodeKind::HLine && v == UnifiedNodeKind::Bump)
        return {{0, a.v}, {1, a.u}};
    if (u == UnifiedNodeKind::HLine && v == UnifiedNodeKind::VLine)
        return {{2, a.u}, {3, a.v}};
    if (u == UnifiedNodeKind::VLine && v == UnifiedNodeKind::HLine)
        return {{2, a.v}, {3, a.u}};
    return {};
}

auto locked_resources(const UnifiedGraph &g,
                      const std::Vector<RoutingNet> &nets,
                      const SatRoutingResult &sat) -> Locked {
    auto out = Locked{};
    out.nodes.assign(g.nodes.size(), false);
    for (const auto &path : sat.paths) {
        const auto *net = net_for(nets, path.net_id);
        if (net == nullptr || !net->is_sync_bus)
            continue;
        out.nets.insert(path.net_id);
        for (int v : path.node_path)
            if (physical(g, v))
                out.nodes[v] = true;
        for (int a : path_arcs(g, path.node_path)) {
            const auto &e = g.arcs[a];
            if (e.physical_switch_id >= 0) {
                out.switches.insert(e.physical_switch_id);
                for (auto key : endpoints(g, e))
                    ++out.matching[key];
            }
            if (e.mode_group_id >= 0) {
                const auto [it, ok] = out.modes.emplace(
                    e.mode_group_id, e.is_vline_track_straight);
                if (!ok && it->second != e.is_vline_track_straight)
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
        throw std::logic_error(
            "V22 incumbent does not determine a unique COBUnit");
    return unit_bit(*units.begin());
}

// This is deliberately a pair projection, not a reconstruction from the
// union scope.  The final incumbent is then injected so the warm start remains
// legal even though SAT originally encoded the net-wide union scope.
auto make_pair_domain(const UnifiedGraph &g,
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
        const bool sink_is_tob =
            g.nodes[pair.sink].kind != UnifiedNodeKind::Track;
        for (int v : fallback_scope.node_ids) {
            const auto &node = g.nodes[v];
            if (!physical(g, v) || !node_unit_eligible(node, pair.unit))
                continue;
            if (node.kind == UnifiedNodeKind::Track)
                allowed[v] =
                    state->allowed_channels.contains(GlobalChannelCoord{
                        node.track_dir, node.track_row, node.track_col});
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
    for (int a : fallback_scope.arc_ids) {
        const auto &e = g.arcs[a];
        if (!allowed[e.u] || !allowed[e.v] ||
            !arc_unit_eligible(g, e, pair.unit))
            continue;
        if (locked.nodes[e.u] || locked.nodes[e.v])
            continue;
        if (e.physical_switch_id >= 0 &&
            locked.switches.contains(e.physical_switch_id))
            continue;
        if (e.mode_group_id >= 0 && locked.modes.contains(e.mode_group_id) &&
            locked.modes.at(e.mode_group_id) != e.is_vline_track_straight)
            continue;
        candidates.insert(a);
    }
    for (int a : path_arcs(g, incumbent.node_path)) {
        const auto &e = g.arcs[a];
        if (locked.nodes[e.u] || locked.nodes[e.v] ||
            (e.physical_switch_id >= 0 &&
             locked.switches.contains(e.physical_switch_id)))
            throw std::logic_error("V22 incumbent conflicts with fixed bus");
        candidates.insert(a);
        pair.incumbent.insert(a);
    }
    auto forward = std::Vector<std::Vector<int>>(g.nodes.size()),
         reverse = forward;
    for (int a : candidates) {
        forward[g.arcs[a].u].push_back(a);
        reverse[g.arcs[a].v].push_back(a);
    }
    auto from = std::Vector<bool>(g.nodes.size()), to = from;
    auto q = std::queue<int>{};
    from[pair.source] = true;
    q.push(pair.source);
    while (!q.empty()) {
        const int v = q.front();
        q.pop();
        for (int a : forward[v])
            if (!from[g.arcs[a].v]) {
                from[g.arcs[a].v] = true;
                q.push(g.arcs[a].v);
            }
    }
    if (!from[pair.sink])
        throw std::logic_error("V22 pair-local scope disconnects sink");
    to[pair.sink] = true;
    q.push(pair.sink);
    while (!q.empty()) {
        const int v = q.front();
        q.pop();
        for (int a : reverse[v])
            if (!to[g.arcs[a].u]) {
                to[g.arcs[a].u] = true;
                q.push(g.arcs[a].u);
            }
    }
    auto nodes = std::set<int>{pair.source, pair.sink};
    for (int a : candidates)
        if (from[g.arcs[a].u] && to[g.arcs[a].v]) {
            pair.arcs.push_back(a);
            nodes.insert(g.arcs[a].u);
            nodes.insert(g.arcs[a].v);
        }
    pair.nodes.assign(nodes.begin(), nodes.end());
    const auto active = std::set<int>(pair.arcs.begin(), pair.arcs.end());
    for (int a : pair.incumbent)
        if (!active.contains(a))
            throw std::logic_error("V22 pair domain dropped incumbent arc");
}
auto prepare(const UnifiedGraph &g, const std::Vector<RoutingNet> &routing_nets,
             const std::Vector<UnifiedSatNetScope> &scopes,
             const SatRoutingResult &sat, const RoutingProblemState *state)
    -> std::pair<Prepared, Locked> {
    auto prepared = Prepared{};
    for (const auto &net : routing_nets)
        if (is_post_sat_ilp_target(net))
            prepared.selected.insert(net.net_id);
    const auto locked = locked_resources(g, routing_nets, sat);
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
                    resolve_graph_node(
                        g, routing_net.sources[path->source_index]) ||
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
            make_pair_domain(g, *scope, local, locked, pair, *path);
            net.pairs.push_back(pair.id);
            prepared.pairs.push_back(std::move(pair));
        }
        prepared.nets.push_back(std::move(net));
    }
    auto owner = std::map<std::tuple<int, int, int>, std::size_t>{};
    auto adj = std::Vector<std::set<std::size_t>>(prepared.nets.size());
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
        for (int a : pair.arcs) {
            const auto &e = g.arcs[a];
            if (e.physical_switch_id >= 0)
                record({1, e.physical_switch_id, 0}, pair.net);
            if (e.mode_group_id >= 0)
                record({2, e.mode_group_id, 0}, pair.net);
            for (const auto [stage, v] : endpoints(g, e))
                record({3, stage, v}, pair.net);
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
                for (auto v : adj[n])
                    if (!seen[v]) {
                        seen[v] = true;
                        q.push(v);
                    }
            }
            prepared.components.push_back(std::move(component));
        }
    return {std::move(prepared), locked};
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
    auto row(double lo, double hi,
             const std::Vector<std::pair<int, double>> &in) -> void {
        auto sums = std::map<int, double>{};
        for (const auto [x, v] : in)
            sums[x] += v;
        auto ids = std::Vector<HighsInt>{};
        auto values = std::Vector<double>{};
        for (const auto [x, v] : sums)
            if (v != 0) {
                ids.push_back(x);
                values.push_back(v);
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
    static auto check(HighsStatus s) -> void {
        if (s == HighsStatus::kError)
            throw std::runtime_error("V22 HiGHS API failure");
    }
    HighsLogSink log_;
    Highs h_;
    std::size_t vars_{}, rows_{}, nz_{};
};
auto solve_component(const UnifiedGraph &g, const Prepared &p,
                     const Locked &locked, const std::Vector<std::size_t> &ids,
                     const SatRoutingResult &sat,
                     const PostSatIlpOptions &options) -> Result {
    const auto begin = std::chrono::steady_clock::now();
    auto result = Result{};
    auto mip = Mip(options.verbose_level, options.highs_log_path,
                   options.highs_time_limit_minutes);
    auto pv = std::map<std::size_t, PairVars>{};
    auto nv = std::map<std::size_t, NetVars>{};
    for (auto net_id : ids) {
        const auto &net = p.nets[net_id];
        auto &v = nv[net_id];
        auto nodes = std::set<int>{};
        auto switches = std::set<int>{};
        for (auto pair_id : net.pairs) {
            const auto &pair = p.pairs[pair_id];
            for (int a : pair.arcs) {
                pv[pair_id].f.emplace(a, mip.bin());
                ++result.stats.f;
                nodes.insert(g.arcs[a].u);
                nodes.insert(g.arcs[a].v);
                if (g.arcs[a].physical_switch_id >= 0)
                    switches.insert(g.arcs[a].physical_switch_id);
            }
            for (int node : pair.nodes) {
                pv[pair_id].d.emplace(node, mip.bin());
                ++result.stats.d;
                nodes.insert(node);
            }
        }
        for (int node : nodes) {
            v.y.emplace(node, mip.bin(length_node(g, node) ? 1. : 0.));
            ++result.stats.y;
        }
        for (int sw : switches) {
            v.s.emplace(sw, mip.bin());
            ++result.stats.s;
        }
    }
    for (auto net_id : ids) {
        const auto &net = p.nets[net_id];
        const auto &nvars = nv.at(net_id);
        for (auto pair_id : net.pairs) {
            const auto &pair = p.pairs[pair_id];
            const auto &vars = pv.at(pair_id);
            for (int a : pair.arcs) {
                const auto &e = g.arcs[a];
                mip.row(-kHighsInf, 0,
                        {{vars.f.at(a), 1}, {vars.d.at(e.u), -1}});
                ++result.stats.f_implies_d;
                mip.row(-kHighsInf, 0,
                        {{vars.f.at(a), 1}, {vars.d.at(e.v), -1}});
                ++result.stats.f_implies_d;
                if (e.physical_switch_id >= 0) {
                    mip.row(-kHighsInf, 0,
                            {{vars.f.at(a), 1},
                             {nvars.s.at(e.physical_switch_id), -1}});
                    ++result.stats.f_implies_s;
                }
            }
            for (int node : pair.nodes) {
                auto conservation = std::Vector<std::pair<int, double>>{},
                     incoming = conservation;
                for (int a : g.out_arc_ids[node])
                    if (vars.f.contains(a))
                        conservation.emplace_back(vars.f.at(a), 1);
                for (int a : g.in_arc_ids[node])
                    if (vars.f.contains(a)) {
                        conservation.emplace_back(vars.f.at(a), -1);
                        incoming.emplace_back(vars.f.at(a), 1);
                    }
                mip.row(node == pair.source ? 1.
                        : node == pair.sink ? -1.
                                            : 0.,
                        node == pair.source ? 1.
                        : node == pair.sink ? -1.
                                            : 0.,
                        conservation);
                ++result.stats.flow_balance;
                if (node == pair.source) {
                    mip.row(0, 0, incoming);
                    ++result.stats.source_no_incoming;
                } else {
                    incoming.emplace_back(vars.d.at(node), -1);
                    mip.row(0, 0, incoming);
                    ++result.stats.node_incoming;
                }
                if (node == pair.sink) {
                    auto outgoing = std::Vector<std::pair<int, double>>{};
                    for (int a : g.out_arc_ids[node])
                        if (vars.f.contains(a))
                            outgoing.emplace_back(vars.f.at(a), 1);
                    mip.row(0, 0, outgoing);
                    ++result.stats.sink_no_outgoing;
                }
                if (node == pair.source || node == pair.sink) {
                    mip.row(1, 1, {{vars.d.at(node), 1}});
                    ++result.stats.endpoint_d_fixed;
                }
                mip.row(-kHighsInf, 0,
                        {{vars.d.at(node), 1}, {nvars.y.at(node), -1}});
                ++result.stats.d_implies_y;
            }
        }
        for (const auto [node, y] : nvars.y) {
            auto support = std::Vector<std::pair<int, double>>{{y, 1}};
            for (auto pair_id : net.pairs)
                if (pv.at(pair_id).d.contains(node))
                    support.emplace_back(pv.at(pair_id).d.at(node), -1);
            mip.row(-kHighsInf, 0, support);
            ++result.stats.y_support;
        }
        for (const auto [sw, s] : nvars.s) {
            auto support = std::Vector<std::pair<int, double>>{{s, 1}};
            for (auto pair_id : net.pairs)
                for (int a : p.pairs[pair_id].arcs)
                    if (g.arcs[a].physical_switch_id == sw)
                        support.emplace_back(pv.at(pair_id).f.at(a), -1);
            mip.row(-kHighsInf, 0, support);
            ++result.stats.s_support;
        }
    }
    auto occupancy = std::map<int, std::Vector<int>>{};
    auto switches = std::map<int, std::Vector<int>>{};
    auto matching = std::map<std::pair<int, int>, std::set<int>>{};
    auto modes = std::map<int, std::set<std::pair<int, bool>>>{};
    for (auto net_id : ids) {
        const auto &net = p.nets[net_id];
        const auto &vars = nv.at(net_id);
        for (const auto [node, y] : vars.y)
            occupancy[node].push_back(y);
        for (const auto [sw, s] : vars.s) {
            switches[sw].push_back(s);
            for (auto pair_id : net.pairs)
                for (int a : p.pairs[pair_id].arcs)
                    if (g.arcs[a].physical_switch_id == sw) {
                        for (auto key : endpoints(g, g.arcs[a]))
                            matching[key].insert(s);
                        if (g.arcs[a].mode_group_id >= 0)
                            modes[g.arcs[a].mode_group_id].insert(
                                {s, g.arcs[a].is_vline_track_straight});
                    }
        }
    }
    for (const auto &[_, xs] : occupancy)
        if (xs.size() > 1) {
            auto row = std::Vector<std::pair<int, double>>{};
            for (int x : xs)
                row.emplace_back(x, 1);
            mip.row(-kHighsInf, 1, row);
            ++result.stats.node_exclusivity;
        }
    for (const auto &[_, xs] : switches)
        if (xs.size() > 1) {
            auto row = std::Vector<std::pair<int, double>>{};
            for (int x : xs)
                row.emplace_back(x, 1);
            mip.row(-kHighsInf, 1, row);
            ++result.stats.switch_exclusivity;
        }
    for (const auto &[key, xs] : matching) {
        auto row = std::Vector<std::pair<int, double>>{};
        for (int x : xs)
            row.emplace_back(x, 1);
        mip.row(
            -kHighsInf,
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
        for (const auto [s, straight] : uses) {
            if (straight)
                mip.row(-kHighsInf, 0, {{s, 1}, {mvars.at(group), -1}});
            else
                mip.row(-kHighsInf, 1, {{s, 1}, {mvars.at(group), 1}});
            ++result.stats.mode_binding;
        }
    }
    auto start = std::map<int, double>{};
    for (int v = 0; v < static_cast<int>(mip.vars()); ++v)
        start[v] = 0.;
    for (auto net_id : ids)
        for (auto pair_id : p.nets[net_id].pairs) {
            const auto &pair = p.pairs[pair_id];
            for (int a : pair.incumbent) {
                start[pv.at(pair_id).f.at(a)] = 1;
                const auto &e = g.arcs[a];
                start[pv.at(pair_id).d.at(e.u)] =
                    start[pv.at(pair_id).d.at(e.v)] = 1;
                start[nv.at(net_id).y.at(e.u)] =
                    start[nv.at(net_id).y.at(e.v)] = 1;
                if (e.physical_switch_id >= 0)
                    start[nv.at(net_id).s.at(e.physical_switch_id)] = 1;
            }
        }
    for (const auto [group, m] : mvars)
        if (sat.vline_mode_straight_by_group.contains(group))
            start[m] = sat.vline_mode_straight_by_group.at(group);
    mip.start(start);
    result.stats.start = start.size();
    result.stats.build = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - begin)
                             .count();
    result.stats.rows = mip.rows();
    result.stats.nz = mip.nz();
    std::size_t pairs = 0;
    for (auto net : ids)
        pairs += p.nets[net].pairs.size();
    if (options.verbose_level >= 1)
        log_post_sat_ilp_model_stats(p, locked, ids, result.stats);
    debug::info_fmt("V22 ILP component model: nets={} pairs={} vars={} "
                    "constraints={} F={} D={} Y={} S={} M={} warm_start={}",
                    ids.size(), pairs, mip.vars(), mip.rows(), result.stats.f,
                    result.stats.d, result.stats.y, result.stats.s,
                    result.stats.m, result.stats.start);
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
            for (const auto [a, v] : pv.at(pair_id).f)
                if (values[v] > .5)
                    result.pairs[pair_id].arcs.insert(a);
            for (const auto [node, v] : pv.at(pair_id).d)
                if (values[v] > .5)
                    result.pairs[pair_id].nodes.insert(node);
        }
    for (const auto [group, m] : mvars)
        result.modes[group] = values[m] > .5;
    result.stats.objective = mip.info().objective_function_value;
    result.stats.bound = mip.info().mip_dual_bound;
    result.stats.gap = mip.info().mip_gap;
    result.ok = true;
    return result;
}
#else
auto solve_component(const UnifiedGraph &, const Prepared &, const Locked &,
                     const std::Vector<std::size_t> &, const SatRoutingResult &,
                     const PostSatIlpOptions &) -> Result {
    auto r = Result{};
    r.status = "HIGHS_UNAVAILABLE";
    return r;
}
#endif

auto valid(const UnifiedGraph &g, const Prepared &p, const Result &r) -> bool {
    if (r.pairs.size() != p.pairs.size())
        return false;
    for (const auto &pair : p.pairs) {
        const auto arcs = std::set<int>(pair.arcs.begin(), pair.arcs.end());
        const auto nodes = std::set<int>(pair.nodes.begin(), pair.nodes.end());
        if (!r.pairs[pair.id].nodes.contains(pair.source) ||
            !r.pairs[pair.id].nodes.contains(pair.sink))
            return false;
        for (int a : r.pairs[pair.id].arcs)
            if (!arcs.contains(a) ||
                !r.pairs[pair.id].nodes.contains(g.arcs[a].u) ||
                !r.pairs[pair.id].nodes.contains(g.arcs[a].v))
                return false;
        auto seen = std::set<int>{pair.source};
        auto q = std::queue<int>{};
        q.push(pair.source);
        while (!q.empty()) {
            const int v = q.front();
            q.pop();
            for (int a : g.out_arc_ids[v])
                if (r.pairs[pair.id].arcs.contains(a) &&
                    seen.insert(g.arcs[a].v).second)
                    q.push(g.arcs[a].v);
        }
        if (!seen.contains(pair.sink))
            return false;
        for (int v : r.pairs[pair.id].nodes)
            if (!nodes.contains(v))
                return false;
    }
    return true;
}
auto extract(const UnifiedGraph &g, const Prepared &p, const Result &r,
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
            const int v = q.front();
            q.pop();
            for (int a : g.out_arc_ids[v])
                if (r.pairs[pair.id].arcs.contains(a) &&
                    seen.insert(g.arcs[a].v).second) {
                    predecessor[g.arcs[a].v] = a;
                    q.push(g.arcs[a].v);
                }
        }
        auto path = std::Vector<int>{pair.sink};
        for (int v = pair.sink; v != pair.source;) {
            const int a = predecessor.at(v);
            v = g.arcs[a].u;
            path.push_back(v);
        }
        std::reverse(path.begin(), path.end());
        out.paths.push_back({pair.net_id, pair.source_index, pair.demand,
                             pair.physical_source, std::move(path)});
    }
    auto switches = std::set<int>{}, used_modes = std::set<int>{};
    for (const auto &path : out.paths)
        for (int a : path_arcs(g, path.node_path))
            if (const auto &e = g.arcs[a]; e.physical_switch_id >= 0) {
                switches.insert(e.physical_switch_id);
                if (e.mode_group_id >= 0)
                    used_modes.insert(e.mode_group_id);
            }
    for (const auto [group, mode] : r.modes)
        if (used_modes.contains(group))
            out.vline_mode_straight_by_group[group] = mode;
    out.used_tob_switch_ids.assign(switches.begin(), switches.end());
    out.total_wirelength = total_wirelength(g, out);
    return out;
}
auto validate_routes(const UnifiedGraph &g, const std::Vector<RoutingNet> &nets,
                     const Prepared &p, const SatRoutingResult &baseline,
                     const SatRoutingResult &result, double objective)
    -> std::Vector<std::String> {
    auto errors = std::Vector<std::String>{};
    auto by_key = std::map<std::tuple<std::size_t, std::size_t, std::size_t>,
                           const Pair *>{};
    for (const auto &pair : p.pairs)
        by_key[{pair.net_id, pair.demand, pair.source_index}] = &pair;
    using ResourceOwner = std::tuple<bool, std::size_t, std::size_t>;
    auto node_owner = std::map<int, ResourceOwner>{};
    auto switch_owner = std::map<int, ResourceOwner>{};
    auto exemplar = std::map<int, const UnifiedArc *>{};
    for (const auto &path : result.paths) {
        const auto *net = net_for(nets, path.net_id);
        if (net == nullptr) {
            errors.push_back("unknown net");
            continue;
        }
        const auto key =
            std::tuple{path.net_id, path.demand_id, path.source_index};
        const auto *pair = by_key.contains(key) ? by_key.at(key) : nullptr;
        if (pair != nullptr) {
            for (int v : path.node_path)
                if (std::ranges::find(pair->nodes, v) == pair->nodes.end())
                    errors.push_back("pair path leaves local node scope");
            for (int a : path_arcs(g, path.node_path))
                if (std::ranges::find(pair->arcs, a) == pair->arcs.end())
                    errors.push_back("pair path leaves local arc scope");
        }
        const auto owner =
            net->is_sync_bus ? ResourceOwner{true, path.net_id, path.demand_id}
                             : ResourceOwner{false, path.net_id, 0};
        for (int v : path.node_path)
            if (physical(g, v)) {
                const auto [it, ok] = node_owner.emplace(v, owner);
                if (!ok && it->second != owner)
                    errors.push_back("physical node multiple net owners");
            }
        for (int a : path_arcs(g, path.node_path)) {
            const auto &e = g.arcs[a];
            if (e.physical_switch_id >= 0) {
                const auto sw = e.physical_switch_id;
                const auto [it, ok] = switch_owner.emplace(sw, owner);
                if (!ok && it->second != owner)
                    errors.push_back("physical switch multiple net owners");
                exemplar.try_emplace(sw, &e);
            }
            if (e.mode_group_id >= 0) {
                const auto mode =
                    result.vline_mode_straight_by_group.find(e.mode_group_id);
                if (mode == result.vline_mode_straight_by_group.end() ||
                    mode->second != e.is_vline_track_straight)
                    errors.push_back("straight/swap mode conflict");
            }
        }
    }
    auto matching = std::map<std::pair<int, int>, int>{};
    for (const auto &[_, arc] : exemplar)
        for (auto key : endpoints(g, *arc))
            ++matching[key];
    for (const auto &[_, count] : matching)
        if (count > 1)
            errors.push_back("partial matching conflict");
    for (const auto &net : nets)
        for (const auto &d : net.demands) {
            auto count = 0;
            for (const auto &path : result.paths)
                if (path.net_id == net.net_id && path.demand_id == d.demand_id)
                    ++count;
            if (count != 1)
                errors.push_back("missing/duplicate demand path");
        }
    for (const auto &path : baseline.paths) {
        const auto *net = net_for(nets, path.net_id);
        const auto it =
            std::find_if(result.paths.begin(), result.paths.end(),
                         [&](const auto &candidate) {
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
    std::size_t union_wl = 0;
    for (const auto &net : p.nets)
        union_wl += net_wirelength(g, paths_for(result, net.net_id));
    if (std::abs(static_cast<double>(union_wl) - objective) > .5)
        errors.push_back("whole-net union objective mismatch");
    return errors;
}
auto stamp(SatRoutingResult &r, const Stats &s) -> void {
    r.post_sat_ilp_build_ms = s.build;
    r.post_sat_ilp_solve_ms = s.solve;
    r.post_sat_ilp_variables = s.f + s.d + s.y + s.s + s.m;
    r.post_sat_ilp_constraints = s.rows;
    r.post_sat_ilp_objective = s.objective;
    r.post_sat_ilp_bound = s.bound;
    r.post_sat_ilp_gap = s.gap;
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
    const auto finish = [&](SatRoutingResult &r) {
        r.post_sat_ilp_total_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - begin)
                .count();
    };
    try {
        auto [p, locked] = prepare(g, nets, scopes, sat, state);
        fallback.post_sat_ilp_parents = p.nets.size();
        fallback.post_sat_ilp_segments = p.pairs.size();
        fallback.post_sat_ilp_components = p.components.size();
        if (p.nets.empty()) {
            fallback.post_sat_ilp_status = "SKIPPED_NO_NONBUS_NETS";
            finish(fallback);
            return fallback;
        }
        std::size_t node_slots = 0, arc_slots = 0;
        for (const auto &pair : p.pairs) {
            node_slots += pair.nodes.size();
            arc_slots += pair.arcs.size();
        }
        debug::info_fmt(
            "V22 post-SAT ILP prepare: nonbus_nets={} pairs={} components={} "
            "fixed_bus_nets={} pair_local_node_slots={} "
            "pair_local_arc_slots={} "
            "locked_nodes={} locked_switches={}",
            p.nets.size(), p.pairs.size(), p.components.size(),
            locked.nets.size(), node_slots, arc_slots,
            std::count(locked.nodes.begin(), locked.nodes.end(), true),
            locked.switches.size());
        auto all = Result{};
        all.pairs.resize(p.pairs.size());
        for (std::size_t i = 0; i < p.components.size(); ++i) {
            const auto &r =
                solve_component(g, p, locked, p.components[i], sat, options);
            all.stats.f += r.stats.f;
            all.stats.d += r.stats.d;
            all.stats.y += r.stats.y;
            all.stats.s += r.stats.s;
            all.stats.m += r.stats.m;
            all.stats.rows += r.stats.rows;
            all.stats.nz += r.stats.nz;
            all.stats.start += r.stats.start;
            all.stats.f_implies_d += r.stats.f_implies_d;
            all.stats.f_implies_s += r.stats.f_implies_s;
            all.stats.flow_balance += r.stats.flow_balance;
            all.stats.source_no_incoming += r.stats.source_no_incoming;
            all.stats.node_incoming += r.stats.node_incoming;
            all.stats.sink_no_outgoing += r.stats.sink_no_outgoing;
            all.stats.endpoint_d_fixed += r.stats.endpoint_d_fixed;
            all.stats.d_implies_y += r.stats.d_implies_y;
            all.stats.y_support += r.stats.y_support;
            all.stats.s_support += r.stats.s_support;
            all.stats.node_exclusivity += r.stats.node_exclusivity;
            all.stats.switch_exclusivity += r.stats.switch_exclusivity;
            all.stats.tob_matching += r.stats.tob_matching;
            all.stats.mode_binding += r.stats.mode_binding;
            all.stats.build += r.stats.build;
            all.stats.solve += r.stats.solve;
            all.stats.objective += r.stats.objective;
            all.stats.bound += r.stats.bound;
            all.stats.gap = std::max(all.stats.gap, r.stats.gap);
            if (!r.ok) {
                fallback.post_sat_ilp_status =
                    std::format("COMPONENT_{}_{}", i, r.status);
                stamp(fallback, all.stats);
                finish(fallback);
                return fallback;
            }
            for (const auto &pair : p.pairs)
                if (std::ranges::find(p.components[i], pair.net) !=
                    p.components[i].end())
                    all.pairs[pair.id] = r.pairs[pair.id];
            all.modes.insert(r.modes.begin(), r.modes.end());
            all.status = r.status;
        }
        if (!valid(g, p, all)) {
            fallback.post_sat_ilp_status = "MODEL_VALIDATION_FAILED";
            stamp(fallback, all.stats);
            finish(fallback);
            return fallback;
        }
        auto candidate = extract(g, p, all, sat);
        const auto errors =
            validate_routes(g, nets, p, sat, candidate, all.stats.objective);
        if (!errors.empty()) {
            fallback.post_sat_ilp_status = "ROUTE_VALIDATION_FAILED";
            stamp(fallback, all.stats);
            finish(fallback);
            for (const auto &e : errors)
                debug::warning_fmt("  V22 route validation: {}", e);
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
        candidate.post_sat_ilp_parents = p.nets.size();
        candidate.post_sat_ilp_segments = p.pairs.size();
        candidate.post_sat_ilp_components = p.components.size();
        stamp(candidate, all.stats);
        finish(candidate);
        debug::info_fmt("V22 post-SAT ILP accepted: components={} pairs={} "
                        "wirelength={} -> {} build_ms={} solve_ms={}",
                        p.components.size(), p.pairs.size(),
                        sat.total_wirelength, candidate.total_wirelength,
                        candidate.post_sat_ilp_build_ms,
                        candidate.post_sat_ilp_solve_ms);
        return candidate;
    } catch (const std::exception &e) {
        fallback.post_sat_ilp_status = std::format("EXCEPTION: {}", e.what());
        finish(fallback);
        debug::warning_fmt("V22 ILP fallback: {}", e.what());
        return fallback;
    }
}
} // namespace PR_tool
