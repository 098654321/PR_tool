#include "direct_ilp/direct_router.hh"

#include "common/cob_unit_mask.hh"
#include "common/route_metrics.hh"

#include <debug/debug.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#ifdef USE_HIGHS
#include "global_route_v17/highs_log_sink.hh"
#include <Highs.h>
#endif

namespace PR_tool {
namespace {

struct Commodity {
    std::size_t net_index{};
    std::size_t demand_index{};
    std::size_t owner{};
    int source{-1};
    int sink{-1};
    std::size_t fixed_source_index{};
    std::Vector<int> edges;
    std::Vector<int> nodes;
    std::unordered_map<int, int> virtual_edge_by_source;
};

struct Prepared {
    std::Vector<Commodity> commodities;
    std::size_t owners{};
};

auto pair_key(std::size_t a, int b) -> std::uint64_t {
    return (static_cast<std::uint64_t>(a) << 32U)
        | static_cast<std::uint32_t>(b);
}

auto other(const DirectEdge& edge, int node) -> int {
    if (edge.u == node) return edge.v;
    if (edge.v == node) return edge.u;
    throw std::logic_error("edge is not incident to node");
}

auto scope_for(const std::Vector<RoutingScope>& scopes, std::size_t net_id)
    -> const RoutingScope& {
    const auto it = std::find_if(scopes.begin(), scopes.end(),
                                 [&](const auto& s) { return s.net_id == net_id; });
    if (it == scopes.end()) throw std::runtime_error("direct ILP net scope is missing");
    return *it;
}

auto prepare(const UnifiedGraph& graph, const DirectGraph& direct,
             const std::Vector<RoutingNet>& nets,
             const std::Vector<RoutingScope>& scopes) -> Prepared {
    auto out = Prepared{};
    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        const auto& net = nets[net_index];
        const auto& scope = scope_for(scopes, net.net_id);
        const std::size_t shared_owner = net.is_sync_bus ? 0 : out.owners++;
        auto scope_edges = std::set<int>{};
        for (int arc_id : scope.arc_ids) {
            if (arc_id < 0 || static_cast<std::size_t>(arc_id) >= direct.edge_by_arc.size()) continue;
            const int edge_id = direct.edge_by_arc[static_cast<std::size_t>(arc_id)];
            if (edge_id >= 0) scope_edges.insert(edge_id);
        }
        for (std::size_t demand_index = 0; demand_index < net.demands.size(); ++demand_index) {
            const auto& demand = net.demands[demand_index];
            if (demand.candidate_source_indices.empty())
                throw std::runtime_error("direct ILP demand has no source candidate");
            auto c = Commodity{};
            c.net_index = net_index;
            c.demand_index = demand_index;
            c.owner = net.is_sync_bus ? out.owners++ : shared_owner;
            c.sink = resolve_graph_node(graph, demand.sink);
            c.fixed_source_index = demand.candidate_source_indices.front();
            c.source = net.kind == RoutingNetKind::PNnet
                ? net.virtual_source_node
                : resolve_graph_node(graph, net.sources.at(c.fixed_source_index));
            if (c.source < 0 || c.sink < 0 || c.source == c.sink)
                throw std::runtime_error("direct ILP demand has invalid endpoints");
            auto allowed_source_nodes = std::set<int>{};
            if (net.kind == RoutingNetKind::PNnet) {
                for (auto source_index : demand.candidate_source_indices) {
                    const int node = resolve_graph_node(graph, net.sources.at(source_index));
                    if (node < 0) throw std::runtime_error("PN source is absent from graph");
                    allowed_source_nodes.insert(node);
                }
            }
            std::uint16_t unit_mask = 0xffff;
            if (net.kind == RoutingNetKind::PNnet) {
                unit_mask = 0;
                for (int node : allowed_source_nodes)
                    unit_mask = static_cast<std::uint16_t>(
                        unit_mask | unit_bit(graph.nodes[static_cast<std::size_t>(node)].unit));
            } else if (graph.nodes[static_cast<std::size_t>(c.source)].kind
                       == UnifiedNodeKind::Track) {
                unit_mask = unit_bit(graph.nodes[static_cast<std::size_t>(c.source)].unit);
            }
            auto eligible = std::Vector<char>(direct.edges.size(), 0);
            for (int edge_id : scope_edges) {
                const auto& edge = direct.edges[static_cast<std::size_t>(edge_id)];
                if (edge.virtual_source) {
                    if (net.kind != RoutingNetKind::PNnet || edge.u != c.source
                        || !allowed_source_nodes.contains(edge.v)) continue;
                    c.virtual_edge_by_source.emplace(edge.v, edge_id);
                } else if (!node_unit_eligible(graph.nodes[static_cast<std::size_t>(edge.u)], unit_mask)
                           || !node_unit_eligible(graph.nodes[static_cast<std::size_t>(edge.v)], unit_mask)) {
                    continue;
                }
                eligible[static_cast<std::size_t>(edge_id)] = 1;
            }
            auto reached = std::Vector<char>(graph.nodes.size(), 0);
            auto queue = std::queue<int>{};
            reached[static_cast<std::size_t>(c.source)] = 1;
            queue.push(c.source);
            while (!queue.empty()) {
                const int node = queue.front();
                queue.pop();
                for (int edge_id : direct.incident[static_cast<std::size_t>(node)]) {
                    if (!eligible[static_cast<std::size_t>(edge_id)]) continue;
                    const int next = other(direct.edges[static_cast<std::size_t>(edge_id)], node);
                    if (reached[static_cast<std::size_t>(next)]) continue;
                    reached[static_cast<std::size_t>(next)] = 1;
                    queue.push(next);
                }
            }
            if (!reached[static_cast<std::size_t>(c.sink)])
                throw std::runtime_error(std::format("net {} demand {} disconnected in bbox+1+TOB patch",
                                                     net.net_id, demand.demand_id));
            auto node_set = std::set<int>{c.source, c.sink};
            for (int edge_id : scope_edges) {
                if (!eligible[static_cast<std::size_t>(edge_id)]) continue;
                const auto& edge = direct.edges[static_cast<std::size_t>(edge_id)];
                if (!reached[static_cast<std::size_t>(edge.u)]
                    || !reached[static_cast<std::size_t>(edge.v)]) continue;
                c.edges.push_back(edge_id);
                node_set.insert(edge.u);
                node_set.insert(edge.v);
            }
            c.nodes.assign(node_set.begin(), node_set.end());
            out.commodities.push_back(std::move(c));
        }
    }
    return out;
}

#ifdef USE_HIGHS
class Mip {
public:
    explicit Mip(const DirectIlpOptions& options)
        : log_(options.highs_log_path, options.verbose_level >= 2, false) {
        log_.attach(highs_);
        check(highs_.setOptionValue("mip_rel_gap", 0.015), "mip_rel_gap");
        if (options.time_limit_minutes > 0)
            check(highs_.setOptionValue("time_limit", 60.0 * options.time_limit_minutes),
                  "time_limit");
    }

    auto binary(double cost = 0.0) -> int {
        const int id = static_cast<int>(vars_++);
        check(highs_.addCol(cost, 0, 1, 0, nullptr, nullptr), "addCol");
        check(highs_.changeColIntegrality(id, HighsVarType::kInteger), "integrality");
        return id;
    }

    auto row(double lower, double upper,
             const std::Vector<std::pair<int, double>>& terms) -> void {
        auto ids = std::Vector<HighsInt>{};
        auto values = std::Vector<double>{};
        ids.reserve(terms.size());
        values.reserve(terms.size());
        for (const auto& [id, value] : terms) {
            if (value == 0.0) continue;
            ids.push_back(id);
            values.push_back(value);
        }
        check(highs_.addRow(lower, upper, static_cast<HighsInt>(ids.size()),
                            ids.data(), values.data()), "addRow");
        ++rows_;
        nonzeros_ += ids.size();
    }

    auto run() -> bool {
        check(highs_.run(), "run");
        const auto status = highs_.getModelStatus();
        return status == HighsModelStatus::kOptimal
            || (status == HighsModelStatus::kTimeLimit
                && highs_.getInfo().primal_solution_status == kSolutionStatusFeasible);
    }
    auto values() const -> const std::Vector<double>& { return highs_.getSolution().col_value; }
    auto status() const -> std::String { return highs_.modelStatusToString(highs_.getModelStatus()); }
    auto info() const -> const HighsInfo& { return highs_.getInfo(); }
    auto vars() const -> std::size_t { return vars_; }
    auto rows() const -> std::size_t { return rows_; }
    auto nonzeros() const -> std::size_t { return nonzeros_; }

private:
    static auto check(HighsStatus status, const char* operation) -> void {
        if (status == HighsStatus::kError)
            throw std::runtime_error(std::format("HiGHS {} failed", operation));
    }
    HighsLogSink log_;
    Highs highs_;
    std::size_t vars_{}, rows_{}, nonzeros_{};
};

struct CommodityVars {
    std::unordered_map<int, int> edge;
    std::unordered_map<int, int> node;
};

auto matching_keys(const UnifiedGraph& graph, const DirectEdge& edge)
    -> std::Vector<std::pair<int, int>> {
    if (edge.switch_kind == PhysicalSwitchKind::BumpH) {
        const int bump = graph.nodes[static_cast<std::size_t>(edge.u)].kind
                == UnifiedNodeKind::Bump ? edge.u : edge.v;
        const int hline = bump == edge.u ? edge.v : edge.u;
        return {{0, bump}, {1, hline}};
    }
    if (edge.switch_kind == PhysicalSwitchKind::HLineVLine) {
        const int hline = graph.nodes[static_cast<std::size_t>(edge.u)].kind
                == UnifiedNodeKind::HLine ? edge.u : edge.v;
        const int vline = hline == edge.u ? edge.v : edge.u;
        return {{2, hline}, {3, vline}};
    }
    return {};
}

#endif

} // namespace

auto solve_direct_ilp(const UnifiedGraph& graph, const DirectGraph& direct,
                      const std::Vector<RoutingNet>& nets,
                      const std::Vector<RoutingScope>& scopes,
                      const DirectIlpOptions& options) -> DirectIlpResult {
    auto out = DirectIlpResult{};
#ifndef USE_HIGHS
    (void)graph; (void)direct; (void)nets; (void)scopes; (void)options;
    out.route.message = "HIGHS_UNAVAILABLE";
    return out;
#else
    try {
        const auto begin = std::chrono::steady_clock::now();
        const auto prepared = prepare(graph, direct, nets, scopes);
        out.stats.commodities = prepared.commodities.size();
        out.stats.owners = prepared.owners;
        auto mip = Mip(options);
        auto vars = std::Vector<CommodityVars>(prepared.commodities.size());
        auto edge_uses = std::Vector<std::Vector<int>>(direct.edges.size());
        auto owner_node_uses = std::unordered_map<std::uint64_t, std::Vector<int>>{};
        auto node_owners = std::unordered_map<int, std::Vector<int>>{};
        for (std::size_t ci = 0; ci < prepared.commodities.size(); ++ci) {
            const auto& c = prepared.commodities[ci];
            auto& v = vars[ci];
            v.edge.reserve(c.edges.size());
            v.node.reserve(c.nodes.size());
            for (int edge_id : c.edges) {
                const int column = mip.binary();
                v.edge.emplace(edge_id, column);
                edge_uses[static_cast<std::size_t>(edge_id)].push_back(column);
                ++out.stats.edge_vars;
            }
            for (int node : c.nodes) {
                if (node == c.source && nets[c.net_index].kind == RoutingNetKind::PNnet)
                    continue;
                const int column = mip.binary();
                v.node.emplace(node, column);
                owner_node_uses[pair_key(c.owner, node)].push_back(column);
                ++out.stats.node_vars;
            }
        }
        auto owner_node_vars = std::unordered_map<std::uint64_t, int>{};
        owner_node_vars.reserve(owner_node_uses.size());
        for (const auto& [key, uses] : owner_node_uses) {
            const int node = static_cast<int>(static_cast<std::uint32_t>(key));
            const int column = mip.binary(is_wirelength_resource_node(graph, node) ? 1.0 : 0.0);
            owner_node_vars.emplace(key, column);
            node_owners[node].push_back(column);
            ++out.stats.owner_vars;
            auto upper = std::Vector<std::pair<int, double>>{{column, 1}};
            upper.reserve(uses.size() + 1);
            for (int d : uses) {
                mip.row(-kHighsInf, 0, {{d, 1}, {column, -1}});
                upper.emplace_back(d, -1);
            }
            mip.row(-kHighsInf, 0, upper);
        }
        for (const auto& [_, owners] : node_owners) {
            if (owners.size() < 2) continue;
            auto row = std::Vector<std::pair<int, double>>{};
            for (int y : owners) row.emplace_back(y, 1);
            mip.row(-kHighsInf, 1, row);
        }
        for (std::size_t ci = 0; ci < prepared.commodities.size(); ++ci) {
            const auto& c = prepared.commodities[ci];
            const auto& v = vars[ci];
            const auto& net = nets[c.net_index];
            for (int node : c.nodes) {
                auto degree = std::Vector<std::pair<int, double>>{};
                degree.reserve(direct.incident[static_cast<std::size_t>(node)].size() + 1);
                for (int edge_id : direct.incident[static_cast<std::size_t>(node)]) {
                    const auto it = v.edge.find(edge_id);
                    if (it != v.edge.end()) degree.emplace_back(it->second, 1);
                }
                if (node == c.source || node == c.sink) {
                    mip.row(1, 1, degree);
                    if (const auto it = v.node.find(node); it != v.node.end())
                        mip.row(1, 1, {{it->second, 1}});
                } else if (net.kind == RoutingNetKind::PNnet
                           && c.virtual_edge_by_source.contains(node)) {
                    const int root_edge = v.edge.at(c.virtual_edge_by_source.at(node));
                    degree.erase(std::remove_if(degree.begin(), degree.end(),
                        [&](const auto& term) { return term.first == root_edge; }), degree.end());
                    degree.emplace_back(root_edge, -1);
                    mip.row(0, 0, degree);
                    mip.row(0, 0, {{v.node.at(node), 1}, {root_edge, -1}});
                } else {
                    degree.emplace_back(v.node.at(node), -2);
                    mip.row(0, 0, degree);
                }
            }
            for (int edge_id : c.edges) {
                const auto& edge = direct.edges[static_cast<std::size_t>(edge_id)];
                if (edge.virtual_source) continue;
                const int f = v.edge.at(edge_id);
                mip.row(-kHighsInf, 0, {{f, 1}, {v.node.at(edge.u), -1}});
                mip.row(-kHighsInf, 0, {{f, 1}, {v.node.at(edge.v), -1}});
            }
        }
        auto switch_vars = std::unordered_map<int, int>{};
        auto matching = std::map<std::pair<int, int>, std::set<int>>{};
        auto modes = std::map<int, std::Vector<std::pair<int, bool>>>{};
        for (int edge_id = 0; edge_id < static_cast<int>(direct.edges.size()); ++edge_id) {
            const auto& edge = direct.edges[static_cast<std::size_t>(edge_id)];
            if (edge.switch_id < 0 || edge_uses[static_cast<std::size_t>(edge_id)].empty())
                continue;
            const int y = mip.binary();
            switch_vars.emplace(edge_id, y);
            ++out.stats.switch_vars;
            auto upper = std::Vector<std::pair<int, double>>{{y, 1}};
            for (int f : edge_uses[static_cast<std::size_t>(edge_id)]) {
                mip.row(-kHighsInf, 0, {{f, 1}, {y, -1}});
                upper.emplace_back(f, -1);
            }
            mip.row(-kHighsInf, 0, upper);
            for (const auto& key : matching_keys(graph, edge)) matching[key].insert(y);
            if (edge.mode_group >= 0) modes[edge.mode_group].push_back({y, edge.straight});
        }
        for (const auto& [_, values] : matching) {
            if (values.size() < 2) continue;
            auto row = std::Vector<std::pair<int, double>>{};
            for (int y : values) row.emplace_back(y, 1);
            mip.row(-kHighsInf, 1, row);
        }
        for (const auto& [_, uses] : modes) {
            const int mode = mip.binary();
            ++out.stats.mode_vars;
            for (const auto& [y, straight] : uses) {
                if (straight) mip.row(-kHighsInf, 0, {{y, 1}, {mode, -1}});
                else mip.row(-kHighsInf, 1, {{y, 1}, {mode, 1}});
            }
        }
        for (std::size_t ni = 0; ni < nets.size(); ++ni) {
            if (!nets[ni].is_sync_bus) continue;
            auto members = std::Vector<std::size_t>{};
            for (std::size_t ci = 0; ci < prepared.commodities.size(); ++ci)
                if (prepared.commodities[ci].net_index == ni) members.push_back(ci);
            if (members.size() < 2) continue;
            const auto length_terms = [&](std::size_t ci, double sign,
                                          std::Vector<std::pair<int, double>>& row) {
                for (const auto& [node, column] : vars[ci].node)
                    if (is_wirelength_resource_node(graph, node)) row.emplace_back(column, sign);
            };
            for (std::size_t i = 1; i < members.size(); ++i) {
                auto row = std::Vector<std::pair<int, double>>{};
                length_terms(members[i], 1, row);
                length_terms(members[0], -1, row);
                mip.row(0, 0, row);
            }
        }
        out.stats.variables = mip.vars();
        out.stats.rows = mip.rows();
        out.stats.nonzeros = mip.nonzeros();
        out.stats.build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - begin).count();
        debug::info_fmt("direct ILP model: commodities={} owners={} vars={} rows={} nz={} F={} D={} Y={} switch={} mode={} build_ms={}",
            out.stats.commodities, out.stats.owners, out.stats.variables,
            out.stats.rows, out.stats.nonzeros, out.stats.edge_vars,
            out.stats.node_vars, out.stats.owner_vars, out.stats.switch_vars,
            out.stats.mode_vars, out.stats.build_ms);
        const auto solve_begin = std::chrono::steady_clock::now();
        const bool feasible = mip.run();
        out.stats.solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - solve_begin).count();
        out.stats.status = mip.status();
        debug::info_fmt("direct ILP solve: status={} feasible={} solve_ms={}",
                        out.stats.status, feasible, out.stats.solve_ms);
        if (!feasible) {
            out.route.message = out.stats.status;
            return out;
        }
        out.stats.objective = mip.info().objective_function_value;
        out.stats.bound = mip.info().mip_dual_bound;
        out.stats.gap = mip.info().mip_gap;
        const auto& values = mip.values();
        for (std::size_t ci = 0; ci < prepared.commodities.size(); ++ci) {
            const auto& c = prepared.commodities[ci];
            const auto& net = nets[c.net_index];
            const auto& demand = net.demands[c.demand_index];
            const auto& v = vars[ci];
            auto predecessor = std::unordered_map<int, std::pair<int, int>>{};
            auto seen = std::set<int>{c.source};
            auto queue = std::queue<int>{};
            queue.push(c.source);
            while (!queue.empty() && !seen.contains(c.sink)) {
                const int node = queue.front(); queue.pop();
                for (int edge_id : direct.incident[static_cast<std::size_t>(node)]) {
                    const auto it = v.edge.find(edge_id);
                    if (it == v.edge.end() || values[static_cast<std::size_t>(it->second)] < 0.5)
                        continue;
                    const int next = other(direct.edges[static_cast<std::size_t>(edge_id)], node);
                    if (!seen.insert(next).second) continue;
                    predecessor[next] = {node, edge_id};
                    queue.push(next);
                }
            }
            if (!seen.contains(c.sink)) throw std::runtime_error("selected ILP edges do not reach sink");
            auto nodes = std::Vector<int>{c.sink};
            auto path_edges = std::Vector<int>{};
            for (int node = c.sink; node != c.source;) {
                const auto [previous, edge_id] = predecessor.at(node);
                path_edges.push_back(edge_id);
                node = previous;
                nodes.push_back(node);
            }
            std::reverse(nodes.begin(), nodes.end());
            std::reverse(path_edges.begin(), path_edges.end());
            std::size_t source_index = c.fixed_source_index;
            int physical_source = -1;
            if (net.kind == RoutingNetKind::PNnet) {
                if (nodes.size() < 3) throw std::runtime_error("PN path lacks physical source");
                physical_source = nodes[1];
                const auto it = std::find_if(demand.candidate_source_indices.begin(),
                    demand.candidate_source_indices.end(), [&](std::size_t index) {
                        return resolve_graph_node(graph, net.sources[index]) == physical_source;
                    });
                if (it == demand.candidate_source_indices.end())
                    throw std::runtime_error("PN path selected noncandidate source");
                source_index = *it;
                nodes.erase(nodes.begin());
                path_edges.erase(path_edges.begin());
            }
            out.route.paths.push_back({net.net_id, source_index, demand.demand_id,
                                       physical_source, std::move(nodes)});
            for (int edge_id : path_edges) {
                const auto& edge = direct.edges[static_cast<std::size_t>(edge_id)];
                if (edge.switch_id >= 0)
                    out.route.used_tob_switch_ids.push_back(edge.switch_id);
                if (edge.mode_group >= 0)
                    out.route.vline_mode_straight_by_group[static_cast<std::size_t>(edge.mode_group)]
                        = edge.straight;
            }
        }
        std::sort(out.route.used_tob_switch_ids.begin(), out.route.used_tob_switch_ids.end());
        out.route.used_tob_switch_ids.erase(
            std::unique(out.route.used_tob_switch_ids.begin(),
                        out.route.used_tob_switch_ids.end()),
            out.route.used_tob_switch_ids.end());
        out.route.total_wirelength = total_wirelength(graph, out.route);
        out.route.ok = true;
        out.route.message = "FEASIBLE";
        debug::info_fmt("direct ILP result: objective={:.0f} bound={:.3f} gap={:.6f} paths={} actual_wirelength={}",
                        out.stats.objective, out.stats.bound, out.stats.gap,
                        out.route.paths.size(), out.route.total_wirelength);
        if (std::abs(out.stats.objective
                     - static_cast<double>(out.route.total_wirelength)) > 0.5) {
            debug::info("direct ILP selected extra resources outside extracted paths; "
                        "RRR uses actual path wirelength");
        }
        for (const auto& path : out.route.paths) {
            auto physical_path = std::String{};
            for (int node : path.node_path) {
                if (!physical_path.empty()) physical_path += " -> ";
                physical_path += format_unified_node(graph, node);
            }
            debug::info_fmt("  ILP route net={} demand={} source={} physical_source={} path={}",
                path.net_id, path.demand_id, path.source_index,
                path.physical_source_node, physical_path);
        }
        return out;
    } catch (const std::exception& error) {
        out.route.ok = false;
        out.route.message = error.what();
        debug::error_fmt("direct ILP failed: {}", error.what());
        return out;
    }
#endif
}

} // namespace PR_tool
