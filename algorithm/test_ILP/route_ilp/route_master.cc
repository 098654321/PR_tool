#include "route_ilp/route_master.hh"

#include "common/highs_log_sink.hh"
#include "common/hw_map.hh"
#include "common/route_metrics.hh"
#include "scope/scope_bbox.hh"

#include <Highs.h>
#include <debug/debug.hh>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>

namespace PR_tool {
namespace {

struct MasterSolution {
    bool feasible{};
    std::String status;
    std::Vector<std::Vector<double>> x;
    std::Vector<double> slack;
    std::Vector<double> owner_dual;
    std::map<RouteResource, double> resource_dual;
    double objective{};
    double bound{};
    double gap{};
    std::size_t vars{};
    std::size_t rows{};
    std::size_t nonzeros{};
    std::array<std::size_t, 4> resource_rows{};
};

auto check(HighsStatus status, const char* operation) -> void {
    if (status == HighsStatus::kError)
        throw std::runtime_error(std::format("route ILP HiGHS {} failed", operation));
}

auto add_row(Highs& highs, double lower, double upper,
             const std::Vector<std::pair<int, double>>& terms,
             MasterSolution& out) -> void {
    auto ids = std::Vector<HighsInt>{};
    auto values = std::Vector<double>{};
    for (const auto& [id, value] : terms) {
        ids.push_back(id); values.push_back(value);
    }
    check(highs.addRow(lower, upper, static_cast<HighsInt>(ids.size()),
                       ids.data(), values.data()), "addRow");
    ++out.rows;
    out.nonzeros += ids.size();
}

auto solve_master(const std::Vector<std::Vector<RouteColumn>>& pool,
                  bool integer, double big_m, const RouteIlpOptions& options,
                  bool truncate_log, double remaining_seconds) -> MasterSolution {
    auto out = MasterSolution{};
    auto log = HighsLogSink(options.highs_log_path, options.verbose_level >= 2,
                            !truncate_log);
    auto highs = Highs{};
    log.attach(highs);
    check(highs.setOptionValue("mip_rel_gap", 0.015), "mip_rel_gap");
    if (options.time_limit_minutes > 0)
        check(highs.setOptionValue("time_limit", std::max(0.1, remaining_seconds)),
              "time_limit");
    auto cols = std::Vector<std::Vector<int>>(pool.size());
    auto slack_cols = std::Vector<int>(pool.size());
    for (std::size_t i = 0; i < pool.size(); ++i) {
        for (const auto& candidate : pool[i]) {
            const int id = static_cast<int>(out.vars++);
            check(highs.addCol(static_cast<double>(candidate.wirelength), 0, 1,
                               0, nullptr, nullptr), "add route column");
            if (integer)
                check(highs.changeColIntegrality(id, HighsVarType::kInteger),
                      "route integrality");
            cols[i].push_back(id);
        }
        const int id = static_cast<int>(out.vars++);
        check(highs.addCol(big_m, 0, 1, 0, nullptr, nullptr), "add slack column");
        if (integer)
            check(highs.changeColIntegrality(id, HighsVarType::kInteger),
                  "slack integrality");
        slack_cols[i] = id;
    }
    for (std::size_t i = 0; i < pool.size(); ++i) {
        auto terms = std::Vector<std::pair<int, double>>{};
        for (int id : cols[i]) terms.emplace_back(id, 1.0);
        terms.emplace_back(slack_cols[i], 1.0);
        add_row(highs, 1, 1, terms, out);
    }
    auto resources = std::map<RouteResource,
        std::Vector<std::pair<int, double>>>{};
    for (std::size_t i = 0; i < pool.size(); ++i)
        for (std::size_t j = 0; j < pool[i].size(); ++j)
            for (const auto& resource : pool[i][j].resources)
                resources[resource].emplace_back(cols[i][j], 1.0);
    for (const auto& [resource, terms] : resources)
        if (terms.size() > 1) {
            add_row(highs, -kHighsInf, 1, terms, out);
            ++out.resource_rows.at(static_cast<std::size_t>(resource.kind));
        }
    check(highs.run(), "run");
    out.status = highs.modelStatusToString(highs.getModelStatus());
    const auto& solution = highs.getSolution();
    out.feasible = highs.getInfo().primal_solution_status == kSolutionStatusFeasible &&
        solution.col_value.size() == out.vars;
    if (!out.feasible) return out;
    out.objective = highs.getInfo().objective_function_value;
    out.bound = highs.getInfo().mip_dual_bound;
    out.gap = highs.getInfo().mip_gap;
    out.x.resize(pool.size());
    out.slack.resize(pool.size());
    for (std::size_t i = 0; i < pool.size(); ++i) {
        for (int id : cols[i]) out.x[i].push_back(solution.col_value[id]);
        out.slack[i] = solution.col_value[slack_cols[i]];
    }
    if (!integer && solution.row_dual.size() == out.rows) {
        out.owner_dual.assign(solution.row_dual.begin(),
                              solution.row_dual.begin() + pool.size());
        std::size_t row = pool.size();
        for (const auto& [resource, terms] : resources)
            if (terms.size() > 1)
                out.resource_dual[resource] = solution.row_dual[row++];
    }
    return out;
}

auto same_column(const RouteColumn& a, const RouteColumn& b) -> bool {
    if (a.paths.size() != b.paths.size() || a.resources != b.resources) return false;
    for (std::size_t j = 0; j < a.paths.size(); ++j)
        if (a.paths[j].demand_id != b.paths[j].demand_id ||
            a.paths[j].source_index != b.paths[j].source_index ||
            a.paths[j].node_path != b.paths[j].node_path) return false;
    return true;
}

auto route_metadata(const UnifiedGraph& graph, RoutingResult& result) -> void {
    auto switches = std::set<int>{};
    result.vline_mode_straight_by_group.clear();
    for (const auto& path : result.paths)
        for (std::size_t j = 1; j < path.node_path.size(); ++j)
            for (int aid : graph.out_arc_ids[static_cast<std::size_t>(path.node_path[j - 1])]) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(aid)];
                if (arc.v != path.node_path[j]) continue;
                if (arc.physical_switch_id >= 0) switches.insert(arc.physical_switch_id);
                if (arc.mode_group_id >= 0)
                    result.vline_mode_straight_by_group[
                        static_cast<std::size_t>(arc.mode_group_id)] =
                        arc.is_vline_track_straight;
                break;
            }
    result.used_tob_switch_ids.assign(switches.begin(), switches.end());
    result.total_wirelength = total_wirelength(graph, result);
}

auto resource_anchor(const UnifiedGraph& graph, RouteResource resource,
                     const std::map<int, int>& switch_nodes) -> hardware::COBCoord {
    int node_id = resource.id;
    if (resource.kind == 1 || resource.kind == 3) {
        const auto it = switch_nodes.find(resource.id);
        if (it == switch_nodes.end()) throw std::logic_error("switch lacks TOB anchor");
        node_id = it->second;
    }
    const auto& node = graph.nodes.at(static_cast<std::size_t>(node_id));
    if (node.kind != UnifiedNodeKind::Track) return tob_anchor_cob(node.tob);
    auto coord = hardware::COBCoord{node.track_row, node.track_col};
    if (node.track_dir == 0 && coord.col == graph.cols) --coord.col;
    if (node.track_dir != 0 && coord.row == graph.rows) --coord.row;
    return coord;
}

auto in_box(hardware::COBCoord coord, const IlpBoundingBox& box) -> bool {
    return coord.row >= box.row_min && coord.row <= box.row_max &&
           coord.col >= box.col_min && coord.col <= box.col_max;
}

} // namespace

auto solve_route_ilp_impl(const UnifiedGraph& graph,
                          const std::Vector<RoutingNet>& nets,
                          const std::Vector<RoutingScope>& scopes,
                          const RouteIlpOptions& options,
                          const std::map<std::size_t, std::size_t>& forced_lengths,
                          bool allow_length_increase,
                          std::chrono::steady_clock::time_point global_begin)
    -> RouteIlpResult {
    const auto start = std::chrono::steady_clock::now();
    const auto remaining = [&]() -> double {
        if (options.time_limit_minutes <= 0) return 1e9;
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - global_begin).count();
        return 60.0 * options.time_limit_minutes - elapsed;
    };
    auto result = RouteIlpResult{};
    const auto owners = route_owners(nets);
    auto switch_nodes = std::map<int, int>{};
    for (const auto& arc : graph.arcs)
        if (arc.physical_switch_id >= 0 &&
            !switch_nodes.contains(arc.physical_switch_id)) {
            const int node = graph.nodes[static_cast<std::size_t>(arc.u)].kind ==
                    UnifiedNodeKind::Track ? arc.v : arc.u;
            switch_nodes.emplace(arc.physical_switch_id, node);
        }
    auto owner_endpoints = std::Vector<std::Vector<hardware::COBCoord>>(owners.size());
    for (std::size_t i = 0; i < owners.size(); ++i) {
        const auto& net = net_for_owner(nets, owners[i]);
        for (const auto& source : net.sources)
            owner_endpoints[i].push_back(resource_anchor(graph,
                {0, resolve_graph_node(graph, source), 0}, switch_nodes));
        for (const auto& demand : net.demands)
            if (!net.is_sync_bus || demand.demand_id == owners[i].demand_id)
                owner_endpoints[i].push_back(resource_anchor(graph,
                    {0, resolve_graph_node(graph, demand.sink), 0}, switch_nodes));
    }
    auto pool = std::Vector<std::Vector<RouteColumn>>(owners.size());
    auto shortest = std::Vector<RouteColumn>(owners.size());
    for (std::size_t i = 0; i < owners.size(); ++i) {
        const auto& net = net_for_owner(nets, owners[i]);
        const auto& scope = scope_for_owner(scopes, owners[i]);
        shortest[i] = find_route_column(graph, net, scope, owners[i], {});
        if (net.is_sync_bus && !shortest[i].paths.empty())
            result.bus_lengths[net.net_id] = std::max(
                result.bus_lengths[net.net_id], shortest[i].wirelength);
    }
    for (const auto& [bus, target] : forced_lengths)
        result.bus_lengths[bus] = target;
    for (std::size_t i = 0; i < owners.size(); ++i) {
        const auto& net = net_for_owner(nets, owners[i]);
        if (!net.is_sync_bus) {
            if (!shortest[i].paths.empty()) pool[i].push_back(shortest[i]);
            continue;
        }
        const auto target = result.bus_lengths[net.net_id];
        if (target == 0) continue;
        if (shortest[i].wirelength == target && !shortest[i].paths.empty())
            pool[i].push_back(shortest[i]);
        else {
            auto search = RouteSearchOptions{}; search.exact_length = target;
            auto column = find_route_column(graph, net,
                scope_for_owner(scopes, owners[i]), owners[i], search);
            if (!column.paths.empty()) pool[i].push_back(std::move(column));
        }
    }
    double cost_bound = 0.0;
    for (const auto owner : owners)
        for (int node : scope_for_owner(scopes, owner).node_ids)
            if (is_wirelength_resource_node(graph, node)) cost_bound += 1.0;
    const double big_m = std::max(10000.0, cost_bound + 1.0);
    debug::info_fmt("route ILP initial: owners={} columns={} bus_groups={} M={}",
        owners.size(), std::ranges::fold_left(pool, std::size_t{},
            [](std::size_t n, const auto& values) { return n + values.size(); }),
        result.bus_lengths.size(), big_m);
    bool first_log = forced_lengths.empty();
    int stagnant = 0;
    auto lp_objectives = std::Vector<double>{};
    for (int round = 0; round < 12; ++round) {
        if (remaining() <= 0.0) break;
        auto lp = solve_master(pool, false, big_m, options, first_log,
                               remaining());
        first_log = false;
        debug::info_fmt("route ILP LP round={} status={} objective={} vars={} rows={} nonzeros={}",
            round, lp.status, lp.objective, lp.vars, lp.rows, lp.nonzeros);
        if (!lp.feasible) break;
        lp_objectives.push_back(lp.objective);
        if (lp_objectives.size() > 10) {
            const double previous = lp_objectives[lp_objectives.size() - 11];
            const double improvement = (previous - lp.objective) /
                std::max(1.0, std::abs(previous));
            if (improvement < 0.01) {
                debug::info_fmt("route ILP pricing stop: ten-round LP improvement={}",
                                improvement);
                break;
            }
        }
        auto selected = std::set<std::size_t>{};
        auto predicted = std::map<RouteResource, std::set<std::size_t>>{};
        for (std::size_t i = 0; i < owners.size(); ++i) {
            if (lp.slack[i] > 1e-6) selected.insert(i);
            if (pool[i].empty()) continue;
            const auto top = static_cast<std::size_t>(std::distance(lp.x[i].begin(),
                std::max_element(lp.x[i].begin(), lp.x[i].end())));
            if (lp.x[i][top] <= 1e-6) continue;
            for (const auto& resource : pool[i][top].resources)
                predicted[resource].insert(i);
        }
        for (const auto& [resource, dual] : lp.resource_dual)
            if (dual < -1e-8) predicted.try_emplace(resource);
        auto candidate_users = std::map<RouteResource, std::set<std::size_t>>{};
        for (std::size_t i = 0; i < owners.size(); ++i)
            for (const auto& column : pool[i])
                for (const auto& resource : column.resources)
                    candidate_users[resource].insert(i);
        for (const auto& [resource, users] : predicted) {
            const auto price = lp.resource_dual.find(resource);
            if (users.size() <= 1 && (price == lp.resource_dual.end() ||
                price->second >= -1e-8)) continue;
            selected.insert(users.begin(), users.end());
            const auto known = candidate_users.find(resource);
            if (known != candidate_users.end())
                selected.insert(known->second.begin(), known->second.end());
            const auto anchor = resource_anchor(graph, resource, switch_nodes);
            const auto neighborhood = expand_pair_bbox_one_cell({
                anchor.row, anchor.row, anchor.col, anchor.col});
            for (std::size_t i = 0; i < owners.size(); ++i)
                if (std::ranges::any_of(owner_endpoints[i],
                    [&](const auto endpoint) { return in_box(endpoint, neighborhood); }))
                    selected.insert(i);
        }
        // Few-net instances can afford a periodic full pass to catch owners
        // whose endpoint neighborhood, but not current route, meets a hotspot.
        if (selected.empty() || round % 3 == 2)
            for (std::size_t i = 0; i < owners.size(); ++i) selected.insert(i);
        debug::info_fmt("route ILP pricing selection: round={} owners={}",
                        round, selected.size());
        auto added = std::size_t{};
        for (const std::size_t i : selected) {
            const auto& net = net_for_owner(nets, owners[i]);
            const auto& scope = scope_for_owner(scopes, owners[i]);
            auto options_for_search = RouteSearchOptions{};
            if (net.is_sync_bus) options_for_search.exact_length =
                result.bus_lengths[net.net_id];
            for (const auto& [resource, dual] : lp.resource_dual)
                if (dual < -1e-8) options_for_search.prices[resource] = -dual;
            std::size_t base = 0;
            if (!pool[i].empty())
                for (std::size_t j = 1; j < pool[i].size(); ++j)
                    if (lp.x[i][j] > lp.x[i][base]) base = j;
            auto terminals = std::Vector<std::pair<std::size_t, std::size_t>>{};
            if (!pool[i].empty())
                for (const auto& path : pool[i][base].paths) {
                    auto other_nodes = std::set<int>{};
                    for (const auto& other : pool[i][base].paths)
                        if (other.demand_id != path.demand_id)
                            other_nodes.insert(other.node_path.begin(),
                                               other.node_path.end());
                    std::size_t branch_cost = 0;
                    for (int node : path.node_path)
                        if (!other_nodes.contains(node) &&
                            is_wirelength_resource_node(graph, node)) ++branch_cost;
                    terminals.emplace_back(branch_cost, path.demand_id);
                }
            std::sort(terminals.begin(), terminals.end(), std::greater{});
            if (!pool[i].empty())
                for (const auto& path : pool[i][base].paths)
                    for (int node : path.node_path)
                        options_for_search.discouraged_nodes.insert(node);
            const auto discouraged = options_for_search.discouraged_nodes;
            auto negative = std::Vector<RouteColumn>{};
            auto feasibility = std::Vector<RouteColumn>{};
            for (int variant = 0; variant < 5; ++variant) {
                options_for_search.variant = round * 5 + variant;
                options_for_search.discouraged_nodes = variant == 0 ?
                    std::set<int>{} : discouraged;
                const RouteColumn* base_column = pool[i].empty() ?
                    nullptr : &pool[i][base];
                const std::size_t terminal = terminals.empty() ? 0 :
                    terminals[static_cast<std::size_t>(variant) % terminals.size()].second;
                auto candidate = find_route_column(graph, net, scope, owners[i],
                                                   options_for_search,
                                                   base_column, terminal);
                if (candidate.paths.empty() || std::ranges::any_of(pool[i],
                    [&](const auto& old) { return same_column(old, candidate); }) ||
                    std::ranges::any_of(negative, [&](const auto& old) {
                        return same_column(old, candidate); }) ||
                    std::ranges::any_of(feasibility, [&](const auto& old) {
                        return same_column(old, candidate); })) continue;
                double reduced = static_cast<double>(candidate.wirelength) -
                    (i < lp.owner_dual.size() ? lp.owner_dual[i] : 0.0);
                for (const auto& resource : candidate.resources) {
                    const auto it = lp.resource_dual.find(resource);
                    if (it != lp.resource_dual.end()) reduced -= it->second;
                }
                if (reduced < -1e-7) negative.push_back(std::move(candidate));
                else if (lp.slack[i] > 1e-6)
                    feasibility.push_back(std::move(candidate));
            }
            auto by_length = [](const RouteColumn& a, const RouteColumn& b) {
                return std::tuple{a.wirelength, a.paths.size()} <
                       std::tuple{b.wirelength, b.paths.size()};
            };
            std::sort(negative.begin(), negative.end(), by_length);
            std::sort(feasibility.begin(), feasibility.end(), by_length);
            auto& chosen = negative.empty() ? feasibility : negative;
            for (std::size_t j = 0; j < std::min<std::size_t>(2, chosen.size()); ++j) {
                pool[i].push_back(std::move(chosen[j]));
                ++added;
            }
        }
        debug::info_fmt("route ILP pricing round={} added={}", round, added);
        stagnant = added == 0 ? stagnant + 1 : 0;
        bool sync_slack = false;
        for (std::size_t i = 0; i < owners.size(); ++i)
            if (lp.slack[i] > 1e-6 &&
                net_for_owner(nets, owners[i]).is_sync_bus) sync_slack = true;
        if (added == 0 && (!sync_slack || stagnant >= 5)) break;
    }
    auto mip = solve_master(pool, true, big_m, options, first_log,
                            remaining());
    debug::info_fmt("route ILP MIP: status={} objective={} bound={} gap={} vars={} rows={} nonzeros={}",
        mip.status, mip.objective, mip.bound, mip.gap,
        mip.vars, mip.rows, mip.nonzeros);
    debug::info_fmt("route ILP model: x={} s={} owner_rows={} node_rows={} switch_rows={} matching_rows={} mode_rows={}",
        mip.vars - owners.size(), owners.size(), owners.size(),
        mip.resource_rows[0], mip.resource_rows[1],
        mip.resource_rows[2], mip.resource_rows[3]);
    for (std::size_t i = 0; i < owners.size(); ++i) {
        const double slack = mip.feasible ? mip.slack[i] : 1.0;
        const auto chosen = mip.feasible ? std::ranges::count_if(mip.x[i],
            [](double value) { return value > 0.5; }) : 0;
        debug::info_fmt("  route owner net={} demand={} columns={} chosen={} s={}",
            owners[i].net_id, owners[i].demand_id, pool[i].size(), chosen, slack);
    }
    result.status = mip.status;
    result.has_integer_solution = mip.feasible;
    if (!mip.feasible) {
        // The all-slack assignment is always an integer feasible fallback.
        for (const auto owner : owners) result.missing.insert(owner);
        result.has_integer_solution = true;
        result.status = "ALL_SLACK_FALLBACK";
    } else {
        for (std::size_t i = 0; i < owners.size(); ++i) {
            bool selected = false;
            for (std::size_t j = 0; j < pool[i].size(); ++j)
                if (mip.x[i][j] > 0.5) {
                    result.route.paths.insert(result.route.paths.end(),
                        pool[i][j].paths.begin(), pool[i][j].paths.end());
                    selected = true;
                    break;
                }
            if (!selected) result.missing.insert(owners[i]);
        }
    }
    route_metadata(graph, result.route);
    result.route.ok = result.missing.empty();
    result.route.message = result.route.ok ? "COMPLETE" : "PARTIAL";
    debug::info_fmt("route ILP result: routed={} missing={} wirelength={} solve_ms={}",
        owners.size() - result.missing.size(), result.missing.size(),
        result.route.total_wirelength,
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count());
    if (allow_length_increase && remaining() > 0.0) {
        auto increased = std::map<std::size_t, std::size_t>{};
        for (const auto owner : result.missing) {
            const auto& net = net_for_owner(nets, owner);
            if (net.is_sync_bus && result.bus_lengths[net.net_id] > 0)
                increased[net.net_id] = result.bus_lengths[net.net_id] + 1;
        }
        if (!increased.empty()) {
            for (const auto& [bus, length] : increased)
                debug::info_fmt("route ILP SyncBus length increase: bus={} L={}",
                                bus, length);
            return solve_route_ilp_impl(graph, nets, scopes, options, increased,
                                       false, global_begin);
        }
    }
    return result;
}

auto solve_route_ilp(const UnifiedGraph& graph,
                     const std::Vector<RoutingNet>& nets,
                     const std::Vector<RoutingScope>& scopes,
                     const RouteIlpOptions& options) -> RouteIlpResult {
    return solve_route_ilp_impl(graph, nets, scopes, options, {}, true,
                               std::chrono::steady_clock::now());
}

} // namespace PR_tool
