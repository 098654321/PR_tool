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
#include <string_view>

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
                // Switch and matching-port reuse already shares a physical node.
                if (resource.kind == 0 || resource.kind == 3)
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

auto log_column_paths(const UnifiedGraph& graph, const RouteColumn& column,
                      std::string_view label) -> void {
    for (const auto& path : column.paths) {
        auto description = std::String{};
        for (int node : path.node_path) {
            if (!description.empty()) description += " -> ";
            description += format_unified_node(graph, node);
        }
        debug::info_fmt("{} route net={} demand={} source={} path={}", label,
                        path.net_id, path.demand_id, path.source_index, description);
    }
}

auto format_resource_node(const UnifiedGraph& graph, RouteResource resource,
                          const std::map<int, int>& switch_nodes) -> std::String {
    if (resource.kind == 0)
        return format_unified_node(graph, resource.id);
    if (resource.kind == 3) {
        const auto found = switch_nodes.find(resource.id);
        if (found != switch_nodes.end())
            return std::format("{} (mode conflict {}, {})",
                format_unified_node(graph, found->second), resource.id, resource.extra);
    }
    return std::format("resource(kind={}, id={}, extra={})",
                       resource.kind, resource.id, resource.extra);
}

auto same_column(const RouteColumn& a, const RouteColumn& b) -> bool {
    if (a.paths.size() != b.paths.size() || a.resources != b.resources) return false;
    for (const auto& path : a.paths)
        if (std::ranges::none_of(b.paths, [&](const auto& other) {
            return path.demand_id == other.demand_id &&
                   path.source_index == other.source_index &&
                   path.node_path == other.node_path;
        })) return false;
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
                          std::chrono::steady_clock::time_point global_begin)
    -> RouteIlpResult {
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = options.time_limit_minutes > 0 ?
        global_begin + std::chrono::minutes(options.time_limit_minutes) :
        std::chrono::steady_clock::time_point::max();
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
        auto search = RouteSearchOptions{};
        search.deadline = deadline;
        shortest[i] = find_route_column(graph, net, scope, owners[i], search);
        if (net.is_sync_bus && !shortest[i].paths.empty())
            result.bus_lengths[net.net_id] = std::max(
                result.bus_lengths[net.net_id], shortest[i].wirelength);
    }
    const auto initial_bus_lengths = result.bus_lengths;
    auto sat_columns = std::Vector<RouteColumn>(owners.size());
    auto sat_bus_lengths = std::map<std::size_t, std::size_t>{};
    if (options.initial_sat_route != nullptr) {
        for (std::size_t i = 0; i < owners.size(); ++i) {
            const auto& net = net_for_owner(nets, owners[i]);
            auto paths = std::Vector<SourceSinkPairPath>{};
            for (const auto& path : options.initial_sat_route->paths)
                if (path.net_id == net.net_id &&
                    (!net.is_sync_bus || path.demand_id == owners[i].demand_id))
                    paths.push_back(path);
            sat_columns[i] = route_column_from_paths(graph, net, owners[i], paths);
            if (net.is_sync_bus) {
                const auto [it, inserted] = sat_bus_lengths.emplace(
                    net.net_id, sat_columns[i].wirelength);
                if (!inserted && it->second != sat_columns[i].wirelength)
                    throw std::logic_error("initial SAT SyncBus lengths disagree");
            }
        }
        for (const auto& [bus, length] : sat_bus_lengths)
            result.bus_lengths[bus] = length;
        debug::info_fmt("route ILP initial SAT columns: owners={} wirelength={}",
            sat_columns.size(), options.initial_sat_route->total_wirelength);
    }
    for (std::size_t i = 0; i < owners.size(); ++i) {
        const auto& net = net_for_owner(nets, owners[i]);
        if (!net.is_sync_bus) {
            if (!shortest[i].paths.empty()) pool[i].push_back(shortest[i]);
            if (options.initial_sat_route != nullptr &&
                std::ranges::none_of(pool[i], [&](const auto& column) {
                    return same_column(column, sat_columns[i]);
                })) pool[i].push_back(sat_columns[i]);
            continue;
        }
        const auto target = result.bus_lengths[net.net_id];
        if (target == 0) continue;
        if (shortest[i].wirelength == target && !shortest[i].paths.empty())
            pool[i].push_back(shortest[i]);
        else {
            auto search = RouteSearchOptions{}; search.exact_length = target;
            search.deadline = deadline;
            auto column = find_route_column(graph, net,
                scope_for_owner(scopes, owners[i]), owners[i], search);
            if (!column.paths.empty()) pool[i].push_back(std::move(column));
        }
        if (options.initial_sat_route != nullptr &&
            std::ranges::none_of(pool[i], [&](const auto& column) {
                return same_column(column, sat_columns[i]);
            })) pool[i].push_back(sat_columns[i]);
    }
    std::size_t min_lmin = std::numeric_limits<std::size_t>::max();
    std::size_t max_lmin = 0;
    for (const auto& [_, length] : initial_bus_lengths) {
        min_lmin = std::min(min_lmin, length);
        max_lmin = std::max(max_lmin, length);
    }
    if (options.big_m_mode != RouteBigMMode::Default) {
        if (initial_bus_lengths.empty())
            throw std::invalid_argument("selected M mode requires a SyncBus");
        for (std::size_t i = 0; i < owners.size(); ++i)
            if (net_for_owner(nets, owners[i]).is_sync_bus &&
                shortest[i].paths.empty())
                throw std::invalid_argument(
                    "selected M mode requires an initial path for every SyncBus lane");
    }
    std::size_t max_initial_owner_wirelength = 0;
    std::size_t initial_covered_owners = 0;
    for (const auto& columns : pool)
        if (!columns.empty()) {
            ++initial_covered_owners;
            for (const auto& column : columns)
                max_initial_owner_wirelength = std::max(max_initial_owner_wirelength,
                                                       column.wirelength);
        }
    double big_m = 10000.0;
    switch (options.big_m_mode) {
        case RouteBigMMode::Default: break;
        case RouteBigMMode::GapOne:
        case RouteBigMMode::GapTwo:
        case RouteBigMMode::GapThree:
        case RouteBigMMode::GapFour: {
            if (initial_covered_owners == 0)
                throw std::invalid_argument("selected M mode requires an initial route column");
            const int divisor = options.big_m_mode == RouteBigMMode::GapOne ? 1 :
                options.big_m_mode == RouteBigMMode::GapTwo ? 2 :
                options.big_m_mode == RouteBigMMode::GapThree ? 3 : 4;
            const double base = static_cast<double>(max_lmin) + 1.0;
            big_m = base + (static_cast<double>(max_initial_owner_wirelength) - base) /
                             divisor;
            break;
        }
    }
    result.big_m = big_m;
    debug::info_fmt("route ILP M selection: mode={} min_Lmin={} max_Lmin={} initial_max_owner={} covered_owners={}/{} M={}",
        route_big_m_mode_name(options.big_m_mode),
        initial_bus_lengths.empty() ? "n/a" : std::to_string(min_lmin),
        initial_bus_lengths.empty() ? "n/a" : std::to_string(max_lmin),
        max_initial_owner_wirelength, initial_covered_owners, owners.size(), big_m);
    debug::info_fmt("route ILP initial: owners={} columns={} bus_groups={} M={}",
        owners.size(), std::ranges::fold_left(pool, std::size_t{},
            [](std::size_t n, const auto& values) { return n + values.size(); }),
        result.bus_lengths.size(), big_m);
    bool first_log = true;
    int stagnant = 0;
    double previous_lp_objective = std::numeric_limits<double>::quiet_NaN();
    int round = 0;
    auto mip = MasterSolution{};
    while (true) {
    for (; round < 64; ++round) {
        if (remaining() <= 0.0) break;
        auto lp = solve_master(pool, false, big_m, options, first_log,
                               remaining());
        first_log = false;
        debug::info_fmt("route ILP LP round={} status={} objective={} vars={} rows={} nonzeros={}",
            round, lp.status, lp.objective, lp.vars, lp.rows, lp.nonzeros);
        if (!lp.feasible) break;
        double wirelength_objective = 0.0;
        for (std::size_t i = 0; i < owners.size(); ++i)
            for (std::size_t j = 0; j < pool[i].size(); ++j)
                wirelength_objective += pool[i][j].wirelength * lp.x[i][j];
        const bool has_previous = std::isfinite(previous_lp_objective);
        const double change = has_previous ? previous_lp_objective - lp.objective : 0.0;
        stagnant = has_previous && change >= 0.0 &&
            change < 0.001 * wirelength_objective + 1e-9 ? stagnant + 1 : 0;
        previous_lp_objective = lp.objective;
        debug::info_fmt("route ILP LP progress: round={} wirelength_part={} objective_drop={} stagnant={}/5",
                        round, wirelength_objective,
                        has_previous ? std::to_string(change) : "n/a", stagnant);
        if (options.verbose_level >= 2)
            for (std::size_t i = 0; i < owners.size(); ++i) {
                if (pool[i].empty() || lp.x[i].empty()) {
                    debug::info_fmt("route ILP LP base: net={} demand={} s={} path=none",
                        owners[i].net_id, owners[i].demand_id, lp.slack[i]);
                    continue;
                }
                const auto top = static_cast<std::size_t>(std::distance(lp.x[i].begin(),
                    std::max_element(lp.x[i].begin(), lp.x[i].end())));
                if (lp.x[i][top] <= 1e-6) {
                    debug::info_fmt("route ILP LP base: net={} demand={} s={} path=none",
                        owners[i].net_id, owners[i].demand_id, lp.slack[i]);
                    continue;
                }
                debug::info_fmt("route ILP LP base: net={} demand={} column={} x={} wirelength={}",
                    owners[i].net_id, owners[i].demand_id, top, lp.x[i][top],
                    pool[i][top].wirelength);
                log_column_paths(graph, pool[i][top], "route ILP LP base");
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
                if (resource.kind == 0 || resource.kind == 3)
                    predicted[resource].insert(i);
        }
        for (const auto& [resource, dual] : lp.resource_dual)
            if (dual < -1e-8) predicted.try_emplace(resource);
        if (options.verbose_level >= 2) {
            const auto most_negative = std::min_element(lp.resource_dual.begin(),
                lp.resource_dual.end(), [](const auto& a, const auto& b) {
                    return a.second < b.second;
                });
            if (most_negative != lp.resource_dual.end() && most_negative->second < -1e-8)
                debug::info_fmt("route ILP hotspot: most negative pi_e={} node={}",
                    most_negative->second,
                    format_resource_node(graph, most_negative->first, switch_nodes));
            else debug::info("route ILP hotspot: most negative pi_e=none");
            const auto most_congested = std::max_element(predicted.begin(),
                predicted.end(), [](const auto& a, const auto& b) {
                    return a.second.size() < b.second.size();
                });
            if (most_congested != predicted.end() && !most_congested->second.empty())
                debug::info_fmt("route ILP hotspot: largest eta_e-u_e={} eta_e={} node={}",
                    static_cast<int>(most_congested->second.size()) - 1,
                    most_congested->second.size(),
                    format_resource_node(graph, most_congested->first, switch_nodes));
            else debug::info("route ILP hotspot: largest eta_e-u_e=none");
        }
        if (stagnant >= 5) {
            debug::info("route ILP pricing stop: five consecutive LP objective drops below 0.1% of wirelength part");
            break;
        }
        auto candidate_users = std::map<RouteResource, std::set<std::size_t>>{};
        for (std::size_t i = 0; i < owners.size(); ++i)
            for (const auto& column : pool[i])
                for (const auto& resource : column.resources)
                    if (resource.kind == 0 || resource.kind == 3)
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
        auto updated = std::size_t{};
        for (const std::size_t i : selected) {
            const auto& net = net_for_owner(nets, owners[i]);
            const auto& scope = scope_for_owner(scopes, owners[i]);
            auto options_for_search = RouteSearchOptions{};
            options_for_search.deadline = deadline;
            if (net.is_sync_bus) options_for_search.exact_length =
                result.bus_lengths[net.net_id];
            for (const auto& [resource, dual] : lp.resource_dual)
                if (dual < -1e-8) options_for_search.prices[resource] = -dual;
            std::size_t base = 0;
            if (!pool[i].empty())
                for (std::size_t j = 1; j < pool[i].size(); ++j)
                    if (lp.x[i][j] > lp.x[i][base]) base = j;
            auto terminals = pool[i].empty() ?
                std::Vector<std::pair<double, std::size_t>>{} :
                terminal_branch_weights(graph, pool[i][base], options_for_search);
            std::sort(terminals.begin(), terminals.end(), std::greater{});
            if (!pool[i].empty())
                for (const auto& path : pool[i][base].paths)
                    for (int node : path.node_path)
                        options_for_search.discouraged_nodes.insert(node);
            const auto discouraged = options_for_search.discouraged_nodes;
            auto by_length = [](const RouteColumn& a, const RouteColumn& b) {
                return std::tuple{a.wirelength, a.paths.size()} <
                       std::tuple{b.wirelength, b.paths.size()};
            };
            auto additions = std::Vector<RouteColumn>{};
            const auto selected_terminals = terminals.empty() ? std::size_t{1} :
                std::min<std::size_t>(5, terminals.size());
            for (std::size_t t = 0; t < selected_terminals; ++t) {
                const std::size_t terminal = terminals.empty() ? 0 : terminals[t].second;
                auto negative = std::Vector<RouteColumn>{};
                auto feasibility = std::Vector<RouteColumn>{};
                auto novel = std::Vector<RouteColumn>{};
                auto proposals = std::Vector<RouteColumn>{};
                const bool multi_terminal = !pool[i].empty() &&
                    pool[i][base].paths.size() > 1;
                if (multi_terminal) {
                    options_for_search.discouraged_nodes.clear();
                    proposals = find_terminal_columns(graph, net, scope, owners[i],
                        options_for_search, pool[i][base], terminal);
                } else {
                    const int variants = selected_terminals == 1 ? 5 : 2;
                    for (int variant = 0; variant < variants; ++variant) {
                        options_for_search.variant = round * 10 + static_cast<int>(t) * 2 + variant;
                        options_for_search.discouraged_nodes = variant == 0 ?
                            std::set<int>{} : discouraged;
                        auto candidate = find_route_column(graph, net, scope, owners[i],
                            options_for_search,
                            pool[i].empty() ? nullptr : &pool[i][base], terminal);
                        if (!candidate.paths.empty()) proposals.push_back(std::move(candidate));
                    }
                }
                for (auto& candidate : proposals) {
                    if (candidate.paths.empty() || std::ranges::any_of(pool[i],
                        [&](const auto& old) { return same_column(old, candidate); }) ||
                        std::ranges::any_of(additions, [&](const auto& old) {
                            return same_column(old, candidate); }) ||
                        std::ranges::any_of(novel, [&](const auto& old) {
                            return same_column(old, candidate); }) ||
                        std::ranges::any_of(negative, [&](const auto& old) {
                            return same_column(old, candidate); }) ||
                        std::ranges::any_of(feasibility, [&](const auto& old) {
                            return same_column(old, candidate); })) continue;
                    if (multi_terminal) {
                        novel.push_back(std::move(candidate));
                        continue;
                    }
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
                std::sort(negative.begin(), negative.end(), by_length);
                std::sort(feasibility.begin(), feasibility.end(), by_length);
                std::sort(novel.begin(), novel.end(), by_length);
                auto& chosen = multi_terminal ? novel :
                    negative.empty() ? feasibility : negative;
                const auto take = std::min<std::size_t>(2, chosen.size());
                if (take > 0 && options.verbose_level >= 2) {
                    debug::info_fmt("route ILP pool update: net={} demand={} terminal={} added={} shortest_wirelength={}",
                        owners[i].net_id, owners[i].demand_id, terminal, take,
                        chosen.front().wirelength);
                    log_column_paths(graph, chosen.front(), "route ILP pool update");
                }
                for (std::size_t j = 0; j < take; ++j)
                    additions.push_back(std::move(chosen[j]));
            }
            if (!additions.empty()) ++updated;
            added += additions.size();
            for (auto& column : additions) pool[i].push_back(std::move(column));
        }
        debug::info_fmt("route ILP pricing round={} added={} updated_owners={}", round, added, updated);
        bool sync_slack = false;
        for (std::size_t i = 0; i < owners.size(); ++i)
            if (lp.slack[i] > 1e-6 &&
                net_for_owner(nets, owners[i]).is_sync_bus) sync_slack = true;
        if (added == 0 && !sync_slack) break;
    }
    mip = solve_master(pool, true, big_m, options, first_log,
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
    auto increased = std::map<std::size_t, std::size_t>{};
    if (remaining() > 0.0 && round < 64)
        for (std::size_t i = 0; i < owners.size(); ++i) {
            const auto& net = net_for_owner(nets, owners[i]);
            if (options.initial_sat_route == nullptr && net.is_sync_bus &&
                (!mip.feasible || mip.slack[i] > 0.5) &&
                initial_bus_lengths.contains(net.net_id) &&
                result.bus_lengths[net.net_id] == initial_bus_lengths.at(net.net_id))
                increased[net.net_id] = result.bus_lengths[net.net_id] + 1;
        }
    if (increased.empty()) break;
    for (const auto& [bus, length] : increased) {
        result.bus_lengths[bus] = length;
        debug::info_fmt("route ILP SyncBus length increase: bus={} L={} (retain other owner pools)",
                        bus, length);
    }
    for (std::size_t i = 0; i < owners.size(); ++i) {
        const auto& net = net_for_owner(nets, owners[i]);
        if (!net.is_sync_bus || !increased.contains(net.net_id)) continue;
        pool[i].clear();
        auto search = RouteSearchOptions{};
        search.exact_length = result.bus_lengths.at(net.net_id);
        search.deadline = deadline;
        auto column = find_route_column(graph, net,
            scope_for_owner(scopes, owners[i]), owners[i], search);
        if (!column.paths.empty()) pool[i].push_back(std::move(column));
        debug::info_fmt("route ILP SyncBus pool rebuilt: net={} demand={} columns={} L={}",
            owners[i].net_id, owners[i].demand_id, pool[i].size(), search.exact_length);
        if (options.verbose_level >= 2 && !pool[i].empty())
            log_column_paths(graph, pool[i].front(), "route ILP SyncBus pool rebuilt");
    }
    stagnant = 0;
    previous_lp_objective = std::numeric_limits<double>::quiet_NaN();
    ++round;
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
    return result;
}

auto solve_route_ilp(const UnifiedGraph& graph,
                     const std::Vector<RoutingNet>& nets,
                     const std::Vector<RoutingScope>& scopes,
                     const RouteIlpOptions& options) -> RouteIlpResult {
    return solve_route_ilp_impl(graph, nets, scopes, options,
                               std::chrono::steady_clock::now());
}

} // namespace PR_tool
