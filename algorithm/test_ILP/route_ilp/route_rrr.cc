#include "route_ilp/route_rrr.hh"

#include "common/route_metrics.hh"
#include "direct_ilp/direct_validate.hh"

#include <debug/debug.hh>

#include <algorithm>
#include <chrono>
#include <deque>
#include <limits>
#include <stdexcept>

namespace PR_tool {
namespace {

using Selection = std::map<RouteOwner, RouteColumn>;

auto find_arc(const UnifiedGraph& graph, int u, int v) -> int {
    for (int id : graph.out_arc_ids[static_cast<std::size_t>(u)])
        if (graph.arcs[static_cast<std::size_t>(id)].v == v) return id;
    return -1;
}

auto assemble(const UnifiedGraph& graph, const Selection& selected,
              std::size_t owner_count) -> RoutingResult {
    auto route = RoutingResult{};
    route.ok = selected.size() == owner_count;
    route.message = route.ok ? "COMPLETE" : "PARTIAL";
    auto switches = std::set<int>{};
    for (const auto& [_, column] : selected)
        for (const auto& path : column.paths) {
            route.paths.push_back(path);
            for (std::size_t j = 1; j < path.node_path.size(); ++j) {
                const int aid = find_arc(graph, path.node_path[j - 1], path.node_path[j]);
                if (aid < 0) throw std::logic_error("RRR assembled missing arc");
                const auto& arc = graph.arcs[static_cast<std::size_t>(aid)];
                if (arc.physical_switch_id >= 0) switches.insert(arc.physical_switch_id);
                if (arc.mode_group_id >= 0)
                    route.vline_mode_straight_by_group[
                        static_cast<std::size_t>(arc.mode_group_id)] =
                        arc.is_vline_track_straight;
            }
        }
    route.used_tob_switch_ids.assign(switches.begin(), switches.end());
    route.total_wirelength = total_wirelength(graph, route);
    return route;
}

auto no_conflicts(const Selection& selected) -> bool {
    for (auto first = selected.begin(); first != selected.end(); ++first)
        for (auto second = std::next(first); second != selected.end(); ++second)
            if (route_columns_conflict(first->second, second->second)) return false;
    return true;
}

auto search(const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
            const std::Vector<RoutingScope>& scopes, RouteOwner owner,
            const Selection& occupied,
            const std::map<std::size_t, std::size_t>& bus_lengths,
            int variant, bool hard_avoid,
            const std::map<RouteResource, double>& history,
            const RouteColumn* former = nullptr) -> RouteColumn {
    auto options = RouteSearchOptions{};
    options.variant = variant;
    options.prices = history;
    const auto& net = net_for_owner(nets, owner);
    if (net.is_sync_bus) {
        const auto it = bus_lengths.find(net.net_id);
        if (it == bus_lengths.end()) return RouteColumn{owner};
        options.exact_length = it->second;
    }
    for (const auto& [_, candidate] : occupied)
        for (const auto& resource : candidate.resources)
            options.prices[resource] += hard_avoid ? 1000.0 : 10.0;
    if (former != nullptr && variant > 0)
        for (const auto& path : former->paths)
            for (int node : path.node_path)
                options.discouraged_nodes.insert(node);
    if (net.is_sync_bus && former != nullptr && variant >= 1 && variant <= 3) {
        const int tail_percent = variant == 1 ? 50 : variant == 2 ? 75 : 100;
        const auto cut_tail = find_sync_prefix_column(graph, net,
            scope_for_owner(scopes, owner), owner, options, *former, tail_percent);
        if (!cut_tail.paths.empty()) return cut_tail;
    }
    return find_route_column(graph, net, scope_for_owner(scopes, owner), owner,
                             options);
}

auto missing_count(std::size_t owners, const Selection& selection) -> std::size_t {
    return owners - selection.size();
}

auto transaction(const UnifiedGraph& graph,
                 const std::Vector<RoutingNet>& nets,
                 const std::Vector<RoutingScope>& scopes,
                 const std::map<std::size_t, std::size_t>& bus_lengths,
                 const Selection& incumbent, RouteOwner seed,
                 int variant, const std::map<RouteResource, double>& history)
    -> Selection {
    auto working = incumbent;
    auto former = RouteColumn{};
    const auto old = working.find(seed);
    if (old != working.end()) {
        former = old->second;
        working.erase(old);
    }
    const auto fresh = search(graph, nets, scopes, seed, working, bus_lengths,
        variant, false, history, former.paths.empty() ? nullptr : &former);
    if (fresh.paths.empty()) return {};
    auto displaced = std::Vector<RouteOwner>{};
    for (const auto& [other, column] : working)
        if (route_columns_conflict(fresh, column)) displaced.push_back(other);
    for (const auto other : displaced) working.erase(other);
    working.emplace(seed, fresh);
    auto pending = std::deque<RouteOwner>(displaced.begin(), displaced.end());
    auto locked = std::set<RouteOwner>{seed};
    while (!pending.empty()) {
        const auto other = pending.front(); pending.pop_front();
        const auto old_column = incumbent.at(other);
        auto best = RouteColumn{};
        auto best_conflicts = std::Vector<RouteOwner>{};
        std::size_t best_count = std::numeric_limits<std::size_t>::max();
        for (int attempt = 0; attempt < 4; ++attempt) {
            auto replacement = search(graph, nets, scopes, other, working,
                bus_lengths, variant + attempt, true, history, &old_column);
            if (replacement.paths.empty()) continue;
            auto conflicts = std::Vector<RouteOwner>{};
            bool invalid = false;
            for (const auto& [owner, column] : working)
                if (route_columns_conflict(replacement, column)) {
                    if (locked.contains(owner)) { invalid = true; break; }
                    conflicts.push_back(owner);
                }
            if (!invalid && conflicts.size() < best_count) {
                best = std::move(replacement);
                best_conflicts = std::move(conflicts);
                best_count = best_conflicts.size();
                if (best_count == 0) break;
            }
        }
        if (best.paths.empty()) return {};
        for (const auto conflict : best_conflicts) {
            working.erase(conflict);
            pending.push_back(conflict);
        }
        working.emplace(other, std::move(best));
        locked.insert(other);
    }
    return working;
}

} // namespace

auto optimize_route_columns_rrr(const UnifiedGraph& graph,
                                const std::Vector<RoutingNet>& nets,
                                const std::Vector<RoutingScope>& scopes,
                                const RouteIlpResult& baseline,
                                int /*verbose_level*/,
                                std::chrono::steady_clock::time_point deadline)
    -> RoutingResult {
    const auto begin = std::chrono::steady_clock::now();
    const auto owners = route_owners(nets);
    auto selected = Selection{};
    for (const auto owner : owners) {
        auto paths = std::Vector<SourceSinkPairPath>{};
        const auto& net = net_for_owner(nets, owner);
        for (const auto& path : baseline.route.paths)
            if (path.net_id == owner.net_id &&
                (!net.is_sync_bus || path.demand_id == owner.demand_id))
                paths.push_back(path);
        if (!paths.empty())
            selected.emplace(owner, route_column_from_paths(graph, net, owner,
                                                              paths));
    }
    if (!no_conflicts(selected))
        throw std::logic_error("RRR baseline has resource conflict");
    auto actual_missing = std::set<RouteOwner>{};
    for (const auto owner : owners)
        if (!selected.contains(owner)) actual_missing.insert(owner);
    if (actual_missing != baseline.missing)
        throw std::logic_error("RRR baseline slack and paths disagree");
    auto incumbent = assemble(graph, selected, owners.size());
    if (!baseline.has_integer_solution ||
        !validate_partial_route(graph, nets, scopes, incumbent,
                                baseline.bus_lengths))
        throw std::logic_error("RRR baseline is not a legal partial route");
    const auto original_wirelength = incumbent.total_wirelength;
    auto accepted = 0;
    auto attempts = 0;
    auto history = std::map<RouteResource, double>{};
    debug::info_fmt("route RRR start: owners={} missing={} wirelength={}",
        owners.size(), missing_count(owners.size(), selected),
        incumbent.total_wirelength);
    for (int sweep = 0; sweep < 2; ++sweep) {
        bool improved = false;
        auto order = std::Vector<RouteOwner>{};
        for (const auto owner : owners)
            if (!selected.contains(owner)) order.push_back(owner);
        for (const auto owner : owners)
            if (selected.contains(owner)) order.push_back(owner);
        for (const auto owner : order) {
            if (std::chrono::steady_clock::now() >= deadline) break;
            for (int variant = 0; variant < 6; ++variant) {
                if (std::chrono::steady_clock::now() >= deadline) break;
                ++attempts;
                auto trial = transaction(graph, nets, scopes,
                    baseline.bus_lengths, selected, owner, variant, history);
                if (trial.empty() || !no_conflicts(trial)) {
                    const auto current = selected.find(owner);
                    if (current != selected.end())
                        for (const auto& resource : current->second.resources)
                            history[resource] += 0.25;
                    continue;
                }
                auto candidate = assemble(graph, trial, owners.size());
                if (!validate_partial_route(graph, nets, scopes, candidate,
                                            baseline.bus_lengths)) continue;
                const auto old_missing = missing_count(owners.size(), selected);
                const auto new_missing = missing_count(owners.size(), trial);
                if (new_missing > old_missing ||
                    (new_missing == old_missing &&
                     candidate.total_wirelength >= incumbent.total_wirelength))
                    continue;
                debug::info_fmt("route RRR accept: sweep={} net={} demand={} missing={}->{} wirelength={}->{}",
                    sweep, owner.net_id, owner.demand_id, old_missing, new_missing,
                    incumbent.total_wirelength, candidate.total_wirelength);
                selected = std::move(trial);
                incumbent = std::move(candidate);
                improved = true; ++accepted;
                break;
            }
        }
        if (!improved) break;
    }
    incumbent.rrr_attempted = true;
    incumbent.rrr_accepted = accepted > 0;
    const bool sync_missing = std::ranges::any_of(owners,
        [&](RouteOwner owner) {
            return !selected.contains(owner) &&
                   net_for_owner(nets, owner).is_sync_bus;
        });
    incumbent.rrr_status = incumbent.ok ?
        (accepted > 0 ? "COMPLETE_IMPROVED" : "COMPLETE_UNCHANGED") :
        std::chrono::steady_clock::now() >= deadline ? "TIME_LIMIT" :
        sync_missing ? "NO_SOLUTION_WITHIN_SYNC_LENGTH_CAP" : "INCOMPLETE";
    incumbent.rrr_triggers = attempts;
    incumbent.rrr_accepted_triggers = accepted;
    incumbent.rrr_baseline_wirelength = original_wirelength;
    incumbent.rrr_wirelength = incumbent.total_wirelength;
    incumbent.rrr_total_ms = std::chrono::duration_cast<
        std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count();
    debug::info_fmt("route RRR summary: status={} missing={} accepted={} attempts={} wirelength={}->{} total_ms={}",
        incumbent.rrr_status, missing_count(owners.size(), selected), accepted,
        attempts, original_wirelength, incumbent.total_wirelength,
        incumbent.rrr_total_ms);
    for (const auto owner : owners)
        if (!selected.contains(owner))
            debug::info_fmt("route RRR missing: net={} demand={}",
                            owner.net_id, owner.demand_id);
    return incumbent;
}

} // namespace PR_tool
