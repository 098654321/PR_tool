#pragma once

#include "transaction_search.hh"
#include "route_validate.hh"

#include <deque>

namespace PR_tool::rrr_detail {

inline auto unit_indices(const std::Vector<OwnerRecord>& owners, std::size_t net_index)
    -> std::Vector<std::size_t> {
    auto result = std::Vector<std::size_t>{};
    for (std::size_t i = 0; i < owners.size(); ++i)
        if (owners[i].net_index == net_index) result.push_back(i);
    return result;
}

inline auto owner_connected(const OwnerRecord& owner) -> bool {
    return !owner.demand_paths.empty() && std::ranges::all_of(owner.demand_paths,
        [](const auto& p) { return !p.empty(); });
}

inline auto missing_owners(const std::Vector<OwnerRecord>& owners) -> int {
    return static_cast<int>(std::ranges::count_if(owners,
        [](const auto& owner) { return !owner_connected(owner); }));
}

inline auto unit_connected(const std::Vector<OwnerRecord>& owners, std::size_t ni) -> bool {
    for (auto i : unit_indices(owners, ni)) if (!owner_connected(owners[i])) return false;
    return true;
}

inline auto rip_unit(std::Vector<OwnerRecord>& owners, ResourceModel& resources, std::size_t ni) -> void {
    for (auto i : unit_indices(owners, ni)) rip_owner(owners[i], resources);
}

inline auto units_conflict(const std::Vector<OwnerRecord>& owners, std::size_t a, std::size_t b) -> bool {
    auto model = ResourceModel{};
    for (const auto& owner : owners)
        if (owner.net_index == a || owner.net_index == b)
            model.claim(owner.id, std::Vector<ResourceKey>(owner.claimed.begin(), owner.claimed.end()));
    return model.overflow() != 0;
}

inline auto route_unit(std::size_t ni, std::Vector<OwnerRecord>& owners,
                       const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                       ResourceModel& resources, RrrParams params,
                       hardware::Interposer* interposer) -> bool {
    if (!nets[ni].is_sync_bus) {
        const auto indices = unit_indices(owners, ni);
        return search_transaction_owner(owners[indices.front()], graph, nets, resources, params);
    }
    if (params.variant >= 1 && params.variant <= 3)
        params.r_sequence = {params.variant == 1 ? 0.5 : params.variant == 2 ? 0.75 : 1.0};
    try {
        if (!route_sync_group(ni, owners, graph, nets, resources, params, interposer)) return false;
    } catch (const std::runtime_error& error) {
        if (std::string_view(error.what()).starts_with("maze:")) return false;
        throw;
    }
    for (auto i : unit_indices(owners, ni)) if (!owner_paths_valid(graph, owners[i])) return false;
    return true;
}

// Same displacement queue, locking and four-research rule as test_ILP RRR.
// A SyncNet is an atomic unit because a target advance must roll back all lanes.
inline auto transaction(std::size_t seed, int variant,
                        const UnifiedGraph& graph, const std::Vector<RoutingNet>& nets,
                        std::Vector<OwnerRecord>& owners, ResourceModel& resources,
                        RrrParams params, hardware::Interposer* interposer) -> bool {
    const auto incumbent = owners;
    params.variant = variant;
    rip_unit(owners, resources, seed);
    if (!route_unit(seed, owners, graph, nets, resources, params, interposer)) return false;
    auto pending = std::deque<std::size_t>{};
    auto locked = std::set<std::size_t>{seed};
    for (std::size_t ni = 0; ni < nets.size(); ++ni)
        if (ni != seed && unit_connected(owners, ni) && units_conflict(owners, seed, ni)) {
            rip_unit(owners, resources, ni); pending.push_back(ni);
        }
    while (!pending.empty()) {
        const auto ni = pending.front(); pending.pop_front();
        auto best = std::Vector<OwnerRecord>{};
        auto best_resources = resources;
        auto best_conflicts = std::Vector<std::size_t>{};
        auto best_count = std::numeric_limits<std::size_t>::max();
        for (int attempt = 0; attempt < 4; ++attempt) {
            auto trial = owners;
            auto trial_resources = resources;
            params.variant = variant + attempt;
            // Preserve the current group's target even while its paths are absent.
            for (auto index : unit_indices(trial, ni)) trial[index].sync_target = incumbent[index].sync_target;
            if (!route_unit(ni, trial, graph, nets, trial_resources, params, interposer)) continue;
            auto conflicts = std::Vector<std::size_t>{};
            bool invalid = false;
            for (std::size_t other = 0; other < nets.size(); ++other)
                if (other != ni && unit_connected(trial, other) && units_conflict(trial, ni, other)) {
                    if (locked.contains(other)) { invalid = true; break; }
                    conflicts.push_back(other);
                }
            if (!invalid && conflicts.size() < best_count) {
                best = std::move(trial); best_resources = std::move(trial_resources);
                best_count = conflicts.size(); best_conflicts = std::move(conflicts);
                if (best_count == 0) break;
            }
        }
        if (best.empty()) return false;
        owners = std::move(best); resources = std::move(best_resources);
        for (auto other : best_conflicts) { rip_unit(owners, resources, other); pending.push_back(other); }
        locked.insert(ni);
    }
    if (resources.overflow() != 0) return false;
    auto candidate = RrrResult{}; candidate.paths = collect_paths_by_net(nets, owners);
    return validate_rrr_solution(graph, nets, candidate, interposer, true);
}

} // namespace PR_tool::rrr_detail
