#include "precompute/pre_routing_warm_start.hh"

#include "ilp_allocation/tob_ilp_model.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/routeerror.hh>
#include <circuit/basedie.hh>
#include <circuit/net/net.hh>
#include <circuit/net/types/syncnet.hh>
#include <debug/debug.hh>
#include <hardware/bump/bump.hh>
#include <hardware/interposer.hh>
#include <hardware/tob/tob.hh>
#include <parse/reader/module.hh>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <tuple>

namespace PR_tool {
namespace {

auto bump_to_coord(const hardware::Bump* bump) -> Bump_coord {
    const auto bump_index = bump->index();
    const auto tob_coord = bump->tob()->coord();
    return Bump_coord {
        static_cast<std::size_t>(tob_coord.row * hardware::Interposer::TOB_ARRAY_WIDTH + tob_coord.col),
        bump_index / 64,
        (bump_index % 64) / 8,
        bump_index % 8
    };
}

auto set_warm_value(TobIlpWarmStart& warm, const std::String& name, const double value) -> void {
    const auto it = warm.values.find(name);
    if (it == warm.values.end()) {
        warm.values.emplace(name, value);
        return;
    }
    if (std::fabs(it->second - value) > 1e-9) {
        debug::warning_fmt(
            "pre-routing ILP warm start: conflicting value for {} (kept {}, ignored {})",
            name,
            it->second,
            value);
    }
}

auto track_to_jk_straight(const Bump_coord& b, const std::size_t track)
    -> std::optional<std::tuple<std::size_t, std::size_t, bool>> {
    std::size_t v = 0;
    bool use_straight = false;
    if (b.Bank == 0) {
        if (track < 64) {
            v = track;
            use_straight = true;
        }
        else {
            v = track - 64;
            use_straight = false;
        }
    }
    else {
        if (track >= 64) {
            v = track - 64;
            use_straight = true;
        }
        else {
            v = track;
            use_straight = false;
        }
    }
    if (v >= 64) {
        return std::nullopt;
    }
    return std::tuple<std::size_t, std::size_t, bool> {v / 8, v % 8, use_straight};
}

auto set_track_choice_values(TobIlpWarmStart& warm, const Bump_coord& b, const std::size_t track) -> bool {
    const auto choice = track_to_jk_straight(b, track);
    if (!choice.has_value()) {
        return false;
    }
    const auto [j, k, use_straight] = *choice;
    const auto v = j * 8 + k;
    set_warm_value(warm, w_var(b, j, k), 1.0);
    set_warm_value(warm, s_var(b.TOB, v), use_straight ? 1.0 : 0.0);
    set_warm_value(warm, qs_var(b, j, k), use_straight ? 1.0 : 0.0);
    set_warm_value(warm, qw_var(b, j, k), use_straight ? 0.0 : 1.0);
    return true;
}

auto add_tob_choice(
    TobIlpWarmStart& warm,
    std::map<Bump_coord, std::size_t>& chosen_track_by_bump,
    const hardware::Bump* bump,
    const hardware::TOBConnector& connector
) -> void {
    const auto b = bump_to_coord(bump);
    const auto v = connector.vert_index() % 64;
    const auto j = v / 8;
    const auto k = v % 8;
    const auto track = connector.track_index();
    const auto straight_track = b.Bank == 0 ? v : v + 64;
    const auto wrap_track = b.Bank == 0 ? v + 64 : v;

    if (track != straight_track && track != wrap_track) {
        debug::warning_fmt(
            "pre-routing ILP warm start: skip bump(T{},B{},G{},I{}) unexpected connector track {} for vert {}",
            b.TOB,
            b.Bank,
            b.Group,
            b.Index,
            track,
            connector.vert_index());
        return;
    }

    const bool use_straight = track == straight_track;
    (void)j;
    (void)k;
    (void)use_straight;
    if (!set_track_choice_values(warm, b, track)) {
        return;
    }

    const auto it = chosen_track_by_bump.find(b);
    if (it == chosen_track_by_bump.end()) {
        chosen_track_by_bump.emplace(b, track);
    }
    else if (it->second != track) {
        debug::warning_fmt(
            "pre-routing ILP warm start: bump(T{},B{},G{},I{}) saw multiple tracks (kept {}, ignored {})",
            b.TOB,
            b.Bank,
            b.Group,
            b.Index,
            it->second,
            track);
    }
}

struct GreedyTrackState {
    std::map<Bump_coord, std::size_t> chosen_track_by_bump;
    std::set<std::tuple<std::size_t, std::size_t, std::size_t, std::size_t>> used_hori;
    std::set<std::tuple<std::size_t, std::size_t, std::size_t, std::size_t>> used_vert;
};

auto seed_greedy_state(GreedyTrackState& state, const std::map<Bump_coord, std::size_t>& chosen) -> void {
    for (const auto& [b, track] : chosen) {
        const auto choice = track_to_jk_straight(b, track);
        if (!choice.has_value()) {
            continue;
        }
        const auto [j, k, straight] = *choice;
        (void)straight;
        state.chosen_track_by_bump.emplace(b, track);
        state.used_hori.emplace(b.TOB, b.Bank, b.Group, j);
        state.used_vert.emplace(b.TOB, b.Bank, j, k);
    }
}

auto can_assign_track(const GreedyTrackState& state, const Bump_coord& b, const std::size_t track) -> bool {
    if (const auto it = state.chosen_track_by_bump.find(b); it != state.chosen_track_by_bump.end()) {
        return it->second == track;
    }
    const auto choice = track_to_jk_straight(b, track);
    if (!choice.has_value()) {
        return false;
    }
    const auto [j, k, straight] = *choice;
    (void)straight;
    return !state.used_hori.contains({b.TOB, b.Bank, b.Group, j})
        && !state.used_vert.contains({b.TOB, b.Bank, j, k});
}

auto assign_track(
    TobIlpWarmStart& warm,
    GreedyTrackState& state,
    const Bump_coord& b,
    const std::size_t track
) -> bool {
    if (const auto it = state.chosen_track_by_bump.find(b); it != state.chosen_track_by_bump.end()) {
        return it->second == track;
    }
    if (!can_assign_track(state, b, track) || !set_track_choice_values(warm, b, track)) {
        return false;
    }
    const auto [j, k, straight] = *track_to_jk_straight(b, track);
    (void)straight;
    state.chosen_track_by_bump.emplace(b, track);
    state.used_hori.emplace(b.TOB, b.Bank, b.Group, j);
    state.used_vert.emplace(b.TOB, b.Bank, j, k);
    return true;
}

auto fill_missing_ilp_choices(
    TobIlpWarmStart& warm,
    const std::Vector<Net_cost_record>& records,
    std::map<Bump_coord, std::size_t>& chosen_track_by_bump
) -> void {
    auto state = GreedyTrackState {};
    seed_greedy_state(state, chosen_track_by_bump);

    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto& record = records[n];
        if (record.type == Net_type::Bnet) {
            if (record.start_bumps.empty() || record.end_bumps.empty()) {
                continue;
            }
            const auto& start_bump = record.start_bumps.front();
            const auto& end_bump = record.end_bumps.front();
            for (const auto& [end_track, starts] : record.starttrack_by_endtrack) {
                for (const auto start_track : starts) {
                    if (!can_assign_track(state, start_bump, start_track) || !can_assign_track(state, end_bump, end_track)) {
                        continue;
                    }
                    (void)assign_track(warm, state, start_bump, start_track);
                    (void)assign_track(warm, state, end_bump, end_track);
                    goto next_record;
                }
            }
        }
        else if (record.type == Net_type::Tnet) {
            if (record.start_bumps.empty() || record.starttrack_by_endtrack.empty()) {
                continue;
            }
            const auto& start_bump = record.start_bumps.front();
            for (const auto& [end_track, starts] : record.starttrack_by_endtrack) {
                (void)end_track;
                for (const auto start_track : starts) {
                    if (assign_track(warm, state, start_bump, start_track)) {
                        goto next_record;
                    }
                }
            }
        }
        else if (record.type == Net_type::PNnet) {
            if (record.start_bumps.empty() || record.starttrack_by_endtrack.empty()) {
                continue;
            }
            const auto& start_bump = record.start_bumps.front();
            bool selected_y = false;
            if (const auto it = state.chosen_track_by_bump.find(start_bump); it != state.chosen_track_by_bump.end()) {
                for (const auto& [end_track, starts] : record.starttrack_by_endtrack) {
                    if (std::find(starts.begin(), starts.end(), it->second) != starts.end()) {
                        set_warm_value(warm, y_var(n, end_track), 1.0);
                        selected_y = true;
                        break;
                    }
                }
            }
            if (!selected_y) {
                for (const auto& [end_track, starts] : record.starttrack_by_endtrack) {
                    for (const auto start_track : starts) {
                        if (!assign_track(warm, state, start_bump, start_track)) {
                            continue;
                        }
                        set_warm_value(warm, y_var(n, end_track), 1.0);
                        selected_y = true;
                        break;
                    }
                    if (selected_y) {
                        break;
                    }
                }
            }
        }
next_record:
        continue;
    }

    chosen_track_by_bump = std::move(state.chosen_track_by_bump);
}

auto collect_package_choices(
    TobIlpWarmStart& warm,
    std::map<Bump_coord, std::size_t>& chosen_track_by_bump,
    const circuit::PathPackage& package
) -> void {
    for (const auto& entry : package._tob_to_track) {
        const auto& [bump, connector, track] = entry;
        (void)track;
        add_tob_choice(warm, chosen_track_by_bump, bump, connector);
    }
    for (const auto& entry : package._track_to_tob) {
        const auto& [bump, connector, track] = entry;
        (void)track;
        add_tob_choice(warm, chosen_track_by_bump, bump, connector);
    }
}

auto collect_net_choices(
    TobIlpWarmStart& warm,
    std::map<Bump_coord, std::size_t>& chosen_track_by_bump,
    circuit::Net* net
) -> void {
    if (auto* sync = dynamic_cast<circuit::SyncNet*>(net)) {
        for (const auto& sub : sync->btbnets()) {
            collect_package_choices(warm, chosen_track_by_bump, sub->pathpackage());
        }
        for (const auto& sub : sync->bttnets()) {
            collect_package_choices(warm, chosen_track_by_bump, sub->pathpackage());
        }
        for (const auto& sub : sync->ttbnets()) {
            collect_package_choices(warm, chosen_track_by_bump, sub->pathpackage());
        }
        return;
    }
    collect_package_choices(warm, chosen_track_by_bump, net->pathpackage());
}

auto route_shadow_nets(hardware::Interposer* interposer, circuit::BaseDie* basedie, TobIlpWarmStart& warm) -> void {
    auto nets_rc = basedie->nets_to_vector();
    auto nets = std::Vector<circuit::Net*> {};
    nets.reserve(nets_rc.size());
    for (const auto& net : nets_rc) {
        nets.push_back(net.get());
    }

    for (auto* net : nets) {
        net->check_accessable_cobunit();
    }
    interposer->manage_cobunit_resources();

    float max_port_num = 0.0F;
    for (const auto* net : nets) {
        max_port_num = std::max(max_port_num, static_cast<float>(net->port_number()));
    }
    if (max_port_num > 0.0F) {
        for (auto* net : nets) {
            net->update_priority(0.9F * (static_cast<float>(net->port_number()) / max_port_num));
        }
    }
    std::sort(nets.begin(), nets.end(), [](const circuit::Net* lhs, const circuit::Net* rhs) {
        return lhs->priority() > rhs->priority();
    });

    const auto maze = algo::MazeRouteStrategy {};
    auto routed_nets = std::Vector<circuit::Net*> {};
    for (auto* net : nets) {
        try {
            net->search_related_nets(routed_nets);
            net->route(interposer, maze);
            routed_nets.push_back(net);
            ++warm.routed_nets;
        }
        catch (const algo::RouteExpt& e) {
            ++warm.failed_nets;
            debug::warning_fmt("pre-routing maze failed ({}): {}", net->name(), e.what());
        }
        catch (const std::exception& e) {
            ++warm.failed_nets;
            debug::warning_fmt("pre-routing maze failed ({}): {}", net->name(), e.what());
        }
    }

}

auto add_pn_y_choices(
    TobIlpWarmStart& warm,
    const std::Vector<Net_cost_record>& records,
    const std::map<Bump_coord, std::size_t>& chosen_track_by_bump
) -> void {
    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto& record = records[n];
        if (record.type != Net_type::PNnet || record.start_bumps.empty()) {
            continue;
        }
        const auto tr_it = chosen_track_by_bump.find(record.start_bumps.front());
        if (tr_it == chosen_track_by_bump.end()) {
            continue;
        }
        const auto start_track = tr_it->second;
        for (const auto& [end_track, starts] : record.starttrack_by_endtrack) {
            if (std::find(starts.begin(), starts.end(), start_track) == starts.end()) {
                continue;
            }
            set_warm_value(warm, y_var(n, end_track), 1.0);
            break;
        }
    }
}

} // namespace

auto build_ilp_warm_start_from_maze(
    const std::FilePath& config_path,
    const std::Vector<Net_cost_record>& records
) -> TobIlpWarmStart {
    auto warm = TobIlpWarmStart {};
    debug::info("pre-routing: running shadow MazeRouteStrategy before ILP");
    auto [shadow_interposer, shadow_basedie] = parse::read_config(config_path, 0, false);
    algo::build_nets(shadow_basedie.get(), shadow_interposer.get());

    route_shadow_nets(shadow_interposer.get(), shadow_basedie.get(), warm);

    auto chosen_track_by_bump = std::map<Bump_coord, std::size_t> {};
    for (const auto& net : shadow_basedie->nets_to_vector()) {
        collect_net_choices(warm, chosen_track_by_bump, net.get());
    }
    fill_missing_ilp_choices(warm, records, chosen_track_by_bump);
    add_pn_y_choices(warm, records, chosen_track_by_bump);

    debug::info_fmt(
        "pre-routing ILP warm start: routed_nets={} failed_nets={} variable_values={}",
        warm.routed_nets,
        warm.failed_nets,
        warm.values.size());
    return warm;
}

} // namespace PR_tool
