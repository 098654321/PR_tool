#include "ilp_allocation/ilp_apply_interposer.hh"

#include "common/ilp_types.hh"

#include <debug/debug.hh>
#include <hardware/interposer.hh>
#include <hardware/tob/tob.hh>
#include <hardware/track/trackcoord.hh>

#include <format>
#include <map>

namespace PR_tool {

namespace {

auto tob_from_linear(const std::size_t tob_linear) -> hardware::TOBCoord {
    const auto width = static_cast<std::size_t>(hardware::Interposer::TOB_ARRAY_WIDTH);
    return hardware::TOBCoord {
        static_cast<std::i64>(tob_linear / width),
        static_cast<std::i64>(tob_linear % width)};
}

auto bump_index_from_coord(const Bump_coord& bump) -> std::size_t {
    return bump.Bank * 64 + bump.Group * 8 + bump.Index;
}

auto find_bump(hardware::Interposer* interposer, const Bump_coord& bump_coord) -> hardware::Bump* {
    const auto tob = interposer->get_tob(tob_from_linear(bump_coord.TOB));
    if (!tob.has_value()) {
        return nullptr;
    }
    const auto bump_index = bump_index_from_coord(bump_coord);
    const auto bump = (*tob)->get_bump(bump_index);
    if (!bump.has_value()) {
        return nullptr;
    }
    return bump.value();
}

} // namespace

auto apply_tob_ilp_result_to_interposer(
    hardware::Interposer* interposer,
    const TobIlpResult& result
) -> void {
    if (interposer == nullptr) {
        return;
    }

    auto hori_mux_by_tob_v = std::map<std::pair<std::size_t, std::size_t>, std::size_t> {};
    for (const auto& w : result.active_w) {
        if (!w.has_track) {
            continue;
        }
        const auto key = std::pair<std::size_t, std::size_t> {w.bump.TOB, w.j * 8 + w.k};
        if (!hori_mux_by_tob_v.contains(key)) {
            hori_mux_by_tob_v[key] = bump_index_from_coord(w.bump) / hardware::TOB::BUMP_TO_HORI_MUX_SIZE;
        }
    }

    std::size_t s_applied = 0;
    for (const auto& s : result.active_s) {
        const auto tob = interposer->get_tob(tob_from_linear(s.tob));
        if (!tob.has_value()) {
            debug::warning_fmt("apply TOB SAT: TOB {} not found for S(v={})", s.tob, s.v);
            continue;
        }
        const auto key = std::pair<std::size_t, std::size_t> {s.tob, s.v};
        const auto it = hori_mux_by_tob_v.find(key);
        if (it == hori_mux_by_tob_v.end()) {
            debug::info_fmt(
                "apply TOB SAT: active S on TOB {} v={} has no matching W (unexpected after W-derived active_s)",
                s.tob,
                s.v);
            continue;
        }
        const auto hori_index = it->second * hardware::TOB::HORI_TO_VERI_MUX_SIZE + s.j;
        const auto hori_info = hardware::TOB::hori_to_vert_mux_info(hori_index);
        try {
            auto mux_conn = (*tob)->hori_to_vert_muxs(std::get<0>(hori_info))->connector(std::get<1>(hori_info), s.k, false);
            mux_conn.connect();
            s_applied += 1;
        }
        catch (const std::exception& e) {
            debug::warning_fmt("apply TOB SAT: S on TOB {} v={} failed: {}", s.tob, s.v, e.what());
        }
    }

    std::size_t w_applied = 0;
    for (const auto& w : result.active_w) {
        if (!w.has_track) {
            continue;
        }
        auto* bump = find_bump(interposer, w.bump);
        if (bump == nullptr) {
            debug::warning_fmt(
                "apply TOB SAT: bump(T{},B{},G{},I{}) not found",
                w.bump.TOB,
                w.bump.Bank,
                w.bump.Group,
                w.bump.Index);
            continue;
        }
        auto* tob = bump->tob();
        if (tob == nullptr) {
            debug::warning_fmt("apply TOB SAT: bump(T{},B{},G{},I{}) has no TOB", w.bump.TOB, w.bump.Bank, w.bump.Group, w.bump.Index);
            continue;
        }
        try {
            const auto& bump_coord = bump->coord();
            const auto track_coord = hardware::TrackCoord {
                bump_coord.row,
                bump_coord.col,
                hardware::TrackDirection::Vertical,
                w.track};
            const auto track = interposer->get_track(track_coord);
            if (!track.has_value()) {
                debug::warning_fmt(
                    "apply TOB SAT: track {} not found for bump(T{},B{},G{},I{})",
                    w.track,
                    w.bump.TOB,
                    w.bump.Bank,
                    w.bump.Group,
                    w.bump.Index);
                continue;
            }
            bump->set_allocated_track(track.value());
            bump->intersect_access_unit(std::HashSet<std::usize> {map_track(w.track)});
            w_applied += 1;
        }
        catch (const std::exception& e) {
            debug::warning_fmt(
                "apply TOB SAT: W bump(T{},B{},G{},I{}) track={} failed: {}",
                w.bump.TOB,
                w.bump.Bank,
                w.bump.Group,
                w.bump.Index,
                w.track,
                e.what());
        }
    }

    debug::info_fmt("apply TOB SAT to interposer: S configured={}, W bumps configured={}", s_applied, w_applied);
}

} // namespace PR_tool
