#pragma once

#include "hardware/cob/cobcoord.hh"
#include "hardware/interposer.hh"
#include "hardware/tob/tobcoord.hh"
#include "hardware/track/track.hh"
#include "hardware/track/trackcoord.hh"

#include <cstddef>
#include <std/collection.hh>
#include <std/utility.hh>

namespace PR_tool::mcf {

// COB grid 9x12, row-major linear index
inline auto cob_to_linear(hardware::COBCoord c) -> std::size_t {
    return static_cast<std::size_t>(c.row) * hardware::Interposer::COB_ARRAY_WIDTH + static_cast<std::size_t>(c.col);
}

inline auto linear_to_cob(std::size_t i) -> hardware::COBCoord {
    return hardware::COBCoord{static_cast<std::i64>(i / hardware::Interposer::COB_ARRAY_WIDTH),
                              static_cast<std::i64>(i % hardware::Interposer::COB_ARRAY_WIDTH)};
}

// TOB (tr, tc) in 4x4 array lies between two vertically adjacent COBs: upper (1+2*tr, 3*tc) and lower (2*tr, 3*tc)
inline auto tob_pair_cob_coords(std::size_t tob_tr, std::size_t tob_tc)
    -> std::pair<hardware::COBCoord, hardware::COBCoord> {
    const auto c0 = hardware::COBCoord{static_cast<std::i64>(1 + 2 * static_cast<std::i64>(tob_tr)), static_cast<std::i64>(3 * static_cast<std::i64>(tob_tc))};
    const auto c1 = hardware::COBCoord{static_cast<std::i64>(2 * static_cast<std::i64>(tob_tr)), static_cast<std::i64>(3 * static_cast<std::i64>(tob_tc))};
    return {c0, c1};
}

inline auto tob_index_from_linear(std::size_t tob_linear) -> std::pair<std::size_t, std::size_t> {
    const std::size_t w = hardware::Interposer::TOB_ARRAY_WIDTH;
    return {tob_linear / w, tob_linear % w};
}

// Resolve track to a single COB node per plan: take adjacent COBs, keep those inside [0..H) x [0..W);
// if two, prefer (row, col) lexicographically smaller.
inline auto track_to_cob(const hardware::TrackCoord& tc) -> hardware::COBCoord {
    hardware::Track t{tc};
    auto adj = t.adjacent_cob_coords();
    auto best = std::Option<hardware::COBCoord> {};
    for (const auto& [dir, cob] : adj) {
        (void)dir;
        if (cob.row < 0 || cob.col < 0) {
            continue;
        }
        if (cob.row >= static_cast<std::i64>(hardware::Interposer::COB_ARRAY_HEIGHT)) {
            continue;
        }
        if (cob.col >= static_cast<std::i64>(hardware::Interposer::COB_ARRAY_WIDTH)) {
            continue;
        }
        if (!best.has_value()) {
            best.emplace(cob);
        }
        else {
            const auto& a = *best;
            if (cob.row < a.row || (cob.row == a.row && cob.col < a.col)) {
                best.emplace(cob);
            }
        }
    }
    if (!best.has_value()) {
        return hardware::COBCoord{0, 0};
    }
    return *best;
}

} // namespace PR_tool::mcf
