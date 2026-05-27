#include "precompute/ilp_reach_precompute.hh"

#include "ilp_allocation/ilp_speedup.hh"
#include "mcf/mcf_hw_map.hh"

#include <hardware/cob/cobunit.hh>
#include <hardware/interposer.hh>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <format>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace PR_tool {

namespace {

enum class TurnDir {
    Left,
    Right,
    Up,
    Down
};

struct RelationContext {
    bool is_vertical{false};
    bool is_horizontal{false};
    bool is_diagonal{false};
    bool end_right{false};
    bool end_down{false};
    std::size_t delta_rows{0};
    std::size_t delta_cols{0};
    bool end_is_horizontal_track{false};
};

auto all_track_indices() -> const std::Vector<std::size_t>& {
    static const auto kAllTracks = [] {
        auto tracks = std::Vector<std::size_t> {};
        tracks.reserve(128);
        for (std::size_t r = 0; r < 128; ++r) {
            tracks.push_back(r);
        }
        return tracks;
    }();
    return kAllTracks;
}

auto track_bank(std::size_t track) -> std::size_t {
    return track < 64 ? 0 : 1;
}

auto track_unit(std::size_t track) -> std::size_t {
    return track % 8;
}

auto track_inner_index(std::size_t track) -> std::size_t {
    const auto v = track % 64;
    return v / 8;
}

auto make_track(std::size_t bank, std::size_t unit, std::size_t inner_idx) -> std::size_t {
    return bank * 64 + inner_idx * 8 + unit;
}

auto to_char(TurnDir dir) -> char {
    switch (dir) {
        case TurnDir::Left:
            return 'L';
        case TurnDir::Right:
            return 'R';
        case TurnDir::Up:
            return 'U';
        case TurnDir::Down:
            return 'D';
    }
    return 'L';
}

auto map_index(TurnDir from, TurnDir to, std::size_t idx) -> std::size_t {
    using D = hardware::COBDirection;
    const auto from_dir = [&]() -> D {
        switch (from) {
            case TurnDir::Left:
                return D::Left;
            case TurnDir::Right:
                return D::Right;
            case TurnDir::Up:
                return D::Up;
            case TurnDir::Down:
                return D::Down;
        }
        return D::Left;
    }();
    const auto to_dir = [&]() -> D {
        switch (to) {
            case TurnDir::Left:
                return D::Left;
            case TurnDir::Right:
                return D::Right;
            case TurnDir::Up:
                return D::Up;
            case TurnDir::Down:
                return D::Down;
        }
        return D::Up;
    }();
    return static_cast<std::size_t>(hardware::COBUnit::index_map(from_dir, static_cast<std::usize>(idx), to_dir));
}

auto tob_from_linear(std::size_t tob_linear) -> hardware::TOBCoord {
    const auto width = static_cast<std::size_t>(hardware::Interposer::TOB_ARRAY_WIDTH);
    return hardware::TOBCoord {
        static_cast<std::i64>(tob_linear / width),
        static_cast<std::i64>(tob_linear % width)};
}

auto tob_anchor_cob(std::size_t tob_linear) -> hardware::COBCoord {
    const auto tob = tob_from_linear(tob_linear);
    return hardware::COBCoord {
        static_cast<std::i64>(1 + 2 * tob.row),
        static_cast<std::i64>(3 * tob.col)};
}

auto relation_bnet(const Bump_coord& start_bump, const Bump_coord& end_bump) -> RelationContext {
    const auto s = tob_from_linear(start_bump.TOB);
    const auto e = tob_from_linear(end_bump.TOB);
    const auto dr = static_cast<std::size_t>(std::llabs(e.row - s.row));
    const auto dc = static_cast<std::size_t>(std::llabs(e.col - s.col));

    RelationContext ctx {};
    ctx.delta_rows = dr;
    ctx.delta_cols = dc;
    ctx.end_right = e.col > s.col;
    // In document conventions, smaller row means "down".
    ctx.end_down = e.row < s.row;
    if (dc == 0) {
        ctx.is_vertical = true;
    }
    else if (dr == 0) {
        ctx.is_horizontal = true;
    }
    else {
        ctx.is_diagonal = true;
    }
    return ctx;
}

auto relation_tnet(std::size_t start_tob_linear, const hardware::TrackCoord& end_track) -> RelationContext {
    const auto s = tob_anchor_cob(start_tob_linear);
    const auto e = mcf::track_to_cob(end_track);
    const auto dr = static_cast<std::size_t>(std::llabs(e.row - s.row));
    const auto dc = static_cast<std::size_t>(std::llabs(e.col - s.col));

    RelationContext ctx {};
    ctx.delta_rows = dr;
    ctx.delta_cols = dc;
    ctx.end_right = e.col > s.col;
    ctx.end_down = e.row < s.row;
    ctx.end_is_horizontal_track = (end_track.dir == hardware::TrackDirection::Horizontal);
    if (dc == 0) {
        ctx.is_vertical = true;
    }
    else {
        ctx.is_diagonal = true;
    }
    return ctx;
}

auto sort_unique(std::Vector<std::size_t>& values) -> void {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

auto diagonal_pair_for(const RelationContext& ctx)
    -> std::Pair<std::Pair<TurnDir, TurnDir>, std::Pair<TurnDir, TurnDir>> {
    if (ctx.end_right && ctx.end_down) {
        return {{TurnDir::Down, TurnDir::Left}, {TurnDir::Right, TurnDir::Up}}; // DL -> RU
    }
    if (ctx.end_right && !ctx.end_down) {
        return {{TurnDir::Up, TurnDir::Left}, {TurnDir::Right, TurnDir::Down}}; // UL -> RD
    }
    if (!ctx.end_right && ctx.end_down) {
        return {{TurnDir::Down, TurnDir::Right}, {TurnDir::Left, TurnDir::Up}}; // DR -> LU
    }
    return {{TurnDir::Up, TurnDir::Right}, {TurnDir::Left, TurnDir::Down}}; // UR -> LD
}

auto maybe_horizontal_front_step(const RelationContext& ctx) -> std::Option<std::Pair<TurnDir, TurnDir>> {
    if (!ctx.end_is_horizontal_track) {
        return std::nullopt;
    }
    const auto from = ctx.end_right ? TurnDir::Right : TurnDir::Left;
    const auto to = ctx.end_down ? TurnDir::Up : TurnDir::Down;
    return std::Pair<TurnDir, TurnDir> {from, to};
}

auto apply_sequence(
    std::size_t end_track,
    const std::Vector<std::Pair<TurnDir, TurnDir>>& seq,
    std::Vector<IlpReachStep>& out_steps
) -> std::size_t {
    const auto bank = track_bank(end_track);
    const auto unit = track_unit(end_track);
    auto idx = track_inner_index(end_track);
    out_steps.clear();
    out_steps.reserve(seq.size());
    for (const auto& [from, to] : seq) {
        const auto mapped = map_index(from, to, idx);
        out_steps.push_back(IlpReachStep {to_char(from), to_char(to), idx, mapped});
        idx = mapped;
    }
    return make_track(bank, unit, idx);
}

auto ensure_non_empty_startset(
    std::size_t end_track,
    std::Vector<std::size_t>& starts,
    std::map<std::size_t, std::Vector<IlpReachStep>>& reaches
) -> void {
    if (!starts.empty()) {
        return;
    }
    starts.push_back(end_track);
    reaches[end_track] = {};
}

auto fill_vertical_case(
    std::size_t end_track,
    std::Vector<std::size_t>& starts,
    std::map<std::size_t, std::Vector<IlpReachStep>>& reaches
) -> void {
    starts.push_back(end_track);
    reaches[end_track] = {};
}

auto fill_horizontal_case(
    const RelationContext& ctx,
    std::size_t end_track,
    std::Vector<std::size_t>& starts,
    std::map<std::size_t, std::Vector<IlpReachStep>>& reaches
) -> void {
    const auto seqs = [&]() {
        if (ctx.end_right) {
            return std::array<std::array<std::Pair<TurnDir, TurnDir>, 2>, 2> {
                std::array<std::Pair<TurnDir, TurnDir>, 2> {
                    std::Pair<TurnDir, TurnDir> {TurnDir::Down, TurnDir::Left},
                    std::Pair<TurnDir, TurnDir> {TurnDir::Right, TurnDir::Down}},
                std::array<std::Pair<TurnDir, TurnDir>, 2> {
                    std::Pair<TurnDir, TurnDir> {TurnDir::Up, TurnDir::Left},
                    std::Pair<TurnDir, TurnDir> {TurnDir::Right, TurnDir::Up}}};
        }
        return std::array<std::array<std::Pair<TurnDir, TurnDir>, 2>, 2> {
            std::array<std::Pair<TurnDir, TurnDir>, 2> {
                std::Pair<TurnDir, TurnDir> {TurnDir::Down, TurnDir::Right},
                std::Pair<TurnDir, TurnDir> {TurnDir::Left, TurnDir::Down}},
            std::array<std::Pair<TurnDir, TurnDir>, 2> {
                std::Pair<TurnDir, TurnDir> {TurnDir::Up, TurnDir::Right},
                std::Pair<TurnDir, TurnDir> {TurnDir::Left, TurnDir::Up}}};
    }();
    for (const auto& seq_fixed : seqs) {
        auto seq = std::Vector<std::Pair<TurnDir, TurnDir>> {};
        seq.push_back(seq_fixed[0]);
        seq.push_back(seq_fixed[1]);
        auto steps = std::Vector<IlpReachStep> {};
        const auto start_track = apply_sequence(end_track, seq, steps);
        starts.push_back(start_track);
        reaches[start_track] = std::move(steps);
    }
}

auto fill_diagonal_case(
    const Net_cost_record& record,
    const RelationContext& ctx,
    std::size_t end_track,
    std::Vector<std::size_t>& starts,
    std::map<std::size_t, std::Vector<IlpReachStep>>& reaches
) -> void {
    const auto pair = diagonal_pair_for(ctx);
    auto delta = std::min(ctx.delta_rows, ctx.delta_cols);
    if (delta == 0) {   // 如果是Tnet，只需要一次水平到垂直的转弯；如果是Bnet，不可能出现这样的情况
        if (const auto first = maybe_horizontal_front_step(ctx); first.has_value()) {
            auto seq = std::Vector<std::Pair<TurnDir, TurnDir>> {};
            seq.push_back(*first);
            auto steps = std::Vector<IlpReachStep> {};
            const auto start_track = apply_sequence(end_track, seq, steps);
            starts.push_back(start_track);
            if (!reaches.contains(start_track)) {
                reaches[start_track] = std::move(steps);
            }
        }
    }
    for (std::size_t repeat = 1; repeat <= delta; ++repeat) {
        auto seq = std::Vector<std::Pair<TurnDir, TurnDir>> {};
        if (const auto first = maybe_horizontal_front_step(ctx); first.has_value()) {   
            seq.push_back(*first);
        }
        for (std::size_t i = 0; i < repeat; ++i) {
            seq.push_back(pair.first);
            seq.push_back(pair.second);
        }
        auto steps = std::Vector<IlpReachStep> {};
        const auto start_track = apply_sequence(end_track, seq, steps);
        starts.push_back(start_track);
        if (!reaches.contains(start_track)) {
            reaches[start_track] = std::move(steps);
        }
    }
}

auto fill_endtrack_reach(
    const Net_cost_record& record,
    const RelationContext& ctx,
    std::size_t end_track,
    std::Vector<std::size_t>& starts,
    std::map<std::size_t, std::Vector<IlpReachStep>>& reaches
) -> void {
    starts.clear();
    reaches.clear();
    if (ctx.is_vertical) {
        fill_vertical_case(end_track, starts, reaches);
    }
    else if (ctx.is_horizontal) {
        fill_horizontal_case(ctx, end_track, starts, reaches);
    }
    else if (ctx.is_diagonal) {
        fill_diagonal_case(record, ctx, end_track, starts, reaches);
    }
    if (starts.empty()) {
        throw std::logic_error(std::format(
            "precompute: endtrack reach with empty starts (2pin_record=\"{}\", origin_key=\"{}\", record_id={}, end_track={}, "
            "is_vertical={}, is_horizontal={}, is_diagonal={}, delta_rows={}, delta_cols={})",
            record.net_name,
            record.origin_key,
            record.record_id,
            end_track,
            ctx.is_vertical,
            ctx.is_horizontal,
            ctx.is_diagonal,
            ctx.delta_rows,
            ctx.delta_cols));
    }
    sort_unique(starts);
}

auto deduce_tnet_track(const Net_cost_record& record) -> std::Option<hardware::TrackCoord> {
    if (record.mcf_has_end_track) {
        return record.mcf_end_track;
    }
    if (record.mcf_has_start_track) {
        return record.mcf_start_track;
    }
    return std::nullopt;
}

} // namespace

auto precompute_reach_for_records(std::Vector<Net_cost_record>& records) -> IlpReachPrecomputeStats {
    auto stats = IlpReachPrecomputeStats {};
    stats.total_records = records.size();

    for (auto& record : records) {
        record.end_tracks.clear();
        record.starttrack_by_endtrack.clear();
        record.reach_by_end_start.clear();

        if (record.type == Net_type::Bnet) {
            stats.bnet_records += 1;
            record.end_tracks = all_track_indices();
            if (record.start_bumps.empty() || record.end_bumps.empty()) {
                throw std::runtime_error(std::format("precompute: Bnet '{}' missing bump endpoint", record.net_name));
            }
            const auto ctx = relation_bnet(record.start_bumps.front(), record.end_bumps.front());
            for (const auto end_track : record.end_tracks) {
                auto starts = std::Vector<std::size_t> {};
                auto reaches = std::map<std::size_t, std::Vector<IlpReachStep>> {};
                fill_endtrack_reach(record, ctx, end_track, starts, reaches);
                record.starttrack_by_endtrack[end_track] = std::move(starts);
                record.reach_by_end_start[end_track] = std::move(reaches);
            }
        }
        else if (record.type == Net_type::Tnet) {
            stats.tnet_records += 1;
            if (record.start_bumps.empty()) {
                throw std::runtime_error(std::format("precompute: Tnet '{}' missing start bump", record.net_name));
            }
            const auto track_opt = deduce_tnet_track(record);
            if (!track_opt.has_value()) {
                throw std::runtime_error(std::format("precompute: Tnet '{}' missing track endpoint", record.net_name));
            }
            const auto track = *track_opt;
            record.end_tracks.push_back(track.index);
            sort_unique(record.end_tracks);
            const auto ctx = relation_tnet(record.start_bumps.front().TOB, track);
            for (const auto end_track : record.end_tracks) {
                auto starts = std::Vector<std::size_t> {};
                auto reaches = std::map<std::size_t, std::Vector<IlpReachStep>> {};
                fill_endtrack_reach(record, ctx, end_track, starts, reaches);
                record.starttrack_by_endtrack[end_track] = std::move(starts);
                record.reach_by_end_start[end_track] = std::move(reaches);
            }
        }
        else {
            stats.pnnet_records += 1;
            if (record.start_bumps.empty()) {
                throw std::runtime_error(std::format("precompute: PNnet '{}' missing start bump", record.net_name));
            }
            record.end_tracks = record.pn_end_tracks;
            if (record.end_tracks.empty()) {
                throw std::runtime_error(std::format("precompute: PNnet '{}' missing end tracks", record.net_name));
            }
            sort_unique(record.end_tracks);
            for (const auto end_track : record.end_tracks) {
                auto starts = std::Vector<std::size_t> {};
                auto reaches = std::map<std::size_t, std::Vector<IlpReachStep>> {};
                if (const auto it = record.pn_end_track_coord_by_index.find(end_track); it != record.pn_end_track_coord_by_index.end()) {
                    const auto ctx = relation_tnet(record.start_bumps.front().TOB, it->second);
                    fill_endtrack_reach(record, ctx, end_track, starts, reaches);
                }
                else {
                    throw std::runtime_error(std::format("precompute: PNnet '{}' missing end track coord by index", record.net_name));
                }
                record.starttrack_by_endtrack[end_track] = std::move(starts);
                record.reach_by_end_start[end_track] = std::move(reaches);
            }
        }

        stats.total_endtracks += record.end_tracks.size();
        for (const auto& [end_track, starts] : record.starttrack_by_endtrack) {
            (void)end_track;
            stats.total_starttrack_edges += starts.size();
        }
    }

    return stats;
}

} // namespace PR_tool
