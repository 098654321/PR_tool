#include "precompute/tob_path_precompute.hh"

#include "mcf/mcf_hw_map.hh"

#include <hardware/cob/cob.hh>
#include <hardware/cob/cobconnector.hh>
#include <hardware/interposer.hh>
#include <hardware/track/track.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <debug/debug.hh>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <queue>
#include <set>
#include <stdexcept>
#include <thread>
#include <tuple>

namespace PR_tool {

namespace {

using WiltonTag = std::pair<std::i64, std::i64>;

constexpr WiltonTag kNoWiltonTag {-1, -1};

auto encode_wilton_tag(const std::optional<hardware::COBCoord>& cob) -> WiltonTag {
    if (!cob.has_value()) {
        return kNoWiltonTag;
    }
    return {cob->row, cob->col};
}

auto decode_wilton_tag(const WiltonTag tag) -> std::optional<hardware::COBCoord> {
    if (tag == kNoWiltonTag) {
        return std::nullopt;
    }
    return hardware::COBCoord {tag.first, tag.second};
}

struct BfsKey {
    hardware::TrackCoord coord;
    WiltonTag wilton{kNoWiltonTag};

    auto operator<(const BfsKey& other) const -> bool {
        if (coord < other.coord) {
            return true;
        }
        if (other.coord < coord) {
            return false;
        }
        if (wilton < other.wilton) {
            return true;
        }
        if (other.wilton < wilton) {
            return false;
        }
        return false;
    }
};

struct BfsNode {
    BfsKey key;
};

struct BfsPrev {
    BfsKey key;
};

struct PrecomputeTrackArc {
    hardware::TrackCoord next;
    hardware::COBCoord wilton_cob {};
};

struct PrecomputeTrackGraph {
    std::map<hardware::TrackCoord, hardware::Track*> tracks;
    std::map<hardware::TrackCoord, std::Vector<PrecomputeTrackArc>> adjacency;
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

auto populate_end_tracks(Net_cost_record& record) -> void {
    record.end_tracks.clear();
    if (record.type == Net_type::Bnet) {
        record.end_tracks = all_track_indices();
        return;
    }
    if (record.type == Net_type::Tnet) {
        if (record.mcf_has_end_track) {
            record.end_tracks.push_back(record.mcf_end_track.index);
        }
        else if (record.mcf_has_start_track) {
            record.end_tracks.push_back(record.mcf_start_track.index);
        }
        std::sort(record.end_tracks.begin(), record.end_tracks.end());
        record.end_tracks.erase(std::unique(record.end_tracks.begin(), record.end_tracks.end()), record.end_tracks.end());
        return;
    }
    record.end_tracks = record.pn_end_tracks;
    std::sort(record.end_tracks.begin(), record.end_tracks.end());
    record.end_tracks.erase(std::unique(record.end_tracks.begin(), record.end_tracks.end()), record.end_tracks.end());
}

auto end_track_coord_for_record(const Net_cost_record& record, const std::size_t end_track)
    -> std::optional<hardware::TrackCoord> {
    if (record.type == Net_type::Bnet) {
        if (record.end_bumps.empty()) {
            return std::nullopt;
        }
        const auto channel = tob_channel_track_coords(record.end_bumps.front().TOB);
        if (end_track >= channel.size()) {
            return std::nullopt;
        }
        return channel[end_track];
    }
    if (record.type == Net_type::Tnet) {
        if (record.mcf_has_end_track && record.mcf_end_track.index == end_track) {
            return record.mcf_end_track;
        }
        if (record.mcf_has_start_track && record.mcf_start_track.index == end_track) {
            return record.mcf_start_track;
        }
        return std::nullopt;
    }
    const auto it = record.pn_end_track_coord_by_index.find(end_track);
    if (it == record.pn_end_track_coord_by_index.end()) {
        return std::nullopt;
    }
    return it->second;
}

auto start_track_coord(const std::size_t start_tob, const std::size_t start_track) -> std::optional<hardware::TrackCoord> {
    const auto channel = tob_channel_track_coords(start_tob);
    if (start_track >= channel.size()) {
        return std::nullopt;
    }
    return channel[start_track];
}

auto candidate_start_tracks_for_end(const std::size_t start_tob, const std::size_t end_track) -> std::Vector<std::size_t> {
    const auto cob_unit = map_track(end_track);
    auto out = std::Vector<std::size_t> {};
    for (const auto& tc : tob_channel_track_coords(start_tob)) {
        if (map_track(tc.index) == cob_unit) {
            out.push_back(tc.index);
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

auto clamp_path_bbox(IlpBoundingBox box) -> IlpBoundingBox {
    const auto max_row = static_cast<std::i64>(hardware::Interposer::COB_ARRAY_HEIGHT) - 1;
    const auto max_col = static_cast<std::i64>(hardware::Interposer::COB_ARRAY_WIDTH) - 1;
    box.row_min = std::max<std::i64>(0, std::min(box.row_min, max_row));
    box.row_max = std::max<std::i64>(0, std::min(box.row_max, max_row));
    box.col_min = std::max<std::i64>(0, std::min(box.col_min, max_col));
    box.col_max = std::max<std::i64>(0, std::min(box.col_max, max_col));
    if (box.row_min > box.row_max) {
        std::swap(box.row_min, box.row_max);
    }
    if (box.col_min > box.col_max) {
        std::swap(box.col_min, box.col_max);
    }
    return box;
}

auto path_bbox_from_tracks(
    const Net_cost_record& record,
    const std::size_t end_track,
    const std::Vector<hardware::Track*>& path
) -> IlpBoundingBox {
    if (record.start_bumps.empty()) {
        return IlpBoundingBox {0, 0, 0, 0};
    }
    auto box = IlpBoundingBox {
        std::numeric_limits<std::i64>::max(),
        std::numeric_limits<std::i64>::min(),
        std::numeric_limits<std::i64>::max(),
        std::numeric_limits<std::i64>::min()};
    merge_coord_into_bbox(box, tob_anchor_cob(record.start_bumps.front().TOB));
    if (record.type == Net_type::Bnet && !record.end_bumps.empty()) {
        merge_coord_into_bbox(box, tob_anchor_cob(record.end_bumps.front().TOB));
        if (const auto end_tc = end_track_coord_for_record(record, end_track); end_tc.has_value()) {
            merge_coord_into_bbox(box, mcf::track_to_cob(*end_tc));
        }
    }
    else if (const auto end_tc = end_track_coord_for_record(record, end_track); end_tc.has_value()) {
        merge_coord_into_bbox(box, mcf::track_to_cob(*end_tc));
    }
    for (const auto* track : path) {
        merge_coord_into_bbox(box, mcf::track_to_cob(track->coord()));
    }
    return clamp_path_bbox(box);
}

auto make_bfs_key(
    const hardware::TrackCoord& coord,
    const std::optional<hardware::COBCoord>& last_wilton
) -> BfsKey {
    return BfsKey {coord, encode_wilton_tag(last_wilton)};
}

auto path_has_repeated_tracks(const std::Vector<hardware::Track*>& path) -> bool {
    auto seen = std::set<hardware::TrackCoord> {};
    for (const auto* track : path) {
        if (track == nullptr) {
            return true;
        }
        if (!seen.insert(track->coord()).second) {
            return true;
        }
    }
    return false;
}

auto constrained_shortest_path(
    const PrecomputeTrackGraph& graph,
    const hardware::TrackCoord& end_coord,
    const hardware::TrackCoord& target_coord
) -> std::Vector<hardware::Track*> {
    const auto end_track_it = graph.tracks.find(end_coord);
    const auto target_track_it = graph.tracks.find(target_coord);
    if (end_track_it == graph.tracks.end() || target_track_it == graph.tracks.end()) {
        return {};
    }
    auto* const end_track = end_track_it->second;
    auto* const target_track = target_track_it->second;
    if (end_track == target_track) {
        return {end_track};
    }

    auto prev = std::map<BfsKey, BfsPrev> {};
    auto q = std::queue<BfsNode> {};
    const auto start_key = make_bfs_key(end_track->coord(), std::nullopt);
    q.push(BfsNode {start_key});
    prev.emplace(start_key, BfsPrev {start_key});

    while (!q.empty()) {
        const auto node = q.front();
        q.pop();
        const auto track_it = graph.tracks.find(node.key.coord);
        if (track_it == graph.tracks.end()) {
            continue;
        }
        if (track_it->second == target_track) {
            auto path = std::Vector<hardware::Track*> {};
            auto key = node.key;
            while (true) {
                const auto path_track_it = graph.tracks.find(key.coord);
                if (path_track_it == graph.tracks.end()) {
                    break;
                }
                path.push_back(path_track_it->second);
                const auto it = prev.find(key);
                if (it == prev.end()) {
                    break;
                }
                const auto parent = it->second.key;
                if (parent.coord == key.coord && parent.wilton == key.wilton) {
                    break;
                }
                key = parent;
            }
            std::reverse(path.begin(), path.end());
            if (path_has_repeated_tracks(path)) {
                continue;
            }
            return path;
        }

        const auto next_it = graph.adjacency.find(node.key.coord);
        if (next_it == graph.adjacency.end()) {
            continue;
        }
        const auto last_wilton_cob = decode_wilton_tag(node.key.wilton);
        for (const auto& arc : next_it->second) {
            if (last_wilton_cob.has_value() && *last_wilton_cob == arc.wilton_cob) {
                continue;
            }
            const auto next_key = make_bfs_key(arc.next, arc.wilton_cob);
            if (prev.contains(next_key)) {
                continue;
            }
            prev.emplace(next_key, BfsPrev {node.key});
            q.push(BfsNode {next_key});
        }
    }
    return {};
}

auto build_precompute_track_graph(hardware::Interposer* interposer) -> PrecomputeTrackGraph {
    auto graph = PrecomputeTrackGraph {};
    for (std::i64 row = 0; row <= hardware::Interposer::COB_ARRAY_HEIGHT; ++row) {
        for (std::i64 col = 0; col <= hardware::Interposer::COB_ARRAY_WIDTH; ++col) {
            for (const auto dir : {hardware::TrackDirection::Horizontal, hardware::TrackDirection::Vertical}) {
                for (std::size_t index = 0; index < hardware::COB::INDEX_SIZE; ++index) {
                    const auto coord = hardware::TrackCoord {row, col, dir, index};
                    const auto track_opt = interposer->get_track(coord);
                    if (track_opt.has_value()) {
                        graph.tracks.emplace(coord, *track_opt);
                    }
                }
            }
        }
    }
    for (const auto& [coord, track] : graph.tracks) {
        auto arcs = std::Vector<PrecomputeTrackArc> {};
        for (const auto& [next_track, connector] : interposer->adjacent_tracks(track)) {
            arcs.push_back(PrecomputeTrackArc {next_track->coord(), connector.coord()});
        }
        graph.adjacency.emplace(coord, std::move(arcs));
    }
    return graph;
}

auto build_length_layers(TobEndTrackPrecompute& end_data) -> void {
    auto length_to_tracks = std::map<std::size_t, std::Vector<std::size_t>> {};
    for (const auto start_track : end_data.candidate_start_tracks) {
        const auto it = end_data.by_start_track.find(start_track);
        if (it == end_data.by_start_track.end()) {
            continue;
        }
        length_to_tracks[it->second.path_length].push_back(start_track);
    }
    end_data.length_layers.clear();
    for (auto& [length, tracks] : length_to_tracks) {
        (void)length;
        std::sort(tracks.begin(), tracks.end());
        tracks.erase(std::unique(tracks.begin(), tracks.end()), tracks.end());
        end_data.length_layers.push_back(std::move(tracks));
    }
}

auto precompute_end_track(
    const Net_cost_record& record,
    const PrecomputeTrackGraph& graph,
    const std::size_t end_track
) -> TobEndTrackPrecompute {
    auto out = TobEndTrackPrecompute {};
    if (record.start_bumps.empty()) {
        return out;
    }
    const auto start_tob = record.start_bumps.front().TOB;
    const auto end_coord_opt = end_track_coord_for_record(record, end_track);
    if (!end_coord_opt.has_value()) {
        return out;
    }

    out.candidate_start_tracks = candidate_start_tracks_for_end(start_tob, end_track);
    for (const auto start_track : out.candidate_start_tracks) {
        const auto target_opt = start_track_coord(start_tob, start_track);
        if (!target_opt.has_value()) {
            continue;
        }
        const auto path = constrained_shortest_path(graph, *end_coord_opt, *target_opt);
        if (path.empty()) {
            continue;
        }
        auto entry = TobPathEntry {};
        entry.path_length = path.size();
        entry.path_bbox = path_bbox_from_tracks(record, end_track, path);
        entry.area = bbox_area(entry.path_bbox);
        out.by_start_track.emplace(start_track, std::move(entry));
    }

    auto reachable = std::Vector<std::size_t> {};
    reachable.reserve(out.by_start_track.size());
    for (const auto start_track : out.candidate_start_tracks) {
        if (out.by_start_track.contains(start_track)) {
            reachable.push_back(start_track);
        }
    }
    out.candidate_start_tracks = std::move(reachable);
    build_length_layers(out);
    return out;
}

struct PathPrecomputeWorkItem {
    std::size_t record_index{0};
    std::size_t end_track{0};
};

constexpr std::size_t kPathPrecomputeBarWidth = 20;

auto format_progress_bar(const std::size_t percent) -> std::String {
    const auto filled = std::min(kPathPrecomputeBarWidth, percent * kPathPrecomputeBarWidth / 100);
    auto bar = std::String {};
    bar.reserve(kPathPrecomputeBarWidth);
    for (std::size_t i = 0; i < kPathPrecomputeBarWidth; ++i) {
        bar += i < filled ? '#' : '-';
    }
    return bar;
}

auto log_path_precompute_progress(const std::size_t done, const std::size_t total, const std::size_t percent)
    -> void {
    debug::info_fmt(
        "path precompute progress: [{}] {}% ({}/{})",
        format_progress_bar(percent),
        percent,
        done,
        total);
}

struct PathPrecomputeProgress {
    std::size_t total{0};
    std::atomic<std::size_t> completed{0};
    std::size_t last_logged_percent{0};
    std::mutex log_mutex {};

    explicit PathPrecomputeProgress(const std::size_t total_work_items) : total(total_work_items) {
        if (total == 0) {
            return;
        }
        log_path_precompute_progress(0, total, 0);
    }

    auto mark_one_done() -> void {
        if (total == 0) {
            return;
        }
        const auto done = ++completed;
        const auto percent = std::min<std::size_t>(100, done * 100 / total);
        std::lock_guard lock(log_mutex);
        if (percent > last_logged_percent || done == total) {
            last_logged_percent = done == total ? 100 : percent;
            log_path_precompute_progress(done, total, last_logged_percent);
        }
    }
};

} // namespace

auto tob_channel_track_coords(const std::size_t tob_linear) -> std::Vector<hardware::TrackCoord> {
    const auto [tr, tc] = mcf::tob_index_from_linear(tob_linear);
    const auto upper = hardware::COBCoord {
        static_cast<std::i64>(1 + 2 * static_cast<std::i64>(tr)),
        static_cast<std::i64>(3 * static_cast<std::i64>(tc))};
    auto out = std::Vector<hardware::TrackCoord> {};
    out.reserve(128);
    for (std::size_t idx = 0; idx < 128; ++idx) {
        out.push_back(hardware::TrackCoord {
            upper.row,
            upper.col,
            hardware::TrackDirection::Vertical,
            idx});
    }
    return out;
}

auto precompute_all_path_caches(
    std::Vector<Net_cost_record>& records,
    hardware::Interposer* interposer,
    const bool enable_parallel
) -> TobPathPrecomputeCache {
    const auto t0 = std::chrono::steady_clock::now();
    auto cache = TobPathPrecomputeCache {};
    cache.by_record.resize(records.size());
    if (interposer == nullptr) {
        return cache;
    }

    auto work_items = std::Vector<PathPrecomputeWorkItem> {};
    for (std::size_t record_index = 0; record_index < records.size(); ++record_index) {
        auto& record = records[record_index];
        populate_end_tracks(record);
        record.starttrack_by_endtrack.clear();
        for (const auto end_track : record.end_tracks) {
            work_items.push_back(PathPrecomputeWorkItem {record_index, end_track});
        }
    }

    auto end_results = std::Vector<TobEndTrackPrecompute>(work_items.size());
    if (work_items.empty()) {
        return cache;
    }

    const auto graph = build_precompute_track_graph(interposer);
    auto progress = PathPrecomputeProgress {work_items.size()};

    const auto run_chunk = [&](const std::size_t begin, const std::size_t end) -> void {
        for (std::size_t work_index = begin; work_index < end; ++work_index) {
            const auto& item = work_items[work_index];
            end_results[work_index] = precompute_end_track(
                records[item.record_index],
                graph,
                item.end_track);
            progress.mark_one_done();
        }
    };

    if (!enable_parallel) {
        run_chunk(0, work_items.size());
    }
    else {
        const auto hw_threads = std::thread::hardware_concurrency();
        const auto num_threads = std::max<std::size_t>(
            1,
            hw_threads > 0 ? static_cast<std::size_t>(hw_threads) : 1);
        const auto chunk_size = (work_items.size() + num_threads - 1) / num_threads;
        auto futures = std::Vector<std::future<void>> {};
        futures.reserve(num_threads);
        for (std::size_t thread_index = 0; thread_index < num_threads; ++thread_index) {
            const auto begin = thread_index * chunk_size;
            if (begin >= work_items.size()) {
                break;
            }
            const auto end = std::min(begin + chunk_size, work_items.size());
            futures.push_back(std::async(std::launch::async, [&, begin, end]() {
                run_chunk(begin, end);
            }));
        }
        for (auto& future : futures) {
            future.get();
        }
    }

    for (std::size_t work_index = 0; work_index < work_items.size(); ++work_index) {
        const auto& item = work_items[work_index];
        cache.by_record[item.record_index].emplace(item.end_track, std::move(end_results[work_index]));
    }

    const auto t1 = std::chrono::steady_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    debug::info_fmt(
        "path precompute: parallel={} records={} work_items={} ms={}",
        enable_parallel,
        records.size(),
        work_items.size(),
        ms);
    return cache;
}

auto count_starttrack_edges(const std::Vector<Net_cost_record>& records) -> std::size_t {
    auto total = std::size_t {0};
    for (const auto& record : records) {
        for (const auto& [end_track, starts] : record.starttrack_by_endtrack) {
            (void)end_track;
            total += starts.size();
        }
    }
    return total;
}

auto apply_tier_to_starttracks(
    std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache,
    const TobTierState& state
) -> std::size_t {
    auto total = std::size_t {0};
    for (std::size_t record_index = 0; record_index < records.size(); ++record_index) {
        auto& record = records[record_index];
        record.starttrack_by_endtrack.clear();
        if (record_index >= cache.by_record.size()) {
            continue;
        }
        const auto tier = state.tier_for_record(record_index);
        const auto& per_end = cache.by_record[record_index];
        for (const auto end_track : record.end_tracks) {
            const auto end_it = per_end.find(end_track);
            if (end_it == per_end.end()) {
                continue;
            }
            const auto& end_data = end_it->second;
            auto starts = std::Vector<std::size_t> {};
            const auto layers_to_open = std::min(tier + 1, end_data.length_layers.size());
            for (std::size_t layer = 0; layer < layers_to_open; ++layer) {
                for (const auto start_track : end_data.length_layers[layer]) {
                    starts.push_back(start_track);
                }
            }
            std::sort(starts.begin(), starts.end());
            starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
            if (!starts.empty()) {
                record.starttrack_by_endtrack.emplace(end_track, std::move(starts));
                total += record.starttrack_by_endtrack[end_track].size();
            }
        }
    }
    return count_starttrack_edges(records);
}

auto expand_tier(
    TobTierState& state,
    const std::Vector<std::size_t>& record_indices,
    std::Vector<Net_cost_record>& records,
    const TobPathPrecomputeCache& cache
) -> ExpandTierResult {
    const auto edges_before = count_starttrack_edges(records);
    auto unique_indices = record_indices;
    std::sort(unique_indices.begin(), unique_indices.end());
    unique_indices.erase(std::unique(unique_indices.begin(), unique_indices.end()), unique_indices.end());

    auto changed = std::Vector<std::size_t> {};
    for (const auto idx : unique_indices) {
        if (idx >= state.tier_by_record.size()) {
            continue;
        }
        ++state.tier_by_record[idx];
        changed.push_back(idx);
    }

    apply_tier_to_starttracks(records, cache, state);
    const auto edges_after = count_starttrack_edges(records);
    return ExpandTierResult {
        std::move(changed),
        edges_after > edges_before ? edges_after - edges_before : 0};
}

auto lookup_path_bbox(
    const TobPathPrecomputeCache& cache,
    const std::size_t record_index,
    const std::size_t end_track,
    const std::size_t start_track
) -> std::optional<IlpBoundingBox> {
    if (record_index >= cache.by_record.size()) {
        return std::nullopt;
    }
    const auto end_it = cache.by_record[record_index].find(end_track);
    if (end_it == cache.by_record[record_index].end()) {
        return std::nullopt;
    }
    const auto start_it = end_it->second.by_start_track.find(start_track);
    if (start_it == end_it->second.by_start_track.end()) {
        return std::nullopt;
    }
    return start_it->second.path_bbox;
}

} // namespace PR_tool
