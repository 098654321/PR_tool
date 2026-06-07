#include "precompute/ilp_bounding_box.hh"

#include "mcf/mcf_hw_map.hh"

#include <hardware/interposer.hh>
#include <hardware/tob/tobcoord.hh>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace PR_tool {

namespace {

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

auto clamp_bbox(IlpBoundingBox box) -> IlpBoundingBox {
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

auto rect_hull(const IlpBoundingBox& a, const IlpBoundingBox& b) -> IlpBoundingBox {
    return clamp_bbox(IlpBoundingBox {
        std::min(a.row_min, b.row_min),
        std::max(a.row_max, b.row_max),
        std::min(a.col_min, b.col_min),
        std::max(a.col_max, b.col_max)});
}

auto bbox_bnet(const Bump_coord& start_bump, const Bump_coord& end_bump) -> IlpBoundingBox {
    const auto s_tob = tob_from_linear(start_bump.TOB);
    const auto e_tob = tob_from_linear(end_bump.TOB);
    const auto s = tob_anchor_cob(start_bump.TOB);
    const auto e = tob_anchor_cob(end_bump.TOB);

    const auto dr = std::llabs(e_tob.row - s_tob.row);
    const auto dc = std::llabs(e_tob.col - s_tob.col);

    IlpBoundingBox box {};
    if (dc == 0) {
        box.row_min = std::min(s.row, e.row);
        box.row_max = std::max(s.row, e.row);
        box.col_min = s.col;
        box.col_max = s.col;
    }
    else if (dr == 0) {
        box.row_min = s.row - 1;
        box.row_max = s.row;
        box.col_min = std::min(s.col, e.col);
        box.col_max = std::max(s.col, e.col);
    }
    else {
        const auto upper_row = std::max(s.row, e.row);
        const auto lower_row = std::min(s.row, e.row);
        box.row_min = lower_row;
        box.row_max = upper_row - 1;
        box.col_min = std::min(s.col, e.col);
        box.col_max = std::max(s.col, e.col);
    }
    return clamp_bbox(box);
}

auto bbox_tnet(std::size_t start_tob_linear, const hardware::TrackCoord& end_track) -> IlpBoundingBox {
    const auto tob = tob_anchor_cob(start_tob_linear);
    const auto port = mcf::track_to_cob(end_track);

    IlpBoundingBox box {};
    if (port.col == tob.col) {
        if (port.row >= tob.row) {
            box.row_min = tob.row;
            box.row_max = port.row;
        }
        else {
            box.row_min = port.row;
            box.row_max = tob.row - 1;
        }
        box.col_min = tob.col;
        box.col_max = tob.col;
    }
    else if (port.row >= tob.row) {
        box.row_min = tob.row;
        box.row_max = port.row;
        box.col_min = std::min(tob.col, port.col);
        box.col_max = std::max(tob.col, port.col);
    }
    else {
        box.row_min = port.row;
        box.row_max = tob.row - 1;
        box.col_min = std::min(tob.col, port.col);
        box.col_max = std::max(tob.col, port.col);
    }
    return clamp_bbox(box);
}

} // namespace

auto is_sync_bus_record(const Net_cost_record& record) -> bool {
    const auto& origin_key = record.origin_key.empty() ? record.net_name : record.origin_key;
    if (!origin_key.starts_with(kSyncBusOriginPrefix)) {
        return false;
    }
    const auto suffix = origin_key.substr(kSyncBusOriginPrefix.size());
    if (suffix.empty()) {
        return false;
    }
    for (const char ch : suffix) {
        if (!std::isdigit(static_cast<unsigned char>(ch))) {
            return false;
        }
    }
    return std::stoi(std::string(suffix)) > 0;
}

auto compute_bounding_box_level0(const Net_cost_record& record) -> IlpBoundingBox {
    if (record.type == Net_type::Bnet) {
        if (record.start_bumps.empty() || record.end_bumps.empty()) {
            return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
        }
        return bbox_bnet(record.start_bumps.front(), record.end_bumps.front());
    }
    if (record.type == Net_type::Tnet) {
        if (record.start_bumps.empty()) {
            return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
        }
        if (record.mcf_has_end_track) {
            return bbox_tnet(record.start_bumps.front().TOB, record.mcf_end_track);
        }
        if (record.mcf_has_start_track) {
            return bbox_tnet(record.start_bumps.front().TOB, record.mcf_start_track);
        }
        return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
    }
    if (record.start_bumps.empty()) {
        return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
    }
    auto merged = IlpBoundingBox {
        std::numeric_limits<std::i64>::max(),
        std::numeric_limits<std::i64>::min(),
        std::numeric_limits<std::i64>::max(),
        std::numeric_limits<std::i64>::min()};
    bool any = false;
    for (const auto end_track : record.pn_end_tracks) {
        const auto it = record.pn_end_track_coord_by_index.find(end_track);
        if (it == record.pn_end_track_coord_by_index.end()) {
            continue;
        }
        const auto box = bbox_tnet(record.start_bumps.front().TOB, it->second);
        if (!any) {
            merged = box;
            any = true;
        }
        else {
            merged = rect_hull(merged, box);
        }
    }
    if (!any) {
        return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
    }
    return merged;
}

auto expand_bounding_box(const IlpBoundingBox& base, const std::size_t range_level, const bool is_bus_net) -> IlpBoundingBox {
    if (range_level == 0 || is_bus_net) {
        return base;
    }
    auto box = base;
    const auto expand = static_cast<std::i64>(range_level);
    box.row_min -= expand;
    box.row_max += expand;
    box.col_min -= expand;
    box.col_max += expand;
    return clamp_bbox(box);
}

auto compute_bounding_box(const Net_cost_record& record, const std::size_t range_level) -> IlpBoundingBox {
    const auto base = compute_bounding_box_level0(record);
    return expand_bounding_box(base, range_level, is_sync_bus_record(record));
}

auto rect_hull_boxes(const std::Vector<IlpBoundingBox>& boxes) -> IlpBoundingBox {
    if (boxes.empty()) {
        return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
    }
    auto merged = boxes.front();
    for (std::size_t i = 1; i < boxes.size(); ++i) {
        merged = rect_hull(merged, boxes[i]);
    }
    return merged;
}

} // namespace PR_tool
