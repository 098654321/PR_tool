#include "precompute/ilp_bounding_box.hh"

#include "mcf/mcf_hw_map.hh"

#include <hardware/interposer.hh>
#include <hardware/tob/tobcoord.hh>

#include <algorithm>
#include <cctype>
#include <format>

namespace PR_tool {

namespace {

auto tob_from_linear(std::size_t tob_linear) -> hardware::TOBCoord {
    const auto width = static_cast<std::size_t>(hardware::Interposer::TOB_ARRAY_WIDTH);
    return hardware::TOBCoord {
        static_cast<std::i64>(tob_linear / width),
        static_cast<std::i64>(tob_linear % width)};
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

} // namespace

auto tob_anchor_cob(const std::size_t tob_linear) -> hardware::COBCoord {
    const auto tob = tob_from_linear(tob_linear);
    return hardware::COBCoord {
        static_cast<std::i64>(1 + 2 * tob.row),
        static_cast<std::i64>(3 * tob.col)};
}

auto merge_coord_into_bbox(IlpBoundingBox& box, const hardware::COBCoord& cob) -> void {
    box.row_min = std::min(box.row_min, cob.row);
    box.row_max = std::max(box.row_max, cob.row);
    box.col_min = std::min(box.col_min, cob.col);
    box.col_max = std::max(box.col_max, cob.col);
}

auto bbox_area(const IlpBoundingBox& box) -> std::size_t {
    const auto rows = static_cast<std::size_t>(box.row_max - box.row_min + 1);
    const auto cols = static_cast<std::size_t>(box.col_max - box.col_min + 1);
    return rows * cols;
}

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

auto clamp_bbox_to_cob_array(IlpBoundingBox box) -> IlpBoundingBox {
    return clamp_bbox(box);
}

auto full_cob_array_bbox() -> IlpBoundingBox {
    const auto max_row = static_cast<std::i64>(hardware::Interposer::COB_ARRAY_HEIGHT) - 1;
    const auto max_col = static_cast<std::i64>(hardware::Interposer::COB_ARRAY_WIDTH) - 1;
    return IlpBoundingBox {0, max_row, 0, max_col};
}

auto is_full_cob_array_bbox(const IlpBoundingBox& box) -> bool {
    const auto full = full_cob_array_bbox();
    return box.row_min == full.row_min && box.row_max == full.row_max && box.col_min == full.col_min
        && box.col_max == full.col_max;
}

auto expand_bbox_by_one(IlpBoundingBox& box) -> bool {
    const auto before = box;
    box.row_min -= 1;
    box.row_max += 1;
    box.col_min -= 1;
    box.col_max += 1;
    box = clamp_bbox_to_cob_array(box);
    return box.row_min != before.row_min || box.row_max != before.row_max || box.col_min != before.col_min
        || box.col_max != before.col_max;
}

auto format_bbox(const IlpBoundingBox& box) -> std::String {
    return std::format("({},{},{},{})", box.row_min, box.row_max, box.col_min, box.col_max);
}

} // namespace PR_tool
