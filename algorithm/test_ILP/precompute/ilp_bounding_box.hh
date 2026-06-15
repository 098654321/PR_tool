#pragma once

#include "common/ilp_types.hh"

#include <hardware/cob/cobcoord.hh>
#include <hardware/tob/tobcoord.hh>

#include <cstddef>
#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct IlpBoundingBox {
    std::i64 row_min{0};
    std::i64 row_max{0};
    std::i64 col_min{0};
    std::i64 col_max{0};

    auto contains(const hardware::COBCoord& cob) const -> bool {
        return cob.row >= row_min && cob.row <= row_max && cob.col >= col_min && cob.col <= col_max;
    }
};

constexpr std::string_view kSyncBusOriginPrefix = "SyncNet in group ";

auto is_sync_bus_record(const Net_cost_record& record) -> bool;

auto tob_anchor_cob(std::size_t tob_linear) -> hardware::COBCoord;

auto rect_hull_boxes(const std::Vector<IlpBoundingBox>& boxes) -> IlpBoundingBox;

auto merge_coord_into_bbox(IlpBoundingBox& box, const hardware::COBCoord& cob) -> void;

auto bbox_area(const IlpBoundingBox& box) -> std::size_t;

auto clamp_bbox_to_cob_array(IlpBoundingBox box) -> IlpBoundingBox;

auto full_cob_array_bbox() -> IlpBoundingBox;

auto is_full_cob_array_bbox(const IlpBoundingBox& box) -> bool;

/// Expand row/col bounds by 1 in each direction, clamped to the COB array.
/// Returns false if the bbox did not change (already at full array extent).
auto expand_bbox_by_one(IlpBoundingBox& box) -> bool;

auto format_bbox(const IlpBoundingBox& box) -> std::String;

} // namespace PR_tool
