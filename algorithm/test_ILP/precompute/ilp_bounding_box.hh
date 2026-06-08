#pragma once

#include "common/ilp_types.hh"

#include <hardware/cob/cobcoord.hh>

#include <cstddef>
#include <std/collection.hh>

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

auto compute_bounding_box_level0(const Net_cost_record& record) -> IlpBoundingBox;

auto expand_bounding_box(const IlpBoundingBox& base, std::size_t range_level) -> IlpBoundingBox;

auto compute_bounding_box(const Net_cost_record& record, std::size_t range_level) -> IlpBoundingBox;

auto rect_hull_boxes(const std::Vector<IlpBoundingBox>& boxes) -> IlpBoundingBox;

} // namespace PR_tool
