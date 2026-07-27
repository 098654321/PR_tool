#pragma once

#include "common/routing_types.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>

namespace PR_tool {

auto clamp_bbox_to_cob_array(IlpBoundingBox box) -> IlpBoundingBox;

auto merge_coord_into_bbox(IlpBoundingBox& box, const hardware::COBCoord& cob) -> void;

auto rect_hull_boxes(const std::Vector<IlpBoundingBox>& boxes) -> IlpBoundingBox;

auto format_bbox(const IlpBoundingBox& box) -> std::String;

auto compute_scope_child_bboxes(const RoutingNet& net) -> std::Vector<IlpBoundingBox>;

auto compute_pnnet_demand_pair_bbox(const RoutingNet& net, std::size_t demand_index) -> IlpBoundingBox;

auto compute_scope_bbox_for_net(const RoutingNet& net) -> IlpBoundingBox;

auto assign_scope_bboxes(std::Vector<RoutingNet>& nets) -> void;

auto full_chip_bbox() -> IlpBoundingBox;

auto expand_pair_bbox_one_cell(IlpBoundingBox box) -> IlpBoundingBox;

auto is_full_chip_bbox(const IlpBoundingBox& box) -> bool;

} // namespace PR_tool
