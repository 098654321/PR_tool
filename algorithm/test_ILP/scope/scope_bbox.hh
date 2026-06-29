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

auto compute_scope_bbox_for_net(const RoutingNet& net) -> IlpBoundingBox;

auto assign_scope_bboxes(std::Vector<RoutingNet>& nets) -> void;

} // namespace PR_tool
