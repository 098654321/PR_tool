#include "scope/scope_bbox.hh"

#include "common/hw_map.hh"

#include <algorithm>
#include <format>

namespace PR_tool {

namespace {

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

auto bump_cob(const Bump_coord& bump) -> hardware::COBCoord {
    return tob_anchor_cob(bump.TOB);
}

auto compute_bnet_bbox(const Bump_coord& start_bump, const Bump_coord& end_bump) -> IlpBoundingBox {
    const auto start = bump_cob(start_bump);
    const auto end = bump_cob(end_bump);
    const auto [start_tr, start_tc] = tob_index_from_linear(start_bump.TOB);
    const auto [end_tr, end_tc] = tob_index_from_linear(end_bump.TOB);

    if (start_tc != end_tc && start_tr != end_tr) {
        const auto top_row = std::max(start.row, end.row);
        const auto bottom_row = std::min(start.row, end.row);
        const auto left_col = std::min(start.col, end.col);
        const auto right_col = std::max(start.col, end.col);
        return clamp_bbox(IlpBoundingBox {bottom_row, top_row - 1, left_col, right_col});
    }
    if (start_tr == end_tr && start_tc != end_tc) {
        const auto row = start.row;
        return clamp_bbox(IlpBoundingBox {row - 1, row, std::min(start.col, end.col), std::max(start.col, end.col)});
    }
    return clamp_bbox(IlpBoundingBox {
        std::min(start.row, end.row),
        std::max(start.row, end.row),
        start.col,
        start.col});
}

auto compute_tnet_bbox(const hardware::TrackCoord& track_coord, const Bump_coord& bump) -> IlpBoundingBox {
    const auto port = track_to_cob(track_coord);
    const auto tob = bump_cob(bump);

    if (tob.col != port.col && port.row >= tob.row) {
        return clamp_bbox(IlpBoundingBox {
            tob.row,
            port.row,
            std::min(port.col, tob.col),
            std::max(port.col, tob.col)});
    }
    if (tob.col != port.col && port.row < tob.row) {
        return clamp_bbox(IlpBoundingBox {
            port.row,
            tob.row - 1,
            std::min(port.col, tob.col),
            std::max(port.col, tob.col)});
    }
    if (tob.col == port.col && port.row >= tob.row) {
        return clamp_bbox(IlpBoundingBox {tob.row, port.row, port.col, port.col});
    }
    return clamp_bbox(IlpBoundingBox {port.row, tob.row - 1, port.col, port.col});
}

auto endpoint_coords(const RoutingNet& net) -> std::Vector<hardware::COBCoord> {
    auto coords = std::Vector<hardware::COBCoord> {};
    for (const auto& src : net.sources) {
        if (src.kind == GraphNodeRef::Kind::Bump) {
            coords.emplace_back(bump_cob(src.bump));
        }
        else if (src.kind == GraphNodeRef::Kind::Track) {
            coords.emplace_back(track_to_cob(src.track_coord));
        }
    }
    for (const auto& demand : net.demands) {
        if (demand.sink.kind == GraphNodeRef::Kind::Bump) {
            coords.emplace_back(bump_cob(demand.sink.bump));
        }
        else if (demand.sink.kind == GraphNodeRef::Kind::Track) {
            coords.emplace_back(track_to_cob(demand.sink.track_coord));
        }
    }
    return coords;
}

auto ensure_endpoints_in_bbox(IlpBoundingBox box, const RoutingNet& net) -> IlpBoundingBox {
    for (const auto& cob : endpoint_coords(net)) {
        merge_coord_into_bbox(box, cob);
    }
    return clamp_bbox(box);
}

} // namespace

auto clamp_bbox_to_cob_array(IlpBoundingBox box) -> IlpBoundingBox {
    return clamp_bbox(box);
}

auto merge_coord_into_bbox(IlpBoundingBox& box, const hardware::COBCoord& cob) -> void {
    box.row_min = std::min(box.row_min, cob.row);
    box.row_max = std::max(box.row_max, cob.row);
    box.col_min = std::min(box.col_min, cob.col);
    box.col_max = std::max(box.col_max, cob.col);
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

auto format_bbox(const IlpBoundingBox& box) -> std::String {
    return std::format("({},{},{},{})", box.row_min, box.row_max, box.col_min, box.col_max);
}

auto compute_scope_child_bboxes(const RoutingNet& net) -> std::Vector<IlpBoundingBox> {
    auto boxes = std::Vector<IlpBoundingBox> {};

    if (net.kind == RoutingNetKind::Bnet) {
        for (const auto& demand : net.demands) {
            if (demand.candidate_source_indices.empty()) {
                continue;
            }
            const auto source_index = demand.candidate_source_indices.front();
            if (source_index >= net.sources.size()) {
                continue;
            }
            boxes.emplace_back(compute_bnet_bbox(
                net.sources[source_index].bump,
                demand.sink.bump));
        }
        return boxes;
    }

    if (net.kind == RoutingNetKind::Tnet) {
        for (const auto& demand : net.demands) {
            if (demand.candidate_source_indices.empty()) {
                continue;
            }
            const auto source_index = demand.candidate_source_indices.front();
            if (source_index >= net.sources.size()) {
                continue;
            }
            boxes.emplace_back(compute_tnet_bbox(
                net.sources[source_index].track_coord,
                demand.sink.bump));
        }
        return boxes;
    }

    if (net.kind == RoutingNetKind::PNnet) {
        for (std::size_t demand_index = 0; demand_index < net.demands.size(); ++demand_index) {
            boxes.push_back(compute_pnnet_demand_pair_bbox(net, demand_index));
        }
        return boxes;
    }

    return boxes;
}

auto compute_pnnet_demand_pair_bbox(const RoutingNet& net, std::size_t demand_index) -> IlpBoundingBox {
    if (net.kind != RoutingNetKind::PNnet || demand_index >= net.demands.size()) {
        return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
    }
    const auto& demand = net.demands[demand_index];
    auto candidate_boxes = std::Vector<IlpBoundingBox> {};
    candidate_boxes.reserve(demand.candidate_source_indices.size());
    for (const std::size_t source_index : demand.candidate_source_indices) {
        if (source_index >= net.sources.size()) {
            continue;
        }
        candidate_boxes.emplace_back(compute_tnet_bbox(
            net.sources[source_index].track_coord,
            demand.sink.bump));
    }
    if (candidate_boxes.empty()) {
        return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
    }
    return rect_hull_boxes(candidate_boxes);
}

auto compute_scope_bbox_for_net(const RoutingNet& net) -> IlpBoundingBox {
    const auto child_boxes = compute_scope_child_bboxes(net);
    if (child_boxes.empty()) {
        return clamp_bbox(IlpBoundingBox {0, 0, 0, 0});
    }
    return ensure_endpoints_in_bbox(rect_hull_boxes(child_boxes), net);
}

auto assign_scope_bboxes(std::Vector<RoutingNet>& nets) -> void {
    for (auto& net : nets) {
        net.scope_bbox = compute_scope_bbox_for_net(net);
        net.has_scope_bbox = true;
    }
}

auto full_chip_bbox() -> IlpBoundingBox {
    const auto max_row = static_cast<std::i64>(hardware::Interposer::COB_ARRAY_HEIGHT) - 1;
    const auto max_col = static_cast<std::i64>(hardware::Interposer::COB_ARRAY_WIDTH) - 1;
    return clamp_bbox(IlpBoundingBox {0, max_row, 0, max_col});
}

auto expand_pair_bbox_one_cell(IlpBoundingBox box) -> IlpBoundingBox {
    box.row_min -= 1;
    box.row_max += 1;
    box.col_min -= 1;
    box.col_max += 1;
    return clamp_bbox(box);
}

auto is_full_chip_bbox(const IlpBoundingBox& box) -> bool {
    const auto chip = full_chip_bbox();
    return box.row_min <= chip.row_min
        && box.row_max >= chip.row_max
        && box.col_min <= chip.col_min
        && box.col_max >= chip.col_max;
}

} // namespace PR_tool
