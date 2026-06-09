#include "mcf/mcf_bbox.hh"

#include "debug/debug.hh"

#include <hardware/cob/cob.hh>

namespace PR_tool {

namespace {

auto cob_from_linear(int cob_linear, int cols) -> hardware::COBCoord {
    return hardware::COBCoord {
        static_cast<std::i64>(cob_linear / cols),
        static_cast<std::i64>(cob_linear % cols)};
}

auto is_straight_through(
    hardware::COBDirection from,
    hardware::COBDirection to
) -> bool {
    using D = hardware::COBDirection;
    return (from == D::Left && to == D::Right) || (from == D::Right && to == D::Left)
        || (from == D::Up && to == D::Down) || (from == D::Down && to == D::Up);
}

} // namespace

auto physical_arc_in_bbox(const McfArc& arc, const IlpBoundingBox& bbox, const int cols) -> bool {
    if (arc.is_virtual || arc.cob < 0) {
        return true;
    }
    const auto cob = cob_from_linear(arc.cob, cols);
    if (arc.is_turn) {
        return bbox.contains(cob);
    }
    if (is_straight_through(arc.from_dir, arc.to_dir)) {
        using D = hardware::COBDirection;
        const bool lr = (arc.from_dir == D::Left && arc.to_dir == D::Right)
            || (arc.from_dir == D::Right && arc.to_dir == D::Left);
        if (lr) {
            const hardware::COBCoord neighbor {
                cob.row,
                cob.col + 1};
            return bbox.contains(cob) && bbox.contains(neighbor);
        }
        const hardware::COBCoord neighbor {
            cob.row + 1,
            cob.col};
        return bbox.contains(cob) && bbox.contains(neighbor);
    }
    return bbox.contains(cob);
}

auto arc_allowed_in_mcf_bbox(
    const McfArc& arc,
    const bool restricted,
    const IlpBoundingBox& bbox,
    const int cols
) -> bool {
    if (!restricted) {
        return true;
    }
    return physical_arc_in_bbox(arc, bbox, cols);
}

auto build_mcf_bbox_context(
    const std::Vector<Net_cost_record>& records,
    const std::Vector<McfBBoxCommodityInput>& commodities,
    const TobBBoxExpansionState& state
) -> McfBBoxContext {
    McfBBoxContext ctx {};
    ctx.range_level = state.max_rho();
    ctx.bbox_expand_by_record = state.rho_by_record;
    ctx.per_commodity.resize(commodities.size());

    std::size_t restricted_count = 0;
    for (std::size_t i = 0; i < commodities.size(); ++i) {
        const auto& input = commodities[i];
        const auto& record = records[input.record_index];
        auto& out = ctx.per_commodity[i];
        if (record.type == Net_type::PNnet) {
            out.restricted = false;
            continue;
        }
        out.restricted = true;
        out.box = compute_bounding_box(record, state.rho_for_record(input.record_index));
        ++restricted_count;
    }

    auto bus_members = std::map<std::String, std::Vector<std::size_t>> {};
    for (std::size_t i = 0; i < commodities.size(); ++i) {
        if (!commodities[i].is_bus) {
            continue;
        }
        bus_members[commodities[i].bus_key].push_back(i);
    }
    for (const auto& [bus_key, members] : bus_members) {
        auto boxes = std::Vector<IlpBoundingBox> {};
        boxes.reserve(members.size());
        for (const auto idx : members) {
            const auto& bb = ctx.per_commodity[idx];
            if (bb.restricted) {
                boxes.push_back(bb.box);
            }
        }
        if (!boxes.empty()) {
            ctx.per_bus_key[bus_key] = rect_hull_boxes(boxes);
        }
    }

    debug::info_fmt(
        "MCF bbox: max_rho={} commodities={} restricted={} bus_groups={}",
        state.max_rho(),
        commodities.size(),
        restricted_count,
        ctx.per_bus_key.size());
    return ctx;
}

auto build_mcf_bbox_context(
    const std::Vector<Net_cost_record>& records,
    const std::Vector<McfBBoxCommodityInput>& commodities,
    const std::size_t range_level
) -> McfBBoxContext {
    return build_mcf_bbox_context(records, commodities, TobBBoxExpansionState::uniform(records.size(), range_level));
}

auto compute_origin_group_bbox(
    const std::Vector<std::size_t>& global_commodity_ids,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& ctx
) -> McfCommodityBBox {
    auto boxes = std::Vector<IlpBoundingBox> {};
    const auto count = std::min(global_commodity_ids.size(), record_indices.size());
    for (std::size_t i = 0; i < count; ++i) {
        const auto global_idx = global_commodity_ids[i];
        const auto& record = records[record_indices[i]];
        if (record.type == Net_type::PNnet) {
            continue;
        }
        if (global_idx >= ctx.per_commodity.size()) {
            continue;
        }
        const auto& bb = ctx.per_commodity[global_idx];
        if (!bb.restricted) {
            continue;
        }
        boxes.push_back(bb.box);
    }
    if (boxes.empty()) {
        return McfCommodityBBox {};
    }
    if (boxes.size() == 1) {
        return McfCommodityBBox {true, boxes.front()};
    }
    return McfCommodityBBox {true, rect_hull_boxes(boxes)};
}

auto resolve_mcf_bbox(
    const McfBBoxContext& ctx,
    const std::size_t commodity_index,
    const McfBBoxCommodityInput& commodity,
    const McfArcBBoxMode mode,
    const McfCommodityBBox& origin_group_bbox
) -> McfCommodityBBox {
    if (mode == McfArcBBoxMode::Bus) {
        const auto it = ctx.per_bus_key.find(commodity.bus_key);
        if (it == ctx.per_bus_key.end()) {
            return McfCommodityBBox {};
        }
        return McfCommodityBBox {true, it->second};
    }
    if (mode == McfArcBBoxMode::SimpleOriginGroup) {
        return origin_group_bbox;
    }
    if (commodity_index < ctx.per_commodity.size()) {
        return ctx.per_commodity[commodity_index];
    }
    return McfCommodityBBox {};
}

} // namespace PR_tool
