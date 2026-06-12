#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "mcf/mcf_graph.hh"
#include "precompute/ilp_bounding_box.hh"
#include "precompute/tob_path_precompute.hh"

#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct McfCommodityBBox {
    bool restricted{false};
    IlpBoundingBox box {};
};

struct McfBBoxCommodityInput {
    std::size_t record_index{0};
    bool is_bus{false};
    std::String bus_key;
};

struct McfBBoxContext {
    std::size_t max_tier{0};
    std::Vector<std::size_t> tier_by_record;
    std::Vector<McfCommodityBBox> per_commodity;
    std::map<std::String, IlpBoundingBox> per_bus_key;
};

enum class McfArcBBoxMode {
    Bus,
    SimpleCommodity,
    SimpleOriginGroup,
};

auto physical_arc_in_bbox(const McfArc& arc, const IlpBoundingBox& bbox, int cols) -> bool;

auto track_node_allowed_in_mcf_bbox(const McfNodeMeta& node, const IlpBoundingBox& bbox) -> bool;

auto arc_allowed_in_mcf_bbox(
    const McfArc& arc,
    const McfNodeMeta& u,
    const McfNodeMeta& v,
    bool restricted,
    const IlpBoundingBox& bbox,
    int cols,
    int endpoint_src = -1,
    int endpoint_snk = -1
) -> bool;

auto build_mcf_bbox_context(
    const std::Vector<Net_cost_record>& records,
    const std::Vector<McfBBoxCommodityInput>& commodities,
    const TobIlpResult& ilp_result,
    const TobPathPrecomputeCache& cache
) -> McfBBoxContext;

auto compute_origin_group_bbox(
    const std::Vector<std::size_t>& global_commodity_ids,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const McfBBoxContext& ctx
) -> McfCommodityBBox;

auto resolve_mcf_bbox(
    const McfBBoxContext& ctx,
    std::size_t commodity_index,
    const McfBBoxCommodityInput& commodity,
    McfArcBBoxMode mode,
    const McfCommodityBBox& origin_group_bbox
) -> McfCommodityBBox;

} // namespace PR_tool
