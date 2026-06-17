#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "mcf/mcf_graph.hh"
#include "precompute/ilp_bounding_box.hh"
#include "precompute/tob_path_precompute.hh"

#include <std/collection.hh>
#include <std/string.hh>

#include <functional>
#include <optional>

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

struct McfBBoxExpandState {
    std::map<std::pair<std::size_t, std::String>, IlpBoundingBox> simple_origin_hull;
};

struct McfBBoxExpandResult {
    bool any_exhausted{false};
    std::String exhausted_key;
};

using McfSimpleOriginGroupKey = std::pair<std::size_t, std::String>;

enum class McfArcBBoxMode {
    Bus,
    SimpleCommodity,
    SimpleOriginGroup,
    /// Caller-supplied bbox in origin_group_bbox (e.g. refined segment override).
    SimpleExplicit,
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

auto expand_bus_hulls(McfBBoxContext& ctx, const std::Vector<std::String>& bus_keys) -> McfBBoxExpandResult;

auto expand_simple_origin_hulls(
    McfBBoxExpandState& state,
    const std::Vector<McfSimpleOriginGroupKey>& origin_groups,
    const std::function<IlpBoundingBox(const McfSimpleOriginGroupKey&)>& base_hull_for
) -> McfBBoxExpandResult;

auto lookup_simple_origin_hull(
    const McfBBoxExpandState& state,
    const McfSimpleOriginGroupKey& key
) -> std::optional<IlpBoundingBox>;

auto bbox_from_physical_guide_path(
    const McfGlobalGraph& graph,
    const std::Vector<int>& guide_path
) -> McfCommodityBBox;

/// Tight hull from guide_path, then expand until guide_path_bbox_connected or array limit.
auto segment_bbox_from_guide_path(
    const McfGlobalGraph& graph,
    const std::Vector<int>& guide_path,
    std::size_t cob_unit
) -> McfCommodityBBox;

auto commodity_bbox_connected(
    const McfGlobalGraph& graph,
    int src,
    int snk,
    std::size_t cob_unit,
    const McfCommodityBBox& effective_bbox
) -> bool;

/// True when every hop on guide_path is allowed in bbox (preferred segment validation).
auto guide_path_bbox_connected(
    const McfGlobalGraph& graph,
    const std::Vector<int>& guide_path,
    std::size_t cob_unit,
    const McfCommodityBBox& effective_bbox
) -> bool;

} // namespace PR_tool
