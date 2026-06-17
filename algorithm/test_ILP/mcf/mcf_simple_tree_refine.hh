#pragma once

#include "mcf/cob_mcf_router.hh"
#include "mcf/mcf_bbox.hh"
#include "mcf/mcf_graph.hh"

#include <std/collection.hh>
#include <std/string.hh>

#include <optional>

namespace PR_tool {

constexpr std::size_t kMcfSyntheticRecordIdBase = 1'000'000'000ULL;
constexpr std::size_t kMcfSyntheticRecordIdPerUnit = 10'000ULL;
constexpr double kSimpleMcfTreeSeedMipGap = 0.30;

inline auto mcf_synthetic_record_id(const std::size_t unit_c, const std::size_t local_seg_idx) -> std::size_t {
    return kMcfSyntheticRecordIdBase + unit_c * kMcfSyntheticRecordIdPerUnit + local_seg_idx;
}

struct SimpleTreeSegment {
    int src{-1};
    int snk{-1};
    std::Vector<int> guide_path;
    McfCommodityBBox bbox {};
    std::String parent_origin_key;
    std::Vector<std::size_t> parent_record_indices;
};

struct OriginTreeCompressResult {
    std::String origin_key;
    bool fallback{false};
    std::Vector<SimpleTreeSegment> segments;
    std::Vector<std::size_t> child_record_indices;
    std::Vector<std::size_t> child_record_id_list;
};

struct UnitTreeCompressSummary {
    std::Vector<OriginTreeCompressResult> per_origin;
    std::size_t refined_segment_count{0};
    std::size_t fallback_origin_count{0};
};

auto compress_origin_paths_to_segments(
    const McfGlobalGraph& graph,
    const std::String& origin_key,
    const std::Vector<McfPathInfo>& child_paths,
    const std::set<int>& physical_endpoints
) -> OriginTreeCompressResult;

auto compress_unit_multi_fanout_origins(
    const McfGlobalGraph& graph,
    const std::Vector<McfPathInfo>& stage1_paths,
    const std::Vector<Net_cost_record>& records,
    const std::size_t unit_c,
    const std::function<std::String(const Net_cost_record&)>& origin_group_key_fn
) -> UnitTreeCompressSummary;

auto merge_refined_paths_for_output(
    const McfGlobalGraph& graph,
    const std::Vector<McfPathInfo>& stage1_paths,
    const std::Vector<McfPathInfo>& refine_paths,
    const UnitTreeCompressSummary& compress_summary,
    const std::Vector<Net_cost_record>& records
) -> std::Vector<McfPathInfo>;

} // namespace PR_tool
