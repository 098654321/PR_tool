#include "mcf/mcf_bbox.hh"

#include <debug/debug.hh>

#include <hardware/cob/cob.hh>

#include <format>
#include <queue>
#include <stdexcept>

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

auto path_bbox_for_record(
    const std::size_t record_index,
    const Net_cost_record& record,
    const TobIlpRecordTrackEndpoint& endpoint,
    const TobPathPrecomputeCache& cache
) -> std::optional<IlpBoundingBox> {
    if (!endpoint.has_start_track || !endpoint.has_end_track) {
        return std::nullopt;
    }
    return lookup_path_bbox(cache, record_index, endpoint.end_track, endpoint.start_track);
}

auto is_endpoint_node(const int node_id, const int endpoint_src, const int endpoint_snk) -> bool {
    return node_id >= 0 && (node_id == endpoint_src || node_id == endpoint_snk);
}

auto arc_incident_to_endpoint(const McfArc& arc, const int endpoint_src, const int endpoint_snk) -> bool {
    return is_endpoint_node(arc.u, endpoint_src, endpoint_snk)
        || is_endpoint_node(arc.v, endpoint_src, endpoint_snk);
}

auto endpoint_incident_arc_in_bbox(
    const McfArc& arc,
    const IlpBoundingBox& bbox,
    const int cols,
    const int endpoint_src,
    const int endpoint_snk
) -> bool {
    if (!arc_incident_to_endpoint(arc, endpoint_src, endpoint_snk)) {
        return false;
    }
    if (arc.cob < 0) {
        return false;
    }
    return bbox.contains(cob_from_linear(arc.cob, cols));
}

auto node_allowed_in_mcf_bbox(
    const McfNodeMeta& node,
    const int node_id,
    const IlpBoundingBox& bbox,
    const int endpoint_src,
    const int endpoint_snk
) -> bool {
    if (is_endpoint_node(node_id, endpoint_src, endpoint_snk)) {
        return true;
    }
    return track_node_allowed_in_mcf_bbox(node, bbox);
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

auto track_node_allowed_in_mcf_bbox(const McfNodeMeta& node, const IlpBoundingBox& bbox) -> bool {
    if (node.is_virtual) {
        return true;
    }
    if (node.track_dir == 0) {
        return node.track_col != bbox.col_min && node.track_col != bbox.col_max + 1;
    }
    return node.track_row != bbox.row_min && node.track_row != bbox.row_max + 1;
}

auto arc_allowed_in_mcf_bbox(
    const McfArc& arc,
    const McfNodeMeta& u,
    const McfNodeMeta& v,
    const bool restricted,
    const IlpBoundingBox& bbox,
    const int cols,
    const int endpoint_src,
    const int endpoint_snk
) -> bool {
    if (!restricted) {
        return true;
    }
    if (!physical_arc_in_bbox(arc, bbox, cols)
        && !endpoint_incident_arc_in_bbox(arc, bbox, cols, endpoint_src, endpoint_snk)) {
        return false;
    }
    if (!node_allowed_in_mcf_bbox(u, arc.u, bbox, endpoint_src, endpoint_snk)) {
        return false;
    }
    if (!node_allowed_in_mcf_bbox(v, arc.v, bbox, endpoint_src, endpoint_snk)) {
        return false;
    }
    return true;
}

auto build_mcf_bbox_context(
    const std::Vector<Net_cost_record>& records,
    const std::Vector<McfBBoxCommodityInput>& commodities,
    const TobIlpResult& ilp_result,
    const TobPathPrecomputeCache& cache
) -> McfBBoxContext {
    McfBBoxContext ctx {};
    ctx.max_tier = ilp_result.max_tier;
    ctx.tier_by_record = ilp_result.tier_by_record;
    ctx.per_commodity.resize(commodities.size());

    std::size_t restricted_count = 0;
    for (std::size_t i = 0; i < commodities.size(); ++i) {
        const auto& input = commodities[i];
        const auto& record = records[input.record_index];
        auto& out = ctx.per_commodity[i];
        out.restricted = true;
        if (input.record_index >= ilp_result.record_track_endpoints.size()) {
            throw std::runtime_error(std::format(
                "MCF bbox: missing endpoint for record_index={} record_id={} net=\"{}\"",
                input.record_index,
                record.record_id,
                record.net_name));
        }
        const auto& endpoint = ilp_result.record_track_endpoints[input.record_index];
        const auto bbox_opt = path_bbox_for_record(input.record_index, record, endpoint, cache);
        if (!bbox_opt.has_value()) {
            throw std::runtime_error(std::format(
                "MCF bbox: missing path bbox for record_index={} record_id={} net=\"{}\" end_track={} start_track={}",
                input.record_index,
                record.record_id,
                record.net_name,
                endpoint.end_track,
                endpoint.start_track));
        }
        out.box = *bbox_opt;
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
        "MCF bbox: max_tier={} commodities={} restricted={} bus_groups={}",
        ctx.max_tier,
        commodities.size(),
        restricted_count,
        ctx.per_bus_key.size());
    return ctx;
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
    if (mode == McfArcBBoxMode::SimpleExplicit) {
        return origin_group_bbox;
    }
    if (commodity_index < ctx.per_commodity.size()) {
        return ctx.per_commodity[commodity_index];
    }
    return McfCommodityBBox {};
}

auto expand_bus_hulls(McfBBoxContext& ctx, const std::Vector<std::String>& bus_keys) -> McfBBoxExpandResult {
    auto out = McfBBoxExpandResult {};
    for (const auto& bus_key : bus_keys) {
        const auto it = ctx.per_bus_key.find(bus_key);
        if (it == ctx.per_bus_key.end()) {
            continue;
        }
        auto& box = it->second;
        if (!expand_bbox_by_one(box)) {
            out.any_exhausted = true;
            out.exhausted_key = bus_key;
        }
    }
    return out;
}

auto undirected_edge_key(const int u, const int v) -> std::pair<int, int> {
    if (u <= v) {
        return {u, v};
    }
    return {v, u};
}

auto append_arc_cob_coords(
    const McfArc& arc,
    const int cols,
    std::Vector<hardware::COBCoord>& out
) -> void {
    if (arc.is_virtual || arc.cob < 0) {
        return;
    }
    const auto cob = cob_from_linear(arc.cob, cols);
    out.push_back(cob);
    if (arc.is_turn) {
        return;
    }
    if (is_straight_through(arc.from_dir, arc.to_dir)) {
        using D = hardware::COBDirection;
        const bool lr = (arc.from_dir == D::Left && arc.to_dir == D::Right)
            || (arc.from_dir == D::Right && arc.to_dir == D::Left);
        if (lr) {
            out.push_back(hardware::COBCoord {cob.row, cob.col + 1});
        }
        else {
            out.push_back(hardware::COBCoord {cob.row + 1, cob.col});
        }
    }
}

auto bbox_hull_from_cob_coords(const std::Vector<hardware::COBCoord>& coords) -> IlpBoundingBox {
    if (coords.empty()) {
        return {};
    }
    auto box = IlpBoundingBox {
        coords.front().row,
        coords.front().col,
        coords.front().row,
        coords.front().col};
    for (std::size_t i = 1; i < coords.size(); ++i) {
        box.row_min = std::min(box.row_min, coords[i].row);
        box.row_max = std::max(box.row_max, coords[i].row);
        box.col_min = std::min(box.col_min, coords[i].col);
        box.col_max = std::max(box.col_max, coords[i].col);
    }
    return box;
}

auto find_undirected_arc(
    const McfGlobalGraph& graph,
    const int u,
    const int v
) -> const McfArc* {
    for (const auto& arc : graph.arcs) {
        if (arc.is_virtual) {
            continue;
        }
        if ((arc.u == u && arc.v == v) || (arc.u == v && arc.v == u)) {
            return &arc;
        }
    }
    return nullptr;
}

auto bbox_from_physical_guide_path(
    const McfGlobalGraph& graph,
    const std::Vector<int>& guide_path
) -> McfCommodityBBox {
    auto coords = std::Vector<hardware::COBCoord> {};
    for (const auto node : guide_path) {
        if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size()) {
            continue;
        }
        const auto& meta = graph.nodes[static_cast<std::size_t>(node)];
        if (meta.is_virtual) {
            continue;
        }
        coords.push_back(hardware::COBCoord {
            static_cast<std::i64>(meta.track_row),
            static_cast<std::i64>(meta.track_col)});
    }
    for (std::size_t i = 0; i + 1 < guide_path.size(); ++i) {
        const auto u = guide_path[i];
        const auto v = guide_path[i + 1];
        const auto* arc = find_undirected_arc(graph, u, v);
        if (arc != nullptr) {
            append_arc_cob_coords(*arc, graph.cols, coords);
        }
    }
    if (coords.empty()) {
        return McfCommodityBBox {};
    }
    return McfCommodityBBox {true, bbox_hull_from_cob_coords(coords)};
}

auto commodity_bbox_connected(
    const McfGlobalGraph& graph,
    const int src,
    const int snk,
    const std::size_t cob_unit,
    const McfCommodityBBox& effective_bbox
) -> bool {
    if (!effective_bbox.restricted) {
        return true;
    }
    auto adj = std::map<int, std::Vector<std::pair<int, int>>> {};
    for (std::size_t a = 0; a < graph.arcs.size(); ++a) {
        const auto& arc = graph.arcs[a];
        if (arc.unit != cob_unit) {
            continue;
        }
        adj[arc.u].push_back({arc.v, static_cast<int>(a)});
        adj[arc.v].push_back({arc.u, static_cast<int>(a)});
    }
    auto prev = std::vector<int>(graph.nodes.size(), -1);
    auto q = std::queue<int> {};
    q.push(src);
    prev[static_cast<std::size_t>(src)] = src;
    while (!q.empty()) {
        const auto node = q.front();
        q.pop();
        if (node == snk) {
            return true;
        }
        const auto it = adj.find(node);
        if (it == adj.end()) {
            continue;
        }
        for (const auto [next, arc_id] : it->second) {
            if (prev[static_cast<std::size_t>(next)] != -1) {
                continue;
            }
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            const auto from = arc.u == node ? arc.u : arc.v;
            const auto to = arc.u == node ? arc.v : arc.u;
            if (!arc_allowed_in_mcf_bbox(
                    arc,
                    graph.nodes[static_cast<std::size_t>(from)],
                    graph.nodes[static_cast<std::size_t>(to)],
                    true,
                    effective_bbox.box,
                    graph.cols,
                    src,
                    snk)) {
                continue;
            }
            prev[static_cast<std::size_t>(next)] = node;
            q.push(next);
        }
    }
    return false;
}

auto guide_path_bbox_connected(
    const McfGlobalGraph& graph,
    const std::Vector<int>& guide_path,
    const std::size_t cob_unit,
    const McfCommodityBBox& effective_bbox
) -> bool {
    if (!effective_bbox.restricted || guide_path.size() < 2) {
        return guide_path.size() >= 2;
    }
    const auto src = guide_path.front();
    const auto snk = guide_path.back();
    for (std::size_t i = 0; i + 1 < guide_path.size(); ++i) {
        const auto u = guide_path[i];
        const auto v = guide_path[i + 1];
        const auto* arc = find_undirected_arc(graph, u, v);
        if (arc == nullptr) {
            return false;
        }
        if (arc->unit != cob_unit) {
            return false;
        }
        const auto from = arc->u == u ? arc->u : arc->v;
        const auto to = arc->u == u ? arc->v : arc->u;
        if (!arc_allowed_in_mcf_bbox(
                *arc,
                graph.nodes[static_cast<std::size_t>(from)],
                graph.nodes[static_cast<std::size_t>(to)],
                true,
                effective_bbox.box,
                graph.cols,
                src,
                snk)) {
            return false;
        }
    }
    return true;
}

auto segment_bbox_from_guide_path(
    const McfGlobalGraph& graph,
    const std::Vector<int>& guide_path,
    const std::size_t cob_unit
) -> McfCommodityBBox {
    auto bbox = bbox_from_physical_guide_path(graph, guide_path);
    if (!bbox.restricted || guide_path.size() < 2) {
        return bbox;
    }
    while (!guide_path_bbox_connected(graph, guide_path, cob_unit, bbox)) {
        if (!expand_bbox_by_one(bbox.box)) {
            break;
        }
    }
    return bbox;
}

auto lookup_simple_origin_hull(
    const McfBBoxExpandState& state,
    const McfSimpleOriginGroupKey& key
) -> std::optional<IlpBoundingBox> {
    const auto it = state.simple_origin_hull.find(key);
    if (it == state.simple_origin_hull.end()) {
        return std::nullopt;
    }
    return it->second;
}

auto expand_simple_origin_hulls(
    McfBBoxExpandState& state,
    const std::Vector<McfSimpleOriginGroupKey>& origin_groups,
    const std::function<IlpBoundingBox(const McfSimpleOriginGroupKey&)>& base_hull_for
) -> McfBBoxExpandResult {
    auto out = McfBBoxExpandResult {};
    for (const auto& key : origin_groups) {
        auto box = IlpBoundingBox {};
        const auto it = state.simple_origin_hull.find(key);
        if (it != state.simple_origin_hull.end()) {
            box = it->second;
        }
        else {
            box = base_hull_for(key);
        }
        if (!expand_bbox_by_one(box)) {
            out.any_exhausted = true;
            out.exhausted_key = std::format("unit{}:{}", key.first, key.second);
        }
        state.simple_origin_hull[key] = box;
    }
    return out;
}

} // namespace PR_tool
