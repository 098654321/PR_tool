#include "maze_check/simple_maze_routing.hh"

#include "ilp_allocation/ilp_apply_interposer.hh"
#include "maze_check/maze_route_ilp_fixed.hh"
#include "mcf/mcf_graph.hh"

#include <algo/router/routeerror.hh>
#include <circuit/basedie.hh>
#include <circuit/net/net.hh>
#include <debug/debug.hh>
#include <hardware/track/track.hh>
#include <hardware/track/trackcoord.hh>

#include <algorithm>
#include <chrono>
#include <format>
#include <map>
#include <set>

namespace PR_tool {
namespace {

struct OriginWorkItem {
    std::String origin_uid;
    std::Vector<std::size_t> record_indices;
    std::size_t representative_cob_unit{0};
    float port_number{0.0f};
};

auto find_net_by_uid(circuit::BaseDie& basedie, const std::String& uid) -> circuit::Net* {
    for (const auto& net : basedie.nets_to_vector()) {
        if (net->uid() == uid) {
            return net.get();
        }
    }
    return nullptr;
}

auto node_from_track_coord(
    const McfGlobalGraph& graph,
    const std::size_t unit,
    const hardware::TrackCoord& tc
) -> int {
    const int dir = tc.dir == hardware::TrackDirection::Horizontal ? 0 : 1;
    const McfNodeKey key {
        unit,
        dir,
        static_cast<int>(tc.row),
        static_cast<int>(tc.col),
        tc.index};
    const auto it = graph.node_id_by_key.find(key);
    if (it == graph.node_id_by_key.end()) {
        return -1;
    }
    return it->second;
}

auto build_unit_path_from_routed(
    const McfGlobalGraph& graph,
    const SimpleMazeCommodityInput& commodity,
    const algo::routed_path& routed
) -> std::Vector<int> {
    auto unit_path = std::Vector<int> {};
    if (commodity.src >= 0) {
        unit_path.push_back(commodity.src);
    }
    for (const auto& [track, connector] : routed) {
        (void)connector;
        if (track == nullptr) {
            continue;
        }
        const auto node = node_from_track_coord(graph, commodity.cob_unit, track->coord());
        if (node < 0) {
            continue;
        }
        if (unit_path.empty() || unit_path.back() != node) {
            unit_path.push_back(node);
        }
    }
    if (commodity.snk >= 0 && (unit_path.empty() || unit_path.back() != commodity.snk)) {
        unit_path.push_back(commodity.snk);
    }
    return unit_path;
}

auto build_track_path_from_routed(const algo::routed_path& routed) -> std::Vector<std::size_t> {
    auto track_path = std::Vector<std::size_t> {};
    for (const auto& [track, connector] : routed) {
        (void)connector;
        if (track == nullptr) {
            continue;
        }
        const auto idx = track->coord().index;
        if (track_path.empty() || track_path.back() != idx) {
            track_path.push_back(idx);
        }
    }
    return track_path;
}

auto mcf_path_info_from_commodity(
    const McfGlobalGraph& graph,
    const SimpleMazeCommodityInput& commodity,
    const OriginRouteSegments& segments
) -> McfPathInfo {
    auto info = McfPathInfo {};
    info.label = commodity.label;
    info.origin_name = commodity.origin_name;
    info.record_id = commodity.record_id;
    info.src = commodity.src;
    info.snk = commodity.snk;
    info.demand = commodity.demand;
    info.cob_unit = commodity.cob_unit;
    info.start_track = commodity.start_track;
    info.end_track = commodity.end_track;
    info.record_indices = commodity.record_indices.empty() ? std::Vector<std::size_t> {commodity.record_index}
                                                           : commodity.record_indices;

    const auto seg_it = segments.by_record_index.find(commodity.record_index);
    if (seg_it != segments.by_record_index.end()) {
        const auto unit_path = build_unit_path_from_routed(graph, commodity, seg_it->second);
        const auto track_path = build_track_path_from_routed(seg_it->second);
        if (!unit_path.empty()) {
            info.unit_paths.push_back(unit_path);
        }
        if (!track_path.empty()) {
            info.track_paths.push_back(std::move(track_path));
        }
    }
    return info;
}

auto collect_origin_work_items(
    circuit::BaseDie& basedie,
    const std::Vector<SimpleMazeCommodityInput>& commodities
) -> std::Vector<OriginWorkItem> {
    auto by_uid = std::map<std::String, OriginWorkItem> {};
    for (const auto& commodity : commodities) {
        auto& item = by_uid[commodity.origin_uid];
        if (item.origin_uid.empty()) {
            item.origin_uid = commodity.origin_uid;
            item.representative_cob_unit = commodity.cob_unit;
            if (auto* net = find_net_by_uid(basedie, commodity.origin_uid)) {
                item.port_number = static_cast<float>(net->port_number());
            }
        }
        if (std::find(item.record_indices.begin(), item.record_indices.end(), commodity.record_index)
            == item.record_indices.end()) {
            item.record_indices.push_back(commodity.record_index);
        }
    }

    auto out = std::Vector<OriginWorkItem> {};
    out.reserve(by_uid.size());
    for (auto& [uid, item] : by_uid) {
        (void)uid;
        std::sort(item.record_indices.begin(), item.record_indices.end());
        out.push_back(std::move(item));
    }
    std::sort(out.begin(), out.end(), [](const OriginWorkItem& a, const OriginWorkItem& b) {
        return a.port_number > b.port_number;
    });
    return out;
}

auto commodities_for_origin(
    const std::Vector<SimpleMazeCommodityInput>& commodities,
    const std::String& origin_uid
) -> std::Vector<const SimpleMazeCommodityInput*> {
    auto out = std::Vector<const SimpleMazeCommodityInput*> {};
    for (const auto& commodity : commodities) {
        if (commodity.origin_uid == origin_uid) {
            out.push_back(&commodity);
        }
    }
    return out;
}

auto mark_unit_flags(
    const std::Vector<SimpleMazeCommodityInput>& commodities,
    const std::map<std::String, bool>& origin_ok,
    SimpleMazeSolveResult& out
) -> void {
    auto unit_has = std::array<bool, 16> {};
    auto unit_all_ok = std::array<bool, 16> {};
    unit_all_ok.fill(true);

    for (const auto& commodity : commodities) {
        if (commodity.cob_unit >= 16) {
            continue;
        }
        unit_has[commodity.cob_unit] = true;
        const auto it = origin_ok.find(commodity.origin_uid);
        if (it == origin_ok.end() || !it->second) {
            unit_all_ok[commodity.cob_unit] = false;
        }
    }

    for (std::size_t u = 0; u < 16; ++u) {
        out.has_simple_commodities[u] = unit_has[u];
        out.simple_mcf_ok[u] = !unit_has[u] || unit_all_ok[u];
    }
}

} // namespace

auto solve_simple_mcf_with_maze(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const McfGlobalGraph& graph,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const std::Vector<SimpleMazeCommodityInput>& commodities,
    const std::array<std::Vector<McfPathInfo>, 16>& bus_paths_by_unit,
    const bool verbose_maze_records
) -> SimpleMazeSolveResult {
    const auto t0 = std::chrono::steady_clock::now();
    auto out = SimpleMazeSolveResult {};

    if (interposer == nullptr) {
        out.all_ok = false;
        debug::error("simple-maze: interposer is null");
        return out;
    }
    if (commodities.empty()) {
        out.solve_ms = 0;
        return out;
    }

    interposer->reset_regs();
    apply_tob_ilp_result_to_interposer(interposer, ilp_result);
    suspend_mcf_paths_on_interposer(interposer, graph, bus_paths_by_unit);

    auto session_result = CobMcfFullResult {};
    session_result.paths_by_unit = bus_paths_by_unit;

    const auto origins = collect_origin_work_items(basedie, commodities);
    auto origin_ok = std::map<std::String, bool> {};
    auto routed_nets = std::Vector<circuit::Net*> {};

    debug::info_fmt("simple-maze: routing {} origin net(s) sequentially", origins.size());

    for (const auto& origin : origins) {
        origin_ok[origin.origin_uid] = false;
        auto* net = find_net_by_uid(basedie, origin.origin_uid);
        if (net == nullptr) {
            out.all_ok = false;
            debug::info_fmt(
                "simple-maze origin=\"{}\" COBUnit={} records={} result=FAILED reason=\"net not found in basedie\"",
                origin.origin_uid,
                origin.representative_cob_unit,
                origin.record_indices.size());
            continue;
        }

        std::size_t last_failed_record_index {0};
        try {
            net->check_accessable_cobunit();
            interposer->manage_cobunit_resources();
            net->search_related_nets(routed_nets);

            MazeIlpFixedContext ctx {};
            ctx.graph = &graph;
            ctx.mcf_result = &session_result;
            ctx.use_session_paths = true;
            ctx.log_prefix = "simple-maze";
            ctx.verbose_records = verbose_maze_records;
            ctx.last_failed_record_index = &last_failed_record_index;

            OriginRouteSegments segments {};
            const auto package = route_origin_net_ilp_fixed(
                interposer,
                net,
                origin.record_indices,
                records,
                ilp_result,
                &ctx,
                &segments);

            routed_nets.push_back(net);

            for (const auto* commodity : commodities_for_origin(commodities, origin.origin_uid)) {
                auto info = mcf_path_info_from_commodity(graph, *commodity, segments);
                out.paths_by_unit[commodity->cob_unit].push_back(std::move(info));
            }
            session_result.paths_by_unit = bus_paths_by_unit;
            for (std::size_t u = 0; u < 16; ++u) {
                for (const auto& info : out.paths_by_unit[u]) {
                    session_result.paths_by_unit[u].push_back(info);
                }
            }

            origin_ok[origin.origin_uid] = true;
            const auto path_summary = verbose_maze_records ? segments_path_text(segments)
                                                           : pathpackage_regular_path_text(package);
            debug::info_fmt(
                "simple-maze origin=\"{}\" COBUnit={} records={} result=OK path_len={} path=\"{}\"",
                net->name(),
                origin.representative_cob_unit,
                origin.record_indices.size(),
                package._regular_path.size(),
                path_summary);
        }
        catch (const algo::RouteExpt& e) {
            out.all_ok = false;
            debug::info_fmt(
                "simple-maze origin=\"{}\" COBUnit={} records={} failed_at_record_index={} result=FAILED reason=\"{}\"",
                net->name(),
                origin.representative_cob_unit,
                origin.record_indices.size(),
                last_failed_record_index,
                e.what());
        }
        catch (const std::exception& e) {
            out.all_ok = false;
            debug::info_fmt(
                "simple-maze origin=\"{}\" COBUnit={} records={} failed_at_record_index={} result=FAILED reason=\"{}\"",
                net->name(),
                origin.representative_cob_unit,
                origin.record_indices.size(),
                last_failed_record_index,
                e.what());
        }
    }

    mark_unit_flags(commodities, origin_ok, out);

    const auto t1 = std::chrono::steady_clock::now();
    out.solve_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count());
    debug::info_fmt(
        "simple-maze summary: origins={} all_ok={} solve_ms={}",
        origins.size(),
        out.all_ok,
        out.solve_ms);
    return out;
}

} // namespace PR_tool
