#include "ilp_v15/v15_ilp_optimizer.hh"

#include "ilp_v15/v15_ilp_extract.hh"
#include "ilp_v15/v15_ilp_model.hh"
#include "ilp_v15/v15_ilp_prepare.hh"
#include "ilp_v15/v15_ilp_segment.hh"
#include "ilp_v15/v15_ilp_validate.hh"
#include "sat/routing_path_log.hh"
#include "sat/routing_round_diagnostics.hh"

#include <algorithm>
#include <chrono>
#include <debug/debug.hh>
#include <format>

namespace PR_tool {

namespace {

constexpr auto kV15Banner =
    "************************************************************************************************************************";

auto paths_for_net(
    const SatRoutingResult& result,
    std::size_t net_id
) -> std::Vector<const SourceSinkPairPath*> {
    auto paths = std::Vector<const SourceSinkPairPath*> {};
    for (const auto& path : result.paths) {
        if (path.net_id == net_id) {
            paths.push_back(&path);
        }
    }
    return paths;
}

auto status_name(V15IlpStatus status) -> const char* {
    switch (status) {
        case V15IlpStatus::SkippedNoCandidates:
            return "SKIPPED_NO_CANDIDATES";
        case V15IlpStatus::Optimal:
            return "OPTIMAL";
        case V15IlpStatus::Suboptimal:
            return "SUBOPTIMAL";
        case V15IlpStatus::Failed:
            return "FAILED";
    }
    return "UNKNOWN";
}

auto format_node_ids(const std::Vector<int>& nodes) -> std::String {
    auto out = std::String {};
    for (std::size_t index = 0; index < nodes.size(); ++index) {
        if (index != 0) {
            out += ",";
        }
        out += std::to_string(nodes[index]);
    }
    return out;
}

auto log_validation(const V15ValidationReport& report, int verbose_level) -> void {
    if (report.ok) {
        debug::info_fmt("v15 ILP validation: PASS paths={}", report.checked_paths);
        return;
    }
    debug::warning_fmt(
        "v15 ILP validation: FAIL paths={} violations={}",
        report.checked_paths,
        report.violations.size());
    if (verbose_level >= 1) {
        for (const auto& violation : report.violations) {
            debug::warning_fmt("  v15 validation: {}", violation);
        }
    }
}

} // namespace

auto optimize_v15_routes(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const DelayPrecomputeResult& delays,
    const SatRoutingResult& sat_result,
    const V15IlpOptimizeOptions& options
) -> V15IlpOptimizeResult {
    auto out = V15IlpOptimizeResult {};
    out.routing = sat_result;
    if (!options.enabled) {
        out.status = V15IlpStatus::SkippedNoCandidates;
        out.message = "DISABLED";
        return out;
    }

    const auto ilp_begin = std::chrono::steady_clock::now();
    const auto elapsed_since_ilp_begin_ms = [&]() -> long long {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - ilp_begin).count();
    };
    const auto stamp_ilp_total = [&](V15IlpOptimizeResult& result) {
        result.stats.total_ms = elapsed_since_ilp_begin_ms();
    };

    debug::info(kV15Banner);
    if (options.time_limit_hours.has_value()) {
        debug::info_fmt(
            "v15 ILP optimization begin: threshold={:.2f}% segment_bbox_pad={} time_limit_hours={} gurobi_log={}/v15_ilp.log",
            options.stretch_threshold_percent,
            options.segment_bbox_pad,
            options.time_limit_hours.value(),
            options.gurobi_log_dir);
    } else {
        debug::info_fmt(
            "v15 ILP optimization begin: threshold={:.2f}% segment_bbox_pad={} time_limit=unlimited gurobi_log={}/v15_ilp.log",
            options.stretch_threshold_percent,
            options.segment_bbox_pad,
            options.gurobi_log_dir);
    }

    const auto stretch = collect_net_stretch_info(interposer, graph, nets, delays, sat_result);
    for (const auto& entry : stretch) {
        if (entry.shortest == 0) {
            debug::warning_fmt(
                "v15 ILP skip net=\"{}\" id={}: theoretical shortest wirelength is zero",
                entry.name,
                entry.net_id);
        }
    }
    out.selected_net_ids = select_v15_net_ids(stretch, options.stretch_threshold_percent);
    out.stats.selected_nets = out.selected_net_ids.size();
    out.stats.locked_nets = nets.size() - out.selected_net_ids.size();
    if (options.verbose_level >= 1) {
        for (const auto& entry : stretch) {
            debug::info_fmt(
                "v15 ILP candidate net=\"{}\" id={} actual={} shortest={} delta={:.2f}% selected={}",
                entry.name,
                entry.net_id,
                entry.actual,
                entry.shortest,
                entry.delta_percent,
                out.selected_net_ids.contains(entry.net_id));
        }
    }
    debug::info_fmt(
        "v15 ILP selection: selected_nets={} locked_nets={} total_nets={}",
        out.stats.selected_nets,
        out.stats.locked_nets,
        nets.size());

    if (out.selected_net_ids.empty()) {
        out.status = V15IlpStatus::SkippedNoCandidates;
        out.message = "SKIPPED_NO_CANDIDATES";
        out.stats.pre_ms = elapsed_since_ilp_begin_ms();
        stamp_ilp_total(out);
        debug::info("v15 ILP optimization end: status=SKIPPED_NO_CANDIDATES");
        debug::info(kV15Banner);
        return out;
    }

    try {
        const auto locked = collect_v15_locked_resources(graph, sat_result, out.selected_net_ids);
        const auto prepared = build_v15_parents_and_segments(
            graph,
            nets,
            scopes,
            sat_result,
            out.selected_net_ids,
            options.segment_bbox_pad,
            locked);
        out.stats.parents = prepared.parents.size();
        out.stats.segments = prepared.segments.size();

        const auto mip_start = build_v15_sat_mip_start(prepared, sat_result);
        std::size_t decomposed_parents = 0;
        std::size_t bus_parents = 0;
        for (const auto& parent : prepared.parents) {
            if (parent.is_bus_member) {
                ++bus_parents;
            }
            if (parent.decomposed) {
                ++decomposed_parents;
            }
            if (options.verbose_level >= 1) {
                debug::info_fmt(
                    "v15 parent id={} net={} fixed_sat_tracks=[{}] tree_nodes={}=>{} tree_edges={}=>{} virtual_hops={}",
                    parent.parent_id,
                    parent.routing_net_id,
                    format_node_ids(parent.fixed_track_nodes),
                    parent.tree_nodes_before_prune,
                    parent.tree_nodes_after_prune,
                    parent.tree_edges_before_prune,
                    parent.tree_edges_after_prune,
                    parent.fixed_track_nodes.size());
            }
        }
        if (options.verbose_level >= 1) {
            for (const auto& segment : prepared.segments) {
                debug::info_fmt(
                    "v15 segment guide coverage: PASS parent={} segment={} endpoints={}=>{} arcs={} virtual_hop={}",
                    segment.parent_id,
                    segment.segment_id,
                    segment.endpoint_a,
                    segment.endpoint_b,
                    segment.guide_arc_ids.size(),
                    segment.is_virtual_hop);
            }
        }
        std::size_t mip_x_count = 0;
        std::size_t mip_y_count = 0;
        std::size_t mip_f_count = 0;
        for (const auto& [parent_id, arcs] : mip_start.parent_arc_ids) {
            (void)parent_id;
            mip_x_count += arcs.size();
        }
        for (const auto& [parent_id, nodes] : mip_start.parent_node_ids) {
            (void)parent_id;
            mip_y_count += nodes.size();
        }
        for (const auto& [segment_id, arcs] : mip_start.segment_flow_arc_ids) {
            (void)segment_id;
            mip_f_count += arcs.size();
        }
        debug::info_fmt(
            "v15 ILP preparation: parents={} segments={} decomposed_parents={} bus_parents={} locked_nodes={} locked_switches={} mip_start={} F_start={} x_start={} y_start={}",
            prepared.parents.size(),
            prepared.segments.size(),
            decomposed_parents,
            bus_parents,
            std::count(locked.node_used.begin(), locked.node_used.end(), true),
            locked.switch_used.size(),
            mip_start.available,
            mip_f_count,
            mip_x_count,
            mip_y_count);

        const auto model_begin = std::chrono::steady_clock::now();
        const auto model_result =
            solve_v15_ilp_model(graph, prepared, locked, options, mip_start);
        const auto model_end = std::chrono::steady_clock::now();
        out.stats = model_result.stats;
        out.stats.selected_nets = out.selected_net_ids.size();
        out.stats.locked_nets = nets.size() - out.selected_net_ids.size();
        out.stats.parents = prepared.parents.size();
        out.stats.segments = prepared.segments.size();
        if (out.stats.model_build_ms == 0) {
            out.stats.model_build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                model_end - model_begin).count() - out.stats.solve_ms;
        }
        // prep + model build = wall until optimize returns, minus Gurobi solve.
        out.stats.pre_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            model_end - ilp_begin).count() - out.stats.solve_ms;
        if (out.stats.pre_ms < 0) {
            out.stats.pre_ms = 0;
        }
        out.status = model_result.status;
        out.message = model_result.message;
        debug::info_fmt(
            "v15 ILP model: F={} x={} y={} M={} constraints={} nonzeros={} model_build_ms={} gurobi_optimize_ms={} status={} solutions={}",
            out.stats.f_vars,
            out.stats.x_vars,
            out.stats.y_vars,
            out.stats.mode_vars,
            out.stats.constraints,
            out.stats.nonzeros,
            out.stats.model_build_ms,
            out.stats.solve_ms,
            status_name(out.status),
            out.stats.solution_count);

        if (!model_result.ok) {
            out.status = V15IlpStatus::Failed;
            out.routing = sat_result;
            stamp_ilp_total(out);
            debug::warning_fmt(
                "v15 ILP optimization failed: {} fallback_to_SAT=true",
                model_result.message);
            debug::info(kV15Banner);
            return out;
        }

        const auto extracted = extract_v15_routing_solution(
            graph,
            nets,
            prepared,
            model_result,
            sat_result,
            out.selected_net_ids);
        const auto validation = validate_v15_routing_solution(
            graph,
            nets,
            scopes,
            prepared,
            model_result,
            extracted,
            out.selected_net_ids);
        log_validation(validation, options.verbose_level);
        if (!extracted.ok || !validation.ok) {
            out.status = V15IlpStatus::Failed;
            out.message = extracted.ok ? "V15_VALIDATION_FAILED" : extracted.message;
            out.routing = sat_result;
            stamp_ilp_total(out);
            debug::warning_fmt(
                "v15 ILP optimization failed: {} fallback_to_SAT=true",
                out.message);
            debug::info(kV15Banner);
            return out;
        }

        for (const auto& entry : stretch) {
            if (!out.selected_net_ids.contains(entry.net_id)) {
                continue;
            }
            const auto before = net_wirelength(graph, paths_for_net(sat_result, entry.net_id));
            const auto after = net_wirelength(graph, paths_for_net(extracted, entry.net_id));
            debug::info_fmt(
                "v15 ILP net id={} wirelength={} -> {} delta={:+d}",
                entry.net_id,
                before,
                after,
                static_cast<long long>(after) - static_cast<long long>(before));
        }
        debug::info_fmt(
            "v15 ILP objective={:.0f} best_bound={:.0f} mip_gap={:.6f} total_wirelength={} -> {}",
            out.stats.objective,
            out.stats.best_bound,
            out.stats.mip_gap,
            sat_result.total_wirelength,
            extracted.total_wirelength);
        out.routing = extracted;
        stamp_ilp_total(out);
        debug::info_fmt("v15 ILP optimization end: status={}", status_name(out.status));
        debug::info(kV15Banner);
        return out;
    }
    catch (const V15PreparationInvariantError& error) {
        debug::error_fmt(
            "v15 ILP preparation invariant failed: {}; terminating without SAT fallback",
            error.what());
        debug::info(kV15Banner);
        throw;
    }
    catch (const std::exception& error) {
        out.status = V15IlpStatus::Failed;
        out.message = error.what();
        out.routing = sat_result;
        out.stats.pre_ms = elapsed_since_ilp_begin_ms();
        stamp_ilp_total(out);
        debug::warning_fmt(
            "v15 ILP optimization exception: {} fallback_to_SAT=true",
            out.message);
        debug::info(kV15Banner);
        return out;
    }
}

} // namespace PR_tool
