#include "maze_check/maze_check.hh"

#include "common/ilp_types.hh"
#include "ilp_allocation/ilp_apply_interposer.hh"
#include "maze_check/maze_route_ilp_fixed.hh"
#include "mcf/mcf_graph.hh"

#include <algo/router/common/maze/mazererouter.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/routeerror.hh>
#include <circuit/basedie.hh>
#include <circuit/net/net.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/bbsnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/btsnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <circuit/path/pathpackage.hh>
#include <debug/debug.hh>
#include <hardware/bump/bump.hh>
#include <hardware/cob/cobconnector.hh>
#include <hardware/interposer.hh>
#include <hardware/tob/tob.hh>
#include <hardware/tob/tobconnector.hh>
#include <hardware/track/track.hh>
#include <hardware/track/trackcoord.hh>

#include <algorithm>
#include <chrono>
#include <format>
#include <map>
#include <optional>
#include <set>
#include <tuple>

namespace PR_tool {
namespace {

using RoutedPath = algo::routed_path;

enum class MazeOriginOutcome {
    Pending,
    Ok,
    Failed,
    Skipped
};

struct FailedRecordRef {
    std::size_t record_index{0};
    std::size_t record_id{0};
    std::String net_name;
    std::String origin_key;
    std::String origin_uid;
    std::size_t cob_unit{0};
};

struct OriginMazeState {
    MazeOriginOutcome outcome{MazeOriginOutcome::Pending};
    std::String message;
    std::String path_text;
    std::size_t path_hops{0};
    std::size_t representative_cob_unit{0};
    std::Vector<std::size_t> record_indices;
};

struct MazeCheckContext {
    std::String log_prefix;
    std::String timing_phase;
    std::Vector<FailedRecordRef> failed_records;
    std::map<std::String, OriginMazeState> origin_states;
    MazeCheckSummary summary;
};

auto record_origin_key(const Net_cost_record& record) -> std::String {
    return record.origin_key.empty() ? record.net_name : record.origin_key;
}

auto is_simple_mcf_record(const Net_cost_record& record) -> bool {
    return !is_sync_bus_mcf_origin_key(record_origin_key(record));
}


auto collect_failed_simple_records(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const std::String& log_prefix
) -> std::Vector<FailedRecordRef> {
    auto out = std::Vector<FailedRecordRef> {};
    if (records.size() != ilp_result.record_track_endpoints.size()) {
        debug::warning_fmt("{}: record/SAT endpoint size mismatch; skip failed-record collection", log_prefix);
        return out;
    }
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto& record = records[i];
        if (!is_simple_mcf_record(record)) {
            continue;
        }
        const auto cob_unit = ilp_result.record_track_endpoints[i].cob_unit;
        if (cob_unit >= 16) {
            continue;
        }
        if (!mcf_result.has_simple_commodities[cob_unit]) {
            continue;
        }
        if (mcf_result.simple_mcf_ok[cob_unit]) {
            continue;
        }
        out.push_back(FailedRecordRef {
            i,
            record.record_id,
            record.net_name,
            record_origin_key(record),
            record_origin_group_uid(record),
            cob_unit});
    }
    return out;
}

auto find_net_by_uid(circuit::BaseDie* basedie, const std::String& uid) -> circuit::Net* {
    if (basedie == nullptr) {
        return nullptr;
    }
    for (const auto& net : basedie->nets_to_vector()) {
        if (net->uid() == uid) {
            return net.get();
        }
    }
    return nullptr;
}

auto outcome_label(const MazeOriginOutcome outcome) -> const char* {
    switch (outcome) {
        case MazeOriginOutcome::Ok:
            return "OK";
        case MazeOriginOutcome::Failed:
            return "FAILED";
        case MazeOriginOutcome::Skipped:
            return "SKIP";
        case MazeOriginOutcome::Pending:
        default:
            return "PENDING";
    }
}


auto prepare_maze_check_context(
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const std::String& log_prefix,
    const std::String& timing_phase
) -> MazeCheckContext {
    auto ctx = MazeCheckContext {};
    ctx.log_prefix = log_prefix;
    ctx.timing_phase = timing_phase;
    ctx.failed_records = collect_failed_simple_records(records, ilp_result, mcf_result, log_prefix);
    ctx.summary.failed_records = static_cast<int>(ctx.failed_records.size());

    auto failed_units = std::set<std::size_t> {};
    for (const auto& ref : ctx.failed_records) {
        failed_units.insert(ref.cob_unit);
    }
    ctx.summary.failed_units = static_cast<int>(failed_units.size());

    for (const auto& ref : ctx.failed_records) {
        auto& state = ctx.origin_states[ref.origin_uid];
        if (state.record_indices.empty()) {
            state.representative_cob_unit = ref.cob_unit;
        }
        state.record_indices.push_back(ref.record_index);
    }

    return ctx;
}

auto log_maze_check_summary(const MazeCheckContext& ctx) -> void {
    debug::info_fmt(
        "{} summary: origins_routed={} ok={} failed={} skipped={}",
        ctx.log_prefix,
        ctx.summary.unique_origins_routed,
        ctx.summary.maze_ok,
        ctx.summary.maze_failed,
        ctx.summary.maze_skipped);
}

auto log_record_shared_results(const MazeCheckContext& ctx) -> void {
    for (const auto& ref : ctx.failed_records) {
        const auto it = ctx.origin_states.find(ref.origin_uid);
        if (it == ctx.origin_states.end()) {
            continue;
        }
        debug::info_fmt(
            "{} record_id={} net=\"{}\" origin=\"{}\" COBUnit={} maze={} (shared origin result)",
            ctx.log_prefix,
            ref.record_id,
            ref.net_name,
                ref.origin_key,
            ref.cob_unit,
            outcome_label(it->second.outcome));
    }
}

auto run_maze_check_loop(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfGridDims cob_grid,
    const CobMcfFullResult& mcf_result,
    MazeCheckContext& ctx,
    const auto& route_origin
) -> void {
    if (ctx.failed_records.empty()) {
        debug::info_fmt("{}: no failed SimpleMCF units, skip", ctx.log_prefix);
        return;
    }

    auto failed_units = std::set<std::size_t> {};
    for (const auto& ref : ctx.failed_records) {
        failed_units.insert(ref.cob_unit);
    }
    auto failed_units_text = std::String {};
    for (const auto u : failed_units) {
        if (!failed_units_text.empty()) {
            failed_units_text += ",";
        }
        failed_units_text += std::to_string(u);
    }

    debug::info_fmt(
        "{}: failed SimpleMCF units=[{}] failed_records={} unique_origins={}",
        ctx.log_prefix,
        failed_units_text,
        ctx.summary.failed_records,
        ctx.origin_states.size());

    if (interposer == nullptr || basedie == nullptr) {
        debug::error_fmt("{}: interposer or basedie is null; cannot run maze routing", ctx.log_prefix);
        for (auto& [origin, state] : ctx.origin_states) {
            (void)origin;
            state.outcome = MazeOriginOutcome::Skipped;
            state.message = "null interposer or basedie";
            ctx.summary.maze_skipped += 1;
        }
        return;
    }

    apply_tob_ilp_result_to_interposer(interposer, ilp_result);
    const auto graph = build_mcf_track_graph(cob_grid);
    suspend_mcf_paths_on_interposer(interposer, graph, mcf_result.paths_by_unit);

    auto routed_nets = std::Vector<circuit::Net*> {};

    for (auto& [origin_uid, state] : ctx.origin_states) {
        auto* net = find_net_by_uid(basedie, origin_uid);
        if (net == nullptr) {
            state.outcome = MazeOriginOutcome::Skipped;
            state.message = "net not found in basedie";
            ctx.summary.maze_skipped += 1;
            debug::info_fmt(
                "{} origin=\"{}\" COBUnit={} records={} result=SKIP reason={}",
                ctx.log_prefix,
                origin_uid,
                state.representative_cob_unit,
                state.record_indices.size(),
                state.message);
            continue;
        }

        std::size_t last_failed_record_index {0};
        try {
            net->check_accessable_cobunit();
            interposer->manage_cobunit_resources();
            net->search_related_nets(routed_nets);
            MazeIlpFixedContext mcf_ctx {};
            mcf_ctx.graph = &graph;
            mcf_ctx.mcf_result = &mcf_result;
            mcf_ctx.log_prefix = ctx.log_prefix.c_str();
            mcf_ctx.last_failed_record_index = &last_failed_record_index;
            OriginRouteSegments segments {};
            const auto package = route_origin(
                interposer, net, state.record_indices, records, ilp_result, &mcf_ctx, &segments);
            routed_nets.push_back(net);

            state.outcome = MazeOriginOutcome::Ok;
            state.path_text = segments.by_record_index.empty() ? pathpackage_regular_path_text(package)
                                                               : segments_path_text(segments);
            state.path_hops = package._regular_path.size();
            ctx.summary.maze_ok += 1;
            ctx.summary.unique_origins_routed += 1;

            debug::info_fmt(
                "{} origin=\"{}\" COBUnit={} records={} result=OK path_len={} path=\"{}\"",
                ctx.log_prefix,
                net->name(),
                state.representative_cob_unit,
                state.record_indices.size(),
                state.path_hops,
                state.path_text);
        }
        catch (const algo::RouteExpt& e) {
            state.outcome = MazeOriginOutcome::Failed;
            state.message = e.what();
            ctx.summary.maze_failed += 1;
            ctx.summary.unique_origins_routed += 1;
            debug::info_fmt(
                "{} origin=\"{}\" COBUnit={} records={} failed_at_record_index={} result=FAILED reason=\"{}\"",
                ctx.log_prefix,
                net->name(),
                state.representative_cob_unit,
                state.record_indices.size(),
                last_failed_record_index,
                state.message);
        }
        catch (const std::exception& e) {
            state.outcome = MazeOriginOutcome::Failed;
            state.message = e.what();
            ctx.summary.maze_failed += 1;
            ctx.summary.unique_origins_routed += 1;
            debug::info_fmt(
                "{} origin=\"{}\" COBUnit={} records={} failed_at_record_index={} result=FAILED reason=\"{}\"",
                ctx.log_prefix,
                net->name(),
                state.representative_cob_unit,
                state.record_indices.size(),
                last_failed_record_index,
                state.message);
        }
    }
}

auto run_maze_check_impl(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const CobMcfGridDims cob_grid,
    const std::String& log_prefix,
    const std::String& timing_phase,
    const auto& route_origin
) -> MazeCheckSummary {
    const auto t0 = std::chrono::steady_clock::now();
    auto ctx = prepare_maze_check_context(records, ilp_result, mcf_result, log_prefix, timing_phase);
    run_maze_check_loop(interposer, basedie, records, ilp_result, cob_grid, mcf_result, ctx, route_origin);
    log_record_shared_results(ctx);
    log_maze_check_summary(ctx);
    const auto t1 = std::chrono::steady_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    debug::info_fmt("timing phase={} ms={}", timing_phase, ms);
    return ctx.summary;
}

} // namespace

auto run_maze_check_ilp_mcf_after_mcf(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const CobMcfGridDims cob_grid
) -> MazeCheckSummary {
    const auto maze = algo::MazeRouteStrategy {};
    return run_maze_check_impl(
        interposer,
        basedie,
        records,
        ilp_result,
        mcf_result,
        cob_grid,
        "maze-check-ilp-mcf",
        "maze_check_ilp_mcf",
        [&](hardware::Interposer* ip,
            circuit::Net* net,
            const std::Vector<std::size_t>&,
            const std::Vector<Net_cost_record>&,
            const TobIlpResult&,
            const MazeIlpFixedContext*,
            OriginRouteSegments*) {
            net->route(ip, maze);
            return net->pathpackage();
        });
}

auto run_maze_check_mcf_after_mcf(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const CobMcfFullResult& mcf_result,
    const CobMcfGridDims cob_grid
) -> MazeCheckSummary {
    return run_maze_check_impl(
        interposer,
        basedie,
        records,
        ilp_result,
        mcf_result,
        cob_grid,
        "maze-check-mcf",
        "maze_check_mcf",
        [](hardware::Interposer* ip,
            circuit::Net* net,
            const std::Vector<std::size_t>& record_indices,
            const std::Vector<Net_cost_record>& records,
            const TobIlpResult& ilp_result,
            const MazeIlpFixedContext* mcf_ctx,
            OriginRouteSegments* segments_out) {
            return route_origin_net_ilp_fixed(
                ip, net, record_indices, records, ilp_result, mcf_ctx, segments_out);
        });
}

} // namespace PR_tool
