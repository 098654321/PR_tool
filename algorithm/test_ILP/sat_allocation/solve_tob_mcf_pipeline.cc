#include "sat_allocation/solve_tob_mcf_pipeline.hh"

#include "mcf/mcf_graph.hh"
#include "precompute/ilp_bounding_box.hh"
#include "sat_allocation/solve_tob_sat.hh"

#include <algorithm>
#include <chrono>
#include <debug/debug.hh>
#include <format>
#include <set>

namespace PR_tool {

namespace {

auto format_record_indices(const std::Vector<std::size_t>& indices) -> std::String {
    if (indices.empty()) {
        return "{}";
    }
    auto out = std::String {"{"};
    for (std::size_t i = 0; i < indices.size(); ++i) {
        if (i != 0) {
            out += ",";
        }
        out += std::format("{}", indices[i]);
    }
    out += "}";
    return out;
}

auto format_string_vector(const std::Vector<std::String>& values) -> std::String {
    if (values.empty()) {
        return "{}";
    }
    auto out = std::String {"{"};
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i != 0) {
            out += ",";
        }
        out += values[i];
    }
    out += "}";
    return out;
}

auto sat_failure_expansion_records(const std::Vector<Net_cost_record>& records) -> std::Vector<std::size_t> {
    auto out = std::Vector<std::size_t> {};
    for (std::size_t i = 0; i < records.size(); ++i) {
        if (records[i].type == Net_type::Tnet || records[i].type == Net_type::PNnet) {
            out.push_back(i);
        }
    }
    return out;
}

auto bus_member_record_indices(
    const std::Vector<Net_cost_record>& records,
    const std::Vector<std::String>& bus_keys
) -> std::Vector<std::size_t> {
    auto key_set = std::set<std::String> {bus_keys.begin(), bus_keys.end()};
    auto out = std::Vector<std::size_t> {};
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto& record = records[i];
        const auto origin_name = record.origin_key.empty() ? record.net_name : record.origin_key;
        if (!is_sync_bus_record(record)) {
            continue;
        }
        if (key_set.contains(record_origin_group_uid(record)) || key_set.contains(origin_name)) {
            out.push_back(i);
        }
    }
    return out;
}

auto log_mcf_failure_reason(const CobMcfFullResult& mcf, const TobBBoxExpansionState& state) -> void {
    if (mcf.retry_hints.bus_failure_unlocalized) {
        debug::info_fmt(
            "bbox iteration: MCF failed at max_rho={}, BusMCF bus_key unlocalized",
            state.max_rho());
        return;
    }
    if (!mcf.retry_hints.failed_bus_keys.empty()) {
        debug::info_fmt(
            "bbox iteration: MCF failed at max_rho={}, BusMCF bus_keys={}",
            state.max_rho(),
            format_string_vector(mcf.retry_hints.failed_bus_keys));
        return;
    }
    if (!mcf.retry_hints.failed_simple_units.empty()) {
        debug::info_fmt(
            "bbox iteration: MCF failed at max_rho={}, SimpleMCF units={}",
            state.max_rho(),
            format_record_indices(mcf.retry_hints.failed_simple_units));
        return;
    }
    debug::info_fmt(
        "bbox iteration: MCF failed at max_rho={}, no retry hint records",
        state.max_rho());
}

} // namespace

auto solve_tob_mcf_with_range_iteration(
    std::Vector<Net_cost_record>& records,
    hardware::Interposer* interposer,
    const circuit::BaseDie& basedie,
    const CobMcfGridDims cob_grid,
    const bool enable_mcf_parallel,
    const bool enable_pre_routing,
    const bool enable_mcf_obj,
    const bool disable_bus_mcf,
    const bool enable_simple_maze,
    const bool apply_interposer_suspend_on_success,
    const CadicalDiagnosticsOptions& sat_diag,
    const GurobiDiagnosticsOptions& gurobi_diag
) -> TobMcfPipelineResult {
    auto out = TobMcfPipelineResult {};
    auto state = TobBBoxExpansionState::initial(records.size());
    const auto sat_fail_set = sat_failure_expansion_records(records);
    const auto max_attempts = records.size() * kTobBBoxMaxExpand + 1;

    for (std::size_t attempt = 0; attempt < max_attempts; ++attempt) {
        out.attempts += 1;
        debug::info_fmt(
            "SAT+MCF bbox attempt={} max_rho={}",
            attempt,
            state.max_rho());

        const auto sat_t0 = std::chrono::steady_clock::now();
        auto tob = solve_tob_sat_with_bbox_state(records, state, sat_diag);
        const auto sat_t1 = std::chrono::steady_clock::now();
        out.tob_sat_solve_ms += std::chrono::duration_cast<std::chrono::milliseconds>(sat_t1 - sat_t0).count();

        if (!tob.ok) {
            if (tob.model_status == 20) {
                debug::info_fmt("bbox iteration: attempt={} SAT=UNSAT max_rho={}", attempt, state.max_rho());
                const auto changed = state.expand_records(sat_fail_set);
                debug::info_fmt(
                    "bbox iteration: SAT expand changed_records={} max_rho={}",
                    format_record_indices(changed),
                    state.max_rho());
                if (changed.empty()) {
                    out.ok = false;
                    out.message = std::format(
                        "SAT+MCF: SAT UNSAT and no Tnet/PNnet bbox record can expand further (max_rho={})",
                        state.max_rho());
                    out.tob = std::move(tob);
                    out.range_level = state.max_rho();
                    debug::error_fmt("{}", out.message);
                    return out;
                }
            }
            else {
                debug::error_fmt(
                    "bbox iteration: attempt={} SAT=parse_fail ({})",
                    attempt,
                    tob.message);
                out.ok = false;
                out.message = tob.message;
                out.tob = std::move(tob);
                out.range_level = state.max_rho();
                return out;
            }
            continue;
        }

        debug::info_fmt("bbox iteration: attempt={} SAT=SAT max_rho={}", attempt, state.max_rho());

        const auto mcf = run_mcf_global_routing_cob_units(
            records,
            tob,
            interposer,
            basedie,
            cob_grid,
            enable_mcf_parallel,
            enable_pre_routing,
            enable_mcf_obj,
            true,
            disable_bus_mcf,
            enable_simple_maze,
            sat_diag.verbose_reach_endpoints,
            gurobi_diag);

        out.mcf_warm_start_ms += mcf.summary.mcf_warm_start_ms;
        out.mcf_solve_ms += mcf.summary.mcf_solve_ms;
        out.bus_mcf_solve_ms += mcf.summary.bus_mcf_solve_ms;
        for (std::size_t u = 0; u < 16; ++u) {
            out.simple_mcf_solve_ms_by_unit[u] += mcf.summary.simple_mcf_solve_ms_by_unit[u];
        }

        if (!mcf.summary.all_ok) {
            debug::info_fmt("bbox iteration: attempt={} SAT=SAT MCF=fail max_rho={}", attempt, state.max_rho());
            log_mcf_failure_reason(mcf, state);
            if (mcf.retry_hints.bus_failure_unlocalized) {
                out.ok = false;
                out.message = "SAT+MCF: BusMCF failed but failed bus_key could not be localized";
                out.tob = std::move(tob);
                out.mcf = mcf;
                out.range_level = state.max_rho();
                debug::error_fmt("{}", out.message);
                return out;
            }

            auto fail_set = std::Vector<std::size_t> {};
            if (!mcf.retry_hints.failed_bus_keys.empty()) {
                fail_set = bus_member_record_indices(records, mcf.retry_hints.failed_bus_keys);
                if (fail_set.empty()) {
                    fail_set = mcf.retry_hints.failed_record_indices;
                }
            }
            else {
                fail_set = mcf.retry_hints.failed_record_indices;
            }
            std::sort(fail_set.begin(), fail_set.end());
            fail_set.erase(std::unique(fail_set.begin(), fail_set.end()), fail_set.end());

            const auto changed = state.expand_records(fail_set);
            debug::info_fmt(
                "bbox iteration: MCF expand fail_set={} changed_records={} max_rho={}",
                format_record_indices(fail_set),
                format_record_indices(changed),
                state.max_rho());
            if (changed.empty()) {
                out.ok = false;
                out.message = std::format(
                    "SAT+MCF: MCF failed and no localized bbox record can expand further (max_rho={})",
                    state.max_rho());
                out.tob = std::move(tob);
                out.mcf = mcf;
                out.range_level = state.max_rho();
                debug::error_fmt("{}", out.message);
                return out;
            }
            continue;
        }

        debug::info_fmt("bbox iteration: attempt={} SAT=SAT MCF=ok max_rho={}", attempt, state.max_rho());

        if (apply_interposer_suspend_on_success && interposer != nullptr) {
            const auto graph = build_mcf_track_graph(cob_grid);
            suspend_mcf_paths_on_interposer(interposer, graph, mcf.paths_by_unit);
        }

        out.ok = true;
        out.message = std::format("SAT+MCF solved with bbox max_rho={}", state.max_rho());
        out.tob = std::move(tob);
        out.mcf = mcf;
        out.range_level = state.max_rho();
        debug::info_fmt(
            "SAT+MCF solved with bbox max_rho={} (attempts={})",
            state.max_rho(),
            out.attempts);
        return out;
    }

    out.ok = false;
    out.message = std::format("SAT+MCF: bbox retry attempts exhausted (attempts={})", max_attempts);
    out.range_level = state.max_rho();
    debug::error_fmt("{}", out.message);
    return out;
}

} // namespace PR_tool
