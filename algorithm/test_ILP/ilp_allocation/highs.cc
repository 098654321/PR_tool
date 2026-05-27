#include "ilp_allocation/highs.hh"
#include "lp_data/HConst.h"
#include "ilp_allocation/tob_ilp_model.hh"

#include "highs/Highs.h"

#include <algorithm>
#include <format>
#include <limits>
#include <map>
#include <set>
#include <string_view>
#include <thread>
#include <vector>
#include <debug/debug.hh>


namespace PR_tool {

auto solve_tob_ilp_with_highs(
    const std::Vector<Net_cost_record>& records,
    const bool enable_parallel,
    const TobIlpWarmStart* warm_start
)
    -> TobIlpResult {
    
    // build model
    TobIlpResult out {};
    TobIlpModel model {};
    build_tob_ilp_model(model, records);

    // convert to HiGHS LP
    HighsLp lp {};
    std::map<std::String, HighsInt> col_index {};
    model.to_highs_lp(lp, &col_index);

    // set HiGHS options
    Highs highs {};
    highs.setOptionValue("output_flag", false);
    const unsigned int hw_threads = std::thread::hardware_concurrency();
    const HighsInt threads = enable_parallel
                                 ? static_cast<HighsInt>(hw_threads > 1U ? hw_threads : 1U)
                                 : static_cast<HighsInt>(1);
    const HighsStatus parallel_st = enable_parallel ? highs.setOptionValue("parallel", "on")
                                                    : highs.setOptionValue("parallel", "off");
    if (parallel_st != HighsStatus::kOk && enable_parallel) {
        (void)highs.setOptionValue("parallel", "choose");
    }
    const HighsStatus thread_st = highs.setOptionValue("threads", threads);
    if (thread_st != HighsStatus::kOk) {
        (void)highs.setOptionValue("threads", static_cast<HighsInt>(1));
    }
    const HighsStatus pass_st = highs.passModel(std::move(lp));     // 装填模型
    if (pass_st != HighsStatus::kOk) {
        out.ok = false;
        out.message = std::format("HiGHS passModel failed (status={})", static_cast<int>(pass_st));
        return out;
    }
    // set warm start
    if (warm_start != nullptr && !warm_start->values.empty()) {
        auto warm_cols = std::vector<HighsInt> {};
        auto warm_values = std::vector<double> {};
        warm_cols.reserve(warm_start->values.size());
        warm_values.reserve(warm_start->values.size());
        for (const auto& [name, value] : warm_start->values) {
            const auto it = col_index.find(name);
            if (it == col_index.end()) {
                continue;
            }
            warm_cols.push_back(it->second);
            warm_values.push_back(value);
        }
        if (!warm_cols.empty()) {
            (void)highs.setOptionValue("mip_max_start_nodes", static_cast<HighsInt>(0));
            const auto start_st = highs.setSolution(
                static_cast<HighsInt>(warm_cols.size()),
                warm_cols.data(),
                warm_values.data()
            );
            // HiGHS是否接受这个初始解(例如解是否满足当前模型的约束)
            if (start_st != HighsStatus::kOk) {
                debug::warning_fmt(
                    "HiGHS ILP warm start rejected (status={}, requested_values={}, matched_values={})",
                    static_cast<int>(start_st),
                    warm_start->values.size(),
                    warm_cols.size());
            }
            else {
                debug::info_fmt(
                    "HiGHS ILP warm start accepted: requested_values={}, matched_values={}, routed_nets={}, failed_nets={}",
                    warm_start->values.size(),
                    warm_cols.size(),
                    warm_start->routed_nets,
                    warm_start->failed_nets);
            }
        }
        else {
            debug::warning_fmt("HiGHS ILP warm start had no matching variables (requested_values={})", warm_start->values.size());
        }
    }
    // show actual parallel setting accepted by HiGHS
    HighsInt configured_threads = 1;
    const HighsStatus get_threads_st = highs.getOptionValue("threads", configured_threads);
    if (get_threads_st != HighsStatus::kOk || configured_threads < 1) {
        configured_threads = 1;
    }
    std::string parallel_mode = "unknown";
    const HighsStatus get_parallel_st = highs.getOptionValue("parallel", parallel_mode);
    if (get_parallel_st != HighsStatus::kOk) {
        parallel_mode = "unknown";
    }
    debug::info_fmt(
        "HiGHS parallel setup: hw_threads={}, requested_threads={}, configured_threads={}, parallel={}",
        hw_threads,
        threads,
        configured_threads,
        parallel_mode
    );

    const HighsStatus run_st = highs.run();    // 运行HiGHS
    if (run_st != HighsStatus::kOk) {
        out.ok = false;
        out.message = std::format("HiGHS run failed (status={})", static_cast<int>(run_st));
        out.model_status = highs.getModelStatus();
        return out;
    }

    out.model_status = highs.getModelStatus();    
    out.objective = highs.getObjectiveValue();    
    const auto& sol = highs.getSolution();
    if (sol.col_value.empty()) {
        out.ok = false;
        out.message = "HiGHS returned empty solution";
        return out;
    }
    if(out.model_status != HighsModelStatus::kOptimal) {
        debug::info_fmt("HiGHS model status is not optimal (status={})", static_cast<int>(out.model_status));
    }

    // parse solution
    constexpr double z_tol = 0.5;
    const auto is_active = [&](const std::String& var_name) -> bool {
        const auto it = col_index.find(var_name);
        if (it == col_index.end() || it->second < 0) {
            return false;
        }
        const auto idx = static_cast<std::size_t>(it->second);
        if (idx >= sol.col_value.size()) {
            out.ok = false;
            out.message = std::format("column index out of range for variable '{}'", var_name);
            return false;
        }
        return sol.col_value[idx] > z_tol;
    };
    const auto track_from_jk = [](const std::size_t bank, const std::size_t j, const std::size_t k, const bool straight) -> std::size_t {
        const auto v = j * 8 + k;
        std::size_t track = 0;
        if (bank == 0) {
            track = straight ? v : (v + 64);
        }
        else {
            track = straight ? (v + 64) : v;
        }
        return track;
    };
    const auto cob_from_jk = [&](const std::size_t bank, const std::size_t j, const std::size_t k, const bool straight) -> std::size_t {
        return map_track(track_from_jk(bank, j, k, straight));
    };
    const auto relation_bumps_for = [](const Net_cost_record& record) {
        auto relation_bumps = std::Vector<Bump_coord> {};
        if (record.type == Net_type::Bnet) {
            relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
            relation_bumps.insert(relation_bumps.end(), record.end_bumps.begin(), record.end_bumps.end());
        }
        else {
            relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
        }
        std::sort(relation_bumps.begin(), relation_bumps.end());
        relation_bumps.erase(std::unique(relation_bumps.begin(), relation_bumps.end()), relation_bumps.end());
        return relation_bumps;
    };

    const auto pn_bumps_text = [](const Net_cost_record& r) -> std::String {
        if (r.start_bumps.empty()) {
            return std::String("(none)");
        }
        auto s = std::String {};
        for (std::size_t i = 0; i < r.start_bumps.size(); ++i) {
            if (i != 0) {
                s += "; ";
            }
            const auto& b = r.start_bumps[i];
            s += std::format("T{},B{},G{},I{}", b.TOB, b.Bank, b.Group, b.Index);
        }
        return s;
    };

    auto pn_selected_end_track = std::Vector<std::size_t> {};
    pn_selected_end_track.resize(records.size(), std::numeric_limits<std::size_t>::max());
    for (std::size_t n = 0; n < records.size(); ++n) {
        if (records[n].type != Net_type::PNnet) {
            continue;
        }
        int selected = 0;
        std::size_t selected_track = std::numeric_limits<std::size_t>::max();
        for (const auto r_end : records[n].end_tracks) {
            if (!is_active(y_var(n, r_end))) {
                continue;
            }
            selected += 1;
            selected_track = r_end;
        }
        if (selected != 1) {
            out.ok = false;
            const auto& rpn = records[n];
            const auto logical_name = rpn.origin_key.empty() ? std::String("(empty origin_key; use net_name)") : rpn.origin_key;
            auto power_lab = std::string_view {"None"};
            if (rpn.power_kind == IlpPowerKind::Pose) {
                power_lab = "Pose";
            }
            else if (rpn.power_kind == IlpPowerKind::Nege) {
                power_lab = "Nege";
            }
            out.message = std::format(
                "expected exactly one active Y for PNnet: records_index={} record_id={} bit_id={} "
                "2pin_record=\"{}\" logical_net(origin_key)=\"{}\" power_kind={} bump(s)=[{}], got {}",
                n,
                rpn.record_id,
                rpn.bit_id,
                rpn.net_name,
                logical_name,
                power_lab,
                pn_bumps_text(rpn),
                selected);
            return out;
        }
        pn_selected_end_track[n] = selected_track;
    }

    auto all_related_bumps = std::set<Bump_coord> {};
    for (const auto& record : records) {
        const auto relation_bumps = relation_bumps_for(record);
        all_related_bumps.insert(relation_bumps.begin(), relation_bumps.end());
    }

    auto chosen_track_by_bump = std::map<Bump_coord, std::size_t> {};
    for (const auto& bump : all_related_bumps) {
        for (std::size_t j = 0; j < 8; ++j) {
            for (std::size_t k = 0; k < 8; ++k) {
                if (!is_active(w_var(bump, j, k))) {
                    continue;
                }
                const bool qs_active = is_active(qs_var(bump, j, k));
                const bool qw_active = is_active(qw_var(bump, j, k));
                const bool has_track = (qs_active != qw_active);
                const bool use_straight = qs_active && !qw_active;
                const std::size_t track = has_track ? track_from_jk(bump.Bank, j, k, use_straight)
                                                    : std::numeric_limits<std::size_t>::max();
                if (!has_track) {
                    out.ok = false;
                    out.message = std::format(
                        "active W without resolved track for bump(T{},B{},G{},I{})",
                        bump.TOB,
                        bump.Bank,
                        bump.Group,
                        bump.Index);
                    return out;
                }
                if (const auto it = chosen_track_by_bump.find(bump); it == chosen_track_by_bump.end()) {
                    chosen_track_by_bump.emplace(bump, track);
                }
                else if (it->second != track) {
                    out.ok = false;
                    out.message = std::format(
                        "bump(T{},B{},G{},I{}) selects multiple tracks ({}, {})",
                        bump.TOB,
                        bump.Bank,
                        bump.Group,
                        bump.Index,
                        it->second,
                        track);
                    return out;
                }
                out.active_w.push_back(TobIlpWAssignment {bump, j, k, track, has_track, use_straight});
            }
        }
    }
    for (const auto& bump : all_related_bumps) {
        if (!chosen_track_by_bump.contains(bump)) {
            out.ok = false;
            out.message = std::format(
                "no selected track for bump(T{},B{},G{},I{})",
                bump.TOB,
                bump.Bank,
                bump.Group,
                bump.Index);
            return out;
        }
    }

    auto s_by_tv_from_w = std::set<std::pair<std::size_t, std::size_t>> {};
    out.active_s.clear();
    out.active_s.reserve(out.active_w.size());
    for (const auto& w : out.active_w) {
        const auto v = w.j * 8 + w.k;
        const auto key = std::pair<std::size_t, std::size_t> {w.bump.TOB, v};
        if (!s_by_tv_from_w.insert(key).second) {
            continue;
        }
        out.active_s.push_back(TobIlpSAssignment {w.bump.TOB, v, w.j, w.k});
    }

    auto orphan_s_by_tv = std::set<std::pair<std::size_t, std::size_t>> {};
    for (const auto& bump : all_related_bumps) {
        for (std::size_t v = 0; v < 64; ++v) {
            if (!is_active(s_var(bump.TOB, v))) {
                continue;
            }
            const auto key = std::pair<std::size_t, std::size_t> {bump.TOB, v};
            if (s_by_tv_from_w.contains(key)) {
                continue;
            }
            orphan_s_by_tv.insert(key);
        }
    }
    if (!orphan_s_by_tv.empty()) {
        debug::info_fmt("ILP parse: orphan S variables ignored={}", orphan_s_by_tv.size());
    }

    out.assignments.reserve(records.size());
    out.record_track_endpoints.clear();
    out.record_track_endpoints.reserve(records.size());
    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto relation_bumps = relation_bumps_for(records[n]);
        if (relation_bumps.empty()) {
            out.ok = false;
            out.message = std::format("net '{}' has no relation bump", records[n].net_name);
            return out;
        }
        std::size_t derived_cob = std::numeric_limits<std::size_t>::max();
        for (const auto& bump : relation_bumps) {
            const auto tr_it = chosen_track_by_bump.find(bump);
            if (tr_it == chosen_track_by_bump.end()) {
                out.ok = false;
                out.message = std::format(
                    "missing selected track for bump(T{},B{},G{},I{}) in net '{}'",
                    bump.TOB,
                    bump.Bank,
                    bump.Group,
                    bump.Index,
                    records[n].net_name);
                return out;
            }
            const auto cob = map_track(tr_it->second);
            if (derived_cob == std::numeric_limits<std::size_t>::max()) {
                derived_cob = cob;
            }
            else if (derived_cob != cob) {
                out.ok = false;
                out.message = std::format(
                    "track-derived cobunit mismatch in net '{}': {} vs {}",
                    records[n].net_name,
                    derived_cob,
                    cob);
                return out;
            }
        }
        out.assignments.push_back(TobIlpNetAssignment {records[n].net_name, derived_cob});

        TobIlpRecordTrackEndpoint endpoint {};
        endpoint.record_id = records[n].record_id;
        endpoint.cob_unit = derived_cob;
        if (!records[n].start_bumps.empty()) {
            const auto it = chosen_track_by_bump.find(records[n].start_bumps.front());
            if (it != chosen_track_by_bump.end()) {
                endpoint.has_start_track = true;
                endpoint.start_track = it->second;
            }
        }

        if (records[n].type == Net_type::Bnet) {
            if (!records[n].end_bumps.empty()) {
                const auto it = chosen_track_by_bump.find(records[n].end_bumps.front());
                if (it != chosen_track_by_bump.end()) {
                    endpoint.has_end_track = true;
                    endpoint.end_track = it->second;
                }
            }
        }
        else if (records[n].type == Net_type::Tnet) {
            if (records[n].mcf_has_end_track) {
                endpoint.has_end_track = true;
                endpoint.end_track = records[n].mcf_end_track.index;
            }
        }
        else if (records[n].type == Net_type::PNnet) {
            endpoint.has_end_track = true;
            endpoint.end_track = pn_selected_end_track[n];
        }
        out.record_track_endpoints.push_back(endpoint);
    }

    out.route_details.reserve(records.size() * 4);
    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto relation_bumps = relation_bumps_for(records[n]);
        for (const auto& bump : relation_bumps) {
            for (std::size_t j = 0; j < 8; ++j) {
                for (std::size_t k = 0; k < 8; ++k) {
                    if (!is_active(w_var(bump, j, k))) {
                        continue;
                    }
                    const bool qs_active = is_active(qs_var(bump, j, k));
                    const bool qw_active = is_active(qw_var(bump, j, k));
                    if (qs_active == qw_active) {
                        continue;
                    }
                    const bool use_straight = qs_active;
                    const std::size_t s_v = j * 8 + k;
                    out.route_details.push_back(TobIlpNetRouteDetail {
                        records[n].net_name,
                        bump,
                        j,
                        k,
                        s_v,
                        track_from_jk(bump.Bank, j, k, use_straight),
                        cob_from_jk(bump.Bank, j, k, use_straight),
                        use_straight
                    });
                }
            }
        }
    }

    out.ok = true;
    out.message = "ok";
    return out;
}

} // namespace PR_tool
