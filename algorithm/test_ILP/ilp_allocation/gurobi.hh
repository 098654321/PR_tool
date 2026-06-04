#pragma once

#include "common/ilp_types.hh"
#include "ilp_allocation/gurobi_model_stats.hh"

#include <std/collection.hh>
#include <std/string.hh>

namespace PR_tool {

struct TobIlpNetAssignment {
    std::String net_name;
    std::size_t cob_unit;
};

struct TobIlpWAssignment {
    Bump_coord bump;
    std::size_t j;
    std::size_t k;
    std::size_t track;
    bool has_track;
    bool use_straight;
};

struct TobIlpSAssignment {
    std::size_t tob;
    std::size_t v;
    std::size_t j;
    std::size_t k;
};

struct TobIlpNetRouteDetail {
    std::String net_name;
    Bump_coord bump;
    std::size_t j;
    std::size_t k;
    std::size_t s_v;
    std::size_t track;
    std::size_t cob_unit;
    bool use_straight;
};

struct TobIlpRecordTrackEndpoint {
    std::size_t record_id{0};
    std::size_t cob_unit{0};
    bool has_start_track{false};
    std::size_t start_track{0};
    bool has_end_track{false};
    std::size_t end_track{0};
};

struct TobIlpConstraintMeta {
    std::String kind;
    std::String detail;
    std::Vector<std::size_t> related_record_ids;
    std::Vector<std::String> related_origin_keys;
};

struct TobIlpResult {
    bool ok{false};
    std::String message;
    double objective{0.0};
    int model_status{0};
    std::Vector<TobIlpNetAssignment> assignments;
    std::Vector<TobIlpWAssignment> active_w;
    std::Vector<TobIlpSAssignment> active_s;
    std::Vector<TobIlpNetRouteDetail> route_details;
    std::Vector<TobIlpRecordTrackEndpoint> record_track_endpoints;
    std::Vector<TobIlpConstraintMeta> infeasibility_hints;
};

auto solve_tob_ilp_with_gurobi(
    const std::Vector<Net_cost_record>& records,
    bool enable_parallel = false,
    const TobIlpWarmStart* warm_start = nullptr,
    const GurobiDiagnosticsOptions& diag = {}
)
    -> TobIlpResult;

} // namespace PR_tool
