#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "mcf/cob_mcf_router.hh"
#include "mcf/mcf_graph.hh"

#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <circuit/path/pathpackage.hh>
#include <hardware/interposer.hh>
#include <std/collection.hh>

namespace PR_tool::circuit {
class Net;
}

namespace PR_tool {

struct MazeIlpFixedContext {
    const McfGlobalGraph* graph{nullptr};
    const CobMcfFullResult* mcf_result{nullptr};
    /// When true, gather PNnet seed tracks from session paths without requiring simple_mcf_ok.
    bool use_session_paths{false};
    const char* log_prefix{nullptr};
    bool verbose_records{false};
    std::size_t* last_failed_record_index{nullptr};
};

struct OriginRouteSegments {
    std::map<std::size_t, algo::routed_path> by_record_index;
};

auto route_origin_net_ilp_fixed(
    hardware::Interposer* interposer,
    circuit::Net* net,
    const std::Vector<std::size_t>& record_indices,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const MazeIlpFixedContext* mcf_ctx = nullptr,
    OriginRouteSegments* segments_out = nullptr
) -> circuit::PathPackage;

auto pathpackage_regular_path_text(const circuit::PathPackage& package) -> std::String;

auto segments_path_text(const OriginRouteSegments& segments) -> std::String;

} // namespace PR_tool
