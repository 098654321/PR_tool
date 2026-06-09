#pragma once

#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "mcf/cob_mcf_router.hh"
#include "mcf/mcf_graph.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>
#include <array>

namespace PR_tool::circuit {
class BaseDie;
}

namespace PR_tool {

struct SimpleMazeCommodityInput {
    std::String label;
    std::String origin_name;
    std::String origin_uid;
    std::size_t record_index{0};
    std::size_t record_id{0};
    std::Vector<std::size_t> record_indices;
    std::size_t cob_unit{0};
    std::size_t start_track{0};
    std::size_t end_track{0};
    int src{-1};
    int snk{-1};
    int demand{1};
};

struct SimpleMazeSolveResult {
    bool all_ok{true};
    int solve_ms{0};
    std::array<std::Vector<McfPathInfo>, 16> paths_by_unit {};
    std::array<bool, 16> simple_mcf_ok {};
    std::array<bool, 16> has_simple_commodities {};
};

auto solve_simple_mcf_with_maze(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie,
    const McfGlobalGraph& graph,
    const std::Vector<Net_cost_record>& records,
    const TobIlpResult& ilp_result,
    const std::Vector<SimpleMazeCommodityInput>& commodities,
    const std::array<std::Vector<McfPathInfo>, 16>& bus_paths_by_unit,
    bool verbose_maze_records = false
) -> SimpleMazeSolveResult;

} // namespace PR_tool
