#pragma once

#include "mcf/cob_mcf_router.hh"

#include <hardware/cob/cob.hh>
#include <hardware/interposer.hh>
#include <std/collection.hh>
#include <array>
#include <set>
#include <tuple>

namespace PR_tool {

struct McfNodeMeta {
    bool is_virtual{false};
    int virtual_kind{0};
    std::size_t unit{0};
    int track_dir{0};
    int track_row{0};
    int track_col{0};
    std::size_t track{0};
};

struct McfArc {
    int u{0};
    int v{0};
    bool is_virtual{false};
    bool is_turn{false};
    std::size_t unit{0};
    int cob{-1};
    std::size_t track_in{0};
    std::size_t track_out{0};
    hardware::COBDirection from_dir{hardware::COBDirection::Left};
    hardware::COBDirection to_dir{hardware::COBDirection::Left};
};

using McfNodeKey = std::tuple<std::size_t, int, int, int, std::size_t>;

struct McfGlobalGraph {
    int rows{0};
    int cols{0};
    int num_cob{0};
    std::array<int, 16> vp_node_by_unit {};
    std::array<int, 16> vn_node_by_unit {};
    std::Vector<McfNodeMeta> nodes;
    std::Vector<McfArc> arcs;
    std::map<McfNodeKey, int> node_id_by_key;
    std::set<std::pair<int, int>> directed_arc_set;
};

auto build_mcf_track_graph(CobMcfGridDims grid) -> McfGlobalGraph;

auto suspend_mcf_paths_on_interposer(
    hardware::Interposer* interposer,
    const McfGlobalGraph& graph,
    const std::array<std::Vector<McfPathInfo>, 16>& paths_by_unit
) -> void;

auto is_sync_bus_mcf_origin_key(const std::String& origin_key) -> bool;

} // namespace PR_tool
