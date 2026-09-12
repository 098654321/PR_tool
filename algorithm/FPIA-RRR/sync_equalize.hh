#pragma once

#include "hardware_graph.hh"
#include "resource_model.hh"
#include "rrr_types.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>

namespace PR_tool {

// Number of Track nodes kept from the source: floor(N_track * (1-r)).
// Prefix is tracks [0, N_r) in visit order. Tail maze starts at tracks[N_r-1]
// when N_r > 0; N_r == 0 (r = 1.0) remakes the lane from its original sources.
auto sync_track_cut_index(std::size_t Ni, double r) -> std::size_t;

inline auto sync_tob_length_constant(bool is_bnet) -> std::size_t {
    return is_bnet ? 2 : 1;
}

auto sync_lane_length(
    const UnifiedGraph& graph,
    const std::Vector<int>& node_path,
    hardware::Interposer* interposer,
    bool is_bnet
) -> std::size_t;

struct SyncLaneState {
    OwnerId id {};
    bool is_bnet{false};
    std::Vector<int> sources;
    int sink{-1};
    std::Vector<int> path;
};

// Mutates lane paths and ResourceModel claims. Returns true only if every
// lane has the same N_i (Track-node count + TOB constant). Other SyncNet lanes are
// hard-blocked; ordinary nets keep soft C(a|owner).
auto equalize_sync_group(
    const UnifiedGraph& graph,
    ResourceModel& resources,
    const RrrParams& params,
    hardware::Interposer* interposer,
    std::Vector<SyncLaneState>& lanes
) -> bool;

} // namespace PR_tool
