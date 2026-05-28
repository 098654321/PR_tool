#pragma once

#include <cstddef>
#include <map>
#include <std/collection.hh>
#include <std/string.hh>

#include <hardware/track/trackcoord.hh>

namespace PR_tool {

enum class IlpPowerKind {
    None,
    Pose,
    Nege
};

enum class IlpEndpointKind {
    Bump,
    Track
};

struct Bump_coord {
    std::size_t TOB;
    std::size_t Bank;
    std::size_t Group;
    std::size_t Index;

    auto operator==(const Bump_coord& other) const -> bool {
        return TOB == other.TOB && Bank == other.Bank && Group == other.Group && Index == other.Index;
    }

    auto operator<(const Bump_coord& other) const -> bool {
        return std::tie(TOB, Bank, Group, Index) < std::tie(other.TOB, other.Bank, other.Group, other.Index);
    }
};

enum class Net_type {
    Bnet,
    Tnet,
    PNnet
};

struct IlpReachStep {
    char from_dir{'L'};
    char to_dir{'U'};
    std::size_t index_in{0};
    std::size_t index_out{0};
};

struct Net_cost_record {
    std::String net_name;
    Net_type type;
    float bits;
    float lambda;
    std::Vector<Bump_coord> start_bumps;
    std::Vector<Bump_coord> end_bumps;
    std::Vector<std::size_t> candidate_cobunits;
    std::Vector<std::size_t> tnet_fixed_cobunits;

    /// Key to merge 2-pin fragments back to a logical net.
    std::String origin_key {};
    /// Stable uid for matching records to logical nets.
    std::String origin_uid {};
    /// Bit index in one logical/origin net.
    std::size_t bit_id{0};
    /// Stable global id in `build_records` output order.
    std::size_t record_id{0};
    /// True when this Tnet was split from TrackToBumpsNet (SimpleMCF origin aggregation).
    bool from_track_to_bumps_split{false};
    IlpPowerKind power_kind{IlpPowerKind::None};
    IlpEndpointKind mcf_start_kind{IlpEndpointKind::Bump};
    IlpEndpointKind mcf_end_kind{IlpEndpointKind::Bump};
    hardware::TrackCoord mcf_start_track {};
    hardware::TrackCoord mcf_end_track {};
    bool mcf_has_start_track{false};
    bool mcf_has_end_track{false};

    /// Optional endpoint tracks for PN nets (from TracksToBumps begin tracks).
    std::Vector<std::size_t> pn_end_tracks {};
    std::map<std::size_t, hardware::TrackCoord> pn_end_track_coord_by_index {};

    /// First-mod precompute payload.
    std::Vector<std::size_t> end_tracks {};
    std::map<std::size_t, std::Vector<std::size_t>> starttrack_by_endtrack {};
    std::map<std::size_t, std::map<std::size_t, std::Vector<IlpReachStep>>> reach_by_end_start {};
};

struct TobIlpWarmStart {
    std::map<std::String, double> values;
    std::size_t routed_nets{0};
    std::size_t failed_nets{0};
};

inline auto map_track(std::size_t track) -> std::size_t {
    return track < 64 ? track % 8 : track % 8 + 8;
}

} // namespace PR_tool
