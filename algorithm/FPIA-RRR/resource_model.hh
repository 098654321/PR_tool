#pragma once

#include "rrr_types.hh"

#include <std/collection.hh>

namespace PR_tool {

class ResourceModel {
public:
    ResourceModel() = default;
    explicit ResourceModel(RrrParams params);

    auto claim(OwnerId owner, const std::Vector<ResourceKey>& keys) -> void;
    auto release(OwnerId owner) -> void;
    auto release(OwnerId owner, const std::Vector<ResourceKey>& keys) -> void;

    auto overflow() const -> int;
    auto overflow(const ResourceKey& key) const -> int;
    auto mux_distinct_peers(const ResourceKey& key) const -> int;
    auto mux_has_other_peer(OwnerId owner, const ResourceKey& key) const -> bool;
    auto holds(OwnerId owner, const ResourceKey& key) const -> bool;
    auto has_any_owner(const ResourceKey& key) const -> bool;
    auto occupancy_count(const ResourceKey& key) const -> int;
    auto owners_of(const ResourceKey& key) const -> std::Vector<OwnerId>;
    auto history(const ResourceKey& key) const -> double;
    auto history_next() -> void;
    auto type_weight(const ResourceKey& key) const -> int;
    auto selected_unit(OwnerId owner) const -> int;
    auto params() const -> const RrrParams&;

private:
    auto add_ref(OwnerId owner, const ResourceKey& key) -> void;
    auto drop_ref(OwnerId owner, const ResourceKey& key) -> void;
    auto owner_count(const ResourceKey& key) const -> int;
    auto is_capacity_key(const ResourceKey& key) const -> bool;
    auto mux_port_overflow(const ResourceKey& key) const -> int;
    auto distinct_units(OwnerId owner) const -> int;
    auto unit_lock_overflow() const -> int;
    auto collect_mode_owners(int mode_group_id) const -> std::Vector<OwnerId>;
    auto mode_conflicted(int mode_group_id) const -> bool;
    auto refresh_selected_unit(OwnerId owner) -> void;

    RrrParams _params {};
    std::Map<ResourceKey, std::Map<OwnerId, int>> _refs;
    std::Map<OwnerId, std::Map<ResourceKey, int>> _owner_refs;
    std::Map<ResourceKey, std::Map<int, int>> _mux_port_peer_refs;
    std::Map<OwnerId, std::Map<ResourceKey, std::Map<int, int>>> _owner_mux_port_peer_refs;
    std::Map<ResourceKey, double> _history;
    std::Map<OwnerId, int> _selected_unit;
};

} // namespace PR_tool
