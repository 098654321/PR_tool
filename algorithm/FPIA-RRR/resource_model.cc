#include "resource_model.hh"

#include <algorithm>
#include <utility>
#include <set>

namespace PR_tool {

namespace {

auto max0(int value) -> int {
    return value > 0 ? value : 0;
}

auto mux_port_key(const ResourceKey& key) -> ResourceKey {
    return ResourceKey {key.kind, key.id, -1};
}

} // namespace

ResourceModel::ResourceModel(RrrParams params) : _params {std::move(params)} {}

auto ResourceModel::is_capacity_key(const ResourceKey& key) const -> bool {
    return key.kind == ResourceKind::Node
        || key.kind == ResourceKind::PhysicalSwitch
        || key.kind == ResourceKind::MatchingEndpoint;
}

auto ResourceModel::owner_count(const ResourceKey& key) const -> int {
    const auto it = _refs.find(key);
    if (it == _refs.end()) {
        return 0;
    }
    return static_cast<int>(it->second.size());
}

auto ResourceModel::add_ref(OwnerId owner, const ResourceKey& key) -> void {
    if (key.kind == ResourceKind::ModeConflict) {
        return;
    }
    _refs[key][owner] += 1;
    _owner_refs[owner][key] += 1;
    if (is_mux_port_key(key)) {
        const auto port = mux_port_key(key);
        _mux_port_peer_refs[port][key.extra] += 1;
        _owner_mux_port_peer_refs[owner][port][key.extra] += 1;
    }
    if (key.kind == ResourceKind::BnetUnit) {
        const auto selected = _selected_unit.find(owner);
        if (selected == _selected_unit.end()) {
            _selected_unit[owner] = key.id;
        }
    }
}

auto ResourceModel::drop_ref(OwnerId owner, const ResourceKey& key) -> void {
    auto owner_it = _owner_refs.find(owner);
    if (owner_it == _owner_refs.end()) {
        return;
    }
    auto key_it = owner_it->second.find(key);
    if (key_it == owner_it->second.end()) {
        return;
    }
    key_it->second -= 1;
    if (key_it->second <= 0) {
        owner_it->second.erase(key_it);
    }
    if (owner_it->second.empty()) {
        _owner_refs.erase(owner_it);
    }

    auto refs_it = _refs.find(key);
    if (refs_it == _refs.end()) {
        return;
    }
    auto occ_it = refs_it->second.find(owner);
    if (occ_it == refs_it->second.end()) {
        return;
    }
    occ_it->second -= 1;
    if (occ_it->second <= 0) {
        refs_it->second.erase(occ_it);
    }
    if (refs_it->second.empty()) {
        _refs.erase(refs_it);
    }

    if (is_mux_port_key(key)) {
        const auto port = mux_port_key(key);
        auto port_it = _mux_port_peer_refs.find(port);
        if (port_it != _mux_port_peer_refs.end()) {
            auto peer_it = port_it->second.find(key.extra);
            if (peer_it != port_it->second.end()) {
                peer_it->second -= 1;
                if (peer_it->second <= 0) {
                    port_it->second.erase(peer_it);
                }
            }
            if (port_it->second.empty()) {
                _mux_port_peer_refs.erase(port_it);
            }
        }
        auto owner_ports = _owner_mux_port_peer_refs.find(owner);
        if (owner_ports != _owner_mux_port_peer_refs.end()) {
            auto owner_port = owner_ports->second.find(port);
            if (owner_port != owner_ports->second.end()) {
                auto owner_peer = owner_port->second.find(key.extra);
                if (owner_peer != owner_port->second.end() && --owner_peer->second <= 0) {
                    owner_port->second.erase(owner_peer);
                }
                if (owner_port->second.empty()) {
                    owner_ports->second.erase(owner_port);
                }
            }
            if (owner_ports->second.empty()) {
                _owner_mux_port_peer_refs.erase(owner_ports);
            }
        }
    }

    if (key.kind == ResourceKind::BnetUnit) {
        refresh_selected_unit(owner);
    }
}

auto ResourceModel::refresh_selected_unit(OwnerId owner) -> void {
    const auto owner_it = _owner_refs.find(owner);
    if (owner_it == _owner_refs.end()) {
        _selected_unit.erase(owner);
        return;
    }
    const auto selected = _selected_unit.find(owner);
    if (selected != _selected_unit.end()) {
        const auto locked = bnet_unit_key(selected->second);
        const auto locked_it = owner_it->second.find(locked);
        if (locked_it != owner_it->second.end() && locked_it->second > 0) {
            return;
        }
    }
    for (const auto& [key, refs] : owner_it->second) {
        if (key.kind == ResourceKind::BnetUnit && refs > 0) {
            _selected_unit[owner] = key.id;
            return;
        }
    }
    _selected_unit.erase(owner);
}

auto ResourceModel::distinct_units(OwnerId owner) const -> int {
    const auto it = _owner_refs.find(owner);
    if (it == _owner_refs.end()) {
        return 0;
    }
    int count = 0;
    for (const auto& [key, refs] : it->second) {
        if (key.kind == ResourceKind::BnetUnit && refs > 0) {
            ++count;
        }
    }
    return count;
}

auto ResourceModel::unit_lock_overflow() const -> int {
    int total = 0;
    for (const auto& [owner, _] : _owner_refs) {
        total += max0(distinct_units(owner) - 1);
    }
    return total;
}

auto ResourceModel::collect_mode_owners(int mode_group_id) const -> std::Vector<OwnerId> {
    auto owners = std::Vector<OwnerId> {};
    auto seen = std::Set<OwnerId> {};
    const auto take = [&](const ResourceKey& key) {
        const auto it = _refs.find(key);
        if (it == _refs.end()) {
            return;
        }
        for (const auto& [owner, refs] : it->second) {
            if (refs > 0 && seen.insert(owner).second) {
                owners.push_back(owner);
            }
        }
    };
    take(mode_straight_key(mode_group_id));
    take(mode_swap_key(mode_group_id));
    std::sort(owners.begin(), owners.end());
    return owners;
}

auto ResourceModel::mode_conflicted(int mode_group_id) const -> bool {
    return owner_count(mode_straight_key(mode_group_id)) > 0
        && owner_count(mode_swap_key(mode_group_id)) > 0;
}

auto ResourceModel::claim(OwnerId owner, const std::Vector<ResourceKey>& keys) -> void {
    for (const auto& key : keys) {
        add_ref(owner, key);
    }
}

auto ResourceModel::release(OwnerId owner) -> void {
    const auto it = _owner_refs.find(owner);
    if (it == _owner_refs.end()) {
        return;
    }
    auto held = it->second;
    for (const auto& [key, refs] : held) {
        for (int i = 0; i < refs; ++i) {
            drop_ref(owner, key);
        }
    }
}

auto ResourceModel::release(OwnerId owner, const std::Vector<ResourceKey>& keys) -> void {
    for (const auto& key : keys) {
        drop_ref(owner, key);
    }
}

auto ResourceModel::mux_distinct_peers(const ResourceKey& key) const -> int {
    if (!is_mux_port_key(key)) {
        return 0;
    }
    const auto it = _mux_port_peer_refs.find(mux_port_key(key));
    if (it == _mux_port_peer_refs.end()) {
        return 0;
    }
    return static_cast<int>(it->second.size());
}

auto ResourceModel::mux_has_other_peer(OwnerId owner, const ResourceKey& key) const -> bool {
    if (!is_mux_port_key(key)) {
        return false;
    }
    const auto owner_ports = _owner_mux_port_peer_refs.find(owner);
    if (owner_ports == _owner_mux_port_peer_refs.end()) {
        return false;
    }
    const auto peers = owner_ports->second.find(mux_port_key(key));
    if (peers == owner_ports->second.end()) {
        return false;
    }
    return peers->second.size() > (peers->second.contains(key.extra) ? 1U : 0U);
}

auto ResourceModel::holds(OwnerId owner, const ResourceKey& key) const -> bool {
    if (key.kind == ResourceKind::ModeConflict) {
        return false;
    }
    const auto key_it = _refs.find(key);
    return key_it != _refs.end() && key_it->second.contains(owner);
}

auto ResourceModel::has_any_owner(const ResourceKey& key) const -> bool {
    if (key.kind == ResourceKind::ModeConflict) {
        return mode_conflicted(key.id);
    }
    const auto it = _refs.find(key);
    return it != _refs.end() && !it->second.empty();
}

auto ResourceModel::occupancy_count(const ResourceKey& key) const -> int {
    if (key.kind == ResourceKind::ModeConflict) {
        return mode_conflicted(key.id) ? 1 : 0;
    }
    return owner_count(key);
}

auto ResourceModel::mux_port_overflow(const ResourceKey& key) const -> int {
    return max0(mux_distinct_peers(key) - 1);
}

auto ResourceModel::overflow(const ResourceKey& key) const -> int {
    if (key.kind == ResourceKind::ModeConflict) {
        return mode_conflicted(key.id) ? 1 : 0;
    }
    if (is_mux_port_key(key)) {
        return mux_port_overflow(key);
    }
    if (is_capacity_key(key)) {
        return max0(owner_count(key) - 1);
    }
    return 0;
}

auto ResourceModel::overflow() const -> int {
    int total = 0;
    auto mode_groups = std::Set<int> {};
    auto mux_ports = std::Set<std::pair<ResourceKind, int>> {};
    for (const auto& [key, _] : _refs) {
        if (is_mux_port_key(key)) {
            if (mux_ports.insert({key.kind, key.id}).second) {
                total += mux_port_overflow(key);
            }
        } else if (is_capacity_key(key)) {
            total += max0(owner_count(key) - 1);
        } else if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
            mode_groups.insert(key.id);
        }
    }
    for (const int group : mode_groups) {
        total += overflow(mode_conflict_key(group));
    }
    total += unit_lock_overflow();
    return total;
}

auto ResourceModel::owners_of(const ResourceKey& key) const -> std::Vector<OwnerId> {
    if (key.kind == ResourceKind::ModeConflict) {
        if (!mode_conflicted(key.id)) {
            return {};
        }
        return collect_mode_owners(key.id);
    }
    const auto it = _refs.find(key);
    if (it == _refs.end()) {
        return {};
    }
    auto owners = std::Vector<OwnerId> {};
    owners.reserve(it->second.size());
    for (const auto& [owner, refs] : it->second) {
        if (refs > 0) {
            owners.push_back(owner);
        }
    }
    return owners;
}

auto ResourceModel::history(const ResourceKey& key) const -> double {
    const auto stored = is_mux_port_key(key) ? mux_port_key(key) : key;
    const auto it = _history.find(stored);
    if (it == _history.end()) {
        return 0;
    }
    return it->second;
}

auto ResourceModel::history_next() -> void {
    auto keys = std::Set<ResourceKey> {};
    for (const auto& [key, _] : _refs) {
        if (is_mux_port_key(key)) {
            keys.insert(mux_port_key(key));
        } else {
            keys.insert(key);
        }
        if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
            keys.insert(mode_conflict_key(key.id));
        }
    }
    for (const auto& [key, _] : _history) {
        keys.insert(key);
    }
    for (const auto& key : keys) {
        if (key.kind == ResourceKind::ModeStraight || key.kind == ResourceKind::ModeSwap) {
            continue;
        }
        if (is_mux_port_key(key) && key.extra != -1) {
            continue;
        }
        const auto stored = is_mux_port_key(key) ? mux_port_key(key) : key;
        _history[stored] =
            _params.decay * history(stored) + _params.increment * static_cast<double>(overflow(stored));
    }
}

auto ResourceModel::type_weight(const ResourceKey& key) const -> int {
    switch (key.kind) {
    case ResourceKind::Node:
        return 1;
    case ResourceKind::PhysicalSwitch:
    case ResourceKind::MatchingEndpoint:
    case ResourceKind::TobMuxInput:
    case ResourceKind::TobMuxOutput:
        return 2;
    case ResourceKind::ModeConflict:
        return 8;
    default:
        return 0;
    }
}

auto ResourceModel::selected_unit(OwnerId owner) const -> int {
    const auto it = _selected_unit.find(owner);
    if (it == _selected_unit.end()) {
        return -1;
    }
    return it->second;
}

auto ResourceModel::params() const -> const RrrParams& {
    return _params;
}

} // namespace PR_tool
