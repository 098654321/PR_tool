#include "route_validate.hh"

#include "maze_search.hh"
#include "resource_model.hh"
#include "route_log.hh"
#include "sync_equalize.hh"

#include <debug/debug.hh>
#include <format>
#include <map>

namespace PR_tool {

namespace {

auto fail(const std::String& message) -> bool {
    debug::error_fmt("FPIA RRR: validate failed: {}", message);
    return false;
}

auto node_in_graph(const UnifiedGraph& graph, int node_id) -> bool {
    return node_id >= 0 && node_id < static_cast<int>(graph.nodes.size());
}

auto demand_source_set(const UnifiedGraph& graph, const RoutingNet& net, const RoutingDemand& demand)
    -> std::Set<int> {
    auto sources = std::Set<int> {};
    for (const std::size_t index : demand.candidate_source_indices) {
        if (index >= net.sources.size()) {
            continue;
        }
        const int node = resolve_graph_node(graph, net.sources[index]);
        if (node >= 0) {
            sources.insert(node);
        }
    }
    return sources;
}

auto owner_of(const RoutingNet& net, const RoutingDemand& demand) -> OwnerId {
    if (net.is_sync_bus) {
        return OwnerId {net.net_id, demand.demand_id};
    }
    return OwnerId {net.net_id, 0};
}

auto path_contains(const std::Vector<int>& path, int node) -> bool {
    for (const int item : path) {
        if (item == node) {
            return true;
        }
    }
    return false;
}

auto contains_key(const std::Vector<ResourceKey>& keys, const ResourceKey& key) -> bool {
    for (const auto& item : keys) {
        if (item == key) {
            return true;
        }
    }
    return false;
}

} // namespace

auto collect_illegal_tob_fanout(
    const UnifiedGraph& graph,
    const std::Vector<std::Vector<int>>& net_paths
) -> std::Vector<TobMuxFanoutHit> {
    auto bump_hlines = std::map<int, std::Set<int>> {};
    auto hline_bumps = std::map<int, std::Set<int>> {};
    auto hline_vlines = std::map<int, std::Set<int>> {};
    auto vline_hlines = std::map<int, std::Set<int>> {};
    auto vline_tracks = std::map<int, std::Set<int>> {};
    auto track_vlines = std::map<int, std::Set<int>> {};

    auto consider = [&](int u, int v) {
        if (u < 0 || v < 0 || u >= static_cast<int>(graph.nodes.size())
            || v >= static_cast<int>(graph.nodes.size())) {
            return;
        }
        const auto ku = graph.nodes[static_cast<std::size_t>(u)].kind;
        const auto kv = graph.nodes[static_cast<std::size_t>(v)].kind;
        if (ku == UnifiedNodeKind::Bump && kv == UnifiedNodeKind::HLine) {
            bump_hlines[u].insert(v);
            hline_bumps[v].insert(u);
        } else if (ku == UnifiedNodeKind::HLine && kv == UnifiedNodeKind::Bump) {
            bump_hlines[v].insert(u);
            hline_bumps[u].insert(v);
        } else if (ku == UnifiedNodeKind::HLine && kv == UnifiedNodeKind::VLine) {
            hline_vlines[u].insert(v);
            vline_hlines[v].insert(u);
        } else if (ku == UnifiedNodeKind::VLine && kv == UnifiedNodeKind::HLine) {
            hline_vlines[v].insert(u);
            vline_hlines[u].insert(v);
        } else if (ku == UnifiedNodeKind::VLine && kv == UnifiedNodeKind::Track) {
            vline_tracks[u].insert(v);
            track_vlines[v].insert(u);
        } else if (ku == UnifiedNodeKind::Track && kv == UnifiedNodeKind::VLine) {
            vline_tracks[v].insert(u);
            track_vlines[u].insert(v);
        }
    };

    for (const auto& path : net_paths) {
        for (std::size_t i = 0; i + 1 < path.size(); ++i) {
            consider(path[i], path[i + 1]);
        }
    }

    auto hits = std::Vector<TobMuxFanoutHit> {};
    auto emit = [&](
        const std::map<int, std::Set<int>>& table,
        UnifiedNodeKind kind,
        const char* side
    ) {
        for (const auto& [node_id, peers] : table) {
            if (peers.size() <= 1) {
                continue;
            }
            TobMuxFanoutHit hit {};
            hit.node_id = node_id;
            hit.kind = kind;
            hit.node = format_path_node(graph, node_id);
            hit.side = side;
            hit.peer_count = static_cast<int>(peers.size());
            for (const int peer : peers) {
                hit.peers.push_back(peer);
            }
            hits.push_back(std::move(hit));
        }
    };
    emit(bump_hlines, UnifiedNodeKind::Bump, "HLine");
    emit(hline_bumps, UnifiedNodeKind::HLine, "Bump");
    emit(hline_vlines, UnifiedNodeKind::HLine, "VLine");
    emit(vline_hlines, UnifiedNodeKind::VLine, "HLine");
    emit(vline_tracks, UnifiedNodeKind::VLine, "Track");
    emit(track_vlines, UnifiedNodeKind::Track, "VLine");
    return hits;
}

auto validate_rrr_solution(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const RrrResult& result,
    hardware::Interposer* interposer
) -> bool {
    if (result.paths.size() != nets.size()) {
        return fail("path net count mismatch");
    }

    auto resources = ResourceModel {};

    for (std::size_t net_i = 0; net_i < nets.size(); ++net_i) {
        const auto& net = nets[net_i];
        const auto& net_paths = result.paths[net_i];
        if (net_paths.size() != net.demands.size()) {
            return fail("demand path count mismatch");
        }
        for (const auto& path : net_paths) {
            if (path.empty()) {
                return fail("empty demand path");
            }
        }

        auto used = std::Vector<char>(net_paths.size(), 0);
        auto matched = std::Vector<const std::Vector<int>*> {};
        matched.resize(net.demands.size(), nullptr);

        for (std::size_t d = 0; d < net.demands.size(); ++d) {
            const int sink = resolve_graph_node(graph, net.demands[d].sink);
            if (sink < 0) {
                return fail("sink does not resolve");
            }
            int found = -1;
            for (std::size_t p = 0; p < net_paths.size(); ++p) {
                if (used[p] != 0) {
                    continue;
                }
                if (net_paths[p].back() == sink) {
                    found = static_cast<int>(p);
                    break;
                }
            }
            if (found < 0) {
                return fail("demand missing path ending at sink");
            }
            used[static_cast<std::size_t>(found)] = 1;
            matched[d] = &net_paths[static_cast<std::size_t>(found)];
        }

        const bool is_bnet = net.kind == RoutingNetKind::Bnet;
        for (std::size_t d = 0; d < net.demands.size(); ++d) {
            const auto& path = *matched[d];
            for (const int node : path) {
                if (!node_in_graph(graph, node)) {
                    return fail("path node out of range");
                }
            }
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                if (!graph.directed_arc_set.contains({path[i], path[i + 1]})) {
                    return fail("consecutive nodes are not a graph arc");
                }
            }
            const auto sources = demand_source_set(graph, net, net.demands[d]);
            const int first = path.front();
            bool legal_start = sources.contains(first);
            if (!legal_start && !net.is_sync_bus) {
                const bool first_is_track = node_in_graph(graph, first)
                    && graph.nodes[static_cast<std::size_t>(first)].kind == UnifiedNodeKind::Track;
                if (first_is_track) {
                    for (std::size_t other = 0; other < matched.size(); ++other) {
                        if (other == d) {
                            continue;
                        }
                        if (path_contains(*matched[other], first)) {
                            legal_start = true;
                            break;
                        }
                    }
                }
            }
            if (!legal_start) {
                return fail("path does not start at a legal source or tree Track node");
            }
        }

        const auto fanout_hits = collect_illegal_tob_fanout(graph, net_paths);
        if (!fanout_hits.empty()) {
            const auto& hit = fanout_hits.front();
            return fail(std::format(
                "TOB mux fanout on {} {} peers={} net={}",
                hit.node,
                hit.side,
                hit.peer_count,
                net.name));
        }

        auto lane_keys = std::Vector<std::Vector<ResourceKey>> {};
        auto lengths = std::Vector<std::size_t> {};
        for (std::size_t d = 0; d < net.demands.size(); ++d) {
            const auto owner = owner_of(net, net.demands[d]);
            const auto keys = path_resource_keys(graph, *matched[d], is_bnet);
            resources.claim(owner, keys);
            if (net.is_sync_bus) {
                lane_keys.push_back(keys);
                if (interposer != nullptr) {
                    lengths.push_back(sync_lane_length(graph, *matched[d], interposer, is_bnet));
                }
            }
        }

        if (net.is_sync_bus) {
            for (std::size_t a = 0; a < lane_keys.size(); ++a) {
                for (std::size_t b = a + 1; b < lane_keys.size(); ++b) {
                    for (const auto& key : lane_keys[a]) {
                        if (!is_physical_occupancy_key(key)) {
                            continue;
                        }
                        if (contains_key(lane_keys[b], key)) {
                            return fail("SyncNet lanes share claimed keys");
                        }
                    }
                }
            }
            if (interposer != nullptr && lengths.size() > 1) {
                for (std::size_t i = 1; i < lengths.size(); ++i) {
                    if (lengths[i] != lengths[0]) {
                        return fail("SyncNet members have unequal Track-node length");
                    }
                }
            }
        }
    }

    if (resources.overflow() != 0) {
        return fail("rebuilt occupancy overflow is not 0 (includes TOB mux port conflicts)");
    }
    return true;
}

} // namespace PR_tool
