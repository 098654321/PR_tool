#include "route_ilp/route_search.hh"

#include "common/cob_unit_mask.hh"
#include "common/route_metrics.hh"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>

namespace PR_tool {
namespace {

using ModeSwitches = std::map<int, std::pair<std::set<int>, std::set<int>>>;

auto mode_switches(const UnifiedGraph& graph) -> ModeSwitches {
    auto result = ModeSwitches{};
    for (const auto& arc : graph.arcs) {
        if (arc.mode_group_id < 0 || arc.physical_switch_id < 0) continue;
        auto& group = result[arc.mode_group_id];
        (arc.is_vline_track_straight ? group.first : group.second)
            .insert(arc.physical_switch_id);
    }
    return result;
}

auto arc_between(const UnifiedGraph& graph, int u, int v) -> int {
    for (int id : graph.out_arc_ids.at(static_cast<std::size_t>(u)))
        if (graph.arcs[static_cast<std::size_t>(id)].v == v) return id;
    return -1;
}

auto resources_for_arc(const UnifiedGraph& graph, const UnifiedArc& arc,
                       const ModeSwitches& groups)
    -> std::set<RouteResource> {
    auto result = std::set<RouteResource>{};
    if (arc.physical_switch_id < 0) return result;
    result.insert({1, arc.physical_switch_id, 0});
    const auto kind = [&](int n) { return graph.nodes[static_cast<std::size_t>(n)].kind; };
    if (arc.physical_switch_kind == PhysicalSwitchKind::BumpH) {
        const int bump = kind(arc.u) == UnifiedNodeKind::Bump ? arc.u : arc.v;
        const int hline = bump == arc.u ? arc.v : arc.u;
        result.insert({2, bump, 0});
        result.insert({2, hline, 1});
    } else if (arc.physical_switch_kind == PhysicalSwitchKind::HLineVLine) {
        const int hline = kind(arc.u) == UnifiedNodeKind::HLine ? arc.u : arc.v;
        const int vline = hline == arc.u ? arc.v : arc.u;
        result.insert({2, hline, 2});
        result.insert({2, vline, 3});
    }
    if (arc.mode_group_id >= 0) {
        const auto& group = groups.at(arc.mode_group_id);
        const auto& opposites = arc.is_vline_track_straight
            ? group.second : group.first;
        for (int opposite : opposites) {
            const int straight = arc.is_vline_track_straight
                ? arc.physical_switch_id : opposite;
            const int swap = arc.is_vline_track_straight
                ? opposite : arc.physical_switch_id;
            result.insert({3, straight, swap});
        }
    }
    return result;
}

auto make_column(const UnifiedGraph& graph, const RoutingNet& net,
                 RouteOwner owner, const std::Vector<SourceSinkPairPath>& paths,
                 bool complete, const ModeSwitches& groups) -> RouteColumn {
    auto out = RouteColumn{};
    out.owner = owner;
    out.paths = paths;
    auto demands = std::set<std::size_t>{};
    auto matching = std::map<RouteResource, int>{};
    auto modes = std::map<int, bool>{};
    auto wire_nodes = std::set<int>{};
    auto edges = std::set<std::pair<int, int>>{};
    for (const auto& path : paths) {
        if (path.net_id != net.net_id || (net.is_sync_bus &&
            path.demand_id != owner.demand_id) ||
            !demands.insert(path.demand_id).second)
            throw std::logic_error("candidate demand mismatch");
        const auto dit = std::find_if(net.demands.begin(), net.demands.end(),
            [&](const auto& d) { return d.demand_id == path.demand_id; });
        if (dit == net.demands.end() || path.source_index >= net.sources.size() ||
            std::find(dit->candidate_source_indices.begin(),
                      dit->candidate_source_indices.end(), path.source_index)
                == dit->candidate_source_indices.end() || path.node_path.empty() ||
            path.node_path.front() != resolve_graph_node(graph, net.sources[path.source_index]) ||
            path.node_path.back() != resolve_graph_node(graph, dit->sink) ||
            (net.kind == RoutingNetKind::PNnet
                ? path.physical_source_node != path.node_path.front()
                : path.physical_source_node >= 0))
            throw std::logic_error("candidate endpoint mismatch");
        auto seen = std::set<int>{};
        int path_unit = -1;
        for (std::size_t j = 0; j < path.node_path.size(); ++j) {
            const int node = path.node_path[j];
            if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size() ||
                !seen.insert(node).second) throw std::logic_error("candidate repeated node");
            const auto& data = graph.nodes[static_cast<std::size_t>(node)];
            if (data.kind == UnifiedNodeKind::VirtualSource ||
                (data.kind == UnifiedNodeKind::Bump && j != 0 &&
                 j + 1 != path.node_path.size()))
                throw std::logic_error("candidate traverses a Bump or virtual root");
            if (data.kind == UnifiedNodeKind::Track) {
                if (path_unit >= 0 && path_unit != static_cast<int>(data.unit))
                    throw std::logic_error("candidate crosses Track units");
                path_unit = static_cast<int>(data.unit);
            }
            out.resources.insert({0, node, 0});
            if (is_wirelength_resource_node(graph, node)) wire_nodes.insert(node);
            if (j == 0) continue;
            const int previous = path.node_path[j - 1];
            const int arc_id = arc_between(graph, previous, node);
            if (arc_id < 0) throw std::logic_error("candidate missing arc");
            const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
            if (arc.is_virtual_source_arc) throw std::logic_error("candidate virtual arc");
            edges.insert(std::minmax(previous, node));
            for (const auto& resource : resources_for_arc(graph, arc, groups)) {
                out.resources.insert(resource);
                if (resource.kind == 2) {
                    const auto [it, inserted] = matching.emplace(resource,
                        arc.physical_switch_id);
                    if (!inserted && it->second != arc.physical_switch_id)
                        throw std::logic_error("candidate matching conflict");
                }
            }
            if (arc.mode_group_id >= 0) {
                const auto [it, inserted] = modes.emplace(arc.mode_group_id,
                    arc.is_vline_track_straight);
                if (!inserted && it->second != arc.is_vline_track_straight)
                    throw std::logic_error("candidate mode conflict");
            }
        }
    }
    if (complete && demands.size() != (net.is_sync_bus ? 1 : net.demands.size()))
        throw std::logic_error("candidate missing demand");
    // A repeated shared edge is harmless; a distinct cycle in the physical union is not.
    auto parent = std::map<int, int>{};
    const auto root = [&](auto&& self, int n) -> int {
        auto [it, _] = parent.emplace(n, n);
        if (it->second != n) it->second = self(self, it->second);
        return it->second;
    };
    for (const auto& [u, v] : edges) {
        const int a = root(root, u), b = root(root, v);
        if (a == b) throw std::logic_error("candidate physical cycle");
        parent[a] = b;
    }
    out.wirelength = wire_nodes.size();
    return out;
}

auto path_weight(const UnifiedGraph& graph, const UnifiedArc& arc,
                 const std::set<RouteResource>& own,
                 const RouteSearchOptions& options,
                 const ModeSwitches& groups) -> double {
    double weight = 0.001;
    const auto node = RouteResource{0, arc.v, 0};
    if (!own.contains(node) && is_wirelength_resource_node(graph, arc.v)) weight += 1.0;
    const auto add = [&](RouteResource resource) {
        if (own.contains(resource)) return;
        const auto it = options.prices.find(resource);
        if (it != options.prices.end()) weight += std::max(0.0, it->second);
    };
    add(node);
    for (const auto& resource : resources_for_arc(graph, arc, groups)) add(resource);
    if (options.discouraged_nodes.contains(arc.v)) weight += 2.0;
    if (options.variant > 0 && graph.nodes[static_cast<std::size_t>(arc.v)].kind
            == UnifiedNodeKind::Track)
        weight += 0.002 * ((arc.v * 17 + options.variant * 23) % 13);
    return weight;
}

auto conflicts_with_prefix(const UnifiedGraph& graph, const UnifiedArc& arc,
                           const std::Vector<int>& prefix,
                           const ModeSwitches& groups) -> bool {
    if (arc.physical_switch_id < 0) return false;
    const auto current = resources_for_arc(graph, arc, groups);
    for (std::size_t j = 1; j < prefix.size(); ++j) {
        const int previous = arc_between(graph, prefix[j - 1], prefix[j]);
        if (previous < 0) return true;
        const auto& used = graph.arcs[static_cast<std::size_t>(previous)];
        if (used.physical_switch_id < 0) continue;
        if (arc.mode_group_id >= 0 && used.mode_group_id == arc.mode_group_id &&
            arc.is_vline_track_straight != used.is_vline_track_straight)
            return true;
        if (arc.physical_switch_id == used.physical_switch_id) continue;
        const auto earlier = resources_for_arc(graph, used, groups);
        for (const auto& resource : current)
            if (resource.kind == 2 && earlier.contains(resource)) return true;
    }
    return false;
}

struct Label {
    double cost{};
    int node{};
    std::size_t length{};
    std::Vector<int> path;
};

auto search_path(const UnifiedGraph& graph, const RoutingScope& scope,
                 int source, int sink, std::uint16_t unit_mask,
                 const std::set<RouteResource>& own,
                 const RouteSearchOptions& options,
                 const ModeSwitches& groups,
                 const std::set<std::pair<int, int>>& banned_edges = {},
                 const std::set<int>& banned_nodes = {})
    -> std::Vector<int> {
    if (source < 0 || sink < 0 || source == sink) return {};
    auto labels = std::Vector<Label>{};
    labels.push_back({0.0, source,
        is_wirelength_resource_node(graph, source) ? 1U : 0U, {source}});
    using Item = std::pair<double, std::size_t>;
    auto queue = std::priority_queue<Item, std::Vector<Item>, std::greater<Item>>{};
    queue.emplace(0.0, 0);
    auto seen = std::map<std::pair<int, std::size_t>, int>{};
    auto best = std::Vector<double>(graph.nodes.size(),
        std::numeric_limits<double>::infinity());
    best[static_cast<std::size_t>(source)] = 0.0;
    int expansions = 0;
    while (!queue.empty() && expansions++ < options.max_expansions) {
        if (options.shared_expansions != nullptr &&
            --*options.shared_expansions < 0) break;
        if ((expansions == 1 || (expansions & 255) == 0) &&
            std::chrono::steady_clock::now() >= options.deadline) break;
        const auto [cost, index] = queue.top(); queue.pop();
        const auto label = labels[index];
        if (cost != label.cost) continue;
        if (!options.exact_length && cost > best[static_cast<std::size_t>(label.node)] + 1e-9)
            continue;
        if (options.exact_length &&
            seen[{label.node, label.length}]++ >= 16) continue;
        if (label.node == sink) {
            if (!options.exact_length || label.length == options.exact_length)
                return label.path;
            continue;
        }
        for (int aid : graph.out_arc_ids[static_cast<std::size_t>(label.node)]) {
            if (static_cast<std::size_t>(aid) >= scope.arc_offset.size() ||
                scope.arc_offset[static_cast<std::size_t>(aid)] < 0) continue;
            const auto& arc = graph.arcs[static_cast<std::size_t>(aid)];
            if (banned_edges.contains(std::minmax(arc.u, arc.v)) ||
                banned_nodes.contains(arc.v) ||
                arc.is_virtual_source_arc || !arc_unit_eligible(graph, arc, unit_mask) ||
                graph.nodes[static_cast<std::size_t>(arc.v)].kind ==
                    UnifiedNodeKind::VirtualSource ||
                (graph.nodes[static_cast<std::size_t>(arc.v)].kind ==
                    UnifiedNodeKind::Bump && arc.v != sink) ||
                std::find(label.path.begin(), label.path.end(), arc.v) != label.path.end() ||
                (options.exact_length &&
                 conflicts_with_prefix(graph, arc, label.path, groups)))
                continue;
            const std::size_t length = label.length +
                (is_wirelength_resource_node(graph, arc.v) ? 1U : 0U);
            if (options.exact_length && length > options.exact_length) continue;
            const double next = cost + path_weight(graph, arc, own, options, groups);
            if (!options.exact_length) {
                if (next + 1e-9 >= best[static_cast<std::size_t>(arc.v)]) continue;
                best[static_cast<std::size_t>(arc.v)] = next;
            }
            auto path = label.path; path.push_back(arc.v);
            labels.push_back({next, arc.v, length, std::move(path)});
            queue.emplace(next, labels.size() - 1);
        }
    }
    return {};
}

} // namespace

auto route_owners(const std::Vector<RoutingNet>& nets) -> std::Vector<RouteOwner> {
    auto result = std::Vector<RouteOwner>{};
    for (const auto& net : nets) {
        if (net.is_sync_bus) {
            for (const auto& demand : net.demands)
                result.push_back({net.net_id, demand.demand_id});
        } else result.push_back({net.net_id, 0});
    }
    return result;
}

auto net_for_owner(const std::Vector<RoutingNet>& nets, RouteOwner owner)
    -> const RoutingNet& {
    const auto it = std::find_if(nets.begin(), nets.end(),
        [&](const auto& net) { return net.net_id == owner.net_id; });
    if (it == nets.end()) throw std::logic_error("route owner lacks net");
    return *it;
}

auto scope_for_owner(const std::Vector<RoutingScope>& scopes, RouteOwner owner)
    -> const RoutingScope& {
    const auto it = std::find_if(scopes.begin(), scopes.end(),
        [&](const auto& scope) { return scope.net_id == owner.net_id; });
    if (it == scopes.end()) throw std::logic_error("route owner lacks scope");
    return *it;
}

auto route_column_from_paths(const UnifiedGraph& graph, const RoutingNet& net,
                             RouteOwner owner,
                             const std::Vector<SourceSinkPairPath>& paths)
    -> RouteColumn {
    return make_column(graph, net, owner, paths, true, mode_switches(graph));
}

auto find_route_column(const UnifiedGraph& graph, const RoutingNet& net,
                       const RoutingScope& scope, RouteOwner owner,
                       const RouteSearchOptions& options,
                       const RouteColumn* base,
                       std::size_t replace_demand) -> RouteColumn {
    auto out = RouteColumn{}; out.owner = owner;
    const auto groups = mode_switches(graph);
    if (base != nullptr)
        for (const auto& path : base->paths)
            if (path.demand_id != replace_demand) out.paths.push_back(path);
    auto demands = std::Vector<const RoutingDemand*>{};
    for (const auto& demand : net.demands)
        if ((base == nullptr || demand.demand_id == replace_demand) &&
            (!net.is_sync_bus || demand.demand_id == owner.demand_id))
            demands.push_back(&demand);
    if (demands.empty()) return out;
    if (options.variant > 0 && demands.size() > 1)
        std::rotate(demands.begin(), demands.begin() +
            (static_cast<std::size_t>(options.variant) % demands.size()), demands.end());
    for (const auto* demand : demands) {
        double best_cost = std::numeric_limits<double>::infinity();
        auto best_path = SourceSinkPairPath{};
        const int sink = resolve_graph_node(graph, demand->sink);
        for (const auto source_index : demand->candidate_source_indices) {
            const int source = resolve_graph_node(graph, net.sources.at(source_index));
            const auto& snode = graph.nodes.at(static_cast<std::size_t>(source));
            const auto& tnode = graph.nodes.at(static_cast<std::size_t>(sink));
            for (std::size_t unit = 0; unit < 16; ++unit) {
                if ((snode.kind == UnifiedNodeKind::Track && snode.unit != unit) ||
                    (tnode.kind == UnifiedNodeKind::Track && tnode.unit != unit)) continue;
                const auto own = make_column(graph, net, owner, out.paths,
                                             false, groups).resources;
                using Bans = std::set<std::pair<int, int>>;
                auto pending = std::deque<Bans>{{}};
                auto visited = std::set<Bans>{{}};
                int trials = 0;
                while (!pending.empty() && trials++ < 32) {
                    auto bans = std::move(pending.front()); pending.pop_front();
                    const auto path = search_path(graph, scope, source, sink,
                        unit_bit(unit), own, options, groups, bans);
                    if (path.empty()) continue;
                    auto candidate = out.paths;
                    candidate.push_back({net.net_id, source_index,
                        demand->demand_id,
                        net.kind == RoutingNetKind::PNnet ? source : -1, path});
                    try {
                        const auto trial = make_column(graph, net, owner,
                                                       candidate, false, groups);
                        double score = static_cast<double>(trial.wirelength) +
                            0.001 * path.size();
                        for (const auto& resource : trial.resources) {
                            const auto price = options.prices.find(resource);
                            if (price != options.prices.end()) score += price->second;
                        }
                        for (int node : path)
                            if (options.discouraged_nodes.contains(node)) score += 2.0;
                        if (score < best_cost) {
                            best_cost = score;
                            best_path = candidate.back();
                        }
                        // This source/unit is already legal; other variants
                        // supply route diversity during pricing.
                        break;
                    } catch (const std::logic_error&) {
                        // A shortest graph path can violate a TOB matching or
                        // mode rule. Remove one edge at a time to find a legal
                        // detour in the same unit before giving up.
                        for (std::size_t j = 1; j < path.size(); ++j) {
                            auto next = bans;
                            next.insert(std::minmax(path[j - 1], path[j]));
                            if (visited.insert(next).second)
                                pending.push_back(std::move(next));
                        }
                    }
                }
            }
        }
        if (!std::isfinite(best_cost)) return RouteColumn{owner};
        out.paths.push_back(std::move(best_path));
    }
    try { return make_column(graph, net, owner, out.paths, true, groups); }
    catch (const std::logic_error&) { return RouteColumn{owner}; }
}

static auto exclusive_branch_weight_with_groups(
    const UnifiedGraph& graph, const RouteColumn& base,
    std::size_t demand_id, const RouteSearchOptions& options,
    const ModeSwitches& groups) -> double {
    auto retained_nodes = std::set<int>{};
    auto retained_edges = std::set<std::pair<int, int>>{};
    auto retained_resources = std::set<RouteResource>{};
    auto target = static_cast<const SourceSinkPairPath*>(nullptr);
    for (const auto& path : base.paths) {
        if (path.demand_id == demand_id) { target = &path; continue; }
        retained_nodes.insert(path.node_path.begin(), path.node_path.end());
        for (int node : path.node_path) retained_resources.insert({0, node, 0});
        for (std::size_t j = 1; j < path.node_path.size(); ++j) {
            retained_edges.insert(std::minmax(path.node_path[j - 1], path.node_path[j]));
            const int aid = arc_between(graph, path.node_path[j - 1], path.node_path[j]);
            if (aid < 0) throw std::logic_error("retained terminal branch lacks arc");
            const auto resources = resources_for_arc(graph,
                graph.arcs[static_cast<std::size_t>(aid)], groups);
            retained_resources.insert(resources.begin(), resources.end());
        }
    }
    if (target == nullptr) return 0.0;
    double weight = 0.0;
    if (!target->node_path.empty() &&
        !retained_nodes.contains(target->node_path.front())) {
        const int source = target->node_path.front();
        if (is_wirelength_resource_node(graph, source)) weight += 1.0;
        const auto price = options.prices.find({0, source, 0});
        if (price != options.prices.end()) weight += std::max(0.0, price->second);
    }
    for (std::size_t j = 1; j < target->node_path.size(); ++j) {
        const int u = target->node_path[j - 1], v = target->node_path[j];
        if (retained_edges.contains(std::minmax(u, v))) continue;
        const int aid = arc_between(graph, u, v);
        if (aid < 0) throw std::logic_error("terminal branch lacks arc");
        weight += path_weight(graph, graph.arcs[static_cast<std::size_t>(aid)],
                              retained_resources, options, groups);
    }
    return weight;
}

auto exclusive_branch_weight(const UnifiedGraph& graph, const RouteColumn& base,
                             std::size_t demand_id,
                             const RouteSearchOptions& options) -> double {
    return exclusive_branch_weight_with_groups(
        graph, base, demand_id, options, mode_switches(graph));
}

auto terminal_branch_weights(const UnifiedGraph& graph, const RouteColumn& base,
                             const RouteSearchOptions& options)
    -> std::Vector<std::pair<double, std::size_t>> {
    const auto groups = mode_switches(graph);
    auto result = std::Vector<std::pair<double, std::size_t>>{};
    for (const auto& path : base.paths)
        result.emplace_back(exclusive_branch_weight_with_groups(
            graph, base, path.demand_id, options, groups), path.demand_id);
    return result;
}

auto find_terminal_columns(const UnifiedGraph& graph, const RoutingNet& net,
                           const RoutingScope& scope, RouteOwner owner,
                           const RouteSearchOptions& options,
                           const RouteColumn& base,
                           std::size_t replace_demand) -> std::Vector<RouteColumn> {
    auto result = std::Vector<RouteColumn>{};
    const auto demand = std::find_if(net.demands.begin(), net.demands.end(),
        [&](const auto& item) { return item.demand_id == replace_demand; });
    if (demand == net.demands.end() || base.paths.size() < 2) return result;
    auto kept = std::Vector<SourceSinkPairPath>{};
    struct Attachment {
        std::size_t source_index{};
        int physical_source_node{-1};
        std::Vector<int> prefix;
    };
    auto attachments = std::map<int, Attachment>{};
    for (const auto& path : base.paths) {
        if (path.demand_id == replace_demand) continue;
        kept.push_back(path);
        for (std::size_t j = 0; j < path.node_path.size(); ++j)
            attachments.try_emplace(path.node_path[j], Attachment{path.source_index,
                path.physical_source_node,
                std::Vector<int>(path.node_path.begin(), path.node_path.begin() + j + 1)});
    }
    if (kept.empty()) return result;
    if (net.kind == RoutingNetKind::PNnet)
        for (const auto source_index : demand->candidate_source_indices) {
            const int source = resolve_graph_node(graph, net.sources.at(source_index));
            attachments.try_emplace(source, Attachment{source_index, source, {source}});
        }
    const auto groups = mode_switches(graph);
    const auto own = make_column(graph, net, owner, kept, false, groups).resources;
    auto retained = std::set<int>{};
    for (const auto& [node, _] : attachments) retained.insert(node);
    const int sink = resolve_graph_node(graph, demand->sink);
    int shared_expansions = 4 * options.max_expansions;
    auto bounded = options;
    bounded.shared_expansions = &shared_expansions;
    for (const auto& [attachment, prefix] : attachments) {
        if (shared_expansions <= 0 ||
            std::chrono::steady_clock::now() >= options.deadline) break;
        if (std::find(demand->candidate_source_indices.begin(),
                      demand->candidate_source_indices.end(), prefix.source_index)
            == demand->candidate_source_indices.end()) continue;
        const auto& start_node = graph.nodes[static_cast<std::size_t>(attachment)];
        if (start_node.kind == UnifiedNodeKind::Bump && prefix.prefix.size() > 1 &&
            attachment != sink) continue;
        int prefix_unit = -1;
        for (int node : prefix.prefix)
            if (graph.nodes[static_cast<std::size_t>(node)].kind == UnifiedNodeKind::Track)
                prefix_unit = static_cast<int>(graph.nodes[static_cast<std::size_t>(node)].unit);
        const auto& sink_node = graph.nodes[static_cast<std::size_t>(sink)];
        auto banned_nodes = retained;
        banned_nodes.erase(attachment);
        for (std::size_t unit = 0; unit < 16; ++unit) {
            if (shared_expansions <= 0 ||
                std::chrono::steady_clock::now() >= options.deadline) break;
            if ((prefix_unit >= 0 && prefix_unit != static_cast<int>(unit)) ||
                (sink_node.kind == UnifiedNodeKind::Track && sink_node.unit != unit)) continue;
            using Bans = std::set<std::pair<int, int>>;
            auto pending = std::deque<Bans>{{}};
            auto visited = std::set<Bans>{{}};
            int trials = 0;
            while (!pending.empty() && trials++ < 16) {
                if (shared_expansions <= 0 ||
                    std::chrono::steady_clock::now() >= options.deadline) break;
                auto bans = std::move(pending.front()); pending.pop_front();
                auto branch = attachment == sink ? std::Vector<int>{attachment} :
                    search_path(graph, scope, attachment, sink, unit_bit(unit),
                                own, bounded, groups, bans, banned_nodes);
                if (branch.empty()) continue;
                auto full = prefix.prefix;
                full.insert(full.end(), branch.begin() + 1, branch.end());
                auto candidate_paths = kept;
                candidate_paths.push_back({net.net_id, prefix.source_index,
                    demand->demand_id, prefix.physical_source_node, std::move(full)});
                try {
                    auto column = make_column(graph, net, owner,
                                              candidate_paths, true, groups);
                    const auto original = std::find_if(base.paths.begin(), base.paths.end(),
                        [&](const auto& old) { return old.demand_id == replace_demand; });
                    if (original != base.paths.end() &&
                        original->source_index == column.paths.back().source_index &&
                        original->node_path == column.paths.back().node_path)
                        throw std::logic_error("unchanged terminal route");
                    if (std::ranges::none_of(result, [&](const auto& old) {
                        return old.paths.back().source_index ==
                                   column.paths.back().source_index &&
                               old.paths.back().node_path == column.paths.back().node_path;
                    })) result.push_back(std::move(column));
                    break;
                } catch (const std::logic_error&) {
                    for (std::size_t j = 1; j < branch.size(); ++j) {
                        auto next = bans;
                        next.insert(std::minmax(branch[j - 1], branch[j]));
                        if (visited.insert(next).second)
                            pending.push_back(std::move(next));
                    }
                }
            }
        }
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        return std::tuple{a.wirelength, a.paths.back().node_path.size()} <
               std::tuple{b.wirelength, b.paths.back().node_path.size()};
    });
    return result;
}

auto find_sync_prefix_column(const UnifiedGraph& graph, const RoutingNet& net,
                             const RoutingScope& scope, RouteOwner owner,
                             const RouteSearchOptions& options,
                             const RouteColumn& former, int tail_percent)
    -> RouteColumn {
    auto empty = RouteColumn{}; empty.owner = owner;
    if (!net.is_sync_bus || former.paths.size() != 1 ||
        options.exact_length == 0 || tail_percent < 1 || tail_percent > 100)
        return empty;
    const auto& old = former.paths.front();
    if (old.node_path.size() < 2) return empty;
    const std::size_t kept = std::max<std::size_t>(1,
        old.node_path.size() * static_cast<std::size_t>(100 - tail_percent) / 100);
    auto prefix = std::Vector<int>(old.node_path.begin(),
                                    old.node_path.begin() + kept);
    const int cut = prefix.back();
    const int sink = old.node_path.back();
    std::size_t prefix_length = 0;
    for (std::size_t j = 0; j + 1 < prefix.size(); ++j)
        if (is_wirelength_resource_node(graph, prefix[j])) ++prefix_length;
    if (prefix_length >= options.exact_length) return empty;
    const auto suffix_length = options.exact_length - prefix_length;
    auto forbidden = std::set<int>(prefix.begin(), prefix.end());
    forbidden.erase(cut);
    int prefix_unit = -1;
    for (int node : prefix) {
        const auto& data = graph.nodes[static_cast<std::size_t>(node)];
        if (data.kind == UnifiedNodeKind::Track)
            prefix_unit = static_cast<int>(data.unit);
    }
    auto search = options;
    search.exact_length = suffix_length;
    const auto groups = mode_switches(graph);
    for (std::size_t unit = 0; unit < 16; ++unit) {
        if (prefix_unit >= 0 && prefix_unit != static_cast<int>(unit)) continue;
        const auto suffix = search_path(graph, scope, cut, sink, unit_bit(unit),
                                         {}, search, groups, {}, forbidden);
        if (suffix.empty()) continue;
        auto path = prefix;
        path.insert(path.end(), suffix.begin() + 1, suffix.end());
        try {
            return make_column(graph, net, owner,
                {{net.net_id, old.source_index, old.demand_id,
                  old.physical_source_node, std::move(path)}}, true, groups);
        } catch (const std::logic_error&) { continue; }
    }
    return empty;
}

auto route_columns_conflict(const RouteColumn& a, const RouteColumn& b) -> bool {
    const auto& small = a.resources.size() < b.resources.size() ? a.resources : b.resources;
    const auto& large = a.resources.size() < b.resources.size() ? b.resources : a.resources;
    for (const auto& resource : small) if (large.contains(resource)) return true;
    return false;
}

} // namespace PR_tool
