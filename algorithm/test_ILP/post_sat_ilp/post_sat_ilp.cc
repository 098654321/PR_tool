#include "post_sat_ilp/post_sat_ilp.hh"

#include "common/cob_unit_mask.hh"
#include "common/hw_map.hh"
#include "global_route_v17/highs_log_sink.hh"
#include "sat/routing_path_log.hh"
#include "scope/scope_bbox.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <debug/debug.hh>
#include <format>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>

#ifdef USE_HIGHS
#include <Highs.h>
#endif

namespace PR_tool {

namespace {

constexpr double kMipRelativeGap = 0.015;

struct Segment {
    std::size_t id{0};
    std::size_t parent{0};
    int source{-1};
    int sink{-1};
    std::Vector<int> guide_nodes;
    std::Vector<int> guide_arcs;
    std::Vector<int> nodes;
    std::Vector<int> arcs;
};

struct Parent {
    std::size_t id{0};
    std::size_t net_id{0};
    int root{-1};
    std::uint16_t unit_mask{0};
    std::Vector<int> sinks;
    std::Vector<std::size_t> demand_ids;
    std::Vector<std::size_t> source_indices;
    std::Vector<std::size_t> segments;
};

struct Prepared {
    std::Vector<Parent> parents;
    std::Vector<Segment> segments;
    std::set<std::size_t> selected_net_ids;
};

struct LockedResources {
    std::Vector<bool> nodes;
    std::set<int> switches;
    std::set<std::size_t> net_ids;
};

struct SegmentVars {
    std::map<int, int> f;
};

struct ParentVars {
    std::map<int, int> x;
    std::map<int, int> y;
};

struct ModelStats {
    std::size_t f_vars{0};
    std::size_t x_vars{0};
    std::size_t y_vars{0};
    std::size_t mode_vars{0};
    std::size_t flow_rows{0};
    std::size_t flow_to_parent_rows{0};
    std::size_t parent_support_rows{0};
    std::size_t tree_rows{0};
    std::size_t endpoint_rows{0};
    std::size_t occupancy_rows{0};
    std::size_t switch_rows{0};
    std::size_t matching_rows{0};
    std::size_t mode_rows{0};
    std::size_t constraints{0};
    std::size_t nonzeros{0};
    std::size_t mip_start_entries{0};
    long long build_ms{0};
    long long solve_ms{0};
    double objective{0.0};
    double bound{0.0};
    double gap{0.0};
};

struct ParentSolution {
    std::set<int> arcs;
    std::set<int> nodes;
};

struct SegmentSolution {
    std::set<int> arcs;
};

struct ModelResult {
    bool ok{false};
    std::String status;
    ModelStats stats;
    std::Vector<ParentSolution> parents;
    std::Vector<SegmentSolution> segments;
    std::map<int, bool> modes;
};

auto is_physical_node(const UnifiedGraph& graph, const int node) -> bool {
    return node >= 0 && static_cast<std::size_t>(node) < graph.nodes.size() &&
           graph.nodes[static_cast<std::size_t>(node)].kind !=
               UnifiedNodeKind::VirtualSource;
}

auto is_wirelength_node(const UnifiedGraph& graph, const int node) -> bool {
    if (!is_physical_node(graph, node)) {
        return false;
    }
    const auto kind = graph.nodes[static_cast<std::size_t>(node)].kind;
    return kind == UnifiedNodeKind::Track || kind == UnifiedNodeKind::Bump;
}

auto find_arc(const UnifiedGraph& graph, const int u, const int v) -> int {
    if (u < 0 || static_cast<std::size_t>(u) >= graph.out_arc_ids.size()) {
        return -1;
    }
    for (const int arc : graph.out_arc_ids[static_cast<std::size_t>(u)]) {
        if (graph.arcs[static_cast<std::size_t>(arc)].v == v) {
            return arc;
        }
    }
    return -1;
}

auto path_arcs(const UnifiedGraph& graph, const std::Vector<int>& nodes)
    -> std::Vector<int> {
    auto out = std::Vector<int>{};
    for (std::size_t index = 1; index < nodes.size(); ++index) {
        const int arc = find_arc(graph, nodes[index - 1], nodes[index]);
        if (arc < 0) {
            throw std::logic_error(
                std::format("V20 ILP SAT guide has no graph arc {}->{}",
                            nodes[index - 1], nodes[index]));
        }
        out.push_back(arc);
    }
    return out;
}

auto net_for(const std::Vector<RoutingNet>& nets, const std::size_t net_id)
    -> const RoutingNet* {
    const auto it =
        std::find_if(nets.begin(), nets.end(), [&](const RoutingNet& net) {
            return net.net_id == net_id;
        });
    return it == nets.end() ? nullptr : &*it;
}

auto demand_for(const RoutingNet& net, const std::size_t demand_id)
    -> const RoutingDemand* {
    const auto it = std::find_if(
        net.demands.begin(), net.demands.end(),
        [&](const auto& demand) { return demand.demand_id == demand_id; });
    return it == net.demands.end() ? nullptr : &*it;
}

auto scope_for(const std::Vector<UnifiedSatNetScope>& scopes,
               const std::size_t net_id) -> const UnifiedSatNetScope* {
    const auto it =
        std::find_if(scopes.begin(), scopes.end(),
                     [&](const auto& scope) { return scope.net_id == net_id; });
    return it == scopes.end() ? nullptr : &*it;
}

auto paths_for(const SatRoutingResult& result, const std::size_t net_id)
    -> std::Vector<const SourceSinkPairPath*> {
    auto out = std::Vector<const SourceSinkPairPath*>{};
    for (const auto& path : result.paths) {
        if (path.net_id == net_id) {
            out.push_back(&path);
        }
    }
    std::sort(out.begin(), out.end(), [](const auto* lhs, const auto* rhs) {
        return lhs->demand_id < rhs->demand_id;
    });
    return out;
}

auto merge_node_bbox(const UnifiedGraph& graph, const int node,
                     IlpBoundingBox& box) -> void {
    if (!is_physical_node(graph, node)) {
        return;
    }
    const auto& value = graph.nodes[static_cast<std::size_t>(node)];
    switch (value.kind) {
    case UnifiedNodeKind::Track:
        merge_coord_into_bbox(
            box, track_to_cob(hardware::TrackCoord{
                     value.track_row, value.track_col,
                     value.track_dir == 0 ? hardware::TrackDirection::Horizontal
                                          : hardware::TrackDirection::Vertical,
                     value.track_index}));
        break;
    case UnifiedNodeKind::Bump:
        merge_coord_into_bbox(box, tob_anchor_cob(value.bump.TOB));
        break;
    case UnifiedNodeKind::HLine:
    case UnifiedNodeKind::VLine: {
        const auto [row, col] = tob_index_from_linear(value.tob);
        const auto [first, second] = tob_pair_cob_coords(row, col);
        merge_coord_into_bbox(box, first);
        merge_coord_into_bbox(box, second);
        break;
    }
    case UnifiedNodeKind::VirtualSource:
        break;
    }
}

auto guide_bbox(const UnifiedGraph& graph, const std::Vector<int>& guide,
                const int padding) -> IlpBoundingBox {
    auto box = IlpBoundingBox{std::numeric_limits<std::i64>::max(),
                              std::numeric_limits<std::i64>::min(),
                              std::numeric_limits<std::i64>::max(),
                              std::numeric_limits<std::i64>::min()};
    for (const int node : guide) {
        merge_node_bbox(graph, node, box);
    }
    if (box.row_min > box.row_max) {
        throw std::logic_error("V20 ILP segment guide has no physical bbox");
    }
    for (int step = 0; step < padding; ++step) {
        box = expand_pair_bbox_one_cell(box);
    }
    return clamp_bbox_to_cob_array(box);
}

auto collect_locked(const UnifiedGraph& graph, const SatRoutingResult& result,
                    const std::set<std::size_t>& selected) -> LockedResources {
    auto out = LockedResources{};
    out.nodes.assign(graph.nodes.size(), false);
    for (const auto& path : result.paths) {
        if (selected.contains(path.net_id)) {
            continue;
        }
        out.net_ids.insert(path.net_id);
        for (const int node : path.node_path) {
            if (is_physical_node(graph, node)) {
                out.nodes[static_cast<std::size_t>(node)] = true;
            }
        }
        for (const int arc : path_arcs(graph, path.node_path)) {
            const int physical_switch =
                graph.arcs[static_cast<std::size_t>(arc)].physical_switch_id;
            if (physical_switch >= 0) {
                out.switches.insert(physical_switch);
            }
        }
    }
    return out;
}

auto build_rooted_tree(const std::Vector<const SourceSinkPairPath*>& paths,
                       const int root, const std::set<int>& sinks)
    -> std::set<std::pair<int, int>> {
    auto adjacency = std::map<int, std::set<int>>{};
    for (const auto* path : paths) {
        for (std::size_t index = 1; index < path->node_path.size(); ++index) {
            const int u = path->node_path[index - 1];
            const int v = path->node_path[index];
            adjacency[u].insert(v);
            adjacency[v].insert(u);
        }
    }
    auto parent = std::map<int, int>{{root, root}};
    auto queue = std::queue<int>{};
    queue.push(root);
    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        for (const int next : adjacency[node]) {
            if (parent.contains(next)) {
                continue;
            }
            parent.emplace(next, node);
            queue.push(next);
        }
    }
    auto retained = std::set<int>{root};
    for (const int sink : sinks) {
        if (!parent.contains(sink)) {
            throw std::logic_error(std::format(
                "V20 ILP SAT tree cannot reach sink {} from root {}", sink,
                root));
        }
        for (int node = sink; node != root; node = parent.at(node)) {
            retained.insert(node);
        }
    }
    auto edges = std::set<std::pair<int, int>>{};
    for (const int node : retained) {
        if (node != root) {
            edges.emplace(parent.at(node), node);
        }
    }
    return edges;
}

auto tree_segments(const int root, const std::set<int>& sinks,
                   const std::set<std::pair<int, int>>& edges)
    -> std::Vector<std::Vector<int>> {
    auto degree = std::map<int, int>{};
    auto children = std::map<int, std::Vector<int>>{};
    for (const auto& [u, v] : edges) {
        ++degree[u];
        ++degree[v];
        children[u].push_back(v);
    }
    auto critical = sinks;
    critical.insert(root);
    for (const auto& [node, count] : degree) {
        if (count >= 3) {
            critical.insert(node);
        }
    }
    auto out = std::Vector<std::Vector<int>>{};
    auto pending = std::queue<int>{};
    pending.push(root);
    while (!pending.empty()) {
        const int start = pending.front();
        pending.pop();
        for (const int first : children[start]) {
            auto path = std::Vector<int>{start, first};
            int node = first;
            while (!critical.contains(node)) {
                if (!children.contains(node) || children[node].size() != 1) {
                    throw std::logic_error("V20 ILP recovered tree has a "
                                           "non-critical branch/leaf");
                }
                node = children[node].front();
                path.push_back(node);
            }
            out.push_back(path);
            if (children.contains(node)) {
                pending.push(node);
            }
        }
    }
    return out;
}

auto build_segment_domain(const UnifiedGraph& graph,
                          const UnifiedSatNetScope& sat_scope,
                          const Parent& parent, Segment& segment,
                          const LockedResources& locked, const int bbox_padding)
    -> void {
    const auto box = guide_bbox(graph, segment.guide_nodes, bbox_padding);
    const auto guide_nodes =
        std::set<int>(segment.guide_nodes.begin(), segment.guide_nodes.end());
    const auto guide_arcs =
        std::set<int>(segment.guide_arcs.begin(), segment.guide_arcs.end());
    auto allowed = std::Vector<bool>(graph.nodes.size(), false);
    for (const int node : sat_scope.node_ids) {
        if (!is_physical_node(graph, node)) {
            continue;
        }
        const bool forced = guide_nodes.contains(node) ||
                            node == segment.source || node == segment.sink;
        if (!forced && !node_in_scope(graph, node, box)) {
            continue;
        }
        if (!node_unit_eligible(graph.nodes[static_cast<std::size_t>(node)],
                                parent.unit_mask)) {
            continue;
        }
        if (locked.nodes[static_cast<std::size_t>(node)]) {
            if (forced) {
                throw std::logic_error(std::format(
                    "V20 ILP SAT guide node {} is occupied by a locked net",
                    node));
            }
            continue;
        }
        allowed[static_cast<std::size_t>(node)] = true;
    }
    auto candidate = std::Vector<int>{};
    auto forward = std::Vector<std::Vector<int>>(graph.nodes.size());
    auto reverse = std::Vector<std::Vector<int>>(graph.nodes.size());
    for (const int arc : sat_scope.arc_ids) {
        const auto& value = graph.arcs[static_cast<std::size_t>(arc)];
        if (!allowed[static_cast<std::size_t>(value.u)] ||
            !allowed[static_cast<std::size_t>(value.v)] ||
            !arc_unit_eligible(graph, value, parent.unit_mask)) {
            continue;
        }
        if (value.physical_switch_id >= 0 &&
            locked.switches.contains(value.physical_switch_id)) {
            if (guide_arcs.contains(arc)) {
                throw std::logic_error(std::format(
                    "V20 ILP SAT guide switch {} is occupied by a locked net",
                    value.physical_switch_id));
            }
            continue;
        }
        candidate.push_back(arc);
        forward[static_cast<std::size_t>(value.u)].push_back(arc);
        reverse[static_cast<std::size_t>(value.v)].push_back(arc);
    }
    auto from_source = std::Vector<bool>(graph.nodes.size(), false);
    auto to_sink = std::Vector<bool>(graph.nodes.size(), false);
    auto queue = std::queue<int>{};
    from_source[static_cast<std::size_t>(segment.source)] = true;
    queue.push(segment.source);
    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        for (const int arc : forward[static_cast<std::size_t>(node)]) {
            const int next = graph.arcs[static_cast<std::size_t>(arc)].v;
            if (!from_source[static_cast<std::size_t>(next)]) {
                from_source[static_cast<std::size_t>(next)] = true;
                queue.push(next);
            }
        }
    }
    to_sink[static_cast<std::size_t>(segment.sink)] = true;
    queue.push(segment.sink);
    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        for (const int arc : reverse[static_cast<std::size_t>(node)]) {
            const int previous = graph.arcs[static_cast<std::size_t>(arc)].u;
            if (!to_sink[static_cast<std::size_t>(previous)]) {
                to_sink[static_cast<std::size_t>(previous)] = true;
                queue.push(previous);
            }
        }
    }
    if (!from_source[static_cast<std::size_t>(segment.sink)]) {
        throw std::logic_error(
            "V20 ILP segment domain disconnects its SAT endpoints");
    }
    auto nodes = std::set<int>{segment.source, segment.sink};
    for (const int arc : candidate) {
        const auto& value = graph.arcs[static_cast<std::size_t>(arc)];
        if (from_source[static_cast<std::size_t>(value.u)] &&
            to_sink[static_cast<std::size_t>(value.v)]) {
            segment.arcs.push_back(arc);
            nodes.insert(value.u);
            nodes.insert(value.v);
        }
    }
    segment.nodes.assign(nodes.begin(), nodes.end());
    const auto domain_arcs =
        std::set<int>(segment.arcs.begin(), segment.arcs.end());
    const auto domain_nodes =
        std::set<int>(segment.nodes.begin(), segment.nodes.end());
    for (const int node : segment.guide_nodes) {
        if (!domain_nodes.contains(node)) {
            throw std::logic_error(std::format(
                "V20 ILP segment domain dropped SAT guide node {}", node));
        }
    }
    for (const int arc : segment.guide_arcs) {
        if (!domain_arcs.contains(arc)) {
            throw std::logic_error(std::format(
                "V20 ILP segment domain dropped SAT guide arc {}", arc));
        }
    }
}

auto prepare_problem(const UnifiedGraph& graph,
                     const std::Vector<RoutingNet>& nets,
                     const std::Vector<UnifiedSatNetScope>& scopes,
                     const SatRoutingResult& sat_result, const int bbox_padding)
    -> std::pair<Prepared, LockedResources> {
    auto prepared = Prepared{};
    for (const auto& net : nets) {
        if (is_post_sat_ilp_target(net)) {
            prepared.selected_net_ids.insert(net.net_id);
        }
    }
    const auto locked =
        collect_locked(graph, sat_result, prepared.selected_net_ids);
    for (const auto& net : nets) {
        if (!prepared.selected_net_ids.contains(net.net_id)) {
            continue;
        }
        const auto* sat_scope = scope_for(scopes, net.net_id);
        if (sat_scope == nullptr) {
            throw std::logic_error(std::format(
                "V20 ILP net {} has no final SAT scope", net.net_id));
        }
        const auto net_paths = paths_for(sat_result, net.net_id);
        if (net_paths.size() != net.demands.size() || net_paths.empty()) {
            throw std::logic_error(
                std::format("V20 ILP net {} has {} SAT paths for {} demands",
                            net.net_id, net_paths.size(), net.demands.size()));
        }
        auto parent = Parent{};
        parent.id = prepared.parents.size();
        parent.net_id = net.net_id;
        parent.root = net_paths.front()->node_path.empty()
                          ? -1
                          : net_paths.front()->node_path.front();
        if (parent.root < 0 ||
            graph.nodes[static_cast<std::size_t>(parent.root)].kind !=
                UnifiedNodeKind::Track) {
            throw std::logic_error(
                "V20 ILP target root is not a physical Track");
        }
        parent.unit_mask =
            unit_bit(graph.nodes[static_cast<std::size_t>(parent.root)].unit);
        auto sink_set = std::set<int>{};
        for (const auto* path : net_paths) {
            if (path->node_path.empty() ||
                path->node_path.front() != parent.root) {
                throw std::logic_error(
                    "V20 ILP target paths do not share one fixed Track root");
            }
            const auto* demand = demand_for(net, path->demand_id);
            if (demand == nullptr || path->source_index >= net.sources.size()) {
                throw std::logic_error(
                    "V20 ILP SAT path has invalid demand/source metadata");
            }
            const int sink = resolve_graph_node(graph, demand->sink);
            if (path->node_path.back() != sink) {
                throw std::logic_error(
                    "V20 ILP SAT path does not end at its demand sink");
            }
            parent.sinks.push_back(sink);
            parent.demand_ids.push_back(path->demand_id);
            parent.source_indices.push_back(path->source_index);
            sink_set.insert(sink);
        }
        auto guides = std::Vector<std::Vector<int>>{};
        if (net_paths.size() == 1) {
            guides.push_back(net_paths.front()->node_path);
        } else {
            const auto tree =
                build_rooted_tree(net_paths, parent.root, sink_set);
            guides = tree_segments(parent.root, sink_set, tree);
        }
        for (auto& guide : guides) {
            auto segment = Segment{};
            segment.id = prepared.segments.size();
            segment.parent = parent.id;
            segment.source = guide.front();
            segment.sink = guide.back();
            segment.guide_nodes = std::move(guide);
            segment.guide_arcs = path_arcs(graph, segment.guide_nodes);
            build_segment_domain(graph, *sat_scope, parent, segment, locked,
                                 bbox_padding);
            parent.segments.push_back(segment.id);
            prepared.segments.push_back(std::move(segment));
        }
        prepared.parents.push_back(std::move(parent));
    }
    return {std::move(prepared), locked};
}

#ifdef USE_HIGHS

class SparseMip {
  public:
    SparseMip(const int verbose_level, const std::string_view log_path,
              const int time_limit_minutes)
        : log_sink_(log_path, verbose_level >= 2, true) {
        log_sink_.attach(highs_);
        check(highs_.setOptionValue("mip_rel_gap", kMipRelativeGap),
              "mip_rel_gap");
        if (time_limit_minutes > 0) {
            check(highs_.setOptionValue(
                      "time_limit",
                      static_cast<double>(time_limit_minutes) * 60.0),
                  "time_limit");
        }
    }

    auto add_binary(const double cost = 0.0) -> int {
        const int column = static_cast<int>(variables_++);
        check(highs_.addCol(cost, 0.0, 1.0, 0, nullptr, nullptr), "addCol");
        check(highs_.changeColIntegrality(column, HighsVarType::kInteger),
              "integrality");
        return column;
    }

    auto add_row(const double lower, const double upper,
                 const std::Vector<std::pair<int, double>>& terms) -> void {
        auto combined = std::map<int, double>{};
        for (const auto& [column, coefficient] : terms) {
            combined[column] += coefficient;
        }
        auto indices = std::Vector<HighsInt>{};
        auto values = std::Vector<double>{};
        for (const auto& [column, coefficient] : combined) {
            if (coefficient != 0.0) {
                indices.push_back(static_cast<HighsInt>(column));
                values.push_back(coefficient);
            }
        }
        check(highs_.addRow(lower, upper, static_cast<HighsInt>(indices.size()),
                            indices.empty() ? nullptr : indices.data(),
                            values.empty() ? nullptr : values.data()),
              "addRow");
        ++constraints_;
        nonzeros_ += indices.size();
    }

    auto set_start(const std::map<int, double>& values) -> void {
        auto indices = std::Vector<HighsInt>{};
        auto starts = std::Vector<double>{};
        for (const auto& [column, value] : values) {
            indices.push_back(static_cast<HighsInt>(column));
            starts.push_back(value);
        }
        check(highs_.setSolution(static_cast<HighsInt>(indices.size()),
                                 indices.data(), starts.data()),
              "setSolution");
    }

    auto solve() -> HighsModelStatus {
        check(highs_.run(), "run");
        return highs_.getModelStatus();
    }

    [[nodiscard]] auto feasible() const -> bool {
        return highs_.getInfo().primal_solution_status ==
               kSolutionStatusFeasible;
    }
    [[nodiscard]] auto values() const -> const std::Vector<double>& {
        return highs_.getSolution().col_value;
    }
    [[nodiscard]] auto info() const -> const HighsInfo& {
        return highs_.getInfo();
    }
    [[nodiscard]] auto status() const -> std::String {
        return highs_.modelStatusToString(highs_.getModelStatus());
    }
    [[nodiscard]] auto variables() const -> std::size_t { return variables_; }
    [[nodiscard]] auto constraints() const -> std::size_t {
        return constraints_;
    }
    [[nodiscard]] auto nonzeros() const -> std::size_t { return nonzeros_; }

  private:
    static auto check(const HighsStatus status, const char* operation) -> void {
        if (status == HighsStatus::kError) {
            throw std::runtime_error(
                std::format("V20 ILP HiGHS {} failed", operation));
        }
    }

    HighsLogSink log_sink_;
    Highs highs_;
    std::size_t variables_{0};
    std::size_t constraints_{0};
    std::size_t nonzeros_{0};
};

auto solve_model(const UnifiedGraph& graph, const Prepared& prepared,
                 const LockedResources& locked,
                 const SatRoutingResult& sat_result,
                 const PostSatIlpOptions& options) -> ModelResult {
    const auto build_begin = std::chrono::steady_clock::now();
    auto out = ModelResult{};
    auto mip = SparseMip{options.verbose_level, options.highs_log_path,
                         options.highs_time_limit_minutes};
    auto segment_vars = std::Vector<SegmentVars>(prepared.segments.size());
    auto parent_vars = std::Vector<ParentVars>(prepared.parents.size());
    auto parent_arcs = std::Vector<std::set<int>>(prepared.parents.size());
    auto parent_nodes = std::Vector<std::set<int>>(prepared.parents.size());

    for (const auto& parent : prepared.parents) {
        for (const auto segment_id : parent.segments) {
            const auto& segment = prepared.segments[segment_id];
            parent_arcs[parent.id].insert(segment.arcs.begin(),
                                          segment.arcs.end());
            parent_nodes[parent.id].insert(segment.nodes.begin(),
                                           segment.nodes.end());
        }
        for (const int arc : parent_arcs[parent.id]) {
            parent_vars[parent.id].x.emplace(arc, mip.add_binary());
            ++out.stats.x_vars;
        }
        for (const int node : parent_nodes[parent.id]) {
            parent_vars[parent.id].y.emplace(
                node,
                mip.add_binary(is_wirelength_node(graph, node) ? 1.0 : 0.0));
            ++out.stats.y_vars;
        }
    }
    for (const auto& segment : prepared.segments) {
        for (const int arc : segment.arcs) {
            segment_vars[segment.id].f.emplace(arc, mip.add_binary());
            ++out.stats.f_vars;
        }
    }

    for (const auto& segment : prepared.segments) {
        const auto& sv = segment_vars[segment.id];
        const auto& pv = parent_vars[segment.parent];
        for (const auto& [arc, f] : sv.f) {
            mip.add_row(-kHighsInf, 0.0, {{f, 1.0}, {pv.x.at(arc), -1.0}});
            ++out.stats.flow_to_parent_rows;
        }
        for (const int node : segment.nodes) {
            auto terms = std::Vector<std::pair<int, double>>{};
            for (const int arc :
                 graph.out_arc_ids[static_cast<std::size_t>(node)]) {
                if (const auto it = sv.f.find(arc); it != sv.f.end()) {
                    terms.emplace_back(it->second, 1.0);
                }
            }
            for (const int arc :
                 graph.in_arc_ids[static_cast<std::size_t>(node)]) {
                if (const auto it = sv.f.find(arc); it != sv.f.end()) {
                    terms.emplace_back(it->second, -1.0);
                }
            }
            const double balance = node == segment.source ? 1.0
                                   : node == segment.sink ? -1.0
                                                          : 0.0;
            mip.add_row(balance, balance, terms);
            ++out.stats.flow_rows;
        }
    }

    for (const auto& parent : prepared.parents) {
        auto& pv = parent_vars[parent.id];
        for (const auto& [arc, x] : pv.x) {
            auto terms = std::Vector<std::pair<int, double>>{{x, 1.0}};
            for (const auto segment_id : parent.segments) {
                const auto& f = segment_vars[segment_id].f;
                if (const auto it = f.find(arc); it != f.end()) {
                    terms.emplace_back(it->second, -1.0);
                }
            }
            mip.add_row(-kHighsInf, 0.0, terms);
            ++out.stats.parent_support_rows;
            const auto& value = graph.arcs[static_cast<std::size_t>(arc)];
            mip.add_row(-kHighsInf, 0.0, {{x, 1.0}, {pv.y.at(value.u), -1.0}});
            mip.add_row(-kHighsInf, 0.0, {{x, 1.0}, {pv.y.at(value.v), -1.0}});
            out.stats.tree_rows += 2;
        }
        for (const int node : parent_nodes[parent.id]) {
            auto incoming = std::Vector<std::pair<int, double>>{};
            for (const int arc :
                 graph.in_arc_ids[static_cast<std::size_t>(node)]) {
                if (const auto it = pv.x.find(arc); it != pv.x.end()) {
                    incoming.emplace_back(it->second, 1.0);
                }
            }
            if (node == parent.root) {
                mip.add_row(0.0, 0.0, incoming);
                ++out.stats.endpoint_rows;
            } else {
                incoming.emplace_back(pv.y.at(node), -1.0);
                mip.add_row(0.0, 0.0, incoming);
                ++out.stats.tree_rows;
            }
        }
        mip.add_row(1.0, 1.0, {{pv.y.at(parent.root), 1.0}});
        ++out.stats.endpoint_rows;
        for (const int sink : parent.sinks) {
            mip.add_row(1.0, 1.0, {{pv.y.at(sink), 1.0}});
            auto outgoing = std::Vector<std::pair<int, double>>{};
            for (const int arc :
                 graph.out_arc_ids[static_cast<std::size_t>(sink)]) {
                if (const auto it = pv.x.find(arc); it != pv.x.end()) {
                    outgoing.emplace_back(it->second, 1.0);
                }
            }
            mip.add_row(0.0, 0.0, outgoing);
            out.stats.endpoint_rows += 2;
        }
    }

    auto occupancy = std::map<int, std::Vector<int>>{};
    for (const auto& pv : parent_vars) {
        for (const auto& [node, y] : pv.y) {
            occupancy[node].push_back(y);
        }
    }
    for (const auto& [node, variables] : occupancy) {
        auto terms = std::Vector<std::pair<int, double>>{};
        for (const int variable : variables) {
            terms.emplace_back(variable, 1.0);
        }
        mip.add_row(-kHighsInf,
                    locked.nodes[static_cast<std::size_t>(node)] ? 0.0 : 1.0,
                    terms);
        ++out.stats.occupancy_rows;
    }

    struct SwitchUse {
        int locked{0};
        std::Vector<int> variables;
        const UnifiedArc* exemplar{nullptr};
    };
    auto switches = std::map<int, SwitchUse>{};
    for (const auto& arc : graph.arcs) {
        if (arc.physical_switch_id >= 0) {
            switches[arc.physical_switch_id].exemplar = &arc;
        }
    }
    for (const int physical_switch : locked.switches) {
        switches[physical_switch].locked = 1;
    }
    for (const auto& pv : parent_vars) {
        for (const auto& [arc, x] : pv.x) {
            const int physical_switch =
                graph.arcs[static_cast<std::size_t>(arc)].physical_switch_id;
            if (physical_switch >= 0) {
                switches[physical_switch].variables.push_back(x);
            }
        }
    }
    for (const auto& [physical_switch, use] : switches) {
        (void)physical_switch;
        if (use.locked == 0 && use.variables.empty()) {
            continue;
        }
        auto terms = std::Vector<std::pair<int, double>>{};
        for (const int variable : use.variables)
            terms.emplace_back(variable, 1.0);
        mip.add_row(-kHighsInf, 1.0 - use.locked, terms);
        ++out.stats.switch_rows;
    }

    auto matching = std::map<std::pair<int, int>, std::set<int>>{};
    auto active_modes = std::set<int>{};
    for (const auto& [physical_switch, use] : switches) {
        if ((use.locked == 0 && use.variables.empty()) ||
            use.exemplar == nullptr)
            continue;
        const auto& arc = *use.exemplar;
        const auto u = graph.nodes[static_cast<std::size_t>(arc.u)].kind;
        const auto v = graph.nodes[static_cast<std::size_t>(arc.v)].kind;
        if (u == UnifiedNodeKind::Bump && v == UnifiedNodeKind::HLine) {
            matching[{0, arc.u}].insert(physical_switch);
            matching[{1, arc.v}].insert(physical_switch);
        } else if (u == UnifiedNodeKind::HLine && v == UnifiedNodeKind::Bump) {
            matching[{0, arc.v}].insert(physical_switch);
            matching[{1, arc.u}].insert(physical_switch);
        } else if (u == UnifiedNodeKind::HLine && v == UnifiedNodeKind::VLine) {
            matching[{2, arc.u}].insert(physical_switch);
            matching[{3, arc.v}].insert(physical_switch);
        } else if (u == UnifiedNodeKind::VLine && v == UnifiedNodeKind::HLine) {
            matching[{2, arc.v}].insert(physical_switch);
            matching[{3, arc.u}].insert(physical_switch);
        }
        if (arc.physical_switch_kind == PhysicalSwitchKind::VLineTrack &&
            arc.mode_group_id >= 0) {
            active_modes.insert(arc.mode_group_id);
        }
    }
    for (const auto& [key, switch_ids] : matching) {
        (void)key;
        int constant = 0;
        auto terms = std::Vector<std::pair<int, double>>{};
        for (const int physical_switch : switch_ids) {
            const auto& use = switches.at(physical_switch);
            constant += use.locked;
            for (const int variable : use.variables)
                terms.emplace_back(variable, 1.0);
        }
        mip.add_row(-kHighsInf, 1.0 - constant, terms);
        ++out.stats.matching_rows;
    }
    auto mode_vars = std::map<int, int>{};
    for (const int group : active_modes) {
        mode_vars.emplace(group, mip.add_binary());
        ++out.stats.mode_vars;
    }
    for (const auto& [physical_switch, use] : switches) {
        (void)physical_switch;
        if (use.exemplar == nullptr ||
            use.exemplar->physical_switch_kind !=
                PhysicalSwitchKind::VLineTrack ||
            use.exemplar->mode_group_id < 0 ||
            !mode_vars.contains(use.exemplar->mode_group_id))
            continue;
        auto terms = std::Vector<std::pair<int, double>>{};
        for (const int variable : use.variables)
            terms.emplace_back(variable, 1.0);
        const int mode = mode_vars.at(use.exemplar->mode_group_id);
        if (use.exemplar->is_vline_track_straight) {
            terms.emplace_back(mode, -1.0);
            mip.add_row(-kHighsInf, -use.locked, terms);
            ++out.stats.mode_rows;
        }
        if (use.exemplar->is_vline_track_swap) {
            terms.emplace_back(mode, 1.0);
            mip.add_row(-kHighsInf, 1.0 - use.locked, terms);
            ++out.stats.mode_rows;
        }
    }

    auto start = std::map<int, double>{};
    for (int variable = 0; variable < static_cast<int>(mip.variables());
         ++variable) {
        start.emplace(variable, 0.0);
    }
    for (const auto& segment : prepared.segments) {
        for (const int arc : segment.guide_arcs) {
            start[segment_vars[segment.id].f.at(arc)] = 1.0;
            start[parent_vars[segment.parent].x.at(arc)] = 1.0;
            const auto& value = graph.arcs[static_cast<std::size_t>(arc)];
            start[parent_vars[segment.parent].y.at(value.u)] = 1.0;
            start[parent_vars[segment.parent].y.at(value.v)] = 1.0;
        }
    }
    for (const auto& [group, variable] : mode_vars) {
        const auto it = sat_result.vline_mode_straight_by_group.find(
            static_cast<std::size_t>(group));
        start[variable] =
            it != sat_result.vline_mode_straight_by_group.end() && it->second
                ? 1.0
                : 0.0;
    }
    mip.set_start(start);
    out.stats.mip_start_entries = start.size();
    out.stats.build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - build_begin)
                             .count();
    out.stats.constraints = mip.constraints();
    out.stats.nonzeros = mip.nonzeros();
    debug::info_fmt("V20 post-SAT ILP model built: parents={} segments={} "
                    "vars={} constraints={} nonzeros={} F={} X={} Y={} M={} "
                    "warm_start_entries={} build_ms={}",
                    prepared.parents.size(), prepared.segments.size(),
                    mip.variables(), mip.constraints(), mip.nonzeros(),
                    out.stats.f_vars, out.stats.x_vars, out.stats.y_vars,
                    out.stats.mode_vars, out.stats.mip_start_entries,
                    out.stats.build_ms);
    if (options.verbose_level >= 1) {
        debug::info("V20 post-SAT ILP constraints by category:");
        debug::info_fmt("  segment flow                 : {}",
                        out.stats.flow_rows);
        debug::info_fmt("  f implies parent x           : {}",
                        out.stats.flow_to_parent_rows);
        debug::info_fmt("  parent x supported by f      : {}",
                        out.stats.parent_support_rows);
        debug::info_fmt("  parent arborescence          : {}",
                        out.stats.tree_rows);
        debug::info_fmt("  root/sink endpoints          : {}",
                        out.stats.endpoint_rows);
        debug::info_fmt("  physical node occupancy      : {}",
                        out.stats.occupancy_rows);
        debug::info_fmt("  physical switch uniqueness   : {}",
                        out.stats.switch_rows);
        debug::info_fmt("  TOB partial matching         : {}",
                        out.stats.matching_rows);
        debug::info_fmt("  VLine/Track straight-swap    : {}",
                        out.stats.mode_rows);
    }

    const auto solve_begin = std::chrono::steady_clock::now();
    (void)mip.solve();
    out.stats.solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - solve_begin)
                             .count();
    out.status = mip.status();
    if (!mip.feasible()) {
        return out;
    }
    const auto& values = mip.values();
    const auto selected = [&](const int variable) {
        return variable >= 0 &&
               static_cast<std::size_t>(variable) < values.size() &&
               values[static_cast<std::size_t>(variable)] > 0.5;
    };
    out.parents.resize(prepared.parents.size());
    out.segments.resize(prepared.segments.size());
    for (const auto& parent : prepared.parents) {
        for (const auto& [arc, variable] : parent_vars[parent.id].x) {
            if (selected(variable))
                out.parents[parent.id].arcs.insert(arc);
        }
        for (const auto& [node, variable] : parent_vars[parent.id].y) {
            if (selected(variable))
                out.parents[parent.id].nodes.insert(node);
        }
    }
    for (const auto& segment : prepared.segments) {
        for (const auto& [arc, variable] : segment_vars[segment.id].f) {
            if (selected(variable))
                out.segments[segment.id].arcs.insert(arc);
        }
    }
    for (const auto& [group, variable] : mode_vars) {
        out.modes.emplace(group, selected(variable));
    }
    const auto& info = mip.info();
    out.stats.objective = info.objective_function_value;
    out.stats.bound = info.mip_dual_bound;
    out.stats.gap = info.mip_gap;
    out.ok = true;
    return out;
}

#else

auto solve_model(const UnifiedGraph&, const Prepared&, const LockedResources&,
                 const SatRoutingResult&, const PostSatIlpOptions&)
    -> ModelResult {
    auto out = ModelResult{};
    out.status = "HIGHS_UNAVAILABLE";
    return out;
}

#endif

auto validate_model(const UnifiedGraph& graph, const Prepared& prepared,
                    const ModelResult& model) -> std::Vector<std::String> {
    auto errors = std::Vector<std::String>{};
    for (const auto& segment : prepared.segments) {
        const auto& flow = model.segments[segment.id].arcs;
        auto balance = std::map<int, int>{};
        for (const int arc : flow) {
            if (!model.parents[segment.parent].arcs.contains(arc)) {
                errors.push_back(std::format(
                    "segment {} f arc {} has no parent x", segment.id, arc));
            }
            const auto& value = graph.arcs[static_cast<std::size_t>(arc)];
            ++balance[value.u];
            --balance[value.v];
        }
        for (const int node : segment.nodes) {
            const int expected = node == segment.source ? 1
                                 : node == segment.sink ? -1
                                                        : 0;
            if (balance[node] != expected) {
                errors.push_back(std::format(
                    "segment {} flow balance at {} is {}, expected {}",
                    segment.id, node, balance[node], expected));
            }
        }
    }
    for (const auto& parent : prepared.parents) {
        auto supported = std::set<int>{};
        for (const auto segment : parent.segments) {
            supported.insert(model.segments[segment].arcs.begin(),
                             model.segments[segment].arcs.end());
        }
        for (const int arc : model.parents[parent.id].arcs) {
            if (!supported.contains(arc)) {
                errors.push_back(std::format(
                    "parent {} x arc {} has no segment f", parent.id, arc));
            }
        }
        auto visited = std::set<int>{parent.root};
        auto queue = std::queue<int>{};
        queue.push(parent.root);
        while (!queue.empty()) {
            const int node = queue.front();
            queue.pop();
            for (const int arc :
                 graph.out_arc_ids[static_cast<std::size_t>(node)]) {
                if (!model.parents[parent.id].arcs.contains(arc))
                    continue;
                const int next = graph.arcs[static_cast<std::size_t>(arc)].v;
                if (visited.insert(next).second)
                    queue.push(next);
            }
        }
        for (const int sink : parent.sinks) {
            if (!visited.contains(sink)) {
                errors.push_back(std::format("parent {} cannot reach sink {}",
                                             parent.id, sink));
            }
        }
        for (const int node : model.parents[parent.id].nodes) {
            if (!visited.contains(node)) {
                errors.push_back(std::format(
                    "parent {} contains disconnected selected node {}",
                    parent.id, node));
            }
        }
        for (const int arc : model.parents[parent.id].arcs) {
            const auto& value = graph.arcs[static_cast<std::size_t>(arc)];
            if (!visited.contains(value.u) || !visited.contains(value.v)) {
                errors.push_back(std::format(
                    "parent {} contains disconnected selected arc {}",
                    parent.id, arc));
            }
        }
    }
    return errors;
}

auto extract_solution(const UnifiedGraph& graph, const Prepared& prepared,
                      const ModelResult& model,
                      const SatRoutingResult& sat) -> SatRoutingResult {
    auto out = sat;
    out.paths.erase(
        std::remove_if(out.paths.begin(), out.paths.end(),
                       [&](const auto& path) {
                           return prepared.selected_net_ids.contains(
                               path.net_id);
                       }),
        out.paths.end());
    for (const auto& parent : prepared.parents) {
        auto predecessor = std::map<int, int>{};
        auto visited = std::set<int>{parent.root};
        auto queue = std::queue<int>{};
        queue.push(parent.root);
        while (!queue.empty()) {
            const int node = queue.front();
            queue.pop();
            for (const int arc :
                 graph.out_arc_ids[static_cast<std::size_t>(node)]) {
                if (!model.parents[parent.id].arcs.contains(arc))
                    continue;
                const int next = graph.arcs[static_cast<std::size_t>(arc)].v;
                if (visited.insert(next).second) {
                    predecessor.emplace(next, arc);
                    queue.push(next);
                }
            }
        }
        for (std::size_t index = 0; index < parent.sinks.size(); ++index) {
            const int sink = parent.sinks[index];
            auto path = std::Vector<int>{sink};
            for (int node = sink; node != parent.root;) {
                if (!predecessor.contains(node)) {
                    throw std::logic_error(std::format(
                        "V20 ILP extraction cannot reach sink {}", sink));
                }
                const int arc = predecessor.at(node);
                node = graph.arcs[static_cast<std::size_t>(arc)].u;
                path.push_back(node);
            }
            std::reverse(path.begin(), path.end());
            out.paths.push_back(SourceSinkPairPath{
                parent.net_id, parent.source_indices[index],
                parent.demand_ids[index], -1, std::move(path)});
        }
    }
    for (const auto& [group, straight] : model.modes) {
        out.vline_mode_straight_by_group[static_cast<std::size_t>(group)] =
            straight;
    }
    auto switches = std::set<int>{};
    for (const auto& path : out.paths) {
        for (const int arc : path_arcs(graph, path.node_path)) {
            const int physical_switch =
                graph.arcs[static_cast<std::size_t>(arc)].physical_switch_id;
            if (physical_switch >= 0)
                switches.insert(physical_switch);
        }
    }
    out.used_tob_switch_ids.assign(switches.begin(), switches.end());
    out.total_wirelength = total_wirelength(graph, out);
    return out;
}

auto route_validation_errors(const UnifiedGraph& graph,
                             const std::Vector<RoutingNet>& nets,
                             const std::Vector<UnifiedSatNetScope>& scopes,
                             const SatRoutingResult& baseline,
                             const SatRoutingResult& result,
                             const std::set<std::size_t>& selected,
                             const double selected_objective)
    -> std::Vector<std::String> {
    auto errors = std::Vector<std::String>{};
    auto counts = std::map<std::pair<std::size_t, std::size_t>, int>{};
    auto node_owner = std::map<int, std::pair<std::size_t, std::size_t>>{};
    auto switch_owner = std::map<int, std::pair<std::size_t, std::size_t>>{};
    auto switch_exemplar = std::map<int, const UnifiedArc*>{};
    auto sync_lengths = std::map<std::size_t, std::set<std::size_t>>{};
    for (const auto& path : result.paths) {
        ++counts[{path.net_id, path.demand_id}];
        const auto* net = net_for(nets, path.net_id);
        const auto* scope = scope_for(scopes, path.net_id);
        if (net == nullptr || scope == nullptr) {
            errors.push_back("path has no net/scope metadata");
            continue;
        }
        const auto* demand = demand_for(*net, path.demand_id);
        if (demand == nullptr || path.source_index >= net->sources.size()) {
            errors.push_back("path has invalid demand/source metadata");
            continue;
        }
        const int source =
            resolve_graph_node(graph, net->sources[path.source_index]);
        const int sink = resolve_graph_node(graph, demand->sink);
        if (path.node_path.empty() || path.node_path.front() != source ||
            path.node_path.back() != sink) {
            errors.push_back(
                std::format("net {} demand {} has invalid endpoints",
                            path.net_id, path.demand_id));
            continue;
        }
        const auto owner = std::pair{
            path.net_id, net->is_sync_bus ? path.demand_id : std::size_t{0}};
        std::uint16_t unit_mask = 0xffff;
        if (selected.contains(path.net_id)) {
            unit_mask =
                unit_bit(graph.nodes[static_cast<std::size_t>(source)].unit);
        }
        for (const int node : path.node_path) {
            if (!is_physical_node(graph, node)) {
                errors.push_back("refined path contains invalid/virtual node");
                continue;
            }
            if (static_cast<std::size_t>(node) >= scope->node_offset.size() ||
                scope->node_offset[static_cast<std::size_t>(node)] < 0) {
                errors.push_back(std::format(
                    "net {} leaves final SAT node scope", path.net_id));
            }
            if (!node_unit_eligible(graph.nodes[static_cast<std::size_t>(node)],
                                    unit_mask)) {
                errors.push_back(std::format("net {} leaves its fixed COBUnit",
                                             path.net_id));
            }
            const auto [it, inserted] = node_owner.emplace(node, owner);
            if (!inserted && it->second != owner) {
                errors.push_back(
                    std::format("physical node {} has multiple owners", node));
            }
        }
        for (const int arc : path_arcs(graph, path.node_path)) {
            if (static_cast<std::size_t>(arc) >= scope->arc_offset.size() ||
                scope->arc_offset[static_cast<std::size_t>(arc)] < 0) {
                errors.push_back(std::format(
                    "net {} leaves final SAT arc scope", path.net_id));
            }
            const auto& value = graph.arcs[static_cast<std::size_t>(arc)];
            if (!arc_unit_eligible(graph, value, unit_mask)) {
                errors.push_back(std::format(
                    "net {} uses an arc outside fixed COBUnit", path.net_id));
            }
            if (value.physical_switch_id >= 0) {
                const auto [it, inserted] =
                    switch_owner.emplace(value.physical_switch_id, owner);
                if (!inserted && it->second != owner) {
                    errors.push_back(
                        std::format("physical switch {} has multiple owners",
                                    value.physical_switch_id));
                }
                switch_exemplar.try_emplace(value.physical_switch_id, &value);
            }
            if (value.physical_switch_kind == PhysicalSwitchKind::VLineTrack &&
                value.mode_group_id >= 0) {
                const auto mode = result.vline_mode_straight_by_group.find(
                    static_cast<std::size_t>(value.mode_group_id));
                if (mode == result.vline_mode_straight_by_group.end() ||
                    (value.is_vline_track_straight && !mode->second) ||
                    (value.is_vline_track_swap && mode->second)) {
                    errors.push_back(
                        std::format("mode group {} conflicts with route",
                                    value.mode_group_id));
                }
            }
        }
        if (net->is_sync_bus) {
            sync_lengths[net->net_id].insert(
                path_wirelength(graph, path.node_path));
        }
    }
    for (const auto& net : nets) {
        for (const auto& demand : net.demands) {
            if (counts[{net.net_id, demand.demand_id}] != 1) {
                errors.push_back(std::format(
                    "net {} demand {} does not have exactly one path",
                    net.net_id, demand.demand_id));
            }
        }
    }
    auto baseline_by_key =
        std::map<std::tuple<std::size_t, std::size_t, std::size_t>,
                 const SourceSinkPairPath*>{};
    for (const auto& path : baseline.paths) {
        baseline_by_key.emplace(
            std::tuple{path.net_id, path.demand_id, path.source_index}, &path);
    }
    for (const auto& path : result.paths) {
        if (selected.contains(path.net_id))
            continue;
        const auto key =
            std::tuple{path.net_id, path.demand_id, path.source_index};
        const auto it = baseline_by_key.find(key);
        if (it == baseline_by_key.end() ||
            it->second->physical_source_node != path.physical_source_node ||
            it->second->node_path != path.node_path) {
            errors.push_back(std::format(
                "locked net {} changed during refinement", path.net_id));
        }
    }
    auto matching = std::map<std::pair<int, int>, int>{};
    for (const auto& [physical_switch, arc] : switch_exemplar) {
        (void)physical_switch;
        const auto u = graph.nodes[static_cast<std::size_t>(arc->u)].kind;
        const auto v = graph.nodes[static_cast<std::size_t>(arc->v)].kind;
        if (u == UnifiedNodeKind::Bump && v == UnifiedNodeKind::HLine) {
            ++matching[{0, arc->u}];
            ++matching[{1, arc->v}];
        } else if (u == UnifiedNodeKind::HLine && v == UnifiedNodeKind::Bump) {
            ++matching[{0, arc->v}];
            ++matching[{1, arc->u}];
        } else if (u == UnifiedNodeKind::HLine && v == UnifiedNodeKind::VLine) {
            ++matching[{2, arc->u}];
            ++matching[{3, arc->v}];
        } else if (u == UnifiedNodeKind::VLine && v == UnifiedNodeKind::HLine) {
            ++matching[{2, arc->v}];
            ++matching[{3, arc->u}];
        }
    }
    for (const auto& [key, count] : matching) {
        if (count > 1)
            errors.push_back(std::format(
                "partial matching endpoint ({},{}) uses {} switches", key.first,
                key.second, count));
    }
    for (const auto& [net_id, lengths] : sync_lengths) {
        if (lengths.size() > 1)
            errors.push_back(
                std::format("Sync net {} lost equal length", net_id));
    }
    const auto reported_switches = std::set<int>(
        result.used_tob_switch_ids.begin(), result.used_tob_switch_ids.end());
    auto actual_switches = std::set<int>{};
    for (const auto& [physical_switch, _] : switch_owner)
        actual_switches.insert(physical_switch);
    if (reported_switches != actual_switches)
        errors.push_back("reported switch set is inconsistent");
    if (total_wirelength(graph, result) != result.total_wirelength) {
        errors.push_back("reported wirelength is inconsistent");
    }
    std::size_t selected_wirelength = 0;
    for (const auto net_id : selected) {
        selected_wirelength += net_wirelength(graph, paths_for(result, net_id));
    }
    if (std::abs(static_cast<double>(selected_wirelength) -
                 selected_objective) > 0.5) {
        errors.push_back(std::format(
            "selected-net wirelength {} differs from ILP y objective {:.3f}",
            selected_wirelength, selected_objective));
    }
    return errors;
}

auto stamp_stats(SatRoutingResult& out, const ModelStats& stats) -> void {
    out.post_sat_ilp_build_ms = stats.build_ms;
    out.post_sat_ilp_solve_ms = stats.solve_ms;
    out.post_sat_ilp_variables =
        stats.f_vars + stats.x_vars + stats.y_vars + stats.mode_vars;
    out.post_sat_ilp_constraints = stats.constraints;
    out.post_sat_ilp_objective = stats.objective;
    out.post_sat_ilp_bound = stats.bound;
    out.post_sat_ilp_gap = stats.gap;
}

} // namespace

auto is_post_sat_ilp_target(const RoutingNet& net) -> bool {
    if (!net.post_sat_ilp_target || net.is_sync_bus ||
        net.kind != RoutingNetKind::Tnet || net.sources.empty() ||
        net.demands.empty()) {
        return false;
    }
    std::optional<std::size_t> common_source;
    for (const auto& demand : net.demands) {
        if (demand.candidate_source_indices.size() != 1 ||
            demand.sink.kind != GraphNodeRef::Kind::Bump) {
            return false;
        }
        const auto source = demand.candidate_source_indices.front();
        if (source >= net.sources.size() ||
            net.sources[source].kind != GraphNodeRef::Kind::Track) {
            return false;
        }
        if (common_source.has_value() && common_source.value() != source) {
            return false;
        }
        common_source = source;
    }
    return true;
}

auto optimize_post_sat_routes(const UnifiedGraph& graph,
                              const std::Vector<RoutingNet>& nets,
                              const std::Vector<UnifiedSatNetScope>& scopes,
                              const SatRoutingResult& sat_result,
                              const PostSatIlpOptions& options)
    -> SatRoutingResult {
    auto fallback = sat_result;
    fallback.post_sat_ilp_attempted = true;
    fallback.post_sat_ilp_baseline_wirelength = sat_result.total_wirelength;
    fallback.post_sat_ilp_wirelength = sat_result.total_wirelength;
    const auto total_begin = std::chrono::steady_clock::now();
    const auto finish = [&](SatRoutingResult& result) {
        result.post_sat_ilp_total_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - total_begin)
                .count();
    };
    debug::info("========== V20 post-SAT HiGHS ILP refinement ==========");
    try {
        auto [prepared, locked] = prepare_problem(
            graph, nets, scopes, sat_result, options.segment_bbox_pad);
        fallback.post_sat_ilp_parents = prepared.parents.size();
        fallback.post_sat_ilp_segments = prepared.segments.size();
        if (prepared.parents.empty()) {
            fallback.post_sat_ilp_status = "SKIPPED_NO_TARGETS";
            finish(fallback);
            debug::info(
                "V20 post-SAT ILP skipped: no fixed Track-to-Bump target nets");
            return fallback;
        }
        debug::info_fmt(
            "V20 post-SAT ILP prepare: selected_nets={} parents={} segments={} "
            "segment_node_slots={} segment_arc_slots={} fixed_nets={} "
            "locked_nodes={} locked_switches={} bbox_pad={}",
            prepared.selected_net_ids.size(), prepared.parents.size(),
            prepared.segments.size(),
            std::accumulate(prepared.segments.begin(), prepared.segments.end(),
                            std::size_t{0},
                            [](const std::size_t sum, const Segment& segment) {
                                return sum + segment.nodes.size();
                            }),
            std::accumulate(prepared.segments.begin(), prepared.segments.end(),
                            std::size_t{0},
                            [](const std::size_t sum, const Segment& segment) {
                                return sum + segment.arcs.size();
                            }),
            locked.net_ids.size(),
            std::count(locked.nodes.begin(), locked.nodes.end(), true),
            locked.switches.size(), options.segment_bbox_pad);
        const auto model =
            solve_model(graph, prepared, locked, sat_result, options);
        stamp_stats(fallback, model.stats);
        if (!model.ok) {
            fallback.post_sat_ilp_status = model.status;
            finish(fallback);
            debug::warning_fmt(
                "V20 post-SAT ILP fallback: status={} feasible_incumbent=false",
                model.status);
            return fallback;
        }
        auto model_errors = validate_model(graph, prepared, model);
        if (!model_errors.empty()) {
            fallback.post_sat_ilp_status = "MODEL_VALIDATION_FAILED";
            finish(fallback);
            for (const auto& error : model_errors)
                debug::warning_fmt("  V20 model validation: {}", error);
            return fallback;
        }
        auto candidate = extract_solution(graph, prepared, model, sat_result);
        auto route_errors = route_validation_errors(
            graph, nets, scopes, sat_result, candidate,
            prepared.selected_net_ids, model.stats.objective);
        if (!route_errors.empty()) {
            fallback.post_sat_ilp_status = "ROUTE_VALIDATION_FAILED";
            finish(fallback);
            for (const auto& error : route_errors)
                debug::warning_fmt("  V20 route validation: {}", error);
            return fallback;
        }
        if (candidate.total_wirelength > sat_result.total_wirelength) {
            fallback.post_sat_ilp_status = "DEGRADED_INCUMBENT_REJECTED";
            finish(fallback);
            debug::warning_fmt(
                "V20 post-SAT ILP fallback: detailed wirelength {} -> {}",
                sat_result.total_wirelength, candidate.total_wirelength);
            return fallback;
        }
        candidate.post_sat_ilp_attempted = true;
        candidate.post_sat_ilp_accepted = true;
        candidate.post_sat_ilp_status = model.status;
        candidate.post_sat_ilp_baseline_wirelength =
            sat_result.total_wirelength;
        candidate.post_sat_ilp_wirelength = candidate.total_wirelength;
        candidate.post_sat_ilp_parents = prepared.parents.size();
        candidate.post_sat_ilp_segments = prepared.segments.size();
        stamp_stats(candidate, model.stats);
        finish(candidate);
        for (const auto& parent : prepared.parents) {
            const auto before =
                net_wirelength(graph, paths_for(sat_result, parent.net_id));
            const auto after =
                net_wirelength(graph, paths_for(candidate, parent.net_id));
            debug::info_fmt(
                "V20 post-SAT ILP net: net={} wirelength={}->{} improvement={}",
                parent.net_id, before, after,
                static_cast<long long>(before) - static_cast<long long>(after));
        }
        debug::info_fmt(
            "V20 post-SAT ILP accepted: status={} wirelength={} -> {} "
            "improvement={} objective={:.0f} bound={:.3f} gap={:.6f} "
            "build_ms={} "
            "solve_ms={} total_ms={}",
            model.status, sat_result.total_wirelength,
            candidate.total_wirelength,
            sat_result.total_wirelength - candidate.total_wirelength,
            model.stats.objective, model.stats.bound, model.stats.gap,
            candidate.post_sat_ilp_build_ms, candidate.post_sat_ilp_solve_ms,
            candidate.post_sat_ilp_total_ms);
        return candidate;
    } catch (const std::exception& error) {
        fallback.post_sat_ilp_status =
            std::format("EXCEPTION: {}", error.what());
        finish(fallback);
        debug::warning_fmt(
            "V20 post-SAT ILP exception: {} fallback_to_SAT=true",
            error.what());
        return fallback;
    }
}

} // namespace PR_tool
