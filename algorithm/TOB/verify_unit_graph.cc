#include "direct_ilp/undirected_graph.hh"

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <stdexcept>
#include <vector>

namespace PR_tool {
namespace {

auto require(bool condition, const char* message) -> void {
    if (!condition) throw std::runtime_error(message);
}

auto other(const DirectEdge& edge, int node) -> int {
    return edge.u == node ? edge.v : edge.u;
}

auto kind(const UnifiedGraph& graph, int node) -> UnifiedNodeKind {
    return graph.nodes[static_cast<std::size_t>(node)].kind;
}

auto endpoint_of_kind(const UnifiedGraph& graph, const DirectEdge& edge,
                      UnifiedNodeKind wanted) -> int {
    if (kind(graph, edge.u) == wanted) return edge.u;
    if (kind(graph, edge.v) == wanted) return edge.v;
    throw std::runtime_error("switch edge has an unexpected endpoint kind");
}

auto compatible(const UnifiedGraph& graph, const DirectGraph& direct,
                const std::vector<int>& selected, int candidate) -> bool {
    const auto& edge = direct.edges[static_cast<std::size_t>(candidate)];
    for (int id : selected) {
        const auto& prior = direct.edges[static_cast<std::size_t>(id)];
        if (edge.switch_kind == PhysicalSwitchKind::BumpH
            && prior.switch_kind == PhysicalSwitchKind::BumpH) {
            if (endpoint_of_kind(graph, edge, UnifiedNodeKind::Bump)
                    == endpoint_of_kind(graph, prior, UnifiedNodeKind::Bump)
                || endpoint_of_kind(graph, edge, UnifiedNodeKind::HLine)
                    == endpoint_of_kind(graph, prior, UnifiedNodeKind::HLine)) return false;
        }
        if (edge.switch_kind == PhysicalSwitchKind::HLineVLine
            && prior.switch_kind == PhysicalSwitchKind::HLineVLine) {
            if (endpoint_of_kind(graph, edge, UnifiedNodeKind::HLine)
                    == endpoint_of_kind(graph, prior, UnifiedNodeKind::HLine)
                || endpoint_of_kind(graph, edge, UnifiedNodeKind::VLine)
                    == endpoint_of_kind(graph, prior, UnifiedNodeKind::VLine)) return false;
        }
        if (edge.mode_group >= 0 && edge.mode_group == prior.mode_group
            && edge.straight != prior.straight) return false;
    }
    return true;
}

auto reaches_another_track(const UnifiedGraph& graph, const DirectGraph& direct,
                           int start, int node, std::size_t tob,
                           std::set<int>& visited, std::vector<int>& selected) -> bool {
    if (node != start && kind(graph, node) == UnifiedNodeKind::Track) return true;
    for (int id : direct.incident[static_cast<std::size_t>(node)]) {
        const auto& edge = direct.edges[static_cast<std::size_t>(id)];
        if (edge.switch_kind == PhysicalSwitchKind::None
            || edge.virtual_source || !compatible(graph, direct, selected, id)) continue;
        const int next = other(edge, node);
        if (visited.contains(next)) continue;
        if (kind(graph, next) != UnifiedNodeKind::Track
            && graph.nodes[static_cast<std::size_t>(next)].tob != tob) continue;
        visited.insert(next);
        selected.push_back(id);
        if (reaches_another_track(graph, direct, start, next, tob, visited, selected)) return true;
        selected.pop_back();
        visited.erase(next);
    }
    return false;
}

struct Result {
    std::size_t track_nodes{};
    std::size_t track_components{};
    std::array<std::size_t, 16> components_by_unit{};
    std::array<std::size_t, 16> tracks_by_unit{};
    std::size_t bumps{};
    std::size_t minimum_access_units{16};
    std::size_t maximum_access_units{};
    std::size_t minimum_access_chains{static_cast<std::size_t>(-1)};
    std::size_t maximum_access_chains{};
    std::size_t raw_cross_unit_vline_paths{};
    std::size_t checked_tob_track_starts{};
};

auto verify(const UnifiedGraph& graph, const DirectGraph& direct) -> Result {
    auto result = Result{};
    auto switch_ids = std::set<int>{};
    for (const auto& edge : direct.edges) {
        require(!edge.virtual_source, "verification graph contains a PN virtual edge");
        const auto a = kind(graph, edge.u), b = kind(graph, edge.v);
        if (edge.switch_kind == PhysicalSwitchKind::None) {
            require(a == UnifiedNodeKind::Track && b == UnifiedNodeKind::Track,
                    "non-switch edge leaves a Track unit");
            continue;
        }
        const auto pair_is = [&](UnifiedNodeKind x, UnifiedNodeKind y) {
            return (a == x && b == y) || (a == y && b == x);
        };
        require((edge.switch_kind == PhysicalSwitchKind::BumpH
                    && pair_is(UnifiedNodeKind::Bump, UnifiedNodeKind::HLine))
                || (edge.switch_kind == PhysicalSwitchKind::HLineVLine
                    && pair_is(UnifiedNodeKind::HLine, UnifiedNodeKind::VLine))
                || (edge.switch_kind == PhysicalSwitchKind::VLineTrack
                    && pair_is(UnifiedNodeKind::VLine, UnifiedNodeKind::Track)),
                "TOB switch has an unexpected pair of node kinds");
        require(edge.switch_id >= 0 && switch_ids.insert(edge.switch_id).second,
                "TOB switch ID is missing or repeated");
        if (a != UnifiedNodeKind::Track && b != UnifiedNodeKind::Track)
            require(graph.nodes[static_cast<std::size_t>(edge.u)].tob
                        == graph.nodes[static_cast<std::size_t>(edge.v)].tob,
                    "TOB switch connects different TOBs");
    }
    auto seen = std::vector<bool>(graph.nodes.size(), false);
    for (int start = 0; start < static_cast<int>(graph.nodes.size()); ++start) {
        if (kind(graph, start) != UnifiedNodeKind::Track || seen[start]) continue;
        const auto unit = graph.nodes[static_cast<std::size_t>(start)].unit;
        require(unit < 16, "Track has an invalid unit");
        ++result.track_components;
        ++result.components_by_unit[unit];
        auto pending = std::queue<int>{};
        pending.push(start);
        seen[start] = true;
        while (!pending.empty()) {
            const int node = pending.front(); pending.pop();
            ++result.track_nodes;
            ++result.tracks_by_unit[unit];
            for (int id : direct.incident[static_cast<std::size_t>(node)]) {
                const auto& edge = direct.edges[static_cast<std::size_t>(id)];
                const int next = other(edge, node);
                if (kind(graph, next) != UnifiedNodeKind::Track) continue;
                require(edge.switch_kind == PhysicalSwitchKind::None,
                        "Track-Track edge is marked as a TOB switch");
                require(graph.nodes[static_cast<std::size_t>(next)].unit == unit,
                        "Track-Track edge crosses units");
                if (!seen[next]) { seen[next] = true; pending.push(next); }
            }
        }
    }
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        if (kind(graph, node) == UnifiedNodeKind::Bump) {
            ++result.bumps;
            auto units = std::set<std::size_t>{};
            auto chains_by_unit = std::array<int, 16>{};
            std::size_t chains = 0;
            for (int bh_id : direct.incident[static_cast<std::size_t>(node)]) {
                const auto& bh = direct.edges[static_cast<std::size_t>(bh_id)];
                require(bh.switch_kind == PhysicalSwitchKind::BumpH,
                        "Bump has an unexpected physical edge");
                const int h = other(bh, node);
                for (int hv_id : direct.incident[static_cast<std::size_t>(h)]) {
                    const auto& hv = direct.edges[static_cast<std::size_t>(hv_id)];
                    if (hv.switch_kind != PhysicalSwitchKind::HLineVLine) continue;
                    const int v = other(hv, h);
                    for (int vt_id : direct.incident[static_cast<std::size_t>(v)]) {
                        const auto& vt = direct.edges[static_cast<std::size_t>(vt_id)];
                        if (vt.switch_kind != PhysicalSwitchKind::VLineTrack) continue;
                        const int track = other(vt, v);
                        require(kind(graph, track) == UnifiedNodeKind::Track,
                                "VLine-Track switch does not reach a Track");
                        const auto unit = graph.nodes[static_cast<std::size_t>(track)].unit;
                        require(unit < 16, "Bump access reaches an invalid unit");
                        units.insert(unit);
                        ++chains_by_unit[unit];
                        ++chains;
                    }
                }
            }
            result.minimum_access_units = std::min(result.minimum_access_units, units.size());
            result.maximum_access_units = std::max(result.maximum_access_units, units.size());
            result.minimum_access_chains = std::min(result.minimum_access_chains, chains);
            result.maximum_access_chains = std::max(result.maximum_access_chains, chains);
            require(units.size() == 16, "a Bump cannot access all 16 units");
            require(std::all_of(chains_by_unit.begin(), chains_by_unit.end(),
                                [](int count) { return count == 8; }),
                    "Bump access choices are not evenly distributed over units");
        }
        if (kind(graph, node) != UnifiedNodeKind::VLine) continue;
        auto track_edges = std::vector<int>{};
        for (int id : direct.incident[static_cast<std::size_t>(node)]) {
            const auto& edge = direct.edges[static_cast<std::size_t>(id)];
            if (edge.switch_kind == PhysicalSwitchKind::VLineTrack) track_edges.push_back(id);
        }
        require(track_edges.size() == 2, "VLine does not have two Track choices");
        const auto& a = direct.edges[static_cast<std::size_t>(track_edges[0])];
        const auto& b = direct.edges[static_cast<std::size_t>(track_edges[1])];
        require(a.mode_group >= 0 && a.mode_group == b.mode_group
                    && a.straight != b.straight,
                "VLine Track choices are not opposite modes");
        if (graph.nodes[static_cast<std::size_t>(other(a, node))].unit
            != graph.nodes[static_cast<std::size_t>(other(b, node))].unit)
            ++result.raw_cross_unit_vline_paths;
    }
    for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
        if (kind(graph, node) != UnifiedNodeKind::Track) continue;
        auto adjacent_tobs = std::set<std::size_t>{};
        for (int id : direct.incident[static_cast<std::size_t>(node)]) {
            const auto& edge = direct.edges[static_cast<std::size_t>(id)];
            if (edge.switch_kind != PhysicalSwitchKind::VLineTrack) continue;
            adjacent_tobs.insert(graph.nodes[static_cast<std::size_t>(other(edge, node))].tob);
        }
        for (std::size_t tob : adjacent_tobs) {
            ++result.checked_tob_track_starts;
            auto visited = std::set<int>{node};
            auto selected = std::vector<int>{};
            require(!reaches_another_track(graph, direct, node, node, tob, visited, selected),
                    "a feasible Track-to-Track path crosses a TOB");
        }
    }
    return result;
}

auto write_result(const Result& r, const UnifiedGraph& graph,
                  const DirectGraph& direct, const char* path) -> void {
    auto file = std::ofstream{path};
    require(file.good(), "cannot open result file");
    file << "{\n  \"status\": \"PASS\",\n"
         << "  \"cob_width\": " << graph.cols << ",\n"
         << "  \"cob_height\": " << graph.rows << ",\n"
         << "  \"tob_count\": " << hardware::Interposer::TOB_SIZE << ",\n"
         << "  \"graph_nodes\": " << graph.nodes.size() << ",\n"
         << "  \"undirected_edges\": " << direct.edges.size() << ",\n"
         << "  \"track_nodes\": " << r.track_nodes << ",\n"
         << "  \"track_components\": " << r.track_components << ",\n"
         << "  \"components_by_unit\": [";
    for (std::size_t i = 0; i < 16; ++i) file << (i ? ", " : "") << r.components_by_unit[i];
    file << "],\n  \"tracks_by_unit\": [";
    for (std::size_t i = 0; i < 16; ++i) file << (i ? ", " : "") << r.tracks_by_unit[i];
    file << "],\n  \"bumps\": " << r.bumps << ",\n"
         << "  \"bump_access_units_min\": " << r.minimum_access_units << ",\n"
         << "  \"bump_access_units_max\": " << r.maximum_access_units << ",\n"
         << "  \"bump_access_chains_min\": " << r.minimum_access_chains << ",\n"
         << "  \"bump_access_chains_max\": " << r.maximum_access_chains << ",\n"
         << "  \"raw_cross_unit_vline_paths\": " << r.raw_cross_unit_vline_paths << ",\n"
         << "  \"checked_tob_track_starts\": " << r.checked_tob_track_starts << ",\n"
         << "  \"feasible_tob_track_to_track_paths\": 0\n}\n";
    require(file.good(), "cannot write result file");
}

} // namespace
} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    try {
        if (argc != 2) throw std::invalid_argument("usage: tob_unit_graph_verify RESULT.json");
        const auto graph = PR_tool::build_unified_graph(nullptr, {});
        const auto direct = PR_tool::build_direct_graph(graph);
        const auto result = PR_tool::verify(graph, direct);
        PR_tool::write_result(result, graph, direct, argv[1]);
        std::cout << "PASS: " << result.track_components << " Track components; "
                  << result.bumps << " Bumps each access 16 units; "
                  << result.checked_tob_track_starts
                  << " TOB Track starts have no feasible Track-to-Track path\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "TOB unit graph verification failed: " << error.what() << '\n';
        return 1;
    }
}
