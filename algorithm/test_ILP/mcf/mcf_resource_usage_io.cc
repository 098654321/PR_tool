#include "mcf/mcf_resource_usage_io.hh"

#include <debug/debug.hh>

#include <fstream>
#include <format>
#include <set>

namespace PR_tool {

namespace {

using Arc = McfArc;
using NodeMeta = McfNodeMeta;

auto normalized_edge_key(int u, int v) -> std::pair<int, int> {
    if (u > v) {
        std::swap(u, v);
    }
    return {u, v};
}

auto arc_cob_row_col(const Arc& arc, const int cols) -> std::pair<int, int> {
    return {arc.cob / cols, arc.cob % cols};
}

auto h_channel_key_from_node(const NodeMeta& node, const int cols) -> std::optional<McfHChannelKey> {
    if (node.is_virtual || node.track_dir != 0) {
        return std::nullopt;
    }
    if (node.track_col <= 0 || node.track_col >= cols) {
        return std::nullopt;
    }
    return McfHChannelKey {node.track_row, node.track_col - 1};
}

auto v_channel_key_from_node(const NodeMeta& node, const int rows) -> std::optional<McfVChannelKey> {
    if (node.is_virtual || node.track_dir != 1) {
        return std::nullopt;
    }
    if (node.track_row <= 0 || node.track_row >= rows) {
        return std::nullopt;
    }
    return McfVChannelKey {node.track_row - 1, node.track_col};
}

auto format_phase_marker(std::string_view phase_tag) -> std::String {
    return std::format("=== {} ===", phase_tag);
}

} // namespace

auto build_mcf_resource_catalog(const McfGlobalGraph& graph) -> McfResourceCatalog {
    McfResourceCatalog catalog {};
    catalog.rows = graph.rows;
    catalog.cols = graph.cols;
    auto switch_seen = std::array<std::array<std::array<std::set<std::pair<int, int>>, 32>, 32>, 16> {};

    for (const auto& arc : graph.arcs) {
        if (arc.is_virtual) {
            continue;
        }
        const auto u = static_cast<std::size_t>(arc.unit);
        if (u >= 16) {
            continue;
        }
        const auto [cob_r, cob_c] = arc_cob_row_col(arc, graph.cols);
        if (cob_r < 0 || cob_c < 0 || cob_r >= catalog.rows || cob_c >= catalog.cols) {
            continue;
        }
        const auto edge_key = normalized_edge_key(arc.u, arc.v);
        if (!switch_seen[u][static_cast<std::size_t>(cob_r)][static_cast<std::size_t>(cob_c)].contains(edge_key)) {
            switch_seen[u][static_cast<std::size_t>(cob_r)][static_cast<std::size_t>(cob_c)].insert(edge_key);
            ++catalog.switch_modeled[u][static_cast<std::size_t>(cob_r)][static_cast<std::size_t>(cob_c)];
        }
    }
    return catalog;
}

auto aggregate_mcf_resource_usage(
    const McfGlobalGraph& graph,
    const std::map<std::pair<int, int>, std::size_t>& arc_index,
    const std::array<std::Vector<McfPathInfo>, 16>& paths_by_unit
) -> McfResourceUsage {
    McfResourceUsage usage {};
    usage.rows = graph.rows;
    usage.cols = graph.cols;

    auto seen_switch_edges = std::array<std::set<std::pair<int, int>>, 16> {};
    auto seen_channel_nodes = std::array<std::set<int>, 16> {};

    auto absorb_switch_edge = [&](const std::size_t unit, const int a, const int b) {
        if (unit >= 16) {
            return;
        }
        auto edge = normalized_edge_key(a, b);
        if (seen_switch_edges[unit].contains(edge)) {
            return;
        }
        const auto it = arc_index.find(edge);
        if (it == arc_index.end()) {
            return;
        }
        const auto& arc = graph.arcs[it->second];
        if (arc.unit != unit) {
            return;
        }
        const auto [cob_r, cob_c] = arc_cob_row_col(arc, graph.cols);
        if (cob_r < 0 || cob_c < 0 || cob_r >= usage.rows || cob_c >= usage.cols) {
            return;
        }
        seen_switch_edges[unit].insert(edge);
        ++usage.switches_used[unit][static_cast<std::size_t>(cob_r)][static_cast<std::size_t>(cob_c)];
    };

    auto absorb_channel_node = [&](const std::size_t unit, const int node_id) {
        if (unit >= 16) {
            return;
        }
        if (node_id < 0 || static_cast<std::size_t>(node_id) >= graph.nodes.size()) {
            return;
        }
        if (seen_channel_nodes[unit].contains(node_id)) {
            return;
        }
        const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
        if (node.is_virtual || node.unit != unit) {
            return;
        }
        if (const auto h = h_channel_key_from_node(node, graph.cols)) {
            seen_channel_nodes[unit].insert(node_id);
            ++usage.h_channels_used[unit][*h];
        }
        else if (const auto v = v_channel_key_from_node(node, graph.rows)) {
            seen_channel_nodes[unit].insert(node_id);
            ++usage.v_channels_used[unit][*v];
        }
    };

    for (std::size_t u = 0; u < 16; ++u) {
        for (const auto& info : paths_by_unit[u]) {
            for (const auto& path : info.unit_paths) {
                for (const auto node_id : path) {
                    absorb_channel_node(u, node_id);
                }
                for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                    absorb_switch_edge(u, path[i], path[i + 1]);
                }
            }
        }
    }
    return usage;
}

auto aggregate_mcf_resource_usage_for_unit(
    const McfGlobalGraph& graph,
    const std::map<std::pair<int, int>, std::size_t>& arc_index,
    const std::size_t unit,
    const std::Vector<McfPathInfo>& paths
) -> McfResourceUsage {
    auto paths_by_unit = std::array<std::Vector<McfPathInfo>, 16> {};
    if (unit < 16) {
        paths_by_unit[unit] = paths;
    }
    return aggregate_mcf_resource_usage(graph, arc_index, paths_by_unit);
}

auto format_unit_resource_section(
    const std::string_view phase_tag,
    const std::size_t unit,
    const McfResourceCatalog& catalog,
    const McfResourceUsage& usage,
    const bool ok
) -> std::String {
    auto out = std::String {};
    out += std::format(
        "MCF resource usage ({}, unit={}, ok={})\n",
        phase_tag,
        unit,
        ok ? "true" : "false");
    out += std::format("Unit {}:\n", unit);

    int switch_used_sum = 0;
    int switch_total_sum = 0;
    int channel_used_sum = 0;
    int channel_total_sum = 0;
    int switch_modeled_sum = 0;

    for (int r = 0; r < catalog.rows; ++r) {
        for (int c = 0; c < catalog.cols; ++c) {
            const auto used = usage.switches_used[unit][static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
            const auto modeled = catalog.switch_modeled[unit][static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
            out += std::format("  switches COB({},{})={}/{}\n", r, c, used, kMcfHardwareSwitchesPerCobUnit);
            switch_used_sum += used;
            switch_total_sum += kMcfHardwareSwitchesPerCobUnit;
            switch_modeled_sum += modeled;
        }
    }
    for (int r = 0; r < catalog.rows; ++r) {
        for (int c = 0; c + 1 < catalog.cols; ++c) {
            const McfHChannelKey key {r, c};
            const auto used = usage.h_channels_used[unit].contains(key) ? usage.h_channels_used[unit].at(key) : 0;
            const auto total = kMcfChannelsPerCobLink;
            out += std::format(
                "  channel H COB({},{})-COB({},{})={}/{}\n",
                r,
                c,
                r,
                c + 1,
                used,
                total);
            channel_used_sum += used;
            channel_total_sum += total;
        }
    }
    for (int r = 0; r + 1 < catalog.rows; ++r) {
        for (int c = 0; c < catalog.cols; ++c) {
            const McfVChannelKey key {r, c};
            const auto used = usage.v_channels_used[unit].contains(key) ? usage.v_channels_used[unit].at(key) : 0;
            const auto total = kMcfChannelsPerCobLink;
            out += std::format(
                "  channel V COB({},{})-COB({},{})={}/{}\n",
                r,
                c,
                r + 1,
                c,
                used,
                total);
            channel_used_sum += used;
            channel_total_sum += total;
        }
    }
    out += std::format(
        "  unit_summary switches={}/{} modeled={} channels={}/{}\n",
        switch_used_sum,
        switch_total_sum,
        switch_modeled_sum,
        channel_used_sum,
        channel_total_sum);
    return out;
}

auto format_unit_resource_file_section(
    const std::string_view phase_tag,
    const std::size_t unit,
    const McfResourceCatalog& catalog,
    const McfResourceUsage& usage,
    const bool ok
) -> std::String {
    auto out = format_phase_marker(phase_tag);
    out += '\n';
    out += format_unit_resource_section(phase_tag, unit, catalog, usage, ok);
    return out;
}

McfResourceUsageSink::McfResourceUsageSink(
    McfResourceCatalog catalog,
    std::filesystem::path output_dir
)
    : catalog_(std::move(catalog))
    , output_dir_(std::move(output_dir)) {}

void McfResourceUsageSink::prepare_output_dir() {
    const std::lock_guard lock(mutex_);
    if (dir_prepared_) {
        return;
    }
    std::error_code ec {};
    std::filesystem::remove_all(output_dir_, ec);
    std::filesystem::create_directories(output_dir_, ec);
    if (ec) {
        debug::error_fmt("MCF resource usage: failed to prepare {} ({})", output_dir_.string(), ec.message());
    }
    dir_prepared_ = true;
}

auto McfResourceUsageSink::write_pre_route(
    const std::size_t unit,
    const McfResourceUsage& usage,
    const bool ok
) -> std::String {
    const auto section = format_unit_resource_file_section("pre-route", unit, catalog_, usage, ok);
    write_file(unit, section);
    log_index(unit, "pre-route");
    return section;
}

void McfResourceUsageSink::write_complete(
    const std::size_t unit,
    const std::String& pre_route_section,
    const McfResourceUsage& post_usage,
    const bool ok
) {
    auto content = pre_route_section;
    if (!content.empty() && content.back() != '\n') {
        content += '\n';
    }
    content += format_unit_resource_file_section("post-solve", unit, catalog_, post_usage, ok);
    write_file(unit, content);
    log_index(unit, "post-solve");
}

void McfResourceUsageSink::write_empty(const std::size_t unit) {
    write_file(unit, std::String {});
    log_index(unit, "empty");
}

void McfResourceUsageSink::write_file(const std::size_t unit, const std::String& content) {
    const std::lock_guard lock(mutex_);
    const auto path = output_dir_ / std::format(kMcfResourceUsageFileFmt, unit);
    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        debug::error_fmt("MCF resource usage: failed to open {}", path.string());
        return;
    }
    out << content;
}

void McfResourceUsageSink::log_index(const std::size_t unit, const std::string_view phase) {
    debug::info_fmt(
        "MCF resource usage: wrote {}/{} (phase={})",
        kMcfResourceUsageDir,
        std::format(kMcfResourceUsageFileFmt, unit),
        phase);
}

} // namespace PR_tool
