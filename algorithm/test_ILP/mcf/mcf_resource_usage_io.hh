#pragma once

#include "mcf/cob_mcf_router.hh"
#include "mcf/mcf_graph.hh"

#include <std/string.hh>

#include <array>
#include <filesystem>
#include <map>
#include <mutex>
#include <optional>
#include <string_view>

namespace PR_tool {

inline constexpr auto kMcfResourceUsageDir = "resource-usage";
inline constexpr auto kMcfResourceUsageFileFmt = "unit{}.txt";
inline constexpr int kMcfHardwareSwitchesPerCobUnit = 48;
inline constexpr int kMcfChannelsPerCobLink = 8;

struct McfHChannelKey {
    int r{0};
    int c{0};

    auto operator<=>(const McfHChannelKey&) const = default;
};

struct McfVChannelKey {
    int r{0};
    int c{0};

    auto operator<=>(const McfVChannelKey&) const = default;
};

struct McfResourceCatalog {
    int rows{0};
    int cols{0};
    std::array<std::array<std::array<int, 32>, 32>, 16> switch_modeled {};
    std::array<std::map<McfHChannelKey, int>, 16> h_channel_total {};
    std::array<std::map<McfVChannelKey, int>, 16> v_channel_total {};
};

struct McfResourceUsage {
    int rows{0};
    int cols{0};
    std::array<std::array<std::array<int, 32>, 32>, 16> switches_used {};
    std::array<std::map<McfHChannelKey, int>, 16> h_channels_used {};
    std::array<std::map<McfVChannelKey, int>, 16> v_channels_used {};
};

auto build_mcf_resource_catalog(const McfGlobalGraph& graph) -> McfResourceCatalog;

auto aggregate_mcf_resource_usage(
    const McfGlobalGraph& graph,
    const std::map<std::pair<int, int>, std::size_t>& arc_index,
    const std::array<std::Vector<McfPathInfo>, 16>& paths_by_unit
) -> McfResourceUsage;

auto aggregate_mcf_resource_usage_for_unit(
    const McfGlobalGraph& graph,
    const std::map<std::pair<int, int>, std::size_t>& arc_index,
    std::size_t unit,
    const std::Vector<McfPathInfo>& paths
) -> McfResourceUsage;

auto format_unit_resource_section(
    std::string_view phase_tag,
    std::size_t unit,
    const McfResourceCatalog& catalog,
    const McfResourceUsage& usage,
    bool ok
) -> std::String;

auto format_unit_resource_file_section(
    std::string_view phase_tag,
    std::size_t unit,
    const McfResourceCatalog& catalog,
    const McfResourceUsage& usage,
    bool ok
) -> std::String;

class McfResourceUsageSink {
public:
    McfResourceUsageSink(McfResourceCatalog catalog, std::filesystem::path output_dir);

    void prepare_output_dir();

    auto write_pre_route(std::size_t unit, const McfResourceUsage& usage, bool ok) -> std::String;

    void write_complete(
        std::size_t unit,
        const std::String& pre_route_section,
        const McfResourceUsage& post_usage,
        bool ok
    );

    void write_empty(std::size_t unit);

private:
    void write_file(std::size_t unit, const std::String& content);
    void log_index(std::size_t unit, std::string_view phase);

    McfResourceCatalog catalog_;
    std::filesystem::path output_dir_;
    std::mutex mutex_;
    bool dir_prepared_{false};
};

} // namespace PR_tool
