#pragma once
#include <algo/router/rrr/path_adapter.hh>
#include <parse/reader/config/config.hh>
#include <filesystem>

namespace PR_tool::test {
// Independent register decoder: never calls Writer encoding or live occupancy.
auto check_routed_controlbits(hardware::Interposer* interposer,
    const rrr::UnifiedGraph& graph, const std::vector<rrr::RoutingNet>& nets,
    const std::vector<circuit::Net*>& raw, const parse::RegisterMapConfig& map,
    const std::filesystem::path& directory, bool sparse,
    const rrr::NetPaths* expected = nullptr) -> void;
}
