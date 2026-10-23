#pragma once

#include "hardware_graph.hh"
#include <circuit/net/net.hh>

namespace PR_tool::rrr {
using NetPaths = std::Vector<std::Vector<std::Vector<int>>>;

auto import_paths(hardware::Interposer* interposer, const UnifiedGraph& graph,
                  const std::Vector<RoutingNet>& nets,
                  const std::Vector<circuit::Net*>& raw) -> NetPaths;
auto commit_paths(hardware::Interposer* interposer, const UnifiedGraph& graph,
                  const std::Vector<RoutingNet>& nets, const NetPaths& paths,
                  const std::Vector<circuit::Net*>& raw) -> void;
}
