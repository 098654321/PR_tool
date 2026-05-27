#pragma once

#include "common/ilp_types.hh"

#include <std/file.hh>

namespace PR_tool {

auto build_ilp_warm_start_from_maze(
    const std::FilePath& config_path,
    const std::Vector<Net_cost_record>& records
) -> TobIlpWarmStart;

} // namespace PR_tool
