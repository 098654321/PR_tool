#pragma once

#include "ilp_allocation/highs.hh"

namespace PR_tool::hardware {
class Interposer;
}

namespace PR_tool {

auto apply_tob_ilp_result_to_interposer(
    hardware::Interposer* interposer,
    const TobIlpResult& result
) -> void;

} // namespace PR_tool
