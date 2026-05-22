#pragma once

#include "highs.hh"

#include <std/collection.hh>
#include <std/memory.hh>

namespace PR_tool::circuit {
class Net;
}

namespace PR_tool::hardware {
class Interposer;
}

namespace PR_tool {

auto apply_tob_ilp_result_to_interposer(
    hardware::Interposer* interposer,
    const TobIlpResult& result
) -> void;

auto route_track_to_bumps_nets_post_mcf(
    hardware::Interposer* interposer,
    const std::Vector<std::Rc<circuit::Net>>& all_nets,
    const std::Vector<std::Rc<circuit::Net>>& track_to_bumps_nets
) -> void;

} // namespace PR_tool
