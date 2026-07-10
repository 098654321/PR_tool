#include "sat/routing_round_diagnostics.hh"

#include "sat/ideal_shortest_wirelength.hh"
#include "sat/routing_path_log.hh"

#include <algorithm>
#include <cmath>
#include <debug/debug.hh>
#include <iomanip>
#include <set>
#include <sstream>

namespace PR_tool {

namespace {

constexpr auto kRoundBanner =
    "************************************************************************************************************************";

auto net_by_id(const std::Vector<RoutingNet>& nets, std::size_t net_id) -> const RoutingNet* {
    const auto it = std::find_if(
        nets.begin(),
        nets.end(),
        [&](const RoutingNet& net) { return net.net_id == net_id; });
    return it == nets.end() ? nullptr : &*it;
}

auto paths_for_net(
    const SatRoutingResult& result,
    std::size_t net_id
) -> std::Vector<const SourceSinkPairPath*> {
    auto out = std::Vector<const SourceSinkPairPath*> {};
    for (const auto& path : result.paths) {
        if (path.net_id == net_id) {
            out.push_back(&path);
        }
    }
    return out;
}

auto format_percent(double actual, double shortest) -> std::String {
    if (shortest <= 0.0) {
        return "n/a";
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << ((actual - shortest) * 100.0 / shortest);
    return stream.str();
}

} // namespace

auto collect_net_stretch_info(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const DelayPrecomputeResult& delays,
    const SatRoutingResult& result
) -> std::Vector<NetStretchInfo> {
    auto entries = std::Vector<NetStretchInfo> {};
    if (!result.ok) {
        return entries;
    }
    for (const auto& net : nets) {
        const auto net_paths = paths_for_net(result, net.net_id);
        if (net_paths.empty()) {
            continue;
        }
        const auto actual = net_wirelength(graph, net_paths);
        const auto shortest = ideal_net_wirelength(interposer, graph, net, delays);
        const auto delta = shortest == 0
            ? 0.0
            : (static_cast<double>(actual) - static_cast<double>(shortest))
                * 100.0 / static_cast<double>(shortest);
        entries.push_back(NetStretchInfo {net.net_id, net.name, actual, shortest, delta});
    }
    std::sort(entries.begin(), entries.end(), [](const NetStretchInfo& lhs, const NetStretchInfo& rhs) {
        if (lhs.delta_percent != rhs.delta_percent) {
            return lhs.delta_percent > rhs.delta_percent;
        }
        return lhs.net_id < rhs.net_id;
    });
    return entries;
}

auto feedback_round_status_name(FeedbackRoundStatus status) -> std::String {
    switch (status) {
        case FeedbackRoundStatus::SatSuccess:
            return "SAT_SUCCESS";
        case FeedbackRoundStatus::UnsatExpand:
            return "UNSAT_EXPAND";
        case FeedbackRoundStatus::UnsatExhausted:
            return "UNSAT_EXHAUSTED";
        case FeedbackRoundStatus::SolverError:
            return "SOLVER_ERROR";
        case FeedbackRoundStatus::MemoryLimit:
            return "MEMORY_LIMIT";
        case FeedbackRoundStatus::MaxRoundsExceeded:
            return "MAX_ROUNDS_EXCEEDED";
    }
    return "UNKNOWN";
}

auto unique_failed_net_ids(const std::Vector<PairKey>& critical) -> std::Vector<std::size_t> {
    auto ids = std::set<std::size_t> {};
    for (const auto& key : critical) {
        ids.insert(key.net_id);
    }
    return std::Vector<std::size_t>(ids.begin(), ids.end());
}

auto log_feedback_round_begin(std::size_t round) -> void {
    debug::info(kRoundBanner);
    debug::info_fmt("feedback round={} begin", round);
}

auto log_feedback_round_end(std::size_t round, FeedbackRoundStatus status) -> void {
    debug::info_fmt(
        "feedback round={} end status={}",
        round,
        feedback_round_status_name(status));
    debug::info(kRoundBanner);
}

auto log_failed_nets(
    const std::Vector<RoutingNet>& nets,
    const std::Vector<PairKey>& critical
) -> void {
    const auto net_ids = unique_failed_net_ids(critical);
    debug::info_fmt("feedback failed nets: count={}", net_ids.size());
    for (const std::size_t net_id : net_ids) {
        const auto* net = net_by_id(nets, net_id);
        const auto& name = net == nullptr ? std::String {"?"} : net->name;
        debug::info_fmt("  - \"{}\" (id={})", name, net_id);
    }
}

auto log_non_shortest_nets(
    hardware::Interposer* interposer,
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const DelayPrecomputeResult& delays,
    const SatRoutingResult& result
) -> void {
    if (!result.ok) {
        return;
    }

    auto stretched = collect_net_stretch_info(interposer, graph, nets, delays, result);
    std::erase_if(stretched, [](const NetStretchInfo& entry) {
        return entry.shortest == 0 || entry.actual <= entry.shortest;
    });

    debug::info_fmt("non-shortest nets: count={}", stretched.size());
    for (const auto& entry : stretched) {
        debug::info_fmt(
            "  net=\"{}\" id={} actual={} shortest={} delta={}%",
            entry.name,
            entry.net_id,
            entry.actual,
            entry.shortest,
            format_percent(static_cast<double>(entry.actual), static_cast<double>(entry.shortest)));
    }
    debug::info_fmt(
        "non-shortest summary: stretched={} / total={}",
        stretched.size(),
        nets.size());
}

} // namespace PR_tool
