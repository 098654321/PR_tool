#include "sat/encode_bus_sync.hh"

#include "sat/sat_constraint_kits.hh"

#include <algorithm>
#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

auto sink_d_literal(
    const UnifiedSatModel& model,
    const PairDelayInfo& pair,
    int delay
) -> int {
    const auto source_it = std::find_if(
        model.sources.begin(),
        model.sources.end(),
        [&](const SourceDelayVars& source) {
            return source.net_id == pair.net_id && source.source_index == pair.source_index;
        });
    if (source_it == model.sources.end()) {
        return 0;
    }
    const auto& scope = model.scopes[source_it->scope_index];
    if (pair.sink_node < 0
        || static_cast<std::size_t>(pair.sink_node) >= scope.node_offset.size()) {
        return 0;
    }
    const int node_offset = scope.node_offset[static_cast<std::size_t>(pair.sink_node)];
    if (node_offset < 0 || delay < 0 || delay > source_it->d_max) {
        return 0;
    }
    const int lit = source_it->d_var[static_cast<std::size_t>(node_offset)]
                        [static_cast<std::size_t>(delay)];
    return lit > 0 ? lit : 0;
}

auto source_d_max_for_pair(const UnifiedSatModel& model, const PairDelayInfo& pair) -> int {
    const auto source_it = std::find_if(
        model.sources.begin(),
        model.sources.end(),
        [&](const SourceDelayVars& source) {
            return source.net_id == pair.net_id && source.source_index == pair.source_index;
        });
    return source_it == model.sources.end() ? -1 : source_it->d_max;
}

auto add_false(
    CadicalSession& session,
    int literal,
    SatEncodingStats* stats
) -> void {
    session.add_clause({-literal});
    if (stats != nullptr) {
        stats->add_clauses(SatClauseCategory::SyncBusEqualLength, 1);
    }
}

} // namespace

auto encode_bus_sync_constraints(
    CadicalSession& session,
    const std::Vector<RoutingNet>& nets,
    UnifiedSatModel& model,
    SatEncodingStats* stats
) -> void {
    for (const auto& net : nets) {
        if (!net.is_sync_bus) {
            continue;
        }
        auto bus_pairs = std::Vector<PairDelayInfo*> {};
        for (auto& pair : model.pair_delays) {
            if (pair.net_id == net.net_id) {
                bus_pairs.push_back(&pair);
            }
        }
        if (bus_pairs.size() != net.demands.size()) {
            throw std::logic_error(std::format(
                "Sync bus net {} is missing delay pairs", net.net_id));
        }
        if (bus_pairs.empty()) {
            continue;
        }

        int bus_d_max = 0;
        for (const auto* pair : bus_pairs) {
            const int source_d_max = source_d_max_for_pair(model, *pair);
            if (source_d_max < 0) {
                throw std::logic_error(std::format(
                    "Sync bus net {} is missing source vars for demand {}",
                    net.net_id,
                    pair->demand_id));
            }
            bus_d_max = std::max(bus_d_max, source_d_max);
        }

        const auto* reference = bus_pairs.front();
        for (int delay = 0; delay <= bus_d_max; ++delay) {
            const int reference_lit = sink_d_literal(model, *reference, delay);
            for (std::size_t member = 1; member < bus_pairs.size(); ++member) {
                const int member_lit = sink_d_literal(model, *bus_pairs[member], delay);
                if (reference_lit > 0 && member_lit > 0) {
                    add_equiv(
                        session,
                        member_lit,
                        reference_lit,
                        stats,
                        SatClauseCategory::SyncBusEqualLength);
                }
                else if (reference_lit > 0) {
                    add_false(session, reference_lit, stats);
                }
                else if (member_lit > 0) {
                    add_false(session, member_lit, stats);
                }
                else {
                    continue;
                }
            }
        }
    }
}

} // namespace PR_tool
