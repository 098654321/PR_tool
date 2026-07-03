#include "scope/build_routing_nets.hh"

#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/bbsnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/btsnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <hardware/interposer.hh>

#include <format>
#include <stdexcept>

namespace PR_tool {

namespace {

auto make_bump_ref(const Bump_coord& bump) -> GraphNodeRef {
    GraphNodeRef ref {};
    ref.kind = GraphNodeRef::Kind::Bump;
    ref.bump = bump;
    ref.tob = bump.TOB;
    ref.bank = bump.Bank;
    ref.group = bump.Group;
    ref.line_index = bump.Index;
    return ref;
}

auto make_track_ref(const hardware::TrackCoord& coord, std::size_t track_index) -> GraphNodeRef {
    GraphNodeRef ref {};
    ref.kind = GraphNodeRef::Kind::Track;
    ref.track_coord = coord;
    ref.track_index = track_index;
    return ref;
}

auto same_source(const GraphNodeRef& lhs, const GraphNodeRef& rhs) -> bool {
    if (lhs.kind != rhs.kind) {
        return false;
    }
    if (lhs.kind == GraphNodeRef::Kind::Bump) {
        return lhs.bump == rhs.bump;
    }
    if (lhs.kind == GraphNodeRef::Kind::Track) {
        return lhs.track_coord == rhs.track_coord && lhs.track_index == rhs.track_index;
    }
    return false;
}

auto add_unique_source(RoutingNet& net, GraphNodeRef source) -> std::size_t {
    for (std::size_t i = 0; i < net.sources.size(); ++i) {
        if (same_source(net.sources[i], source)) {
            return i;
        }
    }
    net.sources.emplace_back(std::move(source));
    return net.sources.size() - 1;
}

auto add_demand(
    RoutingNet& net,
    GraphNodeRef sink,
    std::Vector<std::size_t> candidate_source_indices,
    bool fixed_pair
) -> void {
    const auto demand_id = net.demands.size();
    net.demands.emplace_back(RoutingDemand {
        demand_id,
        std::move(sink),
        std::move(candidate_source_indices),
        fixed_pair});
}

auto make_routing_net(const circuit::Net& net, RoutingNetKind kind) -> RoutingNet {
    RoutingNet routing_net {};
    routing_net.name = net.name();
    routing_net.origin_key = net.name();
    routing_net.origin_uid = net.uid();
    routing_net.kind = kind;
    return routing_net;
}

} // namespace

auto validate_v14_routing_nets(const std::Vector<RoutingNet>& nets) -> void {
    for (const auto& net : nets) {
        for (const auto& demand : net.demands) {
            if (net.kind == RoutingNetKind::PNnet) {
                if (demand.candidate_source_indices.empty()) {
                    throw std::invalid_argument(std::format(
                        "v14 PNnet '{}' demand {} requires at least one candidate source",
                        net.name,
                        demand.demand_id));
                }
                continue;
            }
            if (demand.candidate_source_indices.size() != 1) {
                throw std::invalid_argument(std::format(
                    "v14 net '{}' demand {} requires exactly one candidate source, got {}",
                    net.name,
                    demand.demand_id,
                    demand.candidate_source_indices.size()));
            }
        }
    }
}

auto bump_to_routing_coord(const hardware::Bump* bump) -> Bump_coord {
    const auto bump_index = bump->index();
    const auto tob_coord = bump->tob()->coord();
    return Bump_coord {
        static_cast<std::size_t>(tob_coord.row * hardware::Interposer::TOB_ARRAY_WIDTH + tob_coord.col),
        bump_index / 64,
        (bump_index % 64) / 8,
        bump_index % 8};
}

auto build_routing_nets(const std::Vector<std::Rc<circuit::Net>>& nets) -> std::Vector<RoutingNet> {
    auto out = std::Vector<RoutingNet> {};
    std::size_t next_id = 0;

    auto push_net = [&](RoutingNet net) {
        net.net_id = next_id++;
        out.emplace_back(std::move(net));
    };

    for (const auto& net : nets) {
        if (dynamic_cast<const circuit::BumpToBumpsNet*>(net.get()) != nullptr) {
            throw std::runtime_error(std::format(
                "unsupported multi-fanout net BumpToBumpsNet '{}'",
                net->name()));
        }
        if (dynamic_cast<const circuit::BumpToTracksNet*>(net.get()) != nullptr) {
            throw std::runtime_error(std::format(
                "unsupported multi-fanout net BumpToTracksNet '{}'",
                net->name()));
        }

        if (const auto* tsb_net = dynamic_cast<const circuit::TracksToBumpsNet*>(net.get())) {
            auto routing_net = make_routing_net(*net, RoutingNetKind::PNnet);
            for (auto* track : tsb_net->begin_tracks()) {
                add_unique_source(
                    routing_net,
                    make_track_ref(track->coord(), track->coord().index));
            }
            auto candidates = std::Vector<std::size_t> {};
            for (std::size_t i = 0; i < routing_net.sources.size(); ++i) {
                candidates.emplace_back(i);
            }
            for (auto* bump : tsb_net->end_bumps()) {
                add_demand(
                    routing_net,
                    make_bump_ref(bump_to_routing_coord(bump)),
                    candidates,
                    false);
            }
            push_net(std::move(routing_net));
            continue;
        }

        if (const auto* ttbn = dynamic_cast<const circuit::TrackToBumpsNet*>(net.get())) {
            const auto begin_track = ttbn->begin_track()->coord();
            auto routing_net = make_routing_net(*net, RoutingNetKind::Tnet);
            const auto source_index = add_unique_source(
                routing_net,
                make_track_ref(begin_track, begin_track.index));
            for (auto* end_bump : ttbn->end_bumps()) {
                add_demand(
                    routing_net,
                    make_bump_ref(bump_to_routing_coord(end_bump)),
                    {source_index},
                    true);
            }
            push_net(std::move(routing_net));
            continue;
        }

        if (auto* sync_net = dynamic_cast<circuit::SyncNet*>(net.get())) {
            const bool has_bnet_members = !sync_net->btbnets().empty();
            const bool has_tnet_members =
                !sync_net->bttnets().empty() || !sync_net->ttbnets().empty();
            if (has_bnet_members && has_tnet_members) {
                throw std::runtime_error(std::format(
                    "unsupported mixed Bnet/Tnet SyncNet '{}'",
                    net->name()));
            }
            auto routing_net = make_routing_net(
                *net,
                has_bnet_members ? RoutingNetKind::Bnet : RoutingNetKind::Tnet);
            routing_net.is_sync_bus = true;
            for (const auto& btb : sync_net->btbnets()) {
                const auto source_index = add_unique_source(
                    routing_net,
                    make_bump_ref(bump_to_routing_coord(btb->begin_bump())));
                add_demand(
                    routing_net,
                    make_bump_ref(bump_to_routing_coord(btb->end_bump())),
                    {source_index},
                    true);
            }
            for (const auto& btt : sync_net->bttnets()) {
                const auto end_track = btt->end_track()->coord();
                const auto source_index = add_unique_source(
                    routing_net,
                    make_track_ref(end_track, end_track.index));
                add_demand(
                    routing_net,
                    make_bump_ref(bump_to_routing_coord(btt->begin_bump())),
                    {source_index},
                    true);
            }
            for (const auto& ttb : sync_net->ttbnets()) {
                const auto begin_track = ttb->begin_track()->coord();
                const auto source_index = add_unique_source(
                    routing_net,
                    make_track_ref(begin_track, begin_track.index));
                add_demand(
                    routing_net,
                    make_bump_ref(bump_to_routing_coord(ttb->end_bump())),
                    {source_index},
                    true);
            }
            push_net(std::move(routing_net));
            continue;
        }

        if (const auto* bb_net = dynamic_cast<const circuit::BumpToBumpNet*>(net.get())) {
            auto routing_net = make_routing_net(*net, RoutingNetKind::Bnet);
            const auto source_index = add_unique_source(
                routing_net,
                make_bump_ref(bump_to_routing_coord(bb_net->begin_bump())));
            add_demand(
                routing_net,
                make_bump_ref(bump_to_routing_coord(bb_net->end_bump())),
                {source_index},
                true);
            push_net(std::move(routing_net));
            continue;
        }

        if (const auto* bt_net = dynamic_cast<const circuit::BumpToTrackNet*>(net.get())) {
            const auto end_track = bt_net->end_track()->coord();
            auto routing_net = make_routing_net(*net, RoutingNetKind::Tnet);
            const auto source_index = add_unique_source(
                routing_net,
                make_track_ref(end_track, end_track.index));
            add_demand(
                routing_net,
                make_bump_ref(bump_to_routing_coord(bt_net->begin_bump())),
                {source_index},
                true);
            push_net(std::move(routing_net));
            continue;
        }

        if (const auto* tb_net = dynamic_cast<const circuit::TrackToBumpNet*>(net.get())) {
            const auto begin_track = tb_net->begin_track()->coord();
            auto routing_net = make_routing_net(*net, RoutingNetKind::Tnet);
            const auto source_index = add_unique_source(
                routing_net,
                make_track_ref(begin_track, begin_track.index));
            add_demand(
                routing_net,
                make_bump_ref(bump_to_routing_coord(tb_net->end_bump())),
                {source_index},
                true);
            push_net(std::move(routing_net));
            continue;
        }

        throw std::runtime_error(std::format("unsupported net type in build_routing_nets: '{}'", net->name()));
    }

    validate_v14_routing_nets(out);
    return out;
}

} // namespace PR_tool
