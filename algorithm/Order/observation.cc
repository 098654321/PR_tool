#include "observation.hh"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>

namespace order_experiment {
using namespace PR_tool;

static std::string tsv(std::string_view text) {
    std::string result;
    for (char c : text) {
        if (c == '\t') result += "\\t";
        else if (c == '\n') result += "\\n";
        else if (c == '\r') result += "\\r";
        else if (c == '\\') result += "\\\\";
        else result += c;
    }
    return result;
}

std::size_t kind_of(Net* net) {
#define KIND(Type, Index) if (dynamic_cast<circuit::Type*>(net)) return Index;
    KIND(SyncNet, 0) KIND(TrackToBumpsNet, 1) KIND(BumpToTracksNet, 2)
    KIND(BumpToBumpsNet, 3) KIND(BumpToTrackNet, 4) KIND(TrackToBumpNet, 5)
    KIND(BumpToBumpNet, 6) KIND(TracksToBumpsNet, 7)
#undef KIND
    throw std::runtime_error("Unknown Net class");
}

std::vector<Net*> members(Net* net) {
    auto* sync = dynamic_cast<circuit::SyncNet*>(net);
    if (!sync) return {};
    std::vector<Net*> result;
    for (auto& n : sync->btbnets()) result.push_back(n.get());
    for (auto& n : sync->bttnets()) result.push_back(n.get());
    for (auto& n : sync->ttbnets()) result.push_back(n.get());
    return result;
}

void initialize(const std::filesystem::path& output) {
    stats = {};
    auto open = [&](std::ofstream& stream, const char* filename) {
        stream.open(output / filename);
        stream.exceptions(std::ios::badbit | std::ios::failbit);
        stream << std::setprecision(17);
    };
    open(order_log, "order.tsv"); open(net_log, "nets.tsv");
    open(member_log, "sync_members.tsv"); open(path_log, "paths.tsv");
    order_log << "sequence\tuid\tname\tclass\tbase_priority\tports\teffective_priority\n";
    net_log << "sequence\tuid\tclass\tcompleted\tthrew_retry\tlength\tseconds\tsync_members\tsync_min\tsync_max\tsync_equal\n";
    member_log << "sequence\tgroup_uid\tmember_uid\tclass\tlength\tgroup_complete\tgroup_equal\n";
    path_log << "uid\tpath_type\tindex\tresource\tconnector\n";
}

bool compare(Net* a, Net* b) {
    if (a->priority().value() != b->priority().value()) return a->priority() > b->priority();
    return a->uid() < b->uid();
}

void record_order(const std::vector<Net*>& nets) {
    sorted_nets = nets;
    std::set<std::string> uids;
    std::size_t max_ports = 0;
    for (auto* net : nets) max_ports = std::max(max_ports, net->port_number());
    for (std::size_t i = 0; i < nets.size(); ++i) {
        auto* n = nets[i];
        if (n->uid().empty() || !uids.insert(n->uid()).second)
            throw std::runtime_error("Empty or duplicate top-level Net UID");
        const auto kind = kind_of(n);
        const float expected = base_priority(kind) + (kind == 0 ? 0.0f : 0.9 * (float(n->port_number()) / max_ports));
        if (std::abs(n->priority().value() - expected) > 1e-5f || (i && compare(n, nets[i-1])))
            throw std::runtime_error("Priority assignment/order mismatch");
        order_log << i << '\t' << tsv(n->uid()) << '\t' << tsv(n->name()) << '\t' << classes[kind]
                  << '\t' << base_priority(kind) << '\t' << n->port_number() << '\t' << n->priority().value() << '\n';
    }
    order_log.flush();
}

void start_net(std::size_t sequence, Net* net) {
    if (sequence >= sorted_nets.size() || sorted_nets[sequence] != net || sequence != stats.attempted)
        throw std::runtime_error("Actual routing sequence differs from sorted order");
    ++stats.attempted;
    debug::info_fmt("Order experiment: start sequence={}, uid={}, class={}, priority={}",
                    sequence, net->uid(), classes[kind_of(net)], net->priority().value());
}

static void dump_path(Net* net) {
    std::size_t i = 0;
    const auto& package = net->pathpackage();
    for (const auto& [track, connector] : package._regular_path) {
        std::string conn = "none";
        if (connector) conn = std::format("{}:{}:{}:{}:{}", connector->coord(),
            int(connector->from_dir()), connector->from_track_index(), int(connector->to_dir()), connector->to_track_index());
        path_log << tsv(net->uid()) << "\ttrack\t" << i++ << '\t' << track->coord().to_string() << '\t' << conn << '\n';
    }
    auto endpoints = [&](const auto& entries, const char* type) {
        std::size_t index = 0;
        for (const auto& [bump, connector, track] : entries) {
            path_log << tsv(net->uid()) << '\t' << type << '\t' << index++ << '\t'
                     << std::format("{}:{}->{}", bump->tob()->coord(), bump->index(), track->coord()) << '\t'
                     << std::format("{}:{}:{}:{}", connector.bump_index(), connector.hori_index(), connector.vert_index(), connector.track_index()) << '\n';
        }
    };
    endpoints(package._tob_to_track, "bump_to_track");
    endpoints(package._track_to_tob, "track_to_bump");
}

void finish_net(std::size_t sequence, Net* net, Clock::time_point start, bool threw_retry) {
    auto children = members(net);
    std::size_t lo = 0, hi = 0, successful_members = 0;
    const bool sync = kind_of(net) == 0;
    bool complete = !threw_retry && net->length() > 0;
    if (sync) {
        ++stats.sync_groups;
        stats.sync_members += children.size();
        lo = children.empty() ? 0 : children.front()->length();
        for (auto* child : children) {
            lo = std::min(lo, child->length()); hi = std::max(hi, child->length());
            successful_members += child->length() > 0;
            stats.sync_total_length += child->length();
        }
        stats.sync_routed_members += successful_members;
        complete &= !children.empty() && successful_members == children.size();
        stats.sync_failed_groups += !complete;
        stats.sync_unequal_groups += complete && lo != hi;
        complete &= lo == hi;
        for (auto* child : children) {
            member_log << sequence << '\t' << tsv(net->uid()) << '\t' << tsv(child->uid()) << '\t'
                       << classes[kind_of(child)] << '\t' << child->length() << '\t' << complete << '\t' << (complete && lo == hi) << '\n';
            dump_path(child);
        }
    } else dump_path(net);
    stats.routed += complete;
    stats.failed += !complete;
    // Sum the existing Net::length() values, including any residual partial paths.
    // The completion flag prevents incomplete runs from being ranked by length.
    stats.total_length += net->length();
    stats.max_length = std::max(stats.max_length, sync ? hi : net->length());
    const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
    net_log << sequence << '\t' << tsv(net->uid()) << '\t' << classes[kind_of(net)] << '\t' << complete
            << '\t' << threw_retry << '\t' << net->length() << '\t' << seconds << '\t' << children.size()
            << '\t' << lo << '\t' << hi << '\t' << (sync && complete) << '\n';
    net_log.flush(); member_log.flush(); path_log.flush();
    debug::info_fmt("Order experiment: finished sequence={}, complete={}, length={}, seconds={}", sequence, complete, net->length(), seconds);
}
} // namespace order_experiment
