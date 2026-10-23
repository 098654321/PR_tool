#pragma once
#include "order_policy.hh"
#include <circuit/net/nets.hh>
#include <chrono>
#include <filesystem>
#include <fstream>

namespace order_experiment {
using Net = PR_tool::circuit::Net;
using Clock = std::chrono::steady_clock;
struct Statistics {
    std::size_t attempted{}, routed{}, failed{}, sync_groups{}, sync_members{};
    std::size_t sync_failed_groups{}, sync_unequal_groups{}, total_length{}, max_length{};
    std::size_t sync_total_length{}, sync_routed_members{};
};
inline Statistics stats;
inline std::ofstream order_log, net_log, member_log, path_log;
inline std::vector<Net*> sorted_nets;
std::size_t kind_of(Net* net);
std::vector<Net*> members(Net* net);
void initialize(const std::filesystem::path& output);
bool compare(Net* a, Net* b);
void record_order(const std::vector<Net*>& nets);
void start_net(std::size_t sequence, Net* net);
void finish_net(std::size_t sequence, Net* net, Clock::time_point start, bool threw_retry);
void self_test();
} // namespace order_experiment
