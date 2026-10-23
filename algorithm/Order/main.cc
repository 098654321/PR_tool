#include "observation.hh"
#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <algo/router/route_nets.hh>
#include <parse/reader/module.hh>
#include <algorithm>
#include <iomanip>
#include <iostream>

int main(int argc, char** argv) {
    using namespace PR_tool;
    using namespace order_experiment;
    try {
        if (argc == 2 && std::string_view(argv[1]) == "--self-test") {
            self_test();
            std::cout << "PASS: all 8 concrete Net constructors x 3 policies; original bias and SyncNet behavior\n";
            return 0;
        }
        if (argc != 6) throw std::runtime_error("Usage: order_experiment CASE_DIR OUTPUT_DIR SEED POLICY RUN_ID");
        const auto input = std::filesystem::absolute(argv[1]);
        const auto output = std::filesystem::absolute(argv[2]);
        const auto seed = std::stoull(argv[3]);
        policy = std::stoi(argv[4]);
        if (std::find(policy_ids.begin(), policy_ids.end(), policy) == policy_ids.end())
            throw std::runtime_error("Policy must be 0, 2 or 3");
        generator.seed(seed);
        std::filesystem::create_directories(output);
        debug::initial_log(output / "debug.log");
        debug::set_debug_level(debug::DebugLevel::Info);
        debug::info_fmt("Order experiment {}: policy={}, seed={}, COB width={}, mode=0, no placement, no incremental routing",
                        argv[5], policy, seed, static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH));
        initialize(output);
        auto [interposer, basedie, registers] = parse::read_config(input, 0, false);
        algo::build_nets(basedie.get(), interposer.get());
        std::vector<circuit::TopDieInstance*> chips;
        for (auto& [name, chip] : basedie->topdie_insts()) chips.push_back(chip.get());
        std::sort(chips.begin(), chips.end(), [](auto* a, auto* b) { return a->name() < b->name(); });
        std::ofstream placement(output / "placement.tsv");
        placement.exceptions(std::ios::badbit | std::ios::failbit);
        placement << "instance\ttob_row\ttob_col\n";
        for (auto* chip : chips) {
            auto coord = chip->tob()->coord();
            placement << chip->name() << '\t' << coord.row << '\t' << coord.col << '\n';
        }
        placement.close();
        const auto net_count = basedie->nets(0).size();
        if (!net_count) throw std::runtime_error("No top-level Nets loaded");
        const auto start = Clock::now();
        const auto routed = algo::route_nets(interposer.get(), basedie.get(), algo::MazeRouteStrategy{false},
                                              algo::HK{}, 0, false, false);
        const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
        if (stats.attempted != net_count || stats.routed + stats.failed != net_count || sorted_nets.size() != net_count)
            throw std::runtime_error("Incomplete/inconsistent routing counters");
        if (routed.failed_net_names.size() > stats.failed || routed.data._total_length != stats.total_length)
            throw std::runtime_error("Experiment metrics disagree with native route result");
        const bool complete = stats.failed == 0 && routed.failed_net_names.empty();
        std::ofstream result(output / "result.json");
        result.exceptions(std::ios::badbit | std::ios::failbit);
        result << std::setprecision(17) << "{\n";
        auto field = [&](const char* name, auto value) { result << '"' << name << "\":" << value << ",\n"; };
        field("policy", policy); field("seed", seed); field("cob_width", hardware::Interposer::COB_ARRAY_WIDTH);
        field("chips", chips.size()); field("total_nets", net_count); field("attempted_nets", stats.attempted);
        field("routed_nets", stats.routed); field("failed_nets", stats.failed);
        field("retry_failed_nets", routed.failed_net_names.size());
        field("routability_pct", 100.0 * stats.routed / net_count);
        field("total_length", stats.total_length); field("max_length", stats.max_length);
        field("sync_groups", stats.sync_groups); field("sync_members", stats.sync_members);
        field("sync_failed_groups", stats.sync_failed_groups); field("sync_unequal_groups", stats.sync_unequal_groups);
        field("sync_routed_members", stats.sync_routed_members); field("sync_total_length", stats.sync_total_length);
        field("average_routed_sync_length", stats.sync_routed_members ? double(stats.sync_total_length) / stats.sync_routed_members : 0.0);
        field("routing_seconds", seconds);
        result << "\"all_routed\":" << (complete ? "true" : "false") << "\n}\n";
        debug::info_fmt("Order experiment completed: routed={}/{}, failed={}, total length={}, seconds={}, all_routed={}",
                        stats.routed, net_count, stats.failed, stats.total_length, seconds, complete);
        return 0; // A measured routing failure is a valid experimental outcome.
    } catch (const std::exception& error) {
        std::cerr << "Order experiment error: " << error.what() << '\n';
        return 1;
    }
}
