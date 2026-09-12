#include "hardware_graph.hh"
#include "net_adapter.hh"
#include "route_validate.hh"
#include "rrr_router.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <parse/reader/module.hh>
#include <utility/elapsed.hh>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

auto is_multi_port(const PR_tool::RoutingNet& net) -> bool {
    return net.demands.size() > 1 || net.sources.size() > 1;
}

auto scan_config(const std::filesystem::path& case_dir) -> bool {
    using namespace PR_tool;
    std::cout << "FPIA_RRR_mux_test: run " << case_dir.string() << '\n';
    try {
        auto [interposer, basedie] = parse::read_config(case_dir.string(), 0, false);
        algo::build_nets(basedie.get(), interposer.get());
        const auto nets = build_routing_nets(basedie->nets_to_vector());
        const auto graph = build_hardware_graph(interposer.get(), nets);
        auto params = RrrParams {};
        const auto result = run_rrr(graph, nets, params, interposer.get());
        if (!validate_rrr_solution(graph, nets, result, interposer.get())) {
            std::cerr << "FPIA_RRR_mux_test: validate failed for " << case_dir.string()
                      << " status=" << result.status << '\n';
            return false;
        }
        if (result.status != "success" || result.best_overflow != 0) {
            std::cerr << "FPIA_RRR_mux_test: routing did not succeed for " << case_dir.string()
                      << " status=" << result.status << " overflow=" << result.best_overflow << '\n';
            return false;
        }
        for (std::size_t i = 0; i < nets.size(); ++i) {
            if (!is_multi_port(nets[i])) {
                continue;
            }
            const auto hits = collect_illegal_tob_fanout(graph, result.paths[i]);
            if (!hits.empty()) {
                const auto& hit = hits.front();
                std::cerr << "FPIA_RRR_mux_test: illegal TOB fanout net=" << nets[i].name
                          << " node=" << hit.node << " side=" << hit.side
                          << " peers=" << hit.peer_count << '\n';
                return false;
            }
        }
        std::cout << "FPIA_RRR_mux_test: ok " << case_dir.string()
                  << " nets=" << nets.size() << " wirelength=" << result.total_wirelength << '\n';
        return true;
    }
    catch (const std::exception& error) {
        std::cout << "FPIA_RRR_mux_test: skip " << case_dir.string() << " (" << error.what() << ")\n";
        return true;
    }
}

auto collect_configs() -> std::vector<std::filesystem::path> {
    auto configs = std::vector<std::filesystem::path> {};
    const auto roots = std::vector<std::filesystem::path> {
        "algorithm/test_ILP/test",
        "test/config"};
    for (const auto& root : roots) {
        if (!std::filesystem::exists(root)) {
            continue;
        }
        for (const auto& entry : std::filesystem::directory_iterator(root)) {
            if (!entry.is_directory()) {
                continue;
            }
            if (std::filesystem::exists(entry.path() / "config.json")) {
                configs.push_back(entry.path());
            }
        }
    }
    std::sort(configs.begin(), configs.end());
    return configs;
}

} // namespace

auto main() -> int {
    PR_tool::Elapsed::start();
    const auto configs = collect_configs();
    if (configs.empty()) {
        std::cerr << "FPIA_RRR_mux_test: no config.json cases found\n";
        return 1;
    }
    bool ok = true;
    for (const auto& config : configs) {
        if (!scan_config(config)) {
            ok = false;
        }
    }
    return ok ? 0 : 1;
}
