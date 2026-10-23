#include "experiment_support.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <algo/placer/place.hh>
#include <algo/placer/sa/saplacestrategy.hh>
#include <debug/debug.hh>
#include <parse/reader/module.hh>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <set>
#include <string>

using namespace PR_tool;

static void save_placement(const std::filesystem::path& path,
                           const std::vector<circuit::TopDieInstance*>& chips) {
    std::ofstream out(path);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << "instance\ttob_row\ttob_col\n";
    for (auto* chip : chips) {
        auto coord = chip->tob()->coord();
        out << chip->name() << '\t' << coord.row << '\t' << coord.col << '\n';
        debug::info_fmt("Experiment placement: {} at TOB {}", chip->name(), coord);
    }
}

int main(int argc, char** argv) {
    try {
        if (argc != 10) {
            throw std::runtime_error(
                "Usage: sa_experiment CASE_DIR OUTPUT_DIR SEED INIT FREEZE SOLVE COOLING STALL RUN_ID");
        }
        const auto case_dir = std::filesystem::absolute(argv[1]);
        const auto output = std::filesystem::absolute(argv[2]);
        const auto seed = std::stoull(argv[3]);
        const double init = std::stod(argv[4]), freeze = std::stod(argv[5]);
        const auto solve = std::stoull(argv[6]);
        const double cooling = std::stod(argv[7]);
        const auto stall = std::stoull(argv[8]);
        // The last argument is a run identifier, retained in console/debug logs.
        const std::string run_id = argv[9];
        if (!(init > freeze && freeze > 0 && cooling > 0 && cooling < 1 && solve && stall))
            throw std::runtime_error("Invalid annealing parameters");
        std::filesystem::create_directories(output);
        debug::initial_log(output / "debug.log");
        debug::set_debug_level(debug::DebugLevel::Info);
        debug::info_fmt("Experiment {}: seed={}, init={}, freeze={}, solve={}, cooling={}, stall={}, COB width={}",
                        run_id, seed, init, freeze, solve, cooling, stall,
                        static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH));
        auto [interposer, basedie, registers] = parse::read_config(case_dir, 0, false);
        algo::build_nets(basedie.get(), interposer.get());
        std::vector<circuit::TopDieInstance*> chips;
        for (const auto& [name, chip] : basedie->topdie_insts()) chips.push_back(chip.get());
        std::sort(chips.begin(), chips.end(), [](auto* a, auto* b) { return a->name() < b->name(); });
        if (chips.empty()) throw std::runtime_error("No chips loaded");

        algo::SAPlaceStrategy strategy(init, freeze, solve, cooling, stall);
        if (!strategy.is_valid_placement(interposer.get(), chips))
            throw std::runtime_error("Invalid initial placement");
        const auto initial = strategy.evaluate_placement(interposer.get(), chips, basedie.get());
        save_placement(output / "initial_placement.tsv", chips);
        sa_experiment::generator.seed(seed);
        sa_experiment::stats = {};
        sa_experiment::iterations.open(output / "iterations.tsv");
        sa_experiment::iterations.exceptions(std::ios::badbit | std::ios::failbit);
        sa_experiment::iterations << "layer\ttemperature_used\tcurrent_hpwl\tbest_hpwl\tstalled_layers"
            "\tattempts\tswap_attempts\tmove_attempts\taccepted_nonworse\taccepted_uphill\trejected\n";
        const auto start = std::chrono::steady_clock::now();
        algo::place(interposer.get(), chips, basedie.get(), strategy);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        const auto final = strategy.evaluate_placement(interposer.get(), chips, basedie.get());
        if (!strategy.is_valid_placement(interposer.get(), chips))
            throw std::runtime_error("Invalid final placement");
        // Check true hardware ownership/occupancy as well as coordinate legality.
        for (auto* chip : chips) {
            auto real_tob = interposer->get_tob(chip->tob()->coord());
            if (!real_tob || *real_tob != chip->tob() || chip->tob()->is_idle())
                throw std::runtime_error("Final TOB ownership/occupancy mismatch");
        }
        const auto& s = sa_experiment::stats;
        if (final > initial || final != s.best_cost || s.layers == 0)
            throw std::runtime_error("Restored objective differs from layer-end best");
        const auto per_layer = solve * chips.size();
        const auto stall_limit = stall * std::max<std::size_t>(1, chips.size() / 2);
        if (s.attempts != s.layers * per_layer || s.swap_attempts + s.move_attempts != s.attempts)
            throw std::runtime_error("Attempt counter mismatch");
        const bool frozen = s.final_temperature <= freeze;
        const bool stalled = s.stalled_layers >= stall_limit;
        if (!frozen && !stalled) throw std::runtime_error("Missing stopping condition");
        save_placement(output / "final_placement.tsv", chips);
        std::set<circuit::Net*> nets;
        for (auto* chip : chips) for (auto* net : chip->nets()) nets.insert(net);
        std::ofstream result(output / "result.json");
        result.exceptions(std::ios::badbit | std::ios::failbit);
        result << std::setprecision(17) << "{\n";
        auto field = [&](const char* name, auto value) { result << '"' << name << "\":" << value << ",\n"; };
        field("seed", seed); field("chips", chips.size()); field("evaluated_nets", nets.size());
        field("cob_width", hardware::Interposer::COB_ARRAY_WIDTH);
        field("init_temp", init); field("freeze_temp", freeze); field("solve_num", solve);
        field("cooling_rate", cooling); field("max_no_improvement", stall);
        field("effective_solve_num", per_layer); field("effective_stall_limit", stall_limit);
        field("initial_hpwl", initial); field("final_hpwl", final);
        field("improvement_pct", initial ? 100.0 * (initial - final) / initial : 0.0);
        field("placement_seconds", seconds); field("temperature_layers", s.layers);
        field("final_temperature", s.final_temperature); field("stalled_layers", s.stalled_layers);
        field("attempts", s.attempts); field("swap_attempts", s.swap_attempts); field("move_attempts", s.move_attempts);
        field("accepted_nonworse", s.accepted_nonworse); field("accepted_uphill", s.accepted_uphill);
        field("rejected", s.rejected);
        field("invalid_attempts", s.attempts - s.accepted_nonworse - s.accepted_uphill - s.rejected);
        result << "\"valid_placement\":true,\n\"stop_reason\":\""
               << (frozen && stalled ? "freeze_and_stall" : frozen ? "freeze" : "stall") << "\"\n}\n";
        debug::info_fmt("Experiment completed: initial_HPWL={}, final_HPWL={}, layers={}, attempts={}, seconds={}, freeze_stop={}, stall_stop={}",
                        initial, final, s.layers, s.attempts, seconds, frozen, stalled);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "SA experiment failed: " << error.what() << '\n';
        return 1;
    }
}
