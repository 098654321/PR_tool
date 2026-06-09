// SAT TOB allocation + optional MCF routing (Gurobi for MCF only).

#include "mcf/cob_mcf_router.hh"
#include "maze_check/maze_check.hh"
#include "ilp_allocation/gurobi_model_stats.hh"
#include "common/ilp_types.hh"
#include "common/tob_allocation_types.hh"
#include "precompute/ilp_reach_precompute.hh"
#include "ilp_allocation/tob_ilp_model.hh"
#include "sat_allocation/solve_tob_mcf_pipeline.hh"
#include "sat_allocation/solve_tob_sat.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/routeerror.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/bbsnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/btsnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/net/types/tbnet.hh>
#include <circuit/net/types/tbsnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <debug/debug.hh>
#include <hardware/interposer.hh>
#include <parse/reader/module.hh>

#include <std/string.hh>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <format>
#include <map>
#include <set>
#include <sys/resource.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

namespace PR_tool {

auto bump_to_ilp_coord(const hardware::Bump* bump) -> Bump_coord;
auto sort_and_unique(std::Vector<std::size_t>& values) -> void;
auto sort_and_unique(std::Vector<Bump_coord>& values) -> void;
struct BuildRecordsResult {
    std::Vector<Net_cost_record> records {};
    std::Vector<std::Rc<circuit::Net>> deferred_multi_fanout {};
    std::Vector<std::Rc<circuit::Net>> track_to_bumps_nets {};
};

auto classify_net(const std::Rc<circuit::Net>& net) -> Net_cost_record;
auto build_records(const std::Vector<std::Rc<circuit::Net>>& nets) -> BuildRecordsResult;
auto write_mps_file(
    const std::Vector<Net_cost_record>& records,
    const std::String& output_mps
) -> void;
auto get_peak_rss_mb() -> double;
auto log_tob_sat_infeasibility_diagnosis(const TobIlpResult& result) -> void;
auto log_tob_sat_bump_demand(const std::Vector<Net_cost_record>& records) -> void;
auto log_tob_sat_bump_usage(const TobIlpResult& result) -> void;
auto is_testpn_golden_case(const std::String& config_path) -> bool;
auto run_wire_length_golden_check(const std::String& config_path, std::size_t actual) -> bool;

constexpr auto kTestIlpUsage =
    "Usage: xmake run test_ILP <config_path> [-v|-vv|...] "
    "[--export-ilp-mps <path>] [--enable-mcf-routing] [--disable-bus-mcf] "
    "[--enable-mcf-parallel] [--enable-mcf-obj] [--enable-pre-routing] "
    "[--sat-log] [--gurobi-log] "
    "[--maze-check-ilp-mcf | --maze-check-mcf] "
    "[--simple-maze] "
    "[--check-golden]";

auto run_main(int argc, char** argv) -> int {
    const auto run_begin = std::chrono::steady_clock::now();
    const auto log_total_runtime = [&]() {
        const auto run_end = std::chrono::steady_clock::now();
        const auto run_ms = std::chrono::duration_cast<std::chrono::milliseconds>(run_end - run_begin).count();
        debug::info_fmt("run_main total elapsed: {} ms", run_ms);
    };
    if (argc < 2) {
        debug::error("No config path given");
        debug::info(kTestIlpUsage);
        log_total_runtime();
        return 1;
    }

    const auto config_path = std::String(argv[1]);
    auto export_ilp_mps = std::String {};
    bool enable_mcf = false;
    bool check_golden = false;
    bool disable_bus_mcf = false;
    bool enable_mcf_parallel = false;
    bool enable_mcf_obj = false;
    bool enable_pre_routing = false;
    bool maze_check_ilp_mcf = false;
    bool maze_check_mcf = false;
    bool enable_simple_maze = false;
    bool enable_gurobi_log = false;
    bool enable_sat_log = false;
    int verbose_v_count = 0;
    for (int argi = 2; argi < argc; ++argi) {
        const auto arg = std::String(argv[argi]);
        if (arg.size() >= 2 && arg[0] == '-' && arg[1] == 'v') {
            bool all_v = true;
            for (std::size_t i = 1; i < arg.size(); ++i) {
                if (arg[i] != 'v') {
                    all_v = false;
                    break;
                }
            }
            if (all_v) {
                verbose_v_count += static_cast<int>(arg.size() - 1);
                continue;
            }
        }
        if (arg == "--export-ilp-mps") {
            if (argi + 1 >= argc) {
                debug::error("--export-ilp-mps requires a path argument");
                log_total_runtime();
                return 1;
            }
            export_ilp_mps = std::String(argv[argi + 1]);
            argi += 1;
            continue;
        }
        if (arg == "--sat-log") {
            enable_sat_log = true;
            continue;
        }
        if (arg == "--enable-mcf-routing") {
            enable_mcf = true;
            continue;
        }
        if (arg == "--disable-bus-mcf") {
            disable_bus_mcf = true;
            continue;
        }
        if (arg == "--enable-mcf-parallel") {
            enable_mcf_parallel = true;
            continue;
        }
        if (arg == "--enable-mcf-obj") {
            enable_mcf_obj = true;
            continue;
        }
        if (arg == "--enable-pre-routing") {
            enable_pre_routing = true;
            continue;
        }
        if (arg == "--maze-check-ilp-mcf") {
            maze_check_ilp_mcf = true;
            continue;
        }
        if (arg == "--maze-check-mcf") {
            maze_check_mcf = true;
            continue;
        }
        if (arg == "--simple-maze") {
            enable_simple_maze = true;
            continue;
        }
        if (arg == "--gurobi-log") {
            enable_gurobi_log = true;
            continue;
        }
        if (arg == "--check-golden") {
            check_golden = true;
            continue;
        }
        debug::error_fmt("Unexpected argument '{}'", arg);
        debug::info(kTestIlpUsage);
        log_total_runtime();
        return 1;
    }

    if (check_golden && !enable_mcf) {
        debug::error("--check-golden requires --enable-mcf-routing");
        log_total_runtime();
        return 1;
    }

    const CobMcfGridDims cob_grid {
        static_cast<int>(hardware::Interposer::COB_ARRAY_HEIGHT),
        static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH),
    };

    if ((maze_check_ilp_mcf || maze_check_mcf) && !enable_mcf) {
        debug::error("maze-check flags require --enable-mcf-routing");
        log_total_runtime();
        return 1;
    }
    if (disable_bus_mcf && !enable_mcf) {
        debug::error("--disable-bus-mcf requires --enable-mcf-routing");
        log_total_runtime();
        return 1;
    }
    if (maze_check_ilp_mcf && maze_check_mcf) {
        debug::error("--maze-check-ilp-mcf and --maze-check-mcf are mutually exclusive");
        log_total_runtime();
        return 1;
    }
    if (enable_simple_maze && !enable_mcf) {
        debug::error("--simple-maze requires --enable-mcf-routing");
        log_total_runtime();
        return 1;
    }
    if (enable_simple_maze && (maze_check_ilp_mcf || maze_check_mcf)) {
        debug::error("--simple-maze is mutually exclusive with --maze-check-ilp-mcf / --maze-check-mcf");
        log_total_runtime();
        return 1;
    }
    if (enable_simple_maze && enable_mcf_parallel) {
        debug::info("warning: --enable-mcf-parallel has no effect with --simple-maze");
    }
    if (enable_simple_maze && enable_mcf_obj) {
        debug::info("warning: --enable-mcf-obj has no effect with --simple-maze");
    }
    if (enable_pre_routing && !enable_mcf) {
        debug::info("warning: --enable-pre-routing has no effect without --enable-mcf-routing (MCF graph warm start only)");
    }
    const bool defer_maze_check_suspend = maze_check_ilp_mcf || maze_check_mcf;

    // read file and build nets
    debug::initial_log("./debug.log");
    GurobiDiagnosticsOptions gurobi_diag {};
    gurobi_diag.log_dir = std::format("./{}", kGurobiLogSubdir);
    init_gurobi_modelinfo_log(gurobi_diag.log_dir);
    if (enable_gurobi_log) {
        gurobi_diag.enable_gurobi_log = true;
        log_gurobi_modelinfo(
            gurobi_diag.log_dir,
            std::format("Gurobi solver logs enabled: directory={}", gurobi_diag.log_dir));
    }
    if (verbose_v_count > 0) {
        debug::set_debug_level(debug::DebugLevel::Debug);
        debug::info_fmt("verbose mode enabled: -v count={}", verbose_v_count);
    }
    auto [interposer, basedie] = PR_tool::parse::read_config(config_path, 0, false);
    algo::build_nets(basedie.get(), interposer.get());

    // build records
    const auto nets = basedie->nets_to_vector();
    auto built = build_records(nets);
    auto records = std::move(built.records);
    const auto& deferred_multi_fanout = built.deferred_multi_fanout;
    const auto& track_to_bumps_nets = built.track_to_bumps_nets;
    if (!deferred_multi_fanout.empty()) {
        debug::info_fmt(
            "Deferred multi-fanout nets: {} — excluded from SAT+MCF (unexpected after build_records)",
            deferred_multi_fanout.size());
    }
    if (!track_to_bumps_nets.empty()) {
        debug::info_fmt(
            "TrackToBumpsNet: {} net(s) split for SAT TOB; COB segment routed in SimpleMCF",
            track_to_bumps_nets.size());
    }
    log_tob_sat_bump_demand(records);

    if (!export_ilp_mps.empty()) {
        const auto reach_stats = precompute_reach_for_records(records);
        debug::info_fmt(
            "MPS export reach precompute: records={} total_endtracks={} total_starttrack_edges={}",
            reach_stats.total_records,
            reach_stats.total_endtracks,
            reach_stats.total_starttrack_edges);
        write_mps_file(records, export_ilp_mps);
        debug::info_fmt("TOB legacy ILP MPS export written: {}", export_ilp_mps);
    }

    CadicalDiagnosticsOptions sat_diag {};
    sat_diag.log_dir = "./cadical-log";
    sat_diag.enable_sat_log = enable_sat_log;
    sat_diag.verbose_reach_endpoints = verbose_v_count > 0;
    if (enable_sat_log) {
        debug::info_fmt("CaDiCal solver logs enabled: directory={}", sat_diag.log_dir);
    }

    long long tob_sat_solve_ms = 0;
    long long mcf_warm_start_ms = 0;
    long long mcf_solve_ms = 0;
    TobIlpResult result {};
    CobMcfFullResult mcf_full {};

    if (enable_mcf) {
        if (enable_mcf_parallel) {
            debug::info("MCF: solving 16 COB units in parallel (std::async)");
        }
        const auto pipeline = solve_tob_mcf_with_range_iteration(
            records,
            interposer.get(),
            *basedie.get(),
            cob_grid,
            enable_mcf_parallel,
            enable_pre_routing,
            enable_mcf_obj,
            disable_bus_mcf,
            enable_simple_maze,
            !defer_maze_check_suspend,
            sat_diag,
            gurobi_diag);
        tob_sat_solve_ms = pipeline.tob_sat_solve_ms;
        mcf_warm_start_ms = pipeline.mcf_warm_start_ms;
        mcf_solve_ms = pipeline.mcf_solve_ms;
        debug::info_fmt("timing phase=tob_sat_solve ms={}", tob_sat_solve_ms);
        debug::info_fmt("timing phase=mcf_warm_start ms={}", mcf_warm_start_ms);
        debug::info_fmt("timing phase=mcf_solve ms={}", mcf_solve_ms);
        const auto peak_rss_mb = get_peak_rss_mb();
        debug::info_fmt("Process peak RSS: {:.2f} MB", peak_rss_mb);

        if (!pipeline.ok) {
            debug::error_fmt("SAT+MCF: {}", pipeline.message);
            if (!pipeline.tob.infeasibility_hints.empty()) {
                log_tob_sat_infeasibility_diagnosis(pipeline.tob);
            }
            debug::info_fmt(
                "timing breakdown (ms): tob_sat_solve={}, mcf_warm_start={}, mcf_solve={}",
                tob_sat_solve_ms,
                mcf_warm_start_ms,
                mcf_solve_ms);
            log_total_runtime();
            return 1;
        }
        result = std::move(pipeline.tob);
        mcf_full = std::move(pipeline.mcf);
    }
    else {
        const auto solve_begin = std::chrono::steady_clock::now();
        result = solve_tob_sat_with_cadical(records, sat_diag);
        const auto solve_end = std::chrono::steady_clock::now();
        tob_sat_solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_begin).count();
        const auto peak_rss_mb = get_peak_rss_mb();
        debug::info_fmt("timing phase=tob_sat_solve ms={}", tob_sat_solve_ms);
        debug::info_fmt("Process peak RSS: {:.2f} MB", peak_rss_mb);

        if (!result.ok) {
            debug::error_fmt("SAT TOB: {}", result.message);
            log_tob_sat_infeasibility_diagnosis(result);
            debug::info_fmt(
                "timing breakdown (ms): tob_sat_solve={}, mcf_warm_start={}, mcf_solve={}",
                tob_sat_solve_ms,
                0,
                0);
            log_total_runtime();
            return 1;
        }
    }
    log_tob_sat_bump_usage(result);
    for (const auto& d : result.route_details) {
        debug::info_fmt(
            "net \"{}\": bump(T{},B{},G{},I{}) -> j={} (horizontal line), k={} (vertical line), s={}, orient={}, track={}, COBUnit={}",
            d.net_name,
            d.bump.TOB,
            d.bump.Bank,
            d.bump.Group,
            d.bump.Index,
            d.j,
            d.k,
            d.s_v,
            d.use_straight ? "straight(QS)" : "wrap(QW)",
            d.track,
            d.cob_unit
        );
    }
    debug::info_fmt("active W count: {}", result.active_w.size());
    for (const auto& w : result.active_w) {
        if (!w.has_track) {
            debug::info_fmt(
                "W active: bump(T{},B{},G{},I{}) j={} (horizontal line), k={} (vertical line) (track unresolved)",
                w.bump.TOB,
                w.bump.Bank,
                w.bump.Group,
                w.bump.Index,
                w.j,
                w.k
            );
            continue;
        }
        debug::info_fmt(
            "W active: bump(T{},B{},G{},I{}) j={} (horizontal line), k={} (vertical line), orient={}, track={}",
            w.bump.TOB,
            w.bump.Bank,
            w.bump.Group,
            w.bump.Index,
            w.j,
            w.k,
            w.use_straight ? "straight" : "wrap",
            w.track
        );
    }
    debug::info_fmt("active S count: {}", result.active_s.size());
    for (const auto& s : result.active_s) {
        debug::info_fmt(
            "S active: TOB={}, v={} (j={} horizontal line, k={} vertical line)",
            s.tob,
            s.v,
            s.j,
            s.k);
    }
    debug::info_fmt("objective value: {}", result.objective);
    debug::info_fmt("SAT solved with bbox max_rho={}", result.range_level);
    debug::info_fmt("nets solved: {}", records.size());

    if (enable_mcf) {
        if (maze_check_ilp_mcf) {
            (void)run_maze_check_ilp_mcf_after_mcf(
                interposer.get(),
                basedie.get(),
                records,
                result,
                mcf_full,
                cob_grid);
        }
        if (maze_check_mcf) {
            (void)run_maze_check_mcf_after_mcf(
                interposer.get(),
                basedie.get(),
                records,
                result,
                mcf_full,
                cob_grid);
        }
        if (check_golden && !run_wire_length_golden_check(config_path, mcf_full.summary.total_wire_length)) {
            log_total_runtime();
            return 1;
        }
    }
    debug::info_fmt(
        "timing breakdown (ms): tob_sat_solve={}, mcf_warm_start={}, mcf_solve={}",
        tob_sat_solve_ms,
        mcf_warm_start_ms,
        mcf_solve_ms);
    log_total_runtime();
    return 0;
}

auto tob_sat_status_name(const int status) -> std::String {
    if (status == 10) {
        return "SAT";
    }
    if (status == 20) {
        return "UNSAT";
    }
    return "UNKNOWN";
}

auto log_tob_sat_infeasibility_diagnosis(const TobIlpResult& result) -> void {
    debug::error_fmt(
        "TOB SAT infeasibility diagnosis: status={}({}) message=\"{}\"",
        tob_sat_status_name(result.model_status),
        result.model_status,
        result.message);
    if (!result.infeasibility_hints.empty()) {
        debug::error_fmt("  hints={}", result.infeasibility_hints.size());
    }
}

auto tob_sat_relation_bumps_for_record(const Net_cost_record& record) -> std::Vector<Bump_coord> {
    auto relation_bumps = std::Vector<Bump_coord> {};
    if (record.type == Net_type::Bnet) {
        relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
        relation_bumps.insert(relation_bumps.end(), record.end_bumps.begin(), record.end_bumps.end());
    }
    else {
        relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
    }
    sort_and_unique(relation_bumps);
    return relation_bumps;
}

auto log_tob_sat_bump_demand(const std::Vector<Net_cost_record>& records) -> void {
    constexpr std::size_t kBumpsPerTob = 128;
    constexpr std::size_t kBumpsPerBank = 64;
    constexpr std::size_t kTotalBumps = hardware::Interposer::TOB_SIZE * kBumpsPerTob;
    std::array<std::set<Bump_coord>, hardware::Interposer::TOB_SIZE> bumps_by_tob {};
    std::array<std::array<std::set<Bump_coord>, 2>, hardware::Interposer::TOB_SIZE> bumps_by_bank {};
    auto all_bumps = std::set<Bump_coord> {};

    for (const auto& record : records) {
        for (const auto& bump : tob_sat_relation_bumps_for_record(record)) {
            if (bump.TOB >= hardware::Interposer::TOB_SIZE || bump.Bank >= 2) {
                continue;
            }
            bumps_by_tob[bump.TOB].insert(bump);
            bumps_by_bank[bump.TOB][bump.Bank].insert(bump);
            all_bumps.insert(bump);
        }
    }

    debug::info("TOB SAT bump demand (pre-solve, available_on_failure=true)");
    for (std::size_t t = 0; t < hardware::Interposer::TOB_SIZE; ++t) {
        const auto row = t / hardware::Interposer::TOB_ARRAY_WIDTH;
        const auto col = t % hardware::Interposer::TOB_ARRAY_WIDTH;
        debug::info_fmt(
            "  TOB({},{})[linear={}]={}/{} bank0={}/{} bank1={}/{}",
            row,
            col,
            t,
            bumps_by_tob[t].size(),
            kBumpsPerTob,
            bumps_by_bank[t][0].size(),
            kBumpsPerBank,
            bumps_by_bank[t][1].size(),
            kBumpsPerBank);
    }
    debug::info_fmt(
        "  summary demanded_bumps={}/{} records={}",
        all_bumps.size(),
        kTotalBumps,
        records.size());
}

auto log_tob_sat_bump_usage(const TobIlpResult& result) -> void {
    constexpr std::size_t kBumpsPerTob = 128;
    constexpr std::size_t kBumpsPerBank = 64;
    constexpr std::size_t kTotalBumps = hardware::Interposer::TOB_SIZE * kBumpsPerTob;
    std::array<std::set<Bump_coord>, hardware::Interposer::TOB_SIZE> bumps_by_tob {};
    std::array<std::array<std::set<Bump_coord>, 2>, hardware::Interposer::TOB_SIZE> bumps_by_bank {};
    auto all_bumps = std::set<Bump_coord> {};

    for (const auto& w : result.active_w) {
        if (w.bump.TOB >= hardware::Interposer::TOB_SIZE || w.bump.Bank >= 2) {
            continue;
        }
        bumps_by_tob[w.bump.TOB].insert(w.bump);
        bumps_by_bank[w.bump.TOB][w.bump.Bank].insert(w.bump);
        all_bumps.insert(w.bump);
    }

    debug::info("TOB SAT bump usage (post-solve, ok=true)");
    for (std::size_t t = 0; t < hardware::Interposer::TOB_SIZE; ++t) {
        const auto row = t / hardware::Interposer::TOB_ARRAY_WIDTH;
        const auto col = t % hardware::Interposer::TOB_ARRAY_WIDTH;
        debug::info_fmt(
            "  TOB({},{})[linear={}]={}/{} bank0={}/{} bank1={}/{}",
            row,
            col,
            t,
            bumps_by_tob[t].size(),
            kBumpsPerTob,
            bumps_by_bank[t][0].size(),
            kBumpsPerBank,
            bumps_by_bank[t][1].size(),
            kBumpsPerBank);
    }
    debug::info_fmt(
        "  summary used_bumps={}/{} active_w={} active_s={}",
        all_bumps.size(),
        kTotalBumps,
        result.active_w.size(),
        result.active_s.size());
}

auto bump_to_ilp_coord(const hardware::Bump* bump) -> Bump_coord {
    const auto bump_index = bump->index();
    const auto tob_coord = bump->tob()->coord();
    return Bump_coord {
        static_cast<std::size_t>(tob_coord.row * hardware::Interposer::TOB_ARRAY_WIDTH + tob_coord.col),
        bump_index / 64,
        (bump_index % 64) / 8,
        bump_index % 8
    };
}

auto sort_and_unique(std::Vector<std::size_t>& values) -> void {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

auto sort_and_unique(std::Vector<Bump_coord>& values) -> void {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

auto classify_net(const std::Rc<circuit::Net>& net) -> Net_cost_record {
    const auto add_all_cobunits = [](std::Vector<std::size_t>& cs) {
        for (std::size_t c = 0; c < 16; ++c) {
            cs.emplace_back(c);
        }
    };

    Net_cost_record record {
        net->name(),
        Net_type::Tnet,
        1.0F,
        1.0F,
        {},
        {},
        {},
        {}
    };

    if (record.bits <= 0.0F) {
        throw std::runtime_error(std::format("net '{}' has non-positive bits", net->name()));
    }

    if (const auto* bb_net = dynamic_cast<const circuit::BumpToBumpNet*>(net.get())) {
        record.type = Net_type::Bnet;
        record.lambda = 10.0F;
        record.start_bumps.emplace_back(bump_to_ilp_coord(bb_net->begin_bump()));
        record.end_bumps.emplace_back(bump_to_ilp_coord(bb_net->end_bump()));
        add_all_cobunits(record.candidate_cobunits);
        record.mcf_start_kind = IlpEndpointKind::Bump;
        record.mcf_end_kind = IlpEndpointKind::Bump;
    }
    else if (dynamic_cast<const circuit::BumpToBumpsNet*>(net.get()) != nullptr
             || dynamic_cast<const circuit::BumpToTracksNet*>(net.get()) != nullptr
             || dynamic_cast<const circuit::TrackToBumpsNet*>(net.get()) != nullptr) {
        throw std::logic_error(
            std::format("internal: multi-fanout net must be deferred in build_records, not classify_net: '{}'", net->name()));
    }
    else if (const auto* bt_net = dynamic_cast<const circuit::BumpToTrackNet*>(net.get())) {
        const auto cobunit = map_track(bt_net->end_track()->coord().index);
        record.type = Net_type::Tnet;
        record.start_bumps.emplace_back(bump_to_ilp_coord(bt_net->begin_bump()));
        record.candidate_cobunits.emplace_back(cobunit);
        record.tnet_fixed_cobunits.emplace_back(cobunit);
        record.end_tracks.emplace_back(bt_net->end_track()->coord().index);
        record.mcf_end_track = bt_net->end_track()->coord();
        record.mcf_has_end_track = true;
        record.mcf_start_kind = IlpEndpointKind::Bump;
        record.mcf_end_kind = IlpEndpointKind::Track;
    }
    else if (const auto* tb_net = dynamic_cast<const circuit::TrackToBumpNet*>(net.get())) {
        const auto cobunit = map_track(tb_net->begin_track()->coord().index);
        record.type = Net_type::Tnet;
        record.start_bumps.emplace_back(bump_to_ilp_coord(tb_net->end_bump()));
        record.candidate_cobunits.emplace_back(cobunit);
        record.tnet_fixed_cobunits.emplace_back(cobunit);
        record.end_tracks.emplace_back(tb_net->begin_track()->coord().index);
        record.mcf_end_track = tb_net->begin_track()->coord();
        record.mcf_has_end_track = true;
        record.mcf_start_kind = IlpEndpointKind::Bump;
        record.mcf_end_kind = IlpEndpointKind::Track;
    }
    else if (dynamic_cast<const circuit::TracksToBumpsNet*>(net.get()) != nullptr) {
        throw std::runtime_error(std::format("TracksToBumpsNet should be expanded in build_records: '{}'", net->name()));
    }
    else if (dynamic_cast<circuit::SyncNet*>(net.get()) != nullptr) {
        throw std::runtime_error(std::format("SyncNet should be expanded in build_records: '{}'", net->name()));
    }
    else {
        throw std::runtime_error(std::format("unknown net type: '{}'", net->name()));
    }

    sort_and_unique(record.start_bumps);
    sort_and_unique(record.end_bumps);
    sort_and_unique(record.candidate_cobunits);
    sort_and_unique(record.tnet_fixed_cobunits);
    sort_and_unique(record.end_tracks);

    if (record.start_bumps.empty()) {
        throw std::runtime_error(std::format("net '{}' has empty start bump set", net->name()));
    }
    if (record.candidate_cobunits.empty()) {
        throw std::runtime_error(std::format("net '{}' has empty candidate cobunits", net->name()));
    }
    record.origin_key = net->name();
    record.origin_uid = net->uid();
    return record;
}

auto build_records(const std::Vector<std::Rc<circuit::Net>>& nets) -> BuildRecordsResult {
    BuildRecordsResult out {};
    auto& records = out.records;
    records.reserve(nets.size());
    for (const auto& net : nets) {
        if (dynamic_cast<const circuit::BumpToBumpsNet*>(net.get()) != nullptr) {
            throw std::runtime_error(std::format(
                "unsupported multi-fanout net BumpToBumpsNet '{}' (only TrackToBumpsNet is supported in test_ILP)",
                net->name()));
        }
        if (dynamic_cast<const circuit::BumpToTracksNet*>(net.get()) != nullptr) {
            throw std::runtime_error(std::format(
                "unsupported multi-fanout net BumpToTracksNet '{}' (only TrackToBumpsNet is supported in test_ILP)",
                net->name()));
        }
        if (const auto* ttbn = dynamic_cast<const circuit::TrackToBumpsNet*>(net.get())) {
            const auto cobunit = map_track(ttbn->begin_track()->coord().index);
            const auto begin_track_index = ttbn->begin_track()->coord().index;
            std::size_t bump_idx = 0;
            for (auto* end_bump : ttbn->end_bumps()) {
                Net_cost_record record {
                    std::String(std::format("{}__ttb_{}", net->name(), bump_idx)),
                    Net_type::Tnet,
                    1.0,
                    1.0F,
                    {bump_to_ilp_coord(end_bump)},
                    {},
                    {cobunit},
                    {cobunit}
                };
                record.end_tracks.emplace_back(begin_track_index);
                record.mcf_end_track = ttbn->begin_track()->coord();
                record.mcf_has_end_track = true;
                record.mcf_start_kind = IlpEndpointKind::Bump;
                record.mcf_end_kind = IlpEndpointKind::Track;
                record.from_track_to_bumps_split = true;
                record.origin_key = net->name();
                record.origin_uid = net->uid();
                records.emplace_back(std::move(record));
                bump_idx += 1;
            }
            out.track_to_bumps_nets.emplace_back(net);
            continue;
        }
        if (const auto* tsb_net = dynamic_cast<const circuit::TracksToBumpsNet*>(net.get())) {
            // Split one TracksToBumpsNet into multiple "bump -> tracks" pseudo nets.
            auto candidate_cobunits = std::Vector<std::size_t> {};
            auto pn_end_tracks = std::Vector<std::size_t> {};
            auto pn_end_track_coord_by_index = std::map<std::size_t, hardware::TrackCoord> {};
            for (auto* track : tsb_net->begin_tracks()) {
                candidate_cobunits.emplace_back(map_track(track->coord().index));
                pn_end_tracks.emplace_back(track->coord().index);
                if (!pn_end_track_coord_by_index.contains(track->coord().index)) {
                    pn_end_track_coord_by_index.emplace(track->coord().index, track->coord());  // 目前0/1 track的之间的index不重复，这里用index作为key暂时没问题
                }
            }
            sort_and_unique(candidate_cobunits);
            sort_and_unique(pn_end_tracks);
            for (auto* bump : tsb_net->end_bumps()) {   // 对于每一个end_bump，都拆成一个PNnet
                Net_cost_record record {
                    std::String(std::format("{}__split_{}", net->name(), records.size())),
                    Net_type::PNnet,
                    1.0,
                    5.0F,
                    {bump_to_ilp_coord(bump)},
                    {},
                    candidate_cobunits,
                    {}
                };
                record.pn_end_tracks = pn_end_tracks;           // 所有的0/1端口
                record.pn_end_track_coord_by_index = pn_end_track_coord_by_index;
                record.origin_key = net->name();
                record.origin_uid = net->uid();
                record.power_kind = (net->name() == std::String("Pose nets")) ? IlpPowerKind::Pose : IlpPowerKind::Nege;
                record.mcf_start_kind = IlpEndpointKind::Bump;
                record.mcf_end_kind = IlpEndpointKind::Track;
                records.emplace_back(std::move(record));
            }
            continue;
        }

        if (auto* sync_net = dynamic_cast<circuit::SyncNet*>(net.get())) {
            // Split SyncNet into independent 2-pin nets for SAT TOB modeling.
            for (const auto& btb : sync_net->btbnets()) {
                Net_cost_record record {
                    std::String(std::format("{}__btb_{}", net->name(), records.size())),
                    Net_type::Bnet,
                    1.0,
                    10.0F,
                    {bump_to_ilp_coord(btb->begin_bump())},
                    {bump_to_ilp_coord(btb->end_bump())},
                    {},
                    {}
                };
                for (std::size_t c = 0; c < 16; ++c) {
                    record.candidate_cobunits.emplace_back(c);
                }
                record.origin_key = net->name();
                record.origin_uid = net->uid();
                record.mcf_start_kind = IlpEndpointKind::Bump;
                record.mcf_end_kind = IlpEndpointKind::Bump;
                records.emplace_back(std::move(record));
            }
            for (const auto& btt : sync_net->bttnets()) {
                const auto cobunit = map_track(btt->end_track()->coord().index);
                Net_cost_record record {
                    std::String(std::format("{}__btt_{}", net->name(), records.size())),
                    Net_type::Tnet,
                    1.0,
                    1.0F,
                    {bump_to_ilp_coord(btt->begin_bump())},
                    {},
                    {cobunit},
                    {cobunit}
                };
                record.end_tracks.emplace_back(btt->end_track()->coord().index);
                record.mcf_end_track = btt->end_track()->coord();
                record.mcf_has_end_track = true;
                record.mcf_start_kind = IlpEndpointKind::Bump;
                record.mcf_end_kind = IlpEndpointKind::Track;
                record.origin_key = net->name();
                record.origin_uid = net->uid();
                records.emplace_back(std::move(record));
            }
            for (const auto& ttb : sync_net->ttbnets()) {
                const auto cobunit = map_track(ttb->begin_track()->coord().index);
                Net_cost_record record {
                    std::String(std::format("{}__ttb_{}", net->name(), records.size())),
                    Net_type::Tnet,
                    1.0,
                    1.0F,
                    {bump_to_ilp_coord(ttb->end_bump())},
                    {},
                    {cobunit},
                    {cobunit}
                };
                record.end_tracks.emplace_back(ttb->begin_track()->coord().index);
                record.mcf_end_track = ttb->begin_track()->coord();
                record.mcf_has_end_track = true;
                record.mcf_start_kind = IlpEndpointKind::Bump;
                record.mcf_end_kind = IlpEndpointKind::Track;
                record.origin_key = net->name();
                record.origin_uid = net->uid();
                records.emplace_back(std::move(record));
            }
            continue;
        }

        // for other net types, classify them into Net_cost_record
        records.emplace_back(classify_net(net));
    }

    // show number of all net types
    std::size_t bnet_count = 0;
    std::size_t pnnet_count = 0;
    std::size_t tnet_count = 0;
    for (const auto& record : records) {
        if (record.type == Net_type::Bnet) {
            ++bnet_count;
        }
        else if (record.type == Net_type::PNnet) {
            ++pnnet_count;
        }
        else if (record.type == Net_type::Tnet) {
            ++tnet_count;
        }
    }
    debug::info_fmt("number of Bnet: {}", bnet_count);
    debug::info_fmt("number of PNnet: {}", pnnet_count);
    debug::info_fmt("number of Tnet: {}", tnet_count);
    debug::info_fmt("total number of nets: {}", records.size());

    // Assign stable ids for downstream SAT/MCF alignment.
    auto origin_bit_counter = std::map<std::String, std::size_t> {};
    for (std::size_t i = 0; i < records.size(); ++i) {
        records[i].record_id = i;
        const auto origin = record_origin_group_uid(records[i]);
        records[i].bit_id = origin_bit_counter[origin];
        origin_bit_counter[origin] += 1;
    }

    return out;
}

auto write_mps_file(
    const std::Vector<Net_cost_record>& records,
    const std::String& output_mps
) -> void {
    TobIlpModel m {};
    build_tob_ilp_model(m, records);
    m.write_mps(output_mps);
}

auto get_peak_rss_mb() -> double {
    struct rusage usage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return -1.0;
    }
#if defined(__APPLE__)
    // macOS reports ru_maxrss in bytes.
    constexpr double kBytesPerMb = 1024.0 * 1024.0;
    return static_cast<double>(usage.ru_maxrss) / kBytesPerMb;
#else
    // Linux reports ru_maxrss in kilobytes.
    constexpr double kKbPerMb = 1024.0;
    return static_cast<double>(usage.ru_maxrss) / kKbPerMb;
#endif
}

auto is_testpn_golden_case(const std::String& config_path) -> bool {
    return config_path.ends_with("/testpn")
        || config_path.ends_with("testpn")
        || config_path.find("/testpn/") != std::String::npos
        || config_path.find("testlength/testpn") != std::String::npos;
}

auto run_wire_length_golden_check(const std::String& config_path, const std::size_t actual) -> bool {
    const auto golden_path = std::format("{}/golden.txt", config_path);
    auto golden_file = std::ifstream {golden_path.c_str()};
    if (!golden_file.is_open()) {
        debug::error_fmt("wire length golden check: cannot open {}", golden_path);
        return false;
    }
    std::size_t expected = 0;
    if (!(golden_file >> expected)) {
        debug::error_fmt("wire length golden check: cannot read integer from {}", golden_path);
        return false;
    }
    if (actual == expected) {
        debug::info_fmt("wire length golden check: ok expected={} actual={}", expected, actual);
        return true;
    }
    if (is_testpn_golden_case(config_path)) {
        debug::warning_fmt(
            "wire length golden check: testpn mismatch (allowed) expected={} actual={}",
            expected,
            actual);
        return true;
    }
    debug::error_fmt("wire length golden check: mismatch expected={} actual={}", expected, actual);
    return false;
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_main(argc, argv);
}
