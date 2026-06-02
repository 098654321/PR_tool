// Build ILP model from config, solve with Gurobi, optional MPS export.

#include "mcf/cob_mcf_router.hh"
#include "maze_check/maze_check.hh"
#include "ilp_allocation/gurobi.hh"
#include "common/ilp_types.hh"
#include "precompute/ilp_reach_precompute.hh"
#include "precompute/pre_routing_warm_start.hh"
#include "ilp_allocation/tob_ilp_model.hh"

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
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <format>
#include <map>
#include <sys/resource.h>
#include <stdexcept>
#include <string>
#include <tuple>

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

auto run_main(int argc, char** argv) -> int {
    const auto run_begin = std::chrono::steady_clock::now();
    const auto log_total_runtime = [&]() {
        const auto run_end = std::chrono::steady_clock::now();
        const auto run_ms = std::chrono::duration_cast<std::chrono::milliseconds>(run_end - run_begin).count();
        debug::info_fmt("run_main total elapsed: {} ms", run_ms);
    };
    if (argc < 2) {
        debug::error("No config path given");
        debug::info(
            "Usage: xmake run test_ILP <config_path> [output_mps_path] [-v|-vv|...] [--enable-ilp-parallel] "
            "[--cob-rows N --cob-cols M] [--enable-mcf-routing] [--disable-bus-mcf] "
            "[--enable-mcf-parallel] [--enable-mcf-obj] [--enable-pre-routing] "
            "[--maze-check-ilp-mcf | --maze-check-mcf]");
        log_total_runtime();
        return 1;
    }

    const auto config_path = std::String(argv[1]);
    auto output_mps = std::String {};
    bool enable_ilp_parallel = false;
    bool enable_mcf = false;
    bool disable_bus_mcf = false;
    bool enable_mcf_parallel = false;
    bool enable_mcf_obj = false;
    bool enable_pre_routing = false;
    bool maze_check_ilp_mcf = false;
    bool maze_check_mcf = false;
    int verbose_v_count = 0;
    bool cob_rows_set = false;
    bool cob_cols_set = false;
    int cob_rows_cli = 0;
    int cob_cols_cli = 0;
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
        if (arg == "--enable-ilp-parallel") {
            enable_ilp_parallel = true;
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
        if (arg == "--cob-rows") {
            if (argi + 1 >= argc) {
                debug::error("--cob-rows requires an integer argument");
                log_total_runtime();
                return 1;
            }
            cob_rows_cli = std::atoi(argv[argi + 1]);
            cob_rows_set = true;
            argi += 1;
            continue;
        }
        if (arg == "--cob-cols") {
            if (argi + 1 >= argc) {
                debug::error("--cob-cols requires an integer argument");
                log_total_runtime();
                return 1;
            }
            cob_cols_cli = std::atoi(argv[argi + 1]);
            cob_cols_set = true;
            argi += 1;
            continue;
        }
        if (output_mps.empty()) {
            output_mps = arg;
            continue;
        }
        debug::error_fmt("Unexpected argument '{}'", arg);
        debug::info(
            "Usage: xmake run test_ILP <config_path> [output_mps_path] [-v|-vv|...] [--enable-ilp-parallel] "
            "[--cob-rows N --cob-cols M] [--enable-mcf-routing] [--disable-bus-mcf] "
            "[--enable-mcf-parallel] [--enable-mcf-obj] [--enable-pre-routing] "
            "[--maze-check-ilp-mcf | --maze-check-mcf]");
        log_total_runtime();
        return 1;
    }

    const int cob_rows_hw = static_cast<int>(hardware::Interposer::COB_ARRAY_HEIGHT);
    const int cob_cols_hw = static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH);
    int cob_rows = cob_rows_hw;
    int cob_cols = cob_cols_hw;
    if (cob_rows_set != cob_cols_set) {
        debug::error("COB grid: specify both --cob-rows and --cob-cols, or neither (defaults to Interposer dimensions)");
        log_total_runtime();
        return 1;
    }
    if (cob_rows_set) {
        if (cob_rows_cli != cob_rows_hw || cob_cols_cli != cob_cols_hw) {
            debug::error_fmt(
                "COB grid from CLI ({}, {}) must match hardware::Interposer (rows={}, cols={})",
                cob_rows_cli,
                cob_cols_cli,
                cob_rows_hw,
                cob_cols_hw);
            log_total_runtime();
            return 1;
        }
        cob_rows = cob_rows_cli;
        cob_cols = cob_cols_cli;
    }
    const CobMcfGridDims cob_grid {cob_rows, cob_cols};

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
    const bool defer_maze_check_suspend = maze_check_ilp_mcf || maze_check_mcf;

    // read file and build nets
    debug::initial_log("./debug.log");
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
            "Deferred multi-fanout nets: {} — excluded from ILP+MCF (unexpected after build_records)",
            deferred_multi_fanout.size());
    }
    if (!track_to_bumps_nets.empty()) {
        debug::info_fmt(
            "TrackToBumpsNet: {} net(s) split for ILP; COB segment routed in SimpleMCF",
            track_to_bumps_nets.size());
    }

    // precompute reach
    const auto reach_stats = precompute_reach_for_records(records);
    debug::info_fmt(
        "reach precompute: records={} (B={}, T={}, PN={}), total_endtracks={}, total_starttrack_edges={}",
        reach_stats.total_records,
        reach_stats.bnet_records,
        reach_stats.tnet_records,
        reach_stats.pnnet_records,
        reach_stats.total_endtracks,
        reach_stats.total_starttrack_edges);
    if (!output_mps.empty()) {
        write_mps_file(records, output_mps);
        debug::info_fmt("MPS written: {}", output_mps);
    }

    // pre-routing warm start
    auto ilp_warm_start = TobIlpWarmStart {};
    const TobIlpWarmStart* ilp_warm_start_ptr = nullptr;
    long long ilp_warm_start_ms = 0;
    if (enable_pre_routing) {
        const auto ilp_warm_t0 = std::chrono::steady_clock::now();
        ilp_warm_start = build_ilp_warm_start_from_maze(config_path, records);
        const auto ilp_warm_t1 = std::chrono::steady_clock::now();
        ilp_warm_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(ilp_warm_t1 - ilp_warm_t0).count();
        ilp_warm_start_ptr = &ilp_warm_start;
    }
    debug::info_fmt("timing phase=ilp_warm_start ms={}", ilp_warm_start_ms);

    // solve ILP
    const auto solve_begin = std::chrono::steady_clock::now();
    const auto result = solve_tob_ilp_with_gurobi(records, enable_ilp_parallel, ilp_warm_start_ptr);
    const auto solve_end = std::chrono::steady_clock::now();
    const auto ilp_solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_begin).count();
    const auto peak_rss_mb = get_peak_rss_mb();
    debug::info_fmt("timing phase=ilp_solve ms={}", ilp_solve_ms);
    debug::info_fmt("Process peak RSS: {:.2f} MB", peak_rss_mb);

    if (!result.ok) {
        debug::error_fmt("Gurobi: {}", result.message);
        debug::info_fmt(
            "timing breakdown (ms): ilp_warm_start={}, ilp_solve={}, mcf_warm_start={}, mcf_solve={}",
            ilp_warm_start_ms,
            ilp_solve_ms,
            0,
            0);
        log_total_runtime();
        return 1;
    }
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
    debug::info_fmt("nets solved: {}", records.size());

    long long mcf_warm_start_ms = 0;
    long long mcf_solve_ms = 0;
    if (enable_mcf) {
        if (enable_mcf_parallel) {
            debug::info("MCF: solving 16 COB units in parallel (std::async)");
        }
        const auto mcf_full = run_mcf_global_routing_cob_units(
            records,
            result,
            interposer.get(),
            *basedie.get(),
            cob_grid,
            enable_mcf_parallel,
            enable_pre_routing,
            enable_mcf_obj,
            defer_maze_check_suspend,
            disable_bus_mcf);
        mcf_warm_start_ms = mcf_full.summary.mcf_warm_start_ms;
        mcf_solve_ms = mcf_full.summary.mcf_solve_ms;
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
        if (!mcf_full.summary.all_ok) {
            debug::error("MCF global routing: one or more COB unit solves failed; see MCF log lines");
            debug::info_fmt(
                "timing breakdown (ms): ilp_warm_start={}, ilp_solve={}, mcf_warm_start={}, mcf_solve={}",
                ilp_warm_start_ms,
                ilp_solve_ms,
                mcf_warm_start_ms,
                mcf_solve_ms);
            log_total_runtime();
            return 1;
        }
    }
    debug::info_fmt(
        "timing breakdown (ms): ilp_warm_start={}, ilp_solve={}, mcf_warm_start={}, mcf_solve={}",
        ilp_warm_start_ms,
        ilp_solve_ms,
        mcf_warm_start_ms,
        mcf_solve_ms);
    log_total_runtime();
    return 0;
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
            // Split SyncNet into independent 2-pin nets for ILP modeling.
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

    // Assign stable ids for downstream ILP/MCF alignment.
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

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_main(argc, argv);
}
