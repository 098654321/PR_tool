// Wirelength study: ILP → MCF rank-1 → k-shortest path comparison (2-pin Yen / TTB no-good MCF).

#include "mcf/cob_mcf_router.hh"
#include "mcf/mcf_graph.hh"
#include "ilp_allocation/gurobi.hh"
#include "common/ilp_types.hh"
#include "precompute/ilp_reach_precompute.hh"

#include <algo/netbuilder/netbuilder.hh>
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
#include <cstdlib>
#include <format>
#include <map>
#include <stdexcept>

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

auto detect_study_mode(const std::Vector<Net_cost_record>& records) -> std::String {
    std::size_t bnet_count = 0;
    std::size_t ttb_split_count = 0;
    for (const auto& rec : records) {
        if (rec.type == Net_type::Bnet) {
            ++bnet_count;
        }
        if (rec.from_track_to_bumps_split) {
            ++ttb_split_count;
        }
    }
    if (records.size() == 1 && bnet_count == 1) {
        return "2pin";
    }
    if (ttb_split_count > 0) {
        return "ttb";
    }
    throw std::runtime_error("wirelength study: unsupported case (need single Bnet or TrackToBumpsNet split)");
}

auto find_mcf_path_for_record(
    const CobMcfFullResult& mcf,
    const std::size_t record_id
) -> const McfPathInfo* {
    for (std::size_t u = 0; u < 16; ++u) {
        for (const auto& info : mcf.paths_by_unit[u]) {
            if (info.record_id == record_id) {
                return &info;
            }
        }
    }
    return nullptr;
}

auto log_2pin_study(
    const WirelengthStudy2PinResult& study,
    const McfPathInfo& mcf_info
) -> void {
    debug::info_fmt(
        "wirelength study mode=2pin paths_found={}",
        study.k_shortest.size());
    debug::info_fmt(
        "  rank=1 mcf_physical_edges={} yen_physical_edges={} match={}",
        study.mcf_rank1.physical_edges,
        study.k_shortest.empty() ? -1 : study.k_shortest.front().physical_edges,
        study.rank1_mcf_matches_yen ? "yes" : "no");
    debug::info_fmt("  mcf path: {}", study.mcf_rank1.path_text);
    if (!study.k_shortest.empty()) {
        debug::info_fmt("  yen rank1 path: {}", study.k_shortest.front().path_text);
    }
    int prev = study.mcf_rank1.physical_edges;
    for (std::size_t i = 1; i < study.k_shortest.size(); ++i) {
        const auto& r = study.k_shortest[i];
        debug::info_fmt(
            "  rank={} physical_edges={} arc_count={} delta=+{}",
            r.rank,
            r.physical_edges,
            r.arc_count,
            r.physical_edges - prev);
        debug::info_fmt("    path: {}", r.path_text);
        prev = r.physical_edges;
    }
    (void)mcf_info;
}

auto ttb_origin_key(const std::Vector<Net_cost_record>& records) -> std::String {
    for (const auto& rec : records) {
        if (rec.from_track_to_bumps_split) {
            return rec.origin_key.empty() ? rec.net_name : rec.origin_key;
        }
    }
    throw std::runtime_error("wirelength study: no TTB origin found");
}

auto log_ttb_study(const WirelengthStudyTtbResult& study) -> void {
    debug::info_fmt(
        "wirelength study mode=ttb rank1_total_edges={:.0f} commodities={}",
        study.mcf_rank1.total_physical_edges,
        study.mcf_rank1.per_commodity_paths.size());
    double prev = study.mcf_rank1.total_physical_edges;
    for (const auto& alt : study.alternates) {
        debug::info_fmt(
            "  rank={} total_physical_edges={:.0f} delta=+{:.0f} commodities={}",
            alt.rank,
            alt.total_physical_edges,
            alt.total_physical_edges - prev,
            alt.per_commodity_paths.size());
        prev = alt.total_physical_edges;
    }
}

auto run_wirelength_study_main(int argc, char** argv) -> int {
    if (argc < 2) {
        debug::error("No config path given");
        debug::info("Usage: wirelength_study <config_path> [--k=N] [--cob-rows N --cob-cols M] [-v|-vv|...]");
        return 1;
    }

    const auto config_path = std::String(argv[1]);
    int k_paths = 5;
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
                verbose_v_count += static_cast<int>(arg.size()) - 1;
                continue;
            }
        }
        if (arg.rfind("--k=", 0) == 0) {
            k_paths = std::stoi(arg.substr(4));
            continue;
        }
        if (arg == "--cob-rows") {
            cob_rows_set = true;
            if (argi + 1 >= argc) {
                debug::error("--cob-rows requires a value");
                return 1;
            }
            cob_rows_cli = std::stoi(argv[++argi]);
            continue;
        }
        if (arg == "--cob-cols") {
            cob_cols_set = true;
            if (argi + 1 >= argc) {
                debug::error("--cob-cols requires a value");
                return 1;
            }
            cob_cols_cli = std::stoi(argv[++argi]);
            continue;
        }
        debug::warning_fmt("unknown argument: {}", arg);
    }

    if (cob_rows_set != cob_cols_set) {
        debug::error("--cob-rows and --cob-cols must be used together");
        return 1;
    }

    debug::initial_log("./debug.log");
    if (verbose_v_count > 0) {
        debug::set_debug_level(debug::DebugLevel::Debug);
    }

    auto [interposer, basedie] = parse::read_config(config_path, 0, false);
    algo::build_nets(basedie.get(), interposer.get());

    const auto nets = basedie->nets_to_vector();
    auto built = build_records(nets);
    auto records = std::move(built.records);

    const auto reach_stats = precompute_reach_for_records(records);
    debug::info_fmt(
        "reach precompute: records={} (B={}, T={}, PN={})",
        reach_stats.total_records,
        reach_stats.bnet_records,
        reach_stats.tnet_records,
        reach_stats.pnnet_records);

    const auto ilp_result = solve_tob_ilp_with_gurobi(records, false, nullptr);
    if (!ilp_result.ok) {
        debug::error_fmt("ILP failed: {}", ilp_result.message);
        return 1;
    }

    CobMcfGridDims cob_grid {};
    cob_grid.rows = hardware::Interposer::COB_ARRAY_HEIGHT;
    cob_grid.cols = hardware::Interposer::COB_ARRAY_WIDTH;
    if (cob_rows_set) {
        if (cob_rows_cli != cob_grid.rows || cob_cols_cli != cob_grid.cols) {
            debug::error_fmt(
                "cob grid mismatch: CLI {}x{} vs Interposer {}x{}",
                cob_rows_cli,
                cob_cols_cli,
                cob_grid.rows,
                cob_grid.cols);
            return 1;
        }
    }

    const auto mcf_result = run_mcf_global_routing_cob_units(
        records,
        ilp_result,
        interposer.get(),
        *basedie,
        cob_grid,
        false,
        false,
        true,
        true,
        true);

    if (!mcf_result.summary.all_ok) {
        debug::error("MCF failed; cannot run wirelength study");
        return 1;
    }

    const auto mode = detect_study_mode(records);
    auto graph = build_mcf_track_graph(cob_grid);

    debug::info_fmt("wirelength study config={} mode={} k={}", config_path, mode, k_paths);

    if (mode == "2pin") {
        const auto& rec = records.front();
        const auto* mcf_path = find_mcf_path_for_record(mcf_result, rec.record_id);
        if (mcf_path == nullptr || mcf_path->unit_paths.empty()) {
            debug::error("MCF path missing for 2-pin record");
            return 1;
        }
        const auto mcf_node_path = mcf_path->unit_paths.front();
        const auto study = run_wirelength_study_2pin(
            graph,
            mcf_path->src,
            mcf_path->snk,
            0,
            mcf_path->cob_unit,
            k_paths,
            &mcf_node_path);
        log_2pin_study(study, *mcf_path);
    }
    else {
        const auto origin = ttb_origin_key(records);
        const auto study = run_wirelength_study_ttb_origin(
            graph,
            records,
            ilp_result,
            cob_grid,
            origin,
            k_paths);
        log_ttb_study(study);
    }

    debug::info("wirelength study complete");
    return 0;
}

// --- build_records (from test_ILP/main.cc) ---

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
                "unsupported multi-fanout net BumpToBumpsNet '{}' (only TrackToBumpsNet is supported)",
                net->name()));
        }
        if (dynamic_cast<const circuit::BumpToTracksNet*>(net.get()) != nullptr) {
            throw std::runtime_error(std::format(
                "unsupported multi-fanout net BumpToTracksNet '{}'",
                net->name()));
        }
        if (const auto* ttbn = dynamic_cast<const circuit::TrackToBumpsNet*>(net.get())) {
            const auto cobunit = map_track(ttbn->begin_track()->coord().index);
            const auto begin_track_index = ttbn->begin_track()->coord().index;
            std::size_t bump_idx = 0;
            for (auto* end_bump : ttbn->end_bumps()) {
                Net_cost_record record {
                    std::format("{}__ttb_{}", net->name(), bump_idx),
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
        if (dynamic_cast<const circuit::TracksToBumpsNet*>(net.get()) != nullptr) {
            throw std::runtime_error("TracksToBumpsNet not supported in wirelength study");
        }
        if (dynamic_cast<circuit::SyncNet*>(net.get()) != nullptr) {
            throw std::runtime_error("SyncNet not supported in wirelength study");
        }
        records.emplace_back(classify_net(net));
    }

    auto origin_bit_counter = std::map<std::String, std::size_t> {};
    for (std::size_t i = 0; i < records.size(); ++i) {
        records[i].record_id = i;
        const auto origin = record_origin_group_uid(records[i]);
        records[i].bit_id = origin_bit_counter[origin];
        origin_bit_counter[origin] += 1;
    }
    return out;
}

} // namespace PR_tool

auto main(int argc, char** argv) -> int {
    return PR_tool::run_wirelength_study_main(argc, argv);
}
