#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/common/maze/path_length.hh>
#include <circuit/basedie.hh>
#include <circuit/net/net.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/bbsnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/tbnet.hh>
#include <circuit/net/types/tsbsnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/path/pathpackage.hh>
#include <debug/debug.hh>
#include <filesystem>
#include <fstream>
#include <optional>
#include <hardware/bump/bumpcoord.hh>
#include <hardware/interposer.hh>
#include <hardware/track/trackcoord.hh>
#include <parse/reader/module.hh>
#include <parse/writer/module.hh>
#include <regex>
#include <sstream>
#include <std/file.hh>
#include <std/format.hh>

using namespace PR_tool;

namespace {

struct PathSegment {
    std::Option<hardware::BumpCoord> begin_bump {};
    std::Option<hardware::TrackCoord> begin_track {};
    std::Option<hardware::BumpCoord> end_bump {};
    std::Option<hardware::TrackCoord> end_track {};
    std::Vector<hardware::TrackCoord> tracks {};
};

struct ParsedPathBlock {
    int net_index {-1};
    std::Option<hardware::BumpCoord> source_bump {};
    std::Option<hardware::TrackCoord> source_track {};
    std::Vector<hardware::BumpCoord> sink_bumps {};
    std::Vector<hardware::TrackCoord> sink_tracks {};
    // Legacy single-path fields (filled when there is exactly one segment).
    std::Option<hardware::BumpCoord> begin_bump {};
    std::Option<hardware::TrackCoord> begin_track {};
    std::Option<hardware::BumpCoord> end_bump {};
    std::Option<hardware::TrackCoord> end_track {};
    std::Vector<hardware::TrackCoord> tracks {};
    // Power rail nets (nege/pose) emit multiple Printing path... segments.
    std::String power_rail {};  // "", "nege", or "pose"
    std::Vector<PathSegment> segments {};
};

auto parse_i64(const std::String& key, const std::String& text) -> std::i64 {
    std::regex pattern{std::format(R"({}\s*:\s*(-?\d+))", key)};
    std::smatch match;
    if (!std::regex_search(text, match, pattern)) {
        throw std::runtime_error(std::format("missing field '{}' in '{}'", key, text));
    }
    return std::stoll(match[1].str());
}

auto parse_bump_coord(const std::String& line) -> hardware::BumpCoord {
    return hardware::BumpCoord{
        parse_i64("row", line),
        parse_i64("col", line),
        static_cast<std::usize>(parse_i64("index", line)),
    };
}

auto parse_track_coord(const std::String& line) -> hardware::TrackCoord {
    std::regex dir_pattern{R"(dir\s*:\s*(Vertical|Horizontal))"};
    std::smatch match;
    if (!std::regex_search(line, match, dir_pattern)) {
        throw std::runtime_error(std::format("missing track dir in '{}'", line));
    }
    auto dir = match[1].str() == "Vertical" ? hardware::TrackDirection::Vertical
                                            : hardware::TrackDirection::Horizontal;
    return hardware::TrackCoord{
        parse_i64("row", line),
        parse_i64("col", line),
        dir,
        static_cast<std::usize>(parse_i64("index", line)),
    };
}

auto is_bump_line(const std::String& line) -> bool {
    return line.find("index:") != std::String::npos && line.find("dir:") == std::String::npos;
}

auto is_track_line(const std::String& line) -> bool {
    return line.find("dir:") != std::String::npos;
}

auto parse_path_file(const std::FilePath& path) -> std::Vector<ParsedPathBlock> {
    std::ifstream in{path};
    if (!in.is_open()) {
        throw std::runtime_error(std::format("cannot open path file '{}'", path.string()));
    }

    std::Vector<ParsedPathBlock> blocks {};
    ParsedPathBlock current {};
    PathSegment current_seg {};
    bool have_seg = false;
    std::String line {};

    auto flush_segment = [&]() {
        if (!have_seg) {
            return;
        }
        if (current_seg.tracks.empty() && !current_seg.begin_bump.has_value()
            && !current_seg.begin_track.has_value()) {
            current_seg = PathSegment{};
            have_seg = false;
            return;
        }
        current.segments.emplace_back(current_seg);
        current_seg = PathSegment{};
        have_seg = false;
    };

    auto finalize_legacy_fields = [&]() {
        if (current.segments.size() == 1) {
            const auto& seg = current.segments.front();
            current.begin_bump = seg.begin_bump;
            current.begin_track = seg.begin_track;
            current.end_bump = seg.end_bump;
            current.end_track = seg.end_track;
            current.tracks = seg.tracks;
            if (seg.begin_track.has_value() && !current.source_track.has_value()
                && current.power_rail.empty()) {
                current.source_track = seg.begin_track;
            }
        } else if (current.segments.size() > 1) {
            for (const auto& seg : current.segments) {
                current.tracks.insert(current.tracks.end(), seg.tracks.begin(), seg.tracks.end());
            }
        }
    };

    auto flush_block = [&]() {
        flush_segment();
        if (current.net_index >= 0) {
            finalize_legacy_fields();
            blocks.emplace_back(current);
        }
        current = ParsedPathBlock{};
        current_seg = PathSegment{};
        have_seg = false;
    };

    while (std::getline(in, line)) {
        if (line.find("Net index ") != std::String::npos) {
            flush_block();
            std::regex header{R"(Net index (\d+))"};
            std::smatch match;
            if (std::regex_search(line, match, header)) {
                current.net_index = std::stoi(match[1].str());
            }
            continue;
        }
        if (line.find("Source:") != std::String::npos) {
            if (line.find("endpoint:") != std::String::npos) {
                if (line.find("nege") != std::String::npos) {
                    current.power_rail = "nege";
                } else if (line.find("pose") != std::String::npos) {
                    current.power_rail = "pose";
                }
            } else if (is_track_line(line)) {
                current.source_track = parse_track_coord(line);
            } else {
                current.source_bump = parse_bump_coord(line);
            }
            continue;
        }
        if (line.find("Sink") != std::String::npos && line.find('{') != std::String::npos) {
            if (is_track_line(line)) {
                current.sink_tracks.emplace_back(parse_track_coord(line));
            } else if (line.find("endpoint:") == std::String::npos) {
                current.sink_bumps.emplace_back(parse_bump_coord(line));
            }
            continue;
        }
        if (line.find("Printing path...") != std::String::npos) {
            flush_segment();
            have_seg = true;
            continue;
        }
        if (line.find("Begin_bump:") != std::String::npos) {
            have_seg = true;
            current_seg.begin_bump = parse_bump_coord(line);
            continue;
        }
        if (line.find("Begin_track:") != std::String::npos) {
            have_seg = true;
            current_seg.begin_track = parse_track_coord(line);
            continue;
        }
        if (line.find("End_bump:") != std::String::npos) {
            current_seg.end_bump = parse_bump_coord(line);
            flush_segment();
            continue;
        }
        if (line.find("End_track:") != std::String::npos) {
            current_seg.end_track = parse_track_coord(line);
            flush_segment();
            continue;
        }
        if (line.find('{') != std::String::npos) {
            if (is_track_line(line)) {
                have_seg = true;
                current_seg.tracks.emplace_back(parse_track_coord(line));
            } else if (is_bump_line(line)) {
                // ignore standalone bump coord lines in metadata
            }
        }
    }
    flush_block();
    return blocks;
}

auto bump_ptr(hardware::Interposer* interposer, const hardware::BumpCoord& coord)
    -> hardware::Bump* {
    auto bump = interposer->get_bump(coord.row / 2, coord.col / 3, coord.index);
    if (!bump.has_value()) {
        throw std::runtime_error(std::format("bump not found: {}", coord.to_string()));
    }
    return bump.value();
}

auto track_ptr(hardware::Interposer* interposer, const hardware::TrackCoord& coord)
    -> hardware::Track* {
    auto track = interposer->get_track(coord);
    if (!track.has_value()) {
        throw std::runtime_error(std::format("track not found: {}", coord.to_string()));
    }
    return track.value();
}

auto find_cob_info(hardware::Interposer* interposer, hardware::Track* from,
                   hardware::Track* to) -> std::Option<circuit::COBConnectorInfo> {
    for (auto& [adj_track, connector] : interposer->adjacent_tracks(from)) {
        if (adj_track == to) {
            return circuit::COBConnectorInfo{
                connector.coord(),
                connector.from_dir(),
                connector.from_track_index(),
                connector.to_dir(),
                connector.to_track_index(),
            };
        }
    }
    return std::nullopt;
}

auto tob_info_from(hardware::Bump* bump, const hardware::TOBConnector& connector)
    -> circuit::TOBConnectorInfo {
    return circuit::TOBConnectorInfo{
        connector.bump_index(),
        connector.hori_index(),
        connector.vert_index(),
        connector.track_index(),
        connector.single_direction(),
        bump->tob()->coord(),
    };
}

auto build_history_package(hardware::Interposer* interposer, const ParsedPathBlock& block)
    -> circuit::HistoryPathPackage {
    circuit::PathPackage empty_package {};
    circuit::HistoryPathPackage history{empty_package};
    history.clear_all();

    auto append_track_chain = [&](const std::Vector<hardware::TrackCoord>& tracks) {
        if (tracks.empty()) {
            return;
        }
        auto track_ptrs = std::Vector<hardware::Track*> {};
        for (const auto& coord : tracks) {
            track_ptrs.emplace_back(track_ptr(interposer, coord));
        }
        for (std::usize i = 0; i < track_ptrs.size(); ++i) {
            std::Option<circuit::COBConnectorInfo> connector_info {std::nullopt};
            if (i + 1 < track_ptrs.size()) {
                connector_info =
                    find_cob_info(interposer, track_ptrs[i], track_ptrs[i + 1]);
                if (!connector_info.has_value()) {
                    throw std::runtime_error(std::format(
                        "missing COB connector between {} and {}",
                        track_ptrs[i]->coord().to_string(), track_ptrs[i + 1]->coord().to_string()));
                }
            }
            history._regular_path.emplace_back(track_ptrs[i]->coord(), connector_info);
        }
        history._length += algo::path_length(track_ptrs);
    };

    auto append_begin_bump = [&](const hardware::BumpCoord& bump_coord,
                                 const hardware::TrackCoord& first_track) {
        auto* bump = bump_ptr(interposer, bump_coord);
        auto* first = track_ptr(interposer, first_track);
        auto tracks_map = interposer->available_tracks_bump_to_track(bump, true);
        auto iter = tracks_map.find(first);
        if (iter == tracks_map.end()) {
            throw std::runtime_error(std::format(
                "cannot find bump_to_track connector for bump {} -> track {}",
                bump->coord().to_string(), first->coord().to_string()));
        }
        history._tob_to_track.emplace_back(
            bump->coord(), tob_info_from(bump, iter->second), first->coord());
        history._length += 1;
    };

    auto append_end_bump = [&](const hardware::BumpCoord& bump_coord,
                               const hardware::TrackCoord& last_track) {
        auto* bump = bump_ptr(interposer, bump_coord);
        auto* last = track_ptr(interposer, last_track);
        auto tracks_map = interposer->available_tracks_track_to_bump(bump, true);
        auto iter = tracks_map.find(last);
        if (iter == tracks_map.end()) {
            throw std::runtime_error(std::format(
                "cannot find track_to_bump connector for track {} -> bump {}",
                last->coord().to_string(), bump->coord().to_string()));
        }
        history._track_to_tob.emplace_back(
            bump->coord(), tob_info_from(bump, iter->second), last->coord());
        history._length += 1;
    };

    const auto& segments = block.segments;
    if (!segments.empty()) {
        for (const auto& seg : segments) {
            if (seg.tracks.empty()) {
                throw std::runtime_error(std::format(
                    "net {} has empty track list in a path segment", block.net_index));
            }
            append_track_chain(seg.tracks);
            if (seg.begin_bump.has_value()) {
                append_begin_bump(seg.begin_bump.value(), seg.tracks.front());
            }
            if (seg.end_bump.has_value()) {
                append_end_bump(seg.end_bump.value(), seg.tracks.back());
            }
        }
        return history;
    }

    if (block.tracks.empty()) {
        throw std::runtime_error(std::format("net {} has empty track list", block.net_index));
    }
    append_track_chain(block.tracks);
    if (block.begin_bump.has_value()) {
        append_begin_bump(block.begin_bump.value(), block.tracks.front());
    }
    if (block.end_bump.has_value()) {
        append_end_bump(block.end_bump.value(), block.tracks.back());
    }
    return history;
}

auto append_leaf_nets(circuit::Net* net, std::Vector<circuit::Net*>& out) -> void {
    if (auto* sync = dynamic_cast<circuit::SyncNet*>(net)) {
        for (auto& sub : sync->btbnets()) {
            out.emplace_back(sub.get());
        }
        for (auto& sub : sync->bttnets()) {
            out.emplace_back(sub.get());
        }
        for (auto& sub : sync->ttbnets()) {
            out.emplace_back(sub.get());
        }
        return;
    }
    out.emplace_back(net);
}

auto flatten_connection_nets(const std::Vector<std::Rc<circuit::Net>>& nets)
    -> std::Vector<circuit::Net*> {
    std::Vector<circuit::Net*> flat {};
    for (auto& net_rc : nets) {
        append_leaf_nets(net_rc.get(), flat);
    }
    return flat;
}

auto match_block_to_net(const ParsedPathBlock& block, circuit::Net* net) -> bool {
    if (auto* bb = dynamic_cast<circuit::BumpToBumpNet*>(net)) {
        if (!block.source_bump.has_value() || block.sink_bumps.empty()) {
            return false;
        }
        return bb->begin_bump()->coord() == block.source_bump.value()
               && bb->end_bump()->coord() == block.sink_bumps.front();
    }
    if (auto* bbs = dynamic_cast<circuit::BumpToBumpsNet*>(net)) {
        if (!block.source_bump.has_value() || block.sink_bumps.empty()) {
            return false;
        }
        if (bbs->begin_bump()->coord() != block.source_bump.value()) {
            return false;
        }
        if (bbs->end_bumps().size() != block.sink_bumps.size()) {
            return false;
        }
        for (std::usize i = 0; i < block.sink_bumps.size(); ++i) {
            if (bbs->end_bumps()[i]->coord() != block.sink_bumps[i]) {
                return false;
            }
        }
        return true;
    }
    if (auto* ttb = dynamic_cast<circuit::TrackToBumpNet*>(net)) {
        if (!block.source_track.has_value() || block.sink_bumps.empty()) {
            return false;
        }
        return ttb->begin_track()->coord() == block.source_track.value()
               && ttb->end_bump()->coord() == block.sink_bumps.front();
    }
    if (auto* btt = dynamic_cast<circuit::BumpToTrackNet*>(net)) {
        if (!block.source_bump.has_value() || block.sink_tracks.empty()) {
            return false;
        }
        return btt->begin_bump()->coord() == block.source_bump.value()
               && btt->end_track()->coord() == block.sink_tracks.front();
    }
    if (auto* tsbs = dynamic_cast<circuit::TracksToBumpsNet*>(net)) {
        if (block.power_rail.empty() || block.sink_bumps.empty()) {
            return false;
        }
        const auto& name = tsbs->name();
        if (block.power_rail == "nege" && name.find("Nege") == std::String::npos
            && name.find("nege") == std::String::npos) {
            return false;
        }
        if (block.power_rail == "pose" && name.find("Pose") == std::String::npos
            && name.find("pose") == std::String::npos) {
            return false;
        }
        if (tsbs->end_bumps().size() != block.sink_bumps.size()) {
            return false;
        }
        // Order of sinks in path vs NetBuilder may differ; match as a set.
        std::HashSet<hardware::BumpCoord> want {};
        for (const auto& c : block.sink_bumps) {
            want.emplace(c);
        }
        for (auto* bump : tsbs->end_bumps()) {
            if (!want.contains(bump->coord())) {
                return false;
            }
        }
        return true;
    }
    throw std::runtime_error(std::format("unsupported net type for writer test: {}", net->name()));
}

auto usage() -> void {
    debug::info(
        "usage: module_test writer <config_folder> <net_path_info_new.txt> <output_dir> [mode] "
        "[-s|--simplify-controlbits-file] [--simplified-output-dir <dir>]");
}

}  // namespace

void test_writer_main(int argc, char** argv) {
    if (argc < 5) {
        usage();
        throw std::runtime_error("writer test requires config_folder, path file, output_dir");
    }

    bool simplify_controlbits = false;
    std::optional<std::FilePath> simplified_output_dir {};
    std::Vector<std::StringView> positional {};
    for (int i = 2; i < argc; ++i) {
        auto arg = std::StringView{argv[i]};
        if (arg == "-s" || arg == "--simplify-controlbits-file") {
            simplify_controlbits = true;
            continue;
        }
        if (arg == "--simplified-output-dir") {
            if (i + 1 >= argc) {
                throw std::runtime_error("missing path after --simplified-output-dir");
            }
            simplified_output_dir = std::FilePath{argv[++i]};
            continue;
        }
        positional.emplace_back(arg);
    }

    if (positional.size() < 3) {
        usage();
        throw std::runtime_error("writer test requires config_folder, path file, output_dir");
    }

    auto config_folder = std::FilePath{positional[0]};
    auto path_file = std::FilePath{positional[1]};
    auto output_dir = std::FilePath{positional[2]};
    int mode = (positional.size() >= 4) ? std::stoi(std::String{positional[3]}) : 0;

    debug::info_fmt("test_writer: config='{}' path='{}' out='{}' mode={}",
                    config_folder.string(), path_file.string(), output_dir.string(), mode);

    auto blocks = parse_path_file(path_file);

    auto [interposer_box, basedie_box, register_map] = parse::read_config(config_folder, mode, false);
    auto* interposer = interposer_box.get();
    auto* basedie = basedie_box.get();
    algo::build_nets(basedie, interposer);

    auto nets = basedie->nets(mode);
    auto leaf_nets = flatten_connection_nets(nets);

    if (blocks.size() != leaf_nets.size()) {
        debug::warning_fmt("path block count {} != leaf net count {}", blocks.size(),
                           leaf_nets.size());
    }

    std::Vector<bool> block_used(blocks.size(), false);
    std::Vector<std::Pair<circuit::Net*, circuit::HistoryPathPackage>> pending {};

    for (auto* net : leaf_nets) {
        ParsedPathBlock const* matched = nullptr;
        std::usize matched_index = 0;
        for (std::usize i = 0; i < blocks.size(); ++i) {
            if (block_used[i]) {
                continue;
            }
            if (match_block_to_net(blocks[i], net)) {
                matched = &blocks[i];
                matched_index = i;
                break;
            }
        }
        if (matched == nullptr) {
            throw std::runtime_error(std::format("no path block matched net '{}'", net->name()));
        }
        block_used[matched_index] = true;
        pending.emplace_back(net, build_history_package(interposer, *matched));
    }

    for (auto& [net, history] : pending) {
        circuit::PathPackage package{history, interposer};
        package.occupy_all();
        net->set_pathpackage(package);
    }

    // Golden path may list more ExtIO blocks than leaf nets from connections.json.
    // Apply leftovers so COB/TOB bits match golden (connect before matched nets so
    // connect_registers overwrites shared bump dirs in net order).
    std::usize unused_blocks_applied = 0;
    for (std::usize i = 0; i < blocks.size(); ++i) {
        if (block_used[i]) {
            continue;
        }
        circuit::PathPackage package{build_history_package(interposer, blocks[i]), interposer};
        package.occupy_all();
        package.connect_all();
        ++unused_blocks_applied;
    }
    if (unused_blocks_applied > 0) {
        debug::warning_fmt(
            "applied {} unused path blocks not matched to leaf nets", unused_blocks_applied);
    }

    for (auto& net_rc : nets) {
        if (auto* sync = dynamic_cast<circuit::SyncNet*>(net_rc.get())) {
            sync->collect_package();
        }
    }

    std::filesystem::create_directories(output_dir);
    if (simplified_output_dir.has_value()) {
        std::filesystem::create_directories(*simplified_output_dir);
        parse::connect_registers(interposer, basedie, mode);
        parse::write_control_bits_pair(interposer, output_dir, *simplified_output_dir, mode, register_map);
    } else {
        parse::output_from_routing_results(interposer, output_dir, basedie, mode, false, simplify_controlbits, register_map);
    }
}
