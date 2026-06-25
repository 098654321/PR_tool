#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/common/maze/path_length.hh>
#include <circuit/basedie.hh>
#include <circuit/net/net.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/bbsnet.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/tbnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <circuit/path/pathpackage.hh>
#include <debug/debug.hh>
#include <filesystem>
#include <fstream>
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

struct ParsedPathBlock {
    int net_index {-1};
    std::Option<hardware::BumpCoord> source_bump {};
    std::Option<hardware::TrackCoord> source_track {};
    std::Vector<hardware::BumpCoord> sink_bumps {};
    std::Vector<hardware::TrackCoord> sink_tracks {};
    std::Option<hardware::BumpCoord> begin_bump {};
    std::Option<hardware::TrackCoord> begin_track {};
    std::Option<hardware::BumpCoord> end_bump {};
    std::Option<hardware::TrackCoord> end_track {};
    std::Vector<hardware::TrackCoord> tracks {};
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
    std::String line {};

    auto flush_block = [&]() {
        if (current.net_index >= 0) {
            blocks.emplace_back(current);
        }
        current = ParsedPathBlock{};
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
            if (is_track_line(line)) {
                current.source_track = parse_track_coord(line);
            } else {
                current.source_bump = parse_bump_coord(line);
            }
            continue;
        }
        if (line.find("Sink") != std::String::npos && line.find('{') != std::String::npos) {
            if (is_track_line(line)) {
                current.sink_tracks.emplace_back(parse_track_coord(line));
            } else {
                current.sink_bumps.emplace_back(parse_bump_coord(line));
            }
            continue;
        }
        if (line.find("Begin_bump:") != std::String::npos) {
            current.begin_bump = parse_bump_coord(line);
            continue;
        }
        if (line.find("Begin_track:") != std::String::npos) {
            current.begin_track = parse_track_coord(line);
            continue;
        }
        if (line.find("End_bump:") != std::String::npos) {
            current.end_bump = parse_bump_coord(line);
            continue;
        }
        if (line.find("End_track:") != std::String::npos) {
            current.end_track = parse_track_coord(line);
            continue;
        }
        if (line.find('{') != std::String::npos) {
            if (is_track_line(line)) {
                current.tracks.emplace_back(parse_track_coord(line));
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
    if (block.tracks.empty()) {
        throw std::runtime_error(std::format("net {} has empty track list", block.net_index));
    }

    auto track_ptrs = std::Vector<hardware::Track*> {};
    for (const auto& coord : block.tracks) {
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

    history._length = algo::path_length(track_ptrs);

    if (block.begin_bump.has_value()) {
        auto* bump = bump_ptr(interposer, block.begin_bump.value());
        auto tracks_map = interposer->available_tracks_bump_to_track(bump, true);
        auto iter = tracks_map.find(track_ptrs.front());
        if (iter == tracks_map.end()) {
            throw std::runtime_error(std::format(
                "cannot find bump_to_track connector for bump {} -> track {}",
                bump->coord().to_string(), track_ptrs.front()->coord().to_string()));
        }
        history._tob_to_track.emplace_back(
            bump->coord(), tob_info_from(bump, iter->second), track_ptrs.front()->coord());
        history._length += 1;
    }

    if (block.end_bump.has_value()) {
        auto* bump = bump_ptr(interposer, block.end_bump.value());
        auto tracks_map = interposer->available_tracks_track_to_bump(bump, true);
        auto iter = tracks_map.find(track_ptrs.back());
        if (iter == tracks_map.end()) {
            throw std::runtime_error(std::format(
                "cannot find track_to_bump connector for track {} -> bump {}",
                track_ptrs.back()->coord().to_string(), bump->coord().to_string()));
        }
        history._track_to_tob.emplace_back(
            bump->coord(), tob_info_from(bump, iter->second), track_ptrs.back()->coord());
        history._length += 1;
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
    throw std::runtime_error(std::format("unsupported net type for writer test: {}", net->name()));
}

auto usage() -> void {
    debug::info(
        "usage: module_test writer <config_folder> <net_path_info_new.txt> <output_dir> [mode] [-s|--simplify-controlbits-file]");
}

}  // namespace

void test_writer_main(int argc, char** argv) {
    if (argc < 5) {
        usage();
        throw std::runtime_error("writer test requires config_folder, path file, output_dir");
    }

    bool simplify_controlbits = false;
    std::Vector<std::StringView> positional {};
    for (int i = 2; i < argc; ++i) {
        auto arg = std::StringView{argv[i]};
        if (arg == "-s" || arg == "--simplify-controlbits-file") {
            simplify_controlbits = true;
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

    auto [interposer_box, basedie_box] = parse::read_config(config_folder, mode, false);
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

    for (auto& net_rc : nets) {
        if (auto* sync = dynamic_cast<circuit::SyncNet*>(net_rc.get())) {
            sync->collect_package();
        }
    }

    std::filesystem::create_directories(output_dir);
    parse::output_from_routing_results(interposer, output_dir, basedie, mode, false, simplify_controlbits);
}
