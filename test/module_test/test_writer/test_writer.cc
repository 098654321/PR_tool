#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/common/maze/path_length.hh>
#include <circuit/basedie.hh>
#include <circuit/net/net.hh>
#include <circuit/net/types/bbnet.hh>
#include <circuit/net/types/bbsnet.hh>
#include <circuit/path/pathpackage.hh>
#include <debug/debug.hh>
#include <chrono>
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

constexpr auto kLogPath =
    "/Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool/.cursor/debug-8edc9c.log";

// #region agent log
auto agent_log(const char* hypothesis_id, const char* location, const char* message,
               const std::String& data_json) -> void {
    std::ofstream log{kLogPath, std::ios::app};
    if (!log.is_open()) {
        return;
    }
    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
    log << std::format(
        R"({{"sessionId":"8edc9c","runId":"test_writer","hypothesisId":"{}","location":"{}","message":"{}","data":{},"timestamp":{}}})",
        hypothesis_id, location, message, data_json, ts)
        << '\n';
}
// #endregion

struct ParsedPathBlock {
    int net_index {-1};
    std::Option<hardware::BumpCoord> source_bump {};
    std::Vector<hardware::BumpCoord> sink_bumps {};
    std::Option<hardware::BumpCoord> begin_bump {};
    std::Option<hardware::BumpCoord> end_bump {};
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
            current.source_bump = parse_bump_coord(line);
            continue;
        }
        if (line.find("Sink") != std::String::npos && line.find('{') != std::String::npos) {
            current.sink_bumps.emplace_back(parse_bump_coord(line));
            continue;
        }
        if (line.find("Begin_bump:") != std::String::npos) {
            current.begin_bump = parse_bump_coord(line);
            continue;
        }
        if (line.find("End_bump:") != std::String::npos) {
            current.end_bump = parse_bump_coord(line);
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
                // #region agent log
                std::String adj_json = "[";
                bool first = true;
                for (auto& [adj_track, _connector] : interposer->adjacent_tracks(track_ptrs[i])) {
                    if (!first) adj_json += ",";
                    first = false;
                    adj_json += std::format(R"("{}")", adj_track->coord().to_string());
                }
                adj_json += "]";
                agent_log("E", "test_writer.cc:cob", "missing COB connector",
                          std::format(
                              R"({{"net_index":{},"from":"{}","to":"{}","adjacent":{}}})",
                              block.net_index, track_ptrs[i]->coord().to_string(),
                              track_ptrs[i + 1]->coord().to_string(), adj_json));
                // #endregion
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

auto net_endpoint_bumps(circuit::Net* net)
    -> std::Pair<hardware::BumpCoord, std::Vector<hardware::BumpCoord>> {
    if (auto* bb = dynamic_cast<circuit::BumpToBumpNet*>(net)) {
        return {
            bb->begin_bump()->coord(),
            std::Vector<hardware::BumpCoord>{bb->end_bump()->coord()},
        };
    }
    if (auto* bbs = dynamic_cast<circuit::BumpToBumpsNet*>(net)) {
        std::Vector<hardware::BumpCoord> sinks {};
        for (auto* bump : bbs->end_bumps()) {
            sinks.emplace_back(bump->coord());
        }
        return {bbs->begin_bump()->coord(), sinks};
    }
    throw std::runtime_error(std::format("unsupported net type for writer test: {}", net->name()));
}

auto match_block_to_net(const ParsedPathBlock& block, circuit::Net* net) -> bool {
    auto [begin, sinks] = net_endpoint_bumps(net);
    if (sinks.empty()) {
        return false;
    }
    if (!block.source_bump.has_value() || block.sink_bumps.empty()) {
        return false;
    }
    return begin == block.source_bump.value() && sinks.front() == block.sink_bumps.front();
}

auto usage() -> void {
    debug::info(
        "usage: module_test writer <config_folder> <net_path_info_new.txt> <output_dir> [mode]");
}

}  // namespace

void test_writer_main(int argc, char** argv) {
    if (argc < 5) {
        usage();
        throw std::runtime_error("writer test requires config_folder, path file, output_dir");
    }

    auto config_folder = std::FilePath{argv[2]};
    auto path_file = std::FilePath{argv[3]};
    auto output_dir = std::FilePath{argv[4]};
    int mode = (argc >= 6) ? std::stoi(argv[5]) : 0;

    debug::info_fmt("test_writer: config='{}' path='{}' out='{}' mode={}",
                    config_folder.string(), path_file.string(), output_dir.string(), mode);

    auto blocks = parse_path_file(path_file);
    agent_log("D", "test_writer.cc:parse", "parsed path blocks",
              std::format(R"({{"block_count":{}}})", blocks.size()));

    auto [interposer_box, basedie_box] = parse::read_config(config_folder, mode, false);
    auto* interposer = interposer_box.get();
    auto* basedie = basedie_box.get();
    algo::build_nets(basedie, interposer);

    auto nets = basedie->nets(mode);
    agent_log("D", "test_writer.cc:nets", "built nets",
              std::format(R"({{"net_count":{}}})", nets.size()));

    if (blocks.size() != nets.size()) {
        debug::warning_fmt("path block count {} != net count {}", blocks.size(), nets.size());
    }

    std::Vector<bool> block_used(blocks.size(), false);
    std::Vector<std::Pair<circuit::Net*, circuit::HistoryPathPackage>> pending {};

    for (auto& net_rc : nets) {
        auto* net = net_rc.get();
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
        agent_log("E", "test_writer.cc:match", "matched net to path block",
                  std::format(R"({{"net":"{}","block_index":{}}})", net->name(), matched_index));
        pending.emplace_back(net, build_history_package(interposer, *matched));
        agent_log("D", "test_writer.cc:assign", "built history path package",
                  std::format(R"({{"net":"{}","tracks":{}}})", net->name(),
                              matched->tracks.size()));
    }

    for (auto& [net, history] : pending) {
        circuit::PathPackage package{history, interposer};
        package.occupy_all();
        net->set_pathpackage(package);
    }

    std::filesystem::create_directories(output_dir);
    parse::output_from_routing_results(interposer, output_dir, basedie, mode, false);
    agent_log("D", "test_writer.cc:write", "wrote controlbits",
              std::format(R"({{"output_dir":"{}"}})", output_dir.string()));
}
