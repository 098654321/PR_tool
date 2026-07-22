#include <global/debug/debug.hh>
#include <global/std/collection.hh>
#include <global/std/string.hh>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>
#include <circuit/net/nets.hh>
#include <algo/router/common/maze/path_length.hh>
#include <parse/reader/controlbits/controlbits.hh>
#include <string>
#include <tuple>
#include <unordered_map>
#include <exception>
#include <type_traits>
#include <queue>
#include <optional>
#include <vector>
#include <set>
#include <algorithm>
#include <format>


namespace PR_tool {

auto print_help() -> void;
auto parse_arguments(int, char**) -> std::tuple<std::string, int>;
auto load_hardware(
    hardware::Interposer*, 
    const parse::Controlbits&
) -> std::tuple<std::set<hardware::Bump*>, std::set<hardware::Bump*>, std::set<hardware::Track*>, std::set<hardware::Track*>>;
auto check_bump_and_track(
    hardware::Interposer*, 
    const parse::Controlbits&, 
    const std::set<hardware::Bump*>&, 
    const std::set<hardware::Bump*>&, 
    const std::set<hardware::Track*>&, 
    const std::set<hardware::Track*>&
) -> std::tuple<std::unordered_map<hardware::Bump*, hardware::TOBConnector>, std::unordered_map<hardware::Track*, hardware::TOBConnector>>;

using prev_info_t = std::unordered_map<hardware::Track*, std::optional<std::tuple<hardware::Track*, hardware::COBConnector>>>;

struct ParseState {
    std::unordered_map<hardware::Track*, hardware::Bump*> begin_track_to_bump;
    std::unordered_map<hardware::Track*, hardware::Bump*> end_track_to_bump;
    std::set<hardware::Bump*> pending_begin_bumps;
    std::set<hardware::Bump*> pending_end_bumps;
    std::set<hardware::Track*> pending_begin_tracks;
    std::set<hardware::Track*> pending_end_tracks;
    std::size_t net_index {0};
};

auto is_external_track(const hardware::Track*) -> bool;
auto build_path_forward(
    const prev_info_t&, 
    hardware::Track*, 
    hardware::Track*
) -> std::vector<std::tuple<hardware::Track*, std::optional<hardware::COBConnector>>>;
auto build_path_by_backtrace(
    const prev_info_t&, 
    hardware::Track*, 
    hardware::Track*
) -> std::vector<std::tuple<hardware::Track*, std::optional<hardware::COBConnector>>>;
auto reverse_adj_tracks(
    hardware::Interposer*, 
    hardware::Track*, 
    const parse::Controlbits&
) -> std::vector<std::tuple<hardware::Track*, hardware::COBConnector>>;
auto calc_path_length(const circuit::PathPackage&) -> std::size_t;
auto show_nets(circuit::BaseDie*, int) -> void;
auto show_controlbits(const parse::Controlbits&) -> void;
auto show_bumps_and_tracks_collected(
    const std::set<hardware::Bump*>&, 
    const std::set<hardware::Bump*>&, 
    const std::set<hardware::Track*>&, 
    const std::set<hardware::Track*>&
) -> void;
auto show_tobconnectors(const std::unordered_map<hardware::Bump*, hardware::TOBConnector>&) -> void;
auto show_tobconnectors(const std::unordered_map<hardware::Track*, hardware::TOBConnector>&) -> void;
auto build_parse_state(
    const std::set<hardware::Bump*>&, 
    const std::set<hardware::Bump*>&, 
    const std::set<hardware::Track*>&, 
    const std::set<hardware::Track*>&
) -> ParseState;
auto find_path(
    hardware::Interposer*,
    circuit::BaseDie*,
    const parse::Controlbits&,
    int,
    const std::unordered_map<hardware::Bump*, hardware::TOBConnector>&,
    const std::unordered_map<hardware::Track*, hardware::TOBConnector>&,
    ParseState&
) -> void;
auto check_track_to_bump_net(
    hardware::Interposer*,
    circuit::BaseDie*,
    const parse::Controlbits&,
    int,
    const std::unordered_map<hardware::Track*, hardware::TOBConnector>&,
    ParseState&
) -> void;


/////////////////////////////////////////////////////////////

int main_parse(int argc, char** argv) {
try{
    if (argc < 5) {
        print_help();
        return -1;
    }

    // step1: parse arguments
    debug::info("Parse arguments");
    auto [folder, mode] = parse_arguments(argc, argv);

    // step2: initial log
    debug::info("Initial log");
    debug::initial_log("./parse_controlbits.log");

    // step3: init hardware
    debug::info("Init hardware");
    auto interposer = std::make_unique<hardware::Interposer>();
    auto basedie = std::make_unique<circuit::BaseDie>();

    // step4: load controlbits
    // TODO(split-output): still assumes controlbits_<mode>.txt; formal output is now
    // regnamecontrolbit_4part/; readback / compare not updated yet.
    debug::info("Load controlbits");    
    auto controlbits = parse::load_controlbits(folder, mode);
    if (!controlbits.has_value()) {
        throw std::logic_error("Controlbits not found");
    }
    show_controlbits(controlbits.value());

    // step5: load hardware from tob controlbits
    // notice: only part of the tracks are loaded here, because some other tracks are IO ports of the interposer
    //         and will be loaded during the maze algorithm
    debug::info("Load hardware from controlbits");    
    auto [begin_bumps, end_bumps, begin_tracks, end_tracks] = load_hardware(interposer.get(), controlbits.value());
    show_bumps_and_tracks_collected(begin_bumps, end_bumps, begin_tracks, end_tracks);

    // step6: check if there is error in the loaded hardware
    debug::info("Check loaded hardware");    
    auto [begin_tobconnectors, end_tobconnectors] = check_bump_and_track(interposer.get(), controlbits.value(), begin_bumps, end_bumps, begin_tracks, end_tracks);
    auto state = build_parse_state(begin_bumps, end_bumps, begin_tracks, end_tracks);
    show_tobconnectors(begin_tobconnectors);
    show_tobconnectors(end_tobconnectors);

    // step7: 根据 cob 的controlbits信息开始寻找路径
    debug::info("Find path");    
    find_path(interposer.get(), basedie.get(), controlbits.value(), mode, begin_tobconnectors, end_tobconnectors, state);

    // step8: 检查是否还有从IO到bump的net
    debug::info("Check track to bump net");    
    check_track_to_bump_net(interposer.get(), basedie.get(), controlbits.value(), mode, end_tobconnectors, state);

    // step9: show net info
    show_nets(basedie.get(), mode);
}
catch(const std::exception& e) {
    debug::error(std::format("Unexpected exception: {}", e.what()));
    return -1;
}
return 0;
}

/////////////////////////////////////////////////////////////


void print_help() {
    debug::info("Usage: parse_controlbits -folder <controlbits_file_folder> -mode <mode>\n");
    return;
}

std::tuple<std::string, int> parse_arguments(int argc, char** argv) {
    auto arguments = std::vector<std::string>{};
    for (int i = 1; i < argc; i++) {
        arguments.push_back(argv[i]);
    }

    auto argument_index = [&arguments](const std::string& argument) -> int {
        for (int i = 0; i < arguments.size(); i++) {
            if (arguments[i] == argument) {
                return i;
            }
        }
        return -1;
    };

    auto show_help = [](std::string info) -> void {
        debug::info(info);
        print_help();
        std::exit(-1);
    };

    auto file_index = argument_index("-folder");
    if (file_index == -1) {
        show_help("Error: -folder argument not found\n");
    }
    if (file_index + 1 >= arguments.size()) {
        show_help("Error: -folder argument without file path\n");
    }
    auto file_path = arguments[file_index + 1];

    auto mode_index = argument_index("-mode");
    if (mode_index == -1) {
        show_help("Error: -mode argument not found\n");
    }
    if (mode_index + 1 >= arguments.size()) {
        show_help("Error: -mode argument without mode\n");
    }
    auto mode = arguments[mode_index + 1];

    if (argument_index("-v") != -1) {
            debug::set_debug_level(debug::DebugLevel::Debug);
        }

    return {file_path, std::stoi(mode)};
}
    
auto load_hardware(hardware::Interposer* interposer, const parse::Controlbits& controlbits) -> std::tuple<std::set<hardware::Bump*>, std::set<hardware::Bump*>, std::set<hardware::Track*>, std::set<hardware::Track*>>
{
    auto begin_bumps = std::set<hardware::Bump*>{};
    auto end_bumps = std::set<hardware::Bump*>{};
    auto begin_tracks = std::set<hardware::Track*>{};
    auto end_tracks = std::set<hardware::Track*>{};
    std::size_t N = 128;

    for (auto& [tobcoord, bits]: controlbits.tob_bumpsig_direction) {
        for (std::size_t i = 0; i < N; i++) {
            auto bump = interposer->get_bump(tobcoord, i);
            if (!bump.has_value()) {
                throw std::logic_error("load_hardware(): Bump not found");
            }
            if (bits[i] == hardware::TOBBumpDirection::BumpToTOB) {
                begin_bumps.insert(*bump);
            } 
            else if(bits[i] == hardware::TOBBumpDirection::TOBToBump) {
                end_bumps.insert(*bump);
            }
        }
    }
    for (auto& [tobcoord, bits]: controlbits.tob_tracksig_direction) {
        for (std::size_t i = 0; i < N; i++) {
            auto coord_in_interposer = hardware::Interposer::TOB_COORD_MAP.at(tobcoord);
            auto track = interposer->get_track(coord_in_interposer.row, coord_in_interposer.col, hardware::TrackDirection::Vertical, i);
            if (!track.has_value()) {
                throw std::logic_error("load_hardware(): Track not found");
            }
            if (bits[i] == hardware::TOBTrackDirection::TrackToTOB) {
                end_tracks.insert(*track);
            } 
            else if(bits[i] == hardware::TOBTrackDirection::TOBToTrack) {
                begin_tracks.insert(*track);
            }
        }
    }
    return {begin_bumps, end_bumps, begin_tracks, end_tracks};
}

auto check_bump_and_track(hardware::Interposer* interposer, const parse::Controlbits& controlbits, const std::set<hardware::Bump*>& begin_bumps, const std::set<hardware::Bump*>& end_bumps, const std::set<hardware::Track*>& begin_tracks, const std::set<hardware::Track*>& end_tracks) -> std::tuple<std::unordered_map<hardware::Bump*, hardware::TOBConnector>, std::unordered_map<hardware::Track*, hardware::TOBConnector>>
{
    auto begin_tracks_copy = begin_tracks;
    auto end_tracks_copy = end_tracks;
    auto begin_tobconnectors = std::unordered_map<hardware::Bump*, hardware::TOBConnector>{};
    auto end_tobconnectors = std::unordered_map<hardware::Track*, hardware::TOBConnector>{};

    auto get_tobconnector = [&controlbits, interposer](hardware::Bump* bump, hardware::TOBBumpDirection dir, auto& to_tobconnector) -> void {
        auto tobcoord = bump->tob()->coord();
        auto coord_in_interposer = hardware::Interposer::TOB_COORD_MAP.at(tobcoord);
        int bump_index = bump->index();

        int hctrl = controlbits.bumptohctrl.at(tobcoord).at(bump_index) + (bump_index/8)*8;
        int trans_hctrl = (hctrl/64)*64 + (hctrl%8)*8 + (hctrl - (hctrl/64)*64)/8;
        int vctrl = controlbits.hctrltovctrl.at(tobcoord).at(trans_hctrl) + (trans_hctrl/64)*64 + ((trans_hctrl-(trans_hctrl/64)*64)/8)*8;
        int track_index = controlbits.vctrltotrack.at(tobcoord)[vctrl%64] == 0? vctrl : (vctrl+64)%128;
        auto connector = bump->tob()->bump_track_connectors_chain(bump_index, track_index, dir); 
        auto track = interposer->get_track(coord_in_interposer.row, coord_in_interposer.col, hardware::TrackDirection::Vertical, track_index);
        if (!track.has_value()) {
            throw std::logic_error("check_hardware(): Track not found");
        }
        auto tob_sig_direction = dir == hardware::TOBBumpDirection::BumpToTOB? hardware::TOBSignalDirection::BumpToTrack : hardware::TOBSignalDirection::TrackToBump;

        using connector_map_t = std::remove_reference_t<decltype(to_tobconnector)>;
        using key_t = typename connector_map_t::key_type;
        if constexpr (std::is_same_v<key_t, hardware::Bump*>) {
            to_tobconnector.emplace(bump, connector);
        } else if constexpr (std::is_same_v<key_t, hardware::Track*>) {
            to_tobconnector.emplace(*track, connector);
        }
        bump->set_connected_track(*track, tob_sig_direction);
        
        debug::debug_fmt("load_tobconnector(): calculated index = {}->{}->{}->{}", bump_index, hctrl, vctrl, track_index); 
    };

    auto check_track_index = [get_tobconnector](const std::set<hardware::Bump*>& bumps, std::set<hardware::Track*>& tracks_copy, hardware::TOBBumpDirection dir, auto& to_tobconnector) {
        for (auto bump: bumps) {
            get_tobconnector(bump, dir, to_tobconnector);
            auto track = bump->connected_track();
            if (tracks_copy.contains(track)) {
                tracks_copy.erase(track);
            }
            else {
                throw std::logic_error(std::format("check_bump_and_track(): bumps {} contains track {} that is not in tracks", bump->coord(), track->coord()));
            }
        }
        if (!tracks_copy.empty()) {
            throw std::logic_error("check_bump_and_track(): tracks contains track that is not connected to the bumps");
        }
    };
    
    check_track_index(begin_bumps, begin_tracks_copy, hardware::TOBBumpDirection::BumpToTOB, begin_tobconnectors);
    check_track_index(end_bumps, end_tracks_copy, hardware::TOBBumpDirection::TOBToBump, end_tobconnectors);

    return {begin_tobconnectors, end_tobconnectors};
}

auto build_parse_state(const std::set<hardware::Bump*>& begin_bumps, const std::set<hardware::Bump*>& end_bumps, const std::set<hardware::Track*>& begin_tracks, const std::set<hardware::Track*>& end_tracks) -> ParseState {
    auto state = ParseState{};
    state.pending_begin_bumps = begin_bumps;
    state.pending_end_bumps = end_bumps;
    state.pending_begin_tracks = begin_tracks;
    state.pending_end_tracks = end_tracks;

    for (auto bump: begin_bumps) {
        state.begin_track_to_bump.emplace(bump->connected_track(), bump);
    }
    for (auto bump: end_bumps) {
        state.end_track_to_bump.emplace(bump->connected_track(), bump);
    }
    return state;
}

auto find_path(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const parse::Controlbits& controlbits,
    int mode,
    const std::unordered_map<hardware::Bump*, hardware::TOBConnector>& begin_tobconnectors,
    const std::unordered_map<hardware::Track*, hardware::TOBConnector>& end_tobconnectors,
    ParseState& state
) -> void {
    auto all_begin_tracks = std::vector<hardware::Track*>{state.pending_begin_tracks.begin(), state.pending_begin_tracks.end()};
    for (auto begin_track: all_begin_tracks) {
        if (!state.pending_begin_tracks.contains(begin_track)) {
            continue;
        }
        if (!state.begin_track_to_bump.contains(begin_track)) {
            throw std::logic_error("step7: begin track cannot map to begin bump");
        }

        auto queue = std::queue<hardware::Track*>{};
        auto prev_info = prev_info_t{};
        queue.push(begin_track);
        prev_info.emplace(begin_track, std::nullopt);

        auto matched_end_track = static_cast<hardware::Track*>(nullptr);
        auto matched_io_track = static_cast<hardware::Track*>(nullptr);
        while (!queue.empty()) {
            auto cur = queue.front();
            queue.pop();

            if (cur != begin_track && state.pending_end_tracks.contains(cur)) {
                matched_end_track = cur;
                break;
            }
            if (cur != begin_track && is_external_track(cur)) {
                matched_io_track = cur;
                break;
            }

            for (auto& [next_track, connector]: parse::adj_tracks(interposer, cur, controlbits)) {
                if (prev_info.contains(next_track)) {
                    continue;
                }
                prev_info.emplace(next_track, std::make_optional(std::tuple<hardware::Track*, hardware::COBConnector>{cur, connector}));
                queue.push(next_track);
            }
        }

        if (matched_end_track == nullptr && matched_io_track == nullptr) {
            throw std::logic_error(std::format("step7: cannot find reachable end track or IO from begin track {}", begin_track->coord()));
        }

        auto begin_bump = state.begin_track_to_bump.at(begin_track);
        auto& begin_connector = begin_tobconnectors.at(begin_bump);
        auto mode_set = std::HashSet<int>{mode};

        if (matched_end_track != nullptr) {
            if (!state.end_track_to_bump.contains(matched_end_track)) {
                throw std::logic_error("step7: matched end track cannot map to end bump");
            }
            auto end_bump = state.end_track_to_bump.at(matched_end_track);
            auto& end_connector = end_tobconnectors.at(matched_end_track);
            auto regular_path = build_path_by_backtrace(prev_info, begin_track, matched_end_track);

            auto package = circuit::PathPackage{};
            package._regular_path.assign(regular_path.begin(), regular_path.end());
            package._tob_to_track.emplace_back(begin_bump, begin_connector, begin_track);
            package._track_to_tob.emplace_back(end_bump, end_connector, matched_end_track);
            package._length = calc_path_length(package);

            auto net_name = std::String{std::format("Controlbits_BumpToBumpNet_{}", state.net_index)};
            auto net = std::make_shared<circuit::BumpToBumpNet>(begin_bump, end_bump, mode_set, net_name);
            net->set_pathpackage(package);
            basedie->add_net(net, mode);

            state.pending_begin_bumps.erase(begin_bump);
            state.pending_begin_tracks.erase(begin_track);
            state.pending_end_bumps.erase(end_bump);
            state.pending_end_tracks.erase(matched_end_track);
        } else {
            auto regular_path = build_path_by_backtrace(prev_info, begin_track, matched_io_track);

            auto package = circuit::PathPackage{};
            package._regular_path.assign(regular_path.begin(), regular_path.end());
            package._tob_to_track.emplace_back(begin_bump, begin_connector, begin_track);
            package._length = calc_path_length(package);

            auto net_name = std::String{std::format("Controlbits_BumpToTrackNet_{}", state.net_index)};
            auto net = std::make_shared<circuit::BumpToTrackNet>(begin_bump, matched_io_track, mode_set, net_name);
            net->set_pathpackage(package);
            basedie->add_net(net, mode);

            state.pending_begin_bumps.erase(begin_bump);
            state.pending_begin_tracks.erase(begin_track);
        }
        state.net_index += 1;
    }
}

auto check_track_to_bump_net(
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    const parse::Controlbits& controlbits,
    int mode,
    const std::unordered_map<hardware::Track*, hardware::TOBConnector>& end_tobconnectors,
    ParseState& state
) -> void {
    if (!state.pending_begin_bumps.empty()) {
        throw std::logic_error(std::format("step8: begin bumps not empty after step7, remain {}", state.pending_begin_bumps.size()));
    }

    if (!state.pending_end_bumps.empty()) {
        auto all_end_tracks = std::vector<hardware::Track*>{state.pending_end_tracks.begin(), state.pending_end_tracks.end()};
        for (auto end_track: all_end_tracks) {
            if (!state.pending_end_tracks.contains(end_track)) {
                continue;
            }
            if (!state.end_track_to_bump.contains(end_track)) {
                throw std::logic_error("step8: end track cannot map to end bump");
            }

            auto queue = std::queue<hardware::Track*>{};
            auto prev_info = prev_info_t{};
            queue.push(end_track);
            prev_info.emplace(end_track, std::nullopt);

            auto source_io_track = static_cast<hardware::Track*>(nullptr);
            while (!queue.empty()) {
                auto cur = queue.front();
                queue.pop();
                if (is_external_track(cur)) {
                    source_io_track = cur;
                    break;
                }
                for (auto& [prev_track, connector]: reverse_adj_tracks(interposer, cur, controlbits)) {
                    if (prev_info.contains(prev_track)) {
                        continue;
                    }
                    prev_info.emplace(prev_track, std::make_optional(std::tuple<hardware::Track*, hardware::COBConnector>{cur, connector}));
                    queue.push(prev_track);
                }
            }

            if (source_io_track == nullptr) {
                throw std::logic_error(std::format("step8: cannot find IO source for end track {}", end_track->coord()));
            }

            auto end_bump = state.end_track_to_bump.at(end_track);
            auto& end_connector = end_tobconnectors.at(end_track);
            auto regular_path = build_path_forward(prev_info, source_io_track, end_track);

            auto package = circuit::PathPackage{};
            package._regular_path.assign(regular_path.begin(), regular_path.end());
            package._track_to_tob.emplace_back(end_bump, end_connector, end_track);
            package._length = calc_path_length(package);

            auto mode_set = std::HashSet<int>{mode};
            auto net_name = std::String{std::format("Controlbits_TrackToBumpNet_{}", state.net_index)};
            auto net = std::make_shared<circuit::TrackToBumpNet>(source_io_track, end_bump, mode_set, net_name);
            net->set_pathpackage(package);
            basedie->add_net(net, mode);

            state.pending_end_bumps.erase(end_bump);
            state.pending_end_tracks.erase(end_track);
            state.net_index += 1;
        }
    }

    if (!state.pending_end_bumps.empty()) {
        throw std::logic_error(std::format("step8: end bumps still not empty after IO recovery, remain {}", state.pending_end_bumps.size()));
    }
}

auto is_external_track(const hardware::Track* track) -> bool {
    return hardware::Interposer::is_external_port_coord(track->coord()) || hardware::Interposer::is_padctrl_port(track->coord());
}

auto build_path_forward(const prev_info_t& prev_info, hardware::Track* source, hardware::Track* target) -> std::vector<std::tuple<hardware::Track*, std::optional<hardware::COBConnector>>> {
    if (!prev_info.contains(source) || !prev_info.contains(target)) {
        throw std::logic_error("build_path_forward: source or target not found in prev_info");
    }

    auto path = std::vector<std::tuple<hardware::Track*, std::optional<hardware::COBConnector>>>{};
    auto cur = source;
    path.emplace_back(cur, std::nullopt);
    while (true) {
        auto it = prev_info.find(cur);
        if (it == prev_info.end()) {
            throw std::logic_error("build_path_forward: prev_info broken");
        }
        if (!it->second.has_value()) {
            break;
        }
        auto [next, connector] = it->second.value();
        path.emplace_back(next, connector);
        cur = next;
    }
    if (cur != target) {
        throw std::logic_error("build_path_forward: source cannot reach target");
    }
    return path;
}

auto build_path_by_backtrace(const prev_info_t& prev_info, hardware::Track* source, hardware::Track* target) -> std::vector<std::tuple<hardware::Track*, std::optional<hardware::COBConnector>>> {
    if (!prev_info.contains(source) || !prev_info.contains(target)) {
        throw std::logic_error("build_path_by_backtrace: source or target not found in prev_info");
    }

    auto reversed_path = std::vector<std::tuple<hardware::Track*, std::optional<hardware::COBConnector>>>{};
    auto cur = target;
    while (true) {
        auto it = prev_info.find(cur);
        if (it == prev_info.end()) {
            throw std::logic_error("build_path_by_backtrace: prev_info broken");
        }
        if (!it->second.has_value()) {
            break;
        }
        auto [prev, connector] = it->second.value();
        reversed_path.emplace_back(cur, connector);
        cur = prev;
    }
    if (cur != source) {
        throw std::logic_error("build_path_by_backtrace: source cannot reach target");
    }
    reversed_path.emplace_back(source, std::nullopt);
    std::reverse(reversed_path.begin(), reversed_path.end());
    return reversed_path;
}

auto reverse_adj_tracks(hardware::Interposer* interposer, hardware::Track* track, const parse::Controlbits& controlbits) -> std::vector<std::tuple<hardware::Track*, hardware::COBConnector>> {
    auto result = std::vector<std::tuple<hardware::Track*, hardware::COBConnector>>{};
    for (auto& [candidate, _]: interposer->adjacent_tracks(track)) {
        for (auto& [next_track, connector]: parse::adj_tracks(interposer, candidate, controlbits)) {
            if (next_track == track) {
                result.emplace_back(candidate, connector);
                break;
            }
        }
    }
    return result;
}

auto calc_path_length(const circuit::PathPackage& package) -> std::size_t {
    return algo::path_length(package._regular_path) + package._tob_to_track.size() + package._track_to_tob.size();
}

auto show_nets(circuit::BaseDie* basedie, int mode) -> void {
    debug::info_fmt("**************************** Show nets ****************************");
    for (const auto& net: basedie->nets(mode)) {
        debug::info(net->to_string());
        auto l = net->length();
        if (l > 0) {
            debug::info_fmt("Routing length of this net: {}", l);
            debug::info_fmt("Routing priority of this net: {}", net->priority().value());
            net->show_path();
        } else {
            debug::info_fmt("Routing failed for this net: {}", net->name());
        }
    }
}

auto show_controlbits(const parse::Controlbits& controlbits) -> void {
    debug::debug_fmt("controlbits.tob_bumpsig_direction_size: {}", controlbits.tob_bumpsig_direction.size());
    debug::debug_fmt("controlbits.tob_tracksig_direction_size: {}", controlbits.tob_tracksig_direction.size());
    debug::debug_fmt("controlbits.cobsig_direction: {}", controlbits.cobsig_direction.size());
    debug::debug_fmt("controlbits.bumptohctrl: {}", controlbits.bumptohctrl.size());
    debug::debug_fmt("controlbits.hctrltovctrl: {}", controlbits.hctrltovctrl.size());
    debug::debug_fmt("controlbits.vctrltotrack: {}", controlbits.vctrltotrack.size());
    debug::debug_fmt("controlbits.cobsw: {}", controlbits.cobsw.size());
}

auto show_bumps_and_tracks_collected(
    const std::set<hardware::Bump*>& begin_bumps, 
    const std::set<hardware::Bump*>& end_bumps, 
    const std::set<hardware::Track*>& begin_tracks, 
    const std::set<hardware::Track*>& end_tracks
) -> void {
    // print all the bumps and tracks
    debug::debug_fmt("begin_bumps:");
    for (auto& bump: begin_bumps) {
        debug::debug_fmt("Bump: {}", bump->coord(), bump->index());
    }
    debug::debug_fmt("end_bumps:");
    for (auto& bump: end_bumps) {
        debug::debug_fmt("Bump: {}", bump->coord(), bump->index());
    }
    debug::debug_fmt("begin_tracks:");
    for (auto& track: begin_tracks) {
        debug::debug_fmt("Track: {}", track->coord());
    }
    debug::debug_fmt("end_tracks:");
    for (auto& track: end_tracks) {
        debug::debug_fmt("Track: {}", track->coord());
    }
}

auto show_tobconnectors(const std::unordered_map<hardware::Bump*, hardware::TOBConnector>& tobconnectors) -> void {
    debug::debug_fmt("tobconnectors.size: {}", tobconnectors.size());
}

auto show_tobconnectors(const std::unordered_map<hardware::Track*, hardware::TOBConnector>& tobconnectors) -> void {
    debug::debug_fmt("tobconnectors.size: {}", tobconnectors.size());
}





}


int main(int argc, char** argv) {
    try {
        PR_tool::main_parse(argc, argv);
    }
    catch (const std::exception& err) {
        PR_tool::debug::exception("Unexpected exception");
    }
}

