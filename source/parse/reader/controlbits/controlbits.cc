#include "./controlbits.hh"
#include <type_traits>
#include <debug/debug.hh>
#include <algorithm>
#include <circuit/net/nets.hh>
#include <ranges>
#include <algo/router/common/maze/path_length.hh>


namespace PR_tool::parse {

    template <std::usize N>
    void setBits(std::bitset<N>& bits, int startIndex, int length, const std::string& bitString) {
        if (length > N - startIndex) {
            throw std::invalid_argument("Bitstring length exceeds bitset size");
        }

        for (int i = 0; i < length; ++i) {
            if (bitString[i] == '1') {
                bits.set(startIndex + i);
            } else {
                bits.reset(startIndex + i);
            }
        }
    }

    std::vector<std::string> split(const std::string &s, char delimiter) {
        std::vector<std::string> tokens;
        std::stringstream ss(s);
        std::string token;
        while (std::getline(ss, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }
    
    hardware::COBSWDirection parseCOBSWDirection(const std::string &dir) {
        if (dir == "ru") return hardware::COBSWDirection::RU;
        if (dir == "lu") return hardware::COBSWDirection::LU;
        if (dir == "rd") return hardware::COBSWDirection::RD;
        if (dir == "ld") return hardware::COBSWDirection::LD;
        if (dir == "h") return hardware::COBSWDirection::H;
        if (dir == "v") return hardware::COBSWDirection::V;
        throw std::invalid_argument("Invalid COBSWDirection: " + dir);
    }
    
    hardware::COBDirection parseCOBDirection(const std::string &dir) {
        if (dir == "left") return hardware::COBDirection::Left;
        if (dir == "right") return hardware::COBDirection::Right;
        if (dir == "down") return hardware::COBDirection::Down;
        if (dir == "up") return hardware::COBDirection::Up;
        throw std::invalid_argument("Invalid COBDirection: " + dir);
    }

    auto parse_xinzhai(
        const std::Vector<std::String>& tokens,
        Controlbits& controlbits,
        const std::String& reversed_bits,
        std::HashMap<std::String, std::Array<bool, 4>>& parsed_chunks
    ) -> void;
    auto finalize_xinzhai(const std::HashMap<std::String, std::Array<bool, 4>>&) -> void;

    // TODO(split-output): still assumes controlbits_<mode>.txt; formal output is now
    // regnamecontrolbit_4part/; readback / compare not updated yet.
    auto load_controlbits(const std::FilePath& path, int mode) -> std::Option<Controlbits> {
    try {
        auto controlbits_path = path / ("controlbits_" + std::to_string(mode) + ".txt");
        std::ifstream file(controlbits_path);
        if (!file.is_open()) {
            debug::info_fmt("Controlbits file {} not found", controlbits_path.string());
            return std::nullopt;
        }
        else {
            debug::info_fmt("Load controlbits from {}", controlbits_path.string());
        }
    
        Controlbits controlbits;
        std::unordered_map<hardware::TOBCoord, std::bitset<128>> tob2bump_sig, bump2tob_sig, tob2track_sig, track2tob_sig;
        std::HashMap<std::String, std::Array<bool, 4>> xinzhai_parsed_chunks;
        std::string line;
    
        while (std::getline(file, line)) {
            if (line.empty()) 
                continue;
    
            auto [reversed_bits, tokens] = parse_line(line);

            if (tokens[0] == "cob") {
                parse_cob(tokens, controlbits, reversed_bits);
            } else if (tokens[0] == "tob") {
                auto sig_arrays = std::Array<std::unordered_map<hardware::TOBCoord, std::bitset<128>>*, 4> {
                    &tob2bump_sig, &bump2tob_sig, &tob2track_sig, &track2tob_sig
                };
                auto has_value = parse_tob(tokens, controlbits, reversed_bits, sig_arrays);
                if (!has_value) 
                    continue;
            } else if (tokens[0] == "xinzhai") {
                parse_xinzhai(tokens, controlbits, reversed_bits, xinzhai_parsed_chunks);
            }
        }
    
        finalize_xinzhai(xinzhai_parsed_chunks);
        process_bump_sig(controlbits, tob2bump_sig, bump2tob_sig);
        process_track_sig(controlbits, tob2track_sig, track2tob_sig);
    
        return controlbits;
    }
    catch (const std::exception& e) {
        debug::exception_fmt("loading controlbits(): {}", e.what());
    }
    }

    auto bits_to_paths(hardware::Interposer* interposer, circuit::BaseDie* basedie, const Controlbits& controlbits, int mode) -> void {
    try {
        for (auto& net: basedie->nets(mode)) {
            debug::info_fmt("bits_to_paths: process net {}", net->name());
            auto sync_net = dynamic_cast<circuit::SyncNet*>(net.get());
            if (sync_net != nullptr) {
                load_tobconnector_sync(interposer, controlbits, sync_net);
                load_path_with_maze_sync(sync_net, interposer, controlbits);
                load_length_sync(sync_net);
            }
            else {
                auto bumps = net->nodes_direction();
                load_tobconnector(interposer, controlbits, bumps, net.get());
                load_path_with_maze(net.get(), interposer, controlbits);
                load_length(net.get());
            }
        }
        show_path_from_bits(basedie->nets(mode));
    }
    catch (const std::exception& e) {
        debug::exception_fmt("bits_to_paths(): {}", e.what());
    }
    }

    auto parse_line(std::String line) -> std::Tuple<std::String, std::Vector<std::String>> {
        std::stringstream ss(line);
        std::string hex_value, descriptor;
        ss >> hex_value >> descriptor;
        std::bitset<32> bits_value(std::stoul(hex_value, nullptr, 16));
        std::string reversed_bits = bits_value.to_string();
        std::reverse(reversed_bits.begin(), reversed_bits.end());
        auto tokens = split(descriptor, '_');

        return std::Tuple<std::String, std::Vector<std::String>> {
            reversed_bits, tokens
        };
    }

    auto parse_cob(const std::Vector<std::String>& tokens, Controlbits& controlbits, const std::String& reversed_bits) -> void {
    try {
        hardware::COBCoord coord(std::stoll(tokens[1]), std::stoll(tokens[2]));
        if (tokens[3] == "sw") {
            auto iter1 = controlbits.cobsw.emplace(coord, std::HashMap<hardware::COBSWDirection, std::bitset<128>>{});
            auto dir = parseCOBSWDirection(tokens[4]);
            auto iter2 = iter1.first->second.emplace(dir, std::bitset<128>{});

            int index = std::stoi(tokens[5]) * 32;
            setBits<128>(iter2.first->second, index, 32, reversed_bits);
        } else {
            auto iter1 = controlbits.cobsig_direction.emplace(coord, std::HashMap<hardware::COBDirection, std::bitset<128>>{});
            auto dir = parseCOBDirection(tokens[3]);
            auto iter2 = iter1.first->second.emplace(dir, std::bitset<128>{});

            int index = std::stoi(tokens[5]) * 32;
            setBits<128>(iter2.first->second, index, 32, reversed_bits);
        }
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::format("parse_cob(): {}", e.what()));
    }
    }

    auto parse_tob(
        const std::Vector<std::String>& tokens, Controlbits& controlbits, const std::String& reversed_bits,
        std::Array<std::unordered_map<hardware::TOBCoord, std::bitset<128>>*, 4>& sig_arrays
    ) -> bool {
    try {
        auto tob2bump_sig = sig_arrays[0];
        auto bump2tob_sig = sig_arrays[1];
        auto tob2track_sig = sig_arrays[2];
        auto track2tob_sig = sig_arrays[3];

        hardware::TOBCoord coord(std::stoll(tokens[1]), std::stoll(tokens[2]));
        if (tokens[3] == "dly" || tokens[3] == "drv") {
            return false;
        } else if (tokens[3] == "tob2bump" || tokens[3] == "bump2tob") {
            int index = std::stoi(tokens[4].substr(4)) * 64 + std::stoi(tokens[6]) * 32;
            auto &map = (tokens[3] == "tob2bump") ? tob2bump_sig : bump2tob_sig;

            auto iter = map->emplace(coord, std::bitset<128>{});
            setBits<128>(iter.first->second, index, 32, reversed_bits);
        } else if (tokens[3] == "tob2track" || tokens[3] == "track2tob") {
            int index = std::stoi(tokens[4]) * 32;
            auto &map = (tokens[3] == "tob2track") ? tob2track_sig : track2tob_sig;

            auto iter = map->emplace(coord, std::bitset<128>{});
            setBits<128>(iter.first->second, index, 32, reversed_bits);
        } else if (tokens[3] == "hctrl" || tokens[3] == "vctrl") {
            int index = std::stoi(tokens[4].substr(4)) * 64 + std::stoi(tokens[5]) * 8;
            auto &map = (tokens[3] == "hctrl") ? controlbits.bumptohctrl : controlbits.hctrltovctrl;
            
            auto iter = map.emplace(coord, std::Array<int, 128>{});
            for (int i = 0; i < 24; i += 3) {
                int value = (reversed_bits[i] - '0') + (reversed_bits[i + 1] - '0') * 2 + (reversed_bits[i + 2] - '0') * 4; 
                iter.first->second[index + i / 3] = value;
            }
        } else if (tokens[3] == "bank") {
            int index = std::stoi(tokens[5]) * 32;
            
            auto iter = controlbits.vctrltotrack.emplace(coord, std::bitset<64>{});
            setBits<64>(iter.first->second, index, 32, reversed_bits);
        }

        return true;
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::format("parse_tob(): {}", e.what()));
    }
    }

    auto parse_xinzhai(
        const std::Vector<std::String>& tokens,
        Controlbits& controlbits,
        const std::String& reversed_bits,
        std::HashMap<std::String, std::Array<bool, 4>>& parsed_chunks
    ) -> void {
    try {
        std::Bits<128>* target = nullptr;
        std::String reg_name;
        std::usize chunk_index = 0;

        if (tokens.size() == 7 && tokens[1] == "C4" && tokens[2] == "noi" && tokens[4] == "pad" && tokens[5] == "ctrl") {
            if (tokens[3] == "right") {
                target = &controlbits.xinzhai.padctrl_right;
                reg_name = "padctrl_right";
            } else if (tokens[3] == "left") {
                target = &controlbits.xinzhai.padctrl_left;
                reg_name = "padctrl_left";
            } else if (tokens[3] == "up") {
                target = &controlbits.xinzhai.padctrl_up;
                reg_name = "padctrl_up";
            } else if (tokens[3] == "down") {
                target = &controlbits.xinzhai.padctrl_down;
                reg_name = "padctrl_down";
            }

            chunk_index = static_cast<std::usize>(std::stoul(tokens[6]));
        } else if (tokens.size() == 8 && tokens[1] == "C4" && tokens[2] == "noi" && tokens[3] == "SiP" && tokens[5] == "pad" && tokens[6] == "ctrl") {
            if (tokens[4] == "right") {
                target = &controlbits.xinzhai.SiPpadctrl_right;
                reg_name = "SiPpadctrl_right";
            } else if (tokens[4] == "left") {
                target = &controlbits.xinzhai.SiPpadctrl_left;
                reg_name = "SiPpadctrl_left";
            } else if (tokens[4] == "up") {
                target = &controlbits.xinzhai.SiPpadctrl_up;
                reg_name = "SiPpadctrl_up";
            } else if (tokens[4] == "down") {
                target = &controlbits.xinzhai.SiPpadctrl_down;
                reg_name = "SiPpadctrl_down";
            }

            chunk_index = static_cast<std::usize>(std::stoul(tokens[7]));
        } else {
            throw std::invalid_argument("unexpected xinzhai descriptor format");
        }

        if (target == nullptr) {
            throw std::invalid_argument("invalid xinzhai direction token");
        }
        if (chunk_index >= 4) {
            throw std::invalid_argument(std::format("xinzhai chunk index {} out of range [0, 3]", chunk_index));
        }

        setBits<128>(*target, static_cast<int>(chunk_index * 32), 32, reversed_bits);

        auto iter = parsed_chunks.emplace(reg_name, std::Array<bool, 4>{false, false, false, false});
        iter.first->second[chunk_index] = true;
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::format("parse_xinzhai(): {}", e.what()));
    }
    }

    auto finalize_xinzhai(const std::HashMap<std::String, std::Array<bool, 4>>& parsed_chunks) -> void {
    try {
        const auto assert_full = [&](const std::String& reg_name) {
            if (!parsed_chunks.contains(reg_name)) {
                throw std::invalid_argument(std::format("missing xinzhai register {}", reg_name));
            }

            const auto& chunks = parsed_chunks.at(reg_name);
            for (std::usize i = 0; i < 4; ++i) {
                if (!chunks[i]) {
                    throw std::invalid_argument(std::format("xinzhai register {} missing chunk {}", reg_name, i));
                }
            }
        };

        assert_full("padctrl_right");
        assert_full("padctrl_left");
        assert_full("padctrl_up");
        assert_full("padctrl_down");
        assert_full("SiPpadctrl_right");
        assert_full("SiPpadctrl_left");
        assert_full("SiPpadctrl_up");
        assert_full("SiPpadctrl_down");
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::format("finalize_xinzhai(): {}", e.what()));
    }
    }

    auto process_bump_sig(
        Controlbits& controlbits,
        const std::unordered_map<hardware::TOBCoord, std::bitset<128>>& tob2bump_sig,
        const std::unordered_map<hardware::TOBCoord, std::bitset<128>>& bump2tob_sig
    ) -> void {
    try {
        for (const auto& [coord, bitset1] : bump2tob_sig) {
            const auto& bitset2 = tob2bump_sig.at(coord);
            for (int i = 0; i < 128; ++i) {
                if (!bitset1[i] && !bitset2[i])
                    controlbits.tob_bumpsig_direction[coord][i] = hardware::TOBBumpDirection::DisConnected;
                else if (!bitset2[i] && bitset1[i])
                    controlbits.tob_bumpsig_direction[coord][i] = hardware::TOBBumpDirection::BumpToTOB;
                else if (bitset2[i] && !bitset1[i])
                    controlbits.tob_bumpsig_direction[coord][i] = hardware::TOBBumpDirection::TOBToBump;
            }
        }
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::format("process_bump_sig(): {}", e.what()));
    }
    }

    auto process_track_sig(
        Controlbits& controlbits,
        const std::unordered_map<hardware::TOBCoord, std::bitset<128>>& tob2track_sig,
        const std::unordered_map<hardware::TOBCoord, std::bitset<128>>& track2tob_sig
    ) -> void {
    try {
        for (const auto& [coord, bitset1] : track2tob_sig) {
            const auto& bitset2 = tob2track_sig.at(coord);
            for (int i = 0; i < 128; ++i) {
                if (!bitset1[i] && !bitset2[i])
                    controlbits.tob_tracksig_direction[coord][i] = hardware::TOBTrackDirection::DisConnected;
                else if (!bitset2[i] && bitset1[i])
                    controlbits.tob_tracksig_direction[coord][i] = hardware::TOBTrackDirection::TrackToTOB;
                else if (bitset2[i] && !bitset1[i])
                    controlbits.tob_tracksig_direction[coord][i] = hardware::TOBTrackDirection::TOBToTrack;
            }
        }
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::format("process_track_sig(): {}", e.what()));
    }
    }

    auto load_tobconnector(
        hardware::Interposer* interposer, const Controlbits& controlbits,
        std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> bumps,
        circuit::Net* net
    ) -> void {
    try {
        for (auto& [bump, dir]: bumps) {
            auto tobcoord = bump->tob()->coord();
            auto bump_index = bump->index();

            auto tob_bump_dir = controlbits.tob_bumpsig_direction.at(tobcoord);
            if (tob_bump_dir.at(bump_index) == hardware::TOBBumpDirection::DisConnected)
                continue;
            else {
                // get track
                auto hctrl = controlbits.bumptohctrl.at(tobcoord).at(bump_index) + (bump_index/8)*8;
                auto trans_hctrl = (hctrl/64)*64 + (hctrl%8)*8 + (hctrl - (hctrl/64)*64)/8;
                auto vctrl = controlbits.hctrltovctrl.at(tobcoord).at(trans_hctrl) + (hctrl/64)*64 + (hctrl%8)*8;
                auto track_index = controlbits.vctrltotrack.at(tobcoord)[vctrl%64] == 0? vctrl : (vctrl+64)%128;
                auto connector = bump->tob()->bump_track_connectors_chain(bump_index, track_index, dir);   // already set the state to "given_out"
                debug::debug_fmt("load_tobconnector(): calculated index = {}->{}->{}->{}", bump_index, hctrl, vctrl, track_index);

                auto track_coord = hardware::TrackCoord{bump->coord().row, bump->coord().col, hardware::TrackDirection::Vertical, track_index};
                auto track = interposer->get_track(track_coord);
                if (!track.has_value()) {
                    throw std::runtime_error(std::format("track {} is nullopt", track_coord));
                }

                // store in net->pathpackage
                auto& package = net->pathpackage();
                (dir == hardware::TOBBumpDirection::BumpToTOB ? package._tob_to_track : package._track_to_tob).emplace_back(
                    std::Tuple<hardware::Bump *, hardware::TOBConnector, hardware::Track *> {
                        bump, connector, track.value()
                    }
                );
            }
        }
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::format("load_tobconnector(): {}", e.what()));
    }
    }

    auto adj_tracks(hardware::Interposer* interposer, hardware::Track* track, const Controlbits& controlbits) -> std::Vector<std::Tuple<hardware::Track*, hardware::COBConnector>> {
        std::Vector<std::Tuple<hardware::Track*, hardware::COBConnector>> result {};
        for (auto& [adj_track, cobconnector]: interposer->adjacent_tracks(track)){
            const auto& [sw_direction, cob_coord, track_index] = sw_type(track, adj_track);
            const auto& controlbit = controlbits.cobsw.at(cob_coord).at(sw_direction);
            auto cob_index = interposer->get_cob(cob_coord).value()->track_index_to_cob_index(track_index);
            if (controlbit[cob_index])
                result.emplace_back(std::Tuple<hardware::Track*, hardware::COBConnector>{adj_track, cobconnector});
            else 
                continue;
        }
        
        return result;
    }

    // return [cob_sw_direction, cob_coord, track_index]
    auto sw_type(hardware::Track* track, hardware::Track* adj_track) -> std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
        const auto& coord = track->coord();
        const auto& adj_coord = adj_track->coord();

        switch (coord.dir) {
            case hardware::TrackDirection::Horizontal: {
                switch (adj_coord.dir) {
                    case hardware::TrackDirection::Horizontal: {
                        if (adj_coord.col == coord.col) {
                            throw std::runtime_error(
                                std::format("illegal adjacent track {} for track {}", adj_coord, coord)
                            );
                        }
                        auto cob_coord = hardware::COBCoord{
                            coord.row, (adj_coord.col > coord.col ? coord.col : adj_coord.col)
                        };
                        return std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize>{
                            hardware::COBSWDirection::H, cob_coord, coord.index
                        };
                    }
                    case hardware::TrackDirection::Vertical: {
                        auto cob_left_coord = hardware::COBCoord{
                            coord.row, coord.col - 1
                        };
                        auto cob_right_coord = hardware::COBCoord{
                            coord.row, coord.col
                        };
                        if (adj_coord.row > coord.row) {
                            return adj_coord.col < coord.col ? std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
                                hardware::COBSWDirection::RU, cob_left_coord, coord.index
                            } : std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
                                hardware::COBSWDirection::LU, cob_right_coord, coord.index
                            };
                        }
                        else if (adj_coord.row == coord.row) {
                            return adj_coord.col < coord.col ? std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
                                hardware::COBSWDirection::RD, cob_left_coord, coord.index
                            } : std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
                                hardware::COBSWDirection::LD, cob_right_coord, coord.index
                            };
                        }
                        else {
                            throw std::runtime_error(
                                std::format("adjacent track {} is two hop away from current track {}", adj_coord, coord)
                            );
                        }
                    }
                }
            }
            case hardware::TrackDirection::Vertical: {
                switch (adj_coord.dir) {
                    case hardware::TrackDirection::Vertical: {
                        if (adj_coord.row == coord.row) {
                            throw std::runtime_error(
                                std::format("illegal adjacent track {} for track {}", adj_coord, coord)
                            );
                        }
                        auto cob_coord = hardware::COBCoord{
                            (adj_coord.row > coord.row ? coord.row : adj_coord.row), coord.col
                        };
                        return std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize>{
                            hardware::COBSWDirection::V, cob_coord, coord.index
                        };
                    }
                    case hardware::TrackDirection::Horizontal: {
                        auto cob_up_coord = hardware::COBCoord{
                            coord.row, coord.col
                        };
                        auto cob_down_coord = hardware::COBCoord{
                            coord.row - 1, coord.col
                        };
                        if (adj_coord.col > coord.col) {
                            return adj_coord.row < coord.row ? std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
                                hardware::COBSWDirection::RU, cob_down_coord, adj_coord.index
                            } : std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
                                hardware::COBSWDirection::RD, cob_up_coord, adj_coord.index
                            };
                        }
                        else if (adj_coord.col == coord.col) {
                            return adj_coord.row < coord.row ? std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
                                hardware::COBSWDirection::LU, cob_down_coord, adj_coord.index
                            } : std::Tuple<hardware::COBSWDirection, hardware::COBCoord, std::usize> {
                                hardware::COBSWDirection::LD, cob_up_coord, adj_coord.index
                            };
                        }
                        else {
                            throw std::runtime_error(
                                std::format("adjacent track {} is two hop away from current track {}", adj_coord, coord)
                            );
                        }
                    }
                }
            }
        }
    }

    auto load_path_with_maze(circuit::Net* net, hardware::Interposer* interposer, const Controlbits& controlbits) -> void {
    try {
        auto [ports, flag] = net->track_ports();
        if (!flag) {
            throw std::logic_error(
                std::format("Error in net: {}\ncollected tracks number {} does not match with net's ports number {}", net->to_string(), ports.size(), net->port_number())
            );
        }

        auto begin_track = *ports.begin();
        std::Queue<hardware::Track*> q {};
        std::HashMap<hardware::Track*, std::Option<std::Tuple<hardware::Track*, hardware::COBConnector>>> prev_info {};
        auto& package = net->pathpackage();

        // init
        q.push(begin_track);
        prev_info.emplace(begin_track, std::nullopt);
        ports.erase(begin_track);

        // bfs
        while (!q.empty()) {
            auto cur = q.front();
            q.pop();

            if (ports.erase(cur)) {
                // backtrace
                while (true) {
                    auto prev = prev_info.find(cur);
                    if (!prev->second.has_value()) {    // reach begin track
                        break;
                    }
                    if (prev == prev_info.end()) {  // do not reach begin track, but cannot find prev info
                        throw std::runtime_error("Unexpected error in bits_to_paths(): cannot find prev info");
                    }

                    auto [prev_track, connector] = prev->second.value();
                    package._regular_path.emplace_back(cur, connector);
                    cur = prev_track;
                }
                package._regular_path.emplace_back(begin_track, std::nullopt);
                // reverse
                std::reverse(package._regular_path.begin(), package._regular_path.end());
                // set cobconnector state
                for (auto& [t, c]: package._regular_path) {
                    if (c.has_value()) {
                        c.value().suspend();
                    }
                }
            }

            for (auto& [next_track, connector] : adj_tracks(interposer, cur, controlbits)) {
                if (prev_info.find(next_track) != prev_info.end()) {
                    continue;
                }

                q.push(next_track);
                prev_info.emplace(next_track, std::Tuple<hardware::Track*, hardware::COBConnector>{cur, connector});
            }
        }
    }
    catch (const std::exception& e) {
        throw std::runtime_error(
            std::format("load path with maze(): {}", e.what())
        );
    }
    }

    auto load_tobconnector_sync(
        hardware::Interposer* interposer, const Controlbits& controlbits, circuit::SyncNet* net
    ) -> void {
        auto load = [&](circuit::Net* base_net) {
            auto bumps = base_net->nodes_direction();
            load_tobconnector(interposer, controlbits, bumps, base_net);
        };
        for (auto& n: net->btbnets()) {
            load(n.get());
        }
        for (auto& n: net->bttnets()) {
            load(n.get());
        }
        for (auto& n: net->ttbnets()) {
            load(n.get());
        }

        auto& sync_package = net->pathpackage();
        auto collect = [&](circuit::Net* base_net) {
            sync_package._tob_to_track.insert(
                sync_package._tob_to_track.end(),
                base_net->pathpackage()._tob_to_track.begin(),
                base_net->pathpackage()._tob_to_track.end()
            );
            sync_package._track_to_tob.insert(
                sync_package._track_to_tob.end(),
                base_net->pathpackage()._track_to_tob.begin(),
                base_net->pathpackage()._track_to_tob.end()
            );
        };
        for (auto& n: net->btbnets()) {
            collect(n.get());
        }
        for (auto& n: net->bttnets()) {
            collect(n.get());
        }
        for (auto& n: net->ttbnets()) {
            collect(n.get());
        }
    }

    auto load_path_with_maze_sync(circuit::SyncNet* net, hardware::Interposer* interposer, const Controlbits& controlbits) -> void {
        for (auto& n: net->btbnets()) {
            circuit::Net* base_net = n.get();
            load_path_with_maze(base_net, interposer, controlbits);
        }
        for (auto& n: net->bttnets()) {
            circuit::Net* base_net = n.get();
            load_path_with_maze(base_net, interposer, controlbits);
        }
        for (auto& n: net->ttbnets()) {
            circuit::Net* base_net = n.get();
            load_path_with_maze(base_net, interposer, controlbits);
        }

        auto& sync_package = net->pathpackage();
        auto collect = [&](circuit::Net* base_net) {
            sync_package._regular_path.insert(
                sync_package._regular_path.end(),
                base_net->pathpackage()._regular_path.begin(),
                base_net->pathpackage()._regular_path.end()
            );
        };
        for (auto& n: net->btbnets()) {
            collect(n.get());
        }
        for (auto& n: net->bttnets()) {
            collect(n.get());
        }
        for (auto& n: net->ttbnets()) {
            collect(n.get());
        }
    }

    auto load_length(circuit::Net* net) -> void {
        std::usize length{0};
        length += algo::path_length(net->pathpackage()._regular_path);
        length += net->pathpackage()._tob_to_track.size();
        length += net->pathpackage()._track_to_tob.size();

        net->pathpackage()._length = length;
    }

    auto load_length_sync(circuit::SyncNet* net) -> void {
        for (auto& n: net->btbnets()) {
            load_length(n.get());
        }
        for (auto& n: net->bttnets()) {
            load_length(n.get());
        }
        for (auto& n: net->ttbnets()) {
            load_length(n.get());
        }

        auto& sync_package = net->pathpackage();
        auto collect = [&](circuit::Net* base_net) {
            sync_package._length += base_net->pathpackage()._length;
        };
        for (auto& n: net->btbnets()) {
            collect(n.get());
        }
        for (auto& n: net->bttnets()) {
            collect(n.get());
        }
        for (auto& n: net->ttbnets()) {
            collect(n.get());
        }
    }

    auto show_path_from_bits(const std::Vector<std::Rc<circuit::Net>>& nets) -> void {
        debug::info("Show path from control bits:");

        for (const auto& net: nets) {
            debug::info(net->to_string());
            auto l = net->length();

            if (l > 0) {
                debug::info_fmt("Routing length of this net: {}", l);
                debug::info_fmt("Routing priority of this net: {}", net->priority().value());
                net->show_path();
            }
            else {
                debug::info_fmt("net {} is not routed", net->name());
            }
        }
    }

}



