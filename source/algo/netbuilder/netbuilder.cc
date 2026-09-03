
#include "./netbuilder.hh"

#include <hardware/interposer.hh>
#include <circuit/net/nets.hh>
#include <circuit/basedie.hh>
#include <circuit/connection/connection.hh>
#include <circuit/connection/pin.hh>
#include <utility/string.hh>
#include <std/utility.hh>
#include <debug/debug.hh>
#include <std/format.hh>

namespace PR_tool::algo {

    void build_nets(circuit::BaseDie* basedie, hardware::Interposer* interposer) {
        algo::NetBuilder{basedie, interposer}.build();
    }

    NetBuilder::NetBuilder(circuit::BaseDie* basedie, hardware::Interposer* interposer) :
        _interposer{interposer},
        _basedie{basedie}
    {
    }

    auto NetBuilder::build() -> void
    try {
        build_01_ports();
        for (auto& [mode, inner_connection]: this->_basedie->connections()) {
            auto bumps_with_pose = std::Vector<hardware::Bump*> {};
            auto bumps_with_nege = std::Vector<hardware::Bump*> {};
            for (auto& [sync, connections] : inner_connection) {
                if (sync == -1) {
                    this->build_no_sync_nets(
                        connections, -1, mode, bumps_with_pose, bumps_with_nege);
                } else {
                    this->build_sync_net(connections, sync, mode);
                }
            }
            this->build_fixed_nets(mode, bumps_with_pose, bumps_with_nege);
        }
    }
    THROW_UP_WITH("Build nets")

    auto NetBuilder::build_no_sync_nets(
        std::Span<const std::Box<circuit::Connection>> connections,
        int group,
        int m,
        std::Vector<hardware::Bump*>& bumps_with_pose,
        std::Vector<hardware::Bump*>& bumps_with_nege
    ) -> void {
        // How to deal one to mul net?
        // Build a map: start to ends
        auto track_to_bumps = std::HashMap<hardware::Track*, std::Vector<hardware::Bump*>>{};
        auto bump_to_bumps = std::HashMap<hardware::Bump*, std::Vector<hardware::Bump*>>{};
        auto bump_to_tracks = std::HashMap<hardware::Bump*, std::Vector<hardware::Track*>>{};

        // Loop for each connection...
        for (auto& connection : connections) {
            auto& input = connection->input_pin();
            auto& output = connection->output_pin();

            if (output.is_fixed()) {
                debug::exception("Fixed pin as target??");
            }

            auto end_node = this->pin_to_node(output);

            if (input.is_vdd()) {
                // Input is 'vdd'
                std::match(end_node, 
                    [&](hardware::Track* track) {
                        debug::exception_fmt("Invalid net with track to track: {} => {}", input, output);
                    },
                    [&](hardware::Bump* bump) {
                        bumps_with_pose.emplace_back(bump);
                    }
                );
            } 
            else if (input.is_gnd()) {
                // Input is 'gnd'
                std::match(end_node, 
                    [&](hardware::Track* track) {
                        debug::exception_fmt("Invalid net with track to track: {} => {}", input, output);
                    },
                    [&](hardware::Bump* bump) {
                        bumps_with_nege.emplace_back(bump);
                    }
                );
            } 
            else {
                // Four case
                auto begin_node = this->pin_to_node(input);

                std::match(end_node, 
                    [&](hardware::Track* end_track) {
                        std::match(begin_node, 
                            [&](hardware::Track* begin_track) {
                                // 1. Track => Track
                                debug::exception_fmt("Invalid net with track to track: {} => {}", input, output);
                            },
                            [&](hardware::Bump* begin_bump) {
                                // 2. Bump => Track
                                auto& end_tracks = bump_to_tracks.emplace(begin_bump, std::Vector<hardware::Track*>{}).first->second;
                                end_tracks.emplace_back(end_track);
                            }
                        );
                    },
                    [&](hardware::Bump* end_bump) {
                        std::match(begin_node, 
                            [&](hardware::Track* begin_track) {
                                // 3. Track => Bump
                                auto& end_bumps = track_to_bumps.emplace(begin_track, std::Vector<hardware::Bump*>{}).first->second;
                                end_bumps.emplace_back(end_bump);  
                            },
                            [&](hardware::Bump* begin_bump) {
                                // 4. Bump => Bump
                                auto& end_bumps = bump_to_bumps.emplace(begin_bump, std::Vector<hardware::Bump*>{}).first->second;
                                end_bumps.emplace_back(end_bump);
                            }
                        );
                    }
                );
            }
        }

        // Track => Bump
        for (auto& [begin_track, end_bumps] : track_to_bumps) {
            auto coord = begin_track->coord();
            const auto owner_bumps = end_bumps;

            // Create net 
            auto net = std::Rc<circuit::Net>{};
            if (end_bumps.size() == 1) {
                auto net_name = std::String{
                    std::format("TrackToBumpNet_from_cob_{}_{}_to_cob_{}_{}_in_group_{}", begin_track->coord().row, begin_track->coord().col, end_bumps[0]->coord().row, end_bumps[0]->coord().col, group)
                };
                auto uid = std::format(
                    "{}:{}->{}",
                    make_uid_prefix(m, group, "ttb"),
                    track_uid_token(begin_track),
                    bump_uid_token(end_bumps[0]));
                net = std::make_shared<circuit::TrackToBumpNet>(begin_track, end_bumps[0], std::HashSet<int>{m}, net_name, uid);
            } else {
                auto net_name = std::String{
                    std::format("TrackToBumpsNet_from_cob_{}_{}_in_group_{}_with_{}_ends", coord.row, coord.col, group, end_bumps.size())
                };
                auto end_tokens = std::Vector<std::String> {};
                end_tokens.reserve(end_bumps.size());
                for (const auto* bump : end_bumps) {
                    end_tokens.emplace_back(bump_uid_token(bump));
                }
                sort_uid_tokens(end_tokens);
                auto end_part = std::String {};
                for (std::size_t i = 0; i < end_tokens.size(); ++i) {
                    if (i != 0) {
                        end_part += ",";
                    }
                    end_part += end_tokens[i];
                }
                auto uid = std::format(
                    "{}:{}->{}",
                    make_uid_prefix(m, group, "ttbs"),
                    track_uid_token(begin_track),
                    end_part);
                net = std::make_shared<circuit::TrackToBumpsNet>(begin_track, std::move(end_bumps), std::HashSet<int>{m}, net_name, uid);
            }

            this->register_net_at_bump_owners(net.get(), owner_bumps);
            this->_basedie->add_net(net, m);
        }

        // Bump => Bump
        for (auto& [begin_bump, end_bumps] : bump_to_bumps) {
            auto coord = begin_bump->coord();

            // Create net 
            auto owner_bumps = end_bumps;
            owner_bumps.emplace_back(begin_bump);
            auto net = std::Rc<circuit::Net>{};
            if (end_bumps.size() == 1) {
                auto net_name = std::String{
                    std::format("BumpToBumpNet_from_cob_{}_{}_to_cob_{}_{}_in_group_{}", begin_bump->coord().row, begin_bump->coord().col, end_bumps[0]->coord().row, end_bumps[0]->coord().col, group)
                };
                auto uid = std::format(
                    "{}:{}->{}",
                    make_uid_prefix(m, group, "btb"),
                    bump_uid_token(begin_bump),
                    bump_uid_token(end_bumps[0]));
                net = std::make_shared<circuit::BumpToBumpNet>(begin_bump, end_bumps.front(), std::HashSet<int>{m}, net_name, uid);
            } else {
                auto net_name = std::String{
                    std::format("BumpToBumpsNet_from_cob_{}_{}_in_group_{}_with_{}_ends", coord.row, coord.col, group, end_bumps.size())
                };
                auto end_tokens = std::Vector<std::String> {};
                end_tokens.reserve(end_bumps.size());
                for (const auto* bump : end_bumps) {
                    end_tokens.emplace_back(bump_uid_token(bump));
                }
                sort_uid_tokens(end_tokens);
                auto end_part = std::String {};
                for (std::size_t i = 0; i < end_tokens.size(); ++i) {
                    if (i != 0) {
                        end_part += ",";
                    }
                    end_part += end_tokens[i];
                }
                auto uid = std::format(
                    "{}:{}->{}",
                    make_uid_prefix(m, group, "btbs"),
                    bump_uid_token(begin_bump),
                    end_part);
                net = std::make_shared<circuit::BumpToBumpsNet>(begin_bump, std::move(end_bumps), std::HashSet<int>{m}, net_name, uid);
            }

            this->register_net_at_bump_owners(net.get(), owner_bumps);
            this->_basedie->add_net(net, m);
        }

        // Bump => Track
        for (auto& [begin_bump, end_tracks] : bump_to_tracks) {
            auto coord = begin_bump->coord();

            // Create net 
            auto net = std::Rc<circuit::Net>{};
            if (end_tracks.size() == 1) {
                auto net_name = std::String{
                    std::format("BumpToTrackNet_from_cob_{}_{}_to_cob_{}_{}_in_group_{}", begin_bump->coord().row, begin_bump->coord().col, end_tracks[0]->coord().row, end_tracks[0]->coord().col, group)
                };
                auto uid = std::format(
                    "{}:{}->{}",
                    make_uid_prefix(m, group, "btt"),
                    bump_uid_token(begin_bump),
                    track_uid_token(end_tracks[0]));
                net = std::make_shared<circuit::BumpToTrackNet>(begin_bump, end_tracks.front(), std::HashSet<int>{m}, net_name, uid);
            } else {
                auto net_name = std::String{
                    std::format("BumpToTracksNet_from_cob_{}_{}_in_group_{}_with_{}_ends", coord.row, coord.col, group, end_tracks.size())
                };
                auto end_tokens = std::Vector<std::String> {};
                end_tokens.reserve(end_tracks.size());
                for (const auto* track : end_tracks) {
                    end_tokens.emplace_back(track_uid_token(track));
                }
                sort_uid_tokens(end_tokens);
                auto end_part = std::String {};
                for (std::size_t i = 0; i < end_tokens.size(); ++i) {
                    if (i != 0) {
                        end_part += ",";
                    }
                    end_part += end_tokens[i];
                }
                auto uid = std::format(
                    "{}:{}->{}",
                    make_uid_prefix(m, group, "btts"),
                    bump_uid_token(begin_bump),
                    end_part);
                net = std::make_shared<circuit::BumpToTracksNet>(begin_bump, std::move(end_tracks), std::HashSet<int>{m}, net_name, uid);
            }

            // Insert to basedie & topdieinsts
            this->_bump_to_topdie_inst.at(begin_bump)->add_net(net.get());
            this->_basedie->add_net(net, m);
        }
    }

    auto NetBuilder::build_sync_net(std::Span<const std::Box<circuit::Connection>> connections, int group, int m) -> void {
        auto btb_sync_nets = std::Vector<std::Rc<circuit::BumpToBumpNet>>{};
        auto btt_sync_nets = std::Vector<std::Rc<circuit::BumpToTrackNet>>{};
        auto ttb_sync_nets = std::Vector<std::Rc<circuit::TrackToBumpNet>>{};

        for (auto& connection : connections) {
            auto& input = connection->input_pin();
            auto& output = connection->output_pin();

            if (input.is_fixed() || output.is_fixed()) {
                debug::exception("Fixed pin can't as sync");
            }

            auto begin_node  = this->pin_to_node(input);
            auto end_node = this->pin_to_node(output);
        
            // Each connec in sync net should has the same TOB size. 
            // We do not check this condition.... :)
            // Case four case
            std::match(end_node, 
                [&](hardware::Track* end_track) {
                    std::match(begin_node, 
                        [&](hardware::Track* begin_track) {
                            // 1. Track => Track
                            debug::exception_fmt("Invalid net with track to track: {} => {}", input, output);
                        },
                        [&](hardware::Bump* begin_bump) {
                            // 2. Bump => Track
                            auto net_name = std::String{
                                std::format("BumpToTrackNet_from_cob_{}_{}_to_cob_{}_{}_in_group_{}", begin_bump->coord().row, begin_bump->coord().col, end_track->coord().row, end_track->coord().col, group)
                            };
                            auto uid = std::format(
                                "{}:{}->{}",
                                make_uid_prefix(m, group, "btt"),
                                bump_uid_token(begin_bump),
                                track_uid_token(end_track));
                            auto net = std::make_shared<circuit::BumpToTrackNet>(begin_bump, end_track, std::HashSet<int>{m}, net_name, uid);
                            this->_bump_to_topdie_inst.at(begin_bump)->add_net(net.get());
                            btt_sync_nets.emplace_back(std::move(net));
                        }
                    );
                },
                [&](hardware::Bump* end_bump) {
                    std::match(begin_node, 
                        [&](hardware::Track* begin_track) {
                            // 3. Track => Bump
                            auto net_name = std::String{
                                std::format("TrackToBumpNet_from_cob_{}_{}_to_cob_{}_{}_in_group_{}", begin_track->coord().row, begin_track->coord().col, end_bump->coord().row, end_bump->coord().col, group)
                            };
                            auto uid = std::format(
                                "{}:{}->{}",
                                make_uid_prefix(m, group, "ttb"),
                                track_uid_token(begin_track),
                                bump_uid_token(end_bump));
                            auto net = std::make_shared<circuit::TrackToBumpNet>(begin_track, end_bump, std::HashSet<int>{m}, net_name, uid);
                            this->_bump_to_topdie_inst.at(end_bump)->add_net(net.get());
                            ttb_sync_nets.emplace_back(std::move(net)); 
                        },
                        [&](hardware::Bump* begin_bump) {
                            // 4. Bump to Bump
                            auto net_name = std::String{
                                std::format("BumpToBumpNet_from_cob_{}_{}_to_cob_{}_{}_in_group_{}", begin_bump->coord().row, begin_bump->coord().col, end_bump->coord().row, end_bump->coord().col, group)
                            };
                            auto uid = std::format(
                                "{}:{}->{}",
                                make_uid_prefix(m, group, "btb"),
                                bump_uid_token(begin_bump),
                                bump_uid_token(end_bump));
                            auto net = std::make_shared<circuit::BumpToBumpNet>(begin_bump, end_bump, std::HashSet<int>{m}, net_name, uid);
                            this->_bump_to_topdie_inst.at(begin_bump)->add_net(net.get());
                            this->_bump_to_topdie_inst.at(end_bump)->add_net(net.get());
                            btb_sync_nets.emplace_back(std::move(net)); 
                        }
                    );
                }
            );
        }

        auto net_name = std::String(std::format("SyncNet in group {}", group));
        auto sync_uid = std::format(
            "{}:s",
            make_uid_prefix(m, group, "s"));
        this->_basedie->add_net(std::make_shared<circuit::SyncNet>(
            btb_sync_nets,
            btt_sync_nets,
            ttb_sync_nets,
            std::HashSet<int>{m},
            net_name,
            sync_uid
        ), m);
    }

    auto NetBuilder::build_fixed_nets(
        int m,
        const std::Vector<hardware::Bump*>& bumps_with_pose,
        const std::Vector<hardware::Bump*>& bumps_with_nege
    ) -> void {
        // Add nege
        if (!bumps_with_nege.empty()) {
            auto nege_tracks = std::Vector<hardware::Track*>{};
            for (auto& track_coord : NetBuilder::_nege_tracks) {
                auto track = this->_interposer->get_track(track_coord);
                if (!track.has_value()) {
                    debug::exception_fmt("Invalid track coord '{}'", track_coord);
                } else {
                    nege_tracks.emplace_back(*track);
                }
            }

            auto net_name = std::String("Nege nets");
            auto begin_tokens = std::Vector<std::String> {};
            begin_tokens.reserve(nege_tracks.size());
            for (const auto* track : nege_tracks) {
                begin_tokens.emplace_back(track_uid_token(track));
            }
            sort_uid_tokens(begin_tokens);
            auto end_tokens = std::Vector<std::String> {};
            end_tokens.reserve(bumps_with_nege.size());
            for (const auto* bump : bumps_with_nege) {
                end_tokens.emplace_back(bump_uid_token(bump));
            }
            sort_uid_tokens(end_tokens);
            auto begin_part = std::String {};
            for (std::size_t i = 0; i < begin_tokens.size(); ++i) {
                if (i != 0) {
                    begin_part += ",";
                }
                begin_part += begin_tokens[i];
            }
            auto end_part = std::String {};
            for (std::size_t i = 0; i < end_tokens.size(); ++i) {
                if (i != 0) {
                    end_part += ",";
                }
                end_part += end_tokens[i];
            }
            auto uid = std::format(
                "{}:{}->{}",
                make_uid_prefix(m, -1, "tstbs"),
                begin_part,
                end_part);
            auto net = std::make_shared<circuit::TracksToBumpsNet>(
                std::move(nege_tracks), bumps_with_nege, std::HashSet<int>{m}, net_name, uid);
            this->register_net_at_bump_owners(net.get(), net->end_bumps());
            this->_basedie->add_net(net, m);
        }

        // Add pose
        if (!bumps_with_pose.empty()) {
            auto pose_tracks = std::Vector<hardware::Track*>{};
            for (auto& track_coord : NetBuilder::_pose_tracks) {
                auto track = this->_interposer->get_track(track_coord);
                if (!track.has_value()) {
                    debug::exception_fmt("Invalid track coord '{}'", track_coord);
                } else {
                    pose_tracks.emplace_back(*track);
                }
            }

            auto net_name = std::String("Pose nets");
            auto begin_tokens = std::Vector<std::String> {};
            begin_tokens.reserve(pose_tracks.size());
            for (const auto* track : pose_tracks) {
                begin_tokens.emplace_back(track_uid_token(track));
            }
            sort_uid_tokens(begin_tokens);
            auto end_tokens = std::Vector<std::String> {};
            end_tokens.reserve(bumps_with_pose.size());
            for (const auto* bump : bumps_with_pose) {
                end_tokens.emplace_back(bump_uid_token(bump));
            }
            sort_uid_tokens(end_tokens);
            auto begin_part = std::String {};
            for (std::size_t i = 0; i < begin_tokens.size(); ++i) {
                if (i != 0) {
                    begin_part += ",";
                }
                begin_part += begin_tokens[i];
            }
            auto end_part = std::String {};
            for (std::size_t i = 0; i < end_tokens.size(); ++i) {
                if (i != 0) {
                    end_part += ",";
                }
                end_part += end_tokens[i];
            }
            auto uid = std::format(
                "{}:{}->{}",
                make_uid_prefix(m, -1, "tstbs"),
                begin_part,
                end_part);
            auto net = std::make_shared<circuit::TracksToBumpsNet>(
                std::move(pose_tracks), bumps_with_pose, std::HashSet<int>{m}, net_name, uid);
            this->register_net_at_bump_owners(net.get(), net->end_bumps());
            this->_basedie->add_net(net, m);
        }
    }

    auto NetBuilder::register_net_at_bump_owners(
        circuit::Net* net,
        const std::Vector<hardware::Bump*>& bumps
    ) -> void {
        auto topdie_insts = std::HashSet<circuit::TopDieInstance*> {};
        for (auto* bump : bumps) {
            const auto it = this->_bump_to_topdie_inst.find(bump);
            if (it == this->_bump_to_topdie_inst.end()) {
                debug::exception_fmt("Missing topdie instance for bump {}", bump->coord());
            }
            else {
                topdie_insts.emplace(it->second);
            }
        }
        for (auto* topdie_inst : topdie_insts) {
            topdie_inst->add_net(net);
        }
    }

    auto NetBuilder::pin_to_node(const circuit::Pin& pin) -> Node {
        if (pin.is_external_port()) {
            const auto external_port = pin.to_connect_export().port;
            auto track = this->_interposer->get_track(external_port->coord());
            if (!track.has_value()) {
                debug::exception_fmt("External port '{}' has an invalid track coord", external_port->name(), external_port->coord());
            } else {
                return Node{*track};
            }
        }
        else if (pin.is_bump()) {
            /// 
            /// Topdie inst: Search inst by name, and get bump index, and get bump object in interposer
            ///
            // Is pin exit?
            auto& connect_bump = pin.to_connect_bump();
            auto topdie = connect_bump.inst->topdie(); 
            auto res = topdie->pins_map().find(connect_bump.name);
            if (res == topdie->pins_map().end()) {
                debug::exception_fmt("No exit pin name '{}' in topdie '{}'", connect_bump.name, topdie->name());
            }

            auto coord = connect_bump.inst->tob()->coord();
            auto index = res->second;

            auto bump = this->_interposer->get_bump(coord, index);
            if (!bump.has_value()) {
                debug::exception_fmt("Pin '{}' has an invalid bump coord: TOBCoord '{}' with '{}'", connect_bump.name, coord, index);
            }
            this->_bump_to_topdie_inst.emplace(*bump, connect_bump.inst);
            return Node{*bump};
        }
        
        debug::unreachable("This pin can't be source type!!!");
    }

    auto NetBuilder::build_01_ports() -> void {
        this->_pose_tracks = _basedie->pose_ports();
        this->_nege_tracks = _basedie->nege_ports();
    }

    auto NetBuilder::make_uid_prefix(int mode, int group, std::StringView type_token) const -> std::String {
        return std::format("m{}:g{}:{}", mode, group, type_token);
    }

    auto NetBuilder::bump_uid_token(const hardware::Bump* bump) const -> std::String {
        const auto tob_coord = bump->tob()->coord();
        return std::format("(t{},t{},{})", tob_coord.row, tob_coord.col, bump->index());
    }

    auto NetBuilder::track_uid_token(const hardware::Track* track) const -> std::String {
        const auto tc = track->coord();
        const auto dir = tc.dir == hardware::TrackDirection::Horizontal ? "h" : "v";
        return std::format("(c{},c{},{},{})", tc.row, tc.col, dir, tc.index);
    }

    auto NetBuilder::sort_uid_tokens(std::Vector<std::String>& tokens) const -> void {
        std::sort(tokens.begin(), tokens.end());
    }

}
