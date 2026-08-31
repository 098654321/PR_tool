#include "./route_nets.hh"
#include "./command_mode/invoker.hh"
#include "./routeengine.hh"
#include "./incremental/bound_bits/global_bits.hh"
#include <algo/router/routeerror.hh>
#include "debug/debug.hh"
#include <circuit/basedie.hh>
#include <hardware/interposer.hh>
#include <algorithm>
#include <ranges>


namespace PR_tool::algo {

    auto route_nets(
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        const RouteStrategy& strateg,
        const AllocateStrategy& allocator,
        int m,
        bool incremental,
        bool try_all_modes, 
        bool path_exists,
        RouteProgressFn on_progress
    ) -> RouteNetsResult {
        debug::info(
            "\n\
            **********************************************************************************\n\
                                            Route nets\n\
            **********************************************************************************\
            "
        );
        auto invoker = Invoker{};
        auto engine = RouteEngine{basedie->nets(), strateg, allocator, m, incremental, try_all_modes, path_exists, interposer};
        engine.set_progress_callback(std::move(on_progress));
        invoker.set_route_commands(incremental, try_all_modes, path_exists);
        
        while (!invoker.check_command()) {
            try {
                invoker.invoke(interposer, engine);
            } 
            catch (const RetryExpt& err) {
                throw FinalError(err.what());
            }
            catch (const FinalError& err){
                debug::info_fmt("route_nets(): {}", err.what());
                throw err;
            }
            catch (const std::exception& err){
                throw std::runtime_error("route_nets() >> " + std::String(err.what()));
            }
        }

        auto route_data = analyze_results(interposer, engine, incremental, try_all_modes);
        return RouteNetsResult{route_data, engine.failed_net_names()};
    }

    // return total length of all nets
    auto analyze_results(
        hardware::Interposer* interposer,
        RouteEngine& engine,
        bool incremental,
        bool try_all_modes
    ) -> DataPerCycle {
        // record length info
        debug::info(
            "\n\
            **********************************************************************************\n\
                                            Net & Path Infomation\n\
            **********************************************************************************\
            "
        );
        engine.show_net_and_path();

        // show length info
        debug::info(
            "\n\
            **********************************************************************************\n\
                                            Data Analysis\n\
            **********************************************************************************\
            "
        );

        const auto& nets = engine.all_nets_in_modes(engine.mode());
        auto data = engine.show_final_data(nets, incremental);
        return data;
    }

    auto show_retry_expt(circuit::Net* net, RouteEngine& engine, hardware::Interposer* interposer) -> void {
        debug::debug(std::String("\nShow details of retry exception" + net->to_string()));
        
        auto state = net_connection_state(net, interposer);

        show_bump_resources(state, net, interposer, engine);

        debug::debug("\n");
    }

    auto net_connection_state(
        circuit::Net* net, hardware::Interposer*interposer
    ) -> std::Tuple<std::Vector<const PR_tool::hardware::Bump *>, std::Vector<const PR_tool::hardware::Bump *>, std::Vector<const PR_tool::hardware::Track *>> {
        auto state = net->connection_state();
        auto [routable_bumps, unroutable_bumps, unroutable_tracks] = state;

        debug::debug("routable bumps of this net: ");
        if (!routable_bumps.empty()) {
            for (auto& bump: routable_bumps) {
                debug::debug(bump->coord().to_string());

                auto [do_not_care, tobconnector, connected_track] = net->pathpackage().find_bump(bump).value();
                debug::debug_fmt("connected with track {}", connected_track->coord().to_string());

                debug::debug("More available tracks:");
                auto available_tracks = interposer->available_tracks(
                    const_cast<hardware::Bump*>(bump), tobconnector.single_direction()
                );
                if (!available_tracks.empty()) {
                    for (auto& [track, _]: available_tracks) {
                        debug::debug(track->coord().to_string());
                    }
                }
                else {
                    debug::debug("Empty");
                }
            }
            debug::debug("\n");
        }
        else {
            debug::debug("Empty\n");
        }

        debug::debug("unroutable bumps of this net: ");
        if (!unroutable_bumps.empty()) {
            for (auto& bump: unroutable_bumps) {
                debug::debug(bump->coord().to_string());

                debug::debug("More available tracks for TOB_to_track_direction:");
                auto available_tracks_bump_to_track = interposer->available_tracks_bump_to_track(
                    const_cast<hardware::Bump*>(bump)
                );
                if (!available_tracks_bump_to_track.empty()) {
                    for (auto& [track, _]: available_tracks_bump_to_track) {
                        debug::debug(track->coord().to_string());
                    }
                }
                else {
                    debug::debug("Empty");
                }

                debug::debug("More available tracks for track_to_TOB_direction:");
                auto available_tracks_track_to_bump = interposer->available_tracks_track_to_bump(
                    const_cast<hardware::Bump*>(bump)
                );
                if (!available_tracks_track_to_bump.empty()) {
                    for (auto& [track, _]: available_tracks_track_to_bump) {
                        debug::debug(track->coord().to_string());
                    }
                }
                else {
                    debug::debug("Empty");
                }
            }
            debug::debug("\n");
        }
        else {
            debug::debug("Empty\n");
        }

        debug::debug("unroutable tracks of this net");
        for (auto& track: unroutable_tracks) {
            debug::debug(track->coord().to_string());
        };

        return state;
    }


    auto show_bump_resources(
        const std::Tuple<std::Vector<const PR_tool::hardware::Bump *>, std::Vector<const PR_tool::hardware::Bump *>, std::Vector<const PR_tool::hardware::Track *>>& state,
        circuit::Net* net, hardware::Interposer* interposer, RouteEngine& engine
    ) -> void {
        const auto&[routable_bumps, unroutable_bumps, unroutable_tracks] = state;

        if (!unroutable_bumps.empty()) {
            for (auto& track: unroutable_tracks) {
                auto necessary_index = necessary_tracks(track);
                
                for (auto& bump: unroutable_bumps) {
                    auto unoccupied_access_index = search_bumps_connected_with_track(
                        bump, engine, interposer, necessary_index
                    );

                    /* search unoccupied inner mux */
                    std::String msg {"Necessary indexes still unoccupied now: "};
                    for (auto i: unoccupied_access_index) {
                        msg = msg + std::to_string(i) + ", ";
                    }
                    debug::debug(msg);

                    // find tob
                    auto bump_coord = bump->coord();
                    auto tob = interposer->get_tob((bump_coord.row-1)/2, bump_coord.col/3);
                    assert(tob.has_value() && (*tob) != nullptr);

                    // vert-mux
                    auto vert_to_track_mux = unoccupied_vert_to_track_mux(unoccupied_access_index, *tob);

                    // hori-mux
                    auto hori_to_vert_mux = unoccupied_hori_to_vert_mux(vert_to_track_mux, *tob);

                    // bump
                    auto bump_to_hori_mux = unoccupied_bump_to_hori_mux(hori_to_vert_mux, *tob);

                    msg = "Bumps that can be connected to necessary tracks: ";
                    for (auto i: bump_to_hori_mux) {
                        msg = msg + std::to_string(i) + ", ";
                    }
                    debug::debug(msg);
                }
                debug::debug("\n");
            }
        }
    }

    // necessary indexes to connect to the track
    auto necessary_tracks(const hardware::Track* track) -> std::HashSet<std::usize> {
        auto track_coord = track->coord();
        debug::debug_fmt("For track {} :", track_coord.to_string());

        // search
        auto index = track_coord.index;
        auto necessary_index = std::HashSet<std::usize>{};
        for (std::usize i = 0; i < 8; ++i) {
            necessary_index.emplace(
                (index < 64 ? 0 : 64) + i*8 + (index%8)
            );
        }

        // show necessary indexes
        std::String msg {"Necessary indexes: "};
        for (auto i: necessary_index) {
            msg = msg + std::to_string(i) + ", ";
        }
        debug::debug(msg);

        return necessary_index;
    }

    // search bumps which are on the same tob with unrouted bumps 
    // and are already been connected with tracks having the necessary indexes
    auto search_bumps_connected_with_track(
        const hardware::Bump* bump, RouteEngine& engine, hardware::Interposer* interposer,
        const std::HashSet<std::usize>& accessible_index
    ) -> std::HashSet<std::usize> {
        auto bump_coord = bump->coord();
        auto unoccupied_access_index = accessible_index;

        std::HashMap<const hardware::Bump*, const hardware::Track*> filtered_bumps {};
        for (const auto& net: engine.routed_nets()) {
            const auto& package = net->pathpackage();

            auto filter_bumps = [&](
                const std::Tuple<PR_tool::hardware::Bump *, PR_tool::hardware::TOBConnector, PR_tool::hardware::Track *>& t
            ) {
                const auto& [b_ptr, tobconnector, t_ptr] = t;
                const auto& c = b_ptr->coord();
                if (c.col == bump_coord.col && c.row == bump_coord.row && accessible_index.contains(t_ptr->coord().index)) {
                    filtered_bumps.emplace(b_ptr, t_ptr);
                    std::erase_if(unoccupied_access_index, [&](std::usize i) {
                        return i == t_ptr->coord().index;
                    });
                }
            };

            std::for_each(package._tob_to_track.begin(), package._tob_to_track.end(), filter_bumps);
            std::for_each(package._track_to_tob.begin(), package._track_to_tob.end(), filter_bumps);
        }

        // show bumps meet the condition
        for (const auto& [b_ptr, t_ptr]: filtered_bumps) {    
            debug::debug_fmt("Related bump: {}", b_ptr->coord().to_string());
            debug::debug_fmt("Connected with track: {}", t_ptr->coord().to_string());

            auto show_available_tracks = [&](const std::HashMap<hardware::Track *, hardware::TOBConnector>& tracks_pack) {
                if (!tracks_pack.empty()) {
                    for (auto& [track, _]: tracks_pack) {
                        debug::debug(track->coord().to_string());
                    }
                }
                else {
                    debug::debug("Empty");
                }
            };

            debug::debug("More available tracks for TOB_to_track_direction:");
            show_available_tracks(interposer->available_tracks_bump_to_track(const_cast<hardware::Bump*>(b_ptr)));

            debug::debug("More available tracks for track_to_TOB_direction:");
            show_available_tracks(interposer->available_tracks_track_to_bump(const_cast<hardware::Bump*>(b_ptr)));
        }

        return unoccupied_access_index;
    }

    auto unoccupied_vert_to_track_mux(const std::HashSet<std::usize>& unoccupied_access_index, const hardware::TOB* tob) -> std::HashSet<std::usize> {
        std::HashSet<std::usize> unoccupied_vert_mux {};
        for (auto i: unoccupied_access_index) {
            unoccupied_vert_mux.emplace(i);
            unoccupied_vert_mux.emplace((i+64)%128);
        }
        const auto& vert_muxs = tob->vert_to_track_muxs();
        std::erase_if(unoccupied_vert_mux, [&](std::usize i) {
            auto [group, group_index] = tob->vert_to_track_mux_info(i);
            return tob->vert_to_track_register_nth(group, group_index)->is_given_out();
        });
        std::String msg {"necessary & unoccupied vert to track mux: "};
        for (auto i: unoccupied_vert_mux) {
            msg = msg + std::to_string(i) + ", ";
        }
        debug::debug(msg);

        return unoccupied_vert_mux;
    }

    auto unoccupied_hori_to_vert_mux(const std::HashSet<std::usize>& vert_mux, const hardware::TOB* tob) -> std::HashSet<std::usize> {
        std::HashSet<std::usize> unoccupied_hori_mux {};
        for (auto i: vert_mux) {
            auto bank = i / 64;
            auto bank_i = i - 64*bank;
            auto group = bank_i / 8;
            for (std::usize group_index = 0; group_index < 8; ++group_index) {
                unoccupied_hori_mux.emplace(bank*64 + group_index*8 + group);
            }
        }
        std::erase_if(unoccupied_hori_mux, [&](std::usize i) {
            auto [group, group_index] = tob->hori_to_vert_mux_info(i);
            return tob->hori_to_vert_register_nth(group, group_index)->is_given_out();
        });
        std::String msg = "necessary & unoccupied hori to vert mux: ";
        for (auto i: unoccupied_hori_mux) {
            msg = msg + std::to_string(i) + ", ";
        }
        debug::debug(msg);

        return unoccupied_hori_mux;
    }

    auto unoccupied_bump_to_hori_mux(const std::HashSet<std::usize>& hori_mux, const hardware::TOB* tob) -> std::HashSet<std::usize> {
        std::HashSet<std::usize> unoccupied_bump_mux {};

        for (auto i: hori_mux) {
            auto bank = i / 64;
            auto bank_i = i - 64*bank;
            auto group = bank_i / 8;
            for (std::usize group_index = 0; group_index < 8; ++ group_index) {
                unoccupied_bump_mux.emplace(bank*64 + group*8 + group_index);
            }
        }
        std::erase_if(unoccupied_bump_mux, [&](std::usize i) {
            auto [group, group_index] = tob->bump_to_hori_mux_info(i);
            return tob->bump_to_hori_register_nth(group, group_index)->is_given_out();
        });
        std::String msg = "necessary & unoccupied bump to hori mux: ";
        for (auto i: unoccupied_bump_mux) {
            msg = msg + std::to_string(i) + ", ";
        }
        debug::debug(msg);

        return unoccupied_bump_mux;
    }
}