#include <algo/router/common/routing_budget.hh>
#include "./mazeroutestrategy.hh"
#include "./path_length.hh"
#include "hardware/track/track.hh"

#include <circuit/net/nets.hh>
#include <hardware/interposer.hh>
#include <algo/router/routeerror.hh>

#include <std/integer.hh>
#include <std/collection.hh>
#include <std/utility.hh>
#include <std/range.hh>
#include <debug/debug.hh>
#include <algorithm>
#include <std/format.hh>

#include <cassert>
#include <type_traits>

namespace PR_tool::algo {

    auto MazeRouteStrategy::route_bump_to_bump_net(
        hardware::Interposer* interposer, circuit::BumpToBumpNet* net
    ) const -> void {
    try {
        debug::info(std::format("Maze routing for {}", net->name()));
        
        auto begin_bump = net->begin_bump();
        auto end_bump = net->end_bump();
        debug::check(begin_bump->tob() != end_bump->tob(), "Route bump in the same tob");

        // existing path
        auto begin_tracks_vec = existing_path_vec<hardware::Bump>(begin_bump, net);
        auto end_tracks_set = existing_path_set<hardware::Bump>(end_bump, net);

        // route
        auto head = std::Pair<hardware::Bump*, std::Vector<hardware::Track*>>{begin_bump, begin_tracks_vec};
        auto tail = std::Pair<hardware::Bump*, std::HashSet<hardware::Track*>>{end_bump, end_tracks_set};
        auto path_package = this->route_node_to_node_net<hardware::Bump, hardware::Bump>(
            interposer, head, tail
        );

        net->set_pathpackage(path_package);
    }
    catch(RetryExpt& e){
        e.set_net(net);
        throw e;
    }
    }

    auto MazeRouteStrategy::route_track_to_bump_net(
        hardware::Interposer* interposer, circuit::TrackToBumpNet* net
    ) const -> void {
    try {
        debug::info(std::format("Maze routing for {}", net->name()));
        
        auto begin_track = net->begin_track();
        auto end_bump = net->end_bump();

        // existing path
        auto begin_tracks_vec = existing_path_vec<hardware::Track>(begin_track, net);
        auto end_tracks_set = existing_path_set<hardware::Bump>(end_bump, net);

        // route
        auto head = std::Pair<hardware::Track*, std::Vector<hardware::Track*>>{begin_track, begin_tracks_vec};
        auto tail = std::Pair<hardware::Bump*, std::HashSet<hardware::Track*>>{end_bump, end_tracks_set};
        auto path_package = this->route_node_to_node_net<hardware::Track, hardware::Bump>(
            interposer, head, tail
        );

        net->set_pathpackage(path_package);
    }
    catch (RetryExpt& e){
        e.set_net(net);
        throw e;
    }
    }

    auto MazeRouteStrategy::route_bump_to_track_net(
        hardware::Interposer* interposer, circuit::BumpToTrackNet* net
    ) const -> void {
    try {
        debug::info(std::format("Maze routing for {}", net->name()));
        
        auto begin_bump = net->begin_bump();
        auto end_track = net->end_track();

        // existing path
        auto begin_tracks_vec = existing_path_vec<hardware::Bump>(begin_bump, net);
        auto end_tracks_set = existing_path_set<hardware::Track>(end_track, net);

        auto head = std::Pair<hardware::Bump*, std::Vector<hardware::Track*>>{begin_bump, begin_tracks_vec};
        auto tail = std::Pair<hardware::Track*, std::HashSet<hardware::Track*>>{end_track, end_tracks_set};
        auto path_package = this->route_node_to_node_net<hardware::Bump, hardware::Track>(
            interposer, head, tail
        );

        net->set_pathpackage(path_package);
    }
    catch (RetryExpt& e){
        e.set_net(net);
        throw e;
    }
    }

    template<class InputNode, class OutputNode>
    auto MazeRouteStrategy::route_node_to_node_net(
        hardware::Interposer* interposer, 
        std::Pair<InputNode*, std::Vector<hardware::Track*>> input_node, std::Pair<OutputNode*, std::HashSet<hardware::Track*>> output_node
    ) const -> circuit::PathPackage {
        std::usize path_l {0};
        std::Vector<hardware::Track*> begin_tracks_vec {};
        std::HashSet<hardware::Track*> end_tracks_set {};
        std::HashMap<PR_tool::hardware::Track *, PR_tool::hardware::TOBConnector> begin_tracks_map {};
        std::HashMap<PR_tool::hardware::Track *, PR_tool::hardware::TOBConnector> end_tracks_map {};

        // input node
        if constexpr (std::is_same<InputNode, hardware::Bump>::value) {
            begin_tracks_map = interposer->available_tracks_bump_to_track(input_node.first);
            begin_tracks_vec = track_map_to_track_vec(begin_tracks_map, input_node.first->tob()->cobunit_resources());
        }
        if constexpr (std::is_same<InputNode, hardware::Track>::value) {
            begin_tracks_vec.emplace_back(input_node.first);
        }
        begin_tracks_vec.insert(begin_tracks_vec.end(), input_node.second.begin(), input_node.second.end());

        // output node
        if constexpr (std::is_same<OutputNode, hardware::Bump>::value) {
            end_tracks_map = interposer->available_tracks_track_to_bump(output_node.first);
            end_tracks_set = track_map_to_track_set(end_tracks_map);
        }
        if constexpr (std::is_same<OutputNode, hardware::Track>::value) {
            end_tracks_set.emplace(output_node.first);
        }
        end_tracks_set.insert(output_node.second.begin(), output_node.second.end());

        if (begin_tracks_vec.empty()){
            throw RetryExpt("MazeRouteStrategy::route_node_to_node_net(): no available begin tracks");
        }
        if (end_tracks_set.empty()){
            throw RetryExpt("MazeRouteStrategy::route_node_to_node_net(): no available end tracks");
        }

        // route path
        circuit::PathPackage path_package {};
        path_package._regular_path = this->route_path(
            interposer, begin_tracks_vec, end_tracks_set, std::HashSet<hardware::Track*>{}
        );
        auto begin_track = std::get<0>(path_package._regular_path.front());
        auto end_track = std::get<0>(path_package._regular_path.back());

        // store bump to track
        if constexpr(std::is_same<InputNode, hardware::Bump>::value){
            auto begin_map = begin_tracks_map.find(begin_track);
            if (begin_map != begin_tracks_map.end()) {
                begin_map->second.give_out();
                path_package._tob_to_track.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>{input_node.first, begin_map->second, begin_track}
                );
                path_l += 1;
            }
        }
        if constexpr(std::is_same<OutputNode, hardware::Bump>::value){
            auto end_map = end_tracks_map.find(end_track);
            if (end_map != end_tracks_map.end()) {
                end_map->second.give_out();
                path_package._track_to_tob.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>{output_node.first, end_map->second, end_track}
                );
                path_l += 1;
            }
        }

        path_l += path_length(path_package._regular_path);
        path_package._length = path_l;

        return path_package;
    }

    auto MazeRouteStrategy::route_bump_to_bumps_net(
        hardware::Interposer* interposer, circuit::BumpToBumpsNet* net
    )  const -> void {
    try {
        debug::info(std::format("Maze routing for {}", net->name()));
        
        auto begin_bump = net->begin_bump();
        const auto& end_bumps = net->end_bumps();

        auto begin_tracks_vec = existing_path_vec<hardware::Bump>(begin_bump, net);

        std::usize total_length {0};
        circuit::PathPackage path_package {};
        routed_path total_regular_path {};
        for (auto end_bump : end_bumps) {
            auto current_begin_tracks_map = interposer->available_tracks_bump_to_track(begin_bump);
            auto current_begin_tracks = track_map_to_track_vec(current_begin_tracks_map, begin_bump->tob()->cobunit_resources());
            auto total_begin_tracks = begin_tracks_vec;
            total_begin_tracks.insert(total_begin_tracks.end(), current_begin_tracks.begin(), current_begin_tracks.end());

            auto end_tracks = interposer->available_tracks_track_to_bump(end_bump);
            auto end_tracks_set = track_map_to_track_set(end_tracks);
            auto existing_end_tracks = existing_path_set<hardware::Bump>(end_bump, net);
            end_tracks_set.insert(existing_end_tracks.begin(), existing_end_tracks.end());

            if (total_begin_tracks.empty()){
                throw RetryExpt("MazeRouteStrategy::route_bump_to_bumps_net(): no available begin tracks");
            }
            if (end_tracks_set.empty()){
                throw RetryExpt("MazeRouteStrategy::route_bump_to_bumps_net(): no available end tracks");
            }
            
            auto regular_path = this->route_path(
                interposer, total_begin_tracks, end_tracks_set, std::HashSet<hardware::Track*>{}
            );
            // remove repeated tracks
            // regular_path.erase(std::unique(regular_path.begin(), regular_path.end(), [](const auto& a, const auto& b) {
            //     return std::get<0>(a) == std::get<0>(b);
            // }), regular_path.end());
            total_regular_path.insert(total_regular_path.begin(), regular_path.begin(), regular_path.end());

            // Get begin and end track in path
            auto path = std::Vector<hardware::Track*> {};
            for (auto& [t, connector]: regular_path) {
                path.emplace_back(t);
            }
            auto begin_track = path.front();
            auto end_track = path.back();

            // Connect the TOB from end track to end_bump
            if (!end_tracks_set.contains(end_track)){
                throw FinalError("MazeRouteStrategy::route_bump_to_bumps_net(): end track not in end tracks set");
            }

            auto end_map = end_tracks.find(end_track);
            if (end_map != end_tracks.end()) {
                end_map->second.give_out();
                path_package._track_to_tob.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>{end_bump, end_map->second, end_track}
                );
            }
            
            // Is the begin from begin_tracks?
            auto find_res = current_begin_tracks_map.find(begin_track);
            if (find_res != current_begin_tracks_map.end()) {
                find_res->second.give_out();
                path_package._tob_to_track.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>{begin_bump, find_res->second, begin_track}
                );
                total_length += 1;  // head of path is calculate seperately 
            }

            // All track in path can see as `begin_track_set`
            begin_tracks_vec.insert(begin_tracks_vec.end(), path.begin(), path.end());

            total_length += path_length(path);    // path_length(path) + 1(end_bump) - 1(head of path)
        }

        path_package._regular_path = total_regular_path;
        path_package._length = total_length + 1;
        net->set_pathpackage(path_package);
    }
    catch (RetryExpt& e){
        e.set_net(net);
        throw e;
    }
    }

    auto MazeRouteStrategy::route_track_to_bumps_net(
        hardware::Interposer* interposer, circuit::TrackToBumpsNet* net
    ) const -> void {
    try {
        debug::info(std::format("Maze routing for {}", net->name()));
        
        auto begin_track = net->begin_track();
        const auto& end_bumps = net->end_bumps();

        auto begin_tracks_vec = existing_path_vec<hardware::Track>(begin_track, net);
        begin_tracks_vec.emplace_back(begin_track);

        std::usize total_length {0};
        circuit::PathPackage path_package {};
        routed_path total_regular_path {};
        for (auto end_bump : end_bumps) {
            auto end_tracks = interposer->available_tracks_track_to_bump(end_bump);
            auto end_tracks_set = track_map_to_track_set(end_tracks);
            auto existing_end_tracks = existing_path_set<hardware::Bump>(end_bump, net);
            end_tracks_set.insert(existing_end_tracks.begin(), existing_end_tracks.end());

            if (begin_tracks_vec.empty()){
                throw RetryExpt("MazeRouteStrategy::route_track_to_bumps_net(): no available begin tracks", net);
            }
            if (end_tracks_set.empty()){
                throw RetryExpt("MazeRouteStrategy::route_track_to_bumps_net(): no available end tracks", net);
            }

            auto regular_path = this->route_path(
                interposer, begin_tracks_vec, end_tracks_set, std::HashSet<hardware::Track*>{}
            );
            // remove repeated tracks
            // regular_path.erase(std::unique(regular_path.begin(), regular_path.end(), [](const auto& a, const auto& b) {
            //     return std::get<0>(a) == std::get<0>(b);
            // }), regular_path.end());
            total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());

            // Get begin and end track in path
            auto path = std::Vector<hardware::Track*> {};
            for (auto& [t, connector]: regular_path) {
                path.emplace_back(t);
            }
            auto begin_track = path.front();
            auto end_track = path.back();

            auto find_res = end_tracks.find(end_track);
            if (find_res != end_tracks.end()) {
                find_res->second.give_out();
                path_package._track_to_tob.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>{end_bump, find_res->second, end_track}
                );
            }

            // All track in path can see as `begin_track_set`
            begin_tracks_vec.insert(begin_tracks_vec.end(), path.begin(), path.end());

            total_length += path_length(path);  // +1(end_bump) - 1(head)
        }

        path_package._regular_path = total_regular_path;
        path_package._length = total_length + 1;
        net->set_pathpackage(path_package);
    }
    catch (RetryExpt& e){
        e.set_net(net);
        throw e;
    }
    }
    

    auto MazeRouteStrategy::route_bump_to_tracks_net(
        hardware::Interposer* interposer, circuit::BumpToTracksNet* net
    ) const -> void {
    try {
        debug::info(std::format("Maze routing for {}", net->name()));

        auto begin_bump = net->begin_bump();
        const auto& end_tracks = net->end_tracks();

        auto begin_tracks_vec = existing_path_vec<hardware::Bump>(begin_bump, net);

        std::usize total_length {0};
        circuit::PathPackage path_package {};
        routed_path total_regular_path {};
        for (auto end_track : end_tracks) {
            auto current_begin_tracks_map = interposer->available_tracks_bump_to_track(begin_bump);
            auto current_begin_tracks = track_map_to_track_vec(current_begin_tracks_map, begin_bump->tob()->cobunit_resources());
            auto total_begin_tracks = begin_tracks_vec;
            total_begin_tracks.insert(total_begin_tracks.end(), current_begin_tracks.begin(), current_begin_tracks.end());

            auto end_tracks_set = existing_path_set<hardware::Track>(end_track, net);
            end_tracks_set.emplace(end_track);

            if (begin_tracks_vec.empty()){
                throw RetryExpt("MazeRouteStrategy::route_bump_to_tracks_net(): no available begin tracks", net);
            }
            if (end_tracks_set.empty()){
                throw RetryExpt("MazeRouteStrategy::route_bump_to_tracks_net(): no available end tracks", net);
            }
    
            auto regular_path = this->route_path(
                interposer, total_begin_tracks, end_tracks_set, std::HashSet<hardware::Track*>{}
            );
            // remove repeated tracks
            // regular_path.erase(std::unique(regular_path.begin(), regular_path.end(), [](const auto& a, const auto& b) {
            //     return std::get<0>(a) == std::get<0>(b);
            // }), regular_path.end());
            total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());

            // Get begin and end track in path
            auto path = std::Vector<hardware::Track*> {};
            for (auto& [t, connector]: regular_path) {
                path.emplace_back(t);
            }
            auto begin_track = path.front();

            // Is the begin is from begin_tracks?
            auto find_res = current_begin_tracks_map.find(begin_track);
            if (find_res != current_begin_tracks_map.end()) {
                find_res->second.give_out();
                path_package._tob_to_track.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>(begin_bump, find_res->second, begin_track)
                );
                total_length += 1;
            }

            // All track in path can see as `begin_track_set`
            begin_tracks_vec.insert(begin_tracks_vec.end(), path.begin(), path.end());

            total_length += path_length(path) - 1;  // -1 for removing head of path 
        }

        path_package._regular_path = total_regular_path;
        path_package._length = total_length + 1;
        net->set_pathpackage(path_package);
    }
    catch (RetryExpt& e){
        e.set_net(net);
        throw e;
    }
    }

    auto MazeRouteStrategy::route_tracks_to_bumps_net(
        hardware::Interposer* interposer, circuit::TracksToBumpsNet* net
    ) const -> void {
    try {
        debug::info(std::format("Maze routing for {}", net->name()));
        
        auto& begin_tracks = net->begin_tracks();
        auto& end_bumps = net->end_bumps();
        
        auto begin_tracks_vec = std::Vector<hardware::Track*>{};
        for (auto t : begin_tracks) {
            begin_tracks_vec.emplace_back(t);
        }

        std::usize total_length {0};
        circuit::PathPackage path_package {};
        routed_path total_regular_path {};
        for (auto end_bump : end_bumps) {
            auto end_tracks = interposer->available_tracks_track_to_bump(end_bump);
            auto regular_path = this->route_path(
                interposer, begin_tracks_vec, track_map_to_track_set(end_tracks), std::HashSet<hardware::Track*>{}
            );
            // remove repeated tracks
            // regular_path.erase(std::unique(regular_path.begin(), regular_path.end(), [](const auto& a, const auto& b) {
            //     return std::get<0>(a) == std::get<0>(b);
            // }), regular_path.end());
            total_regular_path.insert(total_regular_path.end(), regular_path.begin(), regular_path.end());
            
            auto path = std::Vector<hardware::Track*> {};
            for (auto& [t, connector]: regular_path) {
                path.emplace_back(t);
            }
            auto end_track = path.back();
            if (!end_tracks.contains(end_track)){
                throw FinalError("MazeRouteStrategy::route_tracks_to_bumps_net(): end track not in end tracks set");
            }
            // end_bump->set_connected_track(end_track, hardware::TOBSignalDirection::TrackToBump);
            // end_tracks.find(end_track)->second.connect();
            auto& end_tob_connector = end_tracks.find(end_track)->second;
            end_tob_connector.give_out();
            path_package._track_to_tob.emplace_back(
                std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>(end_bump, end_tob_connector, end_track)
            );

            for (auto t : path) {
                begin_tracks_vec.emplace_back(t);
            }

            total_length += (path_length(path) + 1);
        }

        path_package._regular_path = total_regular_path;
        path_package._length = total_length;
        net->set_pathpackage(path_package);
    }
    catch (RetryExpt& e){
        e.set_net(net);
        throw e;
    }
    }

    auto MazeRouteStrategy::route_sync_net(
        hardware::Interposer* ptr_interposer, circuit::SyncNet* ptr_sync_net
    ) const -> void
    try{
        // three: [bump_to_bump, track_to_bump, bump_to_track]

        debug::info(std::format("Maze routing for synchronized {}", ptr_sync_net->name()));
        
        std::HashSet<hardware::Track*> occupied_tracks_vec {}; 
        std::usize max_length {0};

        // set all begin/end tracks as occupied tracks
        // for these cannot be used by other nets
        for (auto& net: ptr_sync_net->bttnets()){
            occupied_tracks_vec.emplace(net->end_track());
        }

        for (auto& net: ptr_sync_net->ttbnets()){
            occupied_tracks_vec.emplace(net->begin_track());
        }

        // the first round of routing
        if (ptr_sync_net->btbnets().size() > 0){
            auto current_len = sync_preroute<circuit::BumpToBumpNet>(
                ptr_interposer, ptr_sync_net->btbnets(), occupied_tracks_vec
            );
            max_length = current_len > max_length ? current_len : max_length;
        }
        if (ptr_sync_net->ttbnets().size() > 0){
            auto current_len = sync_preroute<circuit::TrackToBumpNet>(
                ptr_interposer, ptr_sync_net->ttbnets(), occupied_tracks_vec
            );
            max_length = current_len > max_length ? current_len : max_length;
        }
        if (ptr_sync_net->bttnets().size() > 0){
            auto current_len = sync_preroute<circuit::BumpToTrackNet>(
                ptr_interposer, ptr_sync_net->bttnets(), occupied_tracks_vec
            );
            max_length = current_len > max_length ? current_len : max_length;
        }

        // reroute for adjusting length
        auto btb_nets = std::Vector<circuit::Net*> {};
        auto ttb_nets = std::Vector<circuit::Net*> {};
        auto btt_nets = std::Vector<circuit::Net*> {};
        for (auto& net: ptr_sync_net->btbnets()) {
            btb_nets.emplace_back(net.get());
        }
        for (auto& net: ptr_sync_net->ttbnets()) {
            ttb_nets.emplace_back(net.get());
        }
        for (auto& net: ptr_sync_net->bttnets()) {
            btt_nets.emplace_back(net.get());
        }
        while (true){
            algo::RoutingBudget::check();
            auto [success, ml] = sync_reroute(
                ptr_interposer, btb_nets, max_length
            );
            if (success){
                auto [success, ml] = sync_reroute(
                    ptr_interposer, ttb_nets, max_length
                );
                if (success){
                    auto [success, ml] = sync_reroute(
                        ptr_interposer, btt_nets, max_length
                    );
                    max_length = ml;
                    if (success){
                        break;      // break only when all three nets are successfully routed
                    }
                    else{
                        continue;
                    }
                }
                else{
                    max_length = ml;
                    continue;
                }
            }
            else{
                max_length = ml;
                continue;
            }
        }
        std::usize total_nets {ptr_sync_net->btbnets().size() + ptr_sync_net->bttnets().size() + ptr_sync_net->ttbnets().size()};
        std::usize total_length = total_nets * max_length;

        // check
        std::usize sum {0};
        for (auto& net: ptr_sync_net->ttbnets()) {
            sum += net->pathpackage()._length;
        }
        for (auto& net: ptr_sync_net->btbnets()) {
            sum += net->pathpackage()._length;
        }
        for (auto& net: ptr_sync_net->bttnets()) {
            sum += net->pathpackage()._length;
        }
        assert(sum == total_length);

        ptr_sync_net->collect_package();
    }
    catch (RetryExpt& e){
        e.set_net(ptr_sync_net);
        throw e;
    }
    catch (const std::exception& e){
        throw std::runtime_error(std::format("MazeRouteStrategy::route_sync_net: {}", std::String(e.what())));
    }

    // Return : Vector<(Track*, COBConnector)>
    auto MazeRouteStrategy::maze_search(
        hardware::Interposer* interposer, 
        const std::Vector<hardware::Track*>& begin_tracks,
        const std::HashSet<hardware::Track*>& end_tracks,
        const std::HashSet<hardware::Track*>& occupied_tracks
    ) const -> algo::routed_path {
        using namespace hardware;

        auto queue = std::Queue<Track*>{};
        auto prev_track_infos = 
            std::HashMap<Track*, std::Option<std::Tuple<Track*, COBConnector>>>{};

        for (auto& t: begin_tracks) {
            queue.push(t);
            prev_track_infos.insert({t, std::nullopt});
        }
        
        while (!queue.empty()) {
            algo::RoutingBudget::check();
            auto track = queue.front();
            queue.pop();

            // if (end_tracks.find(track) != end_tracks.end()) {    
            if (check_found(end_tracks, track)) {
                auto path = std::Vector<std::Tuple<Track*, std::Option<COBConnector>>>{};
                auto cur_track = track;
                while (true) {
                    algo::RoutingBudget::check();
                    auto prev_track_info = prev_track_infos.find(cur_track);
                    if(prev_track_info == prev_track_infos.end()){
                        throw FinalError("MazeRouteStrategy::maze_search(): cannot find previous track");
                    }
                    // Reach start track
                    if (!prev_track_info->second.has_value()) {
                        break;
                    }

                    path.emplace_back(cur_track, std::get<1>(*prev_track_info->second));
                    cur_track = std::get<0>(*prev_track_info->second);
                }
                path.emplace_back(cur_track, std::nullopt);

                return {path};
            }

            for (auto& [next_track, connector] : interposer->adjacent_idle_tracks(track)) {
                if (prev_track_infos.find(next_track) != prev_track_infos.end() || occupied_tracks.contains(next_track)) {
                    continue;
                }

                queue.push(next_track);             
                prev_track_infos.insert({
                    next_track, 
                    std::Tuple<Track*, COBConnector>{track, connector}
                });
            }
        }

        throw RetryExpt("MazeRouteStrategy::maze_search(): path not found");
    }

    // return routed path with positive sequence, and suspend coresponding cobconnector
    auto MazeRouteStrategy::route_path(
        hardware::Interposer* interposer, 
        const std::Vector<hardware::Track*>& begin_tracks,
        const std::HashSet<hardware::Track*>& end_tracks,
        const std::HashSet<hardware::Track*>& occupied_tracks
    ) const -> algo::routed_path
    try {
        auto path_info = maze_search(interposer, begin_tracks, end_tracks, occupied_tracks);  // negative sequence

        auto path = algo::routed_path {};   // positive sequence
        std::transform(path_info.rbegin(), path_info.rend(), std::back_inserter(path), [](const auto& p){
            return p;
        });

        for (auto& [track, cobconnector]: path) {
            if (cobconnector.has_value()) {
                cobconnector.value().suspend();
            }
        }

        return path;
    }
    catch (const RetryExpt& e){
        throw e;
    }
    catch (std::exception& e){
        throw std::runtime_error("MazeRouteStrategy::route_path() >> " + std::String(e.what()));
    }

    auto MazeRouteStrategy::track_map_to_track_vec(
        const std::HashMap<hardware::Track*, 
        hardware::TOBConnector>& map,
        const std::Array<std::usize, hardware::COB::UNIT_SIZE>& cobunit_usage
    ) const -> std::Vector<hardware::Track*> {
        auto vec = std::Vector<hardware::Track*>{};
        for (auto [t, _] : map) {
            vec.emplace_back(t);
        }
        std::sort(vec.begin(), vec.end(), [&cobunit_usage](hardware::Track* track1, hardware::Track* track2){
            auto cobunit = [](hardware::Track* track){
                auto index = track->coord().index;
                auto bank = index / (hardware::TOB::INDEX_SIZE / 2);
                return (bank*hardware::COBUnit::WILTON_SIZE + index%hardware::COBUnit::WILTON_SIZE);
            };
            auto cobunit1 = cobunit(track1);
            auto cobunit2 = cobunit(track2);
            return cobunit_usage[cobunit1] < cobunit_usage[cobunit2];
        });

        return vec;
    }

    auto MazeRouteStrategy::track_map_to_track_set(
        const std::HashMap<hardware::Track*, 
        hardware::TOBConnector>& map
    ) const -> std::HashSet<hardware::Track*> {
        auto set = std::HashSet<hardware::Track*>{};
        for (auto [t, _] : map) {
            set.emplace(t);
        }
        return set;
    }

    template <class Net>
    auto MazeRouteStrategy::sync_preroute(
            hardware::Interposer* interposer,
            std::Vector<std::Rc<Net>>& sync_net,
            std::HashSet<hardware::Track*>& occupied_tracks_vec 
        ) const -> std::usize{
        static_assert(
            std::is_same<Net, circuit::BumpToBumpNet>::value ||\
            std::is_same<Net, circuit::TrackToBumpNet>::value ||\
            std::is_same<Net, circuit::BumpToTrackNet>::value,
            "MazeRouteStrategy::sync_preroute() >> Invalid Net type"
        );
        if (sync_net.size() <= 0){
            throw FinalError("MazeRouteStrategy::sync_preroute(): empty sync_net");
        }

        std::Vector<hardware::Track*> begin_tracks_vec {};
        std::HashSet<hardware::Track*> end_tracks_set {};
        hardware::Bump* begin_bump = nullptr;
        hardware::Bump* end_bump = nullptr;
        std::HashMap<hardware::Track*, hardware::TOBConnector> begin_track_to_tob_map {};
        std::HashMap<hardware::Track*, hardware::TOBConnector> end_track_to_tob_map {};

        for (auto& uptr_net: sync_net){
            auto net = uptr_net.get();
            circuit::PathPackage package {};
            
            // collect begin bumps & begin tracks
            if constexpr (std::is_same<Net, circuit::BumpToBumpNet>::value || std::is_same<Net, circuit::BumpToTrackNet>::value){
                begin_bump = net->begin_bump();
                begin_track_to_tob_map = interposer->available_tracks_bump_to_track(begin_bump);
                begin_tracks_vec = track_map_to_track_vec(begin_track_to_tob_map, begin_bump->tob()->cobunit_resources());
            }
            else if constexpr(std::is_same<Net, circuit::TrackToBumpNet>::value){
                begin_tracks_vec = std::Vector<hardware::Track*>{net->begin_track()};
            }
            // collect end bumps & end tracks
            if constexpr (std::is_same<Net, circuit::BumpToBumpNet>::value || std::is_same<Net, circuit::TrackToBumpNet>::value){
                end_bump = net->end_bump();
                end_track_to_tob_map = interposer->available_tracks_track_to_bump(end_bump);
                end_tracks_set = track_map_to_track_set(end_track_to_tob_map);
            }
            else if constexpr (std::is_same<Net, circuit::BumpToTrackNet>::value){
                end_tracks_set = std::HashSet<hardware::Track*>{net->end_track()};
            }
            
            if (std::is_same<Net, circuit::BumpToBumpNet>::value){
                if (begin_bump->tob()->coord() == end_bump->tob()->coord()){
                    throw FinalError("MazeRouteStrategy::sync_preroute(): begin_bump_tob == end_bump_tob");
                }
            }

            // set end track of Net as unoccupied
            if constexpr(std::is_same<Net, circuit::BumpToTrackNet>::value){
                auto track = net->end_track();
                auto track_coord = track->coord();
                std::erase_if(occupied_tracks_vec, [track_coord](const auto& track) {
                    return track->coord() == track_coord;
                });
            }
            if constexpr(std::is_same<Net, circuit::TrackToBumpNet>::value){
                auto track = net->begin_track();
                auto track_coord = track->coord();
                std::erase_if(occupied_tracks_vec, [track_coord](const auto& track) {
                    return track->coord() == track_coord;
                });
            }

            // route and suspend connector
            package._regular_path = route_path(interposer, begin_tracks_vec, end_tracks_set, occupied_tracks_vec);
                                                                                        
            auto path = std::Vector<hardware::Track*>{};                                                                 
            for (auto& [t, connector]: package._regular_path) {    // get tracks in positive sequence        
                path.emplace_back(t);
            }

            // connect begin bump / end bump
            if (std::is_same<Net, circuit::BumpToBumpNet>::value || std::is_same<Net, circuit::BumpToTrackNet>::value){
                auto begin_track = path.front();
                if (!begin_track_to_tob_map.contains(begin_track)){
                    throw FinalError("MazeRouteStrategy::sync_preroute(): begin track not in begin track map");
                }
                auto& begin_tob_connector = begin_track_to_tob_map.find(begin_track)->second;
                begin_tob_connector.give_out();
                package._tob_to_track.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>(begin_bump, begin_tob_connector, begin_track)
                );
            }
            if (std::is_same<Net, circuit::BumpToBumpNet>::value || std::is_same<Net, circuit::TrackToBumpNet>::value){
                auto end_track = path.back();
                if (!end_track_to_tob_map.contains(end_track)){
                    throw FinalError("MazeRouteStrategy::sync_preroute(): end track not in end track map");
                }
                auto& end_tob_connector = end_track_to_tob_map.find(end_track)->second;
                end_tob_connector.give_out();
                package._track_to_tob.emplace_back(
                    std::Tuple<hardware::Bump*, hardware::TOBConnector, hardware::Track*>(end_bump, end_tob_connector, end_track)
                );
            }

            //calculate length
            if constexpr(std::is_same<Net, circuit::BumpToBumpNet>::value){
                package._length = path_length(package._regular_path) + 2;
            }
            else if constexpr(std::is_same<Net, circuit::TrackToBumpNet>::value || std::is_same<Net, circuit::BumpToTrackNet>::value){
                package._length = path_length(package._regular_path) + 1;
            }
            
            uptr_net->set_pathpackage(package);
        }
        
        // calculate length
        std::usize max_length = 0;
        for (auto& uptr_net: sync_net) {
            auto& pathpackage = uptr_net->pathpackage();
            max_length = max_length < pathpackage._length ? pathpackage._length : max_length;
        }
        
        return max_length;
    }

    // reroute for btb/btt/ttb net
    auto MazeRouteStrategy::sync_reroute(
        hardware::Interposer* interposer, std::Vector<circuit::Net*>& nets, std::usize max_length
    ) const -> std::tuple<bool, std::usize>{
        std::Vector<circuit::Net*> nets_to_be_rerouted {};
        std::String reroute_nets{};

        // collect nets to be rerouted, along with their end bumps and track to tob maps
        for (auto& net: nets) {
            if (net->pathpackage()._length < max_length) {
                nets_to_be_rerouted.push_back(net);
                reroute_nets += net->name() + " \n";
            }
        }

        if (nets_to_be_rerouted.size() > 0) {
            debug::info(std::format("sync_reroute(): reroute nets {}", reroute_nets));
        }

        // reroute
        if (nets_to_be_rerouted.size() > 0) {
            auto [success, ml] = _rerouter->bus_reroute(interposer, nets_to_be_rerouted, max_length);
            
            if (success){   // routing done with ml == max_length
                if (max_length != ml){
                    throw FinalError("MazeRouteStrategy::sync_reroute(): max_length != ml when succeed");
                }
                return std::tuple<std::usize, std::usize>{true, max_length};
            }
            else{           // have longer path OR routing failed
                return std::tuple<std::usize, std::usize>{false, ml};
            }
        }
        else{
            return std::tuple<std::usize, std::usize>{true, max_length};
        }
    }

    template <class Node>
    auto existing_path_vec(Node* node, circuit::Net* net) -> std::Vector<hardware::Track*> {
        static_assert(
            std::is_same<Node, hardware::Bump>::value || std::is_same<Node, hardware::Track>::value,
            "MazeRouteStrategy::existing_path() >> Invalid Node type"
        );

        auto related_nets = net->related_nets<Node>(node);
        auto tracks_vec = std::Vector<hardware::Track*> {};
        for (auto n : related_nets) {
            auto path = n->pathpackage()._regular_path;
            for (auto& [t, _] : path) {
                tracks_vec.emplace_back(t);
            }
        }
        return tracks_vec;
    }

    template <class Node>
    auto existing_path_set(Node* node, circuit::Net* net) -> std::HashSet<hardware::Track*> {
        auto vec = existing_path_vec<Node>(node, net);
        auto set = std::HashSet<hardware::Track*>{};
        for (auto t : vec) {
            set.emplace(t);
        }
        return set;
    }
}
