#include "./syncnet.hh"

#include <hardware/coord.hh>
#include <assert.h>
#include <algorithm>
#include <algo/router/incremental/maze/routing.hh>



namespace PR_tool::circuit
{

    SyncNet::SyncNet(
        std::Vector<std::Rc<BumpToBumpNet>> btbnets,
        std::Vector<std::Rc<BumpToTrackNet>> bttnets,
        std::Vector<std::Rc<TrackToBumpNet>> ttbnets,
        const std::HashSet<int>& modes,
        std::String& name,
        std::String& uid
    ) :
        _btbnets{btbnets},
        _bttnets{bttnets},
        _ttbnets{ttbnets},
        Net{Priority{0}, modes, name, uid}
    {
    }

    auto SyncNet::update_tob_postion(hardware::TOB* prev_tob, hardware::TOB* next_tob) -> void {
        for (auto& net : this->_btbnets) {
            net->update_tob_postion(prev_tob, next_tob);
        }

        for (auto& net : this->_bttnets) {
            net->update_tob_postion(prev_tob, next_tob);
        }

        for (auto& net : this->_ttbnets) {
            net->update_tob_postion(prev_tob, next_tob);
        }
    }

    auto SyncNet::swap_tob_position(hardware::TOB* tob1, hardware::TOB* tob2) -> void {
        for (auto& net : this->_btbnets) {
            net->swap_tob_position(tob1, tob2);
        }
        for (auto& net : this->_bttnets) {
            net->swap_tob_position(tob1, tob2);
        }
        for (auto& net : this->_ttbnets) {
            net->swap_tob_position(tob1, tob2);
        }
    }

    auto SyncNet::route(hardware::Interposer* interposer, const algo::RouteStrategy& strategy) -> void {
        strategy.route_sync_net(interposer, this);
    }

    auto SyncNet::incremental_route(
        hardware::Interposer* interposer, const algo::IncreRouting& strategy, algo::RouteEngine& engine, bool shared
    ) -> bool {
        return strategy.route_sync_net(interposer, this, engine, shared);
    }

    auto SyncNet::update_priority(float bias) -> void {
        assert(0 <= bias && bias < 1);
        for (auto& net : this->_btbnets) {
            net->update_priority(bias);
        }

        for (auto& net : this->_bttnets) {
            net->update_priority(bias);
        }

        for (auto& net : this->_ttbnets) {
            net->update_priority(bias);
        }
    }

    auto SyncNet::coords() const -> std::Vector<hardware::Coord> {
        auto coords = std::Vector<hardware::Coord>{};
        for (auto& net : this->_btbnets) {
            for (auto& coord : net->coords()) {
                coords.emplace_back(coord);
            }
        }

        for (auto& net : this->_bttnets) {
            for (auto& coord : net->coords()) {
                coords.emplace_back(coord);
            }
        }

        for (auto& net : this->_ttbnets) {
            for (auto& coord : net->coords()) {
                coords.emplace_back(coord);
            }
        }

        return coords;
    }

    auto SyncNet::check_accessable_cobunit() -> void {
        for (auto& net: _btbnets) {
            net->check_accessable_cobunit();
        }
        for (auto& net: _bttnets) {
            net->check_accessable_cobunit();
        }
        for (auto& net: _ttbnets) {
            net->check_accessable_cobunit();
        }
    }

    auto SyncNet::accessable_cobunit() -> std::HashMap<hardware::Bump*, std::HashSet<std::usize>> {
        std::HashMap<hardware::Bump*, std::HashSet<std::usize>> map{};

        for (auto& net: _btbnets) {
            const auto& net_map = net->accessable_cobunit();
            map.insert(net_map.begin(), net_map.end());
        }
        for (auto& net: _bttnets) {
            const auto& net_map = net->accessable_cobunit();
            map.insert(net_map.begin(), net_map.end());
        }
        for (auto& net: _ttbnets) {
            const auto& net_map = net->accessable_cobunit();
            map.insert(net_map.begin(), net_map.end());
        }

        return map;
    }


    auto SyncNet::to_string() const -> std::String {
        auto ss = std::StringStream {};
        ss << std::format("{}: Syncnet net:\n", this->_name);
        for (auto& net: _btbnets) {
            ss << "    " << net->to_string() << '\n';
        }
        for (auto& net: _bttnets) {
            ss << "    " << net->to_string() << '\n';
        }
        for (auto& net: _ttbnets) {
            ss << "    " << net->to_string() << '\n';
        }
        ss << "End syncnet net";
        return ss.str();
    }

    auto SyncNet::port_number() const -> std::usize {
        std::usize port_number = 0;
        for (auto& net: _btbnets) {
            port_number += net->port_number();
        }
        for (auto& net: _bttnets) {
            port_number += net->port_number();
        }
        for (auto& net: _ttbnets) {
            port_number += net->port_number();
        }
        return port_number;
    }

    auto SyncNet::set_pathpackage(const circuit::PathPackage& package) -> void {
        if (!package._tob_to_track.empty() && package._track_to_tob.empty()) {
            assert(package._tob_to_track.size() == 1);
            auto bump = std::get<0>(package._tob_to_track[0]);
            for (auto& net: _bttnets) {
                if (net->begin_bump()->coord() == bump->coord())
                {
                    net->set_pathpackage(package);
                }
            }
        }
        else if (package._tob_to_track.empty() && !package._track_to_tob.empty()) {
            assert(package._track_to_tob.size() == 1);
            auto bump = std::get<0>(package._track_to_tob[0]);
            for (auto& net: _ttbnets) {
                if (net->end_bump()->coord() == bump->coord())
                {
                    net->set_pathpackage(package);
                }
            }
        }
        else if (!package._tob_to_track.empty() && !package._track_to_tob.empty()) {
            assert(package._track_to_tob.size() == 1 && package._tob_to_track.size() == 1);
            auto bump_begin = std::get<0>(package._tob_to_track[0]);
            auto bump_end = std::get<0>(package._track_to_tob[0]);
            for (auto& net: _btbnets) {
                if (net->begin_bump()->coord() == bump_begin->coord() && net->end_bump()->coord() == bump_end->coord()){
                    net->set_pathpackage(package);
                }
            }
        }
    }

    auto SyncNet::show_path() const -> void {
        for (auto& net: this->_btbnets) {
            net->show_path();
        }
        for (auto& net: this->_bttnets) {
            net->show_path();
        }
        for (auto& net: this->_ttbnets) {
            net->show_path();
        }
    }

    auto SyncNet::length() const -> std::usize {
        auto total_length = std::usize{0};
        for (auto& net: this->_btbnets) {
            total_length += net->length();
        }
        for (auto& net: this->_bttnets) {
            total_length += net->length();
        }
        for (auto& net: this->_ttbnets) {
            total_length += net->length();
        }
        return total_length;
    }

    auto SyncNet::check_relativity(const hardware::Bump* node) const -> const Net* {
        for (auto& net: this->_btbnets) {
            if (net->check_relativity(node) != nullptr) {
                return net.get();
            }
        }
        for (auto& net: this->_bttnets) {
            if (net->check_relativity(node) != nullptr) {
                return net.get();
            }
        }
        for (auto& net: this->_ttbnets) {
            if (net->check_relativity(node) != nullptr) {
                return net.get();
            }
        }
        return nullptr;
    }

    auto SyncNet::check_relativity(const hardware::Track* node) const -> const Net* {
        for (auto& net: this->_bttnets) {
            if (net->check_relativity(node) != nullptr) {
                return net.get();
            }
        }
        for (auto& net: this->_ttbnets) {
            if (net->check_relativity(node) != nullptr) {
                return net.get();
            }
        }
        return nullptr;
    }

    auto SyncNet::search_related_nets(std::Vector<Net*>& nets) -> void {
        clear_related_nets();
        //* Unsupported operation for syncnet currently
    }

    auto SyncNet::connection_state() const -> std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
        std::Vector<const hardware::Bump*> routable_bumps{}, unroutable_bumps{};
        std::Vector<const hardware::Track*> unroutable_tracks{};

        auto collect_state = [&](circuit::Net* net) {
            auto [rb, urb, urt] = net->connection_state();
            routable_bumps.insert(routable_bumps.end(), rb.begin(), rb.end());
            unroutable_bumps.insert(unroutable_bumps.end(), urb.begin(), urb.end());
            unroutable_tracks.insert(unroutable_tracks.end(), urt.begin(), urt.end());
        };

        for (auto& net: this->_btbnets) {
            collect_state(net.get());
        }
        for (auto& net: this->_bttnets) {
            collect_state(net.get());
        }
        for (auto& net: this->_ttbnets) {
            collect_state(net.get());
        }

        return std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
            routable_bumps, unroutable_bumps, unroutable_tracks
        };
    }

    auto SyncNet::collect_package() -> bool {
        bool flag = true;
        if (
            this->_path_package._regular_path.empty()\
            || this->_path_package._tob_to_track.empty()\
            || this->_path_package._track_to_tob.empty()
        ) {
            flag = false;
        }

        this->_history_path_package = circuit::HistoryPathPackage(this->_path_package);
        auto collect = [&](circuit::Net* net, PathPackage& new_package) {
            auto& package = net->pathpackage();

            new_package._regular_path.insert(new_package._regular_path.end(), package._regular_path.begin(), package._regular_path.end());
            new_package._tob_to_track.insert(new_package._tob_to_track.end(), package._tob_to_track.begin(), package._tob_to_track.end());
            new_package._track_to_tob.insert(new_package._track_to_tob.end(), package._track_to_tob.begin(), package._track_to_tob.end());
            new_package._length += package._length;
        };

        PathPackage total_package{};
        for (auto& net: this->_btbnets) {
            collect(net.get(), total_package);
        }
        for (auto& net: this->_bttnets) {
            collect(net.get(), total_package);
        }
        for (auto& net: this->_ttbnets) {
            collect(net.get(), total_package);
        }
        this->_path_package = total_package;

        return flag;
    }

    auto SyncNet::nodes_map() -> std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> {
        std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> map{};
        for(auto& net: this->_btbnets) {
            for (auto& m: net->nodes_map()) {
                map.emplace(m);
            }
        }
        return map;
    }

    auto SyncNet::nodes_direction() -> std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> {
        std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> map{};

        for (auto& net: this->_btbnets) {
            auto net_map = net->nodes_direction();
            map.insert(net_map.begin(), net_map.end());
        }
        for (auto& net: this->_bttnets) {
            auto net_map = net->nodes_direction();
            map.insert(net_map.begin(), net_map.end());
        }
        for (auto& net: this->_ttbnets) {
            auto net_map = net->nodes_direction();
            map.insert(net_map.begin(), net_map.end());
        }

        return map;
    }

    auto SyncNet::operator == (const Net& net) const -> bool {
    try {
        auto sync_net = dynamic_cast<const SyncNet&>(net);
        bool flag = true;

        for (auto& m: sync_net.btbnets()) {
            bool flag_n = false;
            for (auto& n: this->_btbnets) {
                if (*n == *m) {
                    flag_n = true;
                    break;
                }
            }

            if (!flag_n) {
                flag = false;
                break;
            }
        }
        for (auto& m: sync_net.bttnets()) {
            bool flag_n = false;
            for (auto& n: this->_bttnets) {
                if (*n == *m) {
                    flag_n = true;
                    break;
                }
            }

            if (!flag_n) {
                flag = false;
                break;
            }
        }
        for (auto& m: sync_net.ttbnets()) {
            bool flag_n = false;
            for (auto& n: this->_ttbnets) {
                if (*n == *m) {
                    flag_n = true;
                    break;
                }
            }

            if (!flag_n) {
                flag = false;
                break;
            }
        }

        return flag;
    }
    catch (const std::bad_cast& e) {
        return false;
    }
    }

    auto SyncNet::track_ports() const -> std::Pair<std::HashSet<hardware::Track*>, bool> {
        std::HashSet<hardware::Track*> tracks {};

        for (auto& net: this->_bttnets) {
            auto [ports, flag] = net->track_ports();
            tracks.insert(ports.begin(), ports.end());
        }
        for (auto& net: this->_ttbnets) {
            auto [ports, flag] = net->track_ports();
            tracks.insert(ports.begin(), ports.end());
        }
        for (auto& net: this->_btbnets) {
            auto [ports, flag] = net->track_ports();
            tracks.insert(ports.begin(), ports.end());
        }

        if (tracks.size() < this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, false};
        }
        else if (tracks.size() == this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, true};
        }
        else {
            throw std::logic_error("SyncNet::track_ports(): collected tracks.size() > port_number()");
        }
    }

    auto SyncNet::set_reuse_type(bool reuse_type) -> void {
        this->_reuse_type.emplace(reuse_type);
        for (auto& net: this->_btbnets) {
            net->set_reuse_type(reuse_type);
        }
        for (auto& net: this->_bttnets) {
            net->set_reuse_type(reuse_type);
        }
        for (auto& net: this->_ttbnets) {
            net->set_reuse_type(reuse_type);
        }
    }

    auto SyncNet::clear_path() -> void {
        this->_path_package.clear_all();
        this->_history_path_package.reset();
        this->_related_nets_bump.clear();
        this->_related_nets_track.clear();

        for (auto& net: this->_btbnets) {
            net->clear_path();
        }
        for (auto& net: this->_bttnets) {
            net->clear_path();
        }
        for (auto& net: this->_ttbnets) {
            net->clear_path();
        }
    }

    // return: [num_of_nets, length]
    auto SyncNet::sync_length() const -> std::Tuple<std::usize, std::usize> {
        return std::Tuple<std::usize, std::usize>{
            this->_btbnets.size() + this->_bttnets.size() + this->_ttbnets.size(), this->_path_package._length
        };
    }

    auto SyncNet::name() const -> const std::String& {
        return this->_name;
    }

    auto SyncNet::reset_pathpackage() -> void {
        this->_path_package.reset_all();

        for (auto& net: this->_btbnets) {
            net->reset_pathpackage();
        }
        for (auto& net: this->_bttnets) {
            net->reset_pathpackage();
        }
        for (auto& net: this->_ttbnets) {
            net->reset_pathpackage();
        }
    }

    auto SyncNet::move_history_to_current(hardware::Interposer* interposer) -> void {
        this->_path_package = circuit::PathPackage(this->_history_path_package.value(), interposer);
        
        for (auto& net: this->_btbnets) {
            net->move_history_to_current(interposer);
        }
        for (auto& net: this->_bttnets) {
            net->move_history_to_current(interposer);
        }
        for (auto& net: this->_ttbnets) {
            net->move_history_to_current(interposer);
        }
    }

    auto SyncNet::path_in_order() const -> std::Vector<PathInOrder> {
    try{
        std::Vector<PathInOrder> paths {};
        for (auto& net: this->_btbnets) {
            auto path_in_order = net->path_in_order();
            paths.insert(paths.end(), path_in_order.begin(), path_in_order.end());
        }
        for (auto& net: this->_bttnets) {
            auto path_in_order = net->path_in_order();
            paths.insert(paths.end(), path_in_order.begin(), path_in_order.end());
        }
        for (auto& net: this->_ttbnets) {
            auto path_in_order = net->path_in_order();
            paths.insert(paths.end(), path_in_order.begin(), path_in_order.end());
        }
        debug::info_fmt("SyncNet paths size() = {}", paths.size());

        return paths;
    }
    catch(const std::exception& e) {
        throw std::runtime_error(std::format("SyncNet::path_in_order(): {}", e.what()));
    }
    }

    auto SyncNet::set_history_pathpackage() -> void {
        this->_history_path_package.emplace(HistoryPathPackage(this->_path_package));
        debug::debug_fmt(
            "set history path package with length {} from current path package with length {}", 
            this->_history_path_package.value()._length, this->_path_package._length
        );

        for (auto& net: this->_btbnets) {
            net->set_history_pathpackage();
        }
        for (auto& net: this->_bttnets) {
            net->set_history_pathpackage();
        }
        for (auto& net: this->_ttbnets) {
            net->set_history_pathpackage();
        }
    }

    auto SyncNet::clear_current_package() -> void {
        this->_path_package.clear_all();

        for (auto& net: this->_btbnets) {
            net->clear_current_package();
        }
        for (auto& net: this->_bttnets) {
            net->clear_current_package();
        }
        for (auto& net: this->_ttbnets) {
            net->clear_current_package();
        }
    }

    auto SyncNet::has_tob_in_ports(hardware::TOB* tob) const -> bool {
        for (auto& net: _btbnets) {
            if (net->has_tob_in_ports(tob)) {
                return true;
            }
        }
        for (auto& net: _bttnets) {
            if (net->has_tob_in_ports(tob)) {
                return true;
            }
        }
        for (auto& net: _ttbnets) {
            if (net->has_tob_in_ports(tob)) {
                return true;
            }
        }
        return false;
    }

}


