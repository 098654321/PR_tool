#include "./tbsnet.hh"
#include <hardware/bump/bump.hh>
#include <hardware/track/track.hh>
#include <algorithm>
#include <algo/router/incremental/maze/routing.hh>


namespace PR_tool::circuit {

    TrackToBumpsNet::TrackToBumpsNet(hardware::Track* begin_track, std::Vector<hardware::Bump*> end_bumps, const std::HashSet<int>& modes, std::String& name, std::String& uid) :
        _begin_track{begin_track},
        _end_bumps{std::move(end_bumps)},
        Net{Priority{1}, modes, name, uid}
    {
    }

    TrackToBumpsNet::~TrackToBumpsNet() noexcept {

    }

    auto TrackToBumpsNet::update_tob_postion(hardware::TOB* prev_tob, hardware::TOB* next_tob) -> void {
        for (auto& bump : this->_end_bumps) {
            bump = hardware::Bump::update_bump(bump, prev_tob, next_tob);
        }
    }

    auto TrackToBumpsNet::swap_tob_position(hardware::TOB* tob1, hardware::TOB* tob2) -> void {
        for (auto& bump : this->_end_bumps) {
            bump = hardware::Bump::swap_bump(bump, tob1, tob2);
        }
    }

    auto TrackToBumpsNet::route(hardware::Interposer* interposer, const algo::RouteStrategy& strategy) -> void {
        strategy.route_track_to_bumps_net(interposer, this);
    }

    auto TrackToBumpsNet::incremental_route(
        hardware::Interposer* interposer, const algo::IncreRouting& strategy, algo::RouteEngine& engine, bool shared
    ) -> bool {
        return strategy.route_track_to_bumps_net(interposer, this, engine, shared);
    }

    auto TrackToBumpsNet::update_priority(float bias) -> void {
        assert(0 <= bias && bias < 1);
        this->_priority = Priority{this->_priority.value() + bias};
    }

    auto TrackToBumpsNet::coords() const -> std::Vector<hardware::Coord> {
        auto coords = std::Vector<hardware::Coord>{};
        for (auto bump : this->end_bumps()) {
            coords.emplace_back(bump->coord());
        }
        coords.emplace_back(this->begin_track()->coord());
        return coords;
    }

    auto TrackToBumpsNet::check_accessable_cobunit() -> void {
        std::usize index = _begin_track->coord().index;
        std::usize bank_size = hardware::COB::INDEX_SIZE/2;
        std::usize wilton_size = hardware::COBUnit::WILTON_SIZE;
        std::usize id {(index/bank_size)*wilton_size + (index%wilton_size)};
        
        std::HashSet<std::usize> cobunit_ids {id};
        for (auto bump: _end_bumps){
            bump->intersect_access_unit(cobunit_ids);
        }
    }

    auto TrackToBumpsNet::accessable_cobunit() -> std::HashMap<hardware::Bump*, std::HashSet<std::usize>> {
        std::HashMap<hardware::Bump*, std::HashSet<std::usize>> map{};
        for(auto& b: this->_end_bumps) {
            map.emplace(b, b->accessable_cobunit());
        }
        return map;
    }

    auto TrackToBumpsNet::to_string() const -> std::String {
        auto ss = std::StringStream {};
        ss << std::format("{}: Begin track '{}' to End bumps '[", this->_name, this->_begin_track->coord());
        for (int i = 0; i < this->_end_bumps.size(); ++i) {
            if (i != 0) {
                ss << ", ";
            }
            ss << std::format("{}", this->_end_bumps[i]->coord());
        }
        ss << ']';
        return ss.str();
    }

    auto TrackToBumpsNet::port_number() const -> std::usize {
        return (this->_end_bumps.size() + 1);
    }

    auto TrackToBumpsNet::check_relativity(const hardware::Track* node) const -> const Net* {
        if (this->_begin_track->coord() == node->coord()) {
            return this;
        }
        return nullptr;
    }

    auto TrackToBumpsNet::check_relativity(const hardware::Bump* node) const -> const Net* {
        for (auto bump : this->_end_bumps) {
            if (bump->coord() == node->coord()) {
                return this;
            }
        }
        return nullptr;
    }

    auto TrackToBumpsNet::search_related_nets(std::Vector<Net*>& nets) -> void {
        clear_related_nets();

        auto iter = std::find(nets.begin(), nets.end(), this);
        if (iter != nets.end()) {
            nets.erase(iter);
        }
        this->_related_nets_track.emplace(this->_begin_track, search_nets_node<hardware::Track>(this->_begin_track, nets));
        for (auto bump : this->_end_bumps) {
            this->_related_nets_bump.emplace(bump, search_nets_node<hardware::Bump>(bump, nets));
        }
    }

    auto TrackToBumpsNet::connection_state() const -> std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
        std::vector<const hardware::Bump*> routable_bumps{}, unroutable_bumps{};
        std::Vector<const hardware::Track*> unroutable_tracks{};

        const auto& package = this->pathpackage();
        if (!package.find_track(this->_begin_track).has_value()) {
            unroutable_tracks.emplace_back(this->_begin_track);
        }

        auto classify_bump = [&](hardware::Bump* bump) {
            (package.find_bump(bump).has_value() ? routable_bumps : unroutable_bumps).emplace_back(bump);
        };
        std::for_each(this->_end_bumps.begin(), this->_end_bumps.end(), classify_bump);

        return std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
            routable_bumps, unroutable_bumps, unroutable_tracks
        };
    }

    auto TrackToBumpsNet::nodes_map() -> std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> {
        return std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> {};
    }

    auto TrackToBumpsNet::nodes_direction() -> std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> {
        std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> map{};

        for (auto& b: this->_end_bumps) {
            map.emplace(b, hardware::TOBBumpDirection::TOBToBump);
        }
        return map;
    }

    auto TrackToBumpsNet::operator == (const Net& net) const -> bool {
    try {
        auto cast_net = dynamic_cast<const TrackToBumpsNet&>(net);
        if (this->_begin_track->coord() == cast_net._begin_track->coord()) {
            std::HashSet<hardware::BumpCoord> coords {};
            for (auto& b: this->_end_bumps) {
                coords.emplace(b->coord());
            }

            for (auto& b: cast_net._end_bumps) {
                if (!coords.contains(b->coord())) {
                    return false;
                }
            }
            return true;
        }
        return false;
    }
    catch (const std::bad_cast& e) {
        return false;
    }
    }

    auto TrackToBumpsNet::track_ports() const -> std::Pair<std::HashSet<hardware::Track*>, bool> {
        std::HashSet<hardware::Track*> tracks {this->_begin_track};

        auto collect = [&](const auto& package_v) {
            std::for_each(package_v.begin(), package_v.end(), [&](const auto& package) {
                const auto& [_1, _2, t] = package;
                tracks.emplace(t);
            });
        };
        collect(this->_path_package._tob_to_track);     // should be empty
        collect(this->_path_package._track_to_tob);     

        if (tracks.size() < this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, false};
        }
        else if (tracks.size() == this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, true};
        }
        else {
            throw std::logic_error("TrackToBumpsNet::track_ports(): collected tracks.size() > port_number()");
        }
    }

    auto TrackToBumpsNet::name() const -> const std::String& {
        return this->_name;
    }

    auto TrackToBumpsNet::path_in_order() const -> std::Vector<PathInOrder> {
        return std::Vector<PathInOrder>{};
    }

    auto TrackToBumpsNet::has_tob_in_ports(hardware::TOB* tob) const -> bool {
        for (auto& b: this->_end_bumps) {
            if (b->tob()->coord() == tob->coord()) {
                return true;
            }
        }
        return false;
    }
}
