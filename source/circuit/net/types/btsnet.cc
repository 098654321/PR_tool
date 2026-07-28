#include "./btsnet.hh"
#include <hardware/bump/bump.hh>
#include <hardware/track/track.hh>
#include <algo/router/incremental/maze/routing.hh>


namespace PR_tool::circuit {

    BumpToTracksNet::BumpToTracksNet(hardware::Bump* begin_bump, std::Vector<hardware::Track*> end_tracks, const std::HashSet<int>& modes, std::String& name, std::String& uid) :
        _begin_bump{begin_bump},
        _end_tracks{std::move(end_tracks)},
        Net{Priority{1}, modes, name, uid}
    {
    }

    BumpToTracksNet::~BumpToTracksNet() noexcept {

    }

    auto BumpToTracksNet::update_tob_postion(hardware::TOB* prev_tob, hardware::TOB* next_tob) -> void {
        this->_begin_bump = hardware::Bump::update_bump(this->_begin_bump, prev_tob, next_tob);
    }

    auto BumpToTracksNet::swap_tob_position(hardware::TOB* tob1, hardware::TOB* tob2) -> void {
        this->_begin_bump = hardware::Bump::swap_bump(this->_begin_bump, tob1, tob2);
    }

    auto BumpToTracksNet::route(hardware::Interposer* interposer, const algo::RouteStrategy& strategy) -> void {
        strategy.route_bump_to_tracks_net(interposer, this);
    }

    auto BumpToTracksNet::incremental_route(
        hardware::Interposer* interposer, const algo::IncreRouting& strategy, algo::RouteEngine& engine, bool shared
    ) -> bool {
        return strategy.route_bump_to_tracks_net(interposer, this, engine, shared);
    }

    auto BumpToTracksNet::update_priority(float bias) -> void {
        assert(0 <= bias && bias < 1);
        this->_priority = Priority{this->_priority.value() + bias};
    }

    auto BumpToTracksNet::coords() const -> std::Vector<hardware::Coord> {
        auto coords = std::Vector<hardware::Coord>{};
        for (auto track : this->end_tracks()) {
            coords.emplace_back(track->coord());
        }
        coords.emplace_back(this->begin_bump()->coord());
        return coords;
    }

    auto BumpToTracksNet::check_accessable_cobunit() -> void {
        std::HashSet<std::usize> accessable_cobunit {};
        std::usize bank_size = hardware::COB::INDEX_SIZE/2;
        std::usize wilton_size = hardware::COBUnit::WILTON_SIZE;

        for (auto track: _end_tracks){
            std::usize index = track->coord().index;
            std::usize id {(index/bank_size)*wilton_size + (index%wilton_size)};
            accessable_cobunit.emplace(id);
        }
        _begin_bump->intersect_access_unit(accessable_cobunit);
    }

    auto BumpToTracksNet::accessable_cobunit() -> std::HashMap<hardware::Bump*, std::HashSet<std::usize>> {
        std::HashMap<hardware::Bump*, std::HashSet<std::usize>> map {};
        map.emplace(this->_begin_bump, this->_begin_bump->accessable_cobunit());
        return map;
    }

    auto BumpToTracksNet::to_string() const -> std::String {
        auto ss = std::StringStream {};
        ss << std::format("{}: Begin bump '{}' to End tracks '[", this->_name, this->_begin_bump->coord());
        for (int i = 0; i < this->_end_tracks.size(); ++i) {
            if (i != 0) {
                ss << ", ";
            }
            ss << std::format("{}", this->_end_tracks[i]->coord());
        }
        ss << ']';
        return ss.str();
    }

    auto BumpToTracksNet::port_number() const -> std::usize {
        return (this->_end_tracks.size() + 1);
    }

    auto BumpToTracksNet::check_relativity(const hardware::Bump* node) const -> const Net* {
        if (this->_begin_bump->coord() == node->coord()) {
            return dynamic_cast<const Net*>(this);
        }
        else {
            return nullptr;
        }
    }

    auto BumpToTracksNet::check_relativity(const hardware::Track* node) const -> const Net* {
        for (auto track : this->_end_tracks) {
            if (track->coord() == node->coord()) {
                return dynamic_cast<const Net*>(this);
            }
        }
        return nullptr;
    }

    auto BumpToTracksNet::search_related_nets(std::Vector<Net*>& nets) -> void {
        clear_related_nets();

        auto iter = std::find(nets.begin(), nets.end(), this);
        if (iter != nets.end()) {
            nets.erase(iter);
        }
        this->_related_nets_bump.emplace(this->_begin_bump, search_nets_node<hardware::Bump>(this->_begin_bump, nets));
        for (auto track : this->_end_tracks) {
            this->_related_nets_track.emplace(track, search_nets_node<hardware::Track>(track, nets));
        }
    }

    auto BumpToTracksNet::connection_state() const -> std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
        std::Vector<const hardware::Bump*> routable_bumps{}, unroutable_bumps{};
        std::Vector<const hardware::Track*> unroutable_tracks{};

        const auto& package = this->pathpackage();

        (package.find_bump(this->_begin_bump).has_value() ? routable_bumps : unroutable_bumps).emplace_back(this->_begin_bump);
        for (auto& track: this->_end_tracks) {
            if (!package.find_track(track).has_value()) {
                unroutable_tracks.emplace_back(track);
            }
        }

        return std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
            routable_bumps, unroutable_bumps, unroutable_tracks
        };
    }

    auto BumpToTracksNet::nodes_map() -> std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> {
        return std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> {};
    }

    auto BumpToTracksNet::nodes_direction() -> std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> {
        return std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> {
            {this->_begin_bump, hardware::TOBBumpDirection::BumpToTOB}
        };
    }

    auto BumpToTracksNet::operator == (const Net& net) const -> bool {
    try {
        auto cast_net = dynamic_cast<const BumpToTracksNet&>(net);
        if (this->_begin_bump->coord() == cast_net._begin_bump->coord() && this->_end_tracks.size() == cast_net._end_tracks.size()) {
            std::HashSet<hardware::TrackCoord> coords {};
            for (auto& track : this->_end_tracks) {
                coords.emplace(track->coord());
            }

            for (auto& track : cast_net._end_tracks) {
                if (!coords.contains(track->coord())) {
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

    auto BumpToTracksNet::track_ports() const -> std::Pair<std::HashSet<hardware::Track*>, bool> {
        std::HashSet<hardware::Track*> tracks {};

        for (auto& t: this->_end_tracks) {
            tracks.emplace(t);
        }

        auto collect = [&](const auto& package_v) {
            std::for_each(package_v.begin(), package_v.end(), [&](const auto& package) {
                const auto& [_1, _2, t] = package;
                tracks.emplace(t);
            });
        };
        collect(this->_path_package._tob_to_track);
        collect(this->_path_package._track_to_tob);     // should be empty

        if (tracks.size() < this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, false};
        }
        else if (tracks.size() == this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, true};
        }
        else {
            throw std::logic_error("BumpToTracksNet::track_ports(): collected tracks.size() > port_number()");
        }
    }

    auto BumpToTracksNet::name() const -> const std::String& {
        return this->_name;
    }

    auto BumpToTracksNet::path_in_order() const -> std::Vector<PathInOrder> {
        return std::Vector<PathInOrder>{};
    }

    auto BumpToTracksNet::has_tob_in_ports(hardware::TOB* tob) const -> bool {
        return this->_begin_bump->tob()->coord() == tob->coord();
    }
}
