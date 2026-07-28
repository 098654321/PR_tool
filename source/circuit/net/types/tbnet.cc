#include "./tbnet.hh"
#include <hardware/bump/bump.hh>
#include <hardware/track/track.hh>
#include <algo/router/incremental/maze/routing.hh>


namespace PR_tool::circuit {

    TrackToBumpNet::TrackToBumpNet(hardware::Track* begin_track, hardware::Bump* end_bump, const std::HashSet<int>& modes, std::String& name, std::String& uid) :
        _begin_track{begin_track},
        _end_bump{end_bump},
        Net{Priority{3}, modes, name, uid}
    {
    }

    TrackToBumpNet::~TrackToBumpNet() noexcept {
    }

    auto TrackToBumpNet::update_tob_postion(hardware::TOB* prev_tob, hardware::TOB* next_tob) -> void {
        this->_end_bump = hardware::Bump::update_bump(this->_end_bump, prev_tob, next_tob);
    }

    auto TrackToBumpNet::swap_tob_position(hardware::TOB* tob1, hardware::TOB* tob2) -> void {
        this->_end_bump = hardware::Bump::swap_bump(this->_end_bump, tob1, tob2);
    }

    auto TrackToBumpNet::route(hardware::Interposer* interposer, const algo::RouteStrategy& strategy) -> void {
        strategy.route_track_to_bump_net(interposer, this);
    }

    auto TrackToBumpNet::incremental_route(
        hardware::Interposer* interposer, const algo::IncreRouting& strategy, algo::RouteEngine& engine, bool shared
    ) -> bool {
        return strategy.route_track_to_bump_net(interposer, this, engine, shared);
    }

    auto TrackToBumpNet::update_priority(float bias) -> void {
        assert(0 <= bias && bias < 1);
        this->_priority = Priority{this->_priority.value() + bias};
    }

    auto TrackToBumpNet::coords() const -> std::Vector<hardware::Coord> {
        return std::Vector<hardware::Coord>{this->begin_track()->coord(), this->end_bump()->coord()};
    }

    auto TrackToBumpNet::check_accessable_cobunit() -> void {
        std::usize index = _begin_track->coord().index;
        std::usize bank_size = hardware::COB::INDEX_SIZE/2;
        std::usize wilton_size = hardware::COBUnit::WILTON_SIZE;
        std::usize id {(index/bank_size)*wilton_size + (index%wilton_size)};
        
        std::HashSet<std::usize> cobunit_ids {id};
        _end_bump->intersect_access_unit(cobunit_ids);
    }

    auto TrackToBumpNet::accessable_cobunit() -> std::HashMap<hardware::Bump*, std::HashSet<std::usize>> {
        std::HashMap<hardware::Bump*, std::HashSet<std::usize>> map{};
        map.emplace(this->_end_bump, this->_end_bump->accessable_cobunit());
        return map;
    }

    
    auto TrackToBumpNet::to_string() const -> std::String {
        return std::format(
            "{}: Begin track: '{}' to End bump '{}'", this->_name, this->_begin_track->coord(), this->_end_bump->coord()
        );
    }

    auto TrackToBumpNet::port_number() const -> std::usize {
        return 2;
    }

    auto TrackToBumpNet::check_relativity(const hardware::Bump* node) const -> const Net* {
        if (this->_end_bump->coord() == node->coord()) {
            return this;
        }
        return nullptr;
    }

    auto TrackToBumpNet::check_relativity(const hardware::Track* node) const -> const Net* {
        if (this->_begin_track->coord() == node->coord()) {
            return this;
        }
        return nullptr;
    }

    auto TrackToBumpNet::search_related_nets(std::Vector<Net*>& nets) -> void {
        clear_related_nets();

        auto iter = std::find(nets.begin(), nets.end(), this);
        if (iter != nets.end()) {
            nets.erase(iter);
        }
        this->_related_nets_bump.emplace(this->_end_bump, search_nets_node<hardware::Bump>(this->_end_bump, nets));
        this->_related_nets_track.emplace(this->_begin_track, search_nets_node<hardware::Track>(this->_begin_track, nets));
    }

    auto TrackToBumpNet::connection_state() const -> std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
        std::Vector<const hardware::Bump*> routable_bumps{}, unroutable_bumps{};
        std::Vector<const hardware::Track*> unroutable_tracks{};

        const auto& package = this->pathpackage();

        (package.find_bump(this->_end_bump).has_value() ? routable_bumps : unroutable_bumps).emplace_back(this->_end_bump);
        if (!package.find_track(this->_begin_track).has_value()) {
            unroutable_tracks.emplace_back(this->_begin_track);
        }

        return std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
            routable_bumps, unroutable_bumps, unroutable_tracks
        };
    }

    auto TrackToBumpNet::nodes_map() -> std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> {
        return std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>>{};
    }

    auto TrackToBumpNet::nodes_direction() -> std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> {
        return std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> {
            {this->_end_bump, hardware::TOBBumpDirection::TOBToBump}
        };
    }

    auto TrackToBumpNet::operator == (const Net& net) const -> bool {
    try {
        auto cast_net = dynamic_cast<const TrackToBumpNet&>(net);
        return this->_end_bump->coord() == cast_net._end_bump->coord()
            && this->_begin_track->coord() == cast_net._begin_track->coord();
    }
    catch (const std::bad_cast& e) {
        return false;
    }
    }

    auto TrackToBumpNet::operator == (const TrackToBumpNet& net) const -> bool {
        return this->_end_bump->coord() == net._end_bump->coord()
            && this->_begin_track->coord() == net._begin_track->coord();
    }

    auto TrackToBumpNet::track_ports() const -> std::Pair<std::HashSet<hardware::Track*>, bool> {
        std::HashSet<hardware::Track*> tracks {this->_begin_track};

        auto collect = [&](const auto& package_v) {
            std::for_each(package_v.begin(), package_v.end(), [&](const auto& package) {
                const auto& [_1, _2, t] = package;
                tracks.emplace(t);
            });
        };
        collect(this->_path_package._tob_to_track); // should be empty
        collect(this->_path_package._track_to_tob);

        if (tracks.size() < this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, false};
        }
        else if (tracks.size() == this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, true};
        }
        else {
            throw std::logic_error("TrackToBumpNet::track_ports(): collected tracks.size() > port_number()");
        }
    }

    auto TrackToBumpNet::name() const -> const std::String& {
        return this->_name;
    }

    auto TrackToBumpNet::path_in_order() const -> std::Vector<PathInOrder> {
    try{
        const auto& package = this->pathpackage();
        if (package._track_to_tob.size() > 0) {
            // should has path
            const auto& tail_tobconnector = std::get<1>(*package._track_to_tob.begin());
            const auto& regular_path = package._regular_path;
            return std::Vector<PathInOrder>{
                PathInOrder(
                    std::nullopt,
                    std::nullopt,
                    std::make_optional(this->_end_bump),
                    std::make_optional(tail_tobconnector),
                    regular_path
                )
            };
        }
        else {
            return std::vector<PathInOrder>{};
        }
    }
    catch(const std::exception& e) {
        throw std::runtime_error("TrackToBumpNet::path_in_order(): " + std::string(e.what()));
    }
    }

    auto TrackToBumpNet::has_tob_in_ports(hardware::TOB* tob) const -> bool {
        return this->_end_bump->tob()->coord() == tob->coord();
    }
}
