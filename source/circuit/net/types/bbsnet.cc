#include "./bbsnet.hh"
#include "std/string.hh"
#include <format>
#include <hardware/bump/bump.hh>
#include <algorithm>
#include <algo/router/incremental/maze/routing.hh>


namespace PR_tool::circuit {

    BumpToBumpsNet::BumpToBumpsNet(hardware::Bump* begin_bump, std::Vector<hardware::Bump*> end_bumps, const std::HashSet<int>& modes, std::String& name, std::String& uid) :
        _begin_bump{begin_bump},
        _end_bumps{std::move(end_bumps)},
        Net{Priority{2}, modes, name, uid}
    {
    }

    BumpToBumpsNet::~BumpToBumpsNet() noexcept {

    }

    auto BumpToBumpsNet::update_tob_postion(hardware::TOB* prev_tob, hardware::TOB* next_tob) -> void {
        this->_begin_bump = hardware::Bump::update_bump(this->_begin_bump, prev_tob, next_tob);
        for (auto& bump : this->_end_bumps) {
            bump = hardware::Bump::update_bump(bump, prev_tob, next_tob);
        }
    }

    auto BumpToBumpsNet::swap_tob_position(hardware::TOB* tob1, hardware::TOB* tob2) -> void {
        this->_begin_bump = hardware::Bump::swap_bump(this->_begin_bump, tob1, tob2);
        for (auto& bump : this->_end_bumps) {
            bump = hardware::Bump::swap_bump(bump, tob1, tob2);
        }
    }

    auto BumpToBumpsNet::route(hardware::Interposer* interposer, const algo::RouteStrategy& strategy) -> void {
        strategy.route_bump_to_bumps_net(interposer, this);
    }

    auto BumpToBumpsNet::incremental_route(
        hardware::Interposer* interposer, const algo::IncreRouting& strategy, algo::RouteEngine& engine, bool shared
    ) -> bool {
        return strategy.route_bump_to_bumps_net(interposer, this, engine, shared);
    }

    auto BumpToBumpsNet::update_priority(float bias) -> void {
        assert(0 <= bias && bias < 1);
        this->_priority = Priority{this->_priority.value() + bias};
    }

    auto BumpToBumpsNet::coords() const -> std::Vector<hardware::Coord> {
        auto coords = std::Vector<hardware::Coord>{};
        for (auto bump : this->end_bumps()) {
            coords.emplace_back(bump->coord());
        }
        coords.emplace_back(this->begin_bump()->coord());
        return coords;
    }

    auto BumpToBumpsNet::check_accessable_cobunit() -> void {
        std::HashSet<std::usize> accessable_cobunit {};
        for (std::usize i = 0; i < hardware::COB::UNIT_SIZE; ++i){
            accessable_cobunit.emplace(i);
        }
        _begin_bump->intersect_access_unit(accessable_cobunit);
        for (auto bump : _end_bumps) {
            bump->intersect_access_unit(accessable_cobunit);
        }
    }

    auto BumpToBumpsNet::accessable_cobunit() -> std::HashMap<hardware::Bump*, std::HashSet<std::usize>> {
        std::HashMap<hardware::Bump*, std::HashSet<std::usize>> map{};

        map.emplace(this->_begin_bump, this->_begin_bump->accessable_cobunit());
        for (auto& b: this->_end_bumps) {
            map.emplace(b, b->accessable_cobunit());
        }
        return map;
    }

    auto BumpToBumpsNet::to_string() const -> std::String {
        auto ss = std::StringStream {};
        ss << std::format("{}: Begin bump '{}' to End bumps '[", this->_name, this->_begin_bump->coord());
        for (int i = 0; i < this->_end_bumps.size(); ++i) {
            if (i != 0) {
                ss << ", ";
            }
            ss << std::format("{}", this->_end_bumps[i]->coord());
        }
        ss << ']';
        return ss.str();
    }

    auto BumpToBumpsNet::port_number() const -> std::usize {
        return (this->_end_bumps.size() + 1);
    }

    auto BumpToBumpsNet::check_relativity(const hardware::Bump* node) const -> const Net* {
        if (this->_begin_bump->coord() == node->coord()) {
            return dynamic_cast<const Net*>(this);
        }
        for (auto bump : this->_end_bumps) {
            if (bump->coord() == node->coord()) {
                return dynamic_cast<const Net*>(this);
            }
        }
        return nullptr;
    }

    auto BumpToBumpsNet::search_related_nets(std::Vector<Net*>& nets) -> void {
        clear_related_nets();

        auto iter = std::find(nets.begin(), nets.end(), this);
        if (iter != nets.end()) {
            nets.erase(iter);
        }
        this->_related_nets_bump.emplace(this->_begin_bump, search_nets_node<hardware::Bump>(this->_begin_bump, nets));
        for (auto& bump: this->_end_bumps) {
            this->_related_nets_bump.emplace(bump, search_nets_node<hardware::Bump>(bump, nets));
        }
    }

    auto BumpToBumpsNet::connection_state() const -> std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
        std::Vector<const hardware::Bump*> routable_bumps {}, unroutable_bumps {};

        const auto& package = this->pathpackage();

        auto classify_bumps = [&](const hardware::Bump* bump) {
            (package.find_bump(bump).has_value() ? routable_bumps : unroutable_bumps);
        };
        classify_bumps(this->_begin_bump);
        std::for_each(this->_end_bumps.begin(), this->_end_bumps.end(), classify_bumps);

        return std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> {
            routable_bumps, unroutable_bumps, std::Vector<const hardware::Track*> {}
        };
    }

    auto BumpToBumpsNet::nodes_map() -> std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> {
        std::HashSet<hardware::Bump*> bump_set {};
        for (auto& end_bump: this->_end_bumps) {
            bump_set.emplace(end_bump);
        }

        return std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> {
            {this->_begin_bump, bump_set}
        };
    }

    auto BumpToBumpsNet::nodes_direction() -> std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> {
        std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> map{};
        map.emplace(this->_begin_bump, hardware::TOBBumpDirection::BumpToTOB);
        for (auto& b: this->_end_bumps) {
            map.emplace(b, hardware::TOBBumpDirection::TOBToBump);
        }
        return map;
    }

    auto BumpToBumpsNet::operator == (const Net& net) const -> bool {
    try {
        auto cast_net = dynamic_cast<const BumpToBumpsNet&>(net);
        if (this->_begin_bump->coord() == cast_net._begin_bump->coord() && this->_end_bumps.size() == cast_net._end_bumps.size()) {
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
    
    auto BumpToBumpsNet::track_ports() const -> std::Pair<std::HashSet<hardware::Track*>, bool> {
        std::HashSet<hardware::Track*> tracks {};

        auto collect = [&](const auto& package_v) {
            std::for_each(package_v.begin(), package_v.end(), [&](const auto& package) {
                const auto& [_1, _2, t] = package;
                tracks.emplace(t);
            });
        };
        collect(this->_path_package._tob_to_track);
        collect(this->_path_package._track_to_tob);

        if (tracks.size() < this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, false};
        }
        else if (tracks.size() == this->port_number()) {
            return std::Pair<std::HashSet<hardware::Track*>, bool>{tracks, true};
        }
        else {
            throw std::logic_error("BumpToBumpNet::track_ports(): collected tracks.size() > port_number()");
        }
    }

    auto BumpToBumpsNet::name() const -> const std::String& {
        return this->_name;
    }

    auto BumpToBumpsNet::path_in_order() const -> std::Vector<PathInOrder> {
        return std::Vector<PathInOrder>{};
    }

    auto BumpToBumpsNet::has_tob_in_ports(hardware::TOB* tob) const -> bool {
        if (this->_begin_bump->tob()->coord() == tob->coord()) {
            return true;
        }
        else {
            for (auto& b: this->_end_bumps) {
                if (b->tob()->coord() == tob->coord()) {
                    return true;
                }
            }
            return false;
        }
    }

}
