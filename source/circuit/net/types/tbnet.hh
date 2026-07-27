#pragma once 

#include "../net.hh"
#include <hardware/cob/cob.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>


namespace PR_tool::hardware {
    class Track;
    class Bump;
}

namespace PR_tool::circuit {

    class TrackToBumpNet : public Net {
    public:
        TrackToBumpNet(hardware::Track* begin_track, hardware::Bump* end_bump, const std::HashSet<int>& modes, std::String& name, std::String& uid);
        virtual ~TrackToBumpNet() noexcept override;
    
    public:
        virtual auto update_tob_postion(hardware::TOB* prev_tob, hardware::TOB* next_tob) -> void override;
        virtual auto swap_tob_position(hardware::TOB* tob1, hardware::TOB* tob2) -> void override;
        virtual auto route(hardware::Interposer* interposer, const algo::RouteStrategy& strategy) -> void override;
        virtual auto incremental_route(hardware::Interposer*, const algo::IncreRouting&, algo::RouteEngine&, bool) -> bool override;
        virtual auto update_priority(float bias) -> void override;
        virtual auto check_accessable_cobunit() -> void override; 
        virtual auto accessable_cobunit() -> std::HashMap<hardware::Bump*, std::HashSet<std::usize>> override;
        virtual auto to_string() const -> std::String override;
        virtual auto port_number() const -> std::usize override;
        virtual auto search_related_nets(std::Vector<Net*>& nets) -> void override;
        virtual auto check_relativity(const hardware::Bump* node) const -> const Net* override;
        virtual auto check_relativity(const hardware::Track* node) const -> const Net* override;
        virtual auto connection_state() const -> std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> override;
        virtual auto nodes_map() -> std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> override;
        virtual auto nodes_direction() -> std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> override;
        virtual auto track_ports() const -> std::Pair<std::HashSet<hardware::Track*>, bool> override;
        virtual auto name() const -> const std::String& override;
        virtual auto path_in_order() const -> std::Vector<PathInOrder> override;
        virtual auto has_tob_in_ports(hardware::TOB* tob) const -> bool override;
    
    public:
        virtual auto operator == (const Net& net) const -> bool override;
        auto operator == (const TrackToBumpNet& net) const -> bool;

    public:
        auto begin_track() const -> hardware::Track* { return this->_begin_track; }
        auto end_bump() const -> hardware::Bump* { return this->_end_bump; }
        virtual auto coords() const -> std::Vector<hardware::Coord> override;

    private:
        hardware::Track* _begin_track;
        hardware::Bump* _end_bump;
    };

}