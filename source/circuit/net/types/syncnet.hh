#pragma once


#include "../net.hh"
#include "circuit/net/types/bbnet.hh"
#include "circuit/net/types/btnet.hh"
#include "circuit/net/types/tbnet.hh"
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <circuit/path/pathpackage.hh>

#include <std/collection.hh>
#include <std/memory.hh>



namespace PR_tool::algo
{
    class RouteStrategy;
    struct RerouteStrategy;
}

namespace PR_tool::circuit {

    class PathPackage;

    class BumpToBumpNet;
    class BumpToTrackNet;
    class TrackToBumpNet;

    class SyncNet : public Net {   
    public:
        // temperarily use shallow copy
        SyncNet(
            std::Vector<std::Rc<BumpToBumpNet>> btbnets,
            std::Vector<std::Rc<BumpToTrackNet>> bttnets,
            std::Vector<std::Rc<TrackToBumpNet>> ttbnets,
            const std::HashSet<int>& modes,
            std::String& name,
            std::String& uid
        );
        
    public:
        virtual auto update_tob_postion(hardware::TOB* prev_tob, hardware::TOB* next_tob) -> void override;
        virtual auto swap_tob_position(hardware::TOB* tob1, hardware::TOB* tob2) -> void override;
        virtual auto route(hardware::Interposer* interposer, const algo::RouteStrategy& strategy) -> void override;
        virtual auto incremental_route(hardware::Interposer*, const algo::IncreRouting&, algo::RouteEngine&, bool shared) -> bool override;
        virtual auto update_priority(float bias) -> void override;
        virtual auto coords() const -> std::Vector<hardware::Coord> override;
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
        virtual auto set_reuse_type(bool reuse_type) -> void override;
        virtual auto clear_path() -> void override;
        virtual auto name() const -> const std::String& override;
        virtual auto path_in_order() const -> std::Vector<PathInOrder> override;
        virtual auto clear_current_package() -> void override;
        virtual auto has_tob_in_ports(hardware::TOB* tob) const -> bool override;

        // mode for sync net is determined by _mode itself, not the _modes in net-members
        
        auto show_path() const -> void override;
        auto length() const -> std::usize override;
        auto sync_length() const -> std::Tuple<std::usize, std::usize> override;
        auto set_pathpackage(const circuit::PathPackage&) -> void override;
        auto set_history_pathpackage() -> void override;
        auto reset_pathpackage() -> void override;
        auto move_history_to_current(hardware::Interposer*) -> void override;

        // return false if this->_path_package is already filled
        auto collect_package() -> bool;

    public:
        virtual auto operator == (const Net& net) const -> bool override;
    
    public:
        auto btbnets() -> std::Vector<std::Rc<BumpToBumpNet>>& {return this->_btbnets;}
        auto bttnets() -> std::Vector<std::Rc<BumpToTrackNet>>& {return this->_bttnets;}
        auto ttbnets() -> std::Vector<std::Rc<TrackToBumpNet>>& {return this->_ttbnets;}

    private:
        std::Vector<std::Rc<BumpToBumpNet>>  _btbnets;
        std::Vector<std::Rc<BumpToTrackNet>> _bttnets;
        std::Vector<std::Rc<TrackToBumpNet>> _ttbnets;
    };

}



