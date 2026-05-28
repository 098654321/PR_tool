#pragma once

#include <hardware/coord.hh>
#include <hardware/tob/tobregister.hh>
#include <std/collection.hh>
#include <std/string.hh>
#include <std/utility.hh>
#include <global/debug/debug.hh>
#include <circuit/path/pathpackage.hh>
#include <variant>
#include <cassert>
#include <type_traits>




namespace PR_tool::algo {
    class RouteStrategy;
    class RouteEngine;
    class IncreRouting;
}

namespace PR_tool::circuit {

    class Priority {
    public:
        Priority(float value): _value{value} {}
        auto operator > (const Priority& other) const -> bool {
            return this->_value < other._value;
        }
        auto value() const -> float {
            return this->_value;
        }

    private:
        float _value;
    };

    class Net {
    public:
        virtual auto update_tob_postion(hardware::TOB* prev_tob, hardware::TOB* next_tob) -> void = 0;
        virtual auto route(hardware::Interposer* interposer, const algo::RouteStrategy& strategy) -> void = 0;
        virtual auto incremental_route(hardware::Interposer*, const algo::IncreRouting&, algo::RouteEngine&, bool) -> bool = 0;
        virtual auto update_priority(float bias) -> void = 0;
        virtual auto coords() const -> std::Vector<hardware::Coord> = 0;
        virtual auto path_in_order() const -> std::Vector<PathInOrder> = 0;
        virtual auto has_tob_in_ports(hardware::TOB* tob) const -> bool = 0;

        // update accessable_cobunit for each bump in this net
        virtual auto check_accessable_cobunit() -> void = 0;
        // return unordered_map<bump* -> a set of accessable cobunit>
        virtual auto accessable_cobunit() -> std::HashMap<hardware::Bump*, std::HashSet<std::usize>> = 0;
        // return nodes number
        virtual auto port_number() const -> std::usize = 0;
        // show nodes info
        virtual auto to_string() const -> std::String = 0;
        // for each bump, check if net in nets has the same node with the current bump and store related in member _related_~
        virtual auto search_related_nets(std::Vector<Net*>& nets) -> void = 0;
        // return an unordered_map<begin_bump* -> end_bumps*>
        virtual auto nodes_map() -> std::HashMap<hardware::Bump*, std::HashSet<hardware::Bump*>> = 0;
        // return an unrodered_map<bump*, TOBBumpDirection>
        virtual auto nodes_direction() -> std::HashMap<hardware::Bump*, hardware::TOBBumpDirection> = 0;
        // return all track port or track connected with bump; return false if there is bump not owns a track
        virtual auto track_ports() const -> std::Pair<std::HashSet<hardware::Track*>, bool> = 0;
        // return: (bumps_routable, bumps_unroutable, tracks_unroutable)
        virtual auto connection_state() const -> std::Tuple<std::Vector<const hardware::Bump*>, std::Vector<const hardware::Bump*>, std::Vector<const hardware::Track*>> = 0;
        // swap the position of two TOBs in this net
        virtual auto swap_tob_position(hardware::TOB* tob1, hardware::TOB* tob2) -> void = 0;
        
    public:
        virtual auto clear_related_nets() -> void;
        virtual auto clear_path() -> void;
        virtual auto clear_current_package() -> void;
        virtual auto show_path() const -> void;
        virtual auto length() const -> std::usize;
        virtual auto set_pathpackage(const PathPackage& path_package) -> void;
        virtual auto set_history_pathpackage() -> void;
        virtual auto move_history_to_current(hardware::Interposer*) -> void;
        
        // reset all package elements in iterate routing for incremental routing
        virtual auto reset_pathpackage() -> void {this->_path_package.reset_all();}

        virtual auto sync_length() const -> std::Tuple<std::usize, std::usize> {return std::Tuple<std::usize, std::usize>{0, 0};}
        virtual auto pathpackage() -> PathPackage& {return this->_path_package;}
        virtual auto pathpackage() const -> const PathPackage& {return this->_path_package;}
        virtual auto priority() const -> const Priority& {return this->_priority;}
        virtual auto modes() const -> const std::HashSet<int>& {return this->_modes;}
        virtual auto add_mode(int m) -> void {this->_modes.insert(m);}
        virtual auto set_reuse_type(bool type) -> void {this->_reuse_type.emplace(type);}
        virtual auto reuse_type() const -> std::Option<bool> {return this->_reuse_type;}
        virtual auto history_pathpackage() -> std::optional<HistoryPathPackage>& {return this->_history_path_package;}
        virtual auto name() const -> const std::String& {return this->_name;}
        virtual auto uid() const -> const std::String& {return this->_uid;}

        // check if node is the same with nodes belongs to this 
        virtual auto check_relativity(const hardware::Bump* node) const -> const Net* {return nullptr;}
        virtual auto check_relativity(const hardware::Track* node) const -> const Net* {return nullptr;}
        virtual auto check_modes(int m) -> Net* {return this->_modes.find(m) == this->_modes.end() ? this : nullptr;}
    
    public:
        virtual auto operator == (const Net& other) const -> bool = 0;

    public:
        template <class Node>
        auto related_nets(Node* node) const -> const std::Vector<Net*>& {
            static_assert(
                std::is_same<Node, hardware::Bump>::value || std::is_same<Node, hardware::Track>::value,\
                "Node must be hardware::Bump or hardware::Track"
            );
            
            if constexpr (std::is_same<Node, hardware::Bump>::value) {
                return this->_related_nets_bump.at(node);
            }
            else {
                return this->_related_nets_track.at(node);
            }
        }
    
    protected:
        // search nets which share common nodes with this 
        template <class Node>
        auto search_nets_node(Node* node, std::Vector<Net*>& nets) -> std::Vector<Net*> {
            std::Vector<Net*> related_nets {};
            for (auto& net : nets){
                if (net->check_relativity(node) != nullptr){
                    related_nets.emplace_back(net);
                }
            }
            return related_nets;
        }
    
    public:
        Net(Priority priority, const std::HashSet<int>& modes, std::String& name, std::String& uid):
            _priority{priority}, _name{name}, _uid{uid}, _path_package{}, _history_path_package{std::nullopt}, _modes{modes}, _reuse_type{std::nullopt}, _related_nets_track{}, _related_nets_bump{} {}
        virtual ~Net() noexcept {}
    
    protected:
        Priority _priority;
        std::String _name;
        std::String _uid;
        PathPackage _path_package;
        std::optional<HistoryPathPackage> _history_path_package;
        std::HashSet<int> _modes;
        std::Option<bool> _reuse_type;

        std::HashMap<hardware::Track*, std::Vector<Net*>> _related_nets_track;
        std::HashMap<hardware::Bump*, std::Vector<Net*>> _related_nets_bump;
    };

}