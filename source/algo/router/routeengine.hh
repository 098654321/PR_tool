#pragma once

#include <circuit/net/nets.hh>
#include "./common/routestrategy.hh"
#include "./common/allocatestrategy.hh"
#include "./incremental/maze/routing.hh"
#include "./incremental/recorders/hardware_recorder.hh"
#include "./incremental/bound_bits/global_bits.hh"
#include <std/collection.hh>
#include <std/memory.hh>
#include <hardware/bump/bump.hh>
#include <hardware/track/track.hh>
#include <functional>
#include <algo/route_data.hh>


namespace PR_tool::circuit {
    class Net;
}

namespace PR_tool::hardware {
    class Bump;
    class Track;
}


namespace PR_tool::algo {
 
class RouteEngine {
public:
    RouteEngine(
        const std::HashMap<int, std::Vector<std::Rc<circuit::Net>>>& nets, const RouteStrategy& str, const AllocateStrategy& as, int m,
        bool incremental, bool try_all_modes, bool path_exists, hardware::Interposer* interposer
    );
    ~RouteEngine() = default;

    auto routed_nets() const -> std::Vector<circuit::Net*>;
    auto move_on() -> void { this->_posi += 1; }
    auto reset_position() -> void { this->_posi = 0; }
    auto update_net_seq(std::Vector<circuit::Net*>& nets) -> void;
    auto show_data_in_cycle(std::usize cycle, const std::Vector<circuit::Net*>& nets) -> void;
    auto show_final_data(const std::Vector<circuit::Net*>& nets, bool incre) -> DataPerCycle;
    auto show_net_and_path() -> void;

public:
    auto nets() const -> std::Vector<circuit::Net*>;
    auto nets(int mode) const -> std::Vector<circuit::Net*>;
    auto all_nets_in_modes(int mode) const -> std::Vector<circuit::Net*>;
    auto nets_loaded_with_path() const -> std::Vector<circuit::Net*>;
    auto all_nets() -> std::Vector<circuit::Net*>;

    auto mode() const -> int {return this->_mode;}
    auto incremental() const -> bool {return this->_incremental;}   
    auto position() const -> std::usize {return this->_posi;}
    auto routestrategy() const -> const RouteStrategy& {return this->_routestrategy;}
    auto allocatestrategy() const -> const AllocateStrategy& {return this->_allocator;}
    auto incre_route_strategy() const -> const IncreRouting& {return this->_incre_strategy;}
    auto recorder() -> HardwareRecorder& {return this->_recorder;}
    auto route_data() -> RouteData& {return this->_route_data;}
    auto init_route_data() -> void {this->_route_data.clear_data();}
    auto record_failed_net(const std::String& name) -> void {
        this->_failed_net_names.emplace_back(name);
    }
    auto failed_net_names() const -> const std::Vector<std::String>& {
        return this->_failed_net_names;
    }
    auto clear_failed_net_names() -> void {
        this->_failed_net_names.clear();
    }
    auto collect_data_when_fail(const std::Vector<circuit::Net*>& nets, bool incremental) -> void {
        this->_route_data.collect_data(nets, incremental);
    }
    auto nets_with_mode() -> std::HashMap<int, std::Vector<circuit::Net*>>& {
        return this->_nets;
    }
    auto try_all_modes() const -> bool {return this->_try_all_modes;}
    auto path_exists_in_other_modes() const -> bool {return this->_path_exists;}
    auto set_path_exists(bool path_exists) -> void {this->_path_exists = path_exists;}
    auto set_progress_callback(std::function<void(std::usize, std::usize)> cb) -> void {
        this->_progress = std::move(cb);
    }
    auto report_progress(std::usize done, std::usize total) const -> void {
        if (this->_progress) {
            this->_progress(done, total);
        }
    }

private:
    std::HashMap<int, std::Vector<circuit::Net*>> _nets;
    std::usize _posi;   // point to the net to be routed
    int _mode;

    const RouteStrategy& _routestrategy;
    const AllocateStrategy& _allocator;
    IncreRouting _incre_strategy;
    HardwareRecorder _recorder;
    RouteData _route_data;
    bool _incremental;
    bool _try_all_modes;
    bool _path_exists;
    std::Vector<std::String> _failed_net_names {};
    std::function<void(std::usize, std::usize)> _progress {};
};

}

