#include "./routeengine.hh"
#include <cassert>
#include <type_traits>
#include <ranges>
#include <algorithm>


namespace PR_tool::algo {

RouteEngine::RouteEngine(
    const std::HashMap<int, std::Vector<std::Rc<circuit::Net>>>& nets, const RouteStrategy& str, const AllocateStrategy& as, int m,
    bool incremental, bool try_all_modes, bool path_exists, hardware::Interposer* interposer
)
    : _posi{0}, _routestrategy{str}, _allocator{as}, _mode{m},
      _try_all_modes{try_all_modes}, _incremental{incremental}, _path_exists{path_exists}, _incre_strategy{}, _recorder{interposer}, _route_data{}
{
    for (auto& [m, net_v]: nets) {
        auto res = this->_nets.emplace(m, std::Vector<circuit::Net*>{});

        for (auto& net: net_v) {
            res.first->second.emplace_back(net.get());
        }
    }
    this->_incre_strategy.set_recorder(&this->_recorder);
}

auto RouteEngine::routed_nets() const -> std::Vector<circuit::Net*> {
    assert(this->_posi < this->nets().size());

    std::Vector<circuit::Net*> res {};
    std::usize index = 0;
    for (auto& net: this->nets()) {
        if (index >= this->_posi) {
            break;
        }
        res.emplace_back(net);
        index += 1;
    }
    return res;
}

// only used in simple routing
auto RouteEngine::update_net_seq(std::Vector<circuit::Net*>& nets) -> void {
    this->_nets.at(this->_mode) = nets;
}


auto RouteEngine::show_data_in_cycle(std::usize cycle, const std::Vector<circuit::Net*>& nets) -> void {
    this->_route_data.collect_data_in_cycle(cycle, nets);
    this->_route_data.show_data_in_cycle(cycle, true);
}


auto RouteEngine::show_final_data(const std::Vector<circuit::Net*>& nets, bool incre, std::i64 elapsed_ms) -> DataPerCycle {
    auto data = this->_route_data.collect_data(nets, incre);

    if (incre) {
        auto [monopolized_by_reuse, has_nonreuse] = data._reg_data;
        auto sum = monopolized_by_reuse + has_nonreuse;
        debug::info_fmt("\n\
        Total Length: {}\n\
        Sync Net Number: {}\n\
        Average Sync Length: {}\n\
        Max Length: {}\n\
        Monopolized by Reuse: {}({}%)\n\
        Has Nonreuse: {}({}%)\n\
        Failed routing nubmer: {}\n\
        Parse and Routing Time: {} milliseconds\n\
        ", data._total_length, data._sync_net_number, data._ave_sync_length, data._max_length, monopolized_by_reuse, 100*monopolized_by_reuse/sum, has_nonreuse, 100*has_nonreuse/sum, data._failed_net, elapsed_ms
        );
    }
    else {
        debug::info_fmt("\n\
        Total Length: {}\n\
        Sync Net Number: {}\n\
        Average Sync Length: {}\n\
        Max Length: {}\n\
        Failed routing nubmer: {}\n\
        Parse and Routing Time: {} milliseconds\n\
        ", data._total_length, data._sync_net_number, data._ave_sync_length, data._max_length, data._failed_net, elapsed_ms
        );
    }

    return data;
}


auto RouteEngine::show_net_and_path() -> void {
    auto nets = this->nets();

    for (const auto& net: nets) {
        debug::info(net->to_string());
        auto l = net->length();

        if (l > 0) {
            debug::info_fmt("Routing length of this net: {}", l);
            debug::info_fmt("Routing priority of this net: {}", net->priority().value());
            net->show_path();
        }
        else {
            debug::info_fmt("Routing failed for this net: {}", net->name());
        }
    }
}


// return nets to be routed but without an existing path in the initial sequence in routeengine
auto RouteEngine::nets() const -> std::Vector<circuit::Net*> {
    if (this->_incremental) {
        auto all_nets = std::Vector<circuit::Net*> {};
        auto s = std::Set<circuit::Net*> {};

        if (this->_try_all_modes) {
            for (auto& [m, net_v]: this->_nets) {
                s.insert(net_v.begin(), net_v.end());
            }

            for (auto& [m, net_v]: this->_nets) {
                for (auto& net: net_v) {
                    if (s.contains(net)) {
                        all_nets.emplace_back(net);
                        s.erase(net);
                    }
                }
            }
        }
        else {
            all_nets = this->_nets.at(this->_mode);
        }

        if (!this->_path_exists) {
            // no existing path, return all nets
            return all_nets;
        }
        else {
            if (this->_try_all_modes) {
                return all_nets;
            }
            else {
                // return nets not shared between modes
                auto iter = std::remove_if(all_nets.begin(), all_nets.end(), [&](auto& net) {
                    return net->modes().contains(this->_mode) && net->modes().size() > 1;
                });
                all_nets.erase(iter, all_nets.end());
                return all_nets;
            }
        }
    }
    else {
        return this->_nets.at(this->_mode);
    }
}

// return nets with the initial sequence in routeengine
auto RouteEngine::nets(int mode) const -> std::Vector<circuit::Net*> {
    auto net_in_mode = this->_nets.at(mode);
    
    if (this->_incremental) {
        if (!this->_path_exists) {
            return net_in_mode;
        }
        else {
            if (this->_try_all_modes) {
                return net_in_mode;
            }
            else {
                auto iter = std::remove_if(net_in_mode.begin(), net_in_mode.end(), [&](auto& net) {
                    return net->modes().contains(this->_mode) && net->modes().size() > 1;
                });
                net_in_mode.erase(iter, net_in_mode.end());
                return net_in_mode;
            }
        }
    }
    else {
        return this->_nets.at(this->_mode);
    }
}

auto RouteEngine::all_nets_in_modes(int mode) const -> std::Vector<circuit::Net*> {
    return this->_nets.at(mode);
}

auto RouteEngine::nets_loaded_with_path() const -> std::Vector<circuit::Net*> {
    for (auto& [m, net_v]: this->_nets) {
        for (auto& net: net_v) {
            if (net->pathpackage()._length > 0) {
                debug::info_fmt("Net int mode {} is loaded with path", m);
                return this->all_nets_in_modes(m);
            }
        }
    }

    debug::info_fmt("No net is loaded with path");
    return std::Vector<circuit::Net*> {};
}

auto RouteEngine::all_nets() -> std::Vector<circuit::Net*> {
    auto all_nets = std::Set<circuit::Net*> {};
    for (auto& [m, net_v]: this->_nets) {
        for (auto& net: net_v) {
            if (!all_nets.contains(net)) {
                all_nets.emplace(net);
            }
        }
    }
    return std::Vector<circuit::Net*>(all_nets.begin(), all_nets.end());
}


}

