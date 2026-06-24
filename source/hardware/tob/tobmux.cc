#include "./tobmux.hh"
#include "hardware/tob/tobregister.hh"
#include <std/collection.hh>
#include <std/integer.hh>
#include <cassert>
#include <std/range.hh>
#include <stdexcept>
#include <format>
#include <debug/debug.hh>

namespace PR_tool::hardware {


    TOBMuxConnector::TOBMuxConnector(
        std::usize input_index, 
        std::usize output_index, 
        TOBMuxRegister* reg
    ) :
        _input_index{input_index},
        _output_index{output_index},
        _register{reg}
    {
    }


    auto TOBMuxConnector::connect() -> void {
        this->_register->set(this->_output_index);
    }

    auto TOBMuxConnector::give_out() -> void {
        this->_register->give_out(this->_output_index);
    }

    auto TOBMuxConnector::stay_inside() -> void {
        this->_register->stay_inside();
    }

    auto TOBMuxConnector::disconnect() -> void {
        this->_register->reset();
    }

    auto TOBMuxConnector::check_consistency() const -> void {
        this->_register->check_consistency(this->_output_index);
    }

    auto TOBMuxConnector::check_reg_address() const -> uintptr_t {
        return reinterpret_cast<uintptr_t>(this->_register);
    }

    ////////////////////////////////////////////////////////////////

    TOBMux::TOBMux(std::usize mux_size) :
        _mux_size{mux_size}
    {
        // avoid initializing _registers in initialization list for some unknown in-memory seizure
        this->_registers.reserve(mux_size);
        for (std::size_t i = 0; i < mux_size; ++i) {
            this->_registers.emplace_back(TOBMuxRegister());
        }
    }

    auto TOBMux::available_connectors(std::usize input_index, bool shared) -> std::Vector<TOBMuxConnector> {
        if (!shared) {
            auto& reg = this->_registers.at(input_index);   // source
            if (reg.get().has_value() || reg.is_given_out()) {
                return {};
            }
        
            auto connectors = std::Vector<TOBMuxConnector>{};
            for (const auto output_index : this->available_output_indexes()) {
                connectors.emplace_back(
                    input_index,    // source, [0, 7]
                    output_index,   // target, [0, 7]
                    &this->_registers.at(input_index)
                );
            }

            return connectors;
        }
        else {
            auto connectors = std::Vector<TOBMuxConnector>{};
            for (auto output_index: std::views::iota(0, (int)this->_mux_size)){
                connectors.emplace_back(
                    input_index,
                    output_index,
                    &this->_registers.at(input_index)
                );
            }
            return connectors;
        }
    }

    auto TOBMux::available_output_indexes() const -> std::Vector<std::usize> {
        auto useds = std::Vector<bool>(this->_mux_size, false);
        for (const auto& reg : this->_registers) {
            auto res = reg.get();       // reg is source, res is target
            if (res.has_value()) {    
                useds.at(*res) = true;
            }
            else if (reg.is_given_out()) {
                auto output_index = reg.given_out_index();
                assert(output_index.has_value());
                
                useds.at(output_index.value()) = true;
            }
        }

        auto indexes = std::Vector<std::usize>{};
        for (auto i = 0; i < this->_mux_size; ++i) {
            if (useds.at(i)) {
                continue;
            }
            indexes.emplace_back(i);
        }

        return indexes;
    }

    auto TOBMux::connector(std::usize input_index, std::usize output_index, bool give_out) -> TOBMuxConnector {
    try {
        assert(this->_registers.size() > input_index);
        assert(!this->_registers.at(input_index).is_given_out());

        if (give_out) {
            this->_registers.at(input_index).give_out(output_index);
        }
        
        return TOBMuxConnector {
            input_index, output_index, &this->_registers.at(input_index)
        };
    }
    catch (const std::exception& e) {
        throw std::runtime_error(std::format("TOBMux::connector(): {}", e.what()));
    }
    }

    auto TOBMux::randomly_map_remain_indexes() -> void {
        auto unused_indexes = this->available_output_indexes();

        std::usize index = 0;
        for (auto& reg : this->_registers) {
            if (!reg.get().has_value()) {
                if (reg.is_given_out()) {
                    const auto& out_index = reg.given_out_index();
                    if (!out_index.has_value()) {
                        throw std::runtime_error(
                            "randomly_map(): reg is given out with empty index before randomly mapping");
                    }
                    throw std::runtime_error(std::format(
                        "randomly_map(): reg is given out with index = {} before randomly mapping",
                        out_index.value()));
                }
                reg.set(unused_indexes.at(index));
                index += 1;
            } else {
                assert(reg.is_given_out());
            }
        }
    }

    auto TOBMux::index_map(std::usize input_index) const -> std::Option<std::usize> {
        return this->_registers.at(input_index).get();
    }

    auto TOBMux::registerr(std::usize input_index) -> TOBMuxRegister* {
        return &this->_registers.at(input_index);
    }

    auto TOBMux::registerr(std::usize input_index) const -> const TOBMuxRegister* {
        return &this->_registers.at(input_index);
    }

    auto TOBMux::reset_regs() -> void {
        for (auto& reg : this->_registers) {
            reg.reset();
        }
    }
    
}