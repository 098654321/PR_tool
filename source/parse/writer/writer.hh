#pragma once

#include <std/file.hh>
#include <std/collection.hh>
#include <global/debug/debug.hh>
#include <std/integer.hh>
#include <format>
#include <functional>
#include "./registers/registervalues.hh"
#include "./registers/baseregister.hh"
#include "./register_defaults.hh"
#include <parse/reader/config/config.hh>




namespace PR_tool::hardware
{
    class Interposer;
}


namespace PR_tool::parse {

    class RegisterValue;
    class TobRegister;

    template<std::usize N_bits, std::usize N_parts>
    auto split_bits(const std::Bits<N_bits>& controlbits) -> std::array<std::Bits<N_bits / N_parts>, N_parts>; 

    template<std::usize N_bits, std::usize N_parts>
    auto split_array(const std::array<std::size_t, N_bits>& controlbits) -> std::array<std::array<std::size_t, N_bits / N_parts>, N_parts>;
    

    class Writer
    {
    public:
        Writer(hardware::Interposer* pinterposer);

    public:
        // Fetch once into _values, then write one tree.
        auto fetch_and_write_split(
            const RegisterMapConfig& register_map,
            const std::FilePath& output_root,
            bool simplify
        ) -> void;

        // Fetch once; write full then simplified trees (test-only).
        auto fetch_and_write_split_pair(
            const RegisterMapConfig& register_map,
            const std::FilePath& full_output_root,
            const std::FilePath& simplified_output_root
        ) -> void;
    
    private:
        auto fetch() -> void;
        auto build_regs() -> void;
        auto collect_values() -> void;
        auto write_split_files(
            const RegisterMapConfig& register_map,
            const std::FilePath& output_root,
            bool simplify
        ) -> void;
        auto store_line(const std::String& hex, const std::String& name) -> void;
    
    private:
        auto write_cob() -> void;
        auto write_tob() -> void;
        auto write_xinzhai() -> void;

        auto write_cob_template(const std::Bits<128>&, std::String, std::usize, std::usize) -> void;
        auto write_tob_template64(const std::Bits<64>&, std::String, std::usize, std::usize) -> void;
        auto write_tob_template128(const std::Bits<128>&, std::String, std::usize, std::usize) -> void;
        auto write_tob_template_mux(const std::Array<std::usize, 128>&, std::String, std::usize, std::usize) -> void;
        auto only_this_one_looks_f__king_different_from_others(const std::Bits<128>&, std::String, std::usize, std::usize) -> void;
        auto write_xinzhai_template(const std::Bits<128>&, const std::string& name) -> void;
    
    private:
        auto to_hex(const std::Bits<32>& bits) -> std::String;

    private:
        RegisterValue _rv;
        std::Vector<BaseRegister*> _regs;
        hardware::Interposer* _pinterposer;
        std::HashMap<std::String, std::String> _values;  // name -> hex
    };

}
