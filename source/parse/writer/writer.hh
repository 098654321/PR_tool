#pragma

#include <std/file.hh>
#include <std/collection.hh>
#include <global/debug/debug.hh>
#include <std/integer.hh>
#include <format>
#include <functional>
#include "./registers/registervalues.hh"
#include "./registers/baseregister.hh"
#include "./register_defaults.hh"




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
        Writer(hardware::Interposer* pinterposer, bool simplify = false);

    public:
        auto fetch_and_write(const std::FilePath& file) -> void;
        auto fetch_and_write_pair(const std::FilePath& full_file, const std::FilePath& simplified_file) -> void;
    
    private:
        auto fetch() -> void;
        auto write(std::ofstream& file) -> void;
        auto build_regs() -> void;
        auto maybe_write_line(std::ofstream& file, const std::String& hex, const std::String& name) -> void;
    
    private:
        auto write_cob(std::ofstream& file) -> void;
        auto write_tob(std::ofstream& file) -> void;
        auto write_xinzhai(std::ofstream& file) -> void;

        auto write_cob_template(std::ofstream&, const std::Bits<128>&, std::String, std::usize, std::usize) -> void;
        auto write_tob_template64(std::ofstream&, const std::Bits<64>&, std::String, std::usize, std::usize) -> void;
        auto write_tob_template128(std::ofstream&, const std::Bits<128>&, std::String, std::usize, std::usize) -> void;
        auto write_tob_template_mux(std::ofstream&, const std::Array<std::usize, 128>&, std::String, std::usize, std::usize) -> void;
        auto only_this_one_looks_f__king_different_from_others(std::ofstream&, const std::Bits<128>&, std::String, std::usize, std::usize) -> void;
        auto write_xinzhai_template(std::ofstream&, const std::Bits<128>&, const std::string& name) -> void;
    
    private:
        auto to_hex(const std::Bits<32>& bits) -> std::String;

    private:
        RegisterValue _rv;
        std::Vector<BaseRegister*> _regs;
        hardware::Interposer* _pinterposer;
        bool _simplify;
    };

}


