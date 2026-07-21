#include "./writer.hh"
#include "./registers/tobregister.hh"
#include "./registers/cobregister.hh"
#include "./registers/xinzhai_register.hh"
#include <hardware/interposer.hh>
#include <algorithm>
#include <filesystem>
#include <fstream>

#include <sstream>
#include <iomanip>
#include <iostream>


namespace PR_tool::parse
{

    Writer::Writer(hardware::Interposer* pinterposer):
            _rv{},
            _regs{},
            _pinterposer{pinterposer},
            _values{}
        {}

    auto Writer::store_line(const std::String& hex, const std::String& name) -> void
    {
        _values[name] = hex;
    }

    auto Writer::fetch_and_write_split(
        const RegisterMapConfig& register_map,
        const std::FilePath& output_root,
        bool simplify
    ) -> void
    {
        build_regs();
        fetch();
        collect_values();
        write_split_files(register_map, output_root, simplify);
    }

    auto Writer::fetch_and_write_split_pair(
        const RegisterMapConfig& register_map,
        const std::FilePath& full_output_root,
        const std::FilePath& simplified_output_root
    ) -> void
    {
        build_regs();
        fetch();
        collect_values();
        write_split_files(register_map, full_output_root, false);
        write_split_files(register_map, simplified_output_root, true);
    }

    auto Writer::collect_values() -> void
    {
        _values.clear();
        write_cob();
        write_tob();
        write_xinzhai();
    }

    auto Writer::write_split_files(
        const RegisterMapConfig& register_map,
        const std::FilePath& output_root,
        bool simplify
    ) -> void
    {
        if (register_map.empty()) {
            debug::fatal("register_map is empty; configure reigster_adder in config.json");
        }
        auto out_dir = output_root / "regnamecontrolbit_4part";
        std::filesystem::create_directories(out_dir);

        std::HashSet<std::String> used;
        std::usize omitted = 0;
        std::usize written = 0;

        for (const auto& [filename, regs] : register_map) {
            auto path = out_dir / filename;
            std::ofstream out(path);
            if (!out.is_open()) {
                throw std::runtime_error(std::format("cannot open {}", path.string()));
            }
            for (const auto& [reg_name, address] : regs) {
                auto it = _values.find(reg_name);
                if (it == _values.end()) {
                    debug::warning_fmt("register '{}' in map file '{}' missing from fetch", reg_name, filename);
                    continue;
                }
                used.insert(reg_name);
                const auto& hex = it->second;
                if (simplify && should_omit_simplified_line(hex, reg_name)) {
                    ++omitted;
                    continue;
                }
                out << hex << " " << address << " " << reg_name << "\n";
                ++written;
            }
        }
        for (const auto& [name, _] : _values) {
            if (!used.contains(name)) {
                debug::warning_fmt("register '{}' fetched but not present in register_map", name);
            }
        }
        if (simplify) {
            debug::info_fmt("split write: wrote {} line(s), omitted {} default line(s)", written, omitted);
        }
    }

    auto Writer::build_regs() -> void
    {
        TobRegister* ptr_tr = new TobRegister(_pinterposer);
        CobRegister* ptr_cr = new CobRegister(_pinterposer);
        XinzhaiRegister* ptr_xr = new XinzhaiRegister(_pinterposer);
        _regs.emplace_back(ptr_tr);
        _regs.emplace_back(ptr_cr);
        _regs.emplace_back(ptr_xr);
    }

    auto Writer::fetch() -> void
    {
        for(auto& p_r: _regs)
        {
            p_r->fetch_controlbits(_rv);    
        }
    }

    auto Writer::write_cob() -> void
    {
        for(std::i64 row = 0; row < hardware::Interposer::COB_ARRAY_HEIGHT; ++row)
        {
            for (std::i64 col = 0; col < hardware::Interposer::COB_ARRAY_WIDTH; ++col)
            {
                hardware::COBCoord cobcoord {row, col};
                if (_rv.cobs.contains(cobcoord))
                {
                    auto& cob_value = _rv.cobs.at(cobcoord);

                    write_cob_template(cob_value.right_sel, "right_sel", row, col);
                    write_cob_template(cob_value.left_sel, "left_sel", row, col);
                    write_cob_template(cob_value.up_sel, "up_sel", row, col);
                    write_cob_template(cob_value.down_sel, "down_sel", row, col);
                    write_cob_template(cob_value.sw_ru, "sw_ru", row, col);
                    write_cob_template(cob_value.sw_lu, "sw_lu", row, col);
                    write_cob_template(cob_value.sw_rd, "sw_rd", row, col);
                    write_cob_template(cob_value.sw_ld, "sw_ld", row, col);
                    write_cob_template(cob_value.sw_v, "sw_v", row, col);
                    write_cob_template(cob_value.sw_h, "sw_h", row, col);
                }
                else
                {
                    throw std::runtime_error(std::format("Writer::write_cob(): cannot find cob at ({}, {})", row, col));
                }
            }
        }
    }

    auto Writer::write_tob() -> void
    {
        for(std::i64 row = 0; row < hardware::Interposer::TOB_ARRAY_HEIGHT; ++row)
        {
            for (std::i64 col = 0; col < hardware::Interposer::TOB_ARRAY_WIDTH; ++col)
            {
                hardware::TOBCoord tobcoord {row, col};
                if (_rv.tobs.contains(tobcoord))
                {
                    auto& tob_value {_rv.tobs.at(tobcoord)};

                    only_this_one_looks_f__king_different_from_others(tob_value.tob2bump, "tob2bump", row, col);
                    write_tob_template64(tob_value.dly, "dly", row, col);
                    write_tob_template64(tob_value.drv, "drv", row, col);
                    write_tob_template_mux(tob_value.hctrl, "hctrl", row, col);
                    write_tob_template_mux(tob_value.vctrl, "vctrl", row, col);
                    write_tob_template64(tob_value.bank_mux, "bank_sel", row, col);
                    write_tob_template128(tob_value.tob2track, "tob2track", row, col);
                    only_this_one_looks_f__king_different_from_others(tob_value.bump2tob, "bump2tob", row, col);
                    write_tob_template128(tob_value.track2tob, "track2tob", row, col);
                }
                else
                {
                    throw std::runtime_error(std::format("cannot find cob at ({}, {})", row, col));
                }
            }
        }
    }

    auto Writer::write_xinzhai() -> void {
        write_xinzhai_template(_rv.xinzhai.padctrl_right, "xinzhai_C4_noi_right_pad_ctrl");
        write_xinzhai_template(_rv.xinzhai.padctrl_left, "xinzhai_C4_noi_left_pad_ctrl");
        write_xinzhai_template(_rv.xinzhai.padctrl_up, "xinzhai_C4_noi_up_pad_ctrl");
        write_xinzhai_template(_rv.xinzhai.padctrl_down, "xinzhai_C4_noi_down_pad_ctrl");

        write_xinzhai_template(_rv.xinzhai.SiPpadctrl_right, "xinzhai_C4_noi_SiP_right_pad_ctrl");
        write_xinzhai_template(_rv.xinzhai.SiPpadctrl_left, "xinzhai_C4_noi_SiP_left_pad_ctrl");
        write_xinzhai_template(_rv.xinzhai.SiPpadctrl_up, "xinzhai_C4_noi_SiP_up_pad_ctrl");
        write_xinzhai_template(_rv.xinzhai.SiPpadctrl_down, "xinzhai_C4_noi_SiP_down_pad_ctrl");
    }

    auto Writer::write_xinzhai_template(const std::Bits<128>& bits, const std::string& name) -> void {
        // reverse the bits and make the MSB on the right
        auto reverse_bits = std::Bits<128>{};
        for (std::size_t i = 0; i < 128; ++i) {
            reverse_bits[i] = bits[127 - i];
        }

        // output
        auto splitted_bits = split_bits<128, 4>(reverse_bits);
        for (std::usize i = 0; i < 4; i++)
        {
            std::String output_name = name + "_" + std::to_string(i);
            store_line(to_hex(splitted_bits[i]), output_name);
        }
    }

    template<std::usize N_bits, std::usize N_parts>
    auto split_bits(const std::Bits<N_bits>& controlbits) -> std::array<std::Bits<N_bits / N_parts>, N_parts>
    {
        static_assert(N_bits % N_parts == 0, "N_bits must be divisible by N_parts");

        std::array<std::Bits<N_bits / N_parts>, N_parts> result;

        for (std::size_t i = 0; i < N_parts; ++i) {
            for (std::size_t j = 0; j < N_bits / N_parts; ++j) {
                result[i][j] = controlbits[i * (N_bits / N_parts) + j];
            }
        }

        // with LSB on the left and MSB on the right
        return result;
    }

    template<std::usize N_bits, std::usize N_parts>
    auto split_array(const std::array<std::size_t, N_bits>& controlbits) -> std::array<std::array<std::size_t, N_bits / N_parts>, N_parts>
    {
        static_assert(N_bits % N_parts == 0, "N must be divisible by N_parts");

        std::array<std::array<std::size_t, N_bits / N_parts>, N_parts> result;

        for (std::size_t i = 0; i < N_parts; ++i) {
            for (std::size_t j = 0; j < N_bits / N_parts; ++j) {
                result[i][j] = controlbits[i * (N_bits / N_parts) + j];
            }
        }

        // with LSB on the left and MSB on the right
        return result;
    }

    auto Writer::write_cob_template(const std::Bits<128>& bits, \
                                    std::String reg_name, std::usize row, std::usize col) -> void
    {
        auto splitted_bits = split_bits<128, 4>(bits);
        for (std::usize i = 0; i < 4; i++)
        {
            std::String name = std::format("cob_{}_{}_{}_{}", row, col, reg_name, i);
            store_line(to_hex(splitted_bits[i]), name);
        }
    }

    auto Writer::write_tob_template64(const std::Bits<64>& bits, \
                                    std::String reg_name, std::usize row, std::usize col) -> void
    {
        auto splitted_bits = split_bits<64, 2>(bits);
        for (std::usize i = 0; i < 2; i++)
        {
            std::String name = std::format("tob_{}_{}_{}_{}", row, col, reg_name, i);
            store_line(to_hex(splitted_bits[i]), name);
        }
    }

    auto Writer::write_tob_template128(const std::Bits<128>& bits, \
                                    std::String reg_name, std::usize row, std::usize col) -> void
    {
        auto splitted_bits = split_bits<128, 4>(bits);
        for (std::usize i = 0; i < 4; i++)
        {
            std::String name = std::format("tob_{}_{}_{}_{}", row, col, reg_name, i);
            store_line(to_hex(splitted_bits[i]), name);
        }
    }

    auto Writer::write_tob_template_mux(const std::Array<std::usize, 128>& bits, \
                                        std::String reg_name, std::usize row, std::usize col) -> void
    {   
        auto splitted_bits = split_array<128, 16>(bits);
        std::Array<std::Bits<32>, 16> result {};
        for (std::usize outer_i = 0; outer_i < 16; ++outer_i)
        {
            auto& data = splitted_bits[outer_i];    
            for (std::size_t i = 0; i < data.size(); ++i) {
                std::bitset<3> bits{data[i]};       // trans decimal 0-7 to binary 000-111(MSB on the right)
                for (std::size_t j = 0; j < 3; ++j) {
                    result[outer_i][i * 3 + j] = bits[j];  // MSB on the right
                }
            }
        }                                           // for each 3-bit binary number, MSB is on the right
                                                    // and for the whole 32-bit control signal, MSB is on the right

        for (std::usize i = 0; i < 16; ++i)
        {
            std::usize bank{i/8}, bank_index{i%8};
            std::String name = std::format("tob_{}_{}_{}_bank{}_{}", row, col, reg_name, bank, bank_index);
            store_line(to_hex(result[i]), name);
        }
    }

    auto Writer::only_this_one_looks_f__king_different_from_others(const std::Bits<128>& bits,\
                                        std::String reg_name, std::usize row, std::usize col) -> void
    {
        auto splitted_bits = split_bits<128, 4>(bits);
        for (std::usize i = 0; i < 4; i++)
        {
            std::usize bank{i/2}, bank_index{i%2};
            std::String name = std::format("tob_{}_{}_{}_bank{}_en_{}", row, col, reg_name, bank, bank_index);
            store_line(to_hex(splitted_bits[i]), name);
        }
    }

    auto Writer::to_hex(const std::Bits<32>& bits) -> std::String
    {                                                   
        try{
            std::String binary {bits.to_string()};      // automatically reverse the bits in "to_string()", and MSB is on the left
            std::stringstream hexStream;
            for (std::size_t i = 0; i < binary.size(); i += 4) {
                std::string byte = binary.substr(i, 4);
                int value = std::stoi(byte, nullptr, 2); // Convert binary to decimal
                hexStream << std::hex << std::setw(1) << std::setfill('0') << value; // Convert decimal to hex
            }
            return hexStream.str();
        }
        catch(std::exception& e)
        {
            throw std::runtime_error("Writer::to_hex(): " + std::String(e.what()));
        }
    }
} 
