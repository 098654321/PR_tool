#include "controlbits_parser.hh"
#include <exception>
#include <sstream>
#include <debug/debug.hh>
#include <algorithm>

// TODO(split-output): still assumes controlbits_<mode>.txt; formal output is now
// regnamecontrolbit_4part/; readback / compare not updated yet.

namespace PR_tool::parse {


void compare(std::string file1, std::string file2) {
try{
    ControlBits bits1(file1);
    ControlBits bits2(file2);
    compare_bits(bits1, bits2);
}
catch (const std::exception& e) {
    debug::error_fmt("compare(): {}", e.what());
}
}

ControlBits::ControlBits(const std::string& file) {
    debug::info_fmt("Parsing control bits file: {}", file);

    std::ifstream bits_file(file);
    if (!bits_file.is_open()) {
        throw std::runtime_error("ControlBits::ControlBits(): failed to open file " + file);
    }
    _file_name = file;

    std::string line{};
    while (std::getline(bits_file, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string reg_name{}, value{};
        if (iss >> value >> reg_name) {
            _bits.emplace(reg_name, value);
        }
        else {
            throw std::runtime_error("ControlBits::ControlBits(): failed to parse line " + line);
        }
    }
    _lines = _bits.size();

    debug::info_fmt("Parsing finished, parsed {} lines\n", _lines);

    bits_file.close();
}

void compare_bits(const ControlBits& bits1, const ControlBits& bits2) {
    debug::info_fmt("Comparing control bits between {} and {}", bits1._lines, bits2._lines);

    std::size_t used_reg_1 = 0, used_reg_2 = 0;
    std::size_t not_used_reg_1 = 0, not_used_reg_2 = 0;
    std::size_t diff_reg = 0, same_reg = 0;
    auto is_all_zeros = [&](const std::string& str) {
        return std::all_of(str.begin(), str.end(), [](char c) { return c == '0'; });
    };

    std::unordered_map<std::string, std::string> bigger{}, smaller{};
    if (bits1._lines >= bits2._lines) {
        bigger = bits1._bits;
        smaller = bits2._bits;
    }
    else {
        bigger = bits2._bits;
        smaller = bits1._bits;
    }

    for (const auto& [reg_name_1, value_1] : bigger) {
        if (smaller.contains(reg_name_1)) {
            auto value_2 = smaller.at(reg_name_1);

            is_all_zeros(value_1) ? not_used_reg_1++ : used_reg_1++;
            is_all_zeros(value_2) ? not_used_reg_2++ : used_reg_2++;
            value_1 == value_2 ? same_reg++ : diff_reg++;
        }
        else {
            diff_reg++;
        }
    }

    debug::info_fmt("Comparing finished");
    debug::info_fmt("In file {}, {} regs are used, {} regs are not used", bits1._file_name, used_reg_1, not_used_reg_1);
    debug::info_fmt("In file {}, {} regs are used, {} regs are not used", bits2._file_name, used_reg_2, not_used_reg_2);
    debug::info_fmt("In total, {} regs are different, {} regs are same\n", diff_reg, same_reg);
}
}
