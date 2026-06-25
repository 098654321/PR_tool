#pragma once

#include "std/file.hh"

namespace PR_tool::hardware {
    class Interposer;
}

namespace PR_tool::circuit {
    class BaseDie;
}

namespace PR_tool::parse {

    auto output_from_routing_results(hardware::Interposer* interposer, const std::FilePath& output_path, circuit::BaseDie* basedie, int mode, bool try_all_modes, bool simplify_controlbits = false) -> void;
    auto write_control_bits(hardware::Interposer* interposer, const std::FilePath& output_path, int mode, bool simplify_controlbits = false) -> void;
    auto connect_registers(hardware::Interposer* interposer, circuit::BaseDie* basedie, int mode) -> void;

}