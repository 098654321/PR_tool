#pragma once

#include "std/file.hh"
#include <parse/reader/config/config.hh>

namespace PR_tool::hardware {
    class Interposer;
}

namespace PR_tool::circuit {
    class BaseDie;
}

namespace PR_tool::parse {

    auto output_from_routing_results(
        hardware::Interposer* interposer,
        const std::FilePath& output_path,
        circuit::BaseDie* basedie,
        int mode,
        bool try_all_modes,
        bool simplify_controlbits,
        const RegisterMapConfig& register_map
    ) -> void;

    auto write_control_bits(
        hardware::Interposer* interposer,
        const std::FilePath& output_path,
        int mode,
        bool simplify_controlbits,
        const RegisterMapConfig& register_map
    ) -> void;

    auto write_control_bits_pair(
        hardware::Interposer* interposer,
        const std::FilePath& full_output_path,
        const std::FilePath& simplified_output_path,
        int mode,
        const RegisterMapConfig& register_map
    ) -> void;

    auto connect_registers(hardware::Interposer* interposer, circuit::BaseDie* basedie, int mode) -> void;

}
