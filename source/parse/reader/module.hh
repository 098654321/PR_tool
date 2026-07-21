#pragma once

#include "std/file.hh"
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>
#include <std/utility.hh>
#include <parse/reader/config/config.hh>

#include <hardware/interposer.hh>


namespace PR_tool::parse {

    auto read_config(const std::FilePath& config_folder, int mode, bool try_all_modes) 
        -> std::Tuple<std::Box<hardware::Interposer>, std::Box<circuit::BaseDie>, RegisterMapConfig>;

    auto read_config(
        const std::FilePath& config_folder,
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        int mode,
        bool try_all_modes
    ) -> RegisterMapConfig;

    // return <bool1, bool2>
    // bool1 is true if the controlbis_<mode>.txt exists
    // bool2 is true if bool1 is false and any controlbits_<other_mode>.txt exists
    auto read_controlbits(
        const std::FilePath& config_folder,
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        int mode,
        bool try_all_modes
    ) -> std::Pair<bool, bool>;

}
