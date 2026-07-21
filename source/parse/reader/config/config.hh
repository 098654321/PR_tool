#pragma once

#include "./interposer.hh"
#include "./topdie.hh"
#include "./externalport.hh"
#include "./connection.hh"

#include <std/collection.hh>
#include <std/string.hh>
#include <std/file.hh>

namespace PR_tool::parse {

    using RegisterAddress = std::String;
    using RegisterFileMap = std::HashMap<std::String, RegisterAddress>;      // reg_name -> address
    using RegisterMapConfig = std::HashMap<std::String, RegisterFileMap>;     // filename -> regs

    struct Config {
        InterposerConfig interposer;
        std::HashMap<std::String, TopDieConfig> topdies;
        std::HashMap<std::String, TopdieInstConfig> topdie_insts;
        std::HashMap<std::String, ExternalPortConfig> external_ports;
        std::HashMap<int, std::HashMap<int, std::Vector<ConnectionConfig>>> connections;
        std::HashMap<std::String, std::HashMap<std::String, hardware::TrackCoord>> ports_01;
        RegisterMapConfig register_map;
    };

    auto load_config(const std::FilePath& config_folder, int mode, bool try_all_modes) -> Config;

}
