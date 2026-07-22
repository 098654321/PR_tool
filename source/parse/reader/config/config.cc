
#include "./config.hh"
#include "./interposer.hh"
#include "./topdie.hh"
#include "./externalport.hh"
#include "./connection.hh"

#include <hardware/interposer.hh>
#include <std/collection.hh>
#include <std/file.hh>
#include <debug/debug.hh>
#include <utility/string.hh>
#include <serde/de.hh>
#include <serde/json/json.hh>
// #include <xlnt/xlnt.hpp>

namespace PR_tool::parse {

    struct ConfigFilepaths {
        std::FilePath interposer;
        std::FilePath topdies;
        std::FilePath topdie_insts;
        std::FilePath external_ports;
        std::FilePath connections;
        std::FilePath ports_01;
        std::Option<std::FilePath> reigster_adder;  // optional
    };

}

DESERIALIZE_STRUCT(PR_tool::parse::ConfigFilepaths,
    DE_FILED(interposer)
    DE_FILED(topdies)
    DE_FILED(topdie_insts)
    DE_FILED(external_ports)
    DE_FILED(connections)
    DE_FILED(ports_01)
    DE_OPTION_FILED(reigster_adder)
)

template <class ConnectionConfig>
struct PR_tool::serde::Deserialize<PR_tool::serde::Json, std::HashMap<int, std::Vector<ConnectionConfig>>> {
    static void from(const Json& json, std::HashMap<int, std::Vector<ConnectionConfig>>& value){
        auto& map = json.as_object();
        for (auto& [key, j]: map){
            std::Vector<std::Vector<std::String>> vec {};
            PR_tool::serde::Deserialize<PR_tool::serde::Json, std::Vector<std::Vector<std::String>>>::from(j, vec);

            std::Vector<ConnectionConfig> nets {};
            for (std::usize i = 0; i < vec.size(); ++i) {
                auto& net = vec[i];
                if (net.size() != 2) {
                    throw std::runtime_error(std::format(
                        "Connection pair under sync key '{}' at index {} must have exactly 2 endpoints, got {}",
                        key, i, net.size()));
                }
                nets.emplace_back(ConnectionConfig{net[0], net[1]});
            }
            value.emplace(std::stoi(key), nets);
        }
    }
};

template <class ConnectionConfig>
struct PR_tool::serde::Deserialize<PR_tool::serde::Json, std::HashMap<int, std::HashMap<int, std::Vector<ConnectionConfig>>>> {
    static void from(const Json& json, std::HashMap<int, std::HashMap<int, std::Vector<ConnectionConfig>>>& value){
        auto& map = json.as_object();
        for (auto& [key, j]: map){
            std::HashMap<int, std::Vector<ConnectionConfig>> value_v {};
            PR_tool::serde::Deserialize<PR_tool::serde::Json, std::HashMap<int, std::Vector<ConnectionConfig>>>::from(j, value_v);
            value.emplace(std::stoi(key), value_v);
        }
    }
};

namespace PR_tool::parse {

    static auto load_interposer_config(const std::FilePath& path, InterposerConfig& config) -> void;
    static auto load_topdies_config(const std::FilePath& path, std::HashMap<std::String, TopDieConfig>& topdies) -> void;
    static auto load_topdie_insts_config(const std::FilePath& path, std::HashMap<std::String, TopdieInstConfig>& topdie_insts) -> void;
    static auto load_external_ports_config(const std::FilePath& path, std::HashMap<std::String, ExternalPortConfig>& exports) -> void;
    static auto load_connections_config(const std::FilePath& path, std::HashMap<int, std::HashMap<int, std::Vector<ConnectionConfig>>>& connections, int mode, bool try_all_modes) -> void;
    static auto load_ports_01_config(const std::FilePath& path, std::HashMap<std::String, std::HashMap<std::String, hardware::TrackCoord>>& ports_01) -> void;
    static auto load_register_map_config(const std::FilePath& path, RegisterMapConfig& register_map) -> void;

    static auto load_from_txt(const std::FilePath& path, Config& config, int mode, bool try_all_modes) -> void;
    static auto parse_txt_line(const std::String& topdie1, const std::String& topdie2, const std::Array<int, 11>& numbers, Config& config, int mode, bool try_all_modes) -> void;

    auto load_config(const std::FilePath& config_folder, int mode, bool try_all_modes) -> Config 
    try {
        auto config_paths = ConfigFilepaths{};
        serde::deserialize(serde::Json::load_from(config_folder / "config.json"), config_paths);    // deserialize and store in config_paths

        auto config = Config {};

        load_ports_01_config(config_folder / config_paths.ports_01, config.ports_01);
        if (config_paths.connections.filename().extension().string() == ".json"){
            load_interposer_config(config_folder / config_paths.interposer, config.interposer);
            load_topdies_config(config_folder / config_paths.topdies, config.topdies);
            load_topdie_insts_config(config_folder / config_paths.topdie_insts, config.topdie_insts);
            load_external_ports_config(config_folder / config_paths.external_ports, config.external_ports);
            load_connections_config(config_folder / config_paths.connections, config.connections, mode, try_all_modes);
        }
        else if(config_paths.connections.filename().extension().string() == ".txt"){
            load_from_txt(config_folder / config_paths.connections, config, mode, try_all_modes);
        }
        else{
            debug::exception_fmt("Unspport extension '{}' for connections config", config_paths.connections.filename().extension().string());
        }

        if (config_paths.reigster_adder) {
            load_register_map_config(config_folder / *config_paths.reigster_adder, config.register_map);
        }

        return config;
    } 
    THROW_UP_WITH("Load config")

    static auto load_interposer_config(const std::FilePath& path, InterposerConfig& config) -> void {
        debug::info("Load interposer config");

        // Only support .json
        serde::deserialize(serde::Json::load_from(path), config);   
    }

    static auto load_topdies_config(const std::FilePath& path, std::HashMap<std::String, TopDieConfig>& topdies) -> void 
    try {
        debug::info("Load topdies config");

        auto extension = path.filename().extension().string();
        if (extension == ".json") {
            serde::deserialize(serde::Json::load_from(path), topdies);
        } else if (extension == ".xlsx") {
            debug::unimplement("Load topdie excel");
        } else {
            debug::exception_fmt("Unspport extension '{}' for topdies config", extension);
        }
    } 
    THROW_UP_WITH("Load topdies config")

    static auto load_topdie_insts_config(const std::FilePath& path, std::HashMap<std::String, TopdieInstConfig>& topdie_insts) -> void 
    try {
        debug::info("Load topdir insts config");

        auto extension = path.filename().extension().string();
        if (extension == ".json") {
            serde::deserialize(serde::Json::load_from(path), topdie_insts);
        } else if (extension == ".xlsx") {
            debug::unimplement("Load topdie insts config by excel");
        } else {
            debug::exception_fmt("Unspport extension '{}' for topdie insts config", extension);
        }
    } 
    THROW_UP_WITH("Load topdie insts config")

    static auto load_external_ports_config(const std::FilePath& path, std::HashMap<std::String, ExternalPortConfig>& exports) -> void 
    try {
        debug::info("Load external ports config");
        
        auto extension = path.filename().extension().string();
        if (extension == ".json") {
            serde::deserialize(serde::Json::load_from(path), exports);
        } else if (extension == ".xlsx") {
            debug::unimplement("Load external ports excel");
        } else {
            debug::exception_fmt("Unspport extension '{}' for external ports config", extension);
        }
    }
    THROW_UP_WITH("Load external ports config")

    static auto load_connections_config(const std::FilePath& path, std::HashMap<int, std::HashMap<int, std::Vector<ConnectionConfig>>>& connections, int mode, bool try_all_modes) -> void 
    try {
        debug::info("Load connections config");

        auto extension = path.filename().extension().string();
        if (extension == ".json") {
            if (mode == 0 && !try_all_modes) {
                // simple routing config
                std::HashMap<int, std::Vector<ConnectionConfig>> inner_connections {};
                serde::deserialize(serde::Json::load_from(path), inner_connections);
                connections.emplace(0, inner_connections);
            }
            else {
                // incremental routing config
                serde::deserialize(serde::Json::load_from(path), connections);
            }
        } 
        // else if (extension == ".xlsx") {
        //     auto workbook = xlnt::workbook();
        //     workbook.load(path);

        //     if (mode == 0) {
        //         std::HashMap<int, std::Vector<ConnectionConfig>> inner_connections {};
        //         for (const auto& row : workbook.sheet_by_index(0).rows()) {
        //             // | input  |  output  | syns |
        //             if (row.length() != 3) {
        //                 debug::exception_fmt("Except '3' columns but got '{}'", row.length());
        //             }
    
        //             auto input = row[0].to_string();
        //             if (input.empty()) {
        //                 continue;
        //             }
        //             auto output = row[1].to_string();
        //             auto sync = utility::string_to_i32(row[2].to_string());
    
        //             auto result = inner_connections.emplace(sync, std::Vector<ConnectionConfig>{});
        //             auto& iter = result.first;
        //             assert(iter->first == sync);
        //             auto& v = iter->second.emplace_back(std::move(input), std::move(output));
        //         }
        //         connections.emplace(0, inner_connections);
        //     }
        //     else {
        //         debug::unimplement("do not support the incremental mode config temperorily");
        //     }
        // } 
        else {
            debug::exception_fmt("Unspport extension '{}' for connections config", extension);
        }
    } 
    THROW_UP_WITH("Load connections config")

    static auto load_from_txt(const std::FilePath& path, Config& config, int mode, bool try_all_modes) -> void 
    try {
        if (try_all_modes) {
            debug::unimplement("load from all modes is not supported when loading from .txt file");
        }

        debug::info_fmt("Load from txt with mode = {}", mode);
        config.connections.emplace(mode, std::HashMap<int, std::Vector<ConnectionConfig>>{});

        std::ifstream file(path);
        if (!file.is_open()){
            debug::exception_fmt("Cannot open file '{}'", path.string());
        }

        std::string line, topdie_name1, topdie_name2;
        while (std::getline(file, line)) {
            // skip empty line
            if (line.find_first_not_of(" \t") == std::string::npos) {
                continue;  
            }

            if (line[0] == '#') {
                std::istringstream iss(line.substr(1));  
                if (iss >> topdie_name1 >> topdie_name2) {  
                    ;
                } 
                else {
                    throw std::runtime_error(std::format("Invalie line: '{}'", line));
                }

                continue;  
            }

            std::Array<int, 11> numbers{};
            std::stringstream ss(line);
            int num = 0;
            int pos = 0;
            while (ss >> num) {
                if (pos >= 11) {
                    throw std::runtime_error(std::format(
                        "TXT connections line has more than 11 integers: '{}'", line));
                }
                numbers[static_cast<std::usize>(pos++)] = num;
            }
            if (pos != 11) {
                throw std::runtime_error(std::format(
                    "TXT connections line must have exactly 11 integers (got {}): '{}'",
                    pos, line));
            }

            parse_txt_line(topdie_name1, topdie_name2, numbers, config, mode, try_all_modes);
        }

       file.close();
    }
    THROW_UP_WITH("Load from txt")

    auto parse_txt_line(const std::String& topdie1, const std::String& topdie2, const std::Array<int, 11>& numbers, Config& config, int mode, bool try_all_modes) -> void
    try{
        if (try_all_modes) {
            debug::unimplement("load from all modes is not supported when parsing from .txt file");
        }

        const auto net_tag = numbers.back();
        std::String input{}, output{};

        auto parse_node = [&config](const std::Array<int, 5>& info, std::String& node, const std::String& topdie){
            std::Vector<int> externs { 0,9,18,27,36,45,54,63,120,113,106,99,92,85,78,71 };
            if (topdie != "cpu" && topdie != "AI" && topdie != "mem" && topdie != "extIO" && topdie != "0/1"){
                debug::exception_fmt("Invalid topdie_name '{}'", topdie);
            }
            switch (info[3]){
                case -1: node = "nege"; 
                    break;
                case -2: node = "pose";
                    break;
                case -3: {
                    if (info[4] < 0 || static_cast<std::usize>(info[4]) >= externs.size()) {
                        throw std::runtime_error(std::format(
                            "TXT external port index out of range: {}", info[4]));
                    }
                    auto trackcoord = info[0] == 0?\
                        hardware::TrackCoord(hardware::Interposer::COB_ARRAY_HEIGHT-info[1], info[2], hardware::TrackDirection::Vertical, externs[info[4]]):\
                        hardware::TrackCoord(hardware::Interposer::COB_ARRAY_HEIGHT-info[1], info[2], hardware::TrackDirection::Horizontal, externs[info[4]]);
                    node = std::String{
                        "IO_"+std::to_string(info[0])+"_"+std::to_string(info[1])+\
                        "_"+std::to_string(info[2])+"_"+std::to_string(info[4])
                    };
                    if (!config.external_ports.contains(node)){
                        config.external_ports.emplace(node, ExternalPortConfig{trackcoord});
                    }
                    break;
                }
                default:{
                    auto index = info[3] + 8 * info[4];
                    auto node_postfix = std::String{std::to_string(info[1]) + "_" + std::to_string(info[2]) + "_" + std::to_string(index)};
                    if(!config.topdies.contains(topdie)){
                        config.topdies.emplace(topdie, TopDieConfig{});
                    }
                    if (!config.topdies.at(topdie).pin_map.contains(node_postfix)){
                        config.topdies.at(topdie).pin_map.emplace(node_postfix, index);
                    }

                    std::String topdie_inst {
                        "Topdie_inst_" + std::to_string(info[1]) + "_" + std::to_string(info[2])
                    };
                    node = topdie_inst + "." + node_postfix;
                    auto tobcoord = hardware::TOBCoord((hardware::Interposer::COB_ARRAY_HEIGHT-info[1])/2, info[2]/3);
                    if (!config.topdie_insts.contains(topdie_inst)){
                        config.topdie_insts.emplace(topdie_inst, TopdieInstConfig{topdie, tobcoord});
                    }
                    break;
                }
            }
        };

        parse_node({numbers[0], numbers[1], numbers[2], numbers[3], numbers[4]}, input, topdie1);
        parse_node({numbers[5], numbers[6], numbers[7], numbers[8], numbers[9]}, output, topdie2);

        auto& inner_connection = config.connections.at(mode);
        if (!inner_connection.contains(net_tag)){
            inner_connection.emplace(net_tag, std::Vector<ConnectionConfig>{});
        }
        inner_connection.at(net_tag).emplace_back(ConnectionConfig{input, output});
    }
    THROW_UP_WITH("Parse txt line")

    auto load_ports_01_config(const std::FilePath& path, std::HashMap<std::String, std::HashMap<std::String, hardware::TrackCoord>>& ports_01) -> void
    try {
        debug::info("Load 0/1 ports config");
        
        auto extension = path.filename().extension().string();
        if (extension == ".json") {
            serde::deserialize(serde::Json::load_from(path), ports_01);
        } else if (extension == ".xlsx") {
            debug::unimplement("Load 0/1 ports excel");
        } else {
            debug::exception_fmt("Unspport extension '{}' for 0/1 ports config", extension);
        }
    }
    THROW_UP_WITH("Load 0/1 ports config")

    static auto load_register_map_config(const std::FilePath& path, RegisterMapConfig& register_map) -> void
    try {
        debug::info("Load register map config");

        serde::deserialize(serde::Json::load_from(path), register_map);

        static const std::Array<std::String, 4> expected_files = {
            "botleft_REG0.txt",
            "botright_REG1.txt",
            "topleft_REG2.txt",
            "topright_REG3.txt",
        };

        for (const auto& filename : expected_files) {
            if (!register_map.contains(filename)) {
                debug::fatal_fmt(
                    "Register map '{}' missing required file key '{}'",
                    path.string(),
                    filename
                );
            }
        }

        std::HashSet<std::String> seen_names {};
        for (const auto& [filename, regs] : register_map) {
            for (const auto& [reg_name, _address] : regs) {
                if (seen_names.contains(reg_name)) {
                    debug::fatal_fmt(
                        "Register map '{}' has duplicate reg_name '{}' (seen in '{}')",
                        path.string(),
                        reg_name,
                        filename
                    );
                }
                seen_names.insert(reg_name);
            }
        }
    }
    THROW_UP_WITH("Load register map config")

}