#ifndef TXT2JSON_HH
#define TXT2JSON_HH

#include <parse/reader/config/config.hh>
#include <global/std/file.hh>
#include <global/std/collection.hh>
#include <global/std/integer.hh>
#include <serde/ser.hh>
#include <serde/json/json.hh>
#include <hardware/tob/tobcoord.hh>
#include <hardware/track/trackcoord.hh>
#include <global/debug/debug.hh>


// Implement Serialize specializations locally since they are missing in headers
namespace PR_tool::serde {

    // Basic types
    template <> struct Serialize<Json, bool> {
        static void to(Json& j, const bool& v) { j = Json::boolean(v); }
    };
    template <> struct Serialize<Json, int> {
        static void to(Json& j, const int& v) { j = Json::integer(v); }
    };
    template <> struct Serialize<Json, std::usize> {
        static void to(Json& j, const std::usize& v) { j = Json::integer(static_cast<int>(v)); }
    };
    template <> struct Serialize<Json, std::i64> {
        static void to(Json& j, const std::i64& v) { j = Json::integer(static_cast<int>(v)); }
    };
    template <> struct Serialize<Json, std::String> {
        static void to(Json& j, const std::String& v) { j = Json::string(v); }
    };

    // Containers
    template <typename V>
    struct Serialize<Json, std::Vector<V>> {
        static void to(Json& j, const std::Vector<V>& v) {
            j = Json::array();
            for (const auto& item : v) {
                Json val;
                Serialize<Json, V>::to(val, item);
                j.push(val);
            }
        }
    };

    template <typename V>
    struct Serialize<Json, std::HashMap<std::String, V>> {
        static void to(Json& j, const std::HashMap<std::String, V>& map) {
            j = Json::object();
            // To ensure deterministic output order if needed, we might want to sort keys.
            // But HashMap is unordered. The requirement doesn't specify order, but JSON objects are unordered.
            // However, typical JSON serializers might not sort.
            // Let's just iterate.
            for (const auto& [key, val] : map) {
                Json json_val;
                Serialize<Json, V>::to(json_val, val);
                j.insert(key, json_val);
            }
        }
    };
    
    // Config structs
    SERIALIZE_STRUCT(PR_tool::parse::TopDieConfig,
        SER_FEILD(pin_map)
    )

    SERIALIZE_STRUCT(PR_tool::hardware::Coord,
        SER_FEILD(row)
        SER_FEILD(col)
    )
    
    // TrackDirection Enum
    template <>
    struct Serialize<Json, PR_tool::hardware::TrackDirection> {
        static void to(Json& sr, const PR_tool::hardware::TrackDirection& d) {
            if (d == PR_tool::hardware::TrackDirection::Horizontal) sr = Json::string("hori");
            else sr = Json::string("vert");
        }
    };

    SERIALIZE_STRUCT(PR_tool::hardware::TrackCoord,
        SER_FEILD(row)
        SER_FEILD(col)
        SER_FEILD(dir)
        SER_FEILD(index)
    )

    SERIALIZE_STRUCT(PR_tool::parse::TopdieInstConfig,
        SER_FEILD(topdie)
        SER_FEILD(coord)
    )

    SERIALIZE_STRUCT(PR_tool::parse::ExternalPortConfig,
        SER_FEILD(coord)
    )
}

namespace PR_tool::parse {

    void txt2json(const std::FilePath& config_folder, const std::FilePath& json_folder, int mode, bool try_all_modes) {
        auto config = load_config(config_folder, mode, try_all_modes);
        
        // 1. topdies.json
        debug::info("load topdies.json");
        {
            PR_tool::serde::Json j;
            PR_tool::serde::Serialize<PR_tool::serde::Json, decltype(config.topdies)>::to(j, config.topdies);
            std::OutFile file(json_folder / "topdies.json");
            file << j.to_string();
        }

        // 2. topdie_insts.json
        debug::info("load topdie_insts.json");
        {
            PR_tool::serde::Json j;
            PR_tool::serde::Serialize<PR_tool::serde::Json, decltype(config.topdie_insts)>::to(j, config.topdie_insts);
            std::OutFile file(json_folder / "topdie_insts.json");
            file << j.to_string();
        }

        // 3. external_ports.json
        debug::info("load external_ports.json");
        {
            PR_tool::serde::Json j;
            PR_tool::serde::Serialize<PR_tool::serde::Json, decltype(config.external_ports)>::to(j, config.external_ports);
            std::OutFile file(json_folder / "external_ports.json");
            file << j.to_string();
        }

        // 4. connections.json
        debug::info("load connections.json");
        {
            PR_tool::serde::Json j = PR_tool::serde::Json::object();
            
            for(const auto& [mode, inner_map] : config.connections) {
                 for(const auto& [group_index, vec] : inner_map) {
                     PR_tool::serde::Json arr = PR_tool::serde::Json::array();
                     for(const auto& conn : vec) {
                         PR_tool::serde::Json pair = PR_tool::serde::Json::array();
                         pair.push(PR_tool::serde::Json::string(conn.input));
                         pair.push(PR_tool::serde::Json::string(conn.output));
                         arr.push(pair);
                     }
                     j.insert(std::to_string(group_index), arr);
                 }
            }
            std::OutFile file(json_folder / "connections.json");
            file << j.to_string();
        }

    }

}


int main() {
    PR_tool::debug::initial_log("./debug.log");

    for (int i = 8; i <= 15; i++) {
        std::FilePath config_folder = "/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/excel_2_txt/config";
        std::FilePath json_folder = "/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/excel_2_txt/config/json";
        PR_tool::parse::txt2json(config_folder, json_folder, 0, false);
    }
    
    return 0;
}

#endif  // TXT2JSON_HH
