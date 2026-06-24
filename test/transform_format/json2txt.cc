#include <parse/reader/config/config.hh>
#include <global/std/file.hh>
#include <global/std/collection.hh>
#include <global/std/utility.hh>
#include <global/debug/debug.hh>
#include <utility/string.hh>
#include <serde/json/json.hh>
#include <hardware/interposer.hh>
#include <hardware/track/trackcoord.hh>

#include <algorithm>
#include <format>
#include <iostream>
#include <cctype>

namespace PR_tool::parse {

    namespace {

        struct TxtConnection {
            std::String topdie1;
            std::String topdie2;
            std::Array<int, 5> input{};
            std::Array<int, 5> output{};
            int net_tag{};
            int order{};
        };

        constexpr std::Array<int, 16> kExternIndices = {
            0, 9, 18, 27, 36, 45, 54, 63, 120, 113, 106, 99, 92, 85, 78, 71
        };

        static auto is_integer_token(const std::StringView& token) -> bool {
            if (token.empty()) {
                return false;
            }
            std::usize i = 0;
            if (token[0] == '-' || token[0] == '+') {
                if (token.size() == 1) {
                    return false;
                }
                i = 1;
            }
            for (; i < token.size(); ++i) {
                if (!std::isdigit(static_cast<unsigned char>(token[i]))) {
                    return false;
                }
            }
            return true;
        }

        static auto is_coordinate_encoded_pin(const std::String& pin_name) -> bool {
            auto parts = utility::split(pin_name, '_');
            if (parts.size() != 3) {
                return false;
            }
            return is_integer_token(parts[0]) && is_integer_token(parts[1]) && is_integer_token(parts[2]);
        }

        static auto numbers_from_index(int row, int col, int index) -> std::Array<int, 5> {
            return {0, row, col, index % 8, index / 8};
        }

        static auto parse_coordinate_encoded_pin(const std::String& pin_name) -> std::Array<int, 5> {
            auto parts = utility::split(pin_name, '_');
            const int row = std::stoi(std::String{parts[0]});
            const int col = std::stoi(std::String{parts[1]});
            const int index = std::stoi(std::String{parts[2]});
            return numbers_from_index(row, col, index);
        }

        static auto find_extern_slot_index(std::usize track_index) -> int {
            for (std::usize i = 0; i < kExternIndices.size(); ++i) {
                if (kExternIndices[i] == static_cast<int>(track_index)) {
                    return static_cast<int>(i);
                }
            }
            debug::exception_fmt("Unknown track index '{}' for external port", track_index);
        }

        static auto trackcoord_to_numbers(const hardware::TrackCoord& coord) -> std::Array<int, 5> {
            const int dir = coord.dir == hardware::TrackDirection::Vertical ? 0 : 1;
            const int row = hardware::Interposer::COB_ARRAY_HEIGHT - static_cast<int>(coord.row);
            const int col = static_cast<int>(coord.col);
            const int extern_slot = find_extern_slot_index(coord.index);
            return {dir, row, col, -3, extern_slot};
        }

        static auto parse_logical_inst_pin(
            const std::String& inst_name,
            const std::String& pin_name,
            const Config& config
        ) -> std::Array<int, 5> {
            auto inst_iter = config.topdie_insts.find(inst_name);
            if (inst_iter == config.topdie_insts.end()) {
                debug::exception_fmt("Unknown topdie inst '{}' for pin '{}.{}'", inst_name, inst_name, pin_name);
            }

            const auto& topdie_name = inst_iter->second.topdie;
            auto topdie_iter = config.topdies.find(topdie_name);
            if (topdie_iter == config.topdies.end()) {
                debug::exception_fmt("Unknown topdie '{}' for inst '{}'", topdie_name, inst_name);
            }

            auto pin_iter = topdie_iter->second.pin_map.find(pin_name);
            if (pin_iter == topdie_iter->second.pin_map.end()) {
                debug::exception_fmt("Unknown pin '{}' in topdie '{}'", pin_name, topdie_name);
            }

            const int row = hardware::Interposer::COB_ARRAY_HEIGHT
                - 2 * static_cast<int>(inst_iter->second.coord.row) - 1;
            const int col = 3 * static_cast<int>(inst_iter->second.coord.col);
            return numbers_from_index(row, col, pin_iter->second);
        }

        static auto parse_inst_pin(const std::String& pin, const Config& config) -> std::Array<int, 5> {
            auto dot = pin.find('.');
            if (dot == std::String::npos) {
                debug::exception_fmt("Invalid topdie inst pin '{}'", pin);
            }
            const auto inst_name = pin.substr(0, dot);
            const auto pin_name = pin.substr(dot + 1);

            if (is_coordinate_encoded_pin(pin_name)) {
                return parse_coordinate_encoded_pin(pin_name);
            }
            return parse_logical_inst_pin(inst_name, pin_name, config);
        }

        static auto parse_external_pin(const std::String& pin, const Config& config) -> std::Array<int, 5> {
            auto iter = config.external_ports.find(pin);
            if (iter == config.external_ports.end()) {
                debug::exception_fmt("Unknown external port '{}'", pin);
            }
            return trackcoord_to_numbers(iter->second.coord);
        }

        static auto pin_to_numbers(const std::String& pin, const Config& config) -> std::Array<int, 5> {
            // Mirror parse_txt_line in config.cc: -1 -> nege, -2 -> pose
            if (pin == "nege") {
                return {0, 0, 0, -1, 0};
            }
            if (pin == "pose") {
                return {0, 0, 0, -2, 0};
            }
            if (pin.find('.') == std::String::npos) {
                return parse_external_pin(pin, config);
            }
            return parse_inst_pin(pin, config);
        }

        static auto infer_external_topdie_type(const std::String& pin, const Config& config) -> std::String {
            if (pin.starts_with("IO_")) {
                return "extIO";
            }
            for (const auto& [topdie_name, _] : config.topdies) {
                if (pin == topdie_name || pin.starts_with(topdie_name + "_")) {
                    return topdie_name;
                }
            }
            const auto underscore = pin.find('_');
            if (underscore != std::String::npos) {
                return pin.substr(0, underscore);
            }
            return "extIO";
        }

        static auto pin_topdie_type(const std::String& pin, const Config& config) -> std::String {
            if (pin == "pose" || pin == "nege") {
                return "0/1";
            }
            auto dot = pin.find('.');
            if (dot == std::String::npos) {
                return infer_external_topdie_type(pin, config);
            }
            const auto inst_name = pin.substr(0, dot);
            auto iter = config.topdie_insts.find(inst_name);
            if (iter == config.topdie_insts.end()) {
                debug::exception_fmt("Unknown topdie inst '{}' in pin '{}'", inst_name, pin);
            }
            return iter->second.topdie;
        }

        static auto section_priority(const std::String& topdie1, const std::String& topdie2) -> int {
            static const std::Vector<std::Pair<std::String, std::String>> kSectionOrder = {
                {"cpu", "mem"},
                {"mem", "cpu"},
                {"mem", "mem"},
                {"cpu", "AI"},
                {"extIO", "cpu"},
                {"extIO", "AI"},
                {"extIO", "mem"},
                {"0/1", "cpu"},
                {"0/1", "AI"},
                {"0/1", "mem"},
            };
            for (std::usize i = 0; i < kSectionOrder.size(); ++i) {
                if (kSectionOrder[i].first == topdie1 && kSectionOrder[i].second == topdie2) {
                    return static_cast<int>(i);
                }
            }
            return 1000;
        }

        static auto format_node(const std::Array<int, 5>& node) -> std::String {
            return std::format("{} {} {} {} {}", node[0], node[1], node[2], node[3], node[4]);
        }

        static auto format_txt_line(const TxtConnection& conn) -> std::String {
            constexpr std::usize kInputWidth = 16;
            const auto input_str = format_node(conn.input);
            const auto output_str = format_node(conn.output);
            const auto padding = (input_str.size() < kInputWidth) ? std::String(kInputWidth - input_str.size(), ' ') : " ";
            return input_str + padding + output_str + " " + std::to_string(conn.net_tag);
        }

        static auto collect_connections(const Config& config, int mode) -> std::Vector<TxtConnection> {
            std::Vector<TxtConnection> result;
            int order = 0;

            auto mode_iter = config.connections.find(mode);
            if (mode_iter == config.connections.end()) {
                debug::exception_fmt("No connections found for mode {}", mode);
            }

            std::Vector<int> sync_tags;
            sync_tags.reserve(mode_iter->second.size());
            for (const auto& [sync, _] : mode_iter->second) {
                sync_tags.push_back(sync);
            }
            std::sort(sync_tags.begin(), sync_tags.end());

            for (const int sync : sync_tags) {
                const auto& connections = mode_iter->second.at(sync);
                for (const auto& conn : connections) {
                    TxtConnection entry;
                    entry.topdie1 = pin_topdie_type(conn.input, config);
                    entry.topdie2 = pin_topdie_type(conn.output, config);
                    entry.input = pin_to_numbers(conn.input, config);
                    entry.output = pin_to_numbers(conn.output, config);
                    entry.net_tag = sync;
                    entry.order = order++;
                    result.push_back(std::move(entry));
                }
            }

            return result;
        }

        using SectionKey = std::String;

        static auto make_section_key(const std::String& topdie1, const std::String& topdie2) -> SectionKey {
            return topdie1 + "\t" + topdie2;
        }

        static auto parse_section_key(const SectionKey& key) -> std::Pair<std::String, std::String> {
            auto tab = key.find('\t');
            return {key.substr(0, tab), key.substr(tab + 1)};
        }

    }  // namespace

    void json2txt(const std::FilePath& config_folder, const std::FilePath& txt_path, int mode) {
        auto config = load_config(config_folder, mode, false);
        auto connections = collect_connections(config, mode);

        std::HashMap<SectionKey, std::Vector<TxtConnection>> sections;

        for (auto& conn : connections) {
            sections[make_section_key(conn.topdie1, conn.topdie2)].push_back(std::move(conn));
        }

        std::Vector<SectionKey> section_keys;
        section_keys.reserve(sections.size());
        for (const auto& [key, _] : sections) {
            section_keys.push_back(key);
        }
        std::sort(section_keys.begin(), section_keys.end(), [](const SectionKey& a, const SectionKey& b) {
            auto [a1, a2] = parse_section_key(a);
            auto [b1, b2] = parse_section_key(b);
            const int pa = section_priority(a1, a2);
            const int pb = section_priority(b1, b2);
            if (pa != pb) {
                return pa < pb;
            }
            if (a1 != b1) {
                return a1 < b1;
            }
            return a2 < b2;
        });

        std::OutFile file(txt_path);
        for (std::usize i = 0; i < section_keys.size(); ++i) {
            const auto& key = section_keys[i];
            auto [topdie1, topdie2] = parse_section_key(key);
            auto& lines = sections.at(key);

            std::sort(lines.begin(), lines.end(), [](const TxtConnection& a, const TxtConnection& b) {
                if (a.net_tag != b.net_tag) {
                    return a.net_tag < b.net_tag;
                }
                return a.order < b.order;
            });

            file << "# " << topdie1 << " " << topdie2 << "\n";
            for (const auto& line : lines) {
                file << format_txt_line(line) << "\n";
            }
        }
    }

}  // namespace PR_tool::parse

namespace {

    auto print_help(const char* prog) -> void {
        std::cerr << "Usage: " << prog << " <config_folder> -o <output_dir> [OPTIONS]\n\n";
        std::cerr << "Options:\n";
        std::cerr << "  -o, --output <dir>   Output directory (required)\n";
        std::cerr << "  -n, --name <file>    Output filename (default: inferred from config.json)\n";
        std::cerr << "  -h, --help           Print help\n";
    }

    auto infer_output_name(const std::FilePath& config_folder) -> std::String {
        const auto json = PR_tool::serde::Json::load_from(config_folder / "config.json");
        const auto& obj = json.as_object();
        const auto iter = obj.find("connections");
        if (iter == obj.end()) {
            PR_tool::debug::exception_fmt("Missing 'connections' in config.json");
        }

        const std::FilePath connections_path{std::String{iter->second.as_string()}};
        if (connections_path.extension() == ".txt") {
            return connections_path.filename().string();
        }
        return connections_path.stem().string() + ".txt";
    }

}  // namespace

int main(int argc, char** argv) {
    PR_tool::debug::initial_log("./debug.log");

    std::FilePath config_folder;
    std::FilePath output_dir;
    std::String output_name;
    bool show_help = false;

    if (argc < 2) {
        print_help(argv[0]);
        return 1;
    }

    for (int i = 1; i < argc; ++i) {
        const std::String arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            show_help = true;
        } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
            output_dir = argv[++i];
        } else if ((arg == "-n" || arg == "--name") && i + 1 < argc) {
            output_name = argv[++i];
        } else if (arg.starts_with("-")) {
            std::cerr << "Unknown option: " << arg << "\n";
            print_help(argv[0]);
            return 1;
        } else if (config_folder.empty()) {
            config_folder = arg;
        } else {
            std::cerr << "Unexpected argument: " << arg << "\n";
            print_help(argv[0]);
            return 1;
        }
    }

    if (show_help) {
        print_help(argv[0]);
        return 0;
    }

    if (config_folder.empty() || output_dir.empty()) {
        std::cerr << "Error: <config_folder> and -o <output_dir> are required.\n\n";
        print_help(argv[0]);
        return 1;
    }

    if (output_name.empty()) {
        output_name = infer_output_name(config_folder);
    }

    std::filesystem::create_directories(output_dir);
    const auto txt_path = output_dir / output_name;

    PR_tool::parse::json2txt(config_folder, txt_path, 0);
    PR_tool::debug::info_fmt("Wrote {}", txt_path.string());
    return 0;
}
