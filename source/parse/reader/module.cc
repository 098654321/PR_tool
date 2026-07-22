#include "./module.hh"
#include "./config/config.hh"
#include "./reader.hh"
#include "circuit/basedie.hh"
#include <parse/reader/controlbits/controlbits.hh>
#include "hardware/interposer.hh"
#include <algorithm>

#include <debug/debug.hh>
#include <memory>

namespace PR_tool::parse {

    auto read_config(const std::FilePath& config_folder, int mode, bool try_all_modes)
        -> std::Tuple<std::Box<hardware::Interposer>, std::Box<circuit::BaseDie>, RegisterMapConfig> 
    {
        debug::info_fmt("Read config in '{}'", config_folder.string());

        auto interposer = std::make_unique<hardware::Interposer>();
        auto basedie = std::make_unique<circuit::BaseDie>();

        auto map = read_config(config_folder, interposer.get(), basedie.get(), mode, try_all_modes);
        
        debug::info("Read config done.");
        return {std::move(interposer), std::move(basedie), std::move(map)};
    }

    auto read_config(
        const std::FilePath& config_folder,
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        int mode,
        bool try_all_modes
    ) -> RegisterMapConfig
    {
        auto config = load_config(config_folder, mode, try_all_modes);
        auto reader = Reader{config, interposer, basedie};
        reader.build();
        return std::move(config.register_map);
    }

    // TODO: 改一下返回值，需要能够判断是否需要做增量布线，以及如果要做的情况下是否读入了 controlbits
    // TODO(split-output): still assumes controlbits_<mode>.txt; formal output is now
    // regnamecontrolbit_4part/; readback / compare not updated yet.
    // CLI v1.0.0 does not call read_controlbits for skip-route (always re-routes mode 0).
    auto read_controlbits(
        const std::FilePath& config_folder,
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        int mode,
        bool try_all_modes
    ) -> std::Pair<bool, bool> {
        if (try_all_modes) {                // need to consider all the connections in all the modes, not a single mode
            return {false, false};
        }

        debug::info("Load controlbits ...");
        
        auto controlbits = load_controlbits(config_folder, mode);
        if (controlbits.has_value()) {      // controlbits is loaded
            bits_to_paths(interposer, basedie, controlbits.value(), mode);
            return std::Pair<bool, bool>{true, true};
        }
        else if (mode == 0) {               // the only mode is not routed, return
            return std::Pair<bool, bool>{false, false};
        }
        else {                              // in incre mode, try to load controlbits in other modes
            std::Vector<std::pair<int, std::usize>> mode_size {};
            for (auto& [m, nets]: basedie->nets()) {
                if (m == mode) continue;

                std::usize size {0};
                for (auto& p_net: nets) {
                    size += p_net->port_number();
                }
                mode_size.emplace_back(m, size);
            }

            std::sort(mode_size.begin(), mode_size.end(), [](const std::pair<int, std::usize>& a, const std::pair<int, std::usize>& b) {
                return a.second > b.second;
            });
            
            bool has_controlbits = false;
            for (auto& [m, _]: mode_size) {
                auto controlbits = load_controlbits(config_folder, m);
                if (controlbits.has_value()) {
                    bits_to_paths(interposer, basedie, controlbits.value(), m);
                    has_controlbits = true;
                    break;
                }
            }
            return {false, has_controlbits};
        }        
    }

}
