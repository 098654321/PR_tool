#include "./module.hh"
#include "./writer.hh"
#include "debug/debug.hh"
#include <circuit/basedie.hh>
#include <hardware/cob/cob.hh>
#include <hardware/interposer.hh>

namespace PR_tool::parse {

    namespace {

    // Golden printControlBit.cpp::resetintr01 — default TX ext-port COB sel before output.
    auto apply_ext_io_sel_defaults(hardware::Interposer* interposer) -> void {
        constexpr std::usize kTxPortIndices[] = {71, 78, 85, 92};
        struct CobDefault {
            std::i64 row;
            std::i64 col;
            hardware::COBDirection dir;
        };
        constexpr CobDefault kDefaults[] = {
            {6, 11, hardware::COBDirection::Right},
            {8, 1, hardware::COBDirection::Up},
            {0, 10, hardware::COBDirection::Down},
            {0, 2, hardware::COBDirection::Down},
        };

        for (const auto& def : kDefaults) {
            auto cob = interposer->get_cob(hardware::COBCoord{def.row, def.col});
            if (!cob.has_value()) {
                continue;
            }
            for (const auto port_index : kTxPortIndices) {
                cob.value()->sel_register(def.dir, port_index)->set_cob_to_track();
            }
        }
    }

    }  // namespace

    auto output_from_routing_results(hardware::Interposer* interposer, const std::FilePath& output_path, circuit::BaseDie* basedie, int mode, bool try_all_modes) -> void {
        if (!try_all_modes) {
            connect_registers(interposer, basedie, mode);
            write_control_bits(interposer, output_path, mode);
            interposer->reset_regs();
        }
        else {
            std::set<int> modes;
            for(const auto& [m, _]: basedie->nets()) {
                modes.insert(m);
            }
            for(const auto& m: modes) {
                connect_registers(interposer, basedie, m);
                write_control_bits(interposer, output_path, m);
                interposer->reset_regs();
            }
        }
    }

    auto write_control_bits(hardware::Interposer* interposer, const std::FilePath& output_path, int mode) -> void {
        std::FilePath control_bits_path = output_path / ("controlbits_" + std::to_string(mode) + ".txt");
        debug::info_fmt(
            "\n\
            **********************************************************************************\n\
                            Write control bits into '{}'\n\
            **********************************************************************************\
            ", control_bits_path.string()
        );
        auto writer = parse::Writer{interposer};
        writer.fetch_and_write(control_bits_path);

        debug::info_fmt("END\n\n");
    }

    auto connect_registers(hardware::Interposer* interposer, circuit::BaseDie* basedie, int mode) -> void {
        debug::info("Connecting paths ...");
        auto nets = basedie->nets_to_vector();
        for (auto& net : nets) {
            if (net->modes().contains(mode)) {
                debug::debug_fmt("{} is connecting paths ...", net->name());

                auto& path_package = net->pathpackage();    
                path_package.connect_all();
            }
            // else {
            //     net->pathpackage().reset_all();
            // }
        }
        apply_ext_io_sel_defaults(interposer);
    }

}