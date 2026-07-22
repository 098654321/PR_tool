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

    auto output_from_routing_results(
        hardware::Interposer* interposer,
        const std::FilePath& output_path,
        circuit::BaseDie* basedie,
        int mode,
        bool try_all_modes,
        bool simplify_controlbits,
        const RegisterMapConfig& register_map
    ) -> void {
        if (!try_all_modes) {
            connect_registers(interposer, basedie, mode);
            write_control_bits(interposer, output_path, mode, simplify_controlbits, register_map);
            interposer->reset_regs();
        }
        else {
            // TODO(split-output): try_all_modes needs mode_<m>/regnamecontrolbit_4part/ to avoid overwrite
            std::set<int> modes;
            for(const auto& [m, _]: basedie->nets()) {
                modes.insert(m);
            }
            for(const auto& m: modes) {
                connect_registers(interposer, basedie, m);
                write_control_bits(interposer, output_path, m, simplify_controlbits, register_map);
                interposer->reset_regs();
            }
        }
    }

    auto write_control_bits(
        hardware::Interposer* interposer,
        const std::FilePath& output_path,
        int mode,
        bool simplify_controlbits,
        const RegisterMapConfig& register_map
    ) -> void {
        (void)mode;
        debug::info_fmt(
            "\n\
            **********************************************************************************\n\
                            Write split control bits under '{}'\n\
            **********************************************************************************\
            ",
            (output_path / "regnamecontrolbit_4part").string());
        // TODO(split-output): try_all_modes needs mode_<m>/regnamecontrolbit_4part/ to avoid overwrite
        Writer{interposer}.fetch_and_write_split(register_map, output_path, simplify_controlbits);

        debug::info_fmt("END\n\n");
    }

    auto write_control_bits_pair(
        hardware::Interposer* interposer,
        const std::FilePath& full_output_path,
        const std::FilePath& simplified_output_path,
        int mode,
        const RegisterMapConfig& register_map
    ) -> void {
        (void)mode;
        debug::info_fmt(
            "\n\
            **********************************************************************************\n\
                            Write full and simplified split control bits under '{}' and '{}'\n\
            **********************************************************************************\
            ", (full_output_path / "regnamecontrolbit_4part").string(),
               (simplified_output_path / "regnamecontrolbit_4part").string()
        );
        Writer{interposer}.fetch_and_write_split_pair(register_map, full_output_path, simplified_output_path);
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
