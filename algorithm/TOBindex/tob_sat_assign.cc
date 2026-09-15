// Feasibility SAT for one TOB.
// Encodes the same TOB switch rules as algorithm/test_ILP/sat/encode_tob_special.cc:
//   Y uniqueness, Bump–HLine / HLine–VLine partial matching, VLine–Track M_g mode.
// Input:
//   128 COBUnit ids: search a color (track within the unit) for each bump.
//   --tracks plus 128 track ids: pin each bump to that track and test feasibility.
// Output: "SAT" + 128 colors, or "UNSAT".

#include "sat/sat_constraint_kits.hh"
#include "sat_allocation/cadical_solver.hh"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr int kBumps = 128;
constexpr int kColors = 8;
constexpr int kMux = 16;
constexpr int kVtMux = 64;

struct Instance {
    std::vector<int> units;
    std::vector<int> forced_color;
};

auto exactly_one(PR_tool::CadicalSession& session, const std::vector<int>& lits) -> void {
    PR_tool::add_exactly_one(session, std::span<const int> {lits.data(), lits.size()});
}

auto at_most_one(PR_tool::CadicalSession& session, const std::vector<int>& lits) -> void {
    PR_tool::add_pairwise_at_most_one(
        session, std::span<const int> {lits.data(), lits.size()});
}

auto cobunit_of_track(int track) -> int {
    return track < 64 ? track % kColors : track % kColors + kColors;
}

auto color_of_track(int track) -> int {
    return (track % 64) / kColors;
}

auto read_ints(int argc, char** argv, int argi) -> std::vector<int> {
    std::vector<int> vals;
    vals.reserve(kBumps);
    if (argi < argc) {
        for (int i = argi; i < argc; ++i) {
            vals.push_back(std::atoi(argv[i]));
        }
    } else {
        std::string line;
        std::getline(std::cin, line);
        std::istringstream in {line};
        int v = 0;
        while (in >> v) {
            vals.push_back(v);
        }
    }
    return vals;
}

auto read_instance(int argc, char** argv) -> Instance {
    bool tracks_mode = false;
    int argi = 1;
    if (argc > 1 && std::string_view {argv[1]} == "--tracks") {
        tracks_mode = true;
        argi = 2;
    }
    const auto vals = read_ints(argc, argv, argi);
    if (static_cast<int>(vals.size()) != kBumps) {
        throw std::runtime_error(tracks_mode ? "expected 128 track ids" : "expected 128 COBUnit ids");
    }
    Instance inst;
    inst.units.resize(static_cast<std::size_t>(kBumps));
    if (!tracks_mode) {
        for (int bump = 0; bump < kBumps; ++bump) {
            const int unit = vals[static_cast<std::size_t>(bump)];
            if (unit < 0 || unit >= 16) {
                throw std::runtime_error("COBUnit id must be in [0, 15]");
            }
            inst.units[static_cast<std::size_t>(bump)] = unit;
        }
        return inst;
    }
    inst.forced_color.assign(static_cast<std::size_t>(kBumps), -1);
    auto seen = std::vector<char>(kBumps, 0);
    for (int bump = 0; bump < kBumps; ++bump) {
        const int track = vals[static_cast<std::size_t>(bump)];
        if (track < 0 || track >= kBumps) {
            throw std::runtime_error("track id must be in [0, 127]");
        }
        if (seen[static_cast<std::size_t>(track)] != 0) {
            throw std::runtime_error("duplicate track id");
        }
        seen[static_cast<std::size_t>(track)] = 1;
        inst.units[static_cast<std::size_t>(bump)] = cobunit_of_track(track);
        inst.forced_color[static_cast<std::size_t>(bump)] = color_of_track(track);
    }
    return inst;
}

} // namespace

auto main(int argc, char** argv) -> int {
    try {
        const auto inst = read_instance(argc, argv);
        PR_tool::CadicalSession session {};

        auto x = std::vector<std::vector<int>>(kBumps, std::vector<int>(kColors, 0));
        for (int bump = 0; bump < kBumps; ++bump) {
            for (int color = 0; color < kColors; ++color) {
                x[static_cast<std::size_t>(bump)][static_cast<std::size_t>(color)] =
                    session.new_var();
            }
            exactly_one(session, x[static_cast<std::size_t>(bump)]);
            if (!inst.forced_color.empty()) {
                const int color = inst.forced_color[static_cast<std::size_t>(bump)];
                session.add_clause({x[static_cast<std::size_t>(bump)][static_cast<std::size_t>(color)]});
            }
        }

        auto bh_out = std::vector<std::vector<std::vector<int>>>(
            kMux, std::vector<std::vector<int>>(kColors));
        auto hv_in = std::vector<std::vector<std::vector<int>>>(
            kMux, std::vector<std::vector<int>>(kColors));
        auto hv_out = std::vector<std::vector<std::vector<int>>>(
            kMux, std::vector<std::vector<int>>(kColors));
        auto vt_in = std::vector<std::vector<std::vector<int>>>(
            kVtMux, std::vector<std::vector<int>>(2));
        auto vt_out = std::vector<std::vector<std::vector<int>>>(
            kVtMux, std::vector<std::vector<int>>(2));
        auto m_g = std::vector<int>(kVtMux, 0);
        for (int mux = 0; mux < kVtMux; ++mux) {
            m_g[static_cast<std::size_t>(mux)] = session.new_var();
        }

        for (int bump = 0; bump < kBumps; ++bump) {
            const int unit = inst.units[static_cast<std::size_t>(bump)];
            const int group = bump / kColors;
            const int residue = unit % kColors;
            const int tb = unit / kColors;
            const int bb = bump / 64;
            for (int color = 0; color < kColors; ++color) {
                const int lit = x[static_cast<std::size_t>(bump)][static_cast<std::size_t>(color)];
                const int hv_mux = bb == 0 ? color : color + kColors;
                const int hv_inner = bb == 0 ? group : group - kColors;
                const int vt_mux = color * kColors + residue;
                bh_out[static_cast<std::size_t>(group)][static_cast<std::size_t>(color)].push_back(lit);
                hv_in[static_cast<std::size_t>(hv_mux)][static_cast<std::size_t>(hv_inner)].push_back(lit);
                hv_out[static_cast<std::size_t>(hv_mux)][static_cast<std::size_t>(residue)].push_back(lit);
                vt_in[static_cast<std::size_t>(vt_mux)][static_cast<std::size_t>(bb)].push_back(lit);
                vt_out[static_cast<std::size_t>(vt_mux)][static_cast<std::size_t>(tb)].push_back(lit);

                const int mode = m_g[static_cast<std::size_t>(vt_mux)];
                const bool straight = (bb == tb);
                session.add_clause({-lit, straight ? mode : -mode});
            }
        }

        const auto amo_grid = [&](const std::vector<std::vector<std::vector<int>>>& grid) {
            for (const auto& row : grid) {
                for (const auto& lits : row) {
                    at_most_one(session, lits);
                }
            }
        };
        amo_grid(bh_out);
        amo_grid(hv_in);
        amo_grid(hv_out);
        amo_grid(vt_in);
        amo_grid(vt_out);

        const auto result = session.solve_once();
        if (!result.ok) {
            std::cout << "UNSAT\n";
            return 0;
        }
        std::cout << "SAT\n";
        for (int bump = 0; bump < kBumps; ++bump) {
            int color = -1;
            for (int o = 0; o < kColors; ++o) {
                if (session.value(x[static_cast<std::size_t>(bump)][static_cast<std::size_t>(o)])) {
                    color = o;
                    break;
                }
            }
            if (color < 0) {
                throw std::runtime_error("SAT model missing color for a bump");
            }
            if (!inst.forced_color.empty()
                && color != inst.forced_color[static_cast<std::size_t>(bump)]) {
                throw std::runtime_error("SAT model used a color other than the pinned track");
            }
            if (bump) {
                std::cout << ' ';
            }
            std::cout << color;
        }
        std::cout << '\n';
        return 0;
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "tob_sat_assign: %s\n", e.what());
        return 2;
    }
}
