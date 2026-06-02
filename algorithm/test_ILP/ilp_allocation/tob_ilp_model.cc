#include "ilp_allocation/tob_ilp_model.hh"

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <std/utility.hh>

namespace PR_tool {

auto w_var(const Bump_coord& b, std::size_t j, std::size_t k) -> std::String {
    return std::format("W_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
}

auto s_var(const std::size_t t, const std::size_t v) -> std::String {
    return std::format("S_{}_{}", t, v);
}

auto qs_var(const Bump_coord& b, const std::size_t j, const std::size_t k) -> std::String {
    return std::format("QS_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
}

auto qw_var(const Bump_coord& b, const std::size_t j, const std::size_t k) -> std::String {
    return std::format("QW_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
}

auto y_var(const std::size_t n, const std::size_t r) -> std::String {
    return std::format("Y_{}_{}", n, r);
}

auto TobIlpModel::add_row(std::String name, const char type, const double rhs) -> void {
    if (this->_row_types.contains(name)) {
        return;
    }
    this->_row_types.emplace(name, type);
    this->_row_order.emplace_back(name);
    if (type != 'N') {
        this->_rhs.emplace(name, rhs);
    }
}

auto TobIlpModel::add_binary(std::String var_name) -> void {
    this->_binary_vars.insert(std::move(var_name));
}

auto TobIlpModel::add_objective(std::String var_name, const double coeff) -> void {
    if (std::fabs(coeff) < 1e-12) {
        return;
    }
    this->_objective[var_name] += coeff;
}

auto TobIlpModel::add_coefficient(std::String var_name, std::String row_name, const double coeff) -> void {
    if (std::fabs(coeff) < 1e-12) {
        return;
    }
    this->_columns[var_name].emplace_back(std::move(row_name), coeff);
}

auto TobIlpModel::write_mps(const std::String& path) const -> void {
    std::ofstream out(path);
    if (!out.is_open()) {
        throw std::runtime_error(std::format("cannot open mps output '{}'", path));
    }

    out << "NAME TOB_ALLOC\n";
    out << "ROWS\n";
    for (const auto& row_name : this->_row_order) {
        out << ' ' << this->_row_types.at(row_name) << ' ' << row_name << '\n';
    }

    out << "COLUMNS\n";
    for (const auto& [var_name, entries] : this->_columns) {
        if (const auto it = this->_objective.find(var_name); it != this->_objective.end()) {
            out << "    " << var_name << " OBJ " << it->second << '\n';
        }
        for (const auto& [row_name, coeff] : entries) {
            out << "    " << var_name << ' ' << row_name << ' ' << coeff << '\n';
        }
    }

    out << "RHS\n";
    for (const auto& row_name : this->_row_order) {
        if (const auto it = this->_rhs.find(row_name); it != this->_rhs.end()) {
            if (std::fabs(it->second) < 1e-12) {
                continue;
            }
            out << "    RHS1 " << row_name << ' ' << it->second << '\n';
        }
    }

    out << "BOUNDS\n";
    for (const auto& var_name : this->_binary_vars) {
        out << " BV BND1 " << var_name << '\n';
    }
    out << "ENDATA\n";
}

auto TobIlpModel::linear_data() const -> TobIlpLinearData {
    auto data = TobIlpLinearData {};
    auto row_to_idx = std::map<std::String, std::size_t> {};
    for (const auto& row_name : this->_row_order) {
        const auto it_ty = this->_row_types.find(row_name);
        if (it_ty == this->_row_types.end() || it_ty->second == 'N') {
            continue;
        }
        const auto row_idx = data.rows.size();
        row_to_idx.emplace(row_name, row_idx);
        const double rhs = this->_rhs.contains(row_name) ? this->_rhs.at(row_name) : 0.0;
        data.rows.push_back(TobIlpLinearRow {row_name, it_ty->second, rhs});
    }

    data.columns.reserve(this->_columns.size());
    for (const auto& [var_name, entries] : this->_columns) {
        auto col = TobIlpLinearColumn {};
        col.name = var_name;
        if (const auto o = this->_objective.find(var_name); o != this->_objective.end()) {
            col.objective = o->second;
        }
        col.binary = this->_binary_vars.contains(var_name);
        col.entries.reserve(entries.size());
        for (const auto& [rname, coeff] : entries) {
            const auto rit = row_to_idx.find(rname);
            if (rit == row_to_idx.end()) {
                throw std::runtime_error(std::format("tob ilp: row '{}' missing from index map (OBJ-only row?)", rname));
            }
            col.entries.push_back({rit->second, coeff});
        }
        data.column_index.emplace(var_name, data.columns.size());
        data.columns.push_back(std::move(col));
    }
    return data;
}

// MARK: build_tob_ilp_model
void build_tob_ilp_model(
    TobIlpModel& mps,
    const std::Vector<Net_cost_record>& records
) {
    mps.set_net_count(records.size());

    auto active_bumps = std::set<Bump_coord> {};
    auto active_i = std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::set<std::size_t>> {};

    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto& record = records[n];

        for (const auto& b : record.start_bumps) {
            active_bumps.insert(b);
            active_i[{b.TOB, b.Bank, b.Group}].insert(b.Index);
        }
        for (const auto& b : record.end_bumps) {
            active_bumps.insert(b);
            active_i[{b.TOB, b.Bank, b.Group}].insert(b.Index);
        }
    }

    // 约束1
    for (const auto& b : active_bumps) {
        const auto row = std::format("R_WONE_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index);
        mps.add_row(row, 'E', 1.0);
        for (std::size_t j = 0; j < 8; ++j) {
            for (std::size_t k = 0; k < 8; ++k) {
                const auto w = w_var(b, j, k);
                mps.add_binary(w);
                mps.add_coefficient(w, row, 1.0);
            }
        }
    }

    // 约束2
    for (std::size_t t = 0; t < 16; ++t) {
        for (std::size_t b = 0; b < 2; ++b) {
            for (std::size_t g = 0; g < 8; ++g) {
                const auto key = std::tuple<std::size_t, std::size_t, std::size_t>{t, b, g};
                if (!active_i.contains(key)) {
                    continue;
                }
                for (std::size_t j = 0; j < 8; ++j) {
                    const auto row = std::format("R_HORI_{}_{}_{}_{}", t, b, g, j);
                    mps.add_row(row, 'L', 1.0);
                    for (const auto i : active_i.at(key)) {
                        for (std::size_t k = 0; k < 8; ++k) {
                            mps.add_coefficient(w_var(Bump_coord{t, b, g, i}, j, k), row, 1.0);
                        }
                    }
                }
            }
        }
    }

    // 约束3
    for (std::size_t t = 0; t < 16; ++t) {
        for (std::size_t b = 0; b < 2; ++b) {
            bool tb_active = false;
            for (std::size_t g = 0; g < 8; ++g) {
                if (active_i.contains(std::tuple<std::size_t, std::size_t, std::size_t>{t, b, g})) {
                    tb_active = true;
                    break;
                }
            }
            if (!tb_active) {
                continue;
            }
            for (std::size_t j = 0; j < 8; ++j) {
                for (std::size_t k = 0; k < 8; ++k) {
                    const auto row = std::format("R_VERT_{}_{}_{}_{}", t, b, j, k);
                    mps.add_row(row, 'L', 1.0);
                    for (std::size_t g = 0; g < 8; ++g) {
                        const auto gkey = std::tuple<std::size_t, std::size_t, std::size_t>{t, b, g};
                        if (!active_i.contains(gkey)) {
                            continue;
                        }
                        for (const auto i : active_i.at(gkey)) {
                            mps.add_coefficient(w_var(Bump_coord{t, b, g, i}, j, k), row, 1.0);
                        }
                    }
                }
            }
        }
    }

    auto ensure_q_linearization = [&](const Bump_coord& b, const std::size_t j, const std::size_t k) {
        const auto w = w_var(b, j, k);
        const auto qs = qs_var(b, j, k);
        const auto qw = qw_var(b, j, k);
        const auto s = s_var(b.TOB, j * 8 + k);

        mps.add_binary(w);
        mps.add_binary(s);
        mps.add_binary(qs);
        mps.add_binary(qw);

        {
            const auto row = std::format("R_QS1_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
            mps.add_row(row, 'L', 0.0);
            mps.add_coefficient(qs, row, 1.0);
            mps.add_coefficient(w, row, -1.0);
        }
        {
            const auto row = std::format("R_QS2_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
            mps.add_row(row, 'L', 0.0);
            mps.add_coefficient(qs, row, 1.0);
            mps.add_coefficient(s, row, -1.0);
        }
        {
            const auto row = std::format("R_QS3_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
            mps.add_row(row, 'G', -1.0);
            mps.add_coefficient(qs, row, 1.0);
            mps.add_coefficient(w, row, -1.0);
            mps.add_coefficient(s, row, -1.0);
        }

        {
            const auto row = std::format("R_QW1_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
            mps.add_row(row, 'L', 0.0);
            mps.add_coefficient(qw, row, 1.0);
            mps.add_coefficient(w, row, -1.0);
        }
        {
            const auto row = std::format("R_QW2_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
            mps.add_row(row, 'L', 1.0);
            mps.add_coefficient(qw, row, 1.0);
            mps.add_coefficient(s, row, 1.0);
        }
        {
            const auto row = std::format("R_QW3_{}_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, j, k);
            mps.add_row(row, 'G', 0.0);
            mps.add_coefficient(qw, row, 1.0);
            mps.add_coefficient(w, row, -1.0);
            mps.add_coefficient(s, row, 1.0);
        }
    };

    const auto track_from_jk = [](const std::size_t bank, const std::size_t j, const std::size_t k, const bool straight) -> std::size_t {
        const auto v = j * 8 + k;
        if (bank == 0) {
            return straight ? v : (v + 64);
        }
        return straight ? (v + 64) : v;
    };

    auto processed_q = std::set<std::tuple<Bump_coord, std::size_t, std::size_t>> {};

    const auto ensure_qs_qw = [&](const Bump_coord& b) {
        for (std::size_t j = 0; j < 8; ++j) {
            for (std::size_t k = 0; k < 8; ++k) {
                const auto key = std::tuple<Bump_coord, std::size_t, std::size_t>{b, j, k};
                if (!processed_q.contains(key)) {
                    ensure_q_linearization(b, j, k);
                    processed_q.insert(key);
                }
            }
        }
    };

    const auto add_u_track_expr = [&](const Bump_coord& b, const std::size_t track, const std::String& row, const double coef) {
        for (std::size_t j = 0; j < 8; ++j) {
            for (std::size_t k = 0; k < 8; ++k) {
                if (track_from_jk(b.Bank, j, k, true) == track) {
                    mps.add_coefficient(qs_var(b, j, k), row, coef);
                }
                if (track_from_jk(b.Bank, j, k, false) == track) {
                    mps.add_coefficient(qw_var(b, j, k), row, coef);
                }
            }
        }
    };

    const auto build_allow_mask = [](const std::Vector<std::size_t>& allow_tracks) {
        auto allow = std::array<bool, 128> {};
        allow.fill(false);
        for (const auto r : allow_tracks) {
            if (r < 128) {
                allow[r] = true;
            }
        }
        return allow;
    };

    // Track-level reachability + PN end-track selection.
    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto& record = records[n];
        if (record.type == Net_type::Bnet) {
            if (record.start_bumps.empty() || record.end_bumps.empty()) {
                continue;
            }
            const auto start_bump = record.start_bumps.front();     // 目前start bump只有1个，end bump也只有1个
            const auto end_bump = record.end_bumps.front();
            ensure_qs_qw(start_bump);
            ensure_qs_qw(end_bump);

            const auto end_allow = build_allow_mask(record.end_tracks);
            for (std::size_t r = 0; r < 128; ++r) {
                if (end_allow[r]) {
                    continue;
                }
                const auto row = std::format("R_BEND0_{}_{}", n, r);
                mps.add_row(row, 'E', 0.0);
                add_u_track_expr(end_bump, r, row, 1.0);
            }

            for (std::size_t r_end = 0; r_end < 128; ++r_end) {
                if (!end_allow[r_end]) {
                    continue;
                }
                const auto it_allow = record.starttrack_by_endtrack.find(r_end);
                const auto start_allow = (it_allow == record.starttrack_by_endtrack.end())
                                             ? std::Vector<std::size_t> {}
                                             : it_allow->second;
                const auto start_allow_mask = build_allow_mask(start_allow);
                for (std::size_t r_start = 0; r_start < 128; ++r_start) {
                    if (start_allow_mask[r_start]) {
                        continue;
                    }
                    const auto row = std::format("R_BREACH_{}_{}_{}", n, r_end, r_start);
                    mps.add_row(row, 'L', 1.0);
                    add_u_track_expr(start_bump, r_start, row, 1.0);
                    add_u_track_expr(end_bump, r_end, row, 1.0);
                }
            }
            continue;
        }

        if (record.type == Net_type::Tnet) {
            if (record.start_bumps.empty()) {
                continue;
            }
            const auto start_bump = record.start_bumps.front();
            ensure_qs_qw(start_bump);
            if (record.end_tracks.empty()) {
                continue;
            }
            const auto fixed_end = record.end_tracks.front();
            const auto it_allow = record.starttrack_by_endtrack.find(fixed_end);
            const auto start_allow = (it_allow == record.starttrack_by_endtrack.end())
                                         ? std::Vector<std::size_t> {}
                                         : it_allow->second;
            const auto start_allow_mask = build_allow_mask(start_allow);
            for (std::size_t r = 0; r < 128; ++r) {
                if (start_allow_mask[r]) {
                    continue;
                }
                const auto row = std::format("R_TREACH0_{}_{}", n, r);
                mps.add_row(row, 'E', 0.0);
                add_u_track_expr(start_bump, r, row, 1.0);
            }
            continue;
        }

        if (record.type == Net_type::PNnet) {
            if (record.start_bumps.empty() || record.end_tracks.empty()) {
                continue;
            }
            const auto start_bump = record.start_bumps.front();
            ensure_qs_qw(start_bump);

            const auto ysum_row = std::format("R_PNYSUM_{}", n);
            mps.add_row(ysum_row, 'E', 1.0);
            for (const auto r_end : record.end_tracks) {
                const auto y = y_var(n, r_end);
                mps.add_binary(y);
                mps.add_coefficient(y, ysum_row, 1.0);
            }

            for (const auto r_end : record.end_tracks) {
                const auto it_allow = record.starttrack_by_endtrack.find(r_end);
                const auto start_allow = (it_allow == record.starttrack_by_endtrack.end())
                                             ? std::Vector<std::size_t> {}
                                             : it_allow->second;
                const auto start_allow_mask = build_allow_mask(start_allow);
                for (std::size_t r_start = 0; r_start < 128; ++r_start) {
                    if (start_allow_mask[r_start]) {
                        continue;
                    }
                    const auto row = std::format("R_PNREACH_{}_{}_{}", n, r_end, r_start);
                    mps.add_row(row, 'L', 1.0);
                    add_u_track_expr(start_bump, r_start, row, 1.0);
                    mps.add_coefficient(y_var(n, r_end), row, 1.0);
                }
            }
        }
    }
}

} // namespace PR_tool
