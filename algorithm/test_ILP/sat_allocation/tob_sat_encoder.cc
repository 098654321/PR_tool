#include "sat_allocation/tob_sat_encoder.hh"

#include "ilp_allocation/tob_ilp_model.hh"

#include <algorithm>
#include <format>
#include <map>
#include <set>
#include <tuple>

namespace PR_tool {

namespace {

class SatEncoder {
public:
    auto literal(std::string_view name) -> int {
        const auto key = std::String(name);
        const auto it = _var_index.find(key);
        if (it != _var_index.end()) {
            return it->second;
        }
        const int idx = static_cast<int>(_var_index.size()) + 1;
        _var_index.emplace(key, idx);
        return idx;
    }

    auto pos(std::string_view name) -> int {
        return literal(name);
    }

    auto neg(std::string_view name) -> int {
        return -literal(name);
    }

    auto add_clause(std::initializer_list<int> lits) -> void {
        _clauses.push_back(std::Vector<int> {lits});
    }

    auto add_at_least_one(const std::Vector<int>& lits) -> void {
        _clauses.push_back(lits);
    }

    auto add_at_most_one_pairwise(const std::Vector<int>& lits) -> void {
        for (std::size_t i = 0; i < lits.size(); ++i) {
            for (std::size_t j = i + 1; j < lits.size(); ++j) {
                add_clause({-lits[i], -lits[j]});
            }
        }
    }

    auto add_exactly_one(const std::Vector<int>& lits) -> void {
        add_at_least_one(lits);
        add_at_most_one_pairwise(lits);
    }

    auto add_equiv(int a, int b) -> void {
        add_clause({-a, b});
        add_clause({a, -b});
    }

    auto add_and(int out, int a, int b) -> void {
        add_clause({-out, a});
        add_clause({-out, b});
        add_clause({out, -a, -b});
    }

    auto add_or(int out, const std::Vector<int>& inputs) -> void {
        for (const int in : inputs) {
            add_clause({-in, out});
        }
        auto clause = std::Vector<int> {inputs.begin(), inputs.end()};
        clause.push_back(-out);
        add_at_least_one(clause);
    }

    auto finish() -> TobSatCnf {
        TobSatCnf out {};
        out.num_vars = _var_index.size();
        out.num_clauses = _clauses.size();
        out.clauses = std::move(_clauses);
        out.var_index = std::move(_var_index);
        out.literal = [this](const std::string_view name) -> int {
            const auto key = std::String(name);
            const auto it = _var_index.find(key);
            if (it == _var_index.end()) {
                return 0;
            }
            return it->second;
        };
        return out;
    }

private:
    std::map<std::String, int> _var_index {};
    std::Vector<std::Vector<int>> _clauses {};
};

auto track_from_jk(const std::size_t bank, const std::size_t j, const std::size_t k, const bool straight) -> std::size_t {
    const auto v = j * 8 + k;
    if (bank == 0) {
        return straight ? v : (v + 64);
    }
    return straight ? (v + 64) : v;
}

auto relation_bumps_for(const Net_cost_record& record) -> std::Vector<Bump_coord> {
    auto relation_bumps = std::Vector<Bump_coord> {};
    if (record.type == Net_type::Bnet) {
        relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
        relation_bumps.insert(relation_bumps.end(), record.end_bumps.begin(), record.end_bumps.end());
    }
    else {
        relation_bumps.insert(relation_bumps.end(), record.start_bumps.begin(), record.start_bumps.end());
    }
    std::sort(relation_bumps.begin(), relation_bumps.end());
    relation_bumps.erase(std::unique(relation_bumps.begin(), relation_bumps.end()), relation_bumps.end());
    return relation_bumps;
}

} // namespace

auto a_var(const Bump_coord& b, const std::size_t track) -> std::String {
    return std::format("A_{}_{}_{}_{}_{}", b.TOB, b.Bank, b.Group, b.Index, track);
}

auto build_tob_sat_cnf(const std::Vector<Net_cost_record>& records) -> TobSatCnf {
    SatEncoder enc {};

    auto active_bumps = std::set<Bump_coord> {};
    auto active_i = std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::set<std::size_t>> {};
    auto active_j = std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::set<std::size_t>> {};
    auto active_k = std::map<std::tuple<std::size_t, std::size_t>, std::set<std::pair<std::size_t, std::size_t>>> {};

    for (const auto& record : records) {
        const auto bumps = relation_bumps_for(record);
        for (const auto& bump : bumps) {
            active_bumps.insert(bump);
            active_i[{bump.TOB, bump.Bank, bump.Group}].insert(bump.Index);
            for (std::size_t j = 0; j < 8; ++j) {
                active_j[{bump.TOB, bump.Bank, bump.Group}].insert(j);
                for (std::size_t k = 0; k < 8; ++k) {
                    active_k[{bump.TOB, bump.Bank}].emplace(j, k);
                }
            }
        }
    }

    for (const auto& bump : active_bumps) {
        for (std::size_t j = 0; j < 8; ++j) {
            for (std::size_t k = 0; k < 8; ++k) {
                (void)enc.pos(w_var(bump, j, k));
                (void)enc.pos(qs_var(bump, j, k));
                (void)enc.pos(qw_var(bump, j, k));
            }
        }
    }
    for (const auto& bump : active_bumps) {
        for (std::size_t v = 0; v < 64; ++v) {
            (void)enc.pos(s_var(bump.TOB, v));
        }
    }
    for (std::size_t n = 0; n < records.size(); ++n) {
        if (records[n].type == Net_type::PNnet) {
            for (const auto r_end : records[n].end_tracks) {
                (void)enc.pos(y_var(n, r_end));
            }
        }
    }
    for (const auto& bump : active_bumps) {
        for (std::size_t r = 0; r < 128; ++r) {
            (void)enc.pos(a_var(bump, r));
        }
    }

    for (const auto& bump : active_bumps) {
        auto w_lits = std::Vector<int> {};
        for (std::size_t j = 0; j < 8; ++j) {
            for (std::size_t k = 0; k < 8; ++k) {
                w_lits.push_back(enc.pos(w_var(bump, j, k)));
            }
        }
        enc.add_exactly_one(w_lits);
    }

    for (const auto& [tbg, js] : active_j) {
        const auto [t, b, g] = tbg;
        for (const auto j : js) {
            auto lits = std::Vector<int> {};
            const auto it_i = active_i.find(tbg);
            if (it_i == active_i.end()) {
                continue;
            }
            for (const auto i : it_i->second) {
                const Bump_coord bump {t, b, g, i};
                for (std::size_t k = 0; k < 8; ++k) {
                    lits.push_back(enc.pos(w_var(bump, j, k)));
                }
            }
            if (!lits.empty()) {
                enc.add_at_most_one_pairwise(lits);
            }
        }
    }

    for (const auto& [tb, jk_pairs] : active_k) {
        const auto [t, b] = tb;
        for (const auto [j, k] : jk_pairs) {
            auto lits = std::Vector<int> {};
            for (const auto& [tbg, is] : active_i) {
                if (std::get<0>(tbg) != t || std::get<1>(tbg) != b) {
                    continue;
                }
                const auto [_, __, g] = tbg;
                (void)_;
                (void)__;
                for (const auto i : is) {
                    const Bump_coord bump {t, b, g, i};
                    lits.push_back(enc.pos(w_var(bump, j, k)));
                }
            }
            if (!lits.empty()) {
                enc.add_at_most_one_pairwise(lits);
            }
        }
    }

    for (const auto& bump : active_bumps) {
        for (std::size_t j = 0; j < 8; ++j) {
            for (std::size_t k = 0; k < 8; ++k) {
                const auto v = j * 8 + k;
                const int w = enc.pos(w_var(bump, j, k));
                const int s = enc.pos(s_var(bump.TOB, v));
                const int qs = enc.pos(qs_var(bump, j, k));
                const int qw = enc.pos(qw_var(bump, j, k));
                enc.add_and(qs, w, s);
                enc.add_clause({-qw, w});
                enc.add_clause({-qw, -s});
                enc.add_clause({qw, -w, s});
            }
        }
    }

    for (const auto& bump : active_bumps) {
        for (std::size_t r = 0; r < 128; ++r) {
            auto literals_for_r = std::Vector<int> {};
            for (std::size_t j = 0; j < 8; ++j) {
                for (std::size_t k = 0; k < 8; ++k) {
                    const auto straight_track = track_from_jk(bump.Bank, j, k, true);
                    const auto swap_track = track_from_jk(bump.Bank, j, k, false);
                    if (straight_track == r) {
                        literals_for_r.push_back(enc.pos(qs_var(bump, j, k)));
                    }
                    if (swap_track == r) {
                        literals_for_r.push_back(enc.pos(qw_var(bump, j, k)));
                    }
                }
            }
            const int a = enc.pos(a_var(bump, r));
            if (literals_for_r.empty()) {
                enc.add_clause({-a});
            }
            else {
                for (const int lit : literals_for_r) {
                    enc.add_clause({-lit, a});
                }
                auto clause = literals_for_r;
                clause.push_back(-a);
                enc.add_at_least_one(clause);
            }
        }
        auto a_lits = std::Vector<int> {};
        for (std::size_t r = 0; r < 128; ++r) {
            a_lits.push_back(enc.pos(a_var(bump, r)));
        }
        enc.add_exactly_one(a_lits);
    }

    for (std::size_t t = 0; t < 16; ++t) {
        for (std::size_t v = 0; v < 64; ++v) {
            const auto j = v / 8;
            const auto k = v % 8;
            auto w_lits = std::Vector<int> {};
            for (const auto& bump : active_bumps) {
                if (bump.TOB != t) {
                    continue;
                }
                w_lits.push_back(enc.pos(w_var(bump, j, k)));
            }
            if (w_lits.empty()) {
                continue;
            }
            auto clause = w_lits;
            clause.push_back(enc.neg(s_var(t, v)));
            enc.add_at_least_one(clause);
        }
    }

    for (std::size_t n = 0; n < records.size(); ++n) {
        const auto& record = records[n];
        if (record.type == Net_type::Bnet) {
            if (record.start_bumps.empty() || record.end_bumps.empty()) {
                continue;
            }
            const auto p_s = record.start_bumps.front();
            const auto p_e = record.end_bumps.front();
            auto allowed_end = std::set<std::size_t> {};
            for (const auto r : record.end_tracks) {
                allowed_end.insert(r);
            }
            for (std::size_t r = 0; r < 128; ++r) {
                if (!allowed_end.contains(r)) {
                    enc.add_clause({enc.neg(a_var(p_e, r))});
                }
            }
            for (const auto r_e : record.end_tracks) {
                const auto it = record.starttrack_by_endtrack.find(r_e);
                auto allowed_start = std::set<std::size_t> {};
                if (it != record.starttrack_by_endtrack.end()) {
                    for (const auto r_s : it->second) {
                        allowed_start.insert(r_s);
                    }
                }
                for (std::size_t r_s = 0; r_s < 128; ++r_s) {
                    if (allowed_start.contains(r_s)) {
                        continue;
                    }
                    enc.add_clause({enc.neg(a_var(p_s, r_s)), enc.neg(a_var(p_e, r_e))});
                }
            }
        }
        else if (record.type == Net_type::Tnet) {
            if (record.start_bumps.empty() || record.end_tracks.empty()) {
                continue;
            }
            const auto p_s = record.start_bumps.front();
            const auto r_e = record.end_tracks.front();
            const auto it = record.starttrack_by_endtrack.find(r_e);
            auto allowed_start = std::set<std::size_t> {};
            if (it != record.starttrack_by_endtrack.end()) {
                for (const auto r_s : it->second) {
                    allowed_start.insert(r_s);
                }
            }
            for (std::size_t r_s = 0; r_s < 128; ++r_s) {
                if (!allowed_start.contains(r_s)) {
                    enc.add_clause({enc.neg(a_var(p_s, r_s))});
                }
            }
        }
        else if (record.type == Net_type::PNnet) {
            if (record.start_bumps.empty()) {
                continue;
            }
            const auto p_s = record.start_bumps.front();
            auto y_lits = std::Vector<int> {};
            for (const auto r_e : record.end_tracks) {
                y_lits.push_back(enc.pos(y_var(n, r_e)));
            }
            if (!y_lits.empty()) {
                enc.add_exactly_one(y_lits);
            }
            for (const auto r_e : record.end_tracks) {
                const auto it = record.starttrack_by_endtrack.find(r_e);
                auto allowed_start = std::set<std::size_t> {};
                if (it != record.starttrack_by_endtrack.end()) {
                    for (const auto r_s : it->second) {
                        allowed_start.insert(r_s);
                    }
                }
                const int y = enc.pos(y_var(n, r_e));
                for (std::size_t r_s = 0; r_s < 128; ++r_s) {
                    if (allowed_start.contains(r_s)) {
                        continue;
                    }
                    enc.add_clause({-y, enc.neg(a_var(p_s, r_s))});
                }
            }
        }
    }

    return enc.finish();
}

} // namespace PR_tool
