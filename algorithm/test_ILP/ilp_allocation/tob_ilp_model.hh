#pragma once

#include "common/ilp_types.hh"

#include <cstddef>
#include <map>
#include <set>
#include <std/collection.hh>
#include <std/utility.hh>
#include <std/string.hh>

namespace PR_tool {

struct TobIlpLinearRow {
    std::String name;
    char type{'N'};
    double rhs{0.0};
};

struct TobIlpLinearColumn {
    std::String name;
    double objective{0.0};
    bool binary{false};
    std::Vector<std::Pair<std::size_t, double>> entries;
};

struct TobIlpLinearData {
    std::Vector<TobIlpLinearRow> rows;
    std::Vector<TobIlpLinearColumn> columns;
    std::map<std::String, std::size_t> column_index;
};

class TobIlpModel {
public:
    auto add_row(std::String name, char type, double rhs = 0.0) -> void;
    auto add_binary(std::String var_name) -> void;
    auto add_objective(std::String var_name, double coeff) -> void;
    auto add_coefficient(std::String var_name, std::String row_name, double coeff) -> void;

    auto write_mps(const std::String& path) const -> void;
    auto linear_data() const -> TobIlpLinearData;

    auto set_net_count(std::size_t n) -> void {
        this->_net_count = n;
    }
    auto net_count() const -> std::size_t {
        return this->_net_count;
    }

private:
    std::HashMap<std::String, char> _row_types {};
    std::Vector<std::String> _row_order {};
    std::HashMap<std::String, double> _rhs {};
    std::map<std::String, std::Vector<std::Pair<std::String, double>>> _columns {};
    std::HashMap<std::String, double> _objective {};
    std::set<std::String> _binary_vars {};
    std::size_t _net_count{0};
};

auto w_var(const Bump_coord& b, std::size_t j, std::size_t k) -> std::String;
auto s_var(std::size_t t, std::size_t v) -> std::String;
auto qs_var(const Bump_coord& b, std::size_t j, std::size_t k) -> std::String;
auto qw_var(const Bump_coord& b, std::size_t j, std::size_t k) -> std::String;
auto y_var(std::size_t n, std::size_t r) -> std::String;

void build_tob_ilp_model(
    TobIlpModel& model,
    const std::Vector<Net_cost_record>& records
);

} // namespace PR_tool
