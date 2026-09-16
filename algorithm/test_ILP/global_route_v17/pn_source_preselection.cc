#include "global_route_v17/pn_source_preselection.hh"

#include "common/hw_map.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <debug/debug.hh>
#include <format>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

#ifdef USE_HIGHS
#include <Highs.h>
#endif

namespace PR_tool {

namespace {

constexpr std::size_t kUnitCount = 16;
constexpr std::size_t kResidueCount = 8;
constexpr double kRudyWeight = 1.0;

struct GridPoint {
    int row{0};
    int col{0};
};

struct Candidate {
    std::size_t net_index{0};
    std::size_t demand_index{0};
    std::size_t source_index{0};
    std::size_t unit{0};
    Bump_coord bump{};
    GridPoint sink;
    GridPoint source;
    int distance{0};
    int variable{-1};
};

struct Activation {
    std::size_t net_index{0};
    std::size_t source_index{0};
    int variable{-1};
    std::Vector<std::size_t> candidate_indices;
};

auto port_key(const GraphNodeRef& endpoint) -> GlobalPortKey {
    return GlobalPortKey{endpoint.track_coord.dir == hardware::TrackDirection::Horizontal ? 0 : 1,
                         static_cast<int>(endpoint.track_coord.row),
                         static_cast<int>(endpoint.track_coord.col), endpoint.track_index};
}

auto endpoint_grid_point(const GlobalChannelGraph& graph, const GraphNodeRef& endpoint)
    -> GridPoint {
    int node = -1;
    if (endpoint.kind == GraphNodeRef::Kind::Bump) {
        const auto it = graph.tob_node_by_tob.find(endpoint.bump.TOB);
        if (it != graph.tob_node_by_tob.end()) {
            node = it->second;
        }
    } else if (endpoint.kind == GraphNodeRef::Kind::Track) {
        const auto it = graph.port_node_by_key.find(port_key(endpoint));
        if (it != graph.port_node_by_key.end()) {
            node = it->second;
        }
    }
    if (node < 0 || static_cast<std::size_t>(node) >= graph.nodes.size()) {
        throw std::invalid_argument("V18 PN preselection endpoint is absent from Channel graph");
    }
    const auto& data = graph.nodes[static_cast<std::size_t>(node)];
    return GridPoint{data.row, data.col};
}

auto hpwl(const GridPoint lhs, const GridPoint rhs) -> int {
    return std::abs(lhs.row - rhs.row) + std::abs(lhs.col - rhs.col);
}

auto cell_index(const int row, const int col, const int cols) -> std::size_t {
    return static_cast<std::size_t>(row * cols + col);
}

auto add_rudy(std::Vector<double>& demand, const GridPoint lhs, const GridPoint rhs,
              const std::size_t unit, const double unit_fraction, const int rows, const int cols)
    -> void {
    const int length = hpwl(lhs, rhs);
    if (length == 0 || unit >= kUnitCount || unit_fraction == 0.0) {
        return;
    }
    const int row_lo = std::min(lhs.row, rhs.row);
    const int row_hi = std::max(lhs.row, rhs.row);
    const int col_lo = std::min(lhs.col, rhs.col);
    const int col_hi = std::max(lhs.col, rhs.col);
    const auto area =
        static_cast<double>(row_hi - row_lo + 1) * static_cast<double>(col_hi - col_lo + 1);
    const double rho = unit_fraction * static_cast<double>(length) / area;
    for (int row = row_lo; row <= row_hi; ++row) {
        for (int col = col_lo; col <= col_hi; ++col) {
            if (row < 0 || row >= rows || col < 0 || col >= cols) {
                continue;
            }
            const auto index = cell_index(row, col, cols) * kUnitCount + unit;
            demand[index] += rho;
        }
    }
}

auto median_positive_distance_gap(const std::Vector<Candidate>& candidates,
                                  const std::Vector<std::Vector<std::size_t>>& candidates_by_demand)
    -> double {
    auto gaps = std::Vector<int>{};
    for (const auto& indices : candidates_by_demand) {
        auto distances = std::Vector<int>{};
        distances.reserve(indices.size());
        for (const auto index : indices) {
            distances.push_back(candidates[index].distance);
        }
        std::sort(distances.begin(), distances.end());
        distances.erase(std::unique(distances.begin(), distances.end()), distances.end());
        if (distances.size() >= 2) {
            gaps.push_back(distances[1] - distances[0]);
        }
    }
    gaps.erase(std::remove_if(gaps.begin(), gaps.end(), [](const int gap) { return gap <= 0; }),
               gaps.end());
    if (gaps.empty()) {
        return 1.0;
    }
    std::sort(gaps.begin(), gaps.end());
    const auto middle = gaps.size() / 2;
    if (gaps.size() % 2 != 0) {
        return static_cast<double>(gaps[middle]);
    }
    return (static_cast<double>(gaps[middle - 1]) + static_cast<double>(gaps[middle])) / 2.0;
}

auto transformed_nets(const std::Vector<RoutingNet>& nets, const std::Vector<Candidate>& candidates,
                      const std::Vector<bool>& selected) -> std::Vector<RoutingNet> {
    auto selected_by_net_source =
        std::map<std::pair<std::size_t, std::size_t>, std::Vector<std::size_t>>{};
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        if (selected[index]) {
            selected_by_net_source[{candidates[index].net_index, candidates[index].source_index}]
                .push_back(candidates[index].demand_index);
        }
    }

    auto out = std::Vector<RoutingNet>{};
    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        const auto& net = nets[net_index];
        if (net.kind != RoutingNetKind::PNnet) {
            out.push_back(net);
            continue;
        }
        for (const auto& [key, demand_indices] : selected_by_net_source) {
            if (key.first != net_index) {
                continue;
            }
            const auto source_index = key.second;
            auto tree = RoutingNet{};
            tree.name = std::format("{}#source{}", net.name, source_index);
            tree.origin_uid = net.origin_uid;
            tree.origin_key = net.origin_key;
            tree.kind = RoutingNetKind::Tnet;
            tree.sources = {net.sources.at(source_index)};
            for (const auto demand_index : demand_indices) {
                auto demand = net.demands.at(demand_index);
                demand.demand_id = tree.demands.size();
                demand.candidate_source_indices = {0};
                demand.fixed_pair = true;
                tree.demands.push_back(std::move(demand));
            }
            out.push_back(std::move(tree));
        }
    }
    for (std::size_t net_index = 0; net_index < out.size(); ++net_index) {
        out[net_index].net_id = net_index;
        out[net_index].virtual_source_node = -1;
        out[net_index].global_selected_pn_source_indices.clear();
        for (std::size_t demand = 0; demand < out[net_index].demands.size(); ++demand) {
            out[net_index].demands[demand].demand_id = demand;
        }
    }
    return out;
}

#ifdef USE_HIGHS

class SmallMip {
  public:
    explicit SmallMip(const int verbose_level) {
        highs_.setOptionValue("output_flag", verbose_level >= 3);
        highs_.setOptionValue("mip_rel_gap", 0.0);
    }

    auto add_binary(const double cost) -> int {
        const int column = static_cast<int>(variables_++);
        check(highs_.addCol(cost, 0.0, 1.0, 0, nullptr, nullptr), "addCol");
        check(highs_.changeColIntegrality(column, HighsVarType::kInteger), "integrality");
        return column;
    }

    auto add_continuous(const double cost) -> int {
        const int column = static_cast<int>(variables_++);
        check(highs_.addCol(cost, 0.0, kHighsInf, 0, nullptr, nullptr), "addCol");
        return column;
    }

    auto add_row(const double lower, const double upper,
                 const std::Vector<std::pair<int, double>>& terms) -> void {
        auto indices = std::Vector<HighsInt>{};
        auto values = std::Vector<double>{};
        indices.reserve(terms.size());
        values.reserve(terms.size());
        for (const auto& [column, coefficient] : terms) {
            if (coefficient != 0.0) {
                indices.push_back(static_cast<HighsInt>(column));
                values.push_back(coefficient);
            }
        }
        check(highs_.addRow(lower, upper, static_cast<HighsInt>(indices.size()),
                            indices.empty() ? nullptr : indices.data(),
                            values.empty() ? nullptr : values.data()),
              "addRow");
        ++constraints_;
    }

    auto solve() -> HighsModelStatus {
        check(highs_.run(), "run");
        return highs_.getModelStatus();
    }

    [[nodiscard]] auto values() const -> const std::Vector<double>& {
        return highs_.getSolution().col_value;
    }
    [[nodiscard]] auto objective() const -> double { return highs_.getObjectiveValue(); }
    [[nodiscard]] auto status() const -> std::String {
        return highs_.modelStatusToString(highs_.getModelStatus());
    }
    [[nodiscard]] auto variables() const -> std::size_t { return variables_; }
    [[nodiscard]] auto constraints() const -> std::size_t { return constraints_; }

  private:
    static auto check(const HighsStatus status, const char* operation) -> void {
        if (status == HighsStatus::kError) {
            throw std::runtime_error(std::format("HiGHS PN preselection {} failed", operation));
        }
    }

    Highs highs_;
    std::size_t variables_{0};
    std::size_t constraints_{0};
};

#endif

} // namespace

auto preselect_pn_sources_v18(const GlobalChannelGraph& graph, const std::Vector<RoutingNet>& nets,
                              const int verbose_level) -> PnSourcePreselectionResult {
    const auto total_begin = std::chrono::steady_clock::now();
    auto out = PnSourcePreselectionResult{};
    out.stats.lambda_r = kRudyWeight;
    try {
        const int rows = static_cast<int>(hardware::Interposer::COB_ARRAY_HEIGHT);
        const int cols = static_cast<int>(hardware::Interposer::COB_ARRAY_WIDTH);
        const auto cell_unit_count = static_cast<std::size_t>(rows * cols) * kUnitCount;
        auto base_demand = std::Vector<double>(cell_unit_count, 0.0);
        auto candidates = std::Vector<Candidate>{};
        auto candidates_by_demand = std::Vector<std::Vector<std::size_t>>{};

        auto fixed_unit_load = std::map<std::pair<std::size_t, std::size_t>, std::size_t>{};
        auto fixed_residue_load =
            std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::size_t>{};

        for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
            const auto& net = nets[net_index];
            if (net.kind == RoutingNetKind::PNnet) {
                ++out.stats.pn_nets;
                for (std::size_t demand_index = 0; demand_index < net.demands.size();
                     ++demand_index) {
                    const auto& demand = net.demands[demand_index];
                    if (demand.sink.kind != GraphNodeRef::Kind::Bump) {
                        throw std::invalid_argument("V18 PN preselection requires bump sinks");
                    }
                    const auto demand_slot = candidates_by_demand.size();
                    candidates_by_demand.emplace_back();
                    const auto sink = endpoint_grid_point(graph, demand.sink);
                    auto unique_sources = std::set<std::size_t>{};
                    for (const auto source_index : demand.candidate_source_indices) {
                        if (!unique_sources.insert(source_index).second) {
                            continue;
                        }
                        if (source_index >= net.sources.size() ||
                            net.sources[source_index].kind != GraphNodeRef::Kind::Track) {
                            throw std::invalid_argument(
                                "V18 PN preselection requires physical Track candidates");
                        }
                        const auto source = endpoint_grid_point(graph, net.sources[source_index]);
                        const auto index = candidates.size();
                        candidates.push_back(
                            Candidate{net_index, demand_index, source_index,
                                      map_track(net.sources[source_index].track_index),
                                      demand.sink.bump, sink, source, hpwl(sink, source)});
                        candidates_by_demand[demand_slot].push_back(index);
                    }
                    if (candidates_by_demand.back().empty()) {
                        throw std::invalid_argument(
                            "V18 PN preselection demand has no physical source candidate");
                    }
                    ++out.stats.pn_bumps;
                }
                continue;
            }

            auto sink_points_by_source = std::map<std::size_t, std::Vector<GridPoint>>{};
            for (const auto& demand : net.demands) {
                if (demand.candidate_source_indices.size() != 1) {
                    throw std::invalid_argument(
                        "V18 PN preselection non-PN demand must have one source");
                }
                const auto source_index = demand.candidate_source_indices.front();
                const auto& source = net.sources.at(source_index);
                const auto sink_point = endpoint_grid_point(graph, demand.sink);
                sink_points_by_source[source_index].push_back(sink_point);
                if (net.kind == RoutingNetKind::Tnet) {
                    if (source.kind != GraphNodeRef::Kind::Track) {
                        throw std::invalid_argument(
                            "V18 PN preselection Tnet source must be a Track");
                    }
                    const auto unit = map_track(source.track_index);
                    if (demand.sink.kind == GraphNodeRef::Kind::Bump) {
                        const auto& bump = demand.sink.bump;
                        ++fixed_unit_load[{bump.TOB, unit}];
                        ++fixed_residue_load[{bump.TOB, bump.Bank, unit % kResidueCount}];
                        ++out.stats.fixed_tnet_bumps;
                    }
                }
            }
            for (const auto& [source_index, sink_points] : sink_points_by_source) {
                const auto& source = net.sources.at(source_index);
                const auto source_point = endpoint_grid_point(graph, source);
                auto corner_min = source_point;
                auto corner_max = source_point;
                for (const auto point : sink_points) {
                    corner_min.row = std::min(corner_min.row, point.row);
                    corner_min.col = std::min(corner_min.col, point.col);
                    corner_max.row = std::max(corner_max.row, point.row);
                    corner_max.col = std::max(corner_max.col, point.col);
                }
                if (net.kind == RoutingNetKind::Tnet) {
                    add_rudy(base_demand, corner_min, corner_max, map_track(source.track_index),
                             1.0, rows, cols);
                } else {
                    for (std::size_t unit = 0; unit < kUnitCount; ++unit) {
                        add_rudy(base_demand, corner_min, corner_max, unit,
                                 1.0 / static_cast<double>(kUnitCount), rows, cols);
                    }
                }
            }
        }

        if (out.stats.pn_nets == 0) {
            out.ok = true;
            out.message = "no PNnet";
            out.nets = nets;
            debug::info("V18 PN source preselection: skipped (PNnets=0)");
            return out;
        }

#ifndef USE_HIGHS
        out.message = "HiGHS backend is unavailable";
        return out;
#else
        out.stats.candidates = candidates.size();
        out.stats.lambda_a = median_positive_distance_gap(candidates, candidates_by_demand);

        auto nearest_reference = base_demand;
        for (const auto& demand_candidates : candidates_by_demand) {
            const auto best =
                *std::min_element(demand_candidates.begin(), demand_candidates.end(),
                                  [&](const auto lhs, const auto rhs) {
                                      const auto& l = candidates[lhs];
                                      const auto& r = candidates[rhs];
                                      return std::tie(l.distance, l.unit, l.source_index) <
                                             std::tie(r.distance, r.unit, r.source_index);
                                  });
            const auto& candidate = candidates[best];
            add_rudy(nearest_reference, candidate.sink, candidate.source, candidate.unit, 1.0, rows,
                     cols);
        }
        double reference_sum = 0.0;
        for (const double demand : nearest_reference) {
            reference_sum += demand;
        }
        out.stats.alpha = reference_sum / static_cast<double>(cell_unit_count);

        const auto build_begin = std::chrono::steady_clock::now();
        auto mip = SmallMip{verbose_level};
        for (auto& candidate : candidates) {
            candidate.variable = mip.add_binary(static_cast<double>(candidate.distance));
        }

        auto activations = std::Vector<Activation>{};
        auto activation_by_key = std::map<std::pair<std::size_t, std::size_t>, std::size_t>{};
        for (std::size_t candidate_index = 0; candidate_index < candidates.size();
             ++candidate_index) {
            const auto& candidate = candidates[candidate_index];
            const auto key = std::pair{candidate.net_index, candidate.source_index};
            auto [it, inserted] = activation_by_key.emplace(key, activations.size());
            if (inserted) {
                activations.push_back(Activation{candidate.net_index,
                                                 candidate.source_index,
                                                 mip.add_binary(out.stats.lambda_a),
                                                 {}});
            }
            activations[it->second].candidate_indices.push_back(candidate_index);
        }
        out.stats.source_activations = activations.size();

        for (const auto& demand_candidates : candidates_by_demand) {
            auto terms = std::Vector<std::pair<int, double>>{};
            terms.reserve(demand_candidates.size());
            for (const auto index : demand_candidates) {
                terms.emplace_back(candidates[index].variable, 1.0);
            }
            mip.add_row(1.0, 1.0, terms);
        }
        for (const auto& activation : activations) {
            auto support = std::Vector<std::pair<int, double>>{{activation.variable, 1.0}};
            for (const auto candidate_index : activation.candidate_indices) {
                const int y = candidates[candidate_index].variable;
                mip.add_row(-kHighsInf, 0.0, {{y, 1.0}, {activation.variable, -1.0}});
                support.emplace_back(y, -1.0);
            }
            mip.add_row(-kHighsInf, 0.0, support);
        }

        auto pn_by_tob_unit = std::map<std::pair<std::size_t, std::size_t>, std::Vector<int>>{};
        auto pn_by_tob_bank_residue =
            std::map<std::tuple<std::size_t, std::size_t, std::size_t>, std::Vector<int>>{};
        for (const auto& candidate : candidates) {
            pn_by_tob_unit[{candidate.bump.TOB, candidate.unit}].push_back(candidate.variable);
            pn_by_tob_bank_residue[{candidate.bump.TOB, candidate.bump.Bank,
                                    candidate.unit % kResidueCount}]
                .push_back(candidate.variable);
        }
        auto tob_unit_keys = std::set<std::pair<std::size_t, std::size_t>>{};
        for (const auto& [key, _] : fixed_unit_load) {
            tob_unit_keys.insert(key);
        }
        for (const auto& [key, _] : pn_by_tob_unit) {
            tob_unit_keys.insert(key);
        }
        for (const auto& key : tob_unit_keys) {
            const auto& variables = pn_by_tob_unit[key];
            const auto fixed = fixed_unit_load[key];
            auto terms = std::Vector<std::pair<int, double>>{};
            terms.reserve(variables.size());
            for (const int variable : variables) {
                terms.emplace_back(variable, 1.0);
            }
            mip.add_row(-kHighsInf, 8.0 - static_cast<double>(fixed), terms);
        }
        auto residue_keys = std::set<std::tuple<std::size_t, std::size_t, std::size_t>>{};
        for (const auto& [key, _] : fixed_residue_load) {
            residue_keys.insert(key);
        }
        for (const auto& [key, _] : pn_by_tob_bank_residue) {
            residue_keys.insert(key);
        }
        for (const auto& key : residue_keys) {
            const auto& variables = pn_by_tob_bank_residue[key];
            const auto fixed = fixed_residue_load[key];
            auto terms = std::Vector<std::pair<int, double>>{};
            terms.reserve(variables.size());
            for (const int variable : variables) {
                terms.emplace_back(variable, 1.0);
            }
            mip.add_row(-kHighsInf, 8.0 - static_cast<double>(fixed), terms);
        }

        auto rudy_terms = std::Vector<std::Vector<std::pair<int, double>>>(cell_unit_count);
        for (const auto& candidate : candidates) {
            if (candidate.distance == 0) {
                continue;
            }
            const int row_lo = std::min(candidate.sink.row, candidate.source.row);
            const int row_hi = std::max(candidate.sink.row, candidate.source.row);
            const int col_lo = std::min(candidate.sink.col, candidate.source.col);
            const int col_hi = std::max(candidate.sink.col, candidate.source.col);
            const auto area =
                static_cast<double>(row_hi - row_lo + 1) * static_cast<double>(col_hi - col_lo + 1);
            const double rho = static_cast<double>(candidate.distance) / area;
            for (int row = row_lo; row <= row_hi; ++row) {
                for (int col = col_lo; col <= col_hi; ++col) {
                    if (row < 0 || row >= rows || col < 0 || col >= cols) {
                        continue;
                    }
                    const auto index = cell_index(row, col, cols) * kUnitCount + candidate.unit;
                    rudy_terms[index].emplace_back(candidate.variable, rho);
                }
            }
        }
        for (std::size_t index = 0; index < rudy_terms.size(); ++index) {
            if (rudy_terms[index].empty()) {
                continue;
            }
            const int overflow = mip.add_continuous(out.stats.lambda_r);
            ++out.stats.overflow_vars;
            auto terms = std::Vector<std::pair<int, double>>{{overflow, 1.0}};
            terms.reserve(rudy_terms[index].size() + 1);
            for (const auto& [variable, rho] : rudy_terms[index]) {
                terms.emplace_back(variable, -rho);
            }
            mip.add_row(base_demand[index] - out.stats.alpha, kHighsInf, terms);
        }
        const std::size_t exactly_one_constraints = candidates_by_demand.size();
        const std::size_t activation_constraints = candidates.size() + activations.size();
        const std::size_t tob_unit_constraints = tob_unit_keys.size();
        const std::size_t residue_constraints = residue_keys.size();
        const std::size_t rudy_constraints = out.stats.overflow_vars;
        const auto build_end = std::chrono::steady_clock::now();
        out.stats.constraints = mip.constraints();
        if (exactly_one_constraints + activation_constraints + tob_unit_constraints
                + residue_constraints + rudy_constraints
            != out.stats.constraints) {
            throw std::logic_error("V18 PN preselection constraint statistics do not reconcile");
        }
        out.stats.build_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(build_end - build_begin).count();
        debug::info_fmt("V18 PN source preselection model built: PNnets={} bumps={} "
                        "candidates={} A={} O={} vars={} constraints={} fixed_Tnet_bumps={} "
                        "lambda_A={:.3f} lambda_R={:.3f} alpha={:.6f} build_ms={}",
                        out.stats.pn_nets, out.stats.pn_bumps, out.stats.candidates,
                        out.stats.source_activations, out.stats.overflow_vars, mip.variables(),
                        out.stats.constraints, out.stats.fixed_tnet_bumps, out.stats.lambda_a,
                        out.stats.lambda_r, out.stats.alpha, out.stats.build_ms);
        if (verbose_level >= 1) {
            debug::info("========== V18 PN source preselection ILP stats (-v) ==========");
            debug::info_fmt("  Y   (bump/unit/source choice) : {}", candidates.size());
            debug::info_fmt("  A   (physical source-tree)    : {}", activations.size());
            debug::info_fmt("  O   (cell/unit overflow)      : {}", out.stats.overflow_vars);
            debug::info_fmt("  [1] Y exactly-one             : {}", exactly_one_constraints);
            debug::info_fmt("  [2] Y/A activation            : {}", activation_constraints);
            debug::info_fmt("  [3] TOB unit <= 8             : {}", tob_unit_constraints);
            debug::info_fmt("  [4] TOB bank-residue <= 8     : {}", residue_constraints);
            debug::info_fmt("  [5] unit-aware RUDY overflow  : {}", rudy_constraints);
            debug::info_fmt("  total variables               : {}", mip.variables());
            debug::info_fmt("  total constraints             : {}", mip.constraints());
            debug::info("===============================================================");
        }

        const auto solve_begin = std::chrono::steady_clock::now();
        const auto status = mip.solve();
        const auto solve_end = std::chrono::steady_clock::now();
        out.stats.solve_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(solve_end - solve_begin).count();
        out.message = mip.status();
        if (status != HighsModelStatus::kOptimal) {
            debug::error_fmt("V18 PN source preselection failed: status={} solve_ms={}",
                             out.message, out.stats.solve_ms);
            return out;
        }

        const auto& values = mip.values();
        auto selected = std::Vector<bool>(candidates.size(), false);
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            selected[index] = values.at(static_cast<std::size_t>(candidates[index].variable)) > 0.5;
        }
        out.nets = transformed_nets(nets, candidates, selected);
        out.stats.source_trees = static_cast<std::size_t>(std::count_if(
            activations.begin(), activations.end(), [&](const Activation& activation) {
                return values.at(static_cast<std::size_t>(activation.variable)) > 0.5;
            }));
        out.stats.objective = mip.objective();
        out.ok = true;
        out.message = "Optimal";
        const auto total_end = std::chrono::steady_clock::now();
        out.stats.total_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_begin).count();
        debug::info_fmt("V18 PN source preselection summary: status=OPTIMAL "
                        "source_trees={} transformed_nets={} objective={:.3f} "
                        "total_ms={} build_ms={} solve_ms={}",
                        out.stats.source_trees, out.nets.size(), out.stats.objective,
                        out.stats.total_ms, out.stats.build_ms, out.stats.solve_ms);
        return out;
#endif
    } catch (const std::exception& error) {
        out.ok = false;
        out.message = error.what();
        debug::error_fmt("V18 PN source preselection exception: {}", out.message);
        return out;
    }
}

} // namespace PR_tool
