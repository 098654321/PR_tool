#include "global_route_v17/global_router.hh"

#include "common/hw_map.hh"

#include <algorithm>
#include <array>
#include <chrono>
#include <debug/debug.hh>
#include <format>
#include <optional>
#include <set>
#include <stdexcept>
#include <tuple>

#ifdef USE_HIGHS
#include <Highs.h>
#endif

namespace PR_tool {

auto GlobalRouteStats::ModelBreakdown::total_variables() const -> std::size_t {
    return q_vars + x_vars + z_vars + w_vars + f_vars + source_choice_vars;
}

auto GlobalRouteStats::ModelBreakdown::total_constraints() const -> std::size_t {
    return q_exactly_one + source_exactly_one + w_linearization
        + flow_conservation + flow_implies_channel + source_implies_channel
        + terminal_channel + channel_flow_support + pn_source_unit_coupling
        + pn_x_implies_net_channel + pn_net_channel_support
        + channel_unit_capacity + tob_unit_capacity + tob_bank_residue_capacity
        + sync_bus_equal_length;
}

namespace {

struct SourceOption {
    std::size_t source_index{0};
    int node{-1};
    int channel{-1};
    std::size_t unit{0};
};

struct CommodityData {
    PairKey key;
    std::size_t owner{0};
    int sink_node{-1};
    int sink_channel{-1};
    std::Vector<SourceOption> source_options;
    bool variable_source{false};
};

struct OwnerData {
    GlobalUnitOwnerKey key;
    std::size_t net_index{0};
    std::size_t source_index{0};
    bool has_sat_source{false};
    std::optional<std::size_t> fixed_unit;
    std::array<bool, 16> allowed_units {};
    std::set<Bump_coord> bumps;
    std::Vector<std::size_t> commodity_indices;
};

struct PreparedProblem {
    std::Vector<OwnerData> owners;
    std::Vector<CommodityData> commodities;
    std::map<GlobalUnitOwnerKey, std::size_t> owner_index_by_key;
    std::map<std::size_t, std::Vector<std::size_t>> bus_owner_indices_by_net;
    std::map<std::size_t, std::Vector<std::size_t>> pn_owner_indices_by_net;
};

auto channel_coord(const UnifiedNode& node) -> GlobalChannelCoord {
    return GlobalChannelCoord {node.track_dir, node.track_row, node.track_col};
}

auto endpoint_channel_coord(const GraphNodeRef& endpoint) -> GlobalChannelCoord {
    if (endpoint.kind == GraphNodeRef::Kind::Track) {
        return GlobalChannelCoord {
            endpoint.track_coord.dir == hardware::TrackDirection::Horizontal ? 0 : 1,
            static_cast<int>(endpoint.track_coord.row),
            static_cast<int>(endpoint.track_coord.col)};
    }
    if (endpoint.kind == GraphNodeRef::Kind::Bump) {
        const auto anchor = tob_anchor_cob(endpoint.bump.TOB);
        return GlobalChannelCoord {
            1,
            static_cast<int>(anchor.row),
            static_cast<int>(anchor.col)};
    }
    throw std::invalid_argument("V17 Global Routing endpoint must be Track or Bump");
}

auto endpoint_channel_id(
    const GlobalChannelGraph& graph,
    const GraphNodeRef& endpoint
) -> int {
    const auto coord = endpoint_channel_coord(endpoint);
    const auto it = graph.channel_id_by_coord.find(coord);
    if (it == graph.channel_id_by_coord.end()) {
        throw std::invalid_argument(std::format(
            "V17 Global Routing endpoint channel is missing: dir={} row={} col={}",
            coord.dir,
            coord.row,
            coord.col));
    }
    return it->second;
}

auto port_key(const GraphNodeRef& endpoint) -> GlobalPortKey {
    if (endpoint.kind != GraphNodeRef::Kind::Track) {
        throw std::invalid_argument("V17 Global Routing port key requires a Track endpoint");
    }
    return GlobalPortKey {
        endpoint.track_coord.dir == hardware::TrackDirection::Horizontal ? 0 : 1,
        static_cast<int>(endpoint.track_coord.row),
        static_cast<int>(endpoint.track_coord.col),
        endpoint.track_index};
}

auto endpoint_node_id(
    const GlobalChannelGraph& graph,
    const GraphNodeRef& endpoint
) -> int {
    if (endpoint.kind == GraphNodeRef::Kind::Bump) {
        const auto it = graph.tob_node_by_tob.find(endpoint.bump.TOB);
        if (it != graph.tob_node_by_tob.end()) {
            return it->second;
        }
        throw std::invalid_argument("V17 Global Routing TOB terminal is missing");
    }
    if (endpoint.kind == GraphNodeRef::Kind::Track) {
        const auto it = graph.port_node_by_key.find(port_key(endpoint));
        if (it != graph.port_node_by_key.end()) {
            return it->second;
        }
        throw std::invalid_argument("V17 Global Routing port terminal is missing");
    }
    throw std::invalid_argument("V17 Global Routing endpoint must be Track or Bump");
}

auto adjacent_cobs(
    const GlobalChannelCoord& channel,
    const int rows,
    const int cols
) -> std::Vector<std::pair<int, int>> {
    auto result = std::Vector<std::pair<int, int>> {};
    const auto add = [&](const int row, const int col) {
        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            result.emplace_back(row, col);
        }
    };
    if (channel.dir == 0) {
        add(channel.row, channel.col - 1);
        add(channel.row, channel.col);
    }
    else {
        add(channel.row - 1, channel.col);
        add(channel.row, channel.col);
    }
    return result;
}

auto owner_key_for(
    const RoutingNet& net,
    const RoutingDemand& demand,
    const std::size_t source_index
) -> GlobalUnitOwnerKey {
    if (net.kind == RoutingNetKind::PNnet) {
        return GlobalUnitOwnerKey {net.net_id, demand.demand_id};
    }
    return GlobalUnitOwnerKey {net.net_id, source_index};
}

auto add_bump_if_present(std::set<Bump_coord>& bumps, const GraphNodeRef& endpoint) -> void {
    if (endpoint.kind == GraphNodeRef::Kind::Bump) {
        bumps.insert(endpoint.bump);
    }
}

auto detailed_endpoint_hops(const GraphNodeRef& endpoint) -> int {
    return endpoint.kind == GraphNodeRef::Kind::Bump ? 3 : 0;
}

auto prepare_problem(
    const GlobalChannelGraph& graph,
    const std::Vector<RoutingNet>& nets
) -> PreparedProblem {
    auto problem = PreparedProblem {};
    for (std::size_t net_index = 0; net_index < nets.size(); ++net_index) {
        const auto& net = nets[net_index];
        for (const auto& demand : net.demands) {
            const std::size_t source_index = net.kind == RoutingNetKind::PNnet
                ? 0
                : demand.candidate_source_indices.front();
            const auto owner_key = owner_key_for(net, demand, source_index);
            auto [owner_it, inserted] = problem.owner_index_by_key.emplace(
                owner_key,
                problem.owners.size());
            if (inserted) {
                auto owner = OwnerData {};
                owner.key = owner_key;
                owner.net_index = net_index;
                owner.source_index = source_index;
                owner.has_sat_source = net.kind != RoutingNetKind::PNnet;
                if (net.kind == RoutingNetKind::Tnet) {
                    owner.fixed_unit = map_track(net.sources.at(source_index).track_index);
                    owner.allowed_units[owner.fixed_unit.value()] = true;
                }
                else if (net.kind == RoutingNetKind::Bnet) {
                    owner.allowed_units.fill(true);
                    add_bump_if_present(owner.bumps, net.sources.at(source_index));
                }
                problem.owners.push_back(std::move(owner));
                if (net.kind == RoutingNetKind::PNnet) {
                    problem.pn_owner_indices_by_net[net_index].push_back(owner_it->second);
                }
            }
            auto& owner = problem.owners[owner_it->second];
            add_bump_if_present(owner.bumps, demand.sink);

            auto commodity = CommodityData {};
            commodity.key = PairKey {net.net_id, demand.demand_id, source_index};
            commodity.owner = owner_it->second;
            commodity.sink_node = endpoint_node_id(graph, demand.sink);
            commodity.sink_channel = endpoint_channel_id(graph, demand.sink);
            commodity.variable_source = net.kind == RoutingNetKind::PNnet;
            for (const std::size_t candidate : demand.candidate_source_indices) {
                if (candidate >= net.sources.size()) {
                    throw std::invalid_argument("V17 Global Routing source candidate is out of range");
                }
                const auto& source = net.sources[candidate];
                const auto unit = source.kind == GraphNodeRef::Kind::Track
                    ? map_track(source.track_index)
                    : std::size_t {0};
                commodity.source_options.push_back(SourceOption {
                    candidate,
                    endpoint_node_id(graph, source),
                    endpoint_channel_id(graph, source),
                    unit});
                if (net.kind == RoutingNetKind::PNnet) {
                    owner.allowed_units[unit] = true;
                }
            }
            if (commodity.source_options.empty()) {
                throw std::invalid_argument("V17 Global Routing commodity has no source option");
            }
            const auto commodity_index = problem.commodities.size();
            problem.commodities.push_back(std::move(commodity));
            owner.commodity_indices.push_back(commodity_index);
            if (net.is_sync_bus) {
                problem.bus_owner_indices_by_net[net.net_id].push_back(owner_it->second);
            }
        }
    }
    for (auto& [_, owners] : problem.bus_owner_indices_by_net) {
        std::sort(owners.begin(), owners.end());
        owners.erase(std::unique(owners.begin(), owners.end()), owners.end());
    }
    return problem;
}

#ifdef USE_HIGHS

class HighsMipBuilder {
public:
    explicit HighsMipBuilder(const int verbose_level) {
        highs_.setOptionValue("output_flag", verbose_level >= 3);
        highs_.setOptionValue("mip_rel_gap", 0.0);
    }

    auto add_binary(const double cost = 0.0, const double lower = 0.0, const double upper = 1.0)
        -> int {
        const int column = static_cast<int>(variables_++);
        check(highs_.addCol(cost, lower, upper, 0, nullptr, nullptr), "addCol");
        check(
            highs_.changeColIntegrality(column, HighsVarType::kInteger),
            "changeColIntegrality");
        return column;
    }

    auto add_row(
        const double lower,
        const double upper,
        const std::Vector<std::pair<int, double>>& terms
    ) -> void {
        auto indices = std::Vector<HighsInt> {};
        auto values = std::Vector<double> {};
        indices.reserve(terms.size());
        values.reserve(terms.size());
        for (const auto& [column, value] : terms) {
            if (value == 0.0) {
                continue;
            }
            indices.push_back(static_cast<HighsInt>(column));
            values.push_back(value);
        }
        check(
            highs_.addRow(
                lower,
                upper,
                static_cast<HighsInt>(indices.size()),
                indices.empty() ? nullptr : indices.data(),
                values.empty() ? nullptr : values.data()),
            "addRow");
        ++constraints_;
    }

    auto solve() -> HighsModelStatus {
        check(highs_.run(), "run");
        return highs_.getModelStatus();
    }

    [[nodiscard]] auto solution() const -> const HighsSolution& {
        return highs_.getSolution();
    }

    [[nodiscard]] auto status_string() const -> std::String {
        return highs_.modelStatusToString(highs_.getModelStatus());
    }

    [[nodiscard]] auto variables() const -> std::size_t { return variables_; }
    [[nodiscard]] auto constraints() const -> std::size_t { return constraints_; }

private:
    static auto check(const HighsStatus status, const char* operation) -> void {
        if (status == HighsStatus::kError) {
            throw std::runtime_error(std::format("HiGHS {} failed", operation));
        }
    }

    Highs highs_;
    std::size_t variables_{0};
    std::size_t constraints_{0};
};

struct ModelVars {
    std::Vector<std::array<int, 16>> q;
    std::Vector<std::Vector<int>> x;
    std::Vector<std::Vector<int>> z;
    std::Vector<std::Vector<std::array<int, 16>>> w;
    std::Vector<std::Vector<int>> f;
    std::Vector<std::Vector<int>> source_choice;
};

auto log_global_route_model_stats(
    const GlobalRouteStats::ModelBreakdown& stats,
    const GlobalChannelGraph& graph,
    const std::size_t owners,
    const std::size_t commodities,
    const std::size_t total_variables,
    const std::size_t total_constraints
) -> void {
    debug::info("========== V17 Global Routing ILP model stats (-v) ==========");
    debug::info("Model dimensions:");
    debug::info_fmt("  graph nodes                     : {}", graph.nodes.size());
    debug::info_fmt("    COB nodes                     : {}", graph.cob_node_count);
    debug::info_fmt("    TOB terminal nodes            : {}", graph.tob_terminal_node_count);
    debug::info_fmt("    port terminal nodes           : {}", graph.port_terminal_node_count);
    debug::info_fmt("    boundary terminal nodes       : {}", graph.boundary_terminal_node_count);
    debug::info_fmt("  physical Channel resources      : {}", graph.channels.size());
    debug::info_fmt("  directed traversal arcs         : {}", graph.arcs.size());
    debug::info_fmt("  unit/route owners               : {}", owners);
    debug::info_fmt("  source/sink demands             : {}", commodities);

    debug::info("Binary variables:");
    debug::info_fmt("  Q   (owner COBUnit)             : {}", stats.q_vars);
    debug::info_fmt("  X   (owner Channel occupancy)   : {}", stats.x_vars);
    debug::info_fmt("  Z   (PNnet Channel union)       : {}", stats.z_vars);
    debug::info_fmt("  W   (X and Q)                   : {}", stats.w_vars);
    debug::info_fmt("  W dense slots                   : {}", stats.w_dense_slots);
    if (stats.w_dense_slots > 0) {
        debug::info_fmt(
            "  W active / dense ratio          : {:.3f}",
            static_cast<double>(stats.w_vars)
                / static_cast<double>(stats.w_dense_slots));
    }
    debug::info_fmt("  F   (commodity directed flow)   : {}", stats.f_vars);
    debug::info_fmt("  S   (PN candidate source)       : {}", stats.source_choice_vars);
    debug::info_fmt("  total MIP variables             : {}", total_variables);

    debug::info("Linear constraints by category:");
    debug::info_fmt("  [1]  Q exactly-one              : {}", stats.q_exactly_one);
    debug::info_fmt("  [2]  PN source exactly-one      : {}", stats.source_exactly_one);
    debug::info_fmt("  [3]  W = X and Q                : {}", stats.w_linearization);
    debug::info_fmt("  [4]  commodity flow balance     : {}", stats.flow_conservation);
    debug::info_fmt("  [5]  F implies Channel X        : {}", stats.flow_implies_channel);
    debug::info_fmt("  [6]  PN source implies X        : {}", stats.source_implies_channel);
    debug::info_fmt("  [7]  terminal X fixed           : {}", stats.terminal_channel);
    debug::info_fmt("  [8]  X requires flow/source     : {}", stats.channel_flow_support);
    debug::info_fmt("  [9]  PN source-unit coupling    : {}", stats.pn_source_unit_coupling);
    debug::info_fmt("  [10] PN X implies net Z         : {}", stats.pn_x_implies_net_channel);
    debug::info_fmt("  [11] PN Z requires owner X      : {}", stats.pn_net_channel_support);
    debug::info_fmt("  [12] Channel-unit capacity      : {}", stats.channel_unit_capacity);
    debug::info_fmt("  [13] TOB unit <= 8 (necessary)  : {}", stats.tob_unit_capacity);
    debug::info_fmt(
        "  [14] TOB bank-residue <= 8      : {}",
        stats.tob_bank_residue_capacity);
    debug::info_fmt("  [15] SyncBus Channel equality   : {}", stats.sync_bus_equal_length);
    debug::info_fmt("  total MIP constraints           : {}", total_constraints);
    debug::info("=============================================================");
}

auto add_exactly_one(HighsMipBuilder& mip, const std::Vector<int>& variables) -> void {
    auto terms = std::Vector<std::pair<int, double>> {};
    for (const int variable : variables) {
        terms.emplace_back(variable, 1.0);
    }
    mip.add_row(1.0, 1.0, terms);
}

auto build_and_solve(
    const GlobalChannelGraph& graph,
    const std::Vector<RoutingNet>& nets,
    const PreparedProblem& problem,
    const int verbose_level,
    const GlobalRouteCapacityMode capacity_mode,
    GlobalRouteResult& out
) -> void {
    const auto build_begin = std::chrono::steady_clock::now();
    auto mip = HighsMipBuilder {verbose_level};
    auto vars = ModelVars {};
    const auto owner_count = problem.owners.size();
    const auto channel_count = graph.channels.size();
    const auto arc_count = graph.arcs.size();
    const bool use_capacity_cuts = capacity_mode == GlobalRouteCapacityMode::IterativeCuts;
    auto& model_stats = out.stats.model;
    out.stats.capacity_cuts_enabled = use_capacity_cuts;
    model_stats.w_dense_slots = owner_count * channel_count * 16;

    vars.q.resize(owner_count);
    vars.x.resize(owner_count);
    vars.w.resize(owner_count);
    for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
        const auto& owner = problem.owners[owner_index];
        auto q_vars = std::Vector<int> {};
        for (std::size_t unit = 0; unit < 16; ++unit) {
            double lower = 0.0;
            double upper = owner.allowed_units[unit] ? 1.0 : 0.0;
            if (owner.fixed_unit.has_value()) {
                lower = owner.fixed_unit.value() == unit ? 1.0 : 0.0;
                upper = lower;
            }
            vars.q[owner_index][unit] = mip.add_binary(0.0, lower, upper);
            ++model_stats.q_vars;
            q_vars.push_back(vars.q[owner_index][unit]);
        }
        add_exactly_one(mip, q_vars);
        ++model_stats.q_exactly_one;

        vars.x[owner_index].reserve(channel_count);
        vars.w[owner_index].resize(channel_count);
        for (std::size_t channel = 0; channel < channel_count; ++channel) {
            const double x_cost = nets[owner.net_index].kind == RoutingNetKind::PNnet
                ? 0.0
                : 1.0;
            vars.x[owner_index].push_back(mip.add_binary(x_cost));
            ++model_stats.x_vars;
            vars.w[owner_index][channel].fill(-1);
            if (owner.fixed_unit.has_value() || use_capacity_cuts) {
                continue;
            }
            for (std::size_t unit = 0; unit < 16; ++unit) {
                if (!owner.allowed_units[unit]) {
                    continue;
                }
                const int w = mip.add_binary();
                ++model_stats.w_vars;
                vars.w[owner_index][channel][unit] = w;
                const int x = vars.x[owner_index][channel];
                const int q = vars.q[owner_index][unit];
                mip.add_row(-kHighsInf, 0.0, {{w, 1.0}, {x, -1.0}});
                mip.add_row(-kHighsInf, 0.0, {{w, 1.0}, {q, -1.0}});
                mip.add_row(-1.0, kHighsInf, {{w, 1.0}, {x, -1.0}, {q, -1.0}});
                model_stats.w_linearization += 3;
            }
        }
    }

    vars.z.resize(nets.size());
    for (const auto& [net_index, owner_indices] : problem.pn_owner_indices_by_net) {
        vars.z[net_index].reserve(channel_count);
        for (std::size_t channel = 0; channel < channel_count; ++channel) {
            const int z = mip.add_binary(1.0);
            vars.z[net_index].push_back(z);
            ++model_stats.z_vars;
            auto support_terms = std::Vector<std::pair<int, double>> {{z, 1.0}};
            for (const std::size_t owner_index : owner_indices) {
                const int x = vars.x[owner_index][channel];
                mip.add_row(-kHighsInf, 0.0, {{x, 1.0}, {z, -1.0}});
                ++model_stats.pn_x_implies_net_channel;
                support_terms.emplace_back(x, -1.0);
            }
            mip.add_row(-kHighsInf, 0.0, support_terms);
            ++model_stats.pn_net_channel_support;
        }
    }

    vars.f.resize(problem.commodities.size());
    vars.source_choice.resize(problem.commodities.size());
    for (std::size_t commodity_index = 0;
         commodity_index < problem.commodities.size();
        ++commodity_index) {
        const auto& commodity = problem.commodities[commodity_index];
        auto allowed_port_nodes = std::set<int> {};
        if (graph.nodes[static_cast<std::size_t>(commodity.sink_node)].kind
            == GlobalRouteNodeKind::PortTerminal) {
            allowed_port_nodes.insert(commodity.sink_node);
        }
        for (const auto& option : commodity.source_options) {
            if (graph.nodes[static_cast<std::size_t>(option.node)].kind
                == GlobalRouteNodeKind::PortTerminal) {
                allowed_port_nodes.insert(option.node);
            }
        }
        vars.f[commodity_index].assign(arc_count, -1);
        for (std::size_t arc = 0; arc < arc_count; ++arc) {
            const int restricted_port = graph.arcs[arc].restricted_port_node;
            if (restricted_port >= 0 && !allowed_port_nodes.contains(restricted_port)) {
                continue;
            }
            vars.f[commodity_index][arc] = mip.add_binary();
            ++model_stats.f_vars;
        }
        if (commodity.variable_source) {
            for (std::size_t option = 0; option < commodity.source_options.size(); ++option) {
                vars.source_choice[commodity_index].push_back(mip.add_binary());
                ++model_stats.source_choice_vars;
            }
            add_exactly_one(mip, vars.source_choice[commodity_index]);
            ++model_stats.source_exactly_one;
        }
    }

    for (std::size_t commodity_index = 0;
         commodity_index < problem.commodities.size();
         ++commodity_index) {
        const auto& commodity = problem.commodities[commodity_index];
        const auto owner_index = commodity.owner;
        for (std::size_t node = 0; node < graph.nodes.size(); ++node) {
            auto terms = std::Vector<std::pair<int, double>> {};
            for (const int arc_id : graph.out_arc_ids[node]) {
                const int f = vars.f[commodity_index][static_cast<std::size_t>(arc_id)];
                if (f >= 0) {
                    terms.emplace_back(f, 1.0);
                }
            }
            for (const int arc_id : graph.in_arc_ids[node]) {
                const int f = vars.f[commodity_index][static_cast<std::size_t>(arc_id)];
                if (f >= 0) {
                    terms.emplace_back(f, -1.0);
                }
            }
            double rhs = node == static_cast<std::size_t>(commodity.sink_node) ? -1.0 : 0.0;
            if (commodity.variable_source) {
                for (std::size_t option = 0; option < commodity.source_options.size(); ++option) {
                    if (commodity.source_options[option].node == static_cast<int>(node)) {
                        terms.emplace_back(vars.source_choice[commodity_index][option], -1.0);
                    }
                }
            }
            else if (commodity.source_options.front().node == static_cast<int>(node)) {
                rhs += 1.0;
            }
            mip.add_row(rhs, rhs, terms);
            ++model_stats.flow_conservation;
        }

        for (std::size_t arc_id = 0; arc_id < arc_count; ++arc_id) {
            const auto& arc = graph.arcs[arc_id];
            const int f = vars.f[commodity_index][arc_id];
            if (f < 0) {
                continue;
            }
            mip.add_row(
                -kHighsInf,
                0.0,
                {{f, 1.0},
                 {vars.x[owner_index][static_cast<std::size_t>(arc.channel)], -1.0}});
            ++model_stats.flow_implies_channel;
        }
        if (commodity.variable_source) {
            for (std::size_t option = 0; option < commodity.source_options.size(); ++option) {
                const int source_channel = commodity.source_options[option].channel;
                mip.add_row(
                    -kHighsInf,
                    0.0,
                    {{vars.source_choice[commodity_index][option], 1.0},
                     {vars.x[owner_index][static_cast<std::size_t>(source_channel)], -1.0}});
                ++model_stats.source_implies_channel;
            }
        }
    }

    for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
        const auto& owner = problem.owners[owner_index];
        auto terminal_channels = std::set<int> {};
        for (const std::size_t commodity_index : owner.commodity_indices) {
            const auto& commodity = problem.commodities[commodity_index];
            terminal_channels.insert(commodity.sink_channel);
            if (!commodity.variable_source) {
                terminal_channels.insert(commodity.source_options.front().channel);
            }
        }
        for (const int channel : terminal_channels) {
            mip.add_row(
                1.0,
                1.0,
                {{vars.x[owner_index][static_cast<std::size_t>(channel)], 1.0}});
            ++model_stats.terminal_channel;
        }
        for (std::size_t channel = 0; channel < channel_count; ++channel) {
            if (terminal_channels.contains(static_cast<int>(channel))) {
                continue;
            }
            auto terms = std::Vector<std::pair<int, double>> {
                {vars.x[owner_index][channel], 1.0}};
            for (const std::size_t commodity_index : owner.commodity_indices) {
                for (const int arc_id : graph.arc_ids_by_channel[channel]) {
                    const int f = vars.f[commodity_index][static_cast<std::size_t>(arc_id)];
                    if (f >= 0) {
                        terms.emplace_back(f, -1.0);
                    }
                }
                const auto& commodity = problem.commodities[commodity_index];
                if (commodity.variable_source) {
                    for (std::size_t option = 0; option < commodity.source_options.size(); ++option) {
                        if (commodity.source_options[option].channel == static_cast<int>(channel)) {
                            terms.emplace_back(vars.source_choice[commodity_index][option], -1.0);
                        }
                    }
                }
            }
            mip.add_row(-kHighsInf, 0.0, terms);
            ++model_stats.channel_flow_support;
        }
    }

    for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
        const auto& owner = problem.owners[owner_index];
        if (nets[owner.net_index].kind != RoutingNetKind::PNnet) {
            continue;
        }
        for (std::size_t unit = 0; unit < 16; ++unit) {
            auto terms = std::Vector<std::pair<int, double>> {{vars.q[owner_index][unit], 1.0}};
            for (const std::size_t commodity_index : owner.commodity_indices) {
                const auto& commodity = problem.commodities[commodity_index];
                for (std::size_t option = 0; option < commodity.source_options.size(); ++option) {
                    if (commodity.source_options[option].unit == unit) {
                        terms.emplace_back(vars.source_choice[commodity_index][option], -1.0);
                    }
                }
            }
            mip.add_row(0.0, 0.0, terms);
            ++model_stats.pn_source_unit_coupling;
        }
    }

    if (!use_capacity_cuts) {
        for (std::size_t channel = 0; channel < channel_count; ++channel) {
            for (std::size_t unit = 0; unit < 16; ++unit) {
                auto terms = std::Vector<std::pair<int, double>> {};
                for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
                    const auto& owner = problem.owners[owner_index];
                    if (owner.fixed_unit.has_value()) {
                        if (owner.fixed_unit.value() == unit) {
                            terms.emplace_back(vars.x[owner_index][channel], 1.0);
                        }
                        continue;
                    }
                    const int w = vars.w[owner_index][channel][unit];
                    if (w >= 0) {
                        terms.emplace_back(w, 1.0);
                    }
                }
                if (!terms.empty()) {
                    mip.add_row(-kHighsInf, 8.0, terms);
                    ++model_stats.channel_unit_capacity;
                }
            }
        }
    }

    for (std::size_t tob = 0; tob < hardware::Interposer::TOB_SIZE; ++tob) {
        for (std::size_t unit = 0; unit < 16; ++unit) {
            auto terms = std::Vector<std::pair<int, double>> {};
            for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
                const auto count = static_cast<double>(std::count_if(
                    problem.owners[owner_index].bumps.begin(),
                    problem.owners[owner_index].bumps.end(),
                    [&](const Bump_coord& bump) { return bump.TOB == tob; }));
                if (count != 0.0) {
                    terms.emplace_back(vars.q[owner_index][unit], count);
                }
            }
            if (!terms.empty()) {
                mip.add_row(-kHighsInf, 8.0, terms);
                ++model_stats.tob_unit_capacity;
            }
        }
        for (std::size_t bank = 0; bank < 2; ++bank) {
            for (std::size_t residue = 0; residue < 8; ++residue) {
                auto terms = std::Vector<std::pair<int, double>> {};
                for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
                    const auto count = static_cast<double>(std::count_if(
                        problem.owners[owner_index].bumps.begin(),
                        problem.owners[owner_index].bumps.end(),
                        [&](const Bump_coord& bump) {
                            return bump.TOB == tob && bump.Bank == bank;
                        }));
                    if (count == 0.0) {
                        continue;
                    }
                    terms.emplace_back(vars.q[owner_index][residue], count);
                    terms.emplace_back(vars.q[owner_index][residue + 8], count);
                }
                if (!terms.empty()) {
                    mip.add_row(-kHighsInf, 8.0, terms);
                    ++model_stats.tob_bank_residue_capacity;
                }
            }
        }
    }

    for (const auto& [_, owner_indices] : problem.bus_owner_indices_by_net) {
        if (owner_indices.size() < 2) {
            continue;
        }
        const auto reference = owner_indices.front();
        for (std::size_t index = 1; index < owner_indices.size(); ++index) {
            auto terms = std::Vector<std::pair<int, double>> {};
            for (std::size_t channel = 0; channel < channel_count; ++channel) {
                terms.emplace_back(vars.x[reference][channel], 1.0);
                terms.emplace_back(vars.x[owner_indices[index]][channel], -1.0);
            }
            mip.add_row(0.0, 0.0, terms);
            ++model_stats.sync_bus_equal_length;
        }
    }

    const auto build_end = std::chrono::steady_clock::now();
    out.stats.variables = mip.variables();
    out.stats.constraints = mip.constraints();
    if (model_stats.total_variables() != out.stats.variables
        || model_stats.total_constraints() != out.stats.constraints) {
        throw std::logic_error(std::format(
            "V17 model-stat mismatch: variables={}/{} constraints={}/{}",
            model_stats.total_variables(),
            out.stats.variables,
            model_stats.total_constraints(),
            out.stats.constraints));
    }
    out.stats.build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        build_end - build_begin).count();
    debug::info_fmt(
        "V17 Global Routing model built: vars={} constraints={} build_ms={} owners={} demands={} capacity_mode={} TOB_filters=necessary-only(unit<=8,bank-residue<=8)",
        out.stats.variables,
        out.stats.constraints,
        out.stats.build_ms,
        owner_count,
        problem.commodities.size(),
        use_capacity_cuts ? "iterative-cuts(no-W)" : "dense-W");
    if (verbose_level >= 1) {
        log_global_route_model_stats(
            model_stats,
            graph,
            owner_count,
            problem.commodities.size(),
            out.stats.variables,
            out.stats.constraints);
    }

    auto added_cut_keys = std::set<
        std::tuple<std::size_t, std::size_t, std::array<std::size_t, 9>>> {};
    while (true) {
        const auto solve_begin = std::chrono::steady_clock::now();
        const auto status = mip.solve();
        const auto solve_end = std::chrono::steady_clock::now();
        out.stats.solve_ms += std::chrono::duration_cast<std::chrono::milliseconds>(
            solve_end - solve_begin).count();
        out.message = mip.status_string();
        if (status != HighsModelStatus::kOptimal) {
            out.stats.constraints = mip.constraints();
            debug::error_fmt(
                "V17 Global Routing failed: status={} solve_ms={} capacity_cut_rounds={} capacity_cuts={}",
                out.message,
                out.stats.solve_ms,
                out.stats.capacity_cut_rounds,
                out.stats.capacity_cuts);
            return;
        }
        if (!use_capacity_cuts) {
            break;
        }

        const auto& incumbent = mip.solution().col_value;
        const auto incumbent_selected = [&](const int variable) {
            return variable >= 0
                && static_cast<std::size_t>(variable) < incumbent.size()
                && incumbent[static_cast<std::size_t>(variable)] > 0.5;
        };
        std::size_t cuts_added = 0;
        std::size_t overloaded_resources = 0;
        for (std::size_t channel = 0; channel < channel_count; ++channel) {
            for (std::size_t unit = 0; unit < 16; ++unit) {
                auto loaded_owners = std::Vector<std::size_t> {};
                for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
                    if (incumbent_selected(vars.x[owner_index][channel])
                        && incumbent_selected(vars.q[owner_index][unit])) {
                        loaded_owners.push_back(owner_index);
                    }
                }
                if (loaded_owners.size() <= 8) {
                    continue;
                }
                ++overloaded_resources;
                auto subset = std::array<std::size_t, 9> {};
                std::copy_n(loaded_owners.begin(), subset.size(), subset.begin());
                if (!added_cut_keys.emplace(channel, unit, subset).second) {
                    throw std::logic_error(
                        "V18 capacity separator rediscovered an active violated cut");
                }
                auto terms = std::Vector<std::pair<int, double>> {};
                terms.reserve(2 * subset.size());
                for (const auto owner_index : subset) {
                    terms.emplace_back(vars.x[owner_index][channel], 1.0);
                    terms.emplace_back(vars.q[owner_index][unit], 1.0);
                }
                mip.add_row(-kHighsInf, 17.0, terms);
                ++model_stats.channel_unit_capacity;
                ++cuts_added;
            }
        }
        if (cuts_added == 0) {
            debug::info_fmt(
                "V18 capacity cuts converged: rounds={} cuts={} final_constraints={} solve_ms={}",
                out.stats.capacity_cut_rounds,
                out.stats.capacity_cuts,
                mip.constraints(),
                out.stats.solve_ms);
            break;
        }
        ++out.stats.capacity_cut_rounds;
        out.stats.capacity_cuts += cuts_added;
        out.stats.constraints = mip.constraints();
        debug::info_fmt(
            "V18 capacity cut round: round={} overloaded_channel_units={} cuts_added={} cumulative_cuts={} constraints={} cumulative_solve_ms={}",
            out.stats.capacity_cut_rounds,
            overloaded_resources,
            cuts_added,
            out.stats.capacity_cuts,
            out.stats.constraints,
            out.stats.solve_ms);
    }
    out.stats.constraints = mip.constraints();
    if (model_stats.total_variables() != mip.variables()
        || model_stats.total_constraints() != mip.constraints()) {
        throw std::logic_error(std::format(
            "V17 final model-stat mismatch: variables={}/{} constraints={}/{}",
            model_stats.total_variables(),
            mip.variables(),
            model_stats.total_constraints(),
            mip.constraints()));
    }

    const auto& values = mip.solution().col_value;
    const auto selected = [&](const int variable) {
        return variable >= 0
            && static_cast<std::size_t>(variable) < values.size()
            && values[static_cast<std::size_t>(variable)] > 0.5;
    };
    std::size_t tob_bump_wirelength = 0;
    auto bumps_by_net = std::map<std::size_t, std::set<Bump_coord>> {};
    for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
        std::size_t chosen_unit = 16;
        std::size_t channel_uses = 0;
        for (std::size_t unit = 0; unit < 16; ++unit) {
            if (selected(vars.q[owner_index][unit])) {
                chosen_unit = unit;
                break;
            }
        }
        if (chosen_unit >= 16) {
            throw std::runtime_error("HiGHS returned an owner without a COBUnit");
        }
        for (const int x : vars.x[owner_index]) {
            channel_uses += selected(x) ? 1 : 0;
        }
        out.unit_by_owner.emplace(problem.owners[owner_index].key, chosen_unit);
        out.channel_count_by_owner.emplace(problem.owners[owner_index].key, channel_uses);
        const auto& owner = problem.owners[owner_index];
        if (nets[owner.net_index].kind != RoutingNetKind::PNnet) {
            out.stats.objective += channel_uses;
        }
        bumps_by_net[owner.net_index].insert(owner.bumps.begin(), owner.bumps.end());
        if (verbose_level >= 1) {
            debug::info_fmt(
                "V17 Global Routing owner: net={} owner={} unit={} channels={}",
                problem.owners[owner_index].key.net_id,
                problem.owners[owner_index].key.owner_index,
                chosen_unit,
                channel_uses);
        }
        if (owner.fixed_unit.has_value() && owner.fixed_unit.value() != chosen_unit) {
            throw std::runtime_error("V17 fixed-unit extraction validation failed");
        }
        if (!owner.fixed_unit.has_value() && !use_capacity_cuts) {
            for (std::size_t channel = 0; channel < channel_count; ++channel) {
                for (std::size_t unit = 0; unit < 16; ++unit) {
                    const int w = vars.w[owner_index][channel][unit];
                    if (w < 0) {
                        continue;
                    }
                    const bool expected = selected(vars.x[owner_index][channel])
                        && chosen_unit == unit;
                    if (selected(w) != expected) {
                        throw std::runtime_error("V17 W=X-and-Q extraction validation failed");
                    }
                }
            }
        }
    }
    for (const auto& [net_index, owner_indices] : problem.pn_owner_indices_by_net) {
        std::size_t pn_net_channels = 0;
        for (std::size_t channel = 0; channel < channel_count; ++channel) {
            const bool expected = std::ranges::any_of(
                owner_indices,
                [&](const std::size_t owner_index) {
                    return selected(vars.x[owner_index][channel]);
                });
            const bool net_uses_channel = selected(vars.z[net_index][channel]);
            if (net_uses_channel != expected) {
                throw std::runtime_error("V17 PNnet Z union extraction validation failed");
            }
            pn_net_channels += net_uses_channel ? 1 : 0;
        }
        out.stats.objective += pn_net_channels;
        if (verbose_level >= 1) {
            debug::info_fmt(
                "V17 Global Routing PNnet union: net={} owners={} channels={}",
                nets[net_index].net_id,
                owner_indices.size(),
                pn_net_channels);
        }
    }
    for (const auto& [_, bumps] : bumps_by_net) {
        tob_bump_wirelength += bumps.size();
    }
    out.stats.estimated_wirelength = out.stats.objective + tob_bump_wirelength;
    debug::info_fmt(
        "V17 Global Routing Estimated wirelength: total={} channels={} tob_bumps={}",
        out.stats.estimated_wirelength,
        out.stats.objective,
        tob_bump_wirelength);

    for (std::size_t commodity_index = 0;
         commodity_index < problem.commodities.size();
         ++commodity_index) {
        const auto& commodity = problem.commodities[commodity_index];
        auto& guide = out.pair_channels[commodity.key];
        auto& selected_arc_ids = out.selected_arc_ids_by_pair[commodity.key];
        auto& pair_cobs = out.pair_cobs[commodity.key];
        guide.insert(graph.channels[static_cast<std::size_t>(commodity.sink_channel)]);
        int selected_source_node = commodity.source_options.front().node;
        std::size_t selected_source_index = commodity.source_options.front().source_index;
        if (commodity.variable_source) {
            bool found_source = false;
            for (std::size_t option = 0; option < commodity.source_options.size(); ++option) {
                if (!selected(vars.source_choice[commodity_index][option])) {
                    continue;
                }
                const auto& source = commodity.source_options[option];
                selected_source_node = source.node;
                selected_source_index = source.source_index;
                guide.insert(graph.channels[static_cast<std::size_t>(source.channel)]);
                out.selected_source_index_by_pair.emplace(commodity.key, source.source_index);
                found_source = true;
                break;
            }
            if (!found_source) {
                throw std::runtime_error("V17 PN commodity has no selected source");
            }
            out.selected_unit_by_pair.emplace(
                commodity.key,
                out.unit_by_owner.at(problem.owners[commodity.owner].key));
        }
        else {
            guide.insert(graph.channels[static_cast<std::size_t>(commodity.source_options.front().channel)]);
        }
        std::size_t selected_arcs = 0;
        auto flow_balance = std::Vector<int>(graph.nodes.size(), 0);
        for (std::size_t arc_id = 0; arc_id < arc_count; ++arc_id) {
            if (!selected(vars.f[commodity_index][arc_id])) {
                continue;
            }
            const auto& arc = graph.arcs[arc_id];
            guide.insert(graph.channels[static_cast<std::size_t>(arc.channel)]);
            selected_arc_ids.push_back(static_cast<int>(arc_id));
            for (const int node_id : {arc.u, arc.v}) {
                const auto& node = graph.nodes[static_cast<std::size_t>(node_id)];
                if (node.kind == GlobalRouteNodeKind::Cob) {
                    pair_cobs.emplace(node.row, node.col);
                }
            }
            ++flow_balance[static_cast<std::size_t>(arc.u)];
            --flow_balance[static_cast<std::size_t>(arc.v)];
            ++selected_arcs;
        }
        for (std::size_t node = 0; node < graph.nodes.size(); ++node) {
            int expected = 0;
            if (static_cast<int>(node) == selected_source_node) {
                ++expected;
            }
            if (static_cast<int>(node) == commodity.sink_node) {
                --expected;
            }
            if (flow_balance[node] != expected) {
                throw std::runtime_error("V17 commodity flow-conservation validation failed");
            }
        }
        if (verbose_level >= 1) {
            debug::info_fmt(
                "V17 Global Routing pair: net={} demand={} source={} guide_channels={} selected_arcs={}",
                commodity.key.net_id,
                commodity.key.demand_id,
                selected_source_index,
                guide.size(),
                selected_arcs);
        }
        const auto& net = nets[problem.owners[commodity.owner].net_index];
        const int source_hops = net.kind == RoutingNetKind::PNnet
            ? 1
            : detailed_endpoint_hops(
                net.sources[commodity.source_options.front().source_index]);
        out.detailed_distance_cap_by_pair.emplace(
            commodity.key,
            static_cast<int>(selected_arcs) + source_hops
                + detailed_endpoint_hops(net.demands[commodity.key.demand_id].sink));
    }

    std::size_t max_channel_unit_load = 0;
    for (std::size_t channel = 0; channel < channel_count; ++channel) {
        for (std::size_t unit = 0; unit < 16; ++unit) {
            std::size_t load = 0;
            for (std::size_t owner_index = 0; owner_index < owner_count; ++owner_index) {
                const auto owner_unit = out.unit_by_owner.at(problem.owners[owner_index].key);
                if (owner_unit == unit && selected(vars.x[owner_index][channel])) {
                    ++load;
                }
            }
            max_channel_unit_load = std::max(max_channel_unit_load, load);
            if (load > 8) {
                throw std::runtime_error("V17 Global Routing capacity validation failed");
            }
        }
    }
    for (std::size_t tob = 0; tob < hardware::Interposer::TOB_SIZE; ++tob) {
        std::array<std::size_t, 16> unit_load {};
        std::array<std::array<std::size_t, 8>, 2> bank_residue_load {};
        for (const auto& owner : problem.owners) {
            const auto unit = out.unit_by_owner.at(owner.key);
            for (const auto& bump : owner.bumps) {
                if (bump.TOB != tob) {
                    continue;
                }
                ++unit_load[unit];
                if (bump.Bank < 2) {
                    ++bank_residue_load[bump.Bank][unit % 8];
                }
            }
        }
        if (std::ranges::any_of(unit_load, [](const std::size_t load) { return load > 8; })) {
            throw std::runtime_error("V17 TOB unit-load validation failed");
        }
        for (const auto& bank : bank_residue_load) {
            if (std::ranges::any_of(bank, [](const std::size_t load) { return load > 8; })) {
                throw std::runtime_error("V17 TOB bank-residue validation failed");
            }
        }
    }
    for (const auto& [_, owner_indices] : problem.bus_owner_indices_by_net) {
        if (owner_indices.empty()) {
            continue;
        }
        const auto reference = out.channel_count_by_owner.at(
            problem.owners[owner_indices.front()].key);
        for (const auto owner_index : owner_indices) {
            if (out.channel_count_by_owner.at(problem.owners[owner_index].key) != reference) {
                throw std::runtime_error("V17 bus Channel-count validation failed");
            }
        }
    }
    debug::info_fmt(
        "V17 Global Routing validation: status=ok objective={} max_channel_unit_load={} pairs={}",
        out.stats.objective,
        max_channel_unit_load,
        out.pair_channels.size());
    out.ok = true;
}

#endif

} // namespace

auto build_global_channel_graph(
    const UnifiedGraph& graph,
    const std::Vector<RoutingNet>& nets
) -> GlobalChannelGraph {
    auto out = GlobalChannelGraph {};
    const int rows = graph.rows;
    const int cols = graph.cols;
    for (const auto& node : graph.nodes) {
        if (node.kind != UnifiedNodeKind::Track) {
            continue;
        }
        const auto coord = channel_coord(node);
        if (out.channel_id_by_coord.contains(coord)) {
            continue;
        }
        const int id = static_cast<int>(out.channels.size());
        out.channel_id_by_coord.emplace(coord, id);
        out.channels.push_back(coord);
    }

    const auto add_node = [&](const GlobalRouteNode& node) {
        const int id = static_cast<int>(out.nodes.size());
        out.nodes.push_back(node);
        out.in_arc_ids.emplace_back();
        out.out_arc_ids.emplace_back();
        return id;
    };
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            const int node = add_node(GlobalRouteNode {
                GlobalRouteNodeKind::Cob, row, col});
            out.cob_node_by_coord.emplace(std::pair {row, col}, node);
            ++out.cob_node_count;
        }
    }
    auto tob_by_channel = std::map<GlobalChannelCoord, std::size_t> {};
    for (std::size_t tob = 0; tob < hardware::Interposer::TOB_SIZE; ++tob) {
        const auto anchor = tob_anchor_cob(tob);
        const auto channel = GlobalChannelCoord {
            1, static_cast<int>(anchor.row), static_cast<int>(anchor.col)};
        if (!out.channel_id_by_coord.contains(channel)) {
            throw std::logic_error("V17 TOB attachment Channel is missing");
        }
        const int node = add_node(GlobalRouteNode {
            GlobalRouteNodeKind::TobTerminal,
            static_cast<int>(anchor.row),
            static_cast<int>(anchor.col),
            tob,
            out.channel_id_by_coord.at(channel)});
        out.tob_node_by_tob.emplace(tob, node);
        tob_by_channel.emplace(channel, tob);
        ++out.tob_terminal_node_count;
    }

    out.arc_ids_by_channel.resize(out.channels.size());
    out.adjacent_channel_ids.resize(out.channels.size());
    const auto add_arc = [&](const int u, const int v, const int channel, const int port) {
        const int arc_id = static_cast<int>(out.arcs.size());
        out.arcs.push_back(GlobalChannelArc {u, v, channel, port});
        out.out_arc_ids[static_cast<std::size_t>(u)].push_back(arc_id);
        out.in_arc_ids[static_cast<std::size_t>(v)].push_back(arc_id);
        out.arc_ids_by_channel[static_cast<std::size_t>(channel)].push_back(arc_id);
    };
    const auto add_undirected = [&](const int u, const int v, const int channel, const int port = -1) {
        add_arc(u, v, channel, port);
        add_arc(v, u, channel, port);
    };

    auto incident_channels_by_cob = std::map<std::pair<int, int>, std::Vector<int>> {};
    for (std::size_t channel_id = 0; channel_id < out.channels.size(); ++channel_id) {
        const auto& channel = out.channels[channel_id];
        const auto cobs = adjacent_cobs(channel, rows, cols);
        if (cobs.empty() || cobs.size() > 2) {
            throw std::logic_error("V17 physical Channel has invalid COB adjacency");
        }
        for (const auto& cob : cobs) {
            incident_channels_by_cob[cob].push_back(static_cast<int>(channel_id));
        }
        if (cobs.size() == 1) {
            const int boundary = add_node(GlobalRouteNode {
                GlobalRouteNodeKind::BoundaryTerminal,
                cobs.front().first,
                cobs.front().second,
                0,
                static_cast<int>(channel_id)});
            out.boundary_node_by_channel.emplace(static_cast<int>(channel_id), boundary);
            ++out.boundary_terminal_node_count;
            add_undirected(
                out.cob_node_by_coord.at(cobs.front()),
                boundary,
                static_cast<int>(channel_id));
            continue;
        }
        const auto tob_it = tob_by_channel.find(channel);
        if (tob_it == tob_by_channel.end()) {
            add_undirected(
                out.cob_node_by_coord.at(cobs[0]),
                out.cob_node_by_coord.at(cobs[1]),
                static_cast<int>(channel_id));
            continue;
        }
        const int tob_node = out.tob_node_by_tob.at(tob_it->second);
        add_undirected(
            out.cob_node_by_coord.at(cobs[0]),
            tob_node,
            static_cast<int>(channel_id));
        add_undirected(
            tob_node,
            out.cob_node_by_coord.at(cobs[1]),
            static_cast<int>(channel_id));
    }

    auto port_endpoints = std::map<GlobalPortKey, GraphNodeRef> {};
    for (const auto& net : nets) {
        for (const auto& source : net.sources) {
            if (source.kind == GraphNodeRef::Kind::Track) {
                port_endpoints.emplace(port_key(source), source);
            }
        }
        for (const auto& demand : net.demands) {
            if (demand.sink.kind == GraphNodeRef::Kind::Track) {
                port_endpoints.emplace(port_key(demand.sink), demand.sink);
            }
        }
    }
    for (const auto& [key, endpoint] : port_endpoints) {
        const int channel_id = endpoint_channel_id(out, endpoint);
        const auto cobs = adjacent_cobs(
            out.channels[static_cast<std::size_t>(channel_id)], rows, cols);
        if (cobs.size() != 1) {
            throw std::invalid_argument(
                "V17 external/01 port must lie on a boundary Channel");
        }
        auto node_data = GlobalRouteNode {};
        node_data.kind = GlobalRouteNodeKind::PortTerminal;
        node_data.row = cobs.front().first;
        node_data.col = cobs.front().second;
        node_data.channel = channel_id;
        node_data.port = key;
        const int port_node = add_node(node_data);
        out.port_node_by_key.emplace(key, port_node);
        ++out.port_terminal_node_count;
        add_undirected(
            port_node,
            out.cob_node_by_coord.at(cobs.front()),
            channel_id,
            port_node);
    }

    for (const auto& [_, incident] : incident_channels_by_cob) {
        for (std::size_t lhs = 0; lhs < incident.size(); ++lhs) {
            for (std::size_t rhs = lhs + 1; rhs < incident.size(); ++rhs) {
                out.adjacent_channel_ids[static_cast<std::size_t>(incident[lhs])].push_back(
                    incident[rhs]);
                out.adjacent_channel_ids[static_cast<std::size_t>(incident[rhs])].push_back(
                    incident[lhs]);
            }
        }
    }
    for (auto& adjacent : out.adjacent_channel_ids) {
        std::sort(adjacent.begin(), adjacent.end());
        adjacent.erase(std::unique(adjacent.begin(), adjacent.end()), adjacent.end());
    }
    debug::info_fmt(
        "V17 Global Routing graph: nodes={} cob_nodes={} tob_terminal_nodes={} port_terminal_nodes={} boundary_terminal_nodes={} physical_channels={} directed_traversal_arcs={} collapsed_track_nodes={}",
        out.nodes.size(),
        out.cob_node_count,
        out.tob_terminal_node_count,
        out.port_terminal_node_count,
        out.boundary_terminal_node_count,
        out.channels.size(),
        out.arcs.size(),
        graph.track_node_count);
    return out;
}

auto solve_global_route_v17(
    const UnifiedGraph& graph,
    const GlobalChannelGraph& channel_graph,
    const std::Vector<RoutingNet>& nets,
    const int verbose_level,
    const GlobalRouteCapacityMode capacity_mode
) -> GlobalRouteResult {
    (void)graph;
    const auto total_begin = std::chrono::steady_clock::now();
    auto out = GlobalRouteResult {};
    out.stats.nodes = channel_graph.nodes.size();
    out.stats.cob_nodes = channel_graph.cob_node_count;
    out.stats.tob_terminal_nodes = channel_graph.tob_terminal_node_count;
    out.stats.port_terminal_nodes = channel_graph.port_terminal_node_count;
    out.stats.boundary_terminal_nodes = channel_graph.boundary_terminal_node_count;
    out.stats.channels = channel_graph.channels.size();
    out.stats.arcs = channel_graph.arcs.size();
    try {
        const auto problem = prepare_problem(channel_graph, nets);
        out.stats.owners = problem.owners.size();
        out.stats.commodities = problem.commodities.size();
        debug::info_fmt(
            "V17 Global Routing prepare: nets={} owners={} demands={} "
            "PNnets={} buses={}",
            nets.size(),
            problem.owners.size(),
            out.stats.commodities,
            problem.pn_owner_indices_by_net.size(),
            problem.bus_owner_indices_by_net.size());
#ifdef USE_HIGHS
        build_and_solve(
            channel_graph, nets, problem, verbose_level, capacity_mode, out);
#else
        out.message = "HiGHS backend is unavailable";
        debug::error(out.message);
#endif
    }
    catch (const std::exception& error) {
        out.ok = false;
        out.message = error.what();
        debug::error_fmt("V17 Global Routing exception: {}", out.message);
    }
    const auto total_end = std::chrono::steady_clock::now();
    out.stats.total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        total_end - total_begin).count();
    debug::info_fmt(
        "V17 Global Routing summary: status={} vars={} constraints={} objective={} estimated_wirelength={} total_ms={} build_ms={} solve_ms={}",
        out.ok ? "OPTIMAL" : out.message,
        out.stats.variables,
        out.stats.constraints,
        out.stats.objective,
        out.stats.estimated_wirelength,
        out.stats.total_ms,
        out.stats.build_ms,
        out.stats.solve_ms);
    return out;
}

auto apply_global_route_v17(
    const GlobalRouteResult& route,
    RoutingProblemState& state,
    std::Vector<RoutingNet>& nets
) -> void {
    if (!route.ok) {
        throw std::invalid_argument("cannot apply failed V17 Global Routing result");
    }
    for (auto& pair : state.pairs) {
        const auto it = route.pair_channels.find(pair.key);
        if (it == route.pair_channels.end() || it->second.empty()) {
            throw std::logic_error(std::format(
                "V17 route has no guide for net {} demand {}",
                pair.key.net_id,
                pair.key.demand_id));
        }
        pair.allowed_channels = it->second;
        pair.delays.clear();
        const auto cap = route.detailed_distance_cap_by_pair.find(pair.key);
        pair.global_route_distance_cap = cap == route.detailed_distance_cap_by_pair.end()
            ? -1
            : cap->second;
    }
    for (auto& net : nets) {
        net.global_unit_by_source.clear();
        net.released_global_unit_sources.clear();
        net.global_selected_pn_source_indices.clear();
        for (const auto& [owner, unit] : route.unit_by_owner) {
            if (owner.net_id == net.net_id && net.kind == RoutingNetKind::Bnet) {
                net.global_unit_by_source[owner.owner_index] = unit;
            }
        }
        if (net.kind == RoutingNetKind::PNnet) {
            for (const auto& [pair, source_index] : route.selected_source_index_by_pair) {
                if (pair.net_id == net.net_id) {
                    net.global_selected_pn_source_indices.insert(source_index);
                }
            }
        }
    }
    apply_state_to_nets(state, nets);
}

auto expand_global_route_guides_one_hop(
    const GlobalChannelGraph& graph,
    RoutingProblemState& state,
    const std::Vector<PairKey>& critical_pairs
) -> std::size_t {
    std::size_t added = 0;
    for (const auto& key : critical_pairs) {
        auto* pair = find_pair_state(state, key);
        if (pair == nullptr || pair->allowed_channels.empty()) {
            continue;
        }
        auto expanded = pair->allowed_channels;
        for (const auto& coord : pair->allowed_channels) {
            const auto channel_it = graph.channel_id_by_coord.find(coord);
            if (channel_it == graph.channel_id_by_coord.end()) {
                continue;
            }
            const auto channel = static_cast<std::size_t>(channel_it->second);
            for (const int adjacent : graph.adjacent_channel_ids[channel]) {
                expanded.insert(graph.channels[static_cast<std::size_t>(adjacent)]);
            }
        }
        added += expanded.size() - pair->allowed_channels.size();
        pair->allowed_channels = std::move(expanded);
    }
    return added;
}

auto all_global_route_guides_full(
    const RoutingProblemState& state,
    const std::size_t channel_count
) -> bool {
    return !state.pairs.empty()
        && std::all_of(
            state.pairs.begin(),
            state.pairs.end(),
            [&](const PairRoutingState& pair) {
                return pair.allowed_channels.size() >= channel_count;
            });
}

} // namespace PR_tool
