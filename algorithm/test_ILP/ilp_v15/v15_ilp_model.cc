#include "ilp_v15/v15_ilp_model.hh"

#include <gurobi_c++.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <format>
#include <map>
#include <queue>
#include <set>

namespace PR_tool {

namespace {

constexpr int kModeGroupCount = 16 * 64;

struct CommodityVars {
    std::map<int, GRBVar> f_by_arc;
    std::map<int, GRBVar> x_by_arc;
    std::map<int, GRBVar> y_by_node;
};

// This domain reduction is exact: an arc can participate in a commodity only
// when its endpoints remain usable and it lies on a directed source-to-sink
// walk inside the SAT scope.  In particular, resources fixed by an unselected
// net never need ILP variables.
struct CommodityDomain {
    std::Vector<int> node_ids;
    std::Vector<int> arc_ids;
};

auto is_physical_node(const UnifiedGraph& graph, int node) -> bool {
    return node >= 0
        && static_cast<std::size_t>(node) < graph.nodes.size()
        && graph.nodes[static_cast<std::size_t>(node)].kind != UnifiedNodeKind::VirtualSource;
}

auto is_wirelength_node(const UnifiedGraph& graph, int node) -> bool {
    if (!is_physical_node(graph, node)) {
        return false;
    }
    const auto kind = graph.nodes[static_cast<std::size_t>(node)].kind;
    return kind == UnifiedNodeKind::Track || kind == UnifiedNodeKind::Bump;
}

auto contains_sink(const IlpCommodity& commodity, int node) -> bool {
    return std::find(commodity.sink_nodes.begin(), commodity.sink_nodes.end(), node)
        != commodity.sink_nodes.end();
}

auto build_commodity_domain(
    const UnifiedGraph& graph,
    const UnifiedSatNetScope& scope,
    const IlpCommodity& commodity,
    const V15LockedResources& locked
) -> CommodityDomain {
    auto allowed = std::Vector<bool>(graph.nodes.size(), false);
    for (const int node : scope.node_ids) {
        const bool endpoint = node == commodity.source_node || contains_sink(commodity, node);
        const bool locked_node = is_physical_node(graph, node)
            && static_cast<std::size_t>(node) < locked.node_used.size()
            && locked.node_used[static_cast<std::size_t>(node)];
        allowed[static_cast<std::size_t>(node)] = endpoint || !locked_node;
    }

    auto candidate_arcs = std::Vector<int> {};
    auto forward = std::Vector<std::Vector<int>>(graph.nodes.size());
    auto reverse = std::Vector<std::Vector<int>>(graph.nodes.size());
    for (const int arc_id : scope.arc_ids) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        if (!allowed[static_cast<std::size_t>(arc.u)] || !allowed[static_cast<std::size_t>(arc.v)]) {
            continue;
        }
        candidate_arcs.push_back(arc_id);
        forward[static_cast<std::size_t>(arc.u)].push_back(arc.v);
        reverse[static_cast<std::size_t>(arc.v)].push_back(arc.u);
    }

    auto reachable_from_source = std::Vector<bool>(graph.nodes.size(), false);
    auto queue = std::queue<int> {};
    if (allowed[static_cast<std::size_t>(commodity.source_node)]) {
        reachable_from_source[static_cast<std::size_t>(commodity.source_node)] = true;
        queue.push(commodity.source_node);
    }
    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        for (const int next : forward[static_cast<std::size_t>(node)]) {
            if (!reachable_from_source[static_cast<std::size_t>(next)]) {
                reachable_from_source[static_cast<std::size_t>(next)] = true;
                queue.push(next);
            }
        }
    }

    auto can_reach_sink = std::Vector<bool>(graph.nodes.size(), false);
    for (const int sink : commodity.sink_nodes) {
        if (allowed[static_cast<std::size_t>(sink)] && !can_reach_sink[static_cast<std::size_t>(sink)]) {
            can_reach_sink[static_cast<std::size_t>(sink)] = true;
            queue.push(sink);
        }
    }
    while (!queue.empty()) {
        const int node = queue.front();
        queue.pop();
        for (const int previous : reverse[static_cast<std::size_t>(node)]) {
            if (!can_reach_sink[static_cast<std::size_t>(previous)]) {
                can_reach_sink[static_cast<std::size_t>(previous)] = true;
                queue.push(previous);
            }
        }
    }

    auto out = CommodityDomain {};
    for (const int node : scope.node_ids) {
        if (reachable_from_source[static_cast<std::size_t>(node)]
            && can_reach_sink[static_cast<std::size_t>(node)]) {
            out.node_ids.push_back(node);
        }
    }
    for (const int arc_id : candidate_arcs) {
        const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
        if (reachable_from_source[static_cast<std::size_t>(arc.u)]
            && can_reach_sink[static_cast<std::size_t>(arc.u)]
            && reachable_from_source[static_cast<std::size_t>(arc.v)]
            && can_reach_sink[static_cast<std::size_t>(arc.v)]) {
            out.arc_ids.push_back(arc_id);
        }
    }
    return out;
}

auto use_expression_for_switch(
    const std::map<int, GRBLinExpr>& use_by_switch,
    int switch_id
) -> GRBLinExpr {
    const auto it = use_by_switch.find(switch_id);
    return it == use_by_switch.end() ? GRBLinExpr {0.0} : it->second;
}

auto gurobi_status_name(int status) -> std::String {
    switch (status) {
        case GRB_OPTIMAL: return "OPTIMAL";
        case GRB_SUBOPTIMAL: return "SUBOPTIMAL";
        case GRB_INFEASIBLE: return "INFEASIBLE";
        case GRB_INF_OR_UNBD: return "INF_OR_UNBD";
        case GRB_UNBOUNDED: return "UNBOUNDED";
        case GRB_TIME_LIMIT: return "TIME_LIMIT";
        case GRB_NUMERIC: return "NUMERIC";
        case GRB_INTERRUPTED: return "INTERRUPTED";
        default: return std::format("STATUS_{}", status);
    }
}

} // namespace

auto solve_v15_ilp_model(
    const UnifiedGraph& graph,
    const std::Vector<UnifiedSatNetScope>& scopes,
    const std::Vector<IlpCommodity>& commodities,
    const V15LockedResources& locked,
    const V15IlpOptimizeOptions& options,
    const V15MipStart& mip_start
) -> V15IlpModelResult {
    auto out = V15IlpModelResult {};
    try {
        const auto build_begin = std::chrono::steady_clock::now();
        std::filesystem::create_directories(options.gurobi_log_dir);
        const auto log_path = std::filesystem::path {options.gurobi_log_dir} / "v15_ilp.log";
        std::ofstream(log_path, std::ios::trunc).close();

        auto env = GRBEnv {true};
        env.set(GRB_IntParam_OutputFlag, 1);
        env.set(GRB_IntParam_LogToConsole, 0);
        // The joint case7 model is sparse but large.  Dual simplex avoids the
        // concurrent barrier's substantially higher root-relaxation memory;
        // one thread further reduces the optional post-pass RSS without
        // changing the MIP or its gap criterion.
        env.set(GRB_IntParam_Method, 1);
        env.set(GRB_IntParam_Threads, 1);
        env.set(GRB_StringParam_LogFile, log_path.string());
        env.start();
        auto model = GRBModel {env};
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        auto vars = std::Vector<CommodityVars>(commodities.size());
        auto domains = std::Vector<CommodityDomain>(commodities.size());
        auto objective = GRBLinExpr {0.0};
        for (const auto& commodity : commodities) {
            if (commodity.scope_index >= scopes.size() || commodity.commodity_id >= vars.size()) {
                throw std::invalid_argument("v15 commodity references an invalid scope or ID");
            }
            const auto& scope = scopes[commodity.scope_index];
            auto& cv = vars[commodity.commodity_id];
            auto& domain = domains[commodity.commodity_id];
            domain = build_commodity_domain(graph, scope, commodity, locked);
            for (const int node : domain.node_ids) {
                if (!is_physical_node(graph, node)) {
                    continue;
                }
                auto y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
                cv.y_by_node.emplace(node, y);
                if (is_wirelength_node(graph, node)) {
                    objective += y;
                }
                ++out.stats.y_vars;
            }
            for (const int arc_id : domain.arc_ids) {
                auto f = model.addVar(0.0, commodity.k, 0.0, GRB_INTEGER);
                auto x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
                cv.f_by_arc.emplace(arc_id, f);
                cv.x_by_arc.emplace(arc_id, x);
                ++out.stats.f_vars;
                ++out.stats.x_vars;
            }
        }
        model.setObjective(objective, GRB_MINIMIZE);

        auto mode_vars = std::Vector<GRBVar> {};
        mode_vars.reserve(kModeGroupCount);
        for (int group = 0; group < kModeGroupCount; ++group) {
            mode_vars.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY));
            ++out.stats.mode_vars;
        }
        model.update();

        for (const auto& commodity : commodities) {
            const auto& domain = domains[commodity.commodity_id];
            auto& cv = vars[commodity.commodity_id];
            for (const int arc_id : domain.arc_ids) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                const auto f = cv.f_by_arc.at(arc_id);
                const auto x = cv.x_by_arc.at(arc_id);
                model.addConstr(f >= x);
                model.addConstr(f <= commodity.k * x);
                out.stats.constraints += 2;
                if (const auto it = cv.y_by_node.find(arc.u); it != cv.y_by_node.end()) {
                    model.addConstr(x <= it->second);
                    ++out.stats.constraints;
                }
                if (const auto it = cv.y_by_node.find(arc.v); it != cv.y_by_node.end()) {
                    model.addConstr(x <= it->second);
                    ++out.stats.constraints;
                }
            }

            for (const int node : domain.node_ids) {
                auto out_f = GRBLinExpr {0.0};
                auto in_f = GRBLinExpr {0.0};
                auto out_x = GRBLinExpr {0.0};
                auto in_x = GRBLinExpr {0.0};
                for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(node)]) {
                    if (const auto it = cv.f_by_arc.find(arc_id); it != cv.f_by_arc.end()) {
                        out_f += it->second;
                        out_x += cv.x_by_arc.at(arc_id);
                    }
                }
                for (const int arc_id : graph.in_arc_ids[static_cast<std::size_t>(node)]) {
                    if (const auto it = cv.f_by_arc.find(arc_id); it != cv.f_by_arc.end()) {
                        in_f += it->second;
                        in_x += cv.x_by_arc.at(arc_id);
                    }
                }
                int balance = 0;
                if (node == commodity.source_node) {
                    balance = commodity.k;
                }
                else if (contains_sink(commodity, node)) {
                    balance = -1;
                }
                model.addConstr(out_f - in_f == balance);
                ++out.stats.constraints;

                if (node == commodity.source_node) {
                    model.addConstr(in_x == 0.0);
                    ++out.stats.constraints;
                    if (const auto it = cv.y_by_node.find(node); it != cv.y_by_node.end()) {
                        model.addConstr(it->second == 1.0);
                        ++out.stats.constraints;
                    }
                }
                else if (const auto it = cv.y_by_node.find(node); it != cv.y_by_node.end()) {
                    model.addConstr(in_x == it->second);
                    ++out.stats.constraints;
                }
                if (contains_sink(commodity, node)) {
                    model.addConstr(out_x == 0.0);
                    ++out.stats.constraints;
                    if (const auto it = cv.y_by_node.find(node); it != cv.y_by_node.end()) {
                        model.addConstr(it->second == 1.0);
                        ++out.stats.constraints;
                    }
                }
            }
        }

        for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
            if (!is_physical_node(graph, node)) {
                continue;
            }
            auto occupancy = GRBLinExpr {0.0};
            for (const auto& cv : vars) {
                if (const auto it = cv.y_by_node.find(node); it != cv.y_by_node.end()) {
                    occupancy += it->second;
                }
            }
            const bool is_locked = static_cast<std::size_t>(node) < locked.node_used.size()
                && locked.node_used[static_cast<std::size_t>(node)];
            model.addConstr(occupancy <= (is_locked ? 0.0 : 1.0));
            ++out.stats.constraints;
        }

        auto use_by_switch = std::map<int, GRBLinExpr> {};
        auto exemplar_by_switch = std::map<int, const UnifiedArc*> {};
        for (const auto& arc : graph.arcs) {
            if (arc.physical_switch_id < 0) {
                continue;
            }
            exemplar_by_switch.try_emplace(arc.physical_switch_id, &arc);
            use_by_switch.try_emplace(
                arc.physical_switch_id,
                locked.switch_used.contains(arc.physical_switch_id) ? 1.0 : 0.0);
        }
        for (const auto& commodity : commodities) {
            const auto& cv = vars[commodity.commodity_id];
            for (const auto& [arc_id, x] : cv.x_by_arc) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                if (arc.physical_switch_id >= 0) {
                    use_by_switch.at(arc.physical_switch_id) += x;
                }
            }
        }
        for (const auto& [switch_id, use] : use_by_switch) {
            (void)switch_id;
            model.addConstr(use <= 1.0);
            ++out.stats.constraints;
        }

        auto matching = std::map<std::pair<int, int>, std::set<int>> {};
        for (const auto& [switch_id, arc] : exemplar_by_switch) {
            const auto u_kind = graph.nodes[static_cast<std::size_t>(arc->u)].kind;
            const auto v_kind = graph.nodes[static_cast<std::size_t>(arc->v)].kind;
            if (u_kind == UnifiedNodeKind::Bump && v_kind == UnifiedNodeKind::HLine) {
                matching[{0, arc->u}].insert(switch_id);
                matching[{1, arc->v}].insert(switch_id);
            }
            else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::Bump) {
                matching[{0, arc->v}].insert(switch_id);
                matching[{1, arc->u}].insert(switch_id);
            }
            else if (u_kind == UnifiedNodeKind::HLine && v_kind == UnifiedNodeKind::VLine) {
                matching[{2, arc->u}].insert(switch_id);
                matching[{3, arc->v}].insert(switch_id);
            }
            else if (u_kind == UnifiedNodeKind::VLine && v_kind == UnifiedNodeKind::HLine) {
                matching[{2, arc->v}].insert(switch_id);
                matching[{3, arc->u}].insert(switch_id);
            }
            if (arc->physical_switch_kind == PhysicalSwitchKind::VLineTrack
                && arc->mode_group_id >= 0
                && arc->mode_group_id < kModeGroupCount) {
                const auto use = use_expression_for_switch(use_by_switch, switch_id);
                if (arc->is_vline_track_straight) {
                    model.addConstr(use <= mode_vars[static_cast<std::size_t>(arc->mode_group_id)]);
                    ++out.stats.constraints;
                }
                if (arc->is_vline_track_swap) {
                    model.addConstr(
                        use <= 1.0 - mode_vars[static_cast<std::size_t>(arc->mode_group_id)]);
                    ++out.stats.constraints;
                }
            }
        }
        for (const auto& [key, switch_ids] : matching) {
            (void)key;
            auto use = GRBLinExpr {0.0};
            for (const int switch_id : switch_ids) {
                use += use_expression_for_switch(use_by_switch, switch_id);
            }
            model.addConstr(use <= 1.0);
            ++out.stats.constraints;
        }

        auto bus_members = std::map<std::size_t, std::Vector<std::size_t>> {};
        for (const auto& commodity : commodities) {
            if (commodity.is_bus_member) {
                bus_members[commodity.routing_net_id].push_back(commodity.commodity_id);
            }
        }
        for (const auto& [net_id, ids] : bus_members) {
            (void)net_id;
            if (ids.size() < 2) {
                continue;
            }
            auto reference = GRBLinExpr {0.0};
            for (const auto& [node, y] : vars[ids.front()].y_by_node) {
                if (is_wirelength_node(graph, node)) {
                    reference += y;
                }
            }
            for (std::size_t i = 1; i < ids.size(); ++i) {
                auto length = GRBLinExpr {0.0};
                for (const auto& [node, y] : vars[ids[i]].y_by_node) {
                    if (is_wirelength_node(graph, node)) {
                        length += y;
                    }
                }
                model.addConstr(length == reference);
                ++out.stats.constraints;
            }
        }

        if (mip_start.available) {
            for (const auto& commodity : commodities) {
                auto& cv = vars[commodity.commodity_id];
                const auto selected_it = mip_start.selected_arc_ids.find(commodity.commodity_id);
                const auto used_it = mip_start.used_node_ids.find(commodity.commodity_id);
                for (auto& [arc_id, x] : cv.x_by_arc) {
                    const bool selected = selected_it != mip_start.selected_arc_ids.end()
                        && selected_it->second.contains(arc_id);
                    x.set(GRB_DoubleAttr_Start, selected ? 1.0 : 0.0);
                    const auto flow_it = mip_start.flow_by_arc.find({commodity.commodity_id, arc_id});
                    cv.f_by_arc.at(arc_id).set(
                        GRB_DoubleAttr_Start,
                        flow_it == mip_start.flow_by_arc.end() ? 0.0 : flow_it->second);
                }
                for (auto& [node, y] : cv.y_by_node) {
                    const bool used = used_it != mip_start.used_node_ids.end()
                        && used_it->second.contains(node);
                    y.set(GRB_DoubleAttr_Start, used ? 1.0 : 0.0);
                }
            }
            for (const auto& [group, straight] : mip_start.mode_straight) {
                if (group >= 0 && group < kModeGroupCount) {
                    mode_vars[static_cast<std::size_t>(group)].set(
                        GRB_DoubleAttr_Start,
                        straight ? 1.0 : 0.0);
                }
            }
        }

        const auto build_done = std::chrono::steady_clock::now();
        out.stats.model_build_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            build_done - build_begin).count();
        const auto solve_begin = std::chrono::steady_clock::now();
        model.optimize();
        const auto solve_done = std::chrono::steady_clock::now();
        out.stats.solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            solve_done - solve_begin).count();
        const int status = model.get(GRB_IntAttr_Status);
        out.stats.solution_count = model.get(GRB_IntAttr_SolCount);
        out.status = status == GRB_OPTIMAL
            ? V15IlpStatus::Optimal
            : status == GRB_SUBOPTIMAL && out.stats.solution_count > 0
                ? V15IlpStatus::Suboptimal
                : V15IlpStatus::Failed;
        out.message = gurobi_status_name(status);
        out.ok = out.status == V15IlpStatus::Optimal || out.status == V15IlpStatus::Suboptimal;
        out.stats.constraints = static_cast<std::size_t>(model.get(GRB_IntAttr_NumConstrs));
        out.stats.nonzeros = static_cast<std::size_t>(model.get(GRB_DoubleAttr_DNumNZs));
        if (!out.ok) {
            return out;
        }
        out.stats.objective = model.get(GRB_DoubleAttr_ObjVal);
        out.stats.best_bound = model.get(GRB_DoubleAttr_ObjBound);
        out.stats.mip_gap = model.get(GRB_DoubleAttr_MIPGap);

        for (const auto& commodity : commodities) {
            auto solution = V15CommodityModelSolution {};
            solution.commodity_id = commodity.commodity_id;
            const auto& cv = vars[commodity.commodity_id];
            for (const auto& [arc_id, x] : cv.x_by_arc) {
                if (x.get(GRB_DoubleAttr_X) > 0.5) {
                    solution.selected_arc_ids.insert(arc_id);
                    solution.flow_by_arc.emplace(
                        arc_id,
                        static_cast<int>(std::llround(cv.f_by_arc.at(arc_id).get(GRB_DoubleAttr_X))));
                }
            }
            for (const auto& [node, y] : cv.y_by_node) {
                if (y.get(GRB_DoubleAttr_X) > 0.5) {
                    solution.used_node_ids.insert(node);
                }
            }
            out.commodities.push_back(std::move(solution));
        }
        for (int group = 0; group < kModeGroupCount; ++group) {
            out.mode_straight.emplace(
                group,
                mode_vars[static_cast<std::size_t>(group)].get(GRB_DoubleAttr_X) > 0.5);
        }
        return out;
    }
    catch (const GRBException& error) {
        out.message = std::format("Gurobi error {}: {}", error.getErrorCode(), error.getMessage());
        return out;
    }
    catch (const std::exception& error) {
        out.message = error.what();
        return out;
    }
}

} // namespace PR_tool
