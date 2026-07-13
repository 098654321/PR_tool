#include "ilp_v15/v15_ilp_model.hh"

#include <gurobi_c++.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <debug/debug.hh>
#include <filesystem>
#include <fstream>
#include <format>
#include <map>
#include <queue>
#include <set>

namespace PR_tool {

namespace {

constexpr int kModeGroupCount = 16 * 64;

struct SegmentVars {
    std::map<int, GRBVar> f_by_arc;
};

struct ParentVars {
    std::map<int, GRBVar> x_by_arc;
    std::map<int, GRBVar> y_by_node;
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
    const V15PrepareResult& prepared,
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
        env.set(GRB_IntParam_Method, -1);
        env.set(GRB_IntParam_Threads, 1);
        // Large segment models already have a feasible SAT-derived MIP start.
        // Favor finding and improving incumbents over an expensive aggressive
        // presolve/proof phase. TimeLimit is optional via --time-limit (hours).
        env.set(GRB_IntParam_Presolve, -1);
        env.set(GRB_IntParam_PreSparsify, -1);
        env.set(GRB_IntParam_Symmetry, -1);
        env.set(GRB_IntParam_MIPFocus, 1);
        env.set(GRB_IntParam_Cuts, -1);
        if (options.time_limit_hours.has_value()) {
            env.set(
                GRB_DoubleParam_TimeLimit,
                options.time_limit_hours.value() * 3600.0);
        }
        env.set(GRB_StringParam_LogFile, log_path.string());
        env.start();
        auto model = GRBModel {env};
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        auto segment_vars = std::Vector<SegmentVars>(prepared.segments.size());
        auto parent_vars = std::Vector<ParentVars>(prepared.parents.size());
        auto parent_arcs_by_id = std::Vector<std::set<int>>(prepared.parents.size());
        auto parent_nodes_by_id = std::Vector<std::set<int>>(prepared.parents.size());

        auto objective = GRBLinExpr {0.0};
        for (const auto& parent : prepared.parents) {
            auto& pv = parent_vars[parent.parent_id];
            auto& arcs = parent_arcs_by_id[parent.parent_id];
            auto& nodes = parent_nodes_by_id[parent.parent_id];
            for (const std::size_t segment_id : parent.segment_ids) {
                const auto& segment = prepared.segments[segment_id];
                for (const int arc_id : segment.scope.arc_ids) {
                    arcs.insert(arc_id);
                }
                for (const int node : segment.scope.node_ids) {
                    if (is_physical_node(graph, node)) {
                        nodes.insert(node);
                    }
                }
            }
            for (const int arc_id : arcs) {
                pv.x_by_arc.emplace(arc_id, model.addVar(0.0, 1.0, 0.0, GRB_BINARY));
                ++out.stats.x_vars;
            }
            for (const int node : nodes) {
                auto y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
                pv.y_by_node.emplace(node, y);
                if (is_wirelength_node(graph, node)) {
                    objective += y;
                }
                ++out.stats.y_vars;
            }
            if (options.verbose_level >= 1) {
                debug::info_fmt(
                    "v15 parent net={} id={} segments={} arcs={} nodes={}",
                    parent.routing_net_id,
                    parent.parent_id,
                    parent.segment_ids.size(),
                    arcs.size(),
                    nodes.size());
            }
        }
        for (const auto& segment : prepared.segments) {
            auto& sv = segment_vars[segment.segment_id];
            for (const int arc_id : segment.scope.arc_ids) {
                sv.f_by_arc.emplace(arc_id, model.addVar(0.0, 1.0, 0.0, GRB_BINARY));
                ++out.stats.f_vars;
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

        for (const auto& segment : prepared.segments) {
            auto& sv = segment_vars[segment.segment_id];
            const auto& parent = prepared.parents[segment.parent_id];
            auto& pv = parent_vars[parent.parent_id];
            for (const int arc_id : segment.scope.arc_ids) {
                const auto f = sv.f_by_arc.at(arc_id);
                const auto x = pv.x_by_arc.at(arc_id);
                model.addConstr(f <= x);
                ++out.stats.constraints;
            }
            for (const int node : segment.scope.node_ids) {
                auto out_f = GRBLinExpr {0.0};
                auto in_f = GRBLinExpr {0.0};
                for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(node)]) {
                    if (const auto it = sv.f_by_arc.find(arc_id); it != sv.f_by_arc.end()) {
                        out_f += it->second;
                    }
                }
                for (const int arc_id : graph.in_arc_ids[static_cast<std::size_t>(node)]) {
                    if (const auto it = sv.f_by_arc.find(arc_id); it != sv.f_by_arc.end()) {
                        in_f += it->second;
                    }
                }
                int balance = 0;
                if (node == segment.endpoint_a) {
                    balance = 1;
                }
                else if (node == segment.endpoint_b) {
                    balance = -1;
                }
                model.addConstr(out_f - in_f == balance);
                ++out.stats.constraints;
            }
        }

        for (const auto& parent : prepared.parents) {
            auto& pv = parent_vars[parent.parent_id];
            const auto& arcs = parent_arcs_by_id[parent.parent_id];
            for (const int arc_id : arcs) {
                auto sum_f = GRBLinExpr {0.0};
                for (const std::size_t segment_id : parent.segment_ids) {
                    const auto& sv = segment_vars[segment_id];
                    if (const auto it = sv.f_by_arc.find(arc_id); it != sv.f_by_arc.end()) {
                        sum_f += it->second;
                    }
                }
                model.addConstr(pv.x_by_arc.at(arc_id) <= sum_f);
                ++out.stats.constraints;
            }

            for (const int node : parent_nodes_by_id[parent.parent_id]) {
                auto out_x = GRBLinExpr {0.0};
                auto in_x = GRBLinExpr {0.0};
                for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(node)]) {
                    if (const auto it = pv.x_by_arc.find(arc_id); it != pv.x_by_arc.end()) {
                        out_x += it->second;
                    }
                }
                for (const int arc_id : graph.in_arc_ids[static_cast<std::size_t>(node)]) {
                    if (const auto it = pv.x_by_arc.find(arc_id); it != pv.x_by_arc.end()) {
                        in_x += it->second;
                    }
                }
                if (node != parent.root_node) {
                    model.addConstr(in_x == pv.y_by_node.at(node));
                    ++out.stats.constraints;
                }
            }

            for (const auto& [arc_id, x] : pv.x_by_arc) {
                const auto& arc = graph.arcs[static_cast<std::size_t>(arc_id)];
                if (const auto it = pv.y_by_node.find(arc.u); it != pv.y_by_node.end()) {
                    model.addConstr(x <= it->second);
                    ++out.stats.constraints;
                }
                if (const auto it = pv.y_by_node.find(arc.v); it != pv.y_by_node.end()) {
                    model.addConstr(x <= it->second);
                    ++out.stats.constraints;
                }
            }

            auto in_at_root = GRBLinExpr {0.0};
            for (const int arc_id : graph.in_arc_ids[static_cast<std::size_t>(parent.root_node)]) {
                if (const auto it = pv.x_by_arc.find(arc_id); it != pv.x_by_arc.end()) {
                    in_at_root += it->second;
                }
            }
            model.addConstr(in_at_root == 0.0);
            ++out.stats.constraints;

            for (const int sink : parent.origin_sinks) {
                auto out_at_sink = GRBLinExpr {0.0};
                for (const int arc_id : graph.out_arc_ids[static_cast<std::size_t>(sink)]) {
                    if (const auto it = pv.x_by_arc.find(arc_id); it != pv.x_by_arc.end()) {
                        out_at_sink += it->second;
                    }
                }
                model.addConstr(out_at_sink == 0.0);
                ++out.stats.constraints;
                if (const auto it = pv.y_by_node.find(sink); it != pv.y_by_node.end()) {
                    model.addConstr(it->second == 1.0);
                    ++out.stats.constraints;
                }
            }

            if (is_physical_node(graph, parent.root_node)) {
                if (const auto it = pv.y_by_node.find(parent.root_node); it != pv.y_by_node.end()) {
                    model.addConstr(it->second == 1.0);
                    ++out.stats.constraints;
                }
            }
            for (const int track : parent.fixed_track_nodes) {
                if (const auto it = pv.y_by_node.find(track);
                    it != pv.y_by_node.end()) {
                    model.addConstr(it->second == 1.0);
                    ++out.stats.constraints;
                }
            }
        }

        for (int node = 0; node < static_cast<int>(graph.nodes.size()); ++node) {
            if (!is_physical_node(graph, node)) {
                continue;
            }
            auto occupancy = GRBLinExpr {0.0};
            for (const auto& pv : parent_vars) {
                if (const auto it = pv.y_by_node.find(node); it != pv.y_by_node.end()) {
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
        for (const auto& pv : parent_vars) {
            for (const auto& [arc_id, x] : pv.x_by_arc) {
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
        for (const auto& parent : prepared.parents) {
            if (parent.is_bus_member) {
                bus_members[parent.routing_net_id].push_back(parent.parent_id);
            }
        }
        for (const auto& [net_id, ids] : bus_members) {
            (void)net_id;
            if (ids.size() < 2) {
                continue;
            }
            auto reference = GRBLinExpr {0.0};
            for (const auto& [node, y] : parent_vars[ids.front()].y_by_node) {
                if (is_wirelength_node(graph, node)) {
                    reference += y;
                }
            }
            for (std::size_t i = 1; i < ids.size(); ++i) {
                auto length = GRBLinExpr {0.0};
                for (const auto& [node, y] : parent_vars[ids[i]].y_by_node) {
                    if (is_wirelength_node(graph, node)) {
                        length += y;
                    }
                }
                model.addConstr(length == reference);
                ++out.stats.constraints;
            }
        }

        if (mip_start.available) {
            for (const auto& segment : prepared.segments) {
                auto& sv = segment_vars[segment.segment_id];
                const auto flow_it = mip_start.segment_flow_arc_ids.find(segment.segment_id);
                for (auto& [arc_id, f] : sv.f_by_arc) {
                    const bool selected = flow_it != mip_start.segment_flow_arc_ids.end()
                        && flow_it->second.contains(arc_id);
                    f.set(GRB_DoubleAttr_Start, selected ? 1.0 : 0.0);
                }
            }
            for (const auto& parent : prepared.parents) {
                auto& pv = parent_vars[parent.parent_id];
                const auto arc_it = mip_start.parent_arc_ids.find(parent.parent_id);
                const auto node_it = mip_start.parent_node_ids.find(parent.parent_id);
                for (auto& [arc_id, x] : pv.x_by_arc) {
                    const bool selected = arc_it != mip_start.parent_arc_ids.end()
                        && arc_it->second.contains(arc_id);
                    x.set(GRB_DoubleAttr_Start, selected ? 1.0 : 0.0);
                }
                for (auto& [node, y] : pv.y_by_node) {
                    const bool used = node_it != mip_start.parent_node_ids.end()
                        && node_it->second.contains(node);
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
        out.stats.parents = prepared.parents.size();
        out.stats.segments = prepared.segments.size();

        const auto solve_begin = std::chrono::steady_clock::now();
        model.optimize();
        const auto solve_done = std::chrono::steady_clock::now();
        out.stats.solve_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            solve_done - solve_begin).count();
        const int status = model.get(GRB_IntAttr_Status);
        out.stats.solution_count = model.get(GRB_IntAttr_SolCount);
        const bool has_incumbent = out.stats.solution_count > 0;
        if (status == GRB_OPTIMAL) {
            out.status = V15IlpStatus::Optimal;
        } else if (has_incumbent
            && (status == GRB_SUBOPTIMAL
                || status == GRB_TIME_LIMIT
                || status == GRB_INTERRUPTED)) {
            // Time-limit / interrupt with a feasible incumbent: keep the best
            // found solution instead of falling back to the SAT routing.
            out.status = V15IlpStatus::Suboptimal;
        } else {
            out.status = V15IlpStatus::Failed;
        }
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

        for (const auto& parent : prepared.parents) {
            auto solution = V15ParentModelSolution {};
            solution.parent_id = parent.parent_id;
            const auto& pv = parent_vars[parent.parent_id];
            for (const auto& [arc_id, x] : pv.x_by_arc) {
                if (x.get(GRB_DoubleAttr_X) > 0.5) {
                    solution.selected_arc_ids.insert(arc_id);
                }
            }
            for (const auto& [node, y] : pv.y_by_node) {
                if (y.get(GRB_DoubleAttr_X) > 0.5) {
                    solution.used_node_ids.insert(node);
                }
            }
            out.parents.push_back(std::move(solution));
        }
        for (const auto& segment : prepared.segments) {
            auto solution = V15SegmentModelSolution {};
            solution.segment_id = segment.segment_id;
            const auto& sv = segment_vars[segment.segment_id];
            for (const auto& [arc_id, f] : sv.f_by_arc) {
                if (f.get(GRB_DoubleAttr_X) > 0.5) {
                    solution.flow_arc_ids.insert(arc_id);
                }
            }
            out.segments.push_back(std::move(solution));
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
