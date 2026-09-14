#include "wmaxsat_router.hh"

#include "delay/pair_delay_precompute.hh"
#include "sat/unified_sat_scope.hh"
#include "scope/build_routing_nets.hh"
#include "scope/pair_routing_state.hh"

#include <algo/netbuilder/netbuilder.hh>
#include <debug/debug.hh>

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <climits>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace PR_tool {

namespace {

auto find_source(
    const UnifiedSatModel& model,
    const PairDelayInfo& pair
) -> const SourceDelayVars& {
    const auto it = std::find_if(
        model.sources.begin(),
        model.sources.end(),
        [&](const SourceDelayVars& source) {
            return source.net_id == pair.net_id && source.source_index == pair.source_index;
        });
    if (it == model.sources.end()) {
        throw std::logic_error(std::format(
            "missing source vars for net {} demand {} source {}",
            pair.net_id,
            pair.demand_id,
            pair.source_index));
    }
    return *it;
}

auto d_literal(
    const UnifiedSatModel& model,
    const SourceDelayVars& source,
    int node,
    int delay
) -> int {
    if (node < 0 || delay < 0 || delay > source.d_max) {
        return 0;
    }
    const auto& scope = model.scopes[source.scope_index];
    if (static_cast<std::size_t>(node) >= scope.node_offset.size()) {
        return 0;
    }
    const int offset = scope.node_offset[static_cast<std::size_t>(node)];
    if (offset < 0) {
        return 0;
    }
    const int lit = source.d_var[static_cast<std::size_t>(offset)][static_cast<std::size_t>(delay)];
    return lit > 0 ? lit : 0;
}

auto is_wirelength_node(const UnifiedNode& node) -> bool {
    return node.kind == UnifiedNodeKind::Track || node.kind == UnifiedNodeKind::Bump;
}

auto ensure_u(
    CadicalSession& session,
    WmaxsatEncoding& out,
    std::size_t net_id,
    int node
) -> int {
    const auto key = std::pair {net_id, node};
    const auto existing = out.u_by_net_node.find(key);
    if (existing != out.u_by_net_node.end()) {
        return existing->second;
    }
    const int u = session.new_var();
    out.u_by_net_node.emplace(key, u);
    out.soft_clauses.push_back(WmaxsatSoftClause {1, {-u}});
    return u;
}

auto add_q_and_wirelength_objective(
    CadicalSession& session,
    WmaxsatEncoding& out
) -> void {
    for (const auto& pair : out.model.pair_delays) {
        const auto& source = find_source(out.model, pair);
        auto sink_literals = std::Vector<int> {};
        for (const int delay : pair.delays) {
            const int lit = d_literal(out.model, source, pair.sink_node, delay);
            if (lit > 0) {
                sink_literals.push_back(lit);
            }
        }
        const int q = session.new_var();
        const auto key = PairKey {pair.net_id, pair.demand_id, pair.source_index};
        out.q_by_pair.emplace(key, q);
        auto q_implies_reach = sink_literals;
        q_implies_reach.push_back(-q);
        session.add_clause(q_implies_reach);
        for (const int lit : sink_literals) {
            session.add_clause({-lit, q});
        }
        out.soft_clauses.push_back(WmaxsatSoftClause {kWmaxsatRoutedPairWeight, {q}});

        const auto& source_node = out.graph.nodes[static_cast<std::size_t>(source.source_node)];
        if (is_wirelength_node(source_node)) {
            const int u = ensure_u(session, out, pair.net_id, source.source_node);
            session.add_clause({-q, u});
        }
    }

    for (const auto& source : out.model.sources) {
        const auto& scope = out.model.scopes[source.scope_index];
        for (std::size_t offset = 0; offset < scope.node_ids.size(); ++offset) {
            const int node = scope.node_ids[offset];
            if (!is_wirelength_node(out.graph.nodes[static_cast<std::size_t>(node)])) {
                continue;
            }
            const int u = ensure_u(session, out, source.net_id, node);
            for (int delay = 0; delay <= source.d_max; ++delay) {
                if (node == source.source_node && delay == 0) {
                    continue;
                }
                const int d = source.d_var[offset][static_cast<std::size_t>(delay)];
                if (d > 0) {
                    session.add_clause({-d, u});
                }
            }
        }
    }
}

auto checked_sum_soft_weights(const WmaxsatEncoding& encoding) -> std::uint64_t {
    std::uint64_t sum = 0;
    for (const auto& soft : encoding.soft_clauses) {
        if (soft.weight == 0 || soft.literals.empty()) {
            throw std::logic_error("WCNF soft clauses require a positive weight and a literal");
        }
        if (sum > std::numeric_limits<std::uint64_t>::max() - soft.weight) {
            throw std::overflow_error("WCNF soft-weight sum overflow");
        }
        sum += soft.weight;
    }
    if (sum == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("WCNF TOP overflow");
    }
    return sum;
}

auto assignment_value(const WmaxsatSolverResult& result, int var) -> bool {
    return var > 0 && static_cast<std::size_t>(var) < result.assignment.size()
        && result.assignment[static_cast<std::size_t>(var)];
}

auto find_selected_sink_delay(
    const WmaxsatEncoding& encoding,
    const PairDelayInfo& pair,
    const WmaxsatSolverResult& result
) -> int {
    const auto& source = find_source(encoding.model, pair);
    for (const int delay : pair.delays) {
        const int lit = d_literal(encoding.model, source, pair.sink_node, delay);
        if (assignment_value(result, lit)) {
            return delay;
        }
    }
    return -1;
}

auto backtrace_path(
    const WmaxsatEncoding& encoding,
    const PairDelayInfo& pair,
    int sink_delay,
    const WmaxsatSolverResult& result
) -> std::Vector<int> {
    const auto& source = find_source(encoding.model, pair);
    const auto& scope = encoding.model.scopes[source.scope_index];
    auto reverse_path = std::Vector<int> {pair.sink_node};
    int node = pair.sink_node;
    int delay = sink_delay;
    while (node != source.source_node) {
        if (delay <= 0) {
            throw std::logic_error("route backtrace reached delay zero before source");
        }
        int predecessor = -1;
        for (const int arc_id : encoding.graph.in_arc_ids[static_cast<std::size_t>(node)]) {
            if (scope.arc_offset[static_cast<std::size_t>(arc_id)] < 0) {
                continue;
            }
            const auto& arc = encoding.graph.arcs[static_cast<std::size_t>(arc_id)];
            if (is_tob_arc(arc)) {
                const int a = tob_a_literal(
                    encoding.model, source.model_source_index, arc_id, delay);
                if (assignment_value(result, a)) {
                    predecessor = arc.u;
                    break;
                }
            }
            else if (assignment_value(result, d_literal(encoding.model, source, arc.u, delay - 1))) {
                predecessor = arc.u;
                break;
            }
        }
        if (predecessor < 0) {
            throw std::logic_error(std::format(
                "no selected predecessor for net {} demand {} node {} delay {}",
                pair.net_id, pair.demand_id, node, delay));
        }
        node = predecessor;
        --delay;
        reverse_path.push_back(node);
    }
    std::reverse(reverse_path.begin(), reverse_path.end());
    if (!reverse_path.empty()
        && encoding.graph.nodes[static_cast<std::size_t>(reverse_path.front())].kind
            == UnifiedNodeKind::VirtualSource) {
        reverse_path.erase(reverse_path.begin());
    }
    return reverse_path;
}

auto capture_solver_output(
    const std::filesystem::path& solver_path,
    const std::filesystem::path& wcnf_path
) -> std::pair<int, std::String> {
    int pipe_fd[2] {};
    if (pipe(pipe_fd) != 0) {
        throw std::runtime_error(std::format("pipe failed: {}", std::strerror(errno)));
    }
    const pid_t child = fork();
    if (child < 0) {
        close(pipe_fd[0]);
        close(pipe_fd[1]);
        throw std::runtime_error(std::format("fork failed: {}", std::strerror(errno)));
    }
    if (child == 0) {
        dup2(pipe_fd[1], STDOUT_FILENO);
        dup2(pipe_fd[1], STDERR_FILENO);
        close(pipe_fd[0]);
        close(pipe_fd[1]);
        execl(solver_path.c_str(), solver_path.filename().c_str(), wcnf_path.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    close(pipe_fd[1]);
    auto output = std::String {};
    std::array<char, 4096> buffer {};
    for (;;) {
        const ssize_t bytes = read(pipe_fd[0], buffer.data(), buffer.size());
        if (bytes == 0) {
            break;
        }
        if (bytes < 0) {
            close(pipe_fd[0]);
            throw std::runtime_error(std::format("read solver output failed: {}", std::strerror(errno)));
        }
        output.append(buffer.data(), static_cast<std::size_t>(bytes));
    }
    close(pipe_fd[0]);
    int status = 0;
    if (waitpid(child, &status, 0) < 0) {
        throw std::runtime_error(std::format("waitpid failed: {}", std::strerror(errno)));
    }
    return {WIFEXITED(status) ? WEXITSTATUS(status) : -1, std::move(output)};
}

} // namespace

auto build_wmaxsat_encoding(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie
) -> WmaxsatEncoding {
    algo::build_nets(&basedie, interposer);
    auto out = WmaxsatEncoding {};
    out.nets = build_routing_nets(basedie.nets_to_vector());
    out.graph = build_unified_graph(interposer, out.nets);
    augment_graph_for_pnnet(out.graph, out.nets);

    auto state = init_routing_problem_state(out.nets);
    apply_initial_search_padding(state, out.nets, out.graph, 1, 10);
    apply_state_to_nets(state, out.nets);
    const auto scopes = build_all_scopes(out.graph, out.nets);
    const auto delays = compute_pair_delays(out.graph, out.nets, scopes, &state);

    auto session_options = CadicalDiagnosticsOptions {};
    session_options.capture_clauses = true;
    auto session = CadicalSession {session_options};
    out.model = build_unified_sat_model(
        session, out.graph, out.nets, scopes, delays, nullptr, false);
    add_q_and_wirelength_objective(session, out);
    out.num_vars = session.num_vars();
    out.hard_clauses = session.clauses();

    debug::info_fmt(
        "weighted MAXSAT model: scope_pad=1 delay_pad=10 nets={} pairs={} D/A-hard-vars={} total_vars={} hard_clauses={} soft_clauses={} wire_vars={}",
        out.nets.size(),
        out.model.pair_delays.size(),
        out.num_vars - out.q_by_pair.size() - out.u_by_net_node.size(),
        out.num_vars,
        out.hard_clauses.size(),
        out.soft_clauses.size(),
        out.u_by_net_node.size());
    return out;
}

auto write_wcnf(const WmaxsatEncoding& encoding, const std::filesystem::path& path) -> void {
    const auto soft_sum = checked_sum_soft_weights(encoding);
    const auto top = soft_sum + 1;
    std::filesystem::create_directories(path.parent_path());
    auto stream = std::ofstream {path};
    if (!stream) {
        throw std::runtime_error(std::format("cannot open WCNF output: {}", path.string()));
    }
    stream << "p wcnf " << encoding.num_vars << ' '
           << encoding.hard_clauses.size() + encoding.soft_clauses.size() << ' '
           << top << '\n';
    for (const auto& clause : encoding.hard_clauses) {
        stream << top;
        for (const int lit : clause) {
            stream << ' ' << lit;
        }
        stream << " 0\n";
    }
    for (const auto& soft : encoding.soft_clauses) {
        stream << soft.weight;
        for (const int lit : soft.literals) {
            stream << ' ' << lit;
        }
        stream << " 0\n";
    }
    if (!stream) {
        throw std::runtime_error(std::format("failed while writing WCNF: {}", path.string()));
    }
    debug::info_fmt(
        "weighted MAXSAT WCNF: path={} vars={} hard={} soft={} top={}",
        path.string(), encoding.num_vars, encoding.hard_clauses.size(), encoding.soft_clauses.size(), top);
}

auto run_evalmaxsat(
    const std::filesystem::path& solver_path,
    const std::filesystem::path& wcnf_path,
    const std::size_t num_vars
) -> WmaxsatSolverResult {
    if (!std::filesystem::is_regular_file(solver_path)) {
        throw std::runtime_error(std::format("EvalMaxSAT executable not found: {}", solver_path.string()));
    }
    auto result = WmaxsatSolverResult {};
    auto [exit_code, output] = capture_solver_output(solver_path, wcnf_path);
    result.launched = true;
    result.exit_code = exit_code;
    result.output = std::move(output);
    result.assignment.assign(num_vars + 1, false);
    auto input = std::istringstream {result.output};
    auto line = std::string {};
    while (std::getline(input, line)) {
        if (line.rfind("s ", 0) == 0) {
            result.status = line.substr(2);
            result.optimal = result.status == "OPTIMUM FOUND";
            result.hard_unsat = result.status == "UNSATISFIABLE";
            continue;
        }
        if (line.rfind("o ", 0) == 0) {
            std::uint64_t cost = 0;
            const auto value = std::string_view {line}.substr(2);
            const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), cost);
            if (error == std::errc {} && end == value.data() + value.size()) {
                result.cost = cost;
            }
            continue;
        }
        if (line.rfind("v ", 0) == 0) {
            auto values = std::istringstream {line.substr(2)};
            int literal = 0;
            while (values >> literal) {
                if (literal == 0) {
                    continue;
                }
                const auto var = static_cast<std::size_t>(std::abs(literal));
                if (var <= num_vars) {
                    result.assignment[var] = literal > 0;
                    result.has_model = true;
                }
            }
        }
    }
    debug::info_fmt(
        "EvalMaxSAT: status={} exit_code={} cost={} has_model={}",
        result.status.empty() ? "unknown" : result.status,
        result.exit_code,
        result.cost,
        result.has_model);
    return result;
}

auto log_wmaxsat_solution(
    const WmaxsatEncoding& encoding,
    const WmaxsatSolverResult& result
) -> std::size_t {
    if (!result.has_model) {
        return 0;
    }
    std::size_t routed_pairs = 0;
    for (const auto& pair : encoding.model.pair_delays) {
        const auto key = PairKey {pair.net_id, pair.demand_id, pair.source_index};
        const auto q_it = encoding.q_by_pair.find(key);
        if (q_it == encoding.q_by_pair.end() || !assignment_value(result, q_it->second)) {
            continue;
        }
        const int sink_delay = find_selected_sink_delay(encoding, pair, result);
        if (sink_delay < 0) {
            throw std::logic_error(std::format(
                "q=true without sink D for net {} demand {}", pair.net_id, pair.demand_id));
        }
        const auto path = backtrace_path(encoding, pair, sink_delay, result);
        auto text = std::String {};
        for (std::size_t i = 0; i < path.size(); ++i) {
            if (i != 0) {
                text += " -> ";
            }
            text += format_unified_node(encoding.graph, path[i]);
        }
        debug::info_fmt(
            "weighted MAXSAT path: net={} demand={} source={} distance={} hops={} {}",
            pair.net_id,
            pair.demand_id,
            pair.source_index,
            sink_delay,
            path.empty() ? 0 : path.size() - 1,
            text);
        ++routed_pairs;
    }
    std::size_t wirelength = 0;
    for (const auto& [_, u] : encoding.u_by_net_node) {
        if (assignment_value(result, u)) {
            ++wirelength;
        }
    }
    debug::info_fmt(
        "weighted MAXSAT result: routed_pairs={}/{} total_wirelength={} objective_cost={}",
        routed_pairs,
        encoding.q_by_pair.size(),
        wirelength,
        result.cost);
    if (wirelength >= kWmaxsatRoutedPairWeight) {
        debug::warning_fmt("wirelength {} reaches W_R={}; lexicographic priority is no longer guaranteed", wirelength, kWmaxsatRoutedPairWeight);
    }
    return wirelength;
}

} // namespace PR_tool
