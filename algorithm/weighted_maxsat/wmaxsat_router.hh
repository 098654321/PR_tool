#pragma once

#include "common/routing_types.hh"
#include "graph/unified_routing_graph.hh"
#include "sat/unified_sat_encoder.hh"

#include <circuit/basedie.hh>
#include <hardware/interposer.hh>
#include <std/collection.hh>

#include <cstdint>
#include <filesystem>
#include <map>

namespace PR_tool {

constexpr std::uint64_t kWmaxsatRoutedPairWeight = 10000;

struct WmaxsatSoftClause {
    std::uint64_t weight{0};
    std::Vector<int> literals;
};

struct WmaxsatEncoding {
    UnifiedGraph graph;
    std::Vector<RoutingNet> nets;
    UnifiedSatModel model;
    std::Vector<std::Vector<int>> hard_clauses;
    std::Vector<WmaxsatSoftClause> soft_clauses;
    std::map<PairKey, int> q_by_pair;
    std::map<std::pair<std::size_t, int>, int> u_by_net_node;
    std::size_t num_vars{0};
};

struct WmaxsatSolverResult {
    bool launched{false};
    bool has_model{false};
    bool optimal{false};
    bool hard_unsat{false};
    int exit_code{-1};
    std::uint64_t cost{0};
    std::Vector<bool> assignment;
    std::String status;
    std::String output;
};

auto build_wmaxsat_encoding(
    hardware::Interposer* interposer,
    circuit::BaseDie& basedie
) -> WmaxsatEncoding;

auto write_wcnf(const WmaxsatEncoding& encoding, const std::filesystem::path& path) -> void;

auto run_evalmaxsat(
    const std::filesystem::path& solver_path,
    const std::filesystem::path& wcnf_path,
    std::size_t num_vars
) -> WmaxsatSolverResult;

auto log_wmaxsat_solution(
    const WmaxsatEncoding& encoding,
    const WmaxsatSolverResult& result
) -> std::size_t;

} // namespace PR_tool
