#include "wmaxsat_cli.hh"
#include "wmaxsat_router.hh"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace PR_tool;

auto require(const bool condition, const std::string& message) -> void {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

auto test_cli() -> void {
    const auto options = parse_wmaxsat_cli({
        "case_dir", "-vv", "-o", "out", "--solver", "solver", "--keep-wcnf"});
    require(options.config_path == "case_dir", "CLI must preserve config path");
    require(options.output_dir == "out", "CLI must parse output directory");
    require(options.solver_path == "solver", "CLI must parse solver path");
    require(options.verbose_level == 2, "CLI must count v flags");
    require(options.keep_wcnf, "CLI must keep requested WCNF");
}

auto test_wcnf_writer() -> void {
    auto encoding = WmaxsatEncoding {};
    encoding.num_vars = 2;
    encoding.hard_clauses = {{1, -2}};
    encoding.soft_clauses = {
        WmaxsatSoftClause {kWmaxsatRoutedPairWeight, {1}},
        WmaxsatSoftClause {1, {-2}},
    };
    const auto path = std::filesystem::temp_directory_path() / "weighted_maxsat_unit.wcnf";
    write_wcnf(encoding, path);
    auto input = std::ifstream {path};
    auto header = std::string {};
    auto hard = std::string {};
    auto reward = std::string {};
    auto wirelength = std::string {};
    std::getline(input, header);
    std::getline(input, hard);
    std::getline(input, reward);
    std::getline(input, wirelength);
    require(header == "p wcnf 2 3 10002", "WCNF header must use sum(soft)+1 as TOP");
    require(hard == "10002 1 -2 0", "hard clause must use TOP weight");
    require(reward == "10000 1 0", "pair reward clause must be emitted");
    require(wirelength == "1 -2 0", "wirelength soft clause must be emitted");
    std::filesystem::remove(path);
}

} // namespace

auto main() -> int {
    try {
        test_cli();
        test_wcnf_writer();
        std::cout << "weighted_maxsat_unit: all tests passed\n";
        return 0;
    }
    catch (const std::exception& error) {
        std::cerr << "weighted_maxsat_unit: " << error.what() << '\n';
        return 1;
    }
}
