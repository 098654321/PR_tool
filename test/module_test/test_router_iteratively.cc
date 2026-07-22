#include "./utilty.hh"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <sys/wait.h>

namespace {

constexpr int kDefaultIterations = 100;
constexpr const char* kDefaultConfigPath = "../test/config/case1";
constexpr const char* kDebugLogPath = "./debug.log";

auto build_pr_tool_cmd(const std::string& config_path) -> std::string {
    return "./PR_tool_cli " + config_path + " > /dev/null 2>&1";
}

[[noreturn]] auto fail_iteration(int iteration, const std::string& reason) -> void {
    std::cout << "router_iteratively failed at iteration " << iteration << ": " << reason << std::endl;
    std::exit(-1);
}

auto read_file(const char* path) -> std::string {
    std::ifstream file {path};
    if (!file.is_open()) {
        return {};
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

auto parse_last_match(const std::string& content, const std::regex& pattern) -> std::optional<int> {
    std::sregex_iterator begin {content.begin(), content.end(), pattern};
    std::sregex_iterator end {};
    std::optional<int> last_value;
    for (auto it = begin; it != end; ++it) {
        last_value = std::stoi((*it)[1].str());
    }
    return last_value;
}

auto run_pr_tool_subprocess(int iteration, const std::string& cmd) -> void {
    const int status = std::system(cmd.c_str());
    if (status == -1) {
        fail_iteration(iteration, "failed to launch PR_tool subprocess");
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        fail_iteration(
            iteration,
            std::string {"PR_tool exited abnormally, status="} + std::to_string(status)
        );
    }
}

auto check_debug_log(int iteration, int total_iterations) -> void {
    const auto content = read_file(kDebugLogPath);
    if (content.empty()) {
        fail_iteration(iteration, std::string {"unable to read "} + kDebugLogPath);
    }

    const std::regex total_length_pattern {R"(Total Length:\s*(\d+))"};
    const std::regex failed_routing_pattern {R"(Failed routing nubmer:\s*(\d+))"};

    const auto total_length = parse_last_match(content, total_length_pattern);
    if (!total_length.has_value()) {
        fail_iteration(iteration, "Total Length not found in debug.log");
    }

    const auto failed_routing = parse_last_match(content, failed_routing_pattern);
    if (!failed_routing.has_value()) {
        fail_iteration(iteration, "Failed routing nubmer not found in debug.log");
    }

    if (*failed_routing != 0) {
        fail_iteration(
            iteration,
            "Failed routing nubmer=" + std::to_string(*failed_routing)
        );
    }

    if (content.find("Routing failed for this net:") != std::string::npos) {
        fail_iteration(iteration, "found per-net routing failure in debug.log");
    }

    std::cout << "Iteration " << iteration << "/" << total_iterations
              << ", Total Length=" << *total_length
              << ", Failed routing nubmer=" << *failed_routing << std::endl;
}

} // namespace

void test_router_iteratively_main(int argc, char** argv) {
    const std::string config_path =
        (argc >= 3) ? argv[2] : kDefaultConfigPath;
    int iterations = kDefaultIterations;
    if (argc >= 4) {
        iterations = std::stoi(argv[3]);
    }
    if (iterations <= 0) {
        std::cout << "router_iteratively: iterations must be > 0, got " << iterations << std::endl;
        std::exit(-1);
    }

    const std::string pr_tool_cmd = build_pr_tool_cmd(config_path);

    std::cout << "router_iteratively: config=" << config_path
              << ", iterations=" << iterations << std::endl;

    for (int i = 1; i <= iterations; ++i) {
        run_pr_tool_subprocess(i, pr_tool_cmd);
        check_debug_log(i, iterations);
    }
    std::cout << "All " << iterations << " iterations passed" << std::endl;
}
