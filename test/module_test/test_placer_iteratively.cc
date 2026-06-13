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

constexpr int kIterations = 100;
constexpr int kMaxAllowedTotalLength = 1100;
constexpr const char* kPrToolCmd = "./PR_tool ../test/config/case5 -p > /dev/null 2>&1";
constexpr const char* kDebugLogPath = "./debug.log";

[[noreturn]] auto fail_iteration(int iteration, const std::string& reason) -> void {
    std::cout << "placer_iteratively failed at iteration " << iteration << ": " << reason << std::endl;
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

auto run_pr_tool_subprocess(int iteration) -> void {
    const int status = std::system(kPrToolCmd);
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

auto check_debug_log(int iteration) -> void {
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

    const bool exceeds_max_length = *total_length >= kMaxAllowedTotalLength;

    if (*failed_routing != 0) {
        fail_iteration(
            iteration,
            "Failed routing nubmer=" + std::to_string(*failed_routing)
        );
    }

    if (content.find("Routing failed for this net:") != std::string::npos) {
        fail_iteration(iteration, "found per-net routing failure in debug.log");
    }

    std::cout << "Iteration " << iteration << "/" << kIterations
              << ", Total Length=" << *total_length
              << ", Failed routing nubmer=" << *failed_routing;
    if (exceeds_max_length) {
        std::cout << " (warning: exceeds max allowed " << kMaxAllowedTotalLength << ")";
    }
    std::cout << std::endl;
}

} // namespace

void test_placer_iteratively_main() {
    for (int i = 1; i <= kIterations; ++i) {
        run_pr_tool_subprocess(i);
        check_debug_log(i);
    }
    std::cout << "All " << kIterations << " iterations passed" << std::endl;
}
