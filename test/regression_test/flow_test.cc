#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <string_view>
#include <sys/wait.h>

namespace fs = std::filesystem;

namespace PR_tool::test {
namespace {

constexpr int kMazeIterations = 10;
constexpr int kSatIlpIterations = 1;

constexpr const char* kWriterCases[] = {
    "test1_neighbouring_chiplet",
    "test2_chiplet_IO",
    "test3_chiplet_nege",
    "test4_chiplet_pose",
    "test5_muyan0_spi_uart_jtag",
};

auto shell_ok(int status) -> bool {
    return status != -1 && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

auto run_shell(const std::string& cmd) -> int {
    std::cout << "==> " << cmd << std::endl;
    return std::system(cmd.c_str());
}

auto quote_shell(std::string_view s) -> std::string {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') {
            out += "'\\''";
        } else {
            out += c;
        }
    }
    out += "'";
    return out;
}

auto rebuild_flow_targets_best_effort(const fs::path& repo) -> bool {
    bool ok = true;
    for (const char* target : {"PR_tool_cli", "module_test", "json2txt"}) {
        const auto cmd = "cd " + quote_shell(repo.string()) + " && xmake build -P . " + target;
        ok = shell_ok(run_shell(cmd)) && ok;
    }
    return ok;
}

auto find_repo_root() -> fs::path {
    auto cwd = fs::current_path();
    for (auto p = cwd;; p = p.parent_path()) {
        if (fs::exists(p / "xmake.lua")
            && fs::exists(p / "source" / "hardware" / "interposer.hh")) {
            return p;
        }
        if (p == p.root_path()) {
            break;
        }
    }
    if (cwd.filename() == "output") {
        return cwd.parent_path();
    }
    return cwd;
}

auto read_text(const fs::path& path) -> std::string {
    std::ifstream in {path};
    REQUIRE(in.is_open());
    std::ostringstream buf;
    buf << in.rdbuf();
    return buf.str();
}

auto write_text(const fs::path& path, const std::string& content) -> void {
    std::ofstream out {path, std::ios::trunc};
    REQUIRE(out.is_open());
    out << content;
}

struct CwdGuard {
    fs::path previous;
    explicit CwdGuard(const fs::path& next) : previous(fs::current_path()) {
        fs::current_path(next);
    }
    ~CwdGuard() { fs::current_path(previous); }
    CwdGuard(const CwdGuard&) = delete;
    auto operator=(const CwdGuard&) -> CwdGuard& = delete;
};

/// Ensures COB_ARRAY_WIDTH == target; restores previous content if changed.
struct CobArrayWidthGuard {
    fs::path header_path;
    fs::path repo_path;
    std::string original;
    bool changed = false;
    bool needs_rebuild = false;

    explicit CobArrayWidthGuard(fs::path path, fs::path repo, int target_width)
        : header_path(std::move(path))
        , repo_path(std::move(repo))
        , original(read_text(header_path)) {
        static const std::regex re {R"(COB_ARRAY_WIDTH\s*=\s*\d+)"};
        const std::string replacement =
            "COB_ARRAY_WIDTH   = " + std::to_string(target_width);
        const auto updated = std::regex_replace(original, re, replacement);
        std::smatch match;
        REQUIRE(std::regex_search(original, match, re));
        if (updated == original) {
            return;
        }
        write_text(header_path, updated);
        changed = true;
        std::cout << "Set COB_ARRAY_WIDTH=" << target_width << " in " << header_path
                  << std::endl;
    }

    ~CobArrayWidthGuard() {
        if (changed) {
            std::ofstream out {header_path, std::ios::trunc};
            if (out.is_open()) {
                out << original;
                changed = false;
                needs_rebuild = true;
            } else {
                std::cerr << "WARNING: unable to restore " << header_path
                          << " after an aborted flow." << std::endl;
            }
        }
        if (needs_rebuild) {
            if (rebuild_flow_targets_best_effort(repo_path)) {
                std::cerr << "WARNING: restored " << header_path
                          << " and rebuilt flow targets after an aborted flow." << std::endl;
            } else {
                std::cerr << "WARNING: restored " << header_path
                          << " but failed to rebuild every flow target after an aborted flow."
                          << std::endl;
            }
        }
    }

    auto restore() -> bool {
        if (!changed) {
            return false;
        }
        write_text(header_path, original);
        changed = false;
        needs_rebuild = true;
        std::cout << "Restored " << header_path << std::endl;
        return true;
    }

    auto mark_rebuilt() -> void { needs_rebuild = false; }

    CobArrayWidthGuard(const CobArrayWidthGuard&) = delete;
    auto operator=(const CobArrayWidthGuard&) -> CobArrayWidthGuard& = delete;
};

struct FlowCase {
    const char* config_path;
    int cob_array_width;
    bool run_writer;
};

auto build_target(const fs::path& repo, const char* target) -> void {
    const auto cmd = "cd " + quote_shell(repo.string()) + " && xmake build -P . " + target;
    REQUIRE(shell_ok(run_shell(cmd)));
}

auto check_sat_ilp_log(const fs::path& debug_log) -> void {
    const auto content = read_text(debug_log);
    INFO("SAT/ILP debug log: " << debug_log);
    REQUIRE(content.find("validate summary: PASS") != std::string::npos);
    REQUIRE(content.find("SAT routing ok:") != std::string::npos);
    REQUIRE(content.find(
        "v15 ILP optimization begin: threshold=25.00% segment_bbox_pad=0 time_limit_hours=5"
    ) != std::string::npos);
    REQUIRE(content.find("v15 ILP optimization end: status=") != std::string::npos);
    REQUIRE(content.find("SAT_VALIDATION_FAILED") == std::string::npos);
    REQUIRE(content.find("v15 ILP validation: FAIL") == std::string::npos);
}

auto run_sat_ilp_once(const std::string& config_path, bool place, const fs::path& debug_log) -> void {
    const auto cmd = "./PR_tool_cli " + config_path
        + (place ? " -p" : "")
        + " --router sat --ilp-optimize -L 25 --time-limit 5 > /dev/null 2>&1";
    REQUIRE(shell_ok(run_shell(cmd)));
    check_sat_ilp_log(debug_log);
}

auto run_width_flow(const FlowCase& flow_case) -> void {
    const auto repo = find_repo_root();
    const auto header = repo / "source" / "hardware" / "interposer.hh";
    const auto output_dir = repo / "output";
    const auto module_test = output_dir / "module_test";
    const auto debug_log = output_dir / "debug.log";
    const auto run_case = repo / "test" / "module_test" / "test_writer"
        / "check-controlbits-file" / "scripts" / "run_case.sh";

    REQUIRE(fs::exists(header));
    if (flow_case.run_writer) {
        REQUIRE(fs::exists(run_case));
    }

    CobArrayWidthGuard cob_guard {header, repo, flow_case.cob_array_width};
    build_target(repo, "PR_tool_cli");
    build_target(repo, "module_test");
    if (flow_case.run_writer) {
        build_target(repo, "json2txt");
    }
    REQUIRE(fs::exists(module_test));

    CwdGuard cwd_guard {output_dir};

    // 1) Maze 自动布局布线。
    {
        const auto cmd = std::string {"./module_test placer_iteratively "}
            + flow_case.config_path + " " + std::to_string(kMazeIterations);
        REQUIRE(shell_ok(run_shell(cmd)));
    }

    // 2) Maze 自动布线。
    {
        const auto cmd = std::string {"./module_test router_iteratively "}
            + flow_case.config_path + " " + std::to_string(kMazeIterations);
        REQUIRE(shell_ok(run_shell(cmd)));
    }

    // 3) SAT + ILP 自动布局布线。SAT/ILP 较重，因此只执行一次。
    for (int i = 0; i < kSatIlpIterations; ++i) {
        run_sat_ilp_once(flow_case.config_path, /*place=*/true, debug_log);
    }

    // 4) SAT + ILP 自动布线。
    for (int i = 0; i < kSatIlpIterations; ++i) {
        run_sat_ilp_once(flow_case.config_path, /*place=*/false, debug_log);
    }

    // 5) WIDTH=12 的 controlbits writer golden 检查。
    if (flow_case.run_writer) {
        for (const char* name : kWriterCases) {
            const auto case_rel = std::string {"test/module_test/test_writer/"} + name;
            const auto cmd = "bash " + quote_shell(run_case.string()) + " "
                + quote_shell(case_rel);
            REQUIRE(shell_ok(run_shell(cmd)));
        }
    }

    if (cob_guard.restore()) {
        build_target(repo, "PR_tool_cli");
        build_target(repo, "module_test");
        build_target(repo, "json2txt");
        cob_guard.mark_rebuilt();
    }
}

} // namespace

SCENARIO("flow_test_width_12: case5 maze, SAT+ILP, and writer", "[flow][width12]") {
    run_width_flow(FlowCase {
        .config_path = "../test/config/case5",
        .cob_array_width = 12,
        .run_writer = true,
    });
}

SCENARIO("flow_test_width_13: case7 maze and SAT+ILP", "[flow][width13]") {
    run_width_flow(FlowCase {
        .config_path = "../test/config/case7",
        .cob_array_width = 13,
        .run_writer = false,
    });
}

} // namespace PR_tool::test
