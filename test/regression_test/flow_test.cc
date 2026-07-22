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

constexpr int kFlowIterations = 10;
constexpr int kTargetCobWidth = 12;

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
    std::string original;
    bool changed = false;

    explicit CobArrayWidthGuard(fs::path path, int target_width)
        : header_path(std::move(path))
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
            write_text(header_path, original);
            std::cout << "Restored " << header_path << std::endl;
        }
    }

    CobArrayWidthGuard(const CobArrayWidthGuard&) = delete;
    auto operator=(const CobArrayWidthGuard&) -> CobArrayWidthGuard& = delete;
};

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

} // namespace

SCENARIO("Flow regression: place+route, route-only, writer controlbits", "[flow]") {
    const auto repo = find_repo_root();
    const auto header = repo / "source" / "hardware" / "interposer.hh";
    const auto output_dir = repo / "output";
    const auto module_test = output_dir / "module_test";
    const auto run_case = repo / "test" / "module_test" / "test_writer"
        / "check-controlbits-file" / "scripts" / "run_case.sh";
    const auto case5 = std::string {"../test/config/case5"};

    REQUIRE(fs::exists(header));
    REQUIRE(fs::exists(run_case));

    WHEN("COB_ARRAY_WIDTH=12, rebuild, then run three checks") {
        CobArrayWidthGuard cob_guard {header, kTargetCobWidth};

        // xmake accepts only one target name per `xmake build` invocation.
        const std::string cd_repo = "cd " + quote_shell(repo.string()) + " && ";
        REQUIRE(shell_ok(run_shell(cd_repo + "xmake build PR_tool_cli")));
        REQUIRE(shell_ok(run_shell(cd_repo + "xmake build module_test")));
        REQUIRE(shell_ok(run_shell(cd_repo + "xmake build json2txt")));
        REQUIRE(fs::exists(module_test));

        CwdGuard cwd_guard {output_dir};

        // 1) 自动布局布线
        {
            const std::string cmd =
                "./module_test placer_iteratively " + case5 + " "
                + std::to_string(kFlowIterations);
            REQUIRE(shell_ok(run_shell(cmd)));
        }

        // 2) 自动布线
        {
            const std::string cmd =
                "./module_test router_iteratively " + case5 + " "
                + std::to_string(kFlowIterations);
            REQUIRE(shell_ok(run_shell(cmd)));
        }

        // 3) 输出正确性（无 kiwi 时 run_case.sh WARNING + exit 0 SKIP）
        for (const char* name : kWriterCases) {
            const std::string case_rel =
                std::string {"test/module_test/test_writer/"} + name;
            const std::string cmd =
                "bash " + quote_shell(run_case.string()) + " "
                + quote_shell(case_rel);
            REQUIRE(shell_ok(run_shell(cmd)));
        }
    }
}

} // namespace PR_tool::test
