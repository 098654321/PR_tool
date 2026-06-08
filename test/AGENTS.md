# PR_tool Project Test Suite Documentation

This document provides an overview of the `test/` directory for AI agents. It details the testing infrastructure, data formats, and test cases used to validate the PR_tool EDA tool.

## 0. 2个工作规则

- 必须深入理解我给你的材料，在理解的基础上进行后续动作
- 完成修改之后，维护相应的 AGENTS.md文件。如果改动比较大，需要在项目根目录的.plan目录下生成改动记录文件，内容可以参考该目录下已有的改动记录

## Directory Structure

*   **`config/`**: Integration test cases (Benchmarks).
    *   Contains numbered cases (e.g., `case1`, `case4`, `case20`).
    *   Each case represents a specific design scenario with input configurations and expected outputs.
*   **`module_test/`**: Unit tests.
    *   Target specific modules or classes (e.g., `test_router.cc`, `test_cob.cc`).
    *   Used for verifying isolated functionality.
*   **`regression_test/`**: End-to-end regression testing.
    *   Uses the **Catch2** testing framework (`compile_catch2.cc`).
    *   `test.cc`: Main entry point for regression tests.
*   **`transform_format/`**: Utility tools.
    *   `txt2json.cc`: Tool to convert legacy text-based configurations into the modern JSON format used by the tool.

## Test Case Structure (`config/`)

A typical test case directory (e.g., `test/config/case4/`) contains:

*   **Input Files**:
    *   `config.json`: The master configuration file linking other configs.
    *   `interposer.json`: Interposer physical definition.
    *   `topdies.json`: Definitions of available Top Die types.
    *   `topdie_insts.json`: Instances of Top Dies and their placement coordinates.
    *   `external_ports.json`: Definition of I/O ports.
    *   `connections.json`: The netlist defining connectivity between pins/ports.
    *   `01_ports.json`: Special port configurations (likely for static 0/1 signals).
*   **Validation Files**:
    *   `golden.txt`: Expected output or routing result for regression comparison.
*   **Documentation**:
    *   `description.txt`: Human-readable description of the test case intent.
    *   `*.xlsx`: Excel files sometimes used as the source of truth before conversion to JSON.

## Data Formats

*   **JSON**: The primary format for configuration.
    *   Parsed using the internal `serde` library.
    *   Key structures include `pin_map` (TopDie), `coord` (Placement), and connection arrays.
*   **TXT (Legacy/Routing)**:
    *   Some cases use `.txt` files for routing definitions (e.g., `case_CPU_8.txt`).
    *   Format typically involves source/sink coordinates and net tags.

## How to Run Tests

*   **Unit Tests**: Compiled sources in `module_test/` are typically linked against the core library.
    *   Build: `xmake build PR_tool module_test`
    *   Run from `output/`: `./module_test <test_name>` (e.g. `./module_test placer`)
    *   Run all fast unit tests: `./module_test all`
*   **`placer_iteratively` (slow, not in `all`)**: Runs `./PR_tool ../test/config/case5 -p` 100 times via subprocess. After each run, parses `output/debug.log` and fails if `Total Length >= 1100`, `Failed routing nubmer > 0`, or any `Routing failed for this net:` appears. Expect several minutes of runtime.
    *   Run: `cd output && ./module_test placer_iteratively`
*   **Regression**: The `regression_test` target runs the Catch2 suite, which likely iterates over the `config/` cases, runs the tool, and compares the output against `golden.txt`.
