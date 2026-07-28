# PR_tool Project Test Suite Documentation

This document provides an overview of the `test/` directory for AI agents. It details the testing infrastructure, data formats, and test cases used to validate the PR_tool EDA tool.

## 0. 2个工作规则

- 必须深入理解我给你的材料，在理解的基础上进行后续动作
- 完成修改之后，维护相应的 AGENTS.md文件。如果改动比较大，需要在项目根目录的.plan目录下生成改动记录文件，内容可以参考该目录下已有的改动记录

## Directory Structure

```text
test/
├── config/              # Integration benchmarks (case1 … case22, includes case6)
├── config_3dblox/       # 3DBlox / DEF / LEF style fixtures (optional format track)
├── module_test/
│   ├── test_unit/       # ALL sources linked into `module_test` (xmake: test_unit/**.cc)
│   ├── test_writer/     # Writer golden cases + bash/python harness (no .cc here)
│   └── test_function/   # Extra datasets (testlength, bbox, …); not linked into module_test
├── regression_test/     # Catch2 end-to-end + [flow] orchestrator
└── transform_format/    # txt2json / json2txt
```

*   **`config/`**: Integration test cases (Benchmarks).
    *   Numbered cases `case1` … `case22` (includes `case6`).
    *   Each case: input JSONs + optional `golden.txt` / `description.txt`.
*   **`module_test/test_unit/`**: Unit / iterative / writer C++ entrypoints.
    *   Dispatcher: `test_unit/test.cc` → `./module_test <name>`.
    *   Fast: `cob` `tob` `interposer` `router` `placer` `config` `comparator` `path_length` `debug` `all`.
    *   Slow (not in `all`): `placer_iteratively` / `router_iteratively` (spawn `./PR_tool_cli`).
    *   Writer: `./module_test writer <case_dir> <net_path_info_new.txt> <output_dir> [mode]` — implementation in `test_unit/test_writer.cc`.
*   **`module_test/test_writer/`**: Cases `test1`…`test5` + `check-controlbits-file/` scripts.
    *   Pipeline: `run_case.sh` → json2txt → kiwi golden → path convert → `module_test writer` → `compare_controlbits.py`.
    *   Compares PR `regnamecontrolbit_4part/` (four files, no `-s`) to kiwi golden — no `split_regs` bridge. See `check-controlbits-file/SKILL.md`.
*   **`module_test/test_function/`**: Datasets for length / bbox / wirelength study. Not part of the `module_test` xmake glob (has its own mains under wirelengthstudy).
*   **`regression_test/`**: Catch2 (`test.cc`, `incremental_test.cc`, `flow_test.cc`).
*   **`transform_format/`**: `txt2json.cc` / `json2txt.cc`. Build: `xmake build json2txt`; run: `./json2txt <config_folder> -o <output_dir> [-n name.txt]`.
*   **Related (outside `test/`)**: experimental SAT/ILP router — `algorithm/test_ILP/` (see that directory’s `AGENTS.md`).

## Test Case Structure (`config/`)

A typical test case directory (e.g., `test/config/case4/`) contains:

*   **Input Files**:
    *   `config.json`: Master config; field **`reigster_adder`** (typo key retained) points at file **`register_adder.json`**.
    *   `interposer.json`, `topdies.json`, `topdie_insts.json`, `external_ports.json`, `connections.json`, `01_ports.json`.
    *   `register_adder.json`: Register name → address / quadrant map (required keys: `botleft_REG0.txt` … `topright_REG3.txt`).
*   **Validation Files**:
    *   `golden.txt`: Expected routing result / wirelength bound for regression.
*   **Documentation**:
    *   `description.txt`, optional `*.xlsx`.

## Data Formats

*   **JSON**: Primary configuration; parsed via internal `serde`.
*   **TXT (Legacy/Routing)**: Some cases still use `.txt` netlists (e.g. `case_CPU_8.txt`).

## How to Run Tests

*   Build (from repo root; in a git worktree prefer `xmake build -P . …`):

```bash
xmake build PR_tool_cli
xmake build module_test
xmake build regression_test
xmake build json2txt
```

*   Unit / iterative / writer (from `output/`):

```bash
./module_test placer
./module_test all
./module_test placer_iteratively ../test/config/case5 10
./module_test router_iteratively ../test/config/case5 10
./module_test writer ../test/module_test/test_writer/test1_neighbouring_chiplet \
  <net_path_info_new.txt> <output_dir> 0
```

*   **`placer_iteratively`**: Runs `./PR_tool_cli <config> -p` N times; fails on `Failed routing nubmer > 0` or `Routing failed for this net:`; `Total Length >= 1100` warns only.
*   **`router_iteratively`**: Same failure checks; runs `./PR_tool_cli <config>` without `-p`; no Total Length warning.
*   **`[flow]`** (`flow_test.cc`): Temporarily sets `COB_ARRAY_WIDTH=12`, rebuilds `PR_tool_cli` / `module_test` / `json2txt`, then (1) placer×10 on case5, (2) router×10 on case5, (3) `run_case.sh` for writer test1…test5. Missing kiwi → WARNING + exit 0 (SKIP).
*   **Regression**: `./regression_test` or `./regression_test "[basic]"` / `"[flow]"`.
*   **`[incremental]`**: tagged `[incremental][.]` — run explicitly: `./regression_test '[incremental]'`.
