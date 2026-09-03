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
│   ├── test_gui/        # Headless Qt GUI tests (xmake target: gui_test)
│   │   ├── component/   # Custom-widget contracts and application-owned state
│   │   ├── interaction/ # Local user actions and state transitions
│   │   └── workflow/    # Cross-widget user tasks (without P&R algorithms)
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
    *   Slow (not in `all`): `placer_iteratively` / `router_iteratively` / `sat_ilp` (spawn `./PR_tool_cli` or unified SAT).
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
xmake build gui_test
xmake run gui_test                 # runs with QT_QPA_PLATFORM=offscreen
xmake build regression_test
xmake build json2txt
```

`gui_test` has one `gui_test_main.cc` that runs the three layers in order. It
uses isolated QSettings and non-native dialogs; keep GUI tests deterministic,
use `test/config/case5` for loaded-design fixtures, and do not invoke Place,
Route, or OpenGL rendering from this headless target.

*   Unit / iterative / writer (from `output/`):

```bash
./module_test placer
./module_test all
./module_test placer_iteratively ../test/config/case5 10
./module_test router_iteratively ../test/config/case5 10
./module_test sat_ilp
./module_test writer ../test/module_test/test_writer/test1_neighbouring_chiplet \
  <net_path_info_new.txt> <output_dir> 0
```

*   **`placer_iteratively`**: Runs `./PR_tool_cli <config> -p` N times; fails on `Failed routing nubmer > 0` or `Routing failed for this net:`; `Total Length >= 1100` warns only.
*   **`router_iteratively`**: Same failure checks; runs `./PR_tool_cli <config>` without `-p`; no Total Length warning.
*   **`sat_ilp`** (`test_unit/test_sat_ilp.cc`): In-process SAT smoke — `build_routing_nets` unit test, `solve_unified_sat_and_commit` on `test/config/case1`, optional `algorithm/test_ILP/test/case_2btb` when `COB_ARRAY_WIDTH==13`. Requires `module_test` built with `PR_TOOL_HAS_SAT_ROUTER=1` (default `xmake f --sat_router=y --cadical=y`). CaDiCal via `third_party/cadical`; Gurobi only if exercising v15 ILP paths.
*   **`[flow]`** (`flow_test.cc`): Two width-tagged flows run serially through `CobArrayWidthGuard`: `[width12]` uses case5 and `[width13]` uses case7.  Each runs maze placement+routing×10 and maze routing-only×10, then SAT+ILP placement+routing×1 and SAT+ILP routing-only×1 with `--router sat --ilp-optimize -L 25 --time-limit 5` (the time-limit unit is hours).  SAT steps require SAT validation PASS, SAT commit success, ILP completion, and no SAT/ILP validation failure.  Only `[width12]` runs `run_case.sh` for writer test1…test5; missing kiwi remains WARNING + exit 0 (SKIP).  A changed width is restored and the flow targets are rebuilt before the test returns.
*   **Regression**: `./regression_test` or `./regression_test "[basic]"` / `"[flow]"`.
*   **`[incremental]`**: tagged `[incremental][.]` — run explicitly: `./regression_test '[incremental]'`.

### COB_ARRAY_WIDTH vs test/config cases

Before running any `test/config/caseN` (maze or SAT):

1. Open `test/config/caseN/description.txt` if present.
2. If it contains `COB array = 9 * W` / `!!! COB array = 9 * W` / `9*W`,
   set `Interposer::COB_ARRAY_WIDTH` in `source/hardware/interposer.hh` to **W** (12 or 13), then rebuild.
3. If there is no `description.txt`, or no COB array line, use **WIDTH=12**.
4. After a temporary header change, restore the previous value (see `[flow]` `CobArrayWidthGuard`).

Reference map (check description if unsure): cases 1–6 and 17–22 → 12; cases 7–16 → 13.
