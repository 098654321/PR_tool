# Design: CLI v1.0.0 P0 release fixes

Date: 2026-07-22  
Status: approved; implementation plan written

## 1. Goal

Make the command-line product safe to ship as **version 1.0.0** for the non-incremental place-and-route path: reject unsupported incremental features, never emit success artifacts after routing failure, harden TXT/JSON config parsing, ignore legacy single-file warm-start, and provide a Qt-free `PR_tool_cli` binary that `flow_test` exercises.

## 2. Decisions (locked)

| Topic | Choice |
|-------|--------|
| Delivery style | Approach 1: one P0 item per step; each step → `[flow]` → commit |
| P0#3 cobunit init | **Out of scope** this delivery |
| `-i` / `-c` | Remove from `--help`; any use → `debug::fatal` with exact message below |
| Fatal message | `Incremental routing and the relative functions is not supported in version 1.0.0` |
| Routing failure | Continue remaining nets; clear failed net hardware; collect names; log; **no** `output_from_routing_results`; non-zero exit |
| Failed-net cleanup | `net->clear_path()` (releases occupied connectors/tracks via `PathPackage::clear_all`) |
| SyncNet cleanup | `SyncNet::clear_path()` clears the sync group (self + all child nets); no effect on other top-level nets |
| Legacy `controlbits_*.txt` | Ignore for skip-route; always re-route on CLI non-incremental path |
| Four-file readback/compare | Not implemented (P0#6 option A); leave legacy readers unused by CLI |
| `[incremental]` tests | Keep file; exclude from default Catch run (e.g. tag `[.]`); document v1.0.0 unsupported |
| `PR_tool_cli` | New headless target; `flow_test` / iterative module_tests use it |
| Verification gate | After each step: build + run `regression_test` scenario `[flow]`; only then commit |

## 3. Out of scope

- Initializing `TOB::_cobunit_resources` (P0#3)
- Implementing four-file controlbits readback or compare
- Fixing or shipping incremental routing
- P1/P2 items in `TODO.md` unless required by a step above

## 4. Step order and ownership

| Step | P0 | Primary files |
|------|----|---------------|
| 1 | #1 Disable incremental CLI | `source/app/PR_tool.cc`; `test/regression_test/incremental_test.cc`; docs |
| 2 | #2 Fail-continue + no write | `source/algo/router/command_mode/command/route.cc`; `source/algo/router/route_nets.cc`; `source/app/cli/cli.cc` (+ small helpers as needed) |
| 3 | #4 TXT parse hardening | `source/parse/reader/config/config.cc` |
| 4 | #5 JSON pair runtime check | `source/parse/reader/config/config.cc` |
| 5 | #6 Ignore legacy warm-start | `source/app/cli/cli.cc`; comments/TODOs as needed |
| 6 | #7 Split CLI binary | `xmake.lua`; `test/regression_test/flow_test.cc`; `test/module_test/test_{placer,router}_iteratively.cc`; docs |

Each step ends with: `xmake build …` as needed → run `[flow]` → `git add` relevant files → commit.

## 5. Behavior details

### 5.1 Disable incremental (Step 1)

- `print_help()`: remove `-i` / `--incremental` (and any `-c` help if present).
- Before entering CLI main: if argv contains `-i`, `--incremental`, `-c`, or `--compare` → `debug::fatal("Incremental routing and the relative functions is not supported in version 1.0.0")`.
- Tag `incremental_test` so default `regression_test` / `[flow]` does not run it; keep source for later.
- Update `README.md` / `test/AGENTS.md` / `source/AGENTS.md` briefly: v1.0.0 CLI does not support incremental.

### 5.2 Routing failure continue (Step 2)

Non-incremental `Route::execute` loop:

1. For each net from `engine.position()`:
   - `try`: `net->route(...)`; on success `engine.move_on()`.
   - `catch (RetryExpt)`: `net->clear_path()`; record `net->name()`; `engine.move_on()`; continue.
2. Do **not** rely on outer `route_nets` swallowing `RetryExpt` and still writing outputs. Prefer: `Route` / engine accumulates failed names; `route_nets` returns them (or `DataPerCycle` + failed-name list); `cli_main` checks.
3. If any failed names: `debug::error` listing them in the analysis / end-of-route log section; skip `output_from_routing_results`; `return 1` (or equivalent non-zero).
4. If none failed: write four files; `return 0`.

`FinalError` and unexpected exceptions still abort (do not continue). Incremental command path is unreachable from CLI after Step 1; no change required for release correctness.

### 5.3 TXT / JSON (Steps 3–4)

TXT (`load_from_txt`):

- Require exactly 11 integers per data line; reject short/long lines with line content in the exception.
- Bound-check `info[4]` against `externs` size (16) before indexing.

JSON connection pairs:

- Replace `assert(net.size() == 2)` with a runtime exception including map key and pair index (or equivalent location).

### 5.4 Ignore legacy warm-start (Step 5)

In `cli.cc` non-incremental branch (`mode == 0 && !try_all_modes`):

- Always call `route_nets(...)` (no skip when `controlbits_<mode>.txt` exists).
- Do not call `parse::compare` from CLI (already unreachable without `-c`).
- Leave `read_controlbits` / comparator sources in tree as legacy; CLI must not use them for skip-route.

### 5.5 `PR_tool_cli` (Step 6)

- New `xmake` target `PR_tool_cli`: binary under `./output`, no Qt rules/frameworks; compile CLI entry + `algo` / `circuit` / `hardware` / `parse` / `serde` / `global` (and `app/cli`, `app/PR_tool.cc` / `app/main.cc` as appropriate—exclude `widget/**` and GUI-only translation units).
- Keep existing `PR_tool` as Qt GUI+CLI combined target if still needed for GUI.
- `flow_test`: build `PR_tool_cli` (and other needed targets); iterative tests invoke `./PR_tool_cli`.
- `test_placer_iteratively` / `test_router_iteratively`: subprocess command uses `./PR_tool_cli`.

## 6. Error handling and exit codes

| Condition | User-visible | Exit |
|-----------|--------------|------|
| `-i` / `-c` used | FATAL message (exact string) | `-1` via `debug::fatal` |
| One or more nets failed after continue | ERROR lists failed net names; no REG output | non-zero (`1`) |
| TXT/JSON invalid | Exception / fatal with location | non-zero |
| All nets routed | Four files under `regnamecontrolbit_4part/` | `0` |

## 7. Testing

- **Gate every step:** `regression_test` Catch filter `[flow]` (from repo conventions / `output` cwd as today).
- Step 1 extra: confirm `--help` has no incremental lines; `-i` prints FATAL (can be a one-off shell check in the step).
- Step 2: happy path still passes `[flow]`; failure-path covered by unit/module test if a small fixture exists, otherwise document manual check—do not weaken `[flow]` pass criteria.
- Step 6: `[flow]` must build and run against `PR_tool_cli`.

## 8. Docs to touch

- `TODO.md`: mark completed P0 items when done (optional per-step or final).
- `README.md`, `test/AGENTS.md`, `source/AGENTS.md`: incremental unsupported; `PR_tool_cli` build/run.

## 9. Non-goals reminder

No cobunit resource init; no four-file warm-start; no shipping `-i` behind a flag in v1.0.0.
