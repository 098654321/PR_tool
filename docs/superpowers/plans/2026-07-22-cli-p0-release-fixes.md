# CLI v1.0.0 P0 Release Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ship a safe CLI v1.0.0 non-incremental path: reject `-i`/`-c`, continue-on-net-failure without writing REG files, harden TXT/JSON config parsing, ignore legacy `controlbits_*.txt` warm-start, and add Qt-free `PR_tool_cli` used by `[flow]`.

**Architecture:** Six sequential commits matching the approved spec. Routing failure names are collected inside `Route::execute` via `RouteEngine`, returned from `route_nets` to `cli_main`, which skips `output_from_routing_results` and exits non-zero. Packaging ends with a `PR_TOOL_CLI_ONLY` define so the same `PR_tool.cc` builds headless without linking Qt/GUI.

**Tech Stack:** C++23, xmake, Catch2 (`regression_test` `[flow]`), existing `debug::fatal` / `debug::error`.

**Spec:** `docs/superpowers/specs/2026-07-22-cli-p0-release-fixes-design.md`

**Out of scope:** P0#3 (`_cobunit_resources` init).

**Gate (every Task):** After code changes for that Task:

```bash
cd /Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool
xmake build regression_test
cd output && ./regression_test '[flow]'
```

Expected: Catch scenario `[flow]` passes (all assertions). Only then commit that Task.

---

## File map

| File | Responsibility |
|------|----------------|
| `source/app/PR_tool.cc` | Help text; reject `-i`/`-c`; `PR_TOOL_CLI_ONLY` guards for `-g` |
| `source/app/cli/cli.cc` | Skip write on failures; ignore warm-start; consume failed-name list |
| `source/app/cli/cli.hh` | Update `route` / `cli_main` signatures if needed |
| `source/algo/router/command_mode/command/route.cc` | Per-net try/catch, `clear_path`, record failures |
| `source/algo/router/routeengine.hh` (+ `.cc` if needed) | `failed_net_names` storage + accessors |
| `source/algo/router/route_nets.cc` / `.hh` | Return failed names; stop swallowing `RetryExpt` as success |
| `source/parse/reader/config/config.cc` | TXT field count + bounds; JSON runtime pair check |
| `xmake.lua` | `PR_tool_cli` target; `module_test` dep on `PR_tool_cli` |
| `test/regression_test/flow_test.cc` | Build `PR_tool_cli` |
| `test/module_test/test_placer_iteratively.cc` | Subprocess `./PR_tool_cli` |
| `test/module_test/test_router_iteratively.cc` | Subprocess `./PR_tool_cli` |
| `test/regression_test/incremental_test.cc` | Hide from default run (`[.]`) |
| `README.md` / `test/AGENTS.md` / `source/AGENTS.md` / `TODO.md` | Doc sync |

---

### Task 1: Disable incremental CLI (P0#1)

**Files:**
- Modify: `source/app/PR_tool.cc`
- Modify: `test/regression_test/incremental_test.cc`
- Modify: `README.md` (incremental option / note)
- Modify: `test/AGENTS.md` (incremental tag note)
- Modify: `source/AGENTS.md` (one line: v1.0.0 CLI no incremental)

- [ ] **Step 1: Remove incremental help and reject flags**

In `source/app/PR_tool.cc` `print_help()`, delete the `-i, --incremental` block (lines that print incremental help).

In `main`, after building `arguments` and defining `argument_index`, **before** `-g` / CLI branches that accept config, add:

```cpp
        constexpr auto kIncrementalUnsupported =
            "Incremental routing and the relative functions is not supported in version 1.0.0";

        if (argument_index("-i", "--incremental").has_value()
            || argument_index("-c", "--compare").has_value()) {
            debug::fatal(kIncrementalUnsupported);
        }
```

In the CLI branch, remove the block that parses `incre_opt` / `comp_opt` / `try_all_modes` / `compare`. Call:

```cpp
            return cli_main(arguments[0], std::move(output_path), /*mode=*/0, /*compare=*/std::nullopt,
                            /*try_all_modes=*/false, placement, simplify_controlbits);
```

(Keep `cli_main` signature unchanged for now.)

- [ ] **Step 2: Hide `[incremental]` from default Catch runs**

In `test/regression_test/incremental_test.cc`, change:

```cpp
    SCENARIO("Regression test for incremental routing", "[incremental]"){
```

to:

```cpp
    // v1.0.0: incremental unsupported in CLI; run explicitly with [incremental] if needed later.
    SCENARIO("Regression test for incremental routing", "[incremental][.]"){
```

- [ ] **Step 3: Doc one-liners**

- `README.md`: mark `-i` as unsupported in v1.0.0 (or remove from options table and add a short “Unsupported” note).
- `test/AGENTS.md`: note `[incremental][.]` excluded by default.
- `source/AGENTS.md`: under pipeline / CLI, note incremental not supported in v1.0.0 CLI.

- [ ] **Step 4: Smoke-check FATAL + help**

```bash
cd /Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool
xmake build PR_tool
cd output
./PR_tool -h 2>&1 | tee /tmp/pr_help.txt
# Expect: no "-i" / "incremental" option lines
./PR_tool ../test/config/case1 -i 2>&1 | tee /tmp/pr_i.txt; echo EXIT:$?
# Expect: FATAL line with exact message; non-zero exit
```

- [ ] **Step 5: Run `[flow]` gate**

```bash
cd /Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool
xmake build regression_test
cd output && ./regression_test '[flow]'
```

Expected: all `[flow]` assertions pass.

- [ ] **Step 6: Commit**

```bash
git add source/app/PR_tool.cc test/regression_test/incremental_test.cc README.md test/AGENTS.md source/AGENTS.md
git commit -m "$(cat <<'EOF'
fix(cli): reject incremental flags in v1.0.0

Remove -i/-c from help and FATAL on use; hide [incremental] from default Catch runs.
EOF
)"
```

---

### Task 2: Fail-continue routing without writing outputs (P0#2)

**Files:**
- Modify: `source/algo/router/routeengine.hh`
- Modify: `source/algo/router/command_mode/command/route.cc`
- Modify: `source/algo/router/route_nets.hh`
- Modify: `source/algo/router/route_nets.cc`
- Modify: `source/app/cli/cli.hh`
- Modify: `source/app/cli/cli.cc`
- Grep and update other `route_nets(` call sites if return type changes (`source/app/gui/**`, `source/widget/**`, tests)

- [ ] **Step 1: Add failed-name storage on `RouteEngine`**

In `source/algo/router/routeengine.hh`, add public API and private member:

```cpp
    auto record_failed_net(const std::String& name) -> void {
        this->_failed_net_names.emplace_back(name);
    }
    auto failed_net_names() const -> const std::Vector<std::String>& {
        return this->_failed_net_names;
    }
    auto clear_failed_net_names() -> void {
        this->_failed_net_names.clear();
    }
```

Private:

```cpp
    std::Vector<std::String> _failed_net_names {};
```

Include `<std/string.hh>` if not already available via other headers.

- [ ] **Step 2: Per-net try/catch in `Route::execute`**

Replace the body of `Route::execute` in `source/algo/router/command_mode/command/route.cc` with:

```cpp
auto Route::execute(hardware::Interposer* interposer, RouteEngine& engine) const -> void {
    debug::info("routing ...");

    auto nets = engine.nets();
    auto posi = engine.position();
    for (std::usize i = posi; i < nets.size(); ++i) {
        auto net = nets[i];
        net->set_reuse_type(false);

        auto routed_nets = engine.routed_nets();
        net->search_related_nets(routed_nets);

        try {
            net->route(interposer, engine.routestrategy());
        } catch (const RetryExpt& err) {
            debug::info(err.what());
            show_retry_expt(net, engine, interposer);  // or err.net() if non-null; prefer `net`
            net->clear_path();
            engine.record_failed_net(net->name());
        }
        engine.move_on();
    }
}
```

Add includes:

```cpp
#include <algo/router/routeerror.hh>
#include <algo/router/route_nets.hh>  // for show_retry_expt; or move show_retry_expt decl to a header Route can use
```

If including `route_nets.hh` from `route.cc` is awkward (cycles), either:
- call `show_retry_expt` only if already accessible, or
- inline `debug::info(err.what())` without `show_retry_expt` in v1 (acceptable: still clear_path + record name).

Prefer keeping `show_retry_expt` if linkable without cycle; otherwise log `err.what()` only.

- [ ] **Step 3: Change `route_nets` return to include failed names**

In `route_nets.hh`:

```cpp
    struct RouteNetsResult {
        DataPerCycle data;
        std::Vector<std::String> failed_net_names;
    };

    auto route_nets(
        hardware::Interposer* interposer,
        circuit::BaseDie* basedie,
        const RouteStrategy& strategy,
        const AllocateStrategy& allocator,
        int m,
        bool incremental,
        bool try_all_modes,
        bool path_exists = false
    ) -> RouteNetsResult;
```

In `route_nets.cc`, change the `RetryExpt` handler so a `RetryExpt` that still escapes a command is **not** treated as soft success. After Step 2, `Route` should not throw `RetryExpt`. Keep:

```cpp
        catch (const RetryExpt& err) {
            // Should not happen for non-incremental Route after per-net catch.
            // Re-throw as FinalError so CLI does not write outputs.
            throw FinalError(std::String{"Unhandled RetryExpt: "} + err.what());
        }
```

At end of `route_nets`:

```cpp
        auto route_data = analyze_results(interposer, engine, incremental, try_all_modes);
        return RouteNetsResult{route_data, engine.failed_net_names()};
    }
```

- [ ] **Step 4: Update all `route_nets` call sites**

Grep `route_nets(` under `source/` and `test/`. Pattern:

```cpp
auto result = algo::route_nets(...);
auto route_data = result.data;
// or structured binding:
auto [route_data, failed_names] = /* if you add a tuple conversion */;
```

For GUI/widget callers that ignore failures today: at minimum compile; prefer checking `!result.failed_net_names.empty()` and skipping export / showing error (minimal: log + skip write if they call writer).

- [ ] **Step 5: Wire CLI to skip output and return 1**

In `cli.cc` `route(...)` non-incremental branch:

```cpp
            auto result = algo::route_nets(interposer, basedie, algo::MazeRouteStrategy{false}, algo::HK{}, mode, false, try_all_modes);
            if (!result.failed_net_names.empty()) {
                debug::error("Routing failed for the following net(s):");
                for (const auto& name : result.failed_net_names) {
                    debug::error_fmt("  {}", name);
                }
                // Propagate failure to cli_main: use exception or out-param.
                throw std::runtime_error("routing incomplete: one or more nets failed");
            }
            return true;
```

Cleaner (preferred): change `route` to return an enum / struct:

```cpp
enum class RouteStatus { Skipped, Ok, Failed };

// route(...) -> RouteStatus
// Failed => cli_main does not call output_from_routing_results; return 1
```

Implement `RouteStatus` in `cli.hh` and:

```cpp
        auto status = route(...);
        if (status == RouteStatus::Ok) {
            parse::output_from_routing_results(...);
            return 0;
        }
        if (status == RouteStatus::Failed) {
            return 1;
        }
        return 0; // Skipped
```

Ensure `cli_main` catch path also returns non-zero (today it falls off without `return` after `catch` — fix to `return 1`).

- [ ] **Step 6: Run `[flow]` gate**

Same gate command as Task 1 Step 5. Expected: PASS (case5 should fully route).

- [ ] **Step 7: Commit**

```bash
git add source/algo/router/routeengine.hh \
  source/algo/router/command_mode/command/route.cc \
  source/algo/router/route_nets.hh source/algo/router/route_nets.cc \
  source/app/cli/cli.hh source/app/cli/cli.cc \
  # plus any updated call sites
git commit -m "$(cat <<'EOF'
fix(router): continue after net failure and skip REG write

Catch RetryExpt per net, clear_path, collect names; CLI exits non-zero without output_from_routing_results.
EOF
)"
```

---

### Task 3: Harden TXT config parsing (P0#4)

**Files:**
- Modify: `source/parse/reader/config/config.cc`

- [ ] **Step 1: Enforce exactly 11 integers per data line**

In `load_from_txt`, replace the unbounded parse loop with:

```cpp
            std::Array<int, 11> numbers{};
            std::stringstream ss(line);
            int num = 0;
            int pos = 0;
            while (ss >> num) {
                if (pos >= 11) {
                    throw std::runtime_error(std::format(
                        "TXT connections line has more than 11 integers: '{}'", line));
                }
                numbers[static_cast<std::usize>(pos++)] = num;
            }
            if (pos != 11) {
                throw std::runtime_error(std::format(
                    "TXT connections line must have exactly 11 integers (got {}): '{}'",
                    pos, line));
            }

            parse_txt_line(topdie_name1, topdie_name2, numbers, config, mode, try_all_modes);
```

Remove the old uninitialized `std::Array<int, 11> numbers;` declared outside the loop (move inside as above).

- [ ] **Step 2: Bounds-check `externs[info[4]]`**

In `parse_txt_line` / `parse_node`, before indexing `externs`:

```cpp
                    if (info[4] < 0 || static_cast<std::usize>(info[4]) >= externs.size()) {
                        throw std::runtime_error(std::format(
                            "TXT external port index out of range: {}", info[4]));
                    }
```

- [ ] **Step 3: Run `[flow]` gate**

Same as Task 1 Step 5. Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add source/parse/reader/config/config.cc
git commit -m "$(cat <<'EOF'
fix(config): require 11 TXT fields and bound-check extern index

Reject short/long connection lines and invalid external port indices at parse time.
EOF
)"
```

---

### Task 4: JSON connection pair runtime check (P0#5)

**Files:**
- Modify: `source/parse/reader/config/config.cc`

- [ ] **Step 1: Replace `assert(net.size() == 2)`**

In the `Deserialize` specialization for connection maps:

```cpp
            for (std::usize i = 0; i < vec.size(); ++i) {
                auto& net = vec[i];
                if (net.size() != 2) {
                    throw std::runtime_error(std::format(
                        "Connection pair under sync key '{}' at index {} must have exactly 2 endpoints, got {}",
                        key, i, net.size()));
                }
                nets.emplace_back(ConnectionConfig{net[0], net[1]});
            }
```

Remove `#include <cassert>` usage for this assert if unused elsewhere in the file.

- [ ] **Step 2: Run `[flow]` gate**

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add source/parse/reader/config/config.cc
git commit -m "$(cat <<'EOF'
fix(config): validate JSON connection pairs at runtime

Replace assert(net.size()==2) with an exception that survives Release builds.
EOF
)"
```

---

### Task 5: Ignore legacy controlbits warm-start (P0#6 A)

**Files:**
- Modify: `source/app/cli/cli.cc`
- Modify: `source/parse/reader/module.cc` / `controlbits.cc` comments if needed (clarify legacy-only)

- [ ] **Step 1: Always route on non-incremental CLI path**

Replace the non-incremental branch in `route(...)` with:

```cpp
        if (!try_all_modes && mode == 0) {
            // v1.0.0: ignore legacy controlbits_<mode>.txt warm-start / skip-route.
            auto result = algo::route_nets(
                interposer, basedie, algo::MazeRouteStrategy{false}, algo::HK{},
                mode, false, try_all_modes);
            if (!result.failed_net_names.empty()) {
                debug::error("Routing failed for the following net(s):");
                for (const auto& name : result.failed_net_names) {
                    debug::error_fmt("  {}", name);
                }
                return RouteStatus::Failed;
            }
            return RouteStatus::Ok;
        }
```

Remove `parse::read_controlbits(...)` call from this branch.

Leave the incremental branch in source but unreachable from CLI after Task 1 (optional: `debug::fatal` if entered).

- [ ] **Step 2: Comment legacy readers**

At `load_controlbits` / `read_controlbits` TODO: note “CLI v1.0.0 does not call this for skip-route”.

- [ ] **Step 3: Run `[flow]` gate**

Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add source/app/cli/cli.cc source/parse/reader/module.cc source/parse/reader/controlbits/controlbits.cc
git commit -m "$(cat <<'EOF'
fix(cli): always re-route; ignore legacy controlbits warm-start

v1.0.0 non-incremental CLI no longer skips routing when controlbits_*.txt exists.
EOF
)"
```

---

### Task 6: Add `PR_tool_cli` and point `[flow]` at it (P0#7)

**Files:**
- Modify: `xmake.lua`
- Modify: `source/app/PR_tool.cc` (`PR_TOOL_CLI_ONLY`)
- Modify: `test/regression_test/flow_test.cc`
- Modify: `test/module_test/test_placer_iteratively.cc`
- Modify: `test/module_test/test_router_iteratively.cc`
- Modify: `README.md` / `source/AGENTS.md` / `TODO.md`

- [ ] **Step 1: Guard GUI in `PR_tool.cc`**

```cpp
        if (argument_index("-g", "--gui").has_value()) {
#ifdef PR_TOOL_CLI_ONLY
            debug::fatal("GUI is not available in PR_tool_cli; build/run PR_tool for GUI");
#else
            if (arguments[0] != "-g" && arguments[0] != "--gui") {
                debug::warning_fmt("Use gui model but indicate input config '{}', it will be ignored", arguments[0]);
            }
            return gui_main(argc, argv);
#endif
        }
```

Wrap `#include "gui/gui.hh"`:

```cpp
#ifndef PR_TOOL_CLI_ONLY
#include "gui/gui.hh"
#endif
```

- [ ] **Step 2: Add `PR_tool_cli` target in `xmake.lua`**

Insert after the `PR_tool` target:

```lua
target("PR_tool_cli")
    set_kind("binary")
    set_targetdir("./output")
    set_basename("PR_tool_cli")
    set_default(false)
    add_defines("PR_TOOL_CLI_ONLY")
    add_includedirs("source", "source/global")
    add_files(
        "source/app/main.cc",
        "source/app/PR_tool.cc",
        "source/app/cli/**.cc",
        "source/algo/**.cc",
        "source/circuit/**.cc",
        "source/global/**.cc",
        "source/hardware/**.cc",
        "source/parse/**.cc",
        "source/serde/**.cc"
    )
```

Do **not** add `qt.widgetapp` / `qt.opengl` / `widget/**` / `resource.qrc`.

Change `module_test` dependency:

```lua
    add_deps("PR_tool_cli")
```

(Keep `PR_tool` Qt target for GUI.)

- [ ] **Step 3: Point iterative tests and `flow_test` at `PR_tool_cli`**

`test_placer_iteratively.cc` / `test_router_iteratively.cc`:

```cpp
    return "./PR_tool_cli " + config_path + " -p > /dev/null 2>&1";
// and without -p for router
    return "./PR_tool_cli " + config_path + " > /dev/null 2>&1";
```

`flow_test.cc`:

```cpp
        REQUIRE(shell_ok(run_shell(cd_repo + "xmake build PR_tool_cli")));
        REQUIRE(shell_ok(run_shell(cd_repo + "xmake build module_test")));
        REQUIRE(shell_ok(run_shell(cd_repo + "xmake build json2txt")));
```

- [ ] **Step 4: Docs + TODO**

- Document `xmake build PR_tool_cli` / `xmake run PR_tool_cli`.
- In `TODO.md`, check off completed P0 items (#1,#2,#4,#5,#6,#7); leave #3 open.

- [ ] **Step 5: Build smoke**

```bash
cd /Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool
xmake build PR_tool_cli
# Should succeed without Qt frameworks on the link line
otool -L output/PR_tool_cli | head   # macOS: expect no QtWeb*/QtWidgets if successful
```

- [ ] **Step 6: Run `[flow]` gate**

Expected: PASS; log shows `xmake build PR_tool_cli` and `./PR_tool_cli` subprocesses via module_test.

- [ ] **Step 7: Commit**

```bash
git add xmake.lua source/app/PR_tool.cc \
  test/regression_test/flow_test.cc \
  test/module_test/test_placer_iteratively.cc \
  test/module_test/test_router_iteratively.cc \
  README.md source/AGENTS.md TODO.md
git commit -m "$(cat <<'EOF'
build: add Qt-free PR_tool_cli and point flow regression at it

Headless binary for release packaging; iterative/flow tests invoke PR_tool_cli.
EOF
)"
```

---

## Self-review (plan vs spec)

| Spec requirement | Task |
|------------------|------|
| Reject `-i`/`-c` + exact FATAL + help | Task 1 |
| Hide `[incremental]` default | Task 1 |
| Fail-continue + `clear_path` + log names + no write + non-zero | Task 2 |
| SyncNet cleared via `clear_path` | Task 2 (uses `Net::clear_path` / override) |
| TXT 11 fields + externs bounds | Task 3 |
| JSON runtime pair check | Task 4 |
| Ignore legacy warm-start | Task 5 |
| `PR_tool_cli` + flow uses it | Task 6 |
| Skip P0#3 | Explicit out of scope |
| `[flow]` + commit per step | Each Task gate + commit steps |

No TBD placeholders. Return type `RouteNetsResult` is consistent across Tasks 2–5.

---

## Execution handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-22-cli-p0-release-fixes.md`.

**Two execution options:**

1. **Subagent-Driven (recommended)** — fresh subagent per task, review between tasks  
2. **Inline Execution** — execute tasks in this session with executing-plans checkpoints  

Which approach?
