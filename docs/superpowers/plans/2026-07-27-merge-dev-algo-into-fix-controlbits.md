# Merge `dev.algo_SAT_MCF_latest` into `fix.controlbits` Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Merge algorithm-dev branch content into `fix.controlbits` while keeping fix’s controlbits/CLI/writer fixes, adopting `test_unit/` layout, unifying `register_adder.json` naming, and verifying with flow regression plus a lightweight `test_ILP` smoke suite.

**Architecture:** Checkout `fix.controlbits` as the integration branch and merge `dev.algo_SAT_MCF_latest` into it. Resolve structural conflicts by keeping fix’s `source/` product fixes and `test_writer/` **cases + scripts harness**, relocating unit/iterative/`test_writer.cc` under `test/module_test/test_unit/`, deleting root `AGENTS.md`, keeping current-branch `case5/golden.txt`, and narrowing `module_test` xmake to `test_unit/**.cc` only (so `wirelengthstudy` mains are not linked). `run_case.sh` already invokes `./module_test writer ...` and does not hardcode the `.cc` path—no functional change required there beyond doc path updates. Defer polish of `.gitignore` / `README` / `source/AGENTS.md` / full `xmake` cleanup; leave `tools/parse_controlbits.cc` alone (4-part output may invalidate it).

**Tech Stack:** git, xmake, Catch2 (`regression_test`), CaDiCal (`test_ILP`), optional kiwi (writer compare; skips if missing), Gurobi only if running `--ilp-optimize` smoke.

**Decisions locked (from discussion):**
1. Merge direction: `dev.algo_SAT_MCF_latest` → `fix.controlbits` (on fix, merge dev).
2. Prefer `test_unit/` + `test_function/` layout from dev; port fix unit/iterative tests into `test_unit/`.
3. Keep fix `test/module_test/test_writer/` for **cases + check-controlbits scripts** (`run_case.sh`, compare, etc.). Move fix’s `scripts/test_writer.cc` → `test/module_test/test_unit/test_writer.cc` (replace the empty stub from dev). CLI remains `./module_test writer ...`.
4. Repo-wide rename file `reigster_adder.json` → `register_adder.json`; JSON **key** stays `"reigster_adder"` (schema typo retained); update `"reigster_adder": "..."` **values** to `"register_adder.json"`.
5. `test/config/case5/golden.txt`: keep **dev** content (`1119`).
6. Root `AGENTS.md`: **delete**.
7. `TODO.md`: keep **fix** version.
8. `parse_controlbits.cc`: no intentional fix this merge.
9. `.gitignore` / `README` / `source/AGENTS.md` / remaining `xmake` polish: accept merge result, only fix what is required to build/test; fuller unify later.
10. `module_test` compiles **only** `test/module_test/test_unit/**.cc` (includes `test_writer.cc` after the move). Do **not** glob `test_function/**.cc` or `test_writer/**.cc`.

**Merge-base reference:** `44387d4`. On fix, `ours` = fix, `theirs` = dev.

---

## File structure after Tasks 1–5

```text
test/module_test/
├── test_unit/                          # ALL module_test .cc sources
│   ├── test.cc                         # dispatcher (writer + iterative + REGISTER_TEST)
│   ├── test_writer.cc                  # from fix scripts/ (provides test_writer_main)
│   ├── test_config.cc                  # fix’s register_map unpack
│   ├── test_placer.cc                  # fix’s SAPlaceStrategy 5-arg ctor
│   ├── test_placer_iteratively.cc      # from fix (moved here)
│   ├── test_router_iteratively.cc      # from fix (moved here)
│   ├── test_*.cc                       # cob/tob/router/... from dev layout
│   └── utilty.hh
├── test_function/                      # keep from dev (cases + wirelengthstudy sources)
│   └── ...                             # NOT linked into module_test binary
└── test_writer/                        # cases + bash/python harness only (no .cc)
    ├── check-controlbits-file/scripts/ # run_case.sh, compare_*.py, … (no test_writer.cc)
    └── test1_.../                      # cases; register_adder.json after rename
```

---

### Task 1: Branch checkout and merge commit (conflicts only for decided files)

**Files:**
- Modify via merge: whole tree
- Expect conflict or special handling: `TODO.md`, `test/config/case5/golden.txt`, root `AGENTS.md`, possibly `.gitignore` / `README.md` / `source/AGENTS.md` / `xmake.lua` / `test/module_test/*` modify-delete

- [ ] **Step 1: Ensure clean working tree on current clone**

Run:

```bash
cd /Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool
git status -sb
```

Expected: clean enough to switch branches (no unrelated WIP you care about). If dirty, stash or commit first.

- [ ] **Step 2: Checkout fix and merge dev**

```bash
git checkout fix.controlbits
git merge dev.algo_SAT_MCF_latest
```

Expected: merge stops with conflicts and/or modify-delete prompts. Do **not** `--abort` unless instructed.

- [ ] **Step 3: Resolve decided text conflicts**

While still in the merge:

```bash
# TODO: keep fix (ours)
git checkout --ours TODO.md
git add TODO.md

# case5 golden: keep dev (theirs) = "1119"
git checkout --theirs test/config/case5/golden.txt
git add test/config/case5/golden.txt

# Root AGENTS.md: delete
git rm -f AGENTS.md || rm -f AGENTS.md
git add -u AGENTS.md

# For .gitignore / README.md / source/AGENTS.md: if conflicted, take a combined
# or either side for now (polish later). Prefer keeping both sides' unique bullets
# when trivial; otherwise `git checkout --ours` then re-add any obvious lost lines later.
```

If `xmake.lua` auto-merged cleanly, leave it for Task 4. If conflicted, finish with markers removed by preferring **both** `PR_tool_cli` (fix) and `test_ILP` / `test_ILP_unit` (dev) blocks present; exact glob fix is Task 4.

- [ ] **Step 4: Do not finish module_test path resolution yet**

If git reports modify/delete on `test/module_test/test.cc` (and siblings):

```bash
# Keep deleted at old path for now; Task 2 restores content under test_unit/
git rm -f test/module_test/test.cc \
         test/module_test/test_config.cc \
         test/module_test/test_placer.cc \
         test/module_test/test_cob.cc \
         test/module_test/test_comparator.cc \
         test/module_test/test_debug.cc \
         test/module_test/test_interposer.cc \
         test/module_test/test_path_length.cc \
         test/module_test/test_router.cc \
         test/module_test/test_tob.cc \
         test/module_test/utilty.hh 2>/dev/null || true
```

Ensure `test/module_test/test_unit/` from theirs is staged, and `test/module_test/test_writer/` from ours remains. If iterative files only exist at old paths, keep them temporarily:

```bash
git add test/module_test/test_placer_iteratively.cc \
        test/module_test/test_router_iteratively.cc 2>/dev/null || true
git add test/module_test/test_writer test/module_test/test_unit test/module_test/test_function
```

- [ ] **Step 5: Complete merge commit even if Task 2–4 will follow immediately**

```bash
git status
# Stage any remaining resolved paths
git add -A
git commit -m "$(cat <<'EOF'
merge: bring dev.algo_SAT_MCF_latest into fix.controlbits

Integrate algorithm/test_ILP and test_unit layout with controlbits/CLI fixes.
Structural cleanup of module_test and register_adder rename follow in subsequent commits.
EOF
)"
```

Expected: merge commit succeeds. Working tree may still be broken to build; that is OK until Task 4.

---

### Task 2: Relocate fix unit/iterative/`test_writer.cc` into `test_unit/` and wire dispatcher

**Files:**
- Modify: `test/module_test/test_unit/test.cc`
- Modify: `test/module_test/test_unit/test_config.cc`
- Modify: `test/module_test/test_unit/test_placer.cc`
- Create (move): `test/module_test/test_unit/test_placer_iteratively.cc`
- Create (move): `test/module_test/test_unit/test_router_iteratively.cc`
- Create (move/replace): `test/module_test/test_unit/test_writer.cc` ← from fix `scripts/test_writer.cc` (overwrites empty stub)
- Delete: `test/module_test/test_writer/check-controlbits-file/scripts/test_writer.cc` (after move)
- Modify (docs only): `test/module_test/test_writer/check-controlbits-file/SKILL.md`, `.../references/tools-and-paths.md` — point writer source to `test_unit/test_writer.cc`
- Optional comment touch: `test/module_test/test_writer/check-controlbits-file/scripts/run_case.sh` (behavior unchanged; still calls `"$MODULE_TEST" writer ...`)
- Delete if still present at old paths: `test/module_test/test_placer_iteratively.cc`, `test/module_test/test_router_iteratively.cc`

- [ ] **Step 1: Move iterative sources into `test_unit/`**

```bash
# If files still at module_test root (from fix):
git mv test/module_test/test_placer_iteratively.cc test/module_test/test_unit/test_placer_iteratively.cc
git mv test/module_test/test_router_iteratively.cc test/module_test/test_unit/test_router_iteratively.cc
```

If already only under another path, place them at the `test_unit/` paths above. Includes stay `#include "./utilty.hh"` (same directory).

- [ ] **Step 2: Move `test_writer.cc` into `test_unit/` (replace empty stub)**

`run_case.sh` does **not** compile or path-reference the `.cc` file; it only runs:

```bash
"$MODULE_TEST" writer "$CASE" "$CHECK/net_path_info_new.txt" "$CHECK" "$MODE"
```

So relocating the source is enough for the pipeline; xmake will pick it up via `test_unit/**.cc` (Task 4).

```bash
# Remove empty stub from dev if still present as a tracked empty file
git rm -f test/module_test/test_unit/test_writer.cc 2>/dev/null || rm -f test/module_test/test_unit/test_writer.cc

git mv test/module_test/test_writer/check-controlbits-file/scripts/test_writer.cc \
       test/module_test/test_unit/test_writer.cc
```

No `#include "./utilty.hh"` dependency in `test_writer.cc` today (uses only `source/` headers); no include-path edits required.

- [ ] **Step 3: Update harness docs that still say `scripts/test_writer.cc`**

In `test/module_test/test_writer/check-controlbits-file/SKILL.md` and `references/tools-and-paths.md`, replace references such as:

- `scripts/test_writer.cc` → `../../test_unit/test_writer.cc` or absolute-from-repo `test/module_test/test_unit/test_writer.cc`
- Any note that `module_test` is built via `test/module_test/**.cc` → `test/module_test/test_unit/**.cc`

Optionally add one comment line near step `[5/6]` in `run_case.sh`:

```bash
# Writer implementation: test/module_test/test_unit/test_writer.cc (linked into module_test)
```

Do **not** change the `"$MODULE_TEST" writer ...` invocation.

- [ ] **Step 4: Patch `test_unit/test_config.cc` for `register_map`**

`parse::read_config` on fix returns three values. Replace both case helpers:

```cpp
static auto test_case1() -> void  try{
    auto [i, b, register_map] = parse::read_config("../test/config/case1", 0, false);
    (void)register_map;
    print_config(i.get(), b.get());
}
THROW_UP_WITH("test_case1")

static auto test_case2() -> void  try{
    auto [i, b, register_map] = parse::read_config("../test/config/case2", 0, false);
    (void)register_map;
    print_config(i.get(), b.get());
}
THROW_UP_WITH("test_case2")
```

- [ ] **Step 5: Patch `test_unit/test_placer.cc` for 5-arg `SAPlaceStrategy`**

Replace the live strategy initializer used by `test_basic_placement` with:

```cpp
    algo::SAPlaceStrategy strategy {
        100.0,  // init_temp
        0.5,    // freeze_temp
        50,     // solve_num
        0.95,   // cooling_rate
        50      // max_no_improvement
    };
```

And unpack `register_map` anywhere `read_config` is called in this file:

```cpp
        auto [interposer, basedie, register_map] = read_config(case_path, mode, try_all_modes);
        (void)register_map;
```

- [ ] **Step 6: Patch `test_unit/test.cc` dispatcher for writer + iterative**

Replace the body of `main` so it matches fix’s argc/dispatch behavior while keeping existing `REGISTER_TEST` list. Final `main` should look like:

```cpp
extern void test_cob_main();
extern void test_interposer_main();
extern void test_tob_main();
extern void test_router_main();
extern void test_placer_main();
extern void test_debug_main();
extern void test_config_main();
extern void test_comparator_main();
extern void test_path_length_main();
extern void test_placer_iteratively_main(int argc, char** argv);
extern void test_router_iteratively_main(int argc, char** argv);
extern void test_writer_main(int argc, char** argv);

#define REGISTER_TEST(test_name)\
functions.emplace(#test_name, & test_##test_name##_main);\
if (target == #test_name) {\
    PR_tool::console::println_fmt("Run test '{}'", #test_name);\
    test_##test_name##_main();\
    return 0;\
}\

int main(int argc, char** argv)
try {
    assert(argc >= 2);

    auto functions = std::HashMap<std::StringView, TestFunction>{};
    auto target = std::StringView{argv[1]};

    if (target == "writer") {
        PR_tool::console::println_fmt("Run test 'writer'");
        test_writer_main(argc, argv);
        return 0;
    }
    PR_tool::debug::set_debug_level(PR_tool::debug::DebugLevel::Info);
    PR_tool::debug::initial_log("./debug.log");

    REGISTER_TEST(cob)
    REGISTER_TEST(tob)
    REGISTER_TEST(interposer)
    REGISTER_TEST(router)
    REGISTER_TEST(placer)
    REGISTER_TEST(debug)
    REGISTER_TEST(config)
    REGISTER_TEST(comparator)
    REGISTER_TEST(path_length)

    if (target == "placer_iteratively") {
        PR_tool::console::println_fmt("Run test 'placer_iteratively'");
        test_placer_iteratively_main(argc, argv);
        return 0;
    }

    if (target == "router_iteratively") {
        PR_tool::console::println_fmt("Run test 'router_iteratively'");
        test_router_iteratively_main(argc, argv);
        return 0;
    }

    if (target == "all") {
        for (auto [test_name, test_func] : functions) {
            PR_tool::console::println_fmt("Run test '{}'", test_name);
            test_func();
            PR_tool::console::println("");
        }
        return 0;
    }

    PR_tool::console::println_fmt("No exit test target '{}'", target);
    return 0;
}
catch (const std::exception& err) {
    PR_tool::console::error_fmt("Error in test: {}", err.what());
}
```

- [ ] **Step 7: Verify layout**

```bash
ls test/module_test/*.cc 2>/dev/null || echo "OK: no root-level module_test cc"
ls test/module_test/test_unit/*.cc
test -f test/module_test/test_unit/test_writer.cc && echo "writer cc in test_unit OK"
test ! -f test/module_test/test_writer/check-controlbits-file/scripts/test_writer.cc && echo "scripts/test_writer.cc removed OK"
test -f test/module_test/test_writer/check-controlbits-file/scripts/run_case.sh && echo "run_case.sh kept OK"
```

Expected: all unit/iterative/writer `.cc` under `test_unit/`; `scripts/` keeps bash/python only.

- [ ] **Step 8: Commit**

```bash
git add test/module_test
git commit -m "$(cat <<'EOF'
refactor(test): move unit/iterative/writer sources under test_unit

Keep test_writer cases and run_case.sh under test_writer/; module_test writer CLI unchanged.
EOF
)"
```

---

### Task 3: Repo-wide `register_adder.json` rename

**Files:**
- Rename every `**/reigster_adder.json` → `**/register_adder.json`
- Modify every `**/config.json` whose value still points at `reigster_adder.json`
- Includes: `test/config/**`, `algorithm/test_ILP/test/**`, `test/module_test/test_function/**`, `test/module_test/test_writer/**`, and any other hits

**Invariant:** JSON **key** remains `"reigster_adder"` (code reads that field name). Only the **filename** and config **value** change.

- [ ] **Step 1: Inventory before change**

```bash
git ls-files '*reigster_adder.json' | tee /tmp/reigster_files.txt
rg -n '"reigster_adder"\s*:\s*"reigster_adder\.json"' -g 'config.json' | tee /tmp/reigster_config_values.txt
wc -l /tmp/reigster_files.txt /tmp/reigster_config_values.txt
```

Expected: dozens of files; writer cases on fix still use typo filename.

- [ ] **Step 2: Rename files with git**

```bash
while IFS= read -r f; do
  dir=$(dirname "$f")
  git mv "$f" "$dir/register_adder.json"
done < /tmp/reigster_files.txt
```

If a destination already exists (some cases already renamed on one side), resolve by keeping one file named `register_adder.json` and removing the typo name:

```bash
# example pattern when both exist:
# git rm path/reigster_adder.json
# keep path/register_adder.json
```

Also fix cases where `config.json` already says `register_adder.json` but only typo file existed (e.g. former case5 mismatch): after rename, file and value must agree.

- [ ] **Step 3: Update config values still pointing at typo filename**

```bash
# macOS BSD sed; adjust if Linux
rg -l '"reigster_adder"\s*:\s*"reigster_adder\.json"' -g 'config.json' | while IFS= read -r c; do
  sed -i '' 's/"reigster_adder.json"/"register_adder.json"/g' "$c"
done
```

Do **not** rename the key `reigster_adder` to `register_adder`.

- [ ] **Step 4: Verify zero typo filenames remain; spot-check configs**

```bash
git ls-files '*reigster_adder.json' | wc -l
# expect 0

rg -n 'reigster_adder\.json' -g 'config.json' || echo "OK: no typo filenames in config values"
rg -n '"reigster_adder"' -g 'config.json' | head
# key still present

# spot-check case5
cat test/config/case5/config.json
ls test/config/case5/register_adder.json
cat test/config/case5/golden.txt
# golden must still be 1119
```

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "$(cat <<'EOF'
fix(config): rename reigster_adder.json to register_adder.json repo-wide

Keep config key name reigster_adder for schema compatibility; point values at register_adder.json.
EOF
)"
```

---

### Task 4: Make `xmake.lua` `module_test` compile only `test_unit/**.cc`

**Files:**
- Modify: `xmake.lua` (`target("module_test")` sources)
- Confirm present: `target("PR_tool_cli")`, `target("test_ILP")`, `target("test_ILP_unit")`, `target("json2txt")`, `target("regression_test")`

- [ ] **Step 1: Replace broad `test/module_test/**.cc` glob**

In `xmake.lua`, inside `target("module_test")`, change source selection from:

```lua
    add_files("test/module_test/**.cc")
```

to:

```lua
    add_deps("PR_tool_cli")
    add_includedirs("source", "source/global", "test/module_test", "test/module_test/test_unit")
    add_files("test/module_test/test_unit/**.cc")
```

Keep the existing `add_files("source/algo/**.cc", ...)` block unchanged. Ensure `add_deps("PR_tool_cli")` exists (fix needs `./PR_tool_cli` for iterative tests).

Do **not** add `test/module_test/test_function/**.cc` (wirelengthstudy has its own `main`).  
Do **not** add any path under `test/module_test/test_writer/` (no `.cc` remains there after Task 2).

- [ ] **Step 2: Confirm algorithm and CLI targets survived the merge**

```bash
rg -n 'target\("PR_tool_cli"\)|target\("test_ILP"\)|target\("test_ILP_unit"\)|target\("json2txt"\)|target\("module_test"\)' xmake.lua
```

Expected: all five names present. If `test_ILP` missing, restore the block from `dev.algo_SAT_MCF_latest:xmake.lua`. If `PR_tool_cli` / `json2txt` missing, restore from pre-merge fix.

- [ ] **Step 3: Build critical targets**

```bash
xmake f --cadical=y
xmake build PR_tool_cli
xmake build module_test
xmake build json2txt
xmake build regression_test
xmake build test_ILP
xmake build test_ILP_unit
```

Expected: all succeed. If duplicate symbol / missing `test_writer_main` / wrong `read_config` arity, fix before continuing.

Quick link check for writer entrypoint:

```bash
nm -C ./output/module_test 2>/dev/null | rg 'test_writer_main' || echo "nm unavailable; rely on './output/module_test writer' usage error path in flow"
```

- [ ] **Step 4: Commit**

```bash
git add xmake.lua
git commit -m "$(cat <<'EOF'
build: compile module_test from test_unit only

Writer, iterative, and unit sources all live under test_unit; test_writer/ keeps cases and scripts.
EOF
)"
```

---

### Task 5: Regression A — fix `[flow]` suite

**Files:**
- Test only: `test/regression_test/flow_test.cc` (no code changes expected)
- Runtime may touch `source/hardware/interposer.hh` temporarily via `CobArrayWidthGuard` (restores after)

- [ ] **Step 1: Run only the `[flow]` scenario**

From repo root:

```bash
xmake build regression_test
./output/regression_test "[flow]"
```

Expected:
- Builds `PR_tool_cli`, `module_test`, `json2txt` as part of the scenario
- `placer_iteratively` on case5 (10 iterations) passes
- `router_iteratively` on case5 (10 iterations) passes
- For each of `test1_neighbouring_chiplet` … `test5_muyan0_spi_uart_jtag`: `run_case.sh` either compares successfully **or** prints kiwi missing WARNING and exits 0 (SKIP). Both are acceptable for this gate if kiwi is absent; if kiwi exists, compare must pass.

- [ ] **Step 2: If flow fails, fix root cause on this branch (do not skip)**

Common failures:
- `module_test` missing `writer` / iterative targets → revisit Task 2 dispatcher / Task 4 sources
- `read_config` structured bindings → Task 2 config/placer patches incomplete
- `register_adder.json` missing under a writer case → Task 3 incomplete
- `PR_tool_cli` not built/dep → Task 4 `add_deps`

- [ ] **Step 3: Commit only if Task 5 required code fixes**

```bash
git add -A
git status
# commit only when there are intentional fixes
git commit -m "$(cat <<'EOF'
fix: restore flow regression after merge layout changes
EOF
)"
```

If no fixes needed, skip commit.

---

### Task 6: Regression B — lightweight `test_ILP` smoke (per `algorithm/test_ILP/AGENTS.md`)

**Files:**
- Read-only guidance: `algorithm/test_ILP/AGENTS.md`
- May temporarily edit `source/hardware/interposer.hh` `COB_ARRAY_WIDTH` for case7 notes; restore afterward if changed

- [ ] **Step 1: Unit binary**

```bash
xmake f --cadical=y
xmake build test_ILP_unit
./output/test_ILP_unit
```

Expected: exit 0.

- [ ] **Step 2: Small integrated case with padding variants**

```bash
xmake build test_ILP
./output/test_ILP algorithm/test_ILP/test/case_2btb -v --max-rss-mb 8192
./output/test_ILP algorithm/test_ILP/test/case_2btb -v --max-rss-mb 8192 -s 0 -d 1
./output/test_ILP algorithm/test_ILP/test/case_2btb -v --max-rss-mb 8192 -s 1 -d 1
```

Expected: all three SAT-success; logs show `routing result:` / `unified SAT:`.

- [ ] **Step 3: Optional case7 smoke (document COB width)**

```bash
rg -n 'COB_ARRAY_WIDTH' source/hardware/interposer.hh
# AGENTS.md notes case7 expects width 13. fix branch defaults to 13 after merge.
./output/test_ILP test/config/case7 -v --max-rss-mb 8192
```

Expected: SAT success. Full ILP optimize / full case matrix are **out of scope** here (user will run on server).

- [ ] **Step 4: Record results in commit message only if you had to fix code; otherwise done**

No new docs required. If algorithm sources broke due to `source/` merge interactions, fix minimally under `algorithm/test_ILP/` or the conflicting `source/` API usage, then:

```bash
git add -A
git commit -m "$(cat <<'EOF'
fix(test_ILP): restore smoke after merging into fix.controlbits
EOF
)"
```

---

### Task 7: Final status checklist (no more feature work)

- [ ] **Step 1: Print branch tip and key path sanity**

```bash
git branch --show-current
# expect fix.controlbits

git log --oneline -8
test ! -e AGENTS.md && echo "root AGENTS deleted OK"
test -f TODO.md && echo "TODO present"
test -f test/config/case5/golden.txt && cat test/config/case5/golden.txt
# expect 1119

test -d algorithm/test_ILP
test -d test/module_test/test_unit
test -d test/module_test/test_writer
test -f test/module_test/test_unit/test_placer_iteratively.cc
test -f test/module_test/test_unit/test_writer.cc
test ! -f test/module_test/test_writer/check-controlbits-file/scripts/test_writer.cc
git ls-files '*reigster_adder.json' | wc -l
# expect 0
```

- [ ] **Step 2: Explicitly leave deferred work untouched**

Do **not** in this plan:
- Rewrite `tools/parse_controlbits.cc` for 4-part outputs
- Fully unify `.gitignore` / `README.md` / `source/AGENTS.md` prose
- Run the full server-side `test_ILP` case matrix / long `--ilp-optimize` jobs

- [ ] **Step 3: Hand back summary to user**

Report: merge commit hash, follow-up commits, flow result, test_ILP smoke result, deferred items list.

---

## Self-review

**Spec coverage:**
- Merge into fix → Task 1
- Prefer `test_unit` + iterative + `test_writer.cc` in `test_unit`; keep `test_writer/` cases/`run_case.sh` → Task 2
- `register_adder.json` repo-wide → Task 3
- xmake compile convention (`test_unit/**.cc` only) → Task 4
- Flow regression → Task 5
- test_ILP AGENTS smoke → Task 6
- TODO=fix, root AGENTS delete, golden=dev, parse_controlbits deferred, docs polish deferred → Tasks 1 & 7

**Placeholder scan:** none intentional; commands and code blocks are concrete.

**Type/API consistency:** `read_config` → `(interposer, basedie, register_map)`; `SAPlaceStrategy` 5-arg; `module_test` targets `writer` / `placer_iteratively` / `router_iteratively` as required by `flow_test` + `run_case.sh` (CLI unchanged when `.cc` moves).
