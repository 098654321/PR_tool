# Split Register Output Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make PR_tool write only `{output}/regnamecontrolbit_4part/{botleft_REG0,botright_REG1,topleft_REG2,topright_REG3}.txt` (3 columns), with `-s` omitting default-valued lines; stop emitting `controlbits_<mode>.txt`.

**Architecture:** Load case `reigster_adder` JSON into `RegisterMapConfig`. `Writer` fetches once into `HashMap<reg_name, hex>`, then `write_split_files` emits four files. CLI writes one tree; `test_writer` may pair-write full+simplified. Legacy `split_regs.py` stays for offline old single-file inputs only.

**Tech Stack:** C++23 (xmake), project `serde` JSON, Python3 for compare/verify scripts.

**Spec:** `docs/superpowers/specs/2026-07-21-split-register-output-design.md`

**Git policy:** Do **not** run `git add` or `git commit` in any task. Leave changes unstaged/uncommitted for the human to review and commit.

---

## File map

| File | Action | Responsibility |
|------|--------|----------------|
| `source/parse/reader/config/config.hh` | Modify | Add `RegisterMapConfig` typedefs + `Config::register_map` |
| `source/parse/reader/config/config.cc` | Modify | Optional `reigster_adder` path; `load_register_map_config`; validate |
| `source/parse/reader/module.hh` / `module.cc` | Modify | `read_config` returns/out-params map |
| `source/parse/writer/writer.hh` / `writer.cc` | Modify | Collect values; `write_split_files`; drop single-file write |
| `source/parse/writer/module.hh` / `module.cc` | Modify | Pass map; pair API writes two trees |
| `source/app/cli/cli.cc` | Modify | Thread map into output |
| `source/widget/window.cc` | Modify | Compile fix: store map; TODO on save-as UX |
| `test/**` callers of `read_config` / `output_from_routing_results` | Modify | Destructure third value / pass map |
| `test/config/**/reigster_adder.json` (and `register_adder.json`) | Modify | Copy full map from `tools/register_map.json` |
| `test/module_test/test_writer/**/reigster_adder.json` | Modify | Same copy where writer cases write |
| `test/module_test/test_writer/check-controlbits-file/scripts/compare_controlbits.py` | Modify | Count check before hex compare |
| `test/module_test/test_writer/check-controlbits-file/scripts/verify_simplify_split.py` | Create | T1: full vs simplified four-file check |
| `test/module_test/test_writer/check-controlbits-file/SKILL.md` | Modify | Remove split bridge; direct four-file compare |
| `test/module_test/test_writer/check-controlbits-file/scripts/test_writer.cc` | Modify | Pass register_map; pair dirs under `regnamecontrolbit_4part/` |
| `source/parse/reader/controlbits/controlbits.cc` (+ related) | Modify | TODO comments only |
| `tools/split_regs.py` docstring / `tools/AGENTS.md` | Modify | Legacy offline positioning (A1) |
| `source/AGENTS.md`, `test/AGENTS.md`, `README.md`, `TODO.md` | Modify | Align with spec |
| `tools/register_map.json` | Reference | Template to copy into cases |

**Constants (use everywhere):**

```text
k_split_subdir = "regnamecontrolbit_4part"
k_split_files  = botleft_REG0.txt, botright_REG1.txt, topleft_REG2.txt, topright_REG3.txt
line format    = "{hex} {address} {reg_name}\n"
```

---

### Task 1: Register map types + loader + case data

**Files:**
- Modify: `source/parse/reader/config/config.hh`
- Modify: `source/parse/reader/config/config.cc`
- Modify: every case map file pointed at by `config.json` (`reigster_adder.json` or `register_adder.json`) under `test/config/` and `test/module_test/test_writer/`

- [ ] **Step 1: Add types to `config.hh`**

```cpp
using RegisterAddress = std::String;
using RegisterFileMap = std::HashMap<std::String, RegisterAddress>;      // reg_name -> address
using RegisterMapConfig = std::HashMap<std::String, RegisterFileMap>;     // filename -> regs

struct Config {
    // ... existing fields ...
    RegisterMapConfig register_map;
};
```

- [ ] **Step 2: Extend `ConfigFilepaths` and deserialize in `config.cc`**

```cpp
struct ConfigFilepaths {
    // existing required fields ...
    std::Option<std::FilePath> reigster_adder;  // optional
};

DESERIALIZE_STRUCT(PR_tool::parse::ConfigFilepaths,
    DE_FILED(interposer)
    DE_FILED(topdies)
    DE_FILED(topdie_insts)
    DE_FILED(external_ports)
    DE_FILED(connections)
    DE_FILED(ports_01)
    DE_OPTION_FILED(reigster_adder)
)
```

Implement `load_register_map_config(path, RegisterMapConfig&)` via `serde::deserialize(Json::load_from(path), map)`.

After other loads in `load_config`, if `reigster_adder` has value:

```cpp
load_register_map_config(config_folder / *config_paths.reigster_adder, config.register_map);
```

Validate (fatal on failure):

- Keys include exactly the four filenames above (or at least all four present).
- No duplicate `reg_name` across files.

Empty map after load is allowed until write time (write path fatals); prefer fatal in loader if file exists but parses to `{}` when used for writer cases — **write path is the hard gate** (see Task 3).

- [ ] **Step 3: Populate case map JSON files**

```bash
# For each config.json that sets reigster_adder / register_adder path, copy the template:
# Example for case1:
cp tools/register_map.json test/config/case1/reigster_adder.json
```

Discover targets:

```bash
python3 - <<'PY'
import json, pathlib
root = pathlib.Path("test")
for cfg in root.rglob("config.json"):
    data = json.loads(cfg.read_text())
    key = "reigster_adder" if "reigster_adder" in data else ("register_adder" if "register_adder" in data else None)
    if not key:
        print("NO_MAP_KEY", cfg)
        continue
    target = cfg.parent / data[key]
    print(target)
PY
```

Overwrite each listed target with `tools/register_map.json` contents.

- [ ] **Step 4: Smoke-load one case (optional small C++ or python size check)**

```bash
python3 -c "import json; d=json.load(open('test/config/case1/reigster_adder.json')); assert len(d)==4; assert sum(len(v) for v in d.values())==5216"
```

Expected: no assertion error.

- [ ] **Step 5: Stop — do not `git add` or `git commit`**

Leave Task 1 changes in the working tree for human review.

---

### Task 2: Thread `RegisterMapConfig` through `read_config` and call sites

**Files:**
- Modify: `source/parse/reader/module.hh`
- Modify: `source/parse/reader/module.cc`
- Modify: `source/app/cli/cli.cc`
- Modify: `source/widget/window.cc` (and `window.hh` if a member is needed)
- Modify: all `read_config` / `output_from_routing_results` call sites under `test/` (see grep list in File map)

- [ ] **Step 1: Change API signatures**

`module.hh`:

```cpp
auto read_config(const std::FilePath& config_folder, int mode, bool try_all_modes)
    -> std::Tuple<std::Box<hardware::Interposer>, std::Box<circuit::BaseDie>, RegisterMapConfig>;

auto read_config(
    const std::FilePath& config_folder,
    hardware::Interposer* interposer,
    circuit::BaseDie* basedie,
    int mode,
    bool try_all_modes
) -> RegisterMapConfig;
```

Include `config/config.hh` (or forward-declare typedefs) so `RegisterMapConfig` is visible.

`module.cc` returning overload:

```cpp
auto map = read_config(config_folder, interposer.get(), basedie.get(), mode, try_all_modes);
return {std::move(interposer), std::move(basedie), std::move(map)};
```

In-place overload: after `load_config` + `reader.build()`, `return config.register_map;` (move).

- [ ] **Step 2: Fix compile at every call site**

Pattern for callers that also write:

```cpp
auto [interposer, basedie, register_map] = parse::read_config(config_path, mode, try_all_modes);
// ...
parse::output_from_routing_results(interposer.get(), output_file, basedie.get(), mode, try_all_modes, simplify_controlbits, register_map);
```

Pattern for callers that only need hardware (placer tests, etc.):

```cpp
auto [interposer, basedie, register_map] = parse::read_config(...);
(void)register_map;  // unused until write
```

GUI: store `RegisterMapConfig _register_map` on `Window`; assign from in-place `read_config` return; pass into `write_control_bits`. Add TODO comment near save-as that directory UX is deferred (spec §5 out-of-scope).

**Note:** Task 2 may not compile until Task 3 updates `output_from_routing_results` / `write_control_bits` signatures. Prefer implementing Task 2+3 together in the working tree if needed, or add temporary unused parameters in Task 2 then implement bodies in Task 3. Still **do not** commit.

- [ ] **Step 3: Build**

```bash
xmake build PR_tool module_test regression_test
```

Expected: compile succeeds after Task 3 wired (or interim stubs).

- [ ] **Step 4: Stop — do not `git add` or `git commit`**

Leave Task 2 changes in the working tree for human review.

---

### Task 3: Writer collect + `write_split_files` + module API

**Files:**
- Modify: `source/parse/writer/writer.hh`
- Modify: `source/parse/writer/writer.cc`
- Modify: `source/parse/writer/module.hh`
- Modify: `source/parse/writer/module.cc`

- [ ] **Step 1: Redesign `Writer` public API**

```cpp
class Writer {
public:
    Writer(hardware::Interposer* pinterposer);

    // Fetch once into _values, then write one tree.
    auto fetch_and_write_split(
        const RegisterMapConfig& register_map,
        const std::FilePath& output_root,
        bool simplify
    ) -> void;

    // Fetch once; write full then simplified trees (test-only).
    auto fetch_and_write_split_pair(
        const RegisterMapConfig& register_map,
        const std::FilePath& full_output_root,
        const std::FilePath& simplified_output_root
    ) -> void;

private:
    auto fetch() -> void;
    auto build_regs() -> void;
    auto collect_values() -> void;   // was write(ofstream): fill _values
    auto write_split_files(const RegisterMapConfig&, const std::FilePath& output_root, bool simplify) -> void;
    // keep write_*_template helpers but write into _values instead of ofstream
    auto store_line(const std::String& hex, const std::String& name) -> void;

    RegisterValue _rv;
    std::Vector<BaseRegister*> _regs;
    hardware::Interposer* _pinterposer;
    std::HashMap<std::String, std::String> _values;  // name -> hex
};
```

Remove `_simplify` from constructor; pass `simplify` only into `write_split_files`.

`store_line`: `_values[name] = hex;` (last write wins if duplicate — should not happen).

Change each `maybe_write_line(file, hex, name)` call site inside templates to `store_line(hex, name)`. Remove `ofstream&` parameters from `write_cob` / `write_tob` / templates if they only stream hex lines — signatures become void collectors.

- [ ] **Step 2: Implement `write_split_files`**

```cpp
auto Writer::write_split_files(
    const RegisterMapConfig& register_map,
    const std::FilePath& output_root,
    bool simplify
) -> void {
    if (register_map.empty()) {
        debug::fatal("register_map is empty; configure reigster_adder in config.json");
    }
    auto out_dir = output_root / "regnamecontrolbit_4part";
    std::filesystem::create_directories(out_dir);

    std::UnorderedSet<std::String> used;
    std::usize omitted = 0;
    std::usize written = 0;

    for (const auto& [filename, regs] : register_map) {
        auto path = out_dir / filename;
        std::ofstream out(path);
        if (!out.is_open()) {
            throw std::runtime_error(std::format("cannot open {}", path.string()));
        }
        for (const auto& [reg_name, address] : regs) {
            auto it = _values.find(reg_name);
            if (it == _values.end()) {
                debug::warning_fmt("register '{}' in map file '{}' missing from fetch", reg_name, filename);
                continue;
            }
            used.insert(reg_name);
            const auto& hex = it->second;
            if (simplify && should_omit_simplified_line(hex, reg_name)) {
                ++omitted;
                continue;
            }
            out << hex << " " << address << " " << reg_name << "\n";
            ++written;
        }
    }
    for (const auto& [name, _] : _values) {
        if (!used.contains(name)) {
            debug::warning_fmt("register '{}' fetched but not present in register_map", name);
        }
    }
    if (simplify) {
        debug::info_fmt("split write: wrote {} line(s), omitted {} default line(s)", written, omitted);
    }
}
```

Use the project’s set/map typedefs if `std::UnorderedSet` is uncommon (`std::Set` / existing collection headers).

`fetch_and_write_split`: `build_regs(); fetch(); collect_values(); write_split_files(...)`.

`fetch_and_write_split_pair`: fetch+collect once; `write_split_files(..., false)` then `write_split_files(..., true)`.

- [ ] **Step 3: Update `module.hh` / `module.cc`**

```cpp
auto output_from_routing_results(
    hardware::Interposer* interposer,
    const std::FilePath& output_path,
    circuit::BaseDie* basedie,
    int mode,
    bool try_all_modes,
    bool simplify_controlbits,
    const RegisterMapConfig& register_map
) -> void;

auto write_control_bits(
    hardware::Interposer* interposer,
    const std::FilePath& output_path,
    int mode,
    bool simplify_controlbits,
    const RegisterMapConfig& register_map
) -> void;

auto write_control_bits_pair(
    hardware::Interposer* interposer,
    const std::FilePath& full_output_path,
    const std::FilePath& simplified_output_path,
    int mode,
    const RegisterMapConfig& register_map
) -> void;
```

`write_control_bits` body:

```cpp
debug::info_fmt("Write split control bits under '{}'", (output_path / "regnamecontrolbit_4part").string());
// TODO(split-output): try_all_modes needs mode_<m>/regnamecontrolbit_4part/ to avoid overwrite
Writer{interposer}.fetch_and_write_split(register_map, output_path, simplify_controlbits);
```

Remove creation of `controlbits_<mode>.txt`.

- [ ] **Step 4: Build and quick run**

```bash
xmake build PR_tool module_test
cd output && ./module_test writer ../test/module_test/test_writer/test2_chiplet_IO \
  ../test/module_test/test_writer/test2_chiplet_IO/net_path_info_new.txt \
  ./writer_out_test2 0
ls writer_out_test2/regnamecontrolbit_4part/
```

Expected: four `*_REG*.txt` files; **no** `controlbits_0.txt`.

Spot-check one line shape:

```bash
head -n 1 writer_out_test2/regnamecontrolbit_4part/botleft_REG0.txt
```

Expected: three whitespace-separated fields (`hex`, address like `32'h...`, `reg_name`).

- [ ] **Step 5: Stop — do not `git add` or `git commit`**

Leave Task 3 changes in the working tree for human review.

---

### Task 4: `compare_controlbits.py` count check + SKILL + `test_writer` pair dirs

**Files:**
- Modify: `test/module_test/test_writer/check-controlbits-file/scripts/compare_controlbits.py`
- Modify: `test/module_test/test_writer/check-controlbits-file/SKILL.md`
- Modify: `test/module_test/test_writer/check-controlbits-file/scripts/test_writer.cc`
- Create: `test/module_test/test_writer/check-controlbits-file/scripts/verify_simplify_split.py`

- [ ] **Step 1: Update `compare_controlbits.py`**

After loading `golden_regs` and `split_regs` for each pair, **before** the per-name loop:

```python
if len(split_regs) != len(golden_regs):
    print(
        f"ERROR: register count mismatch in {split_name} "
        f"({len(split_regs)} vs golden {len(golden_regs)})."
    )
    print("Hint: golden compare requires full output; disable -s/--simplify-controlbits-file.")
    return 1
```

Keep name-aligned hex compare; ignore address column (already via `load_split`).

Rename CLI help text: `--split-dir` means PR `regnamecontrolbit_4part/` (not post-split_regs).

- [ ] **Step 2: Rewrite SKILL workflow steps 6–8**

Replace “generate `controlbits_0.txt` → `split_regs.py` → compare” with:

1. Run `test_writer` **without** `-s` into `check_run/` (or similar).
2. Compare `check_run/regnamecontrolbit_4part/` vs golden `regnamecontrolbit_4part/` using `compare_controlbits.py`.
3. Delete steps that invoke `split_regs.py` for the golden path.

Document T1 separately: `--simplified-output-dir` + `verify_simplify_split.py`.

- [ ] **Step 3: Keep `test_writer` pair API**

When `--simplified-output-dir` is set, after connect:

```cpp
parse::write_control_bits_pair(interposer, output_dir, *simplified_output_dir, mode, register_map);
```

Ensure `read_config` provides `register_map`. Single-tree path uses `output_from_routing_results(..., register_map)`.

- [ ] **Step 4: Add `verify_simplify_split.py`**

Script inputs: `--full-dir` and `--simplified-dir` (each contains the four REG files, either directly or under `regnamecontrolbit_4part/` — accept either by checking for subdir).

Logic (mirror `register_defaults.hh` / `split_regs.py`):

```python
# For every line in full files:
#   if should_omit_simplified_line(hex, name): name must NOT appear in simplified
#   else: simplified must have same hex for name
# Simplified must not contain names absent from full
```

Exit 0 on success.

- [ ] **Step 5: Run golden compare on test2 (no `-s`)**

```bash
xmake build module_test
cd output
./module_test writer ../test/module_test/test_writer/test2_chiplet_IO \
  <path_to_net_path_info_new.txt> ./check_run 0
python3 ../test/module_test/test_writer/check-controlbits-file/scripts/compare_controlbits.py \
  --golden-dir ../test/module_test/test_writer/test2_chiplet_IO/golden/regnamecontrolbit_4part \
  --split-dir ./check_run/regnamecontrolbit_4part
```

Expected: `OK: all register values match.` **or** only documented mux-fill differences (same policy as old SKILL step 8). If COB/dly/drv differ unexpectedly, stop and fix Writer.

Adjust golden path if the repo layout differs; discover with `find test/module_test/test_writer/test2* -type d -name regnamecontrolbit_4part`.

- [ ] **Step 6: Run T1 simplify verify**

```bash
./module_test writer ... ./check_run_full 0 --simplified-output-dir ./check_run_simplified
python3 ../test/module_test/test_writer/check-controlbits-file/scripts/verify_simplify_split.py \
  --full-dir ./check_run_full/regnamecontrolbit_4part \
  --simplified-dir ./check_run_simplified/regnamecontrolbit_4part
```

Expected: exit 0.

- [ ] **Step 7: Stop — do not `git add` or `git commit`**

Leave Task 4 changes in the working tree for human review.

---

### Task 5: TODO comments on broken readback / compare / tools docs

**Files:**
- Modify: `source/parse/reader/controlbits/controlbits.cc` (near `load_controlbits`)
- Modify: `source/parse/reader/module.cc` (`read_controlbits`)
- Modify: `source/app/cli/cli.cc` (`parse::compare` / `-c` path)
- Modify: `source/parse/comparator/controlbits_parser.cc` (file header or parse entry)
- Modify: `source/widget/window.cc` (`generateControlBitAs`)
- Modify: `tools/parse_controlbits.cc`
- Modify: `tools/split_regs.py` module docstring
- Modify: `tools/AGENTS.md`
- Modify: `source/AGENTS.md` (§2, §4.2–4.3, §9.4)
- Modify: `test/AGENTS.md`
- Modify: `README.md` (I/O diagram / table)
- Modify: `TODO.md` (align with spec; remove obsolete primary workflow)

- [ ] **Step 1: Add identical-style TODO comments**

```cpp
// TODO(split-output): still assumes controlbits_<mode>.txt; formal output is now
// regnamecontrolbit_4part/; readback / compare not updated yet.
```

- [ ] **Step 2: Update docs per design §7**

- `tools/AGENTS.md`: demote `split_regs` to legacy single-file offline tool; formal Writer writes four files.
- `source/AGENTS.md`: product path is four files; `-s` on Writer; recommend PR full or PR `-s` (not split `-s` as primary).
- `test/AGENTS.md`: one short note on writer golden four-file flow.
- `README.md` + `TODO.md`: sync.

- [ ] **Step 3: Stop — do not `git add` or `git commit`**

Leave Task 5 changes in the working tree for human review.

---

### Task 6: Smoke — case1 iterative placer/router

**Files:** none (run only)

- [ ] **Step 1: Ensure case1 map is populated** (Task 1)

- [ ] **Step 2: Build and run**

```bash
xmake build PR_tool module_test
cd output
./module_test placer_iteratively ../test/config/case1
./module_test router_iteratively ../test/config/case1
```

Expected: no fatal about empty `register_map`; runs complete under normal project criteria (routing failure rules unchanged). Confirm `./regnamecontrolbit_4part/` exists after a normal `./PR_tool ../test/config/case1 -o .` smoke if iterative tests do not leave artifacts.

- [ ] **Step 3: Stop — do not `git add` or `git commit`**

If smoke requires tiny docs/script tweaks, leave them uncommitted with the rest of the working tree.

---

## Spec coverage checklist

| Spec item | Task |
|-----------|------|
| Four-file output, no `controlbits_<mode>.txt` | 3 |
| Load `reigster_adder` / case maps | 1 |
| `read_config` returns map | 2 |
| `-s` omit via `register_defaults` | 3 |
| Missing/extra → `debug::warning`; no `report.log` | 3 |
| CLI one tree (C1); pair write test-only (T1) | 3–4 |
| Golden compare + count check; SKILL without split | 4 |
| `verify_simplify_split.py` | 4 |
| Deferred readback/compare/GUI UX TODOs | 5 |
| Docs A1 for `split_regs` | 5 |
| case1 iterative smoke | 6 |
| Multi-mode subdirectory | comment only in Task 3 |

---

## Self-review notes

- No TBD placeholders in steps.
- Types named consistently: `RegisterMapConfig`, `write_split_files`, `fetch_and_write_split_pair`.
- Task 2+3 may need to land together in the working tree if intermediate compile breaks — called out explicitly; still no git commit.
- Golden path for test2 must be discovered on disk; plan does not invent a missing golden tree.
- No task step runs `git add` / `git commit`.
