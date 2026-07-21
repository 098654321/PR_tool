# Design: PR_tool 直接写出 4 个 split 寄存器文件

Date: 2026-07-21  
Status: pending user review of this spec

## 1. Goal

Replace the single-file controlbits product (`controlbits_<mode>.txt`, two columns `hex reg_name`) with **four register files** under `{output}/regnamecontrolbit_4part/`, matching the current `tools/split_regs.py` layout (three columns `hex address reg_name`).

PR_tool must **not** emit `controlbits_<mode>.txt` as a formal output.

## 2. Decisions (locked)

| Topic | Choice |
|-------|--------|
| Output shape | Scheme C: only the four REG files |
| `-s` primary entry | PR_tool `-s` writes sparse four-file output directly |
| `split_regs.py` | Keep (including `-s`) as **offline tool for legacy/external** single-file `controlbits_*.txt` only (A1) |
| `-s` verification | One fetch → two directory trees (full + simplified); script checks omit/keep rules (T1) |
| CLI dual write | No: CLI writes one set only; pair write is test-only (C1) |
| Writer algorithm | Collect `reg_name → hex` in memory, then write by `register_map` (Approach 1) |
| Config plumbing | `read_config` returns `RegisterMapConfig` (scheme C); map is write-only |
| Missing/extra regs | Both **WARNING** via `debug::warning` → `debug.log` |
| `report.log` | Not used |

## 3. Architecture / data flow

```text
config.json
  └─ reigster_adder → *.json   # isomorphic to tools/register_map.json

read_config(config_folder)
  → (interposer, basedie, register_map)

P&R …
  → PathPackage::connect_all
  → Writer::fetch → HashMap<reg_name, hex>   # fetch once

write_split_files(register_map, values, output_root, simplify)
  → {output_root}/regnamecontrolbit_4part/
       botleft_REG0.txt
       botright_REG1.txt
       topleft_REG2.txt
       topright_REG3.txt
  → each line: hex address reg_name
  → if simplify: omit line when hex == default_hex_for(name)
```

### Default omission rules (`register_defaults.hh`)

| Register pattern | Default hex | Omit when value equals default |
|------------------|-------------|-------------------------------|
| `tob_{r}_{c}_track2tob_{0..3}` (r,c,index ∈ 0..3) | `ffffffff` | yes |
| `tob_{r}_{c}_tob2bump_bank{0,1}_en_{0,1}` | `ffffffff` | yes |
| All other registers | `00000000` | yes |

Hex comparison is **strict literal string equality** (same as current C++ / `split_regs.py`).

### CLI vs test pair write

- CLI: without `-s` → full four files; with `-s` → sparse four files (one tree under `-o`).
- `test_writer` (verification only): fetch once, call `write_split_files` twice into `full/` and `simplified/` roots.

## 4. Config loading and call sites

### Load

- Add optional `reigster_adder` to `ConfigFilepaths` (keep project spelling).
- `load_register_map_config` → `RegisterMapConfig`: `filename → {reg_name → address}`.
- At end of `load_config()`: load if path present.
- Before write: empty/missing map → **fatal** with hint to configure `reigster_adder`.
- Recommended validation (fatal on failure): four expected filenames present; `reg_name` unique across files.
- Populate each relevant case’s map file pointed to by `config.json` (`reigster_adder.json` or `register_adder.json` — do not force rename). Source template: `tools/register_map.json`.

### Pass through

- Returning `read_config` → `Tuple<Interposer, BaseDie, RegisterMapConfig>`.
- In-place GUI overload: out-param or return map, then pass to output entry.
- `output_from_routing_results` / `write_control_bits` take `const RegisterMapConfig&`.
- Update call sites: `cli.cc`, `test_writer`, `regression_test`, GUI paths.
- Do **not** attach map to routing/`Reader::build()`.

## 5. Writer / CLI / errors

### Writer

1. Keep fetch semantics; collect all `reg_name → hex` in memory (equivalent to current `write_*_template` content).
2. Add `write_split_files(...)` as above; stop writing `controlbits_<mode>.txt`.
3. `mode` is not part of the four filenames in this delivery (see multi-mode below).
4. Test API: one fetch → two `write_split_files` calls (simplify false/true).

### CLI

- Keep `-s/--simplify-controlbits-file` for the four-file write path.
- `-o/--output` is the output root; create `regnamecontrolbit_4part/` under it.
- Update `print_help()`.

### Errors / logging

| Case | Behavior |
|------|----------|
| Map missing or empty | `fatal` |
| Map has name, fetch lacks it | `debug::warning` |
| Fetch has name, map lacks it | `debug::warning` (same severity) |
| Optional omit summary under `-s` | `debug::info` aggregate only |
| `report.log` | **not** written; everything goes to existing `debug.log` |

### Out of scope this delivery (TODO comments only)

- Multi-mode: comment that future layout should be `mode_<m>/regnamecontrolbit_4part/` to avoid overwrite.
- `load_controlbits` / `read_controlbits` (skip-route / incremental warm-start).
- `-c/--compare` and `controlbits_parser` (still assume single two-column file).
- GUI `generateControlBitAs` single-file save.
- `tools/parse_controlbits.cc` input convention.

## 6. Testing

| Item | Method |
|------|--------|
| Golden (primary correctness) | `test_writer` **without** `-s`; compare PR `regnamecontrolbit_4part/` to golden; **remove** `split_regs` bridge from SKILLs |
| `compare_controlbits.py` | Align by register name; ignore address on PR 3-column lines; golden remains 2-column; **count check first**; on mismatch hint to disable `-s` |
| `-s` (T1) | Pair directories from one fetch; script validates omit/keep vs `register_defaults` |
| Smoke | `placer_iteratively` / `router_iteratively` on case1 (content of controlbits not asserted) |
| Explicitly not this delivery | Readback round-trip, `-c` compare fix, GUI export fix |

Test data: fill complete register maps for integration cases and `test_writer` cases that go through `read_config` + write.

## 7. Documentation sync

| File | Update |
|------|--------|
| `README.md` | Product is four files; I/O table; `-s` / `-o` semantics |
| `source/AGENTS.md` | §2 / §4.3 / §9.4: write path and `-s`; recommended workflow is PR full or `-s` four-file output |
| `tools/AGENTS.md` | `split_regs` demoted to legacy single-file offline tool (A1) |
| `test/AGENTS.md` | Writer golden uses four files (no split bridge); iterative smoke does not depend on single-file path |
| `TODO.md` | Align with this design (remove obsolete `-s` wording and “full controlbits → split `-s`” as primary path) |

### `split_regs.py` (A1)

- Keep script and `-s`; rules stay mirrored with `register_defaults.hh`.
- Purpose: legacy/external `controlbits_*.txt` → four files.
- **Not** the formal PR_tool output or recommended `-s` entry point.

## 8. Suggested implementation order

1. Config types + `load_register_map_config`; copy full maps into cases.
2. `read_config` returns map; update call sites; thread into output APIs.
3. Writer: in-memory collect + `write_split_files`; remove single-file write; test pair API.
4. `compare_controlbits.py` count check; rewrite SKILLs (no split bridge).
5. `-s` verify script / `test_writer` pair dirs; run golden test2 without `-s`.
6. TODO comments on broken readback/compare/GUI/`parse_controlbits`.
7. Docs: README + three AGENTS + TODO.md alignment.
8. Smoke: case1 placer/router iteratively.

## 9. Risks / constraints

- Breaking change for any script that expects `controlbits_0.txt`.
- Golden compare requires full (non-`-s`) output; count check is mandatory.
- Empty `reigster_adder` must fatal on write.
- C++ churn likely >100 lines: follow `source/AGENTS.md` §0 (`.plan/` record + review) if applicable.

## 10. Non-goals

- Implementing controlbits readback from four files.
- Multi-mode subdirectory layout.
- Removing `split_regs.py` from the repo.
- Changing golden file format to include address columns.
