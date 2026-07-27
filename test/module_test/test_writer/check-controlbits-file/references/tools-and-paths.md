# Tools and paths

## Golden tool (kiwi / xinzhai)

Predecessor / golden router:

`/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/xinzhai_sourcecode1_new`

- Build: `build/kiwi` (see that repo’s `AGENTS.md` / `README.md`)
- Input: TXT netlist (`connections.txt`)
- Outputs (cwd):
  - `regnamecontrolbit_4part/` — four `*_controlbit.txt` files
  - `net_path_info.txt` — paths in **old** coordinate system

Create `regnamecontrolbit_4part/` **before** running kiwi; otherwise write fails.

## Coordinate systems

Old (kiwi) vs PR_tool coords:

`test/transform_format/txt&json_coord_relationship.txt`

## PR_tool map

See `source/AGENTS.md` (hardware / circuit / parse / writer APIs used by the harness).

## Testcases

Under `test/module_test/test_writer/`:

- `test1_neighbouring_chiplet`
- `test2_chiplet_IO`
- `test3_chiplet_nege`
- `test4_chiplet_pose`
- `test5_muyan0_spi_uart_jtag`

Per-case artifacts (local / often gitignored): `<case>/check_run/`.

## Format conversion

| Tool | Role |
|------|------|
| `test/transform_format/json2txt.cc` → `output/json2txt` | JSON case → TXT for kiwi |
| `test/transform_format/txt2json.cc` | reverse |
| `scripts/trans_path_old2new.py` | kiwi `net_path_info.txt` → PR coords |

Pin names ending with `nege` / `pose` (e.g. `xinzhai_nege`) map like Reader (`ends_with`).

## Register output format

- Case `config.json` → `reigster_adder` points at a register map (same shape as `tools/register_map.json`).
- PR Writer writes:

  `regnamecontrolbit_4part/{botleft_REG0,botright_REG1,topleft_REG2,topright_REG3}.txt`

  Three columns: `hex address reg_name`.

- Golden kiwi files: `botleft_controlbit.txt` etc. (two columns: `hex name`).
- `tools/split_regs.py` is **legacy** only (old single `controlbits_<mode>.txt`).

## Skill scripts

| Script | Role |
|--------|------|
| `scripts/run_case.sh` | Full pipeline for one case |
| `test/module_test/test_unit/test_writer.cc` | Built into `module_test` via `test/module_test/test_unit/**.cc` |
| `scripts/compare_controlbits.py` | Golden vs PR four-file compare |
| `scripts/trans_path_old2new.py` | Path coord convert |
| `scripts/verify_simplify_split.py` | T1 full vs simplified |
