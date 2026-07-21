---
name: check-controlbits-file
description: >-
  Debug and validate PR_tool Writer split register output
  (regnamecontrolbit_4part) against the kiwi/xinzhai golden tool by replaying
  golden net paths. Use when checking controlbits, Writer REG files,
  compare_controlbits, test_writer cases test1–test5, or
  regnamecontrolbit_4part mismatches.
disable-model-invocation: true
---

# check-controlbits-file

Validate **Writer only**: inject golden `net_path_info` into PR_tool hardware,
emit four REG files, compare to kiwi golden. Do **not** validate router/placer.

## Hard constraints

1. Work only on the current git branch; **no** `git add` / `commit` / `push`.
2. **Do not** modify the golden kiwi tool.
3. Golden compare must use **full** Writer output (**no** `-s`).
4. If stuck debugging Writer output logic for **30 minutes** without a correct
   fix: stop, report results and a short summary of changes tried.

## Layout

```text
check-controlbits-file/
├── SKILL.md
├── references/          # detailed docs (read on demand)
└── scripts/
    ├── run_case.sh              # preferred one-shot pipeline
    ├── test_writer.cc           # module_test writer harness
    ├── compare_controlbits.py
    ├── trans_path_old2new.py
    └── verify_simplify_split.py
```

Testcase data stays in sibling dirs: `../test1_*` … `../test5_*`.

## Quick workflow

```
Progress:
- [ ] 0 Prepare / build (`xmake build module_test`, `json2txt`)
- [ ] 1 Pick next case under `test/module_test/test_writer/testN_*`
- [ ] 2–7 Prefer scripts/run_case.sh (json2txt → kiwi → path → writer → compare)
- [ ] 5 If path shape unsupported: extend scripts/test_writer.cc, then re-run
- [ ] 8 If diffs: interpret and iterate Writer (or harness) — see references
- [ ] Optional T1: full + simplified pair — see references/simplify-t1.md
```

From repo root:

```bash
bash test/module_test/test_writer/check-controlbits-file/scripts/run_case.sh \
  test/module_test/test_writer/<testcase>
```

## References

- Tools, paths, cases: [references/tools-and-paths.md](references/tools-and-paths.md)
- Manual step details: [references/workflow-details.md](references/workflow-details.md)
- Diff attribution: [references/diff-interpretation.md](references/diff-interpretation.md)
- Simplify T1: [references/simplify-t1.md](references/simplify-t1.md)

## Success criteria

`compare_controlbits.py` prints `OK: all register values match.` (exit 0).
Then advance to the next numbered testcase until all cases pass.
