# Workflow details

Prefer `scripts/run_case.sh`. Manual steps below if debugging a single stage.

## 0. Prepare PR_tool

Build from repo root:

```bash
xmake build module_test
xmake build json2txt   # if needed
```

Relevant modules: `hardware`, `circuit`, `parse` (writer), `global`, `app`.

## 1. Pick a testcase

Use the next numbered dir under `test/module_test/test_writer/` (test1 → test5).

## 2. Golden TXT input

```bash
CASE=test/module_test/test_writer/<testcase>
CHECK=$CASE/check_run
mkdir -p "$CHECK"
./output/json2txt "$CASE" -o "$CHECK" -n connections.txt
```

## 3. Run kiwi

```bash
KIWI=/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/xinzhai_sourcecode1_new/build/kiwi
mkdir -p "$CHECK/kiwi_work/regnamecontrolbit_4part" "$CHECK/golden"
cp "$CHECK/connections.txt" "$CHECK/kiwi_work/"
(cd "$CHECK/kiwi_work" && "$KIWI" -n connections.txt)
cp -R "$CHECK/kiwi_work/regnamecontrolbit_4part" "$CHECK/golden/"
cp "$CHECK/kiwi_work/net_path_info.txt" "$CHECK/"
```

## 4. Convert path coords

```bash
SCRIPTS=test/module_test/test_writer/check-controlbits-file/scripts
python3 "$SCRIPTS/trans_path_old2new.py" \
  "$CHECK/net_path_info.txt" \
  -o "$CHECK/net_path_info_new.txt"
```

## 5. Path replay harness (`test_writer.cc`)

One-time / as-needed: ensure `scripts/test_writer.cc` can parse the path shape
(Bump↔Bump, ExtIO, SyncNet leaves, TracksToBumpsNet / multi-segment nege|pose,
unused path blocks, etc.) and call Writer.

APIs: `source/AGENTS.md`. After changes:

```bash
xmake build module_test
```

Have a separate review pass on harness correctness when the file changes substantially.

## 6. Generate PR four REG files (full, no `-s`)

```bash
cd output
./module_test writer ../$CASE \
  ../$CHECK/net_path_info_new.txt \
  ../$CHECK 0
```

Do **not** pass `-s`. Output:

`$CHECK/regnamecontrolbit_4part/{botleft_REG0,botright_REG1,topleft_REG2,topright_REG3}.txt`

## 7. Compare

```bash
python3 "$SCRIPTS/compare_controlbits.py" \
  --golden-dir "$CHECK/golden/regnamecontrolbit_4part" \
  --split-dir "$CHECK/regnamecontrolbit_4part"
```

Count mismatch → usually means `-s` was used; re-run full.

## 9. Forced stop

If 30 minutes of Writer-logic debugging still does not produce a correct fix:
stop, report status, summarize attempted changes.
