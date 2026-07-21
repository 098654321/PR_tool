# T1: full + simplified pair check

Optional. Does **not** replace golden compare (which must stay full / no `-s`).

`module_test writer` can emit a full tree and a simplified tree in one run:

```bash
CASE=test/module_test/test_writer/<testcase>
CHECK=$CASE/check_run
SCRIPTS=test/module_test/test_writer/check-controlbits-file/scripts
PATH_NEW=$CHECK/net_path_info_new.txt

cd output
./module_test writer ../$CASE "$PATH_NEW" ../$CHECK/full 0 \
  --simplified-output-dir ../$CHECK/simplified

python3 ../$SCRIPTS/verify_simplify_split.py \
  --full-dir ../$CHECK/full/regnamecontrolbit_4part \
  --simplified-dir ../$CHECK/simplified/regnamecontrolbit_4part
```

`verify_simplify_split.py` uses the same default-hex omit rules as
`register_defaults.hh` / `tools/split_regs.py` (strict literal equality):

- Rows that should be omitted from full must not appear in simplified.
- Non-default rows must keep the same hex.
- Simplified must not invent register names absent from full.

Expect exit code 0.
