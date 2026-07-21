#!/usr/bin/env bash
# Run one test_writer case: json2txt → kiwi → path convert → writer → compare.
# Usage (from repo root):
#   bash test/module_test/test_writer/check-controlbits-file/scripts/run_case.sh \
#     test/module_test/test_writer/<testcase>
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SKILL_DIR/../../../.." && pwd)"
cd "$REPO_ROOT"

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <testcase_dir> [mode]" >&2
  echo "  example: $0 test/module_test/test_writer/test1_neighbouring_chiplet" >&2
  exit 2
fi

CASE_ARG="$1"
MODE="${2:-0}"

# Resolve case path relative to repo root
if [[ "$CASE_ARG" = /* ]]; then
  CASE="$CASE_ARG"
else
  CASE="$REPO_ROOT/$CASE_ARG"
fi
if [[ ! -d "$CASE" ]]; then
  echo "error: testcase dir not found: $CASE" >&2
  exit 1
fi
CASE_REL="${CASE#"$REPO_ROOT"/}"

CHECK="$CASE/check_run"
KIWI="${KIWI:-/Users/jiaheng/FDU_files/Tao_group/PR_tool/jsy_version_tool/xinzhai_sourcecode1_new/build/kiwi}"
JSON2TXT="${JSON2TXT:-$REPO_ROOT/output/json2txt}"
MODULE_TEST="${MODULE_TEST:-$REPO_ROOT/output/module_test}"

if [[ ! -x "$KIWI" ]]; then
  echo "error: kiwi not found/executable: $KIWI (override with KIWI=...)" >&2
  exit 1
fi

echo "==> [1/6] build module_test (+ json2txt if missing)"
xmake build module_test
if [[ ! -x "$JSON2TXT" ]]; then
  xmake build json2txt
fi

echo "==> [2/6] json2txt → $CHECK/connections.txt"
mkdir -p "$CHECK"
"$JSON2TXT" "$CASE" -o "$CHECK" -n connections.txt

echo "==> [3/6] kiwi golden"
mkdir -p "$CHECK/kiwi_work/regnamecontrolbit_4part" "$CHECK/golden"
cp "$CHECK/connections.txt" "$CHECK/kiwi_work/"
(cd "$CHECK/kiwi_work" && "$KIWI" -n connections.txt)
if [[ ! -f "$CHECK/kiwi_work/net_path_info.txt" ]]; then
  echo "error: kiwi did not produce net_path_info.txt" >&2
  exit 1
fi
rm -rf "$CHECK/golden/regnamecontrolbit_4part"
cp -R "$CHECK/kiwi_work/regnamecontrolbit_4part" "$CHECK/golden/"
cp "$CHECK/kiwi_work/net_path_info.txt" "$CHECK/"

echo "==> [4/6] path convert"
python3 "$SCRIPT_DIR/trans_path_old2new.py" \
  "$CHECK/net_path_info.txt" \
  -o "$CHECK/net_path_info_new.txt"

echo "==> [5/6] module_test writer (full, no -s)"
if [[ ! -x "$MODULE_TEST" ]]; then
  echo "error: module_test not found: $MODULE_TEST" >&2
  exit 1
fi
"$MODULE_TEST" writer "$CASE" \
  "$CHECK/net_path_info_new.txt" \
  "$CHECK" "$MODE"

echo "==> [6/6] compare_controlbits"
python3 "$SCRIPT_DIR/compare_controlbits.py" \
  --golden-dir "$CHECK/golden/regnamecontrolbit_4part" \
  --split-dir "$CHECK/regnamecontrolbit_4part"

echo "OK: $CASE_REL passed controlbits compare"
