#!/usr/bin/env python3
"""Compare golden regnamecontrolbit_4part files with PR Writer four-file output by register name."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

GOLDEN_TO_SPLIT = {
    "botleft_controlbit.txt": "botleft_REG0.txt",
    "botright_controlbit.txt": "botright_REG1.txt",
    "topleft_controlbit.txt": "topleft_REG2.txt",
    "topright_controlbit.txt": "topright_REG3.txt",
}


def normalize_hex(value: str) -> str:
    value = value.strip().lower()
    if value.startswith("0x"):
        value = value[2:]
    value = value.lstrip("0") or "0"
    return value


def load_golden(path: Path) -> dict[str, str]:
    regs: dict[str, str] = {}
    with path.open(encoding="utf-8") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                print(f"warning: skip invalid golden line {line_num} in {path}: {line}")
                continue
            regs[parts[1]] = normalize_hex(parts[0])
    return regs


def load_split(path: Path) -> dict[str, str]:
    regs: dict[str, str] = {}
    with path.open(encoding="utf-8") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                print(f"warning: skip invalid split line {line_num} in {path}: {line}")
                continue
            regs[parts[2]] = normalize_hex(parts[0])
    return regs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare golden and PR Writer regnamecontrolbit_4part files."
    )
    parser.add_argument(
        "--golden-dir",
        required=True,
        type=Path,
        help="Directory with golden botleft/botright/topleft/topright_controlbit.txt files",
    )
    parser.add_argument(
        "--split-dir",
        required=True,
        type=Path,
        help="PR Writer output regnamecontrolbit_4part/ (four *_REG*.txt files)",
    )
    args = parser.parse_args()

    total_diff = 0

    for golden_name, split_name in GOLDEN_TO_SPLIT.items():
        golden_path = args.golden_dir / golden_name
        split_path = args.split_dir / split_name
        if not golden_path.is_file():
            print(f"error: missing golden file {golden_path}")
            return 1
        if not split_path.is_file():
            print(f"error: missing split file {split_path}")
            return 1

        golden_regs = load_golden(golden_path)
        split_regs = load_split(split_path)

        if len(split_regs) != len(golden_regs):
            print(
                f"ERROR: register count mismatch in {split_name} "
                f"({len(split_regs)} vs golden {len(golden_regs)})."
            )
            print("Hint: golden compare requires full output; disable -s/--simplify-controlbits-file.")
            return 1

        all_names = sorted(set(golden_regs) | set(split_regs))
        file_diff = 0
        for name in all_names:
            g = golden_regs.get(name)
            s = split_regs.get(name)
            if g is None:
                print(f"[MISSING_IN_GOLDEN] {split_name} {name}: split={s}")
                file_diff += 1
            elif s is None:
                print(f"[MISSING_IN_SPLIT] {golden_name} {name}: golden={g}")
                file_diff += 1
            elif g != s:
                print(f"[DIFF] {name}: golden={g} split={s}")
                file_diff += 1

        print(f"{golden_name} vs {split_name}: {file_diff} difference(s)")
        total_diff += file_diff

    if total_diff == 0:
        print("OK: all register values match.")
        return 0
    print(f"FAIL: {total_diff} total difference(s).")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
