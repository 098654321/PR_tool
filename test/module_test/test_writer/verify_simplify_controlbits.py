#!/usr/bin/env python3
"""Verify simplified controlbits output against the full output."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

DEFAULT_HEX = "00000000"
TOB_DLY_DRV_RE = re.compile(r"^tob_\d+_\d+_(dly|drv)_\d+$")


def normalize_hex(value: str) -> str:
    value = value.strip().lower()
    if value.startswith("0x"):
        value = value[2:]
    return value.lstrip("0") or "0"


def is_zero_hex(value: str) -> bool:
    return normalize_hex(value) == "0"


def is_omissible(name: str, hex_value: str) -> bool:
    if not is_zero_hex(hex_value):
        return False
    if name.startswith("cob_"):
        return True
    return TOB_DLY_DRV_RE.match(name) is not None


def load_controlbits(path: Path) -> dict[str, str]:
    regs: dict[str, str] = {}
    with path.open(encoding="utf-8") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                print(f"warning: skip invalid line {line_num} in {path}: {line}")
                continue
            regs[parts[1]] = parts[0]
    return regs


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify simplified controlbits output.")
    parser.add_argument("--full", required=True, type=Path, help="Full controlbits file")
    parser.add_argument("--simplified", required=True, type=Path, help="Simplified controlbits file")
    args = parser.parse_args()

    full = load_controlbits(args.full)
    simplified = load_controlbits(args.simplified)

    errors: list[str] = []

    for name, hex_value in simplified.items():
        if name not in full:
            errors.append(f"simplified register not in full output: {name}")
            continue
        if full[name] != hex_value:
            errors.append(
                f"hex mismatch for {name}: full={full[name]} simplified={hex_value}"
            )

    for name in full:
        if name in simplified:
            continue
        if not is_omissible(name, full[name]):
            errors.append(f"non-omissible register missing from simplified output: {name}")

    for name, hex_value in full.items():
        if is_omissible(name, hex_value) and name in simplified:
            errors.append(f"omissible zero register still present in simplified output: {name}")

    removed = len(full) - len(simplified)
    omissible_in_full = sum(1 for name, hex_value in full.items() if is_omissible(name, hex_value))

    print(f"full lines: {len(full)}")
    print(f"simplified lines: {len(simplified)}")
    print(f"removed lines: {removed}")
    print(f"omissible zero lines in full: {omissible_in_full}")

    if errors:
        print("\nFAILED:")
        for err in errors:
            print(f"  - {err}")
        return 1

    if removed == 0 and omissible_in_full > 0:
        print("\nFAILED: simplify removed nothing but full output has omissible zero lines")
        return 1

    print("\nOK: simplified output is a valid subset with correct omissions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
