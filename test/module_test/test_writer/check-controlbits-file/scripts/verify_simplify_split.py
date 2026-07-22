#!/usr/bin/env python3
"""Verify simplified four-file output is a correct omit-of-defaults view of full output.

Default hex rules mirror tools/split_regs.py and source/parse/writer/register_defaults.hh:
  - tob_{r}_{c}_track2tob_{0..3} and tob_{r}_{c}_tob2bump_bank{0,1}_en_{0,1}: ffffffff
  - all other registers: 00000000
  - hex comparison is strict literal equality
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

SPLIT_FILES = (
    "botleft_REG0.txt",
    "botright_REG1.txt",
    "topleft_REG2.txt",
    "topright_REG3.txt",
)

ZERO_DEFAULT_HEX = "00000000"
FF_DEFAULT_HEX = "ffffffff"

_TRACK2TOB_RE = re.compile(r"^tob_([0-3])_([0-3])_track2tob_([0-3])$")
_TOB2BUMP_EN_RE = re.compile(r"^tob_([0-3])_([0-3])_tob2bump_bank[01]_en_[01]$")


def is_ff_default_register(reg_name: str) -> bool:
    if not reg_name.startswith("tob_"):
        return False
    if _TRACK2TOB_RE.match(reg_name) is not None:
        return True
    return _TOB2BUMP_EN_RE.match(reg_name) is not None


def default_hex_for(reg_name: str) -> str:
    return FF_DEFAULT_HEX if is_ff_default_register(reg_name) else ZERO_DEFAULT_HEX


def should_omit_simplified_line(hex_value: str, reg_name: str) -> bool:
    return hex_value == default_hex_for(reg_name)


def resolve_split_dir(path: Path) -> Path:
    """Accept either the four files directly or nested under regnamecontrolbit_4part/."""
    nested = path / "regnamecontrolbit_4part"
    if nested.is_dir() and (nested / SPLIT_FILES[0]).is_file():
        return nested
    return path


def load_regs(path: Path) -> dict[str, str]:
    """Load {reg_name: hex} from a three-column split file (hex address reg_name)."""
    regs: dict[str, str] = {}
    with path.open(encoding="utf-8") as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 3:
                print(f"warning: skip invalid line {line_num} in {path}: {line}")
                continue
            regs[parts[2]] = parts[0]
    return regs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify simplified split output matches omit-of-defaults from full output."
    )
    parser.add_argument(
        "--full-dir",
        required=True,
        type=Path,
        help="Full (non-simplified) output dir or .../regnamecontrolbit_4part/",
    )
    parser.add_argument(
        "--simplified-dir",
        required=True,
        type=Path,
        help="Simplified (-s) output dir or .../regnamecontrolbit_4part/",
    )
    args = parser.parse_args()

    full_dir = resolve_split_dir(args.full_dir)
    simplified_dir = resolve_split_dir(args.simplified_dir)

    errors = 0

    for filename in SPLIT_FILES:
        full_path = full_dir / filename
        simplified_path = simplified_dir / filename
        if not full_path.is_file():
            print(f"ERROR: missing full file {full_path}")
            return 1
        if not simplified_path.is_file():
            print(f"ERROR: missing simplified file {simplified_path}")
            return 1

        full_regs = load_regs(full_path)
        simplified_regs = load_regs(simplified_path)

        for name, hex_value in full_regs.items():
            if should_omit_simplified_line(hex_value, name):
                if name in simplified_regs:
                    print(
                        f"[SHOULD_OMIT] {filename} {name}: "
                        f"hex={hex_value} still present in simplified"
                    )
                    errors += 1
            else:
                s = simplified_regs.get(name)
                if s is None:
                    print(
                        f"[MISSING_IN_SIMPLIFIED] {filename} {name}: "
                        f"full={hex_value}"
                    )
                    errors += 1
                elif s != hex_value:
                    print(
                        f"[HEX_MISMATCH] {filename} {name}: "
                        f"full={hex_value} simplified={s}"
                    )
                    errors += 1

        for name, hex_value in simplified_regs.items():
            if name not in full_regs:
                print(
                    f"[EXTRA_IN_SIMPLIFIED] {filename} {name}: "
                    f"simplified={hex_value} absent from full"
                )
                errors += 1

        print(
            f"{filename}: full={len(full_regs)} simplified={len(simplified_regs)}"
        )

    if errors == 0:
        print("OK: simplified output matches omit-of-defaults from full.")
        return 0
    print(f"FAIL: {errors} error(s).")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
