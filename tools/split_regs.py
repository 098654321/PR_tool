#!/usr/bin/env python3
"""Legacy offline splitter: single-file controlbits_*.txt → four REG files.

Formal PR_tool product output is Writer → regnamecontrolbit_4part/ (see source/).
Keep this script for legacy/external two-column controlbits only (design A1).
Recommended -s path is PR_tool -s (sparse four-file write), not this script as primary.

Default hex rules (mirrors source/parse/writer/register_defaults.hh):
  - tob_{r}_{c}_track2tob_{0..3} and tob_{r}_{c}_tob2bump_bank{0,1}_en_{0,1}: ffffffff
  - all other registers: 00000000
  - hex comparison is strict literal equality (same as C++ should_omit_simplified_line)
"""

import argparse
import json
import os
import re
import sys

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


def should_skip_split_line(reg_name: str, value: str, simplify: bool) -> bool:
    if not simplify:
        return False
    return should_omit_simplified_line(value, reg_name)


def parse_controlbits(controlbits_path):
    """Parses the controlbits file into a dictionary of {name: value}."""
    control_map = {}
    try:
        with open(controlbits_path, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if not line or line.startswith('#'):
                    continue

                parts = line.split()
                if len(parts) >= 2:
                    value = parts[0]
                    name = parts[1]
                    control_map[name] = value
                else:
                    print(f"Warning: Skipping invalid line {line_num} in {controlbits_path}: {line}")
    except FileNotFoundError:
        print(f"Error: Controlbits file not found: {controlbits_path}")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading controlbits file: {e}")
        sys.exit(1)

    return control_map


def load_register_map(json_path):
    """Loads the register map JSON file."""
    try:
        with open(json_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: Register map JSON file not found: {json_path}")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading JSON file: {e}")
        sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Split a full PR_tool controlbits file into register map files. "
            "Use -s on the full input to omit default-valued registers from split output."
        )
    )
    parser.add_argument("--controlbits", "-c", required=True, help="Path to the input controlbits file (e.g., controlbits_0.txt)")
    parser.add_argument("--json_map", "-j", required=True, help="Path to the register map JSON file")
    parser.add_argument("--output_dir", "-o", help="Directory to save the output files. Defaults to the same directory as controlbits file.")
    parser.add_argument(
        "-s",
        "--simplify-controlbits-file",
        action="store_true",
        help=(
            "Omit split output lines whose hex equals the register default. "
            "Input should be full controlbits from PR_tool (without -s)."
        ),
    )

    args = parser.parse_args()

    simplify = args.simplify_controlbits_file
    controlbits_path = args.controlbits
    json_path = args.json_map
    output_dir = args.output_dir if args.output_dir else os.path.dirname(controlbits_path)

    if not os.path.exists(output_dir):
        try:
            os.makedirs(output_dir)
        except OSError as e:
            print(f"Error creating output directory: {e}")
            sys.exit(1)

    log_file_path = os.path.join(output_dir, "report.log")

    class Logger(object):
        def __init__(self, filename):
            self.terminal = sys.stdout
            self.log = open(filename, "w")

        def write(self, message):
            self.terminal.write(message)
            self.log.write(message)
            self.log.flush()

        def flush(self):
            self.terminal.flush()
            self.log.flush()

    sys.stdout = Logger(log_file_path)

    print(f"Loading register map from {json_path}...")
    reg_map = load_register_map(json_path)

    print(f"Loading controlbits from {controlbits_path}...")
    if simplify:
        print("Simplify mode: enabled (-s); default-valued registers are omitted from split output only")
    control_data = parse_controlbits(controlbits_path)

    used_control_regs = set()
    missing_in_controlbits = []
    omitted_default_lines = 0
    written_lines = 0

    print("\nProcessing files...")

    for filename, regs in reg_map.items():
        output_file_path = os.path.join(output_dir, filename)
        print(f"Generating {output_file_path}...")

        try:
            with open(output_file_path, 'w') as out_f:
                for reg_name, address in regs.items():
                    if reg_name in control_data:
                        value = control_data[reg_name]
                        used_control_regs.add(reg_name)
                        if should_skip_split_line(reg_name, value, simplify):
                            omitted_default_lines += 1
                            continue
                        out_f.write(f"{value} {address} {reg_name}\n")
                        written_lines += 1
                    else:
                        missing_in_controlbits.append((filename, reg_name))
        except Exception as e:
            print(f"Error writing to {output_file_path}: {e}")

    print("\n" + "="*40)
    print("REPORT")
    print("="*40)

    if simplify:
        print(f"\n[OK] Wrote {written_lines} line(s); omitted {omitted_default_lines} default-valued line(s) from split output.")

    if missing_in_controlbits:
        if simplify:
            print(
                f"\n[NOTE] {len(missing_in_controlbits)} register(s) in the JSON map were not found in "
                f"{os.path.basename(controlbits_path)}. With -s, registers are only omitted when present "
                f"in the input and equal to default; registers listed here are absent from the input file."
            )
        else:
            print(f"\n[WARNING] The following registers are defined in JSON but MISSING in {os.path.basename(controlbits_path)}:")
            for fname, rname in missing_in_controlbits:
                print(f"  - {rname} (expected in {fname})")
    else:
        print(f"\n[OK] All registers in JSON were found in {os.path.basename(controlbits_path)}.")

    all_control_regs = set(control_data.keys())
    extra_regs = all_control_regs - used_control_regs

    if extra_regs:
        print(f"\n[WARNING] The following registers are present in {os.path.basename(controlbits_path)} but NOT used in any JSON file mapping:")
        for rname in sorted(extra_regs):
            print(f"  - {rname}")
    else:
        print(f"\n[OK] All registers in {os.path.basename(controlbits_path)} were mapped to files.")

    print("\nDone.")


if __name__ == "__main__":
    main()
