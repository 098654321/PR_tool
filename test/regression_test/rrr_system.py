"""Run actual reader/place/router/writer checks with an external test timeout.

--synthetic-columns is for software tests of incomplete benchmark address maps.
Its generated addresses MUST NOT be used to program hardware.
"""
import argparse
import json
import pathlib
import re
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("module_test", type=pathlib.Path)
    parser.add_argument("config", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--test-timeout", type=float, help="Optional external watchdog in seconds; unset allows long convergence")
    parser.add_argument("--place", action="store_true")
    parser.add_argument("--synthetic-columns", type=int)
    args = parser.parse_args()
    config = args.config.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.synthetic_columns:
        fixture = output / "TEST_ONLY_config"
        fixture.mkdir(exist_ok=True)
        for source in config.glob("*.json"):
            shutil.copy2(source, fixture / source.name)
        settings = json.loads((fixture / "config.json").read_text())
        mapping_path = fixture / settings["reigster_adder"]
        mapping = json.loads(mapping_path.read_text())
        templates = {}
        names = {name for group in mapping.values() for name in group}
        for file, group in mapping.items():
            for name in group:
                match = re.fullmatch(r"cob_(\d+)_(\d+)_(.+)", name)
                if match:
                    row, column, suffix = match.groups()
                    templates[(row, suffix)] = file
        address = 0xE0000000
        for (row, suffix), file in templates.items():
            for column in range(args.synthetic_columns):
                name = f"cob_{row}_{column}_{suffix}"
                if name not in names:
                    mapping[file][name] = f"32'h{address:08x}"
                    address += 4
        mapping_path.write_text(json.dumps(mapping, indent=2))
        (fixture / "TEST_ONLY_ADDRESSES.md").write_text(
            "Synthetic missing COB addresses for software tests only. DO NOT program hardware.\n"
        )
        config = fixture
    command = [str(args.module_test.resolve()), "rrr_system", str(config), str(output),
               "place" if args.place else "route"]
    if args.seconds is not None:
        command.append(str(args.seconds))
    timeout = args.test_timeout
    if timeout is None and args.seconds is not None:
        timeout = max(120, args.seconds + 60)
    result = subprocess.run(command, timeout=timeout)
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
