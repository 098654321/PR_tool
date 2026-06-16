#!/usr/bin/env python3
"""Convert kiwi/xinzhai net_path_info.txt (old-tool coords) to PR_tool path format."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Literal

COB_ARRAY_HEIGHT = 9
EXTERN_INDICES = [0, 9, 18, 27, 36, 45, 54, 63, 120, 113, 106, 99, 92, 85, 78, 71]

EndpointKind = Literal["chip", "extio", "nege", "pose", "unknown"]

COORD_PATTERN = re.compile(
    r"(?P<key>dir|TOB_x|TOB_y|bumpgrid_x|bumpgrid_y|index)\s*:\s*(-?\d+)"
)
BRACED_COORD_LINE = re.compile(r"\{[^}]+\}")


def parse_coord_dict(line: str) -> dict[str, int] | None:
    """Extract integer fields from a brace-wrapped old-tool coordinate line."""
    match = BRACED_COORD_LINE.search(line)
    if not match:
        return None
    fields: dict[str, int] = {}
    for key, value in COORD_PATTERN.findall(match.group(0)):
        fields[key] = int(value)
    return fields or None


def classify_endpoint(coord: dict[str, int]) -> EndpointKind:
    bump_x = coord.get("bumpgrid_x")
    if bump_x is None:
        return "unknown"
    if bump_x == -1:
        return "nege"
    if bump_x == -2:
        return "pose"
    if bump_x == -3:
        return "extio"
    if bump_x >= 0:
        return "chip"
    return "unknown"


def old_dir_to_new(dir_value: int) -> str:
    return "Vertical" if dir_value == 0 else "Horizontal"


def format_bump_coord(row: int, col: int, index: int) -> str:
    return f"{{ row: {row}, col: {col}, index: {index} }}"


def format_track_coord(row: int, col: int, dir_value: int, index: int) -> str:
    return (
        f"{{ row: {row}, col: {col}, dir: {old_dir_to_new(dir_value)}, index: {index} }}"
    )


def convert_chip_bump(coord: dict[str, int]) -> str:
    tob_row = (COB_ARRAY_HEIGHT - coord["TOB_x"] - 1) // 2
    tob_col = coord["TOB_y"] // 3
    row = 1 + 2 * tob_row
    col = 3 * tob_col
    index = coord["bumpgrid_x"] + 8 * coord["bumpgrid_y"]
    # #region agent log
    import json as _json, time as _time
    _log_path = Path(__file__).resolve().parents[3] / ".cursor" / "debug-8edc9c.log"
    with _log_path.open("a", encoding="utf-8") as _lf:
        _lf.write(_json.dumps({"sessionId": "8edc9c", "runId": "trans_path", "hypothesisId": "A",
            "location": "trans_path_old2new.py:convert_chip_bump",
            "message": "chip bump coord conversion",
            "data": {"TOB_x": coord["TOB_x"], "TOB_y": coord["TOB_y"],
                     "tob_row": tob_row, "tob_col": tob_col, "row": row, "col": col, "index": index},
            "timestamp": int(_time.time() * 1000)}) + "\n")
    # #endregion
    return format_bump_coord(row, col, index)


def convert_track(coord: dict[str, int]) -> str:
    row = COB_ARRAY_HEIGHT - coord["TOB_x"]
    col = coord["TOB_y"]
    return format_track_coord(row, col, coord["dir"], coord["index"])


def convert_extio(coord: dict[str, int]) -> str:
    slot = coord["bumpgrid_y"]
    if slot < 0 or slot >= len(EXTERN_INDICES):
        raise ValueError(f"invalid external IO slot: {slot}")
    row = COB_ARRAY_HEIGHT - coord["TOB_x"]
    col = coord["TOB_y"]
    index = EXTERN_INDICES[slot]
    return format_track_coord(row, col, coord["dir"], index)


def convert_metadata_coord_line(line: str) -> str:
    """Convert Source/Sink summary lines while preserving indentation and label."""
    for label in ("Source:", "Sink"):
        if label in line:
            prefix, _, rest = line.partition("{")
            coord = parse_coord_dict("{" + rest)
            if coord is None:
                return line
            kind = classify_endpoint(coord)
            if kind == "chip":
                converted = convert_chip_bump(coord)
            elif kind == "extio":
                converted = convert_extio(coord)
            elif kind in ("nege", "pose"):
                converted = f"{{ endpoint: {kind} }}"
            else:
                return line
            return f"{prefix}{converted}\n"
    return line


def convert_endpoint_line(
    label: str,
    coord: dict[str, int],
    adjacent_tracks: list[dict[str, int]],
    use_first_adjacent: bool,
) -> str:
    kind = classify_endpoint(coord)
    if kind == "chip":
        return f"{label} {convert_chip_bump(coord)}\n"
    if kind == "extio":
        return f"{label.replace('bump', 'track')} {convert_extio(coord)}\n"
    if kind in ("nege", "pose"):
        if adjacent_tracks:
            track = adjacent_tracks[0] if use_first_adjacent else adjacent_tracks[-1]
            track_line = convert_track(track)
            return (
                f"{label.replace('bump', 'track')} {track_line}  # endpoint: {kind}\n"
            )
        sys.stderr.write(
            f"warning: no adjacent track for {kind} endpoint; emitting placeholder\n"
        )
        return f"{label.replace('bump', 'track')} {{ endpoint: {kind} }}\n"
    sys.stderr.write(f"warning: unknown endpoint kind for line starting with {label}\n")
    return f"{label} {line_dict_to_old_brace(coord)}\n"


def line_dict_to_old_brace(coord: dict[str, int]) -> str:
    if "index" in coord and "bumpgrid_x" not in coord:
        return (
            f"{{ dir: {coord['dir']}, TOB_x: {coord['TOB_x']}, "
            f"TOB_y: {coord['TOB_y']}, index: {coord['index']} }}"
        )
    return (
        f"{{ dir: {coord['dir']}, TOB_x: {coord['TOB_x']}, TOB_y: {coord['TOB_y']}, "
        f"bumpgrid_x: {coord['bumpgrid_x']}, bumpgrid_y: {coord['bumpgrid_y']} }}"
    )


def convert_path_block(lines: list[str]) -> list[str]:
    if not lines:
        return []

    output: list[str] = []
    begin_coord: dict[str, int] | None = None
    end_coord: dict[str, int] | None = None
    track_coords: list[dict[str, int]] = []
    failed = False

    for raw in lines:
        line = raw.rstrip("\n")
        stripped = line.strip()

        if stripped == "Printing path...":
            continue

        if stripped == "Routing failed for this sink":
            failed = True
            continue

        if stripped.startswith("Begin_bump:"):
            begin_coord = parse_coord_dict(line)
            continue

        if stripped.startswith("End_bump:"):
            end_coord = parse_coord_dict(line)
            continue

        coord = parse_coord_dict(line)
        if coord is not None and "index" in coord and "bumpgrid_x" not in coord:
            track_coords.append(coord)
            continue

    output.append("Printing path...\n")

    if begin_coord is not None:
        output.append(convert_endpoint_line("Begin_bump:", begin_coord, track_coords, True))

    for track in track_coords:
        output.append(convert_track(track) + "\n")

    if failed:
        output.append("Routing failed for this sink\n")

    if end_coord is not None:
        output.append(convert_endpoint_line("End_bump:", end_coord, track_coords, False))

    output.append("\n")
    return output


def convert_file(input_path: Path, output_path: Path) -> None:
    text = input_path.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)

    output_lines: list[str] = []
    index = 0
    while index < len(lines):
        line = lines[index]
        stripped = line.strip()

        if stripped.startswith("Source:") or stripped.startswith("Sink "):
            output_lines.append(convert_metadata_coord_line(line))
            index += 1
            continue

        if stripped == "Printing path...":
            block = [line]
            index += 1
            while index < len(lines):
                next_line = lines[index]
                next_stripped = next_line.strip()
                if next_stripped == "":
                    block.append(next_line)
                    index += 1
                    break
                if (
                    next_stripped.startswith("Net index ")
                    or next_stripped.startswith("Sync group ")
                    or next_stripped.startswith("End sync group ")
                    or next_stripped.startswith("Printing path...")
                    or next_stripped.startswith("*****")
                ):
                    break
                block.append(next_line)
                index += 1
            output_lines.extend(convert_path_block(block))
            continue

        output_lines.append(line)
        index += 1

    output_path.write_text("".join(output_lines), encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Convert kiwi/xinzhai net_path_info.txt (old-tool coordinates) "
            "to PR_tool debug.log-style path coordinates."
        )
    )
    parser.add_argument(
        "input",
        type=Path,
        help="Path to net_path_info.txt produced by the old routing tool",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Output path (default: <input_dir>/net_path_info_new.txt)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    input_path: Path = args.input
    if not input_path.is_file():
        parser.error(f"input file does not exist: {input_path}")

    output_path: Path = (
        args.output if args.output is not None else input_path.with_name("net_path_info_new.txt")
    )

    try:
        convert_file(input_path, output_path)
    except ValueError as exc:
        sys.stderr.write(f"error: {exc}\n")
        return 1

    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
