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


# Kiwi COB side constants (printControlBit.h)
KIWI_UP, KIWI_DOWN, KIWI_LEFT, KIWI_RIGHT = 0, 1, 2, 3


def kiwi_cob_index(cur: dict[str, int], father: dict[str, int]) -> tuple[int, int]:
    """Replicate printControlBit.cpp cobIndex. cur=downstream, father=upstream."""
    candidates: list[list[tuple[int, int]]] = []
    for track in (cur, father):
        cx, cy, d = track["TOB_x"], track["TOB_y"], track["dir"]
        opts: list[tuple[int, int]] = []
        for _ in range(2):
            opts.append((cx, cy))
            if d and cy > 0:
                cy -= 1
            elif cx > 0:
                cx -= 1
        candidates.append(opts)
    for a in candidates[0]:
        for b in candidates[1]:
            if a == b:
                return b
    raise ValueError(
        f"cannot find shared COB for tracks {cur} and {father}"
    )


def kiwi_cob_side(channel_dir: int, cx: int, cy: int, cob_x: int, cob_y: int) -> int:
    """Replicate printControlBit.cpp cobDirection (channel_dir: 0=vert, 1=horiz)."""
    if channel_dir:
        return KIWI_DOWN if cy == cob_y else KIWI_UP
    return KIWI_LEFT if cx == cob_x else KIWI_RIGHT


def rotate_kiwi_side_cw(side: int) -> int:
    """90° clockwise rotation of COB side (routing.cpp right-rotate comment)."""
    return {
        KIWI_UP: KIWI_RIGHT,
        KIWI_RIGHT: KIWI_DOWN,
        KIWI_DOWN: KIWI_LEFT,
        KIWI_LEFT: KIWI_UP,
    }[side]


def pr_track_from_cob_side(
    cob_x: int, cob_y: int, kiwi_side: int, index: int
) -> tuple[int, int, str, int]:
    """Map kiwi COB + side to PR_TOOL TrackCoord (row, col, dir, index)."""
    pr_row = COB_ARRAY_HEIGHT - 1 - cob_x
    pr_col = cob_y
    pr_side = rotate_kiwi_side_cw(kiwi_side)
    if pr_side == KIWI_DOWN:
        return pr_row, pr_col, "Vertical", index
    if pr_side == KIWI_UP:
        return pr_row + 1, pr_col, "Vertical", index
    if pr_side == KIWI_LEFT:
        return pr_row, pr_col, "Horizontal", index
    return pr_row, pr_col + 1, "Horizontal", index


def format_pr_track(row: int, col: int, dir_name: str, index: int) -> str:
    return f"{{ row: {row}, col: {col}, dir: {dir_name}, index: {index} }}"


def convert_track_at_cob(track: dict[str, int], cob_x: int, cob_y: int) -> str:
    side = kiwi_cob_side(track["dir"], track["TOB_x"], track["TOB_y"], cob_x, cob_y)
    row, col, dir_name, index = pr_track_from_cob_side(cob_x, cob_y, side, track["index"])
    return format_pr_track(row, col, dir_name, index)


def convert_path_tracks(tracks: list[dict[str, int]]) -> list[str]:
    """Convert a forward-ordered track list using pairwise cobIndex."""
    if not tracks:
        return []
    lines: list[str] = []
    for i, track in enumerate(tracks):
        if i + 1 < len(tracks):
            cob_x, cob_y = kiwi_cob_index(tracks[i + 1], track)
        else:
            cob_x, cob_y = kiwi_cob_index(track, tracks[i - 1])
        lines.append(convert_track_at_cob(track, cob_x, cob_y) + "\n")
    return lines


def convert_track(coord: dict[str, int]) -> str:
    """Fallback for isolated track lines: assume COB at channel position."""
    return convert_track_at_cob(coord, coord["TOB_x"], coord["TOB_y"])


def format_bump_coord(row: int, col: int, index: int) -> str:
    return f"{{ row: {row}, col: {col}, index: {index} }}"


def convert_chip_bump(coord: dict[str, int]) -> str:
    tob_row = (COB_ARRAY_HEIGHT - coord["TOB_x"] - 1) // 2
    tob_col = coord["TOB_y"] // 3
    row = 1 + 2 * tob_row
    col = 3 * tob_col
    index = coord["bumpgrid_x"] + 8 * coord["bumpgrid_y"]
    return format_bump_coord(row, col, index)


def convert_extio(coord: dict[str, int]) -> str:
    slot = coord["bumpgrid_y"]
    if slot < 0 or slot >= len(EXTERN_INDICES):
        raise ValueError(f"invalid external IO slot: {slot}")
    # External port TXT→JSON: row = COB_ARRAY_HEIGHT - x (section 4 of coord doc)
    row = COB_ARRAY_HEIGHT - coord["TOB_x"]
    col = coord["TOB_y"]
    index = EXTERN_INDICES[slot]
    dir_name = "Vertical" if coord["dir"] == 0 else "Horizontal"
    return format_pr_track(row, col, dir_name, index)


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

    output.extend(convert_path_tracks(track_coords))

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
