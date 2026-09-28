#!/usr/bin/env python3
"""Plot per-COBUnit physical-Channel load from final routed paths.

The input format and geometry are shared with ``plot_routes.py``.  A Channel
load is the number of distinct physical tracks from one COBUnit that appear in
that Channel.  Repeated multi-terminal path trunks are therefore counted once,
and every load is in [0, 8].
"""

from __future__ import annotations

import argparse
import csv
import os
import tempfile
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "pr_tool-matplotlib"))
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.axes import Axes
from matplotlib.cm import ScalarMappable
from matplotlib.patches import Rectangle

from plot_routes import (
    COB_HALF,
    COB_SIDE,
    EXTERNAL_TRACK_LENGTH,
    TOB_HEIGHT,
    TOB_WIDTH,
    TRACK_COUNT,
    TrackNode,
    read_final_paths,
    validate_paths,
)


UNIT_COUNT = 16
TRACKS_PER_CHANNEL_UNIT = 8
CHANNEL_LINE_WIDTH = 8.0
CHANNEL_OUTLINE_WIDTH = 9.5
ZERO_LOAD_COLOR = "#ececec"
COB_FACE_COLOR = "#d8d8d8"
TOB_FACE_COLOR = "#70a86b"


@dataclass(frozen=True, order=True)
class Channel:
    direction: str
    row: int
    col: int


@dataclass(frozen=True)
class UnitLoadSummary:
    unit: int
    total_track_channel_occupancy: int
    occupied_channels: int
    peak_load: int
    peak_channel_count: int
    saturated_channels: int


def track_unit(track_index: int) -> int:
    """Match test_ILP/common/ilp_types.hh::map_track exactly."""
    if not 0 <= track_index < TRACK_COUNT:
        raise ValueError(f"track index outside [0, {TRACK_COUNT - 1}]: {track_index}")
    return track_index % 8 if track_index < 64 else track_index % 8 + 8


def all_channels(rows: int, cols: int) -> tuple[Channel, ...]:
    horizontal = (
        Channel("H", row, col)
        for row in range(rows)
        for col in range(cols + 1)
    )
    vertical = (
        Channel("V", row, col)
        for row in range(rows + 1)
        for col in range(cols)
    )
    return tuple(horizontal) + tuple(vertical)


def collect_channel_loads(paths: list, rows: int, cols: int) -> list[dict[Channel, int]]:
    """Count distinct routed tracks in every (unit, physical Channel)."""
    tracks_by_unit_channel: list[dict[Channel, set[int]]] = [
        defaultdict(set) for _ in range(UNIT_COUNT)
    ]
    for path in paths:
        for node in path.nodes:
            if not isinstance(node, TrackNode):
                continue
            channel = Channel(node.direction, node.row, node.col)
            unit = track_unit(node.index)
            tracks_by_unit_channel[unit][channel].add(node.index)

    known_channels = frozenset(all_channels(rows, cols))
    loads: list[dict[Channel, int]] = []
    for unit, track_sets in enumerate(tracks_by_unit_channel):
        unknown = set(track_sets) - known_channels
        if unknown:
            raise ValueError(f"unit {unit} contains out-of-array Channels: {sorted(unknown)[:3]}")
        unit_loads = {channel: len(indices) for channel, indices in track_sets.items()}
        overloaded = {channel: load for channel, load in unit_loads.items() if load > 8}
        if overloaded:
            raise ValueError(
                f"unit {unit} exceeds the eight-track Channel capacity: "
                f"{sorted(overloaded.items())[:3]}"
            )
        loads.append(unit_loads)
    return loads


def channel_segment(
    channel: Channel,
    rows: int,
    cols: int,
) -> tuple[tuple[float, float], tuple[float, float]]:
    if channel.direction == "H":
        y = float(channel.row)
        if channel.col == 0:
            return ((-COB_HALF - EXTERNAL_TRACK_LENGTH, y), (-COB_HALF, y))
        if channel.col == cols:
            edge = cols - 1 + COB_HALF
            return ((edge, y), (edge + EXTERNAL_TRACK_LENGTH, y))
        return (
            (channel.col - 1 + COB_HALF, y),
            (channel.col - COB_HALF, y),
        )

    x = float(channel.col)
    if channel.row == 0:
        return ((x, -COB_HALF - EXTERNAL_TRACK_LENGTH), (x, -COB_HALF))
    if channel.row == rows:
        edge = rows - 1 + COB_HALF
        return ((x, edge), (x, edge + EXTERNAL_TRACK_LENGTH))
    return (
        (x, channel.row - 1 + COB_HALF),
        (x, channel.row - COB_HALF),
    )


def channel_midpoint(channel: Channel, rows: int, cols: int) -> tuple[float, float]:
    start, end = channel_segment(channel, rows, cols)
    return ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)


def summarize(unit: int, loads: dict[Channel, int]) -> UnitLoadSummary:
    peak = max(loads.values(), default=0)
    return UnitLoadSummary(
        unit=unit,
        total_track_channel_occupancy=sum(loads.values()),
        occupied_channels=sum(load > 0 for load in loads.values()),
        peak_load=peak,
        peak_channel_count=sum(load == peak for load in loads.values()) if peak else 0,
        saturated_channels=sum(load == TRACKS_PER_CHANNEL_UNIT for load in loads.values()),
    )


def draw_hardware(ax: Axes, rows: int, cols: int) -> None:
    for row in range(rows):
        for col in range(cols):
            ax.add_patch(
                Rectangle(
                    (col - COB_HALF, row - COB_HALF),
                    COB_SIDE,
                    COB_SIDE,
                    facecolor=COB_FACE_COLOR,
                    edgecolor="#555555",
                    linewidth=0.55,
                    zorder=3,
                )
            )

    for tob_row in range(4):
        for tob_col in range(4):
            channel_row = 1 + 2 * tob_row
            channel_col = 3 * tob_col
            if channel_row >= rows or channel_col >= cols:
                continue
            ax.add_patch(
                Rectangle(
                    (channel_col - TOB_WIDTH / 2, channel_row - 0.5 - TOB_HEIGHT / 2),
                    TOB_WIDTH,
                    TOB_HEIGHT,
                    facecolor=TOB_FACE_COLOR,
                    edgecolor="black",
                    linewidth=0.60,
                    zorder=4,
                )
            )


def draw_unit_load(
    loads: dict[Channel, int],
    summary: UnitLoadSummary,
    rows: int,
    cols: int,
    output: Path,
) -> None:
    figure, ax = plt.subplots(figsize=(14.2, 9.8), dpi=240)
    color_map = plt.get_cmap("YlOrRd")
    normalization = colors.Normalize(vmin=0, vmax=TRACKS_PER_CHANNEL_UNIT)

    for channel in all_channels(rows, cols):
        start, end = channel_segment(channel, rows, cols)
        load = loads.get(channel, 0)
        ax.plot(
            (start[0], end[0]),
            (start[1], end[1]),
            color="#555555",
            linewidth=CHANNEL_OUTLINE_WIDTH,
            solid_capstyle="butt",
            zorder=1,
        )
        ax.plot(
            (start[0], end[0]),
            (start[1], end[1]),
            color=ZERO_LOAD_COLOR if load == 0 else color_map(normalization(load)),
            linewidth=CHANNEL_LINE_WIDTH,
            solid_capstyle="butt",
            zorder=2,
        )

    if summary.peak_load:
        peak_x: list[float] = []
        peak_y: list[float] = []
        for channel, load in loads.items():
            if load == summary.peak_load:
                x, y = channel_midpoint(channel, rows, cols)
                peak_x.append(x)
                peak_y.append(y)
        ax.scatter(
            peak_x,
            peak_y,
            marker="o",
            s=26,
            facecolors="none",
            edgecolors="black",
            linewidths=0.85,
            zorder=5,
            label=f"peak load = {summary.peak_load}",
        )

    draw_hardware(ax, rows, cols)
    extent = COB_HALF + EXTERNAL_TRACK_LENGTH + 0.10
    ax.set_xlim(-extent, cols - 1 + extent)
    ax.set_ylim(-extent, rows - 1 + extent)
    ax.set_aspect("equal")
    ax.set_xlabel("COB column")
    ax.set_ylabel("COB row")
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.set_title(
        f"COBUnit {summary.unit} Channel load\n"
        f"track-Channel occupancy={summary.total_track_channel_occupancy}, "
        f"used Channels={summary.occupied_channels}/{len(all_channels(rows, cols))}, "
        f"peak={summary.peak_load}/8 ({summary.peak_channel_count} Channels), "
        f"full={summary.saturated_channels}"
    )
    ax.grid(False)
    if summary.peak_load:
        ax.legend(loc="upper right", framealpha=0.92)

    color_bar = figure.colorbar(
        ScalarMappable(norm=normalization, cmap=color_map),
        ax=ax,
        fraction=0.025,
        pad=0.025,
        ticks=range(TRACKS_PER_CHANNEL_UNIT + 1),
    )
    color_bar.set_label("occupied tracks in this COBUnit / Channel")

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(output, facecolor="white", bbox_inches="tight")
    plt.close(figure)


def write_summary(summaries: list[UnitLoadSummary], output: Path) -> None:
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "unit",
                "total_track_channel_occupancy",
                "occupied_channels",
                "peak_load",
                "peak_channel_count",
                "saturated_channels",
            )
        )
        for item in summaries:
            writer.writerow(
                (
                    item.unit,
                    item.total_track_channel_occupancy,
                    item.occupied_channels,
                    item.peak_load,
                    item.peak_channel_count,
                    item.saturated_channels,
                )
            )


def render(log: Path, rows: int, cols: int, output_dir: Path) -> None:
    paths = read_final_paths(log)
    validate_paths(paths, rows, cols)
    loads_by_unit = collect_channel_loads(paths, rows, cols)
    summaries = [summarize(unit, loads) for unit, loads in enumerate(loads_by_unit)]

    output_dir.mkdir(parents=True, exist_ok=True)
    for unit, loads in enumerate(loads_by_unit):
        output = output_dir / f"unit_{unit:02d}_channel_load.png"
        draw_unit_load(loads, summaries[unit], rows, cols, output)
        print(
            f"unit={unit:02d} occupancy={summaries[unit].total_track_channel_occupancy} "
            f"used_channels={summaries[unit].occupied_channels} "
            f"peak={summaries[unit].peak_load}/8 "
            f"saturated={summaries[unit].saturated_channels} -> {output}"
        )
    summary_path = output_dir / "unit_channel_load_summary.csv"
    write_summary(summaries, summary_path)
    print(f"wrote {len(summaries)} unit heatmaps and summary to {output_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True, help="final verbose routing log")
    parser.add_argument("--row", type=int, required=True, help="number of COB rows")
    parser.add_argument("--col", type=int, required=True, help="number of COB columns")
    parser.add_argument("--output", type=Path, required=True, help="output directory")
    args = parser.parse_args()

    if args.row <= 0 or args.col <= 0:
        parser.error("--row and --col must both be positive integers")
    try:
        render(args.log, args.row, args.col, args.output)
    except (OSError, UnicodeError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
