#!/usr/bin/env python3
"""Draw global-routing guides and final routes in the same hardware view.

The V20 diagnostic log records a global guide as a ``global guide`` block with
one ``selected:`` line.  The final SAT route uses the same ``route net=`` /
``path:`` format consumed by :mod:`plot_routes`.

Guide geometry is Channel-level: an orange dashed line marks every selected
physical Channel.  Final routes keep the existing black Track/TOB geometry so
the two levels remain visually distinct.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import tempfile
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "pr_tool-matplotlib"))
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from plot_routes import (  # noqa: E402
    COB_HALF,
    COB_SIDE,
    EXTERNAL_TRACK_LENGTH,
    ROUTE_LINE_WIDTH,
    TOB_HEIGHT,
    TOB_WIDTH,
    SUCCESS_COLOR,
    RoutedPath,
    TrackNode,
    adjacent_cobs,
    cob_contact,
    read_final_paths,
    track_segment,
    validate_paths,
)
from plot_unit_channel_load import channel_segment  # noqa: E402


GUIDE_HEADER_RE = re.compile(
    r'\bglobal guide net="(?P<net_name>[^"]*)" id=(?P<net_id>-?\d+) '
    r'pair=\(demand=(?P<demand>\d+),source=(?P<source>\d+)\) '
    r'unit=(?P<unit>\d+)\b'
)
SELECTED_RE = re.compile(r"\bselected:\s*(?P<path>.+)$")
CHANNEL_RE = re.compile(
    r"Channel\((?P<direction>[HV]),(?P<row>-?\d+),(?P<col>-?\d+)\)"
)
COB_NODE_RE = re.compile(r"COB\((?P<row>-?\d+),(?P<col>-?\d+)\)")
TOB_NODE_RE = re.compile(r"TOB\((?P<row>\d+),(?P<col>\d+)\)")
PORT_NODE_RE = re.compile(
    r"Port\((?P<direction>[HV]),(?P<row>-?\d+),(?P<col>-?\d+),i(?P<index>\d+)\)"
)

GUIDE_COLOR = "#d95f02"
GUIDE_LINE_WIDTH = 5.0
GUIDE_ALPHA = 0.42
ROUTE_ALPHA = 0.62
COB_FACE_COLOR = "#d6bd00"
TOB_FACE_COLOR = "#70a86b"


@dataclass(frozen=True, order=True)
class Channel:
    direction: str
    row: int
    col: int


@dataclass(frozen=True)
class GuideNode:
    kind: str
    row: int
    col: int
    direction: str | None = None
    index: int | None = None


@dataclass(frozen=True)
class GuidePath:
    net_id: int
    net_name: str
    demand_id: int
    source_id: int
    unit: int
    channels: tuple[Channel, ...]
    nodes: tuple[GuideNode, ...]


@dataclass
class GuideBlock:
    net_id: int
    net_name: str
    demand_id: int
    source_id: int
    unit: int
    channels: tuple[Channel, ...] = ()
    nodes: tuple[GuideNode, ...] = ()


def parse_guide_node(token: str, line_number: int) -> GuideNode:
    channel_match = CHANNEL_RE.fullmatch(token)
    if channel_match:
        return GuideNode(
            "channel",
            int(channel_match.group("row")),
            int(channel_match.group("col")),
            channel_match.group("direction"),
        )

    cob_match = COB_NODE_RE.fullmatch(token)
    if cob_match:
        return GuideNode("cob", int(cob_match.group("row")), int(cob_match.group("col")))

    tob_match = TOB_NODE_RE.fullmatch(token)
    if tob_match:
        return GuideNode("tob", int(tob_match.group("row")), int(tob_match.group("col")))

    port_match = PORT_NODE_RE.fullmatch(token)
    if port_match:
        return GuideNode(
            "port",
            int(port_match.group("row")),
            int(port_match.group("col")),
            port_match.group("direction"),
            int(port_match.group("index")),
        )

    raise ValueError(f"line {line_number}: unsupported global-guide node: {token!r}")


def read_global_guides(log_path: Path) -> list[GuidePath]:
    """Read the latest selected path for each global-guide pair."""
    if not log_path.is_file():
        raise ValueError(f"log file does not exist or is not a regular file: {log_path}")

    latest: dict[tuple[int, int, int], GuidePath] = {}
    block: GuideBlock | None = None

    def save_block(current: GuideBlock | None) -> None:
        if current is None or not current.nodes:
            return
        key = (current.net_id, current.demand_id, current.source_id)
        latest[key] = GuidePath(
            current.net_id,
            current.net_name,
            current.demand_id,
            current.source_id,
            current.unit,
            current.channels,
            current.nodes,
        )

    for line_number, line in enumerate(log_path.read_text(encoding="utf-8").splitlines(), 1):
        header_match = GUIDE_HEADER_RE.search(line)
        if header_match:
            save_block(block)
            block = GuideBlock(
                net_id=int(header_match.group("net_id")),
                net_name=header_match.group("net_name"),
                demand_id=int(header_match.group("demand")),
                source_id=int(header_match.group("source")),
                unit=int(header_match.group("unit")),
            )
            continue

        if block is None:
            continue
        selected_match = SELECTED_RE.search(line)
        if selected_match:
            nodes = tuple(
                parse_guide_node(token.strip(), line_number)
                for token in selected_match.group("path").split(" -> ")
            )
            block.nodes = nodes
            # Preserve selected-path order while removing repeated Channel hops.
            block.channels = tuple(
                dict.fromkeys(
                    Channel(node.direction, node.row, node.col)
                    for node in nodes
                    if node.kind == "channel" and node.direction is not None
                )
            )

    save_block(block)

    if not latest:
        raise ValueError(f"no selected global-guide paths found in {log_path}")
    return sorted(latest.values(), key=lambda guide: (guide.net_id, guide.demand_id, guide.source_id))


def validate_guides(guides: list[GuidePath], rows: int, cols: int) -> None:
    for guide in guides:
        for channel in guide.channels:
            if channel.direction == "H":
                valid = 0 <= channel.row < rows and 0 <= channel.col <= cols
                expected = f"row in [0, {rows - 1}], col in [0, {cols}]"
            else:
                valid = 0 <= channel.row <= rows and 0 <= channel.col < cols
                expected = f"row in [0, {rows}], col in [0, {cols - 1}]"
            if not valid:
                raise ValueError(
                    f"guide net {guide.net_id} demand {guide.demand_id}: Channel "
                    f"{channel} outside {rows}x{cols} array ({expected})"
                )


def classify_net(net_name: str, display: str | None = None) -> str:
    if net_name.startswith("Pose"):
        return "pose"
    if net_name.startswith("Nege"):
        return "nege"
    if net_name.startswith("TrackToBumpsNet_"):
        return "track_to_bumps"
    if net_name.startswith("TrackToBumpNet_"):
        return "track_to_bump"
    if net_name.startswith("BumpToBumpNet_"):
        return "bump_to_bump"
    if display == "SyncBus" or net_name.startswith("SyncNet"):
        return "sync"
    raise ValueError(f"unsupported net type for visualization: {net_name!r}")


PLOT_LABELS = {
    "pose": "Pose",
    "nege": "Nege",
    "track_to_bump": "TrackToBump",
    "track_to_bumps": "TrackToBumps",
    "bump_to_bump": "BumpToBump",
    "sync": "Sync net",
}


def draw_hardware(ax: Axes, rows: int, cols: int) -> None:
    for row in range(rows):
        for col in range(cols):
            ax.add_patch(
                Rectangle(
                    (col - COB_HALF, row - COB_HALF),
                    COB_SIDE,
                    COB_SIDE,
                    facecolor=COB_FACE_COLOR,
                    edgecolor="black",
                    linewidth=0.70,
                    zorder=2,
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
                    zorder=3,
                )
            )


def guide_node_anchor(
    channel: Channel,
    node: GuideNode,
    other: GuideNode,
    rows: int,
    cols: int,
) -> tuple[float, float]:
    start, end = channel_segment(channel, rows, cols)
    if node.kind == "cob":
        center = (float(node.col), float(node.row))
        return min(
            (start, end),
            key=lambda point: (point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2,
        )

    if node.kind == "port":
        if channel.direction == "H":
            return start if channel.col == 0 else end
        return start if channel.row == 0 else end

    if node.kind == "tob":
        expected = (1 + 2 * node.row, 3 * node.col)
        if channel.direction != "V" or (channel.row, channel.col) != expected:
            raise ValueError(
                f"TOB({node.row},{node.col}) is not attached to {channel}"
            )
        center_y = 0.5 + 2.0 * node.row
        if other.kind == "cob" and other.row == channel.row:
            return float(channel.col), center_y + TOB_HEIGHT / 2
        if other.kind == "cob" and other.row == channel.row - 1:
            return float(channel.col), center_y - TOB_HEIGHT / 2
        return float(channel.col), center_y

    raise ValueError(f"unsupported guide endpoint kind: {node.kind!r}")


def draw_guides(ax: Axes, guides: list[GuidePath], rows: int, cols: int) -> int:
    channels = {
        channel
        for guide in guides
        for channel in guide.channels
    }
    segments: set[tuple[tuple[float, float], tuple[float, float]]] = set()
    for guide in guides:
        for index, node in enumerate(guide.nodes):
            if node.kind != "channel":
                continue
            if index == 0 or index + 1 == len(guide.nodes):
                raise ValueError("global-guide Channel node is missing an endpoint")
            channel = Channel(node.direction or "", node.row, node.col)
            previous = guide.nodes[index - 1]
            following = guide.nodes[index + 1]
            start = guide_node_anchor(channel, previous, following, rows, cols)
            end = guide_node_anchor(channel, following, previous, rows, cols)
            segments.add(tuple(sorted((start, end))))

    for start, end in sorted(segments):
        # The underlay makes the endpoint connection visible even where the
        # dashed overlay happens to leave a gap at a short Channel segment.
        ax.plot(
            (start[0], end[0]),
            (start[1], end[1]),
            color=GUIDE_COLOR,
            linewidth=GUIDE_LINE_WIDTH + 2.0,
            alpha=GUIDE_ALPHA * 0.55,
            solid_capstyle="round",
            zorder=4,
        )
        ax.plot(
            (start[0], end[0]),
            (start[1], end[1]),
            color=GUIDE_COLOR,
            linewidth=GUIDE_LINE_WIDTH,
            alpha=GUIDE_ALPHA,
            linestyle=(0, (5.0, 2.5)),
            solid_capstyle="round",
            zorder=4,
        )
    return len(channels)


def draw_route_geometry(ax: Axes, paths: list[RoutedPath], rows: int, cols: int) -> None:
    used_tracks: set[TrackNode] = set()
    used_connectors: set[frozenset[TrackNode]] = set()
    for path in paths:
        for node in path.nodes:
            if isinstance(node, TrackNode):
                used_tracks.add(node)
        for first, second in zip(path.nodes, path.nodes[1:]):
            if isinstance(first, TrackNode) and isinstance(second, TrackNode):
                used_connectors.add(frozenset((first, second)))

    for track in sorted(used_tracks, key=repr):
        start, end = track_segment(track, rows, cols)
        ax.plot(
            (start[0], end[0]),
            (start[1], end[1]),
            color=SUCCESS_COLOR,
            linewidth=ROUTE_LINE_WIDTH,
            alpha=ROUTE_ALPHA,
            solid_capstyle="round",
            zorder=5,
        )

    for connector in sorted(used_connectors, key=repr):
        first, second = tuple(connector)
        shared = adjacent_cobs(first, rows, cols) & adjacent_cobs(second, rows, cols)
        if len(shared) != 1:
            raise ValueError(f"route connector does not meet through exactly one COB: {connector}")
        shared_cob = next(iter(shared))
        start = cob_contact(first, shared_cob)
        end = cob_contact(second, shared_cob)
        ax.plot(
            (start[0], end[0]),
            (start[1], end[1]),
            color=SUCCESS_COLOR,
            linewidth=ROUTE_LINE_WIDTH,
            alpha=ROUTE_ALPHA,
            solid_capstyle="round",
            zorder=5,
        )


def draw_category(
    guides: list[GuidePath],
    paths: list[RoutedPath],
    category: str,
    rows: int,
    cols: int,
    output: Path,
) -> None:
    figure, ax = plt.subplots(figsize=(13.5, 9.5), dpi=300)
    draw_hardware(ax, rows, cols)
    guide_channel_count = draw_guides(ax, guides, rows, cols)
    draw_route_geometry(ax, paths, rows, cols)

    extent = COB_HALF + EXTERNAL_TRACK_LENGTH + 0.08
    ax.set_xlim(-extent, cols - 1 + extent)
    ax.set_ylim(-extent, rows - 1 + extent)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(
        f"{PLOT_LABELS[category]}: global guide + final route\n"
        f"guide pairs={len(guides)}, guide Channels={guide_channel_count}; "
        f"final paths={len(paths)}, final nets={len({path.net_id for path in paths})}",
        pad=10,
    )
    ax.legend(
        handles=[
            Line2D(
                [0], [0], color=GUIDE_COLOR, linewidth=GUIDE_LINE_WIDTH,
                linestyle=(0, (5.0, 2.5)), alpha=0.85, label="global guide (Channel)",
            ),
            Line2D(
                [0], [0], color=SUCCESS_COLOR, linewidth=1.4,
                alpha=ROUTE_ALPHA, label="final route (Track/TOB)",
            ),
        ],
        loc="upper right",
        framealpha=0.92,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout(pad=0.10)
    figure.savefig(output, dpi=300, facecolor="white")
    plt.close(figure)


def render(log: Path, rows: int, cols: int, output_dir: Path) -> None:
    guides = read_global_guides(log)
    validate_guides(guides, rows, cols)

    text = log.read_text(encoding="utf-8")
    if "route net=" in text:
        paths = read_final_paths(log)
        validate_paths(paths, rows, cols)
    else:
        paths = []

    guides_by_category: dict[str, list[GuidePath]] = defaultdict(list)
    paths_by_category: dict[str, list[RoutedPath]] = defaultdict(list)
    for guide in guides:
        guides_by_category[classify_net(guide.net_name)].append(guide)
    for path in paths:
        paths_by_category[classify_net(path.net_name, path.display)].append(path)

    output_dir.mkdir(parents=True, exist_ok=True)
    for category in PLOT_LABELS:
        output = output_dir / f"{category}_guide_and_routes.png"
        draw_category(
            guides_by_category[category],
            paths_by_category[category],
            category,
            rows,
            cols,
            output,
        )
        print(
            f"{category}: guides={len(guides_by_category[category])} "
            f"routes={len(paths_by_category[category])} -> {output}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True, help="V20 debug log")
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
