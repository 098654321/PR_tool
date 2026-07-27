#!/usr/bin/env python3
"""Draw the final SAT/ILP routes with physical COB-channel track geometry.

COB(row, col) is drawn as a square centered at (col, row).  A horizontal
TrackCoord(r, c, H, i) lies between COB(r, c-1) and COB(r, c); a vertical
TrackCoord(r, c, V, i) lies between COB(r-1, c) and COB(r, c).  Track index
changes the lane position inside the channel, rather than merely being shown
as metadata.
"""

from __future__ import annotations

import argparse
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "pr_tool-matplotlib"))
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_ROUTE_LOG_DIR = Path(
    "/Users/jiaheng/FDU_files/Tao_group/论文/papers/ASP-DAC27_SiIF/"
    "Projects/0714_v15/L25"
)
DEFAULT_FAILURE_LOG_DIR = Path(
    "/Users/jiaheng/FDU_files/Tao_group/论文/papers/ASP-DAC27_SiIF/"
    "Projects/0714_v15/4unroutable"
)
DEFAULT_CASES = (
    (DEFAULT_ROUTE_LOG_DIR / "case4.log", DEFAULT_FAILURE_LOG_DIR / "case4.log", 9, 12, False),
    (DEFAULT_ROUTE_LOG_DIR / "case14.log", DEFAULT_FAILURE_LOG_DIR / "case14.log", 9, 13, False),
    (DEFAULT_ROUTE_LOG_DIR / "case15.log", DEFAULT_FAILURE_LOG_DIR / "case15.log", 9, 13, True),
    (DEFAULT_ROUTE_LOG_DIR / "case9.log", DEFAULT_FAILURE_LOG_DIR / "case9.log", 9, 13, False),
)

TRACK_COUNT = 128
COB_SIDE = 0.46
COB_HALF = COB_SIDE / 2
TRACK_EDGE_MARGIN = 0.012
EXTERNAL_TRACK_LENGTH = 0.46
TOB_WIDTH = 0.625
TOB_HEIGHT = 0.19
ROUTE_LINE_WIDTH = 0.6
FAILED_ROUTE_LINE_WIDTH = 0.9
SUCCESS_COLOR = "black"
TRACK_BUMP_FAILURE_COLOR = "red"
BUMP_BUMP_FAILURE_COLOR = "#4A90D9"
PN_FAILURE_COLOR = "green"

ROUTE_RE = re.compile(
    r'\broute net="(?P<net_name>[^"]*)" id=(?P<net_id>\d+) kind=(?:Bnet|Tnet|PNnet) '
    r'display=(?P<display>TwoPin|SyncBus|TrackToBumps|TracksToBumps) '
    r'demands=(?P<demands>\d+)\b'
)
DEMAND_RE = re.compile(r"\b(?:member\s+)?demand=(\d+)\b")
PATH_RE = re.compile(r"\bpath:\s*(.+)$")
TRACK_TOKEN_RE = re.compile(
    r"\{r(?P<row>-?\d+),\s*c(?P<col>-?\d+),\s*(?P<direction>[HV]),\s*i(?P<index>\d+)\}"
)
TOB_TOKEN_RE = re.compile(
    r"TOB\((?P<row>\d+),(?P<col>\d+)\) B\d+ (?:G\d+ [IJ]\d+|V\d+)"
)
ENDPOINT_RE = re.compile(r"\bsrc=.+\bsnk=.+$")
ENDPOINT_NODES_RE = re.compile(r"\bsrc=(?P<source>.+?)\s+snk=(?P<sink>.+)$")
TOB_BUMP_RE = re.compile(
    r"TOB\((?P<row>\d+),(?P<col>\d+)\) B(?P<bank>\d+) "
    r"G(?P<group>\d+) I(?P<index>\d+)"
)
WIRELENGTH_RE = re.compile(r"\bnet_wirelength=\d+\b")
FAILED_RE = re.compile(r"Routing failed for this net: (?P<net_name>.+)$")
LEGACY_BUMP_SOURCE_RE = re.compile(
    r"(?P<net_name>.+): Begin bump: '\{ row: (?P<row>\d+), col: (?P<col>\d+), "
    r"index: (?P<index>\d+) \}' to .+"
)
LEGACY_TRACK_SOURCE_RE = re.compile(
    r"(?P<net_name>.+): Begin track:? '\{ row: (?P<row>\d+), col: (?P<col>\d+), "
    r"dir: PR_tool::hardware::TrackDirection::(?P<direction>Horizontal|Vertical), "
    r"index: (?P<index>\d+) \}' to .+"
)

SourceKey = tuple[str, int, int, str, int]
FailureDescriptor = tuple[str, SourceKey | None]


@dataclass(frozen=True)
class TrackNode:
    row: int
    col: int
    direction: str
    index: int


@dataclass(frozen=True)
class TobNode:
    row: int
    col: int


PhysicalNode = TrackNode | TobNode


@dataclass(frozen=True)
class RoutedPath:
    net_id: int
    net_name: str
    display: str
    endpoint_keys: frozenset[SourceKey]
    demand_id: int
    nodes: tuple[PhysicalNode, ...]


@dataclass
class RouteBlock:
    net_id: int
    net_name: str
    display: str
    demands: int
    start_line: int
    paths: dict[int, tuple[PhysicalNode, ...]]
    endpoint_keys: set[SourceKey]
    current_demand: int = 0
    waiting_for_path: bool = False


def parse_path_nodes(path_text: str, line_number: int) -> tuple[PhysicalNode, ...]:
    nodes: list[PhysicalNode] = []
    tokens = path_text.split(" -> ")
    if not tokens or any(not token for token in tokens):
        raise ValueError(f"line {line_number}: path has an empty hop")

    for token in tokens:
        track_match = TRACK_TOKEN_RE.fullmatch(token)
        tob_match = TOB_TOKEN_RE.fullmatch(token)
        if track_match:
            node: PhysicalNode = TrackNode(
                row=int(track_match.group("row")),
                col=int(track_match.group("col")),
                direction=track_match.group("direction"),
                index=int(track_match.group("index")),
            )
        elif tob_match:
            node = TobNode(int(tob_match.group("row")), int(tob_match.group("col")))
        else:
            raise ValueError(f"line {line_number}: unsupported path hop: {token!r}")
        # Bump/HLine/VLine log tokens all map to the same TOB body.
        if not nodes or node != nodes[-1]:
            nodes.append(node)
    return tuple(nodes)


def parse_endpoint_node(node_text: str, line_number: int) -> SourceKey:
    track_match = TRACK_TOKEN_RE.fullmatch(node_text)
    if track_match:
        return (
            "track",
            int(track_match.group("row")),
            int(track_match.group("col")),
            track_match.group("direction"),
            int(track_match.group("index")),
        )

    bump_match = TOB_BUMP_RE.fullmatch(node_text)
    if bump_match:
        tob_row = int(bump_match.group("row"))
        tob_col = int(bump_match.group("col"))
        bump_index = (
            int(bump_match.group("bank")) * 64
            + int(bump_match.group("group")) * 8
            + int(bump_match.group("index"))
        )
        return ("bump", 1 + 2 * tob_row, 3 * tob_col, "", bump_index)

    raise ValueError(f"line {line_number}: unsupported endpoint node: {node_text!r}")


def parse_endpoint_nodes(line: str, line_number: int) -> set[SourceKey]:
    endpoint_match = ENDPOINT_NODES_RE.search(line)
    if not endpoint_match:
        raise ValueError(f"line {line_number}: endpoint does not contain parseable src/snk nodes")
    return {
        parse_endpoint_node(endpoint_match.group("source"), line_number),
        parse_endpoint_node(endpoint_match.group("sink"), line_number),
    }


def read_failure_descriptors(
    log_path: Path,
    require_markers: bool = True,
) -> set[FailureDescriptor]:
    if not log_path.is_file():
        raise ValueError(f"failure log does not exist or is not a regular file: {log_path}")

    failures: set[FailureDescriptor] = set()
    previous_message = ""
    for line_number, line in enumerate(log_path.read_text(encoding="utf-8").splitlines(), 1):
        message = line.split(" > ", 1)[-1]
        failure_match = FAILED_RE.fullmatch(message)
        if failure_match:
            net_name = failure_match.group("net_name")
            bump_match = LEGACY_BUMP_SOURCE_RE.fullmatch(previous_message)
            track_match = LEGACY_TRACK_SOURCE_RE.fullmatch(previous_message)
            if bump_match and bump_match.group("net_name") == net_name:
                source_key: SourceKey | None = (
                    "bump",
                    int(bump_match.group("row")),
                    int(bump_match.group("col")),
                    "",
                    int(bump_match.group("index")),
                )
            elif track_match and track_match.group("net_name") == net_name:
                source_key = (
                    "track",
                    int(track_match.group("row")),
                    int(track_match.group("col")),
                    "H" if track_match.group("direction") == "Horizontal" else "V",
                    int(track_match.group("index")),
                )
            elif previous_message.startswith(f"{net_name}: Begin tracks "):
                source_key = None
            else:
                raise ValueError(
                    f"line {line_number}: failure marker has no matching net description "
                    f"on the preceding line: {net_name!r}"
                )
            failures.add((net_name, source_key))
        previous_message = message

    if require_markers and not failures:
        raise ValueError(f"no 'Routing failed for this net' markers found in {log_path}")
    return failures


def read_final_paths(log_path: Path) -> list[RoutedPath]:
    """Validate all route blocks and retain the final complete block per net."""
    if not log_path.is_file():
        raise ValueError(f"log file does not exist or is not a regular file: {log_path}")

    latest_blocks: dict[int, RouteBlock] = {}
    block: RouteBlock | None = None

    for line_number, line in enumerate(log_path.read_text(encoding="utf-8").splitlines(), 1):
        route_match = ROUTE_RE.search(line)
        if route_match:
            if block is not None:
                raise ValueError(
                    f"line {line_number}: new route block starts before net {block.net_id} "
                    f"is closed by net_wirelength="
                )
            demands = int(route_match.group("demands"))
            if demands <= 0:
                raise ValueError(f"line {line_number}: route block must declare demands > 0")
            block = RouteBlock(
                net_id=int(route_match.group("net_id")),
                net_name=route_match.group("net_name"),
                display=route_match.group("display"),
                demands=demands,
                start_line=line_number,
                paths={},
                endpoint_keys=set(),
            )
            continue
        if "route net=" in line:
            raise ValueError(f"line {line_number}: malformed route header")
        if block is None:
            if "path:" in line or WIRELENGTH_RE.search(line):
                raise ValueError(f"line {line_number}: route data appears outside a route block")
            continue

        demand_match = DEMAND_RE.search(line)
        if demand_match:
            if block.waiting_for_path:
                raise ValueError(f"line {line_number}: previous endpoint has no following path")
            demand_id = int(demand_match.group(1))
            if not (0 <= demand_id < block.demands):
                raise ValueError(
                    f"line {line_number}: demand {demand_id} is outside [0, {block.demands - 1}]"
                )
            if demand_id in block.paths:
                raise ValueError(f"line {line_number}: duplicate demand {demand_id} in one route block")
            if not ENDPOINT_RE.search(line):
                raise ValueError(f"line {line_number}: demand line must contain src= and snk=")
            block.endpoint_keys.update(parse_endpoint_nodes(line, line_number))
            block.current_demand = demand_id
            block.waiting_for_path = True
            continue

        if ENDPOINT_RE.search(line):
            if block.demands != 1:
                raise ValueError(f"line {line_number}: multi-demand endpoint is missing demand=<id>")
            if block.waiting_for_path or 0 in block.paths:
                raise ValueError(f"line {line_number}: duplicate endpoint for demand 0")
            block.endpoint_keys.update(parse_endpoint_nodes(line, line_number))
            block.current_demand = 0
            block.waiting_for_path = True
            continue

        path_match = PATH_RE.search(line)
        if path_match:
            if not block.waiting_for_path:
                raise ValueError(f"line {line_number}: path has no preceding src=/snk= endpoint")
            nodes = parse_path_nodes(path_match.group(1), line_number)
            if not nodes:
                raise ValueError(f"line {line_number}: path contains no physical nodes")
            block.paths[block.current_demand] = nodes
            block.waiting_for_path = False
            continue

        if WIRELENGTH_RE.search(line):
            if block.waiting_for_path:
                raise ValueError(f"line {line_number}: endpoint has no following path")
            expected = set(range(block.demands))
            if set(block.paths) != expected:
                missing = sorted(expected - set(block.paths))
                raise ValueError(
                    f"route block at line {block.start_line} is incomplete; missing demands {missing}"
                )
            latest_blocks[block.net_id] = block
            block = None

    if block is not None:
        raise ValueError(
            f"route block at line {block.start_line} is not closed by net_wirelength="
        )
    if not latest_blocks:
        raise ValueError(f"no complete route blocks found in {log_path}")

    paths: list[RoutedPath] = []
    for net_id in sorted(latest_blocks):
        final_block = latest_blocks[net_id]
        if not final_block.endpoint_keys:
            raise ValueError(
                f"route block at line {final_block.start_line} has no endpoint nodes"
            )
        for demand_id in sorted(final_block.paths):
            paths.append(
                RoutedPath(
                    net_id,
                    final_block.net_name,
                    final_block.display,
                    frozenset(final_block.endpoint_keys),
                    demand_id,
                    final_block.paths[demand_id],
                )
            )
    return paths


def failure_color_for_net(net_name: str) -> str:
    if net_name.startswith(("TrackToBumpsNet_", "TrackToBumpNet_", "BumpToTrackNet_")):
        return TRACK_BUMP_FAILURE_COLOR
    if net_name.startswith("BumpToBumpNet_"):
        return BUMP_BUMP_FAILURE_COLOR
    if net_name.startswith(("Nege", "Pose")):
        return PN_FAILURE_COLOR
    raise ValueError(f"failed net has an unsupported type: {net_name!r}")


def classify_failed_nets(
    paths: list[RoutedPath],
    failure_log: Path | None,
    fail_non_sync: bool,
) -> dict[int, str]:
    descriptors = (
        read_failure_descriptors(failure_log, require_markers=not fail_non_sync)
        if failure_log is not None
        else set()
    )
    matched_descriptors: set[FailureDescriptor] = set()
    failed_net_names: dict[int, str] = {}

    for path in paths:
        failed = fail_non_sync and path.display != "SyncBus"
        whole_net_descriptor = (path.net_name, None)
        for endpoint_key in path.endpoint_keys:
            exact_descriptor = (path.net_name, endpoint_key)
            if exact_descriptor in descriptors:
                matched_descriptors.add(exact_descriptor)
                failed = True
        if whole_net_descriptor in descriptors:
            matched_descriptors.add(whole_net_descriptor)
            failed = True
        if failed:
            failed_net_names[path.net_id] = path.net_name

    unmatched = descriptors - matched_descriptors
    if unmatched:
        sample = ", ".join(repr(item) for item in sorted(unmatched, key=repr)[:3])
        raise ValueError(
            f"{len(unmatched)} failed routes in {failure_log} do not match the final path log; "
            f"first unmatched entries: {sample}"
        )
    return {
        net_id: failure_color_for_net(net_name)
        for net_id, net_name in failed_net_names.items()
    }


def adjacent_cobs(track: TrackNode, rows: int, cols: int) -> set[tuple[int, int]]:
    if track.direction == "H":
        candidates = {(track.row, track.col - 1), (track.row, track.col)}
    else:
        candidates = {(track.row - 1, track.col), (track.row, track.col)}
    return {(row, col) for row, col in candidates if 0 <= row < rows and 0 <= col < cols}


def validate_paths(paths: list[RoutedPath], rows: int, cols: int) -> None:
    for path in paths:
        for node in path.nodes:
            if isinstance(node, TrackNode):
                if node.direction == "H":
                    in_bounds = 0 <= node.row < rows and 0 <= node.col <= cols
                    expected = f"row in [0, {rows - 1}], col in [0, {cols}]"
                else:
                    in_bounds = 0 <= node.row <= rows and 0 <= node.col < cols
                    expected = f"row in [0, {rows}], col in [0, {cols - 1}]"
                if not in_bounds:
                    raise ValueError(
                        f"net {path.net_id} demand {path.demand_id}: {node.direction} track "
                        f"coordinate outside {rows}x{cols} array ({expected}): {node}"
                    )
                if not (0 <= node.index < TRACK_COUNT):
                    raise ValueError(
                        f"net {path.net_id} demand {path.demand_id}: "
                        f"track index outside [0, 127]: {node}"
                    )
            else:
                channel_row = 1 + 2 * node.row
                channel_col = 3 * node.col
                if not (
                    0 <= node.row < 4
                    and 0 <= node.col < 4
                    and channel_row < rows
                    and channel_col < cols
                ):
                    raise ValueError(
                        f"net {path.net_id} demand {path.demand_id}: TOB {node} "
                        f"does not fit inside the specified {rows}x{cols} COB array"
                    )

        for first, second in zip(path.nodes, path.nodes[1:]):
            if isinstance(first, TrackNode) and isinstance(second, TrackNode):
                shared = adjacent_cobs(first, rows, cols) & adjacent_cobs(second, rows, cols)
                if len(shared) != 1:
                    raise ValueError(
                        f"net {path.net_id} demand {path.demand_id}: consecutive tracks "
                        f"do not meet through exactly one COB: {first} -> {second}"
                    )
            elif isinstance(first, TobNode) != isinstance(second, TobNode):
                tob = first if isinstance(first, TobNode) else second
                track = second if isinstance(second, TrackNode) else first
                assert isinstance(tob, TobNode) and isinstance(track, TrackNode)
                expected = (1 + 2 * tob.row, 3 * tob.col)
                if track.direction != "V" or (track.row, track.col) != expected:
                    raise ValueError(
                        f"net {path.net_id} demand {path.demand_id}: TOB {tob} is not "
                        f"connected to its vertical channel track {track}"
                    )


def horizontal_lane(track: TrackNode) -> float:
    usable = COB_SIDE - 2 * TRACK_EDGE_MARGIN
    return track.row + COB_HALF - TRACK_EDGE_MARGIN - usable * track.index / (TRACK_COUNT - 1)


def vertical_lane(track: TrackNode) -> float:
    usable = COB_SIDE - 2 * TRACK_EDGE_MARGIN
    return track.col + COB_HALF - TRACK_EDGE_MARGIN - usable * track.index / (TRACK_COUNT - 1)


def track_segment(track: TrackNode, rows: int, cols: int) -> tuple[tuple[float, float], tuple[float, float]]:
    if track.direction == "H":
        y = horizontal_lane(track)
        if track.col == 0:
            return ((-COB_HALF - EXTERNAL_TRACK_LENGTH, y), (-COB_HALF, y))
        if track.col == cols:
            edge = cols - 1 + COB_HALF
            return ((edge, y), (edge + EXTERNAL_TRACK_LENGTH, y))
        return ((track.col - 1 + COB_HALF, y), (track.col - COB_HALF, y))

    x = vertical_lane(track)
    if track.row == 0:
        return ((x, -COB_HALF - EXTERNAL_TRACK_LENGTH), (x, -COB_HALF))
    if track.row == rows:
        edge = rows - 1 + COB_HALF
        return ((x, edge), (x, edge + EXTERNAL_TRACK_LENGTH))
    return ((x, track.row - 1 + COB_HALF), (x, track.row - COB_HALF))


def cob_contact(track: TrackNode, cob: tuple[int, int]) -> tuple[float, float]:
    row, col = cob
    if track.direction == "H":
        x = col + COB_HALF if col == track.col - 1 else col - COB_HALF
        return x, horizontal_lane(track)
    y = row + COB_HALF if row == track.row - 1 else row - COB_HALF
    return vertical_lane(track), y


def draw_line(
    ax: Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    color: str,
    line_width: float,
) -> None:
    ax.plot(
        (start[0], end[0]), (start[1], end[1]),
        color=color,
        linewidth=line_width,
        alpha=0.60 if color == SUCCESS_COLOR else 0.75,
        solid_capstyle="round", zorder=3,
    )


def resolve_resource_color(colors: set[str]) -> str:
    failure_colors = colors - {SUCCESS_COLOR}
    if len(failure_colors) > 1:
        raise ValueError(
            f"one physical routing resource is shared by multiple failure types: "
            f"{sorted(failure_colors)}"
        )
    return next(iter(failure_colors), SUCCESS_COLOR)


def draw_array(
    paths: list[RoutedPath],
    failed_net_colors: dict[int, str],
    rows: int,
    cols: int,
    output: Path,
    route_line_width: float = ROUTE_LINE_WIDTH,
) -> None:
    fig, ax = plt.subplots(figsize=(13.5, 9.5), dpi=300)

    for row in range(rows):
        for col in range(cols):
            ax.add_patch(
                Rectangle(
                    (col - COB_HALF, row - COB_HALF), COB_SIDE, COB_SIDE,
                    facecolor="#d6bd00", edgecolor="black", linewidth=0.70, zorder=1,
                )
            )

    # TOB(tr, tc) occupies vertical channel (r=1+2*tr, c=3*tc).
    for tob_row in range(4):
        for tob_col in range(4):
            channel_row = 1 + 2 * tob_row
            channel_col = 3 * tob_col
            if channel_row >= rows or channel_col >= cols:
                continue
            ax.add_patch(
                Rectangle(
                    (channel_col - TOB_WIDTH / 2, channel_row - 0.5 - TOB_HEIGHT / 2),
                    TOB_WIDTH, TOB_HEIGHT,
                    facecolor="#70a86b", edgecolor="black", linewidth=0.60, zorder=2,
                )
            )

    # Fanout paths repeat their shared tree trunks in the log.  Draw each
    # physical track/switch once so every visible wire keeps the same width.
    used_tracks: dict[TrackNode, set[str]] = {}
    used_connectors: dict[frozenset[TrackNode], set[str]] = {}
    for path in paths:
        color = failed_net_colors.get(path.net_id, SUCCESS_COLOR)
        for node in path.nodes:
            if isinstance(node, TrackNode):
                used_tracks.setdefault(node, set()).add(color)
        for first, second in zip(path.nodes, path.nodes[1:]):
            if isinstance(first, TrackNode) and isinstance(second, TrackNode):
                connector = frozenset((first, second))
                used_connectors.setdefault(connector, set()).add(color)

    for track, colors in used_tracks.items():
        start, end = track_segment(track, rows, cols)
        draw_line(ax, start, end, resolve_resource_color(colors), route_line_width)

    for connector, colors in used_connectors.items():
        first, second = tuple(connector)
        shared_cob = next(iter(adjacent_cobs(first, rows, cols) & adjacent_cobs(second, rows, cols)))
        draw_line(
            ax,
            cob_contact(first, shared_cob),
            cob_contact(second, shared_cob),
            resolve_resource_color(colors),
            route_line_width,
        )

    extent = COB_HALF + EXTERNAL_TRACK_LENGTH + 0.08
    ax.set_xlim(-extent, cols - 1 + extent)
    ax.set_ylim(-extent, rows - 1 + extent)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.tight_layout(pad=0.10)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=300, facecolor="white")
    plt.close(fig)


def render_log(
    log: Path,
    rows: int,
    cols: int,
    output: Path,
    failed_output: Path | None = None,
    failure_log: Path | None = None,
    fail_non_sync: bool = False,
) -> None:
    paths = read_final_paths(log)
    validate_paths(paths, rows, cols)
    failed_net_colors = classify_failed_nets(paths, failure_log, fail_non_sync)
    draw_array(paths, {}, rows, cols, output)
    print(
        f"rendered {len(paths)} paths from {len({path.net_id for path in paths})} "
        f"nets in black on a {rows}x{cols} COB array to {output}"
    )
    if failed_output is not None:
        failed_paths = [path for path in paths if path.net_id in failed_net_colors]
        draw_array(
            failed_paths,
            failed_net_colors,
            rows,
            cols,
            failed_output,
            FAILED_ROUTE_LINE_WIDTH,
        )
        print(
            f"rendered {len(failed_paths)} paths from {len(failed_net_colors)} failed nets "
            f"colored by type to {failed_output}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--log", type=Path,
        help="verbose routing log to parse (omit to render the four default cases)",
    )
    parser.add_argument("--row", type=int, help="number of COB rows")
    parser.add_argument("--col", type=int, help="number of COB columns")
    parser.add_argument(
        "--output", type=Path,
        help="output PNG (default: tools/vis_sat/<log-stem>_routes.png)",
    )
    args = parser.parse_args()

    if args.log is None:
        if args.row is not None or args.col is not None or args.output is not None:
            parser.error("--row, --col, and --output require --log")
        jobs = [
            (
                log,
                rows,
                cols,
                SCRIPT_DIR / f"{log.stem}_routes.png",
                SCRIPT_DIR / f"{log.stem}_failed_routes.png",
                failure_log,
                fail_non_sync,
            )
            for log, failure_log, rows, cols, fail_non_sync in DEFAULT_CASES
        ]
    else:
        if args.row is None or args.col is None:
            parser.error("--row and --col are required when --log is specified")
        if args.row <= 0 or args.col <= 0:
            parser.error("--row and --col must both be positive integers")
        jobs = [
            (
                args.log,
                args.row,
                args.col,
                args.output or SCRIPT_DIR / f"{args.log.stem}_routes.png",
                None,
                None,
                False,
            )
        ]

    try:
        for log, rows, cols, output, failed_output, failure_log, fail_non_sync in jobs:
            render_log(
                log,
                rows,
                cols,
                output,
                failed_output,
                failure_log,
                fail_non_sync,
            )
    except (OSError, UnicodeError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
