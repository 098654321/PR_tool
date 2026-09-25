#!/usr/bin/env python3
"""Render v24 route ILP results and iteration traces for the gap-M runs."""

from __future__ import annotations

import argparse
import re
from collections import defaultdict
from dataclasses import dataclass, replace
from pathlib import Path

import plot_routes as route_plot
import plot_unit_channel_load as load_plot

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle


CASES = (8, 11, 14)
MODES = ("gap-1", "gap-2", "gap-3", "gap-4")
ROWS, COLS = 9, 13
TRACK_TOKEN = re.compile(r"T U\d+ (?P<dir>[HV])\((?P<row>-?\d+),(?P<col>-?\d+)\) idx=(?P<idx>\d+)")
BUMP_TOKEN = re.compile(r"B T(?P<tob>\d+) B\d+ G\d+ I\d+")
TOB_TOKEN = re.compile(r"[BHV] T(?P<tob>\d+) (?:B\d+ .+|B\d+ G\d+ .+)")
MAP_RE = re.compile(
    r"net demand map: id=(\d+) name=(.*?) kind=(\S+) sync=(true|false) "
    r"demand=(\d+) sources=(.*?) sink=(.*)$"
)
PATH_RE = re.compile(
    r"(?:route net=|route ILP LP base route net=|route ILP pool update route net=)"
    r"(\d+) demand=(\d+) source=(\d+) path=(.*)$"
)
LP_RE = re.compile(r"route ILP LP round=(\d+)")
UPDATE_RE = re.compile(r"route ILP pool update: net=(\d+) demand=(\d+) added=(\d+)")
HOTSPOT_RE = re.compile(r"route ILP hotspot: most negative pi_e=([^ ]+) node=(.*)$")
TRACK_SEARCH_RE = re.compile(
    r"T U\d+ (?P<dir>[HV])\((?P<row>-?\d+),(?P<col>-?\d+)\) idx=(?P<idx>\d+)"
)
TRACK_TO_BUMPS_NAMES = (
    "TrackToBumpsNet_", "TrackToBumpNet_", "BumpToBumpNet_"
)
BASE_COLOR = "black"
ADDED_COLOR = "#d62728"
PRICE_COLOR = "#006400"


@dataclass(frozen=True)
class DemandMap:
    sources: frozenset[str]
    sink: str


@dataclass
class NetMap:
    name: str
    kind: str
    sync: bool
    demands: dict[int, DemandMap]


@dataclass(frozen=True)
class ParsedPath:
    net_id: int
    demand_id: int
    source_index: int
    source_token: str
    sink_token: str
    nodes: tuple


@dataclass
class Round:
    number: int
    base: list
    added: list
    updated_owners: set[tuple[int, int]]
    added_columns: int = 0
    pi_e: str | None = None
    hotspot: route_plot.TrackNode | None = None


def message(line: str) -> str:
    marker = "> "
    return line.split(marker, 1)[1].strip() if marker in line else ""


def parse_nodes(path_text: str, line_number: int) -> tuple[tuple, str, str]:
    tokens = path_text.split(" -> ")
    if not tokens or any(not item for item in tokens):
        raise ValueError(f"line {line_number}: empty route hop")
    nodes = []
    for token in tokens:
        track = TRACK_TOKEN.fullmatch(token)
        bump = BUMP_TOKEN.fullmatch(token)
        tob = TOB_TOKEN.fullmatch(token)
        if track:
            node = route_plot.TrackNode(
                int(track["row"]), int(track["col"]), track["dir"], int(track["idx"])
            )
        elif bump or tob:
            tob_id = int((bump or tob)["tob"])
            node = route_plot.TobNode(tob_id // 4, tob_id % 4)
        elif token.startswith("R_n "):
            continue
        else:
            raise ValueError(f"line {line_number}: unsupported route hop {token!r}")
        if not nodes or node != nodes[-1]:
            nodes.append(node)
    endpoint_tokens = [token for token in tokens if TRACK_TOKEN.fullmatch(token) or BUMP_TOKEN.fullmatch(token)]
    if not endpoint_tokens:
        raise ValueError(f"line {line_number}: route has no physical source/sink")
    return tuple(nodes), endpoint_tokens[0], endpoint_tokens[-1]


def parse_path(message_text: str, line_number: int) -> ParsedPath | None:
    match = PATH_RE.search(message_text)
    if not match:
        return None
    nodes, source, sink = parse_nodes(match[4], line_number)
    return ParsedPath(
        int(match[1]), int(match[2]), int(match[3]), source, sink, nodes
    )


def read_net_map(path: Path) -> list[NetMap]:
    by_id: dict[int, NetMap] = {}
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        match = MAP_RE.search(message(line))
        if not match:
            continue
        net_id, name, kind, sync, demand_id, sources, sink = match.groups()
        source_tokens = frozenset(item.split("=", 1)[1] for item in sources.split(";") if item)
        record = by_id.setdefault(
            int(net_id), NetMap(name, kind, sync == "true", {})
        )
        if (record.name, record.kind, record.sync) != (name, kind, sync == "true"):
            raise ValueError(f"{path}:{line_number}: inconsistent net metadata")
        record.demands[int(demand_id)] = DemandMap(source_tokens, sink)
    if not by_id:
        raise ValueError(f"no net demand map records found in {path}")
    return list(by_id.values())


def read_route_messages(log: Path, prefix: str) -> list[ParsedPath]:
    parsed = []
    for line_number, line in enumerate(log.read_text(encoding="utf-8").splitlines(), 1):
        msg = message(line)
        if msg.startswith(prefix):
            path = parse_path(msg, line_number)
            if path:
                parsed.append(path)
    return parsed


def match_net(path_records: list[ParsedPath], metadata: list[NetMap], log_id: int) -> NetMap:
    observed = [(item.source_token, item.sink_token) for item in path_records]
    matches = []
    for candidate in metadata:
        if len(candidate.demands) != len(observed):
            continue
        remaining = list(candidate.demands.values())
        valid = True
        for source, sink in observed:
            index = next((i for i, demand in enumerate(remaining)
                          if demand.sink == sink and source in demand.sources), None)
            if index is None:
                valid = False
                break
            remaining.pop(index)
        if valid and not remaining:
            matches.append(candidate)
    labels = {(item.name, item.kind, item.sync) for item in matches}
    if len(labels) != 1:
        raise ValueError(
            f"cannot uniquely classify log net {log_id}: {len(matches)} endpoint matches"
        )
    return matches[0]


def as_routed_path(parsed: ParsedPath, name: str, sync: bool) -> route_plot.RoutedPath:
    display = "SyncBus" if sync else (
        "TrackToBumps" if name.startswith("TrackToBumpsNet_") else "TwoPin"
    )
    return route_plot.RoutedPath(
        parsed.net_id, name, display, frozenset(), parsed.demand_id, parsed.nodes
    )


def read_final_paths(log: Path, metadata: list[NetMap]) -> tuple[list, bool, str]:
    paths = read_route_messages(log, "route net=")
    lines = log.read_text(encoding="utf-8").splitlines()
    complete = any("route ILP + RRR complete:" in message(line) for line in lines)
    status = next((message(line) for line in reversed(lines)
                  if "route ILP + RRR complete:" in message(line)
                  or "route ILP + RRR incomplete:" in message(line)), "no final route status")
    if not complete:
        return [], False, status
    by_id: dict[int, list[ParsedPath]] = defaultdict(list)
    for path in paths:
        by_id[path.net_id].append(path)
    if not by_id:
        raise ValueError(f"complete result in {log} has no final route records")
    output = []
    for net_id, records in sorted(by_id.items()):
        mapping = match_net(records, metadata, net_id)
        output.extend(as_routed_path(item, mapping.name, mapping.sync) for item in records)
    return output, True, status


def read_rounds(log: Path) -> list[Round]:
    rounds: list[Round] = []
    current: Round | None = None
    for line_number, line in enumerate(log.read_text(encoding="utf-8").splitlines(), 1):
        msg = message(line)
        lp = LP_RE.search(msg)
        if lp:
            current = Round(int(lp[1]), [], [], set())
            rounds.append(current)
            continue
        if current is None:
            continue
        update = UPDATE_RE.search(msg)
        if update:
            owner = (int(update[1]), int(update[2]))
            current.updated_owners.add(owner)
            current.added_columns += int(update[3])
            continue
        hot = HOTSPOT_RE.search(msg)
        if hot:
            current.pi_e = hot[1]
            track = TRACK_SEARCH_RE.search(hot[2])
            current.hotspot = route_plot.TrackNode(
                int(track["row"]), int(track["col"]), track["dir"], int(track["idx"])
            ) if track else None
            continue
        parsed = parse_path(msg, line_number)
        if parsed is None:
            continue
        routed = as_routed_path(parsed, f"net {parsed.net_id}", False)
        if msg.startswith("route ILP LP base route "):
            current.base.append(routed)
        elif msg.startswith("route ILP pool update route "):
            current.added.append(routed)
    return rounds


def draw_iteration(round_data: Round, output: Path) -> None:
    fig, ax = plt.subplots(figsize=(13.5, 9.5), dpi=150)
    for row in range(ROWS):
        for col in range(COLS):
            ax.add_patch(Rectangle(
                (col - route_plot.COB_HALF, row - route_plot.COB_HALF),
                route_plot.COB_SIDE, route_plot.COB_SIDE,
                facecolor="#d6bd00", edgecolor="black", linewidth=0.7, zorder=1,
            ))
    for tob_row in range(4):
        for tob_col in range(4):
            channel_row, channel_col = 1 + 2 * tob_row, 3 * tob_col
            if channel_row < ROWS and channel_col < COLS:
                ax.add_patch(Rectangle(
                    (channel_col - route_plot.TOB_WIDTH / 2,
                     channel_row - 0.5 - route_plot.TOB_HEIGHT / 2),
                    route_plot.TOB_WIDTH, route_plot.TOB_HEIGHT,
                    facecolor="#70a86b", edgecolor="black", linewidth=0.6, zorder=2,
                ))

    def draw_layer(paths: list, color: str, width: float, zorder: int) -> None:
        tracks = set()
        connectors = set()
        for path in paths:
            for node in path.nodes:
                if isinstance(node, route_plot.TrackNode):
                    tracks.add(node)
            for first, second in zip(path.nodes, path.nodes[1:]):
                if isinstance(first, route_plot.TrackNode) and isinstance(second, route_plot.TrackNode):
                    connectors.add(frozenset((first, second)))
        for track in sorted(tracks, key=lambda n: (n.row, n.col, n.direction, n.index)):
            route_plot.draw_line(ax, *route_plot.track_segment(track, ROWS, COLS), color, width)
            ax.lines[-1].set_zorder(zorder)
        for pair in sorted(connectors, key=lambda x: tuple(sorted(
                (n.row, n.col, n.direction, n.index) for n in x))):
            first, second = tuple(pair)
            shared = route_plot.adjacent_cobs(first, ROWS, COLS) & route_plot.adjacent_cobs(second, ROWS, COLS)
            if len(shared) != 1:
                continue
            cob = next(iter(shared))
            route_plot.draw_line(ax, route_plot.cob_contact(first, cob),
                                 route_plot.cob_contact(second, cob), color, width)
            ax.lines[-1].set_zorder(zorder)

    draw_layer(round_data.base, BASE_COLOR, 0.60, 3)
    draw_layer(round_data.added, ADDED_COLOR, 1.05, 4)
    if round_data.hotspot is not None:
        route_plot.draw_line(
            ax, *route_plot.track_segment(round_data.hotspot, ROWS, COLS), PRICE_COLOR, 3.0
        )
        ax.lines[-1].set_zorder(6)
    extent = route_plot.COB_HALF + route_plot.EXTERNAL_TRACK_LENGTH + 0.08
    ax.set_xlim(-extent, COLS - 1 + extent)
    ax.set_ylim(-extent, ROWS - 1 + extent)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.suptitle(
        f"LP round {round_data.number}: base paths={len(round_data.base)}, "
        f"added candidate paths={len(round_data.added)}, "
        f"updated owners={len(round_data.updated_owners)}, added columns={round_data.added_columns}, "
        f"pi_e={round_data.pi_e or 'not logged'}",
        fontsize=10,
    )
    fig.legend(handles=[
        Line2D([0], [0], color=BASE_COLOR, lw=1.1, label="LP base tree(s)"),
        Line2D([0], [0], color=ADDED_COLOR, lw=1.5, label="logged new candidate column(s)"),
        Line2D([0], [0], color=PRICE_COLOR, lw=2.5, label="most negative pi_e (largest dual price)"),
    ], loc="lower center", ncol=3, frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, 0.045, 1, 0.95))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, facecolor="white")
    plt.close(fig)


def write_placeholder(output: Path, title: str, status: str) -> None:
    fig, ax = plt.subplots(figsize=(13.5, 9.5), dpi=150)
    ax.axis("off")
    ax.text(0.5, 0.58, title, ha="center", va="center", fontsize=16, weight="bold")
    ax.text(0.5, 0.43, status, ha="center", va="center", fontsize=11, wrap=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, facecolor="white", bbox_inches="tight")
    plt.close(fig)


def render_final(paths: list, visual_dir: Path, status: str) -> None:
    groups = (
        ("all_nege_nets.png", lambda p: p.net_name == "Nege nets", "Nege nets"),
        ("all_pose_nets.png", lambda p: p.net_name == "Pose nets", "Pose nets"),
        ("all_track_to_bumps.png", lambda p: p.net_name.startswith(TRACK_TO_BUMPS_NAMES), "TrackToBump(s) and BumpToBump nets"),
        ("all_sync_nets.png", lambda p: p.display == "SyncBus", "Sync nets"),
    )
    for filename, predicate, label in groups:
        selected = [path for path in paths if predicate(path)]
        if selected:
            route_plot.render_path_group(selected, ROWS, COLS, visual_dir / filename, label)
        else:
            write_placeholder(visual_dir / filename, f"No {label} paths", "No matching final route records were logged.")
    loads_dir = visual_dir / "load-per-unit"
    loads = load_plot.collect_channel_loads(paths, ROWS, COLS)
    summaries = [load_plot.summarize(unit, item) for unit, item in enumerate(loads)]
    loads_dir.mkdir(parents=True, exist_ok=True)
    for unit, item in enumerate(loads):
        load_plot.draw_unit_load(
            item, summaries[unit], ROWS, COLS,
            loads_dir / f"unit_{unit:02d}_channel_load.png",
        )
    load_plot.write_summary(summaries, loads_dir / "unit_channel_load_summary.csv")
    (visual_dir / "RESULT_STATUS.txt").write_text(status + "\n", encoding="utf-8")


def process_case_mode(root: Path, case: int, mode: str) -> int:
    run_dir = root / f"case{case}" / mode
    log = run_dir / "debug.log"
    map_file = root / f"case{case}" / "net-demand-index.log"
    visual_dir = run_dir / "visualization"
    if not log.is_file() or not map_file.is_file():
        raise FileNotFoundError(f"required input missing: {log} or {map_file}")
    metadata = read_net_map(map_file)
    paths, complete, status = read_final_paths(log, metadata)
    if complete:
        route_plot.validate_paths(paths, ROWS, COLS)
        render_final(paths, visual_dir, status)
    else:
        for filename, label in (
            ("all_nege_nets.png", "Nege routes unavailable"),
            ("all_pose_nets.png", "Pose routes unavailable"),
            ("all_track_to_bumps.png", "TrackToBumps/TrackToBump/BumpToBump routes unavailable"),
            ("all_sync_nets.png", "Sync routes unavailable"),
        ):
            write_placeholder(visual_dir / filename, label, status)
        loads_dir = visual_dir / "load-per-unit"
        loads_dir.mkdir(parents=True, exist_ok=True)
        (loads_dir / "README.txt").write_text(
            "Skipped: the run has no complete final route set, so final Channel loads cannot be computed.\n",
            encoding="utf-8",
        )
        (visual_dir / "RESULT_STATUS.txt").write_text(
            status + "\nFinal route records are absent; the four route PNGs are status placeholders.\n",
            encoding="utf-8",
        )
    rounds = read_rounds(log)
    round_dir = visual_dir / "routes-per-round"
    for item in rounds:
        draw_iteration(item, round_dir / f"round_{item.number:03d}.png")
    (round_dir / "README.txt").write_text(
        "Black: LP base trees. Red: one logged full candidate column per updated owner.\n"
        "For SyncBus, each demand is an independent two-pin owner, so its logged candidate is one complete lane.\n"
        "Dark green: the resource with the most negative pi_e (largest dual price); the log reports a resource node, not an arc weight.\n",
        encoding="utf-8",
    )
    print(
        f"case{case}/{mode}: {'complete' if complete else 'incomplete'}, "
        f"final_paths={len(paths)}, LP_rounds={len(rounds)} -> {visual_dir}"
    )
    return len(rounds)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("Projects/v24-0925-M"))
    parser.add_argument("--case", type=int, choices=CASES, action="append")
    parser.add_argument("--mode", choices=MODES, action="append")
    args = parser.parse_args()
    total = 0
    for case in args.case or CASES:
        for mode in args.mode or MODES:
            total += process_case_mode(args.root, case, mode)
    print(f"rendered {total} LP iteration figures")


if __name__ == "__main__":
    main()
