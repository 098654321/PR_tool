"""TOB bump → COBUnit feasibility via mux occupancy.

bank0: 64 bumps, COBUnits 0–7, 8 per unit. Always feasible (Clos 1-factor).
full:  128 bumps, COBUnits 0–15, 8 per unit. Encodes the TOB mux chain as SAT
       (switch uniqueness, Bump–HLine / HLine–VLine matching, VLine–Track M_g)
       and solves with CaDiCaL. Residue-count violations are pruned first.

Every solved assignment claims mux ports, hori/vert wires, and tracks.
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import random
import subprocess
from dataclasses import dataclass, field


GROUP_SIZE = 8
N_GROUPS = 8
N_UNITS = 8
N_BUMPS = N_GROUPS * GROUP_SIZE

N_BH_MUX = 16
N_HV_MUX = 16
N_FULL_BUMPS = 128
N_FULL_UNITS = 16
N_TRACKS = 128
N_WIRES = 128
N_VT_MUX = 64


def cobunit_of_track(track: int) -> int:
    return track % 8 if track < 64 else track % 8 + 8


def random_assignment(rng: random.Random, n_units: int = N_UNITS) -> list[int]:
    units = [u for u in range(n_units) for _ in range(GROUP_SIZE)]
    rng.shuffle(units)
    return units


def _dfs_match(
    group: int,
    seen: set[int],
    match_unit: list[int | None],
    units_of_group: list[list[int]],
) -> bool:
    for unit in units_of_group[group]:
        if unit in seen:
            continue
        seen.add(unit)
        matched = match_unit[unit]
        if matched is None or _dfs_match(matched, seen, match_unit, units_of_group):
            match_unit[unit] = group
            return True
    return False


def _perfect_matching(
    remaining: list[list[tuple[int, int]]],
    rng: random.Random | None = None,
) -> list[int]:
    n_groups = len(remaining)
    units_of_group = [sorted({unit for unit, _ in edges}) for edges in remaining]
    if rng is not None:
        for units in units_of_group:
            rng.shuffle(units)
    match_unit: list[int | None] = [None] * n_groups
    for group in range(n_groups):
        if not _dfs_match(group, set(), match_unit, units_of_group):
            raise RuntimeError(f"no perfect matching at group {group}")

    chosen = [-1] * n_groups
    for unit, group in enumerate(match_unit):
        assert group is not None
        candidates = [i for i, (u, _) in enumerate(remaining[group]) if u == unit]
        i = candidates[0] if rng is None else rng.choice(candidates)
        _, bump = remaining[group].pop(i)
        chosen[group] = bump
    if any(b < 0 for b in chosen):
        raise RuntimeError("incomplete 1-factor")
    return chosen


def _claim(table: list[int | None], index: int, bump: int, label: str) -> None:
    prev = table[index]
    if prev is not None and prev != bump:
        raise RuntimeError(f"{label} reused by bump {prev} and bump {bump}")
    table[index] = bump


def _claim_grid(
    table: list[list[int | None]], mux: int, port: int, bump: int, label: str
) -> None:
    prev = table[mux][port]
    if prev is not None and prev != bump:
        raise RuntimeError(f"{label} mux {mux} port {port} reused by bump {prev} and bump {bump}")
    table[mux][port] = bump


@dataclass
class Path:
    bump: int
    unit: int
    o: int
    hori: int
    vert: int
    track: int
    bh_mux: int
    bh_in: int
    bh_out: int
    hv_mux: int
    hv_in: int
    hv_out: int
    vt_mux: int
    vt_in: int
    vt_out: int


def path_of(bump: int, unit: int, o: int) -> Path:
    """Unique TOB chain for (bump, track) with track determined by (unit, o)."""
    bh_mux = bump // GROUP_SIZE
    bh_in = bump % GROUP_SIZE
    r = unit % GROUP_SIZE
    tb = unit // GROUP_SIZE
    bb = bump // 64
    hori = 8 * bh_mux + o
    if bump < 64:
        hv_mux = hori % GROUP_SIZE
        hv_in = hori // GROUP_SIZE
    else:
        hv_mux = hori % GROUP_SIZE + GROUP_SIZE
        hv_in = (hori - 64) // GROUP_SIZE
    vert = 8 * hv_mux + r
    vt_mux = vert if bb == 0 else vert - 64
    track = vt_mux + 64 * tb
    return Path(
        bump=bump,
        unit=unit,
        o=o,
        hori=hori,
        vert=vert,
        track=track,
        bh_mux=bh_mux,
        bh_in=bh_in,
        bh_out=o,
        hv_mux=hv_mux,
        hv_in=hv_in,
        hv_out=r,
        vt_mux=vt_mux,
        vt_in=bb,
        vt_out=tb,
    )


@dataclass
class Occupancy:
    bump_to_unit: list[int]
    bump_to_hori: list[list[int]]
    hori_to_vert: list[list[int]]
    vt_choice: list[list[int]]
    path_hori: list[int]
    path_vert: list[int]
    path_track: list[int]
    bh_in: list[list[int | None]]
    bh_out: list[list[int | None]]
    hori_wire: list[int | None]
    hv_in: list[list[int | None]]
    hv_out: list[list[int | None]]
    vert_wire: list[int | None]
    vt_in: list[list[int | None]]
    vt_out: list[list[int | None]]
    track: list[int | None]


def empty_occupancy(bump_to_unit: list[int]) -> Occupancy:
    n = len(bump_to_unit)
    return Occupancy(
        bump_to_unit=bump_to_unit,
        bump_to_hori=[[-1] * GROUP_SIZE for _ in range(N_BH_MUX)],
        hori_to_vert=[[-1] * GROUP_SIZE for _ in range(N_HV_MUX)],
        vt_choice=[[-1, -1] for _ in range(N_VT_MUX)],
        path_hori=[-1] * n,
        path_vert=[-1] * n,
        path_track=[-1] * n,
        bh_in=[[None] * GROUP_SIZE for _ in range(N_BH_MUX)],
        bh_out=[[None] * GROUP_SIZE for _ in range(N_BH_MUX)],
        hori_wire=[None] * N_WIRES,
        hv_in=[[None] * GROUP_SIZE for _ in range(N_HV_MUX)],
        hv_out=[[None] * GROUP_SIZE for _ in range(N_HV_MUX)],
        vert_wire=[None] * N_WIRES,
        vt_in=[[None, None] for _ in range(N_VT_MUX)],
        vt_out=[[None, None] for _ in range(N_VT_MUX)],
        track=[None] * N_TRACKS,
    )


def occupy_path(occ: Occupancy, p: Path) -> None:
    occ.path_hori[p.bump] = p.hori
    occ.path_vert[p.bump] = p.vert
    occ.path_track[p.bump] = p.track
    occ.bump_to_hori[p.bh_mux][p.bh_in] = p.bh_out
    occ.hori_to_vert[p.hv_mux][p.hv_in] = p.hv_out
    occ.vt_choice[p.vt_mux][p.vt_in] = p.vt_out

    _claim_grid(occ.bh_in, p.bh_mux, p.bh_in, p.bump, "bump→hori input")
    _claim_grid(occ.bh_out, p.bh_mux, p.bh_out, p.bump, "bump→hori output")
    _claim(occ.hori_wire, p.hori, p.bump, "hori wire")
    _claim_grid(occ.hv_in, p.hv_mux, p.hv_in, p.bump, "hori→vert input")
    _claim_grid(occ.hv_out, p.hv_mux, p.hv_out, p.bump, "hori→vert output")
    _claim(occ.vert_wire, p.vert, p.bump, "vert wire")
    _claim_grid(occ.vt_in, p.vt_mux, p.vt_in, p.bump, "vert→track input")
    _claim_grid(occ.vt_out, p.vt_mux, p.vt_out, p.bump, "vert→track output")
    _claim(occ.track, p.track, p.bump, "track")

    if cobunit_of_track(p.track) != p.unit:
        raise RuntimeError(
            f"bump {p.bump} track {p.track} cobunit {cobunit_of_track(p.track)} != {p.unit}"
        )


def occupy_from_colors(bump_to_unit: list[int], colors: list[int]) -> Occupancy:
    occ = empty_occupancy(bump_to_unit)
    for bump, unit in enumerate(bump_to_unit):
        occupy_path(occ, path_of(bump, unit, colors[bump]))
    return occ


def color_and_occupy_bank0(bump_to_unit: list[int]) -> Occupancy:
    remaining: list[list[tuple[int, int]]] = [[] for _ in range(N_GROUPS)]
    for bump, unit in enumerate(bump_to_unit):
        remaining[bump // GROUP_SIZE].append((unit, bump))

    colors = [-1] * len(bump_to_unit)
    for color in range(GROUP_SIZE):
        for bump in _perfect_matching(remaining):
            colors[bump] = color
    return occupy_from_colors(bump_to_unit, colors)


def bank_residue_counts(bump_to_unit: list[int]) -> list[list[int]]:
    counts = [[0] * GROUP_SIZE for _ in range(2)]
    for bump, unit in enumerate(bump_to_unit):
        counts[bump // 64][unit % GROUP_SIZE] += 1
    return counts


def bank_residue_ok(counts: list[list[int]]) -> bool:
    return all(c == GROUP_SIZE for row in counts for c in row)


def _algorithm_x(
    col_to_rows: dict[str, set[int]],
    row_to_cols: list[list[str]],
    node_limit: int = 50_000,
) -> list[int] | None:
    """Knuth Algorithm X. Returns selected row ids, or None."""
    nodes = 0

    def select(row: int) -> list[tuple[str, set[int]]]:
        removed: list[tuple[str, set[int]]] = []
        for col in row_to_cols[row]:
            for other in list(col_to_rows[col]):
                if other == row:
                    continue
                for other_col in row_to_cols[other]:
                    if other_col != col:
                        col_to_rows[other_col].discard(other)
            removed.append((col, col_to_rows.pop(col)))
        return removed

    def deselect(row: int, removed: list[tuple[str, set[int]]]) -> None:
        for col, rows in reversed(removed):
            col_to_rows[col] = rows
            for other in rows:
                if other == row:
                    continue
                for other_col in row_to_cols[other]:
                    if other_col != col:
                        col_to_rows[other_col].add(other)

    solution: list[int] = []

    def search() -> bool:
        nonlocal nodes
        nodes += 1
        if nodes > node_limit:
            return False
        if not col_to_rows:
            return True
        col = min(col_to_rows, key=lambda k: len(col_to_rows[k]))
        if not col_to_rows[col]:
            return False
        for row in list(col_to_rows[col]):
            solution.append(row)
            removed = select(row)
            if search():
                return True
            deselect(row, removed)
            solution.pop()
        return False

    return solution if search() else None


def _one_factor_bank(
    bump_to_unit: list[int],
    bank: int,
    rng: random.Random | None = None,
) -> dict[int, int]:
    remaining: list[list[tuple[int, int]]] = [[] for _ in range(GROUP_SIZE)]
    base = bank * 64
    for bump in range(base, base + 64):
        residue = bump_to_unit[bump] % GROUP_SIZE
        group = (bump - base) // GROUP_SIZE
        remaining[group].append((residue, bump))
    if rng is not None:
        for edges in remaining:
            rng.shuffle(edges)
    colors: dict[int, int] = {}
    for _color in range(GROUP_SIZE):
        for bump in _perfect_matching(remaining, rng):
            colors[bump] = _color
    return colors


def _match_permutation(adj: list[list[int]]) -> list[int] | None:
    match_right: list[int | None] = [None] * GROUP_SIZE

    def dfs(left: int, seen: set[int]) -> bool:
        for right in adj[left]:
            if right in seen:
                continue
            seen.add(right)
            matched = match_right[right]
            if matched is None or dfs(matched, seen):
                match_right[right] = left
                return True
        return False

    for left in range(GROUP_SIZE):
        if not dfs(left, set()):
            return None
    perm = [0] * GROUP_SIZE
    for right, left in enumerate(match_right):
        assert left is not None
        perm[left] = right
    return perm


def solve_two_bank_colors(
    bump_to_unit: list[int],
    rng: random.Random | None = None,
) -> list[int] | None:
    """Color each bank by 1-factors, then relabel bank1 colors so 2×2 ports fit."""
    attempts = 1 if rng is None else 40
    for _ in range(attempts):
        c0 = _one_factor_bank(bump_to_unit, 0, rng)
        c1 = _one_factor_bank(bump_to_unit, 1, rng)
        bank0_tb = [[-1] * GROUP_SIZE for _ in range(GROUP_SIZE)]
        bank1_tb = [[-1] * GROUP_SIZE for _ in range(GROUP_SIZE)]
        for bump, unit in enumerate(bump_to_unit):
            residue = unit % GROUP_SIZE
            tb = unit // GROUP_SIZE
            if bump < 64:
                bank0_tb[residue][c0[bump]] = tb
            else:
                bank1_tb[residue][c1[bump]] = tb

        adj: list[list[int]] = [[] for _ in range(GROUP_SIZE)]
        for color_l in range(GROUP_SIZE):
            for color_r in range(GROUP_SIZE):
                if all(
                    bank0_tb[residue][color_l] != bank1_tb[residue][color_r]
                    for residue in range(GROUP_SIZE)
                ):
                    adj[color_l].append(color_r)
        perm = _match_permutation(adj)
        if perm is None:
            continue
        inv = [0] * GROUP_SIZE
        for color_l, color_r in enumerate(perm):
            inv[color_r] = color_l
        colors = [0] * len(bump_to_unit)
        for bump in range(64):
            colors[bump] = c0[bump]
        for bump in range(64, len(bump_to_unit)):
            colors[bump] = inv[c1[bump]]
        return colors
    return None


def solve_exact_cover_colors(bump_to_unit: list[int]) -> list[int] | None:
    """Exact cover fallback: each bump picks o∈[0,7] without mux/track reuse."""
    row_to_cols: list[list[str]] = []
    row_meta: list[tuple[int, int]] = []
    col_to_rows: dict[str, set[int]] = {}

    def add_row(bump: int, color: int) -> None:
        unit = bump_to_unit[bump]
        g = bump // GROUP_SIZE
        r = unit % GROUP_SIZE
        tb = unit // GROUP_SIZE
        bb = bump // 64
        hv_mux = color if bb == 0 else color + GROUP_SIZE
        vt_mux = color * GROUP_SIZE + r
        cols = [
            f"B{bump}",
            f"BH{g},{color}",
            f"HV{hv_mux},{r}",
            f"VIN{vt_mux},{bb}",
            f"VOUT{vt_mux},{tb}",
        ]
        rid = len(row_to_cols)
        row_to_cols.append(cols)
        row_meta.append((bump, color))
        for col in cols:
            col_to_rows.setdefault(col, set()).add(rid)

    for bump in range(len(bump_to_unit)):
        for color in range(GROUP_SIZE):
            add_row(bump, color)

    chosen = _algorithm_x(col_to_rows, row_to_cols)
    if chosen is None:
        return None
    colors = [-1] * len(bump_to_unit)
    for rid in chosen:
        bump, color = row_meta[rid]
        colors[bump] = color
    if any(c < 0 for c in colors):
        return None
    return colors


def cadical_assign_bin() -> pathlib.Path:
    root = pathlib.Path(__file__).resolve().parents[2]
    candidates = [
        root / "output" / "tob_sat_assign",
        pathlib.Path.cwd() / "output" / "tob_sat_assign",
    ]
    for path in candidates:
        if path.is_file() and os.access(path, os.X_OK):
            return path
    raise FileNotFoundError(
        "tob_sat_assign not found. From PR_tool run: xmake build tob_sat_assign"
    )


def solve_cadical_colors(bump_to_unit: list[int]) -> list[int] | None:
    proc = subprocess.run(
        [str(cadical_assign_bin()), *[str(u) for u in bump_to_unit]],
        check=False,
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        err = proc.stderr.strip() or proc.stdout.strip() or "tob_sat_assign failed"
        raise RuntimeError(err)
    lines = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
    if not lines or lines[0] == "UNSAT":
        return None
    if lines[0] != "SAT" or len(lines) < 2:
        raise RuntimeError(f"unexpected tob_sat_assign output:\n{proc.stdout}")
    colors = [int(tok) for tok in lines[1].split()]
    if len(colors) != N_FULL_BUMPS:
        raise RuntimeError(f"expected {N_FULL_BUMPS} colors, got {len(colors)}")
    return colors


def solve_full_colors(
    bump_to_unit: list[int],
    rng: random.Random | None = None,
) -> list[int] | None:
    del rng
    return solve_cadical_colors(bump_to_unit)


def solve_full(bump_to_unit: list[int], rng: random.Random | None = None) -> Occupancy | None:
    counts = bank_residue_counts(bump_to_unit)
    if not bank_residue_ok(counts):
        return None
    colors = solve_full_colors(bump_to_unit, rng)
    if colors is None:
        return None
    return occupy_from_colors(bump_to_unit, colors)


def _owners(row: list[int | None]) -> list[str]:
    return ["-" if b is None else str(b) for b in row]


def dump_occupancy(occ: Occupancy, trial: int, title: str) -> None:
    n_units = 1 + max(occ.bump_to_unit)
    print(f"=== {title} trial {trial} occupancy ===")
    print("paths: bump  hori  vert  track  unit")
    for bump, unit in enumerate(occ.bump_to_unit):
        print(
            f"  {bump:4d}  {occ.path_hori[bump]:4d}  {occ.path_vert[bump]:4d}"
            f"  {occ.path_track[bump]:5d}  {unit:4d}"
        )

    print("\nbump→hori 8×8  (mux input inner → output; each output lists owner bump)")
    for mux in range(N_BH_MUX):
        if all(v < 0 for v in occ.bump_to_hori[mux]):
            continue
        mapping = " ".join(
            f"{inner}->{occ.bump_to_hori[mux][inner]}" for inner in range(GROUP_SIZE)
        )
        print(f"  mux {mux}: {mapping}")
        print(f"           out owners: {_owners(occ.bh_out[mux])}")

    print("\nhori wires (index → bump):")
    print("  " + " ".join(f"{i}:{'-' if b is None else b}" for i, b in enumerate(occ.hori_wire)))

    print("\nhori→vert 8×8  (mux input inner → output residue; each output lists owner bump)")
    for mux in range(N_HV_MUX):
        if all(v < 0 for v in occ.hori_to_vert[mux]):
            continue
        mapping = " ".join(
            f"{inner}->{occ.hori_to_vert[mux][inner]}" for inner in range(GROUP_SIZE)
        )
        print(f"  mux {mux}: {mapping}")
        print(f"           out owners: {_owners(occ.hv_out[mux])}")

    print("\nvert wires (index → bump):")
    print("  " + " ".join(f"{i}:{'-' if b is None else b}" for i, b in enumerate(occ.vert_wire)))

    print("\nvert→track 2×2  (in0=bank0, in1=bank1; out0=unit 0-7, out1=unit 8-15)")
    for mux in range(N_VT_MUX):
        if occ.vt_in[mux][0] is None and occ.vt_in[mux][1] is None:
            continue
        in0 = occ.vt_in[mux][0]
        in1 = occ.vt_in[mux][1]
        c0 = occ.vt_choice[mux][0]
        c1 = occ.vt_choice[mux][1]
        print(
            f"  mux {mux:2d}: in0={'-' if in0 is None else f'b{in0}→out{c0}'}"
            f"  in1={'-' if in1 is None else f'b{in1}→out{c1}'}"
            f"  out owners={_owners(occ.vt_out[mux])}"
        )

    print("\ntracks (index → bump); unused marked -")
    print("  " + " ".join(f"{i}:{'-' if b is None else b}" for i, b in enumerate(occ.track)))
    used = [i for i, b in enumerate(occ.track) if b is not None]
    unused = [i for i, b in enumerate(occ.track) if b is None]
    print(f"  used {len(used)} tracks")
    print(f"  unused {len(unused)} tracks: {unused}")

    print("\nunit → tracks (and owner bumps)")
    for unit in range(n_units):
        pairs = [
            (occ.path_track[bump], bump)
            for bump, u in enumerate(occ.bump_to_unit)
            if u == unit
        ]
        pairs.sort()
        print(
            "  unit"
            f" {unit:2d}: "
            + ", ".join(f"t{track}←b{bump}" for track, bump in pairs)
        )
    print("=== occupancy unique: no mux port / wire / track shared by two bumps ===\n")


def dump_assignment(bump_to_unit: list[int]) -> None:
    n_groups = len(bump_to_unit) // GROUP_SIZE
    for group in range(n_groups):
        lo = group * GROUP_SIZE
        hi = lo + GROUP_SIZE
        print(f"  group {group:2d}: {bump_to_unit[lo:hi]}")


def dump_failure(bump_to_unit: list[int], err: Exception) -> None:
    print("FAILED")
    print(f"error: {err}")
    print("bump_to_unit:")
    dump_assignment(bump_to_unit)


def run_bank0_trials(n_trials: int, seed: int, dump_trial: int | None, dump_all: bool) -> int:
    rng = random.Random(seed)
    for trial in range(n_trials):
        assignment = random_assignment(rng, N_UNITS)
        try:
            occ = color_and_occupy_bank0(assignment)
        except Exception as err:
            print(f"[bank0] trial {trial} seed={seed}")
            dump_failure(assignment, err)
            return 1
        if dump_all or trial == dump_trial:
            dump_occupancy(occ, trial, "bank0")
    print(
        f"[bank0] passed {n_trials}/{n_trials} trials "
        f"(seed={seed}, units=0-7); all resources unique"
    )
    return 0


def sequential_full_assignment() -> list[int]:
    return [bump // GROUP_SIZE for bump in range(N_FULL_BUMPS)]


def random_assignment_count_ok(rng: random.Random) -> list[int]:
    """Uniform random 8-per-unit map with 8 bumps of each residue in each bank.

    Residue placement is a uniform shuffle of eight copies of 0..7 per bank.
    For each residue r, k of the eight bank-0 bumps take unit r (the rest take
    r+8), and k of the eight bank-1 bumps take unit r+8. k is sampled with
    weight C(8,k)^2 so each legal (count-ok, 8-per-unit) assignment is equally
    likely. The previous randint(0,8) oversampled the extreme splits k=0 and k=8.
    """
    residues = [0] * N_FULL_BUMPS
    for bank in range(2):
        bank_res = [r for r in range(GROUP_SIZE) for _ in range(GROUP_SIZE)]
        rng.shuffle(bank_res)
        base = bank * 64
        residues[base : base + 64] = bank_res

    assignment = [0] * N_FULL_BUMPS
    k_weights = [math.comb(GROUP_SIZE, k) ** 2 for k in range(GROUP_SIZE + 1)]
    for r in range(GROUP_SIZE):
        bank0 = [b for b in range(64) if residues[b] == r]
        bank1 = [b for b in range(64, N_FULL_BUMPS) if residues[b] == r]
        n_low = rng.choices(range(GROUP_SIZE + 1), weights=k_weights, k=1)[0]
        low0 = set(rng.sample(bank0, n_low))
        high1 = set(rng.sample(bank1, n_low))
        for bump in bank0:
            assignment[bump] = r if bump in low0 else r + GROUP_SIZE
        for bump in bank1:
            assignment[bump] = r + GROUP_SIZE if bump in high1 else r
    return assignment


def _run_full_batch(
    name: str,
    assignments: list[list[int]],
    seed: int,
    dump_trial: int | None,
    dump_all: bool,
) -> tuple[int, int, int, Occupancy | None]:
    feasible = 0
    fail_count = 0
    fail_mux = 0
    first_ok: Occupancy | None = None
    n_assign = len(assignments)
    for trial, assignment in enumerate(assignments):
        counts = bank_residue_counts(assignment)
        if not bank_residue_ok(counts):
            fail_count += 1
            if dump_all:
                print(f"[full {name}] trial {trial} infeasible: residue {counts}")
        else:
            occ = solve_full(assignment, random.Random(seed + trial + 11))
            if occ is None:
                fail_mux += 1
                if fail_mux == 1 or dump_all:
                    print(f"[full {name}] trial {trial} infeasible: CaDiCaL UNSAT")
                    print("bump_to_unit:")
                    dump_assignment(assignment)
            else:
                feasible += 1
                if first_ok is None:
                    first_ok = occ
                if dump_all or trial == dump_trial:
                    dump_occupancy(occ, trial, f"full {name}")
        if n_assign >= 1000 and (trial + 1) % 1000 == 0:
            print(
                f"[full {name}] progress {trial + 1}/{n_assign}: "
                f"feasible {feasible}, infeasible-by-count {fail_count}, "
                f"infeasible-by-mux {fail_mux}",
                flush=True,
            )
    return feasible, fail_count, fail_mux, first_ok


def run_full_trials(n_trials: int, seed: int, dump_trial: int | None, dump_all: bool) -> int:
    sanity = sequential_full_assignment()
    occ = solve_full(sanity)
    if occ is None:
        print("[full] sequential unit=bump//8 should be feasible but solver failed")
        return 1
    print("[full] sanity: unit=bump//8 is feasible")
    if dump_trial is not None or dump_all:
        dump_occupancy(occ, 0, "full sanity unit=bump//8")

    for k in range(GROUP_SIZE):
        wrapped = [((bump + k) % N_FULL_BUMPS) // GROUP_SIZE for bump in range(N_FULL_BUMPS)]
        if solve_full(wrapped) is None:
            print(f"[full] wrap k={k} should be feasible but CaDiCaL returned UNSAT")
            return 1
        rotated = [(bump // GROUP_SIZE + k) % N_FULL_UNITS for bump in range(N_FULL_BUMPS)]
        if solve_full(rotated) is None:
            print(f"[full] rotate k={k} should be feasible but CaDiCaL returned UNSAT")
            return 1
    print("[full] sanity: wrap and rotate k=0..7 are feasible")

    unconstrained = [
        random_assignment(random.Random(seed * 1_000_003 + trial), N_FULL_UNITS)
        for trial in range(n_trials)
    ]
    fea, fail_c, fail_m, first_ok = _run_full_batch(
        "unconstrained", unconstrained, seed, dump_trial, dump_all
    )
    print(
        f"[full unconstrained] seed={seed} trials={n_trials}: "
        f"feasible {fea}/{n_trials}, "
        f"infeasible-by-count {fail_c}/{n_trials}, "
        f"infeasible-by-mux {fail_m}/{n_trials}"
    )
    if fea == 0 and dump_trial is not None and not dump_all:
        print(
            "[full unconstrained] no feasible random 8-per-unit assignment; "
            f"trial 0 per-bank residue counts={bank_residue_counts(unconstrained[0])}"
        )

    constrained = [
        random_assignment_count_ok(random.Random(seed * 1_000_003 + 99 + trial))
        for trial in range(n_trials)
    ]
    fea2, fail_c2, fail_m2, first_ok2 = _run_full_batch(
        "count-ok", constrained, seed + 99, dump_trial if dump_all else None, dump_all
    )
    print(
        f"[full count-ok] seed={seed} trials={n_trials}: "
        f"feasible {fea2}/{n_trials}, "
        f"infeasible-by-count {fail_c2}/{n_trials}, "
        f"infeasible-by-mux {fail_m2}/{n_trials}"
    )
    if dump_trial is not None and not dump_all and first_ok is None and first_ok2 is not None:
        dump_occupancy(first_ok2, 0, "full count-ok first-feasible")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--mode",
        choices=("bank0", "full", "both"),
        default="both",
        help="bank0 = 64 bumps / units 0-7; full = 128 bumps / units 0-15",
    )
    parser.add_argument(
        "--dump-trial",
        type=int,
        default=0,
        help="print occupancy tables for this trial index; negative to skip",
    )
    parser.add_argument(
        "--dump-all",
        action="store_true",
        help="print occupancy tables for every trial",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dump_trial = None if args.dump_trial < 0 else args.dump_trial
    rc = 0
    if args.mode in ("bank0", "both"):
        rc |= run_bank0_trials(args.trials, args.seed, dump_trial, args.dump_all)
    if args.mode in ("full", "both"):
        rc |= run_full_trials(args.trials, args.seed, dump_trial, args.dump_all)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
