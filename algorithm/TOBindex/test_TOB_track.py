"""TOB bump → track feasibility via SAT.

First draw a count-ok bump→COBUnit map (each unit has 8 bumps; every bank has
exactly 8 bumps of each residue r, i.e. unit r plus unit r+8). Then assign each
unit's 8 bumps a random permutation of that unit's 8 tracks. CaDiCaL tests
whether the TOB mux chain can realize the pinned bump→track pairs.

Path (bump, track) is unique, so SAT pins each bump's color and checks switch
uniqueness, Bump–HLine / HLine–VLine matching, and VLine–Track M_g.

Run from PR_tool:
  python3 algorithm/TOBindex/test_TOB_track.py --trials 1000 --dump-trial -1
"""

from __future__ import annotations

import argparse
import random
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import test_TOB_index as tob

N = tob.N_FULL_BUMPS
N_UNITS = tob.N_FULL_UNITS
GROUP = tob.GROUP_SIZE


def tracks_of_unit(unit: int) -> list[int]:
    residue = unit % GROUP
    bank = unit // GROUP
    return [64 * bank + 8 * k + residue for k in range(GROUP)]


def color_of_track(track: int) -> int:
    return (track % 64) // GROUP


def bump_to_unit_from_tracks(bump_to_track: list[int]) -> list[int]:
    return [tob.cobunit_of_track(track) for track in bump_to_track]


def colors_of_tracks(bump_to_track: list[int]) -> list[int]:
    return [color_of_track(track) for track in bump_to_track]


def validate_track_assignment(bump_to_unit: list[int], bump_to_track: list[int]) -> None:
    if len(bump_to_track) != N:
        raise RuntimeError(f"expected {N} tracks, got {len(bump_to_track)}")
    seen: set[int] = set()
    per_unit = [0] * N_UNITS
    for bump, (unit, track) in enumerate(zip(bump_to_unit, bump_to_track, strict=True)):
        if track < 0 or track >= N:
            raise RuntimeError(f"bump {bump} track {track} out of range")
        if track in seen:
            raise RuntimeError(f"track {track} assigned twice")
        seen.add(track)
        if tob.cobunit_of_track(track) != unit:
            raise RuntimeError(
                f"bump {bump} unit {unit} got track {track} "
                f"of unit {tob.cobunit_of_track(track)}"
            )
        per_unit[unit] += 1
    if any(count > GROUP for count in per_unit):
        raise RuntimeError(f"a COBUnit has more than {GROUP} bumps: {per_unit}")
    counts = tob.bank_residue_counts(bump_to_unit)
    if not tob.bank_residue_ok(counts):
        raise RuntimeError(f"residue counts are not all 8: {counts}")


def random_tracks_for_units(rng: random.Random, bump_to_unit: list[int]) -> list[int]:
    by_unit: list[list[int]] = [[] for _ in range(N_UNITS)]
    for bump, unit in enumerate(bump_to_unit):
        by_unit[unit].append(bump)
    bump_to_track = [-1] * N
    for unit, bumps in enumerate(by_unit):
        if len(bumps) > GROUP:
            raise RuntimeError(f"unit {unit} has {len(bumps)} bumps")
        tracks = tracks_of_unit(unit)
        rng.shuffle(tracks)
        chosen = tracks[: len(bumps)]
        rng.shuffle(bumps)
        for bump, track in zip(bumps, chosen, strict=True):
            bump_to_track[bump] = track
    if any(track < 0 for track in bump_to_track):
        raise RuntimeError("some bump has no track")
    validate_track_assignment(bump_to_unit, bump_to_track)
    return bump_to_track


def occupy_tracks(bump_to_track: list[int]) -> tob.Occupancy:
    units = bump_to_unit_from_tracks(bump_to_track)
    colors = colors_of_tracks(bump_to_track)
    occ = tob.occupy_from_colors(units, colors)
    for bump, track in enumerate(bump_to_track):
        if occ.path_track[bump] != track:
            raise RuntimeError(
                f"occupancy track {occ.path_track[bump]} != pinned track {track} "
                f"for bump {bump}"
            )
    return occ


def solve_cadical_tracks(bump_to_track: list[int]) -> list[int] | None:
    proc = subprocess.run(
        [str(tob.cadical_assign_bin()), "--tracks", *[str(t) for t in bump_to_track]],
        check=False,
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        err = proc.stderr.strip() or proc.stdout.strip() or "tob_sat_assign --tracks failed"
        raise RuntimeError(err)
    lines = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
    if not lines or lines[0] == "UNSAT":
        return None
    if lines[0] != "SAT" or len(lines) < 2:
        raise RuntimeError(f"unexpected tob_sat_assign --tracks output:\n{proc.stdout}")
    colors = [int(tok) for tok in lines[1].split()]
    if colors != colors_of_tracks(bump_to_track):
        raise RuntimeError("SAT colors do not match the pinned tracks")
    return colors


def solve_tracks(bump_to_track: list[int]) -> tob.Occupancy | None:
    sat_colors = solve_cadical_tracks(bump_to_track)
    try:
        occ = occupy_tracks(bump_to_track)
    except RuntimeError:
        occ = None
    if sat_colors is None:
        if occ is not None:
            raise RuntimeError("CaDiCaL UNSAT but occupancy accepted the pinned tracks")
        return None
    if occ is None:
        raise RuntimeError("CaDiCaL SAT but occupancy rejected the pinned tracks")
    return occ


def dump_tracks(bump_to_unit: list[int], bump_to_track: list[int]) -> None:
    for unit in range(N_UNITS):
        pairs = [
            (bump, bump_to_track[bump])
            for bump, u in enumerate(bump_to_unit)
            if u == unit
        ]
        pairs.sort()
        print(
            f"  unit {unit:2d}: "
            + ", ".join(f"b{bump}→t{track}" for bump, track in pairs)
        )


def sequential_pinned_tracks() -> list[int]:
    units = tob.sequential_full_assignment()
    return [tob.path_of(bump, units[bump], bump % GROUP).track for bump in range(N)]


def frozen_unit_sat_tracks() -> list[int]:
    units = tob.sequential_full_assignment()
    colors = tob.solve_cadical_colors(units)
    if colors is None:
        raise RuntimeError("sequential unit assignment should be SAT")
    return [tob.path_of(bump, units[bump], colors[bump]).track for bump in range(N)]


def run_trials(n_trials: int, seed: int, dump_trial: int | None, dump_all: bool) -> int:
    frozen = frozen_unit_sat_tracks()
    if solve_tracks(frozen) is None:
        print("[track] pinning a SAT unit-coloring should stay SAT")
        return 1
    print("[track] sanity: pinned SAT unit-coloring remains SAT")

    sequential = sequential_pinned_tracks()
    seq_occ = solve_tracks(sequential)
    print(
        "[track] sanity: sequential unit=bump//8, color=bump%8 is "
        + ("SAT" if seq_occ is not None else "UNSAT")
    )
    if dump_trial is not None or dump_all:
        if seq_occ is not None:
            tob.dump_occupancy(seq_occ, 0, "track sequential")
        else:
            dump_tracks(bump_to_unit_from_tracks(sequential), sequential)

    feasible = 0
    unsat = 0
    first_ok: tob.Occupancy | None = None
    first_unsat: tuple[list[int], list[int]] | None = None
    for trial in range(n_trials):
        rng = random.Random(seed * 1_000_003 + trial)
        units = tob.random_assignment_count_ok(rng)
        tracks = random_tracks_for_units(rng, units)
        occ = solve_tracks(tracks)
        if occ is None:
            unsat += 1
            if first_unsat is None:
                first_unsat = (units, tracks)
                print(f"[track] trial {trial} CaDiCaL UNSAT")
                dump_tracks(units, tracks)
        else:
            feasible += 1
            if first_ok is None:
                first_ok = occ
            if dump_all or trial == dump_trial:
                tob.dump_occupancy(occ, trial, "track random")
        if n_trials >= 1000 and (trial + 1) % 100 == 0:
            print(
                f"[track] progress {trial + 1}/{n_trials}: "
                f"SAT {feasible}, UNSAT {unsat}",
                flush=True,
            )

    print(
        f"[track] seed={seed} trials={n_trials}: "
        f"SAT {feasible}/{n_trials}, UNSAT {unsat}/{n_trials}"
    )
    if dump_trial is not None and not dump_all and first_ok is not None and dump_trial >= n_trials:
        tob.dump_occupancy(first_ok, 0, "track first-SAT")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--dump-trial",
        type=int,
        default=-1,
        help="print occupancy tables for this trial index; negative to skip",
    )
    parser.add_argument(
        "--dump-all",
        action="store_true",
        help="print occupancy tables for every SAT trial",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dump_trial = None if args.dump_trial < 0 else args.dump_trial
    return run_trials(args.trials, args.seed, dump_trial, args.dump_all)


if __name__ == "__main__":
    raise SystemExit(main())
