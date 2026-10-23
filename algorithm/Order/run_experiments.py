#!/usr/bin/env python3
"""Run four Net-priority policies on four cases using isolated source copies."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import os
import platform
import subprocess
import sys
import traceback
import uuid

sys.dont_write_bytecode = True
from build import HERE, ROOT, build
from report import CASES, POLICIES, PRIORITIES, report


def plan(cases, smoke):
    if smoke:
        return [("case1", s) for s in ["current", "current", "new1", "new2", "new3"]]
    return [(case, scheme) for case in cases for scheme in POLICIES]


def save_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)


def validate(result, case, scheme, seed, folder):
    if (result["policy"], result["seed"], result["cob_width"]) != (POLICIES.index(scheme), seed, CASES[case]):
        raise RuntimeError("Policy/seed/width mismatch")
    n = result["total_nets"]
    if result["attempted_nets"] != n or result["routed_nets"] + result["failed_nets"] != n:
        raise RuntimeError("Net counters inconsistent")
    if result["all_routed"] != (result["failed_nets"] == 0 and result["retry_failed_nets"] == 0):
        raise RuntimeError("Incorrect routability flag")
    with (folder / "order.tsv").open() as f:
        order = list(csv.DictReader(f, delimiter="\t"))
    with (folder / "nets.tsv").open() as f:
        actual = list(csv.DictReader(f, delimiter="\t"))
    if len(order) != n or len(actual) != n or [r["uid"] for r in order] != [r["uid"] for r in actual]:
        raise RuntimeError("Actual per-Net logs disagree with the sorted order")
    for row in order:
        if float(row["base_priority"]) != PRIORITIES[row["class"]][POLICIES.index(scheme)]:
            raise RuntimeError("Actual class priority does not match the requested policy")
    sequence = json.dumps([r["uid"] for r in order], ensure_ascii=False).encode()
    return dict(order_sha256=hashlib.sha256(sequence).hexdigest(),
                **{f"count_{kind}": sum(r["class"] == kind for r in order) for kind in PRIORITIES})


def run(args):
    cases = ["case1"] if args.smoke else args.cases
    trials = plan(cases, args.smoke)
    if args.plan:
        print(json.dumps([dict(case=c, scheme=s, priorities={k:v[POLICIES.index(s)] for k,v in PRIORITIES.items()})
                          for c,s in trials], indent=2))
        return 0
    tag = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    folder = HERE / "logs" / f"{'smoke' if args.smoke else 'sweep'}-{tag}-{uuid.uuid4().hex[:6]}"
    folder.mkdir(parents=True)
    with (folder / "runner.log").open("w", buffering=1) as master:
        def log(message):
            print(message, flush=True); master.write(message + "\n")
        manifest = dict(mode="smoke" if args.smoke else "sweep", cases=cases, policies=PRIORITIES,
                        seed=args.seed, timeout=args.timeout, planned_runs=len(trials),
                        platform=platform.platform(), python=sys.version, argv=sys.argv,
                        started_at=datetime.now().astimezone().isoformat(), status="building", input_sha256={})
        rows = []
        def save_manifest():
            (folder / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
        save_manifest()
        log(f"Logs: {folder}\nPlanned runs: {len(trials)}; seed={args.seed}; timeout={args.timeout}s")
        try:
            for case in cases:
                for path in sorted((ROOT / "test/config" / case).rglob("*")):
                    if path.is_file(): manifest["input_sha256"][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
            log("Building isolated width-specific executables...")
            binaries, metadata = build({CASES[c] for c in cases}, args.jobs, args.cxx, folder)
            manifest.update(metadata, status="running"); save_manifest()
            for width, binary in binaries.items():
                with (folder / f"self-test-width{width}.log").open("w") as out:
                    subprocess.run([str(binary), "--self-test"], cwd=folder, stdout=out,
                                   stderr=subprocess.STDOUT, check=True, timeout=30)
            log("PASS: all 8 concrete Net classes x 4 policies (C++ self-test)")
            for index, (case, scheme) in enumerate(trials, 1):
                run_id = f"{index:03d}-{case}-{scheme}"
                run_dir = folder / run_id; run_dir.mkdir()
                cmd = [str(binaries[CASES[case]]), str(ROOT / "test/config" / case), str(run_dir),
                       str(args.seed), str(POLICIES.index(scheme)), run_id]
                (run_dir / "command.json").write_text(json.dumps(cmd, indent=2) + "\n")
                row = dict(run_id=run_id, case=case, scheme=scheme, status="error")
                log(f"[{index}/{len(trials)}] {case} {scheme}")
                try:
                    with (run_dir / "console.log").open("w") as out:
                        completed = subprocess.run(cmd, cwd=run_dir, stdout=out, stderr=subprocess.STDOUT, timeout=args.timeout)
                    if completed.returncode: raise RuntimeError(f"Exit code {completed.returncode}; see console.log")
                    result = json.loads((run_dir / "result.json").read_text())
                    coverage = validate(result, case, scheme, args.seed, run_dir)
                    row.update(result, status="ok")
                    row.update(coverage)
                    log(f"  routed={result['routed_nets']}/{result['total_nets']}; failed={result['failed_nets']}; "
                        f"length={result['total_length']}; seconds={result['routing_seconds']:.3f}")
                except (OSError, ValueError, KeyError, TypeError, RuntimeError, subprocess.TimeoutExpired) as error:
                    row.update(status="timeout" if isinstance(error, subprocess.TimeoutExpired) else "error", error=str(error))
                    log(f"  {row['status'].upper()}: {error}")
                rows.append(row)
                save_csv(folder / "results.csv", rows)
                (folder / "tables.md").write_text(report(rows, cases, manifest, folder))
            # Layout must stay identical across all schemes, including failures.
            for case in cases:
                layouts = [(folder/r["run_id"]/"placement.tsv").read_bytes() for r in rows
                           if r["case"] == case and (folder/r["run_id"]/"placement.tsv").exists()]
                if any(layout != layouts[0] for layout in layouts): raise RuntimeError("Layout changed between policies")
            if args.smoke:
                if any(r["status"] != "ok" for r in rows): raise RuntimeError("Smoke process did not produce all measurements")
                a, b = rows[:2]
                ignored = {"run_id", "routing_seconds"}
                if {k:v for k,v in a.items() if k not in ignored} != {k:v for k,v in b.items() if k not in ignored}:
                    raise RuntimeError("Same-policy summary is not reproducible")
                for filename in ("order.tsv", "sync_members.tsv", "paths.tsv", "placement.tsv"):
                    if (folder/a["run_id"]/filename).read_bytes() != (folder/b["run_id"]/filename).read_bytes():
                        raise RuntimeError(f"Same-policy {filename} is not reproducible")
                manifest["reproducibility"] = "PASS: order, summary, sync members, paths and placement match"
                log(manifest["reproducibility"])
            errors = sum(r["status"] != "ok" for r in rows)
            routing_failures = sum(r["status"] == "ok" and not r["all_routed"] for r in rows)
            manifest.update(status="failed" if errors else "complete", completed_runs=len(rows),
                            process_errors=errors, routing_failed_runs=routing_failures)
            if not args.smoke and cases == list(CASES):
                (HERE / "experiments.md").write_text(report(rows, cases, manifest, folder))
            log(f"Finished: {len(rows)-errors}/{len(trials)} measurements; {routing_failures} incomplete routing outcomes; {errors} process errors.")
            return int(errors > 0)
        except BaseException as error:
            manifest.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed", error=str(error), completed_runs=len(rows))
            master.write(traceback.format_exc()); log(f"Stopped: {error}; details: {folder/'runner.log'}")
            return 130 if isinstance(error, KeyboardInterrupt) else 1
        finally:
            manifest["finished_at"] = datetime.now().astimezone().isoformat(); save_manifest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke", action="store_true", help="case1: repeat current twice, then run new1/new2/new3")
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--seed", type=int, default=20261023)
    parser.add_argument("--cxx", default=os.environ.get("CXX", "c++"))
    parser.add_argument("--jobs", type=int, default=min(4, os.cpu_count() or 1), help="parallel compiler jobs; routing remains serial")
    parser.add_argument("--timeout", type=float, default=600, help="seconds per routing process")
    parser.add_argument("--plan", action="store_true", help="print plan without building or writing files")
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64 or args.jobs < 1 or args.timeout <= 0 or len(set(args.cases)) != len(args.cases):
        parser.error("invalid seed, jobs, timeout or duplicate cases")
    return run(args)


if __name__ == "__main__": sys.exit(main())
