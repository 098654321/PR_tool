#!/usr/bin/env python3
"""Run the agreed one-factor-at-a-time SA study without modifying production code."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import traceback
import uuid

sys.dont_write_bytecode = True
from build import HERE, ROOT, build

BASELINE = dict(init_temp=100.0, freeze_temp=0.5, solve_num=50,
                cooling_rate=0.97, max_no_improvement=50)
VALUES = dict(init_temp=[10, 30, 100, 300, 1000],
              freeze_temp=[0.01, 0.1, 0.5, 1, 5],
              solve_num=[10, 25, 50, 100, 200],
              cooling_rate=[0.80, 0.90, 0.95, 0.97, 0.99],
              max_no_improvement=[5, 10, 25, 50, 100])
CASES = {"case1": 12, "case5": 12, "case7": 13, "case12": 13}
LABELS = dict(init_temp="初始温度", freeze_temp="冻结温度", solve_num="每层尝试基础系数",
              cooling_rate="降温系数", max_no_improvement="连续无改进基础上限")


def plan(cases, smoke):
    if smoke:
        trials = [("baseline", 0, BASELINE.copy()), ("baseline", 0, BASELINE.copy())]
        for name, values in VALUES.items():
            index = 0 if values[-1] == BASELINE[name] else len(values) - 1
            trials.append((name, index, {**BASELINE, name: values[index]}))
        return [("case1", name, index, params) for name, index, params in trials]
    return [(case, name, index, {**BASELINE, name: value})
            for name, values in VALUES.items() for case in cases
            for index, value in enumerate(values)]


def write_csv(path, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def report(rows, cases, manifest, folder):
    successful = {(r["case"], r["parameter"], r["value_index"]): r
                  for r in rows if r["status"] == "ok"}
    lines = ["# SA 布局参数实验", "", f"运行目录：`{folder.relative_to(HERE)}`。",
             f"运行模式：{manifest['mode']}；随机种子：{manifest['seed']}；"
             f"源码指纹：`{manifest['source_fingerprint']}`。", "",
             "每个参数值、每个测例独立加载原始输入并布局一次。只改变当前参数，其他参数取下列默认值：", "",
             "```json", json.dumps(BASELINE, ensure_ascii=False, indent=2), "```", "",
             "表格单元格是布局恢复后重新计算的 HPWL 总和，越小越好。"
             "未完成项记为 —，失败项记为 FAIL；两者均不计为零。"
             "未执行布线，单次结果不代表统计显著性或全局最优。", "",
             "## 测例与预算", "",
             "每层候选尝试次数 = solve_num × N；连续无改进层数上限 = "
             "max_no_improvement × max(1, floor(N/2))。", "",
             "| case | 芯粒数 | COB 阵列 | 默认每层尝试次数 | 默认停滞层数上限 |",
             "| --- | ---: | --- | ---: | ---: |"]
    for case in cases:
        match = next((r for r in rows if r["case"] == case and r["status"] == "ok"), None)
        n = match["chips"] if match else None
        lines.append(f"| {case} | {n if n else '—'} | 9×{CASES[case]} | "
                     f"{50*n if n else '—'} | {50*max(1, n//2) if n else '—'} |")
    for name, values in VALUES.items():
        fixed = ", ".join(f"{k}={v}" for k, v in BASELINE.items() if k != name)
        lines += ["", f"## {LABELS[name]} `{name}`", "", f"固定参数：`{fixed}`。", "",
                  "| case | " + " | ".join(map(str, values)) + " |",
                  "| --- | " + " | ".join(["---:"] * 5) + " |"]
        for case in cases:
            cells = []
            for i in range(5):
                row = successful.get((case, name, i))
                failed = any(r["case"] == case and r["parameter"] == name
                             and r["value_index"] == i and r["status"] != "ok" for r in rows)
                cells.append(str(row["final_hpwl"]) if row else "FAIL" if failed else "—")
            lines.append("| " + case + " | " + " | ".join(cells) + " |")
    lines += ["", "## 日志与实验边界", "",
              "`results.csv` 保存全部参数、实际预算、初始/最终 HPWL、改善比例、布局耗时、"
              "温度层数、终止温度、停滞计数、停止原因、候选操作及接受/拒绝/无效次数。",
              "每次运行目录包含 `debug.log`、`console.log`、`iterations.tsv`、"
              "初始/最终位置及 `result.json`；`manifest.json` 保存输入哈希、构建和运行环境。",
              "耗时包含布局日志开销，不包含配置解析、线网构建、编译及布线。"
              "逐层表中的温度是该层实际使用的温度；尝试及接受计数为累计值。",
              "保留当前 SA 的层末最优记录、提前停止、交换概率公式和合法性限制。"
              "统一使用 mt19937_64 随机源，并按名称排序芯粒、按坐标排序空闲 TOB。",
              "空闲 TOB 选择保留原实现的上界调用 size()-1（random_i64 为右开区间），"
              "因此未顺便修正原有的末项不被抽中行为。case1/5/7 无空闲位置；case12 有4个。",
              "参数之间可能通过停止条件相互影响；需结合停止原因、搜索量和耗时解读目标值。", ""]
    return "\n".join(lines)


def validate_result(result, params, case):
    if any(result[k] != value for k, value in params.items()):
        raise RuntimeError("Executable reported different parameters")
    if result["cob_width"] != CASES[case] or not result["valid_placement"]:
        raise RuntimeError("Width/placement validation failed")
    if result["final_hpwl"] > result["initial_hpwl"]:
        raise RuntimeError("Final HPWL worse than initial placement")
    if result["invalid_attempts"] < 0 or result["attempts"] != sum(
            result[k] for k in ("accepted_nonworse", "accepted_uphill", "rejected", "invalid_attempts")):
        raise RuntimeError("Inconsistent acceptance counters")


def run(args):
    cases = ["case1"] if args.smoke else args.cases
    trials = plan(cases, args.smoke)
    if args.plan:
        print(json.dumps([dict(case=c, parameter=n, value_index=i, **p) for c, n, i, p in trials], indent=2))
        return 0
    tag = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    folder = HERE / "logs" / f"{'smoke' if args.smoke else 'sweep'}-{tag}-{uuid.uuid4().hex[:6]}"
    folder.mkdir(parents=True)
    with (folder / "runner.log").open("w", buffering=1) as master:
        def log(message):
            print(message, flush=True)
            master.write(message + "\n")

        manifest = dict(mode="smoke" if args.smoke else "sweep", seed=args.seed,
                        baseline=BASELINE, values=VALUES, cases=cases, planned_runs=len(trials),
                        platform=platform.platform(), python=sys.version,
                        started_at=datetime.now().astimezone().isoformat(), status="building",
                        argv=sys.argv, input_sha256={})
        manifest_path = folder / "manifest.json"

        def save_manifest():
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")

        save_manifest()
        log(f"Logs: {folder}\nPlanned runs: {len(trials)}; seed: {args.seed}")
        rows = []
        try:
            for case in cases:
                for path in sorted((ROOT / "test/config" / case).rglob("*")):
                    if path.is_file():
                        manifest["input_sha256"][str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
            log("Building isolated placement executables (no Qt / SAT / Gurobi required)...")
            binaries, metadata = build({CASES[c] for c in cases}, args.jobs, args.cxx, folder)
            manifest.update(metadata, status="running")
            save_manifest()
            for number, (case, name, index, params) in enumerate(trials, 1):
                run_id = f"{number:03d}-{case}-{name}-{index}"
                run_dir = folder / run_id
                run_dir.mkdir()
                cmd = [str(binaries[CASES[case]]), str(ROOT / "test/config" / case),
                       str(run_dir), str(args.seed), *[str(v) for v in params.values()], run_id]
                row = dict(run_id=run_id, case=case, parameter=name, value_index=index,
                           parameter_value=params.get(name, ""), status="failed", **params)
                log(f"[{number}/{len(trials)}] {case} {name}={row['parameter_value']}")
                (run_dir / "command.json").write_text(json.dumps(cmd, indent=2) + "\n")
                try:
                    with (run_dir / "console.log").open("w") as console:
                        completed = subprocess.run(cmd, cwd=run_dir, stdout=console,
                                                   stderr=subprocess.STDOUT, timeout=args.timeout)
                    if completed.returncode:
                        raise RuntimeError(f"Exit code {completed.returncode}; see console.log")
                    result = json.loads((run_dir / "result.json").read_text())
                    validate_result(result, params, case)
                    if result["seed"] != args.seed:
                        raise RuntimeError("Seed mismatch")
                    row.update(result, status="ok")
                    log(f"  HPWL {result['initial_hpwl']} -> {result['final_hpwl']}; "
                        f"{result['placement_seconds']:.3f}s; layers={result['temperature_layers']}; "
                        f"stop={result['stop_reason']}")
                except (OSError, ValueError, KeyError, TypeError, RuntimeError, subprocess.TimeoutExpired) as error:
                    row["error"] = str(error)
                    log(f"  FAILED: {error}")
                rows.append(row)
                write_csv(folder / "results.csv", rows)
                (folder / "tables.md").write_text(report(rows, cases, manifest, folder))
            if args.smoke:
                if any(r["status"] != "ok" for r in rows):
                    raise RuntimeError("Smoke run failed")
                first, second = rows[:2]
                ignored = {"run_id", "placement_seconds"}
                if {k: v for k, v in first.items() if k not in ignored} != {
                        k: v for k, v in second.items() if k not in ignored}:
                    raise RuntimeError("Same-seed summary is not reproducible")
                for filename in ("initial_placement.tsv", "final_placement.tsv", "iterations.tsv"):
                    if (folder / first["run_id"] / filename).read_bytes() != (folder / second["run_id"] / filename).read_bytes():
                        raise RuntimeError(f"Same-seed {filename} is not reproducible")
                manifest["reproducibility"] = "PASS: baseline objective, full trajectory and placements match"
                log(manifest["reproducibility"])
            failures = sum(r["status"] != "ok" for r in rows)
            manifest.update(status="failed" if failures else "complete", completed_runs=len(rows), failed_runs=failures)
            if not args.smoke and cases == list(CASES):
                (HERE / "experiments.md").write_text(report(rows, cases, manifest, folder))
            log(f"Finished: {len(rows) - failures}/{len(trials)} successful. Results: {folder / 'results.csv'}")
            return int(failures > 0)
        except BaseException as error:
            manifest.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed",
                            completed_runs=len(rows), error=str(error))
            master.write(traceback.format_exc())
            log(f"Stopped: {error}. Details: {folder / 'runner.log'}")
            return 130 if isinstance(error, KeyboardInterrupt) else 1
        finally:
            manifest["finished_at"] = datetime.now().astimezone().isoformat()
            save_manifest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke", action="store_true", help="case1: duplicate baseline + five parameter variants (7 runs)")
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--seed", type=int, default=20261023)
    parser.add_argument("--jobs", type=int, default=min(4, os.cpu_count() or 1), help="parallel compiler jobs; experiments remain serial")
    parser.add_argument("--cxx", default=os.environ.get("CXX", "c++"), help="C++20 compiler with std::format support")
    parser.add_argument("--timeout", type=float, default=3600, help="seconds per placement process")
    parser.add_argument("--plan", action="store_true", help="print experiment plan without building or writing files")
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64 or args.jobs < 1 or args.timeout <= 0:
        parser.error("seed must be uint64; jobs and timeout must be positive")
    if len(set(args.cases)) != len(args.cases):
        parser.error("duplicate cases")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
