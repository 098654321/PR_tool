"""Generate result tables without treating failed/partial routes as shorter wins."""
from pathlib import Path

HERE = Path(__file__).resolve().parent
CASES = {"case2": 12, "case13": 13, "case5": 12, "case7": 13, "case1": 12, "case12": 13}
DEFAULT_CASES = ["case2", "case13", "case5", "case7"]
POLICIES = ["current", "new2", "new3"]
POLICY_IDS = {"current": 0, "new2": 2, "new3": 3}
PRIORITIES = {
    "SyncNet": [0, 0, 2],
    "TrackToBumpsNet": [1, 2, 3],
    "BumpToTracksNet": [1, 2, 3],
    "BumpToBumpsNet": [2, 1, 4],
    "BumpToTrackNet": [3, 4, 0],
    "TrackToBumpNet": [3, 4, 0],
    "BumpToBumpNet": [4, 3, 1],
    "TracksToBumpsNet": [5, 5, 5],
}


def report(rows, cases, manifest, folder):
    lookup = {(r["case"], r["scheme"]): r for r in rows}
    lines = ["# 布线顺序实验", "", f"运行目录：`{folder.relative_to(HERE)}`；随机种子：{manifest['seed']}。",
             f"源码/工具链指纹：`{manifest['source_fingerprint']}`。", "",
             "每种顺序、每个测例独立运行一次。固定原始布局，不执行SA；mode=0，非增量Maze布线。",
             "只切换下表的基础优先级，小数值先布线。", "",
             "| Net class | " + " | ".join(POLICIES) + " |", "| --- | " + " | ".join(["---:"] * len(POLICIES)) + " |"]
    for name, values in PRIORITIES.items():
        lines.append("| " + name + " | " + " | ".join(map(str, values)) + " |")
    lines += ["", "new2采用用户更正后的bump优先顺序；BumpToTrackNet与TrackToBumpNet保持并列，均为4。",
              "其他参数、端口数偏置、搜索规则及原有线长定义均保持一致。"
              "普通Net偏置为 `0.9*port_number/max_port_number`，数值升序，因此同档端口少者先。",
              "保留SyncNet仅更新内部成员优先级、外层不加端口偏置的现状。"
              "同步组不拆开参与顶层调度；完全同分的顶层Net按UID排序。",
              "TOB接入候选保留COB-unit使用量排序，并在使用量相同时按Track坐标打破平局。",
              "COB阵列：" + "；".join(f"{case}为9×{CASES[case]}" for case in cases) + "。", ""]
    lines += ["## 顶层Net类别覆盖", "", "同步组成员不在此表拆开计数，避免误认为存在可独立调序的二端Net。", "",
              "| case | " + " | ".join(PRIORITIES) + " |",
              "| --- | " + " | ".join(["---:"] * len(PRIORITIES)) + " |"]
    for case in cases:
        sample = next((r for r in rows if r["case"] == case and r["status"] == "ok"), None)
        counts = [str(sample[f"count_{kind}"]) if sample else "—" for kind in PRIORITIES]
        lines.append("| " + case + " | " + " | ".join(counts) + " |")
    lines += ["", "某些测例缺少被调序的类别，修改优先级后实际顺序可能不变。"
              "下表通过排序后的UID序列哈希比较；相同表示本次没有真正改变顶层布线序列。", ""]
    metrics = [("实际顺序与current的比较", "order_sha256"), ("完成状态", None), ("失败顶层Net数量", "failed_nets"),
               ("顶层Net布通率 %", "routability_pct"), ("总线长", "total_length"),
               ("最长成员/普通Net线长", "max_length"), ("布线耗时 s", "routing_seconds"),
               ("同步组等长检查失败数量", "sync_unequal_groups")]
    for title, key in metrics:
        lines += [f"## {title}", "", "| case | " + " | ".join(POLICIES) + " |",
                  "| --- | " + " | ".join(["---:"] * len(POLICIES)) + " |"]
        for case in cases:
            cells = []
            for policy in POLICIES:
                row = lookup.get((case, policy))
                if row is None:
                    value = "—"
                elif row["status"] != "ok":
                    value = row["status"].upper()
                elif key is None:
                    value = "布通" if row["all_routed"] else "未布通"
                elif key == "order_sha256":
                    baseline = lookup.get((case, "current"))
                    value = "—" if not baseline or baseline["status"] != "ok" else (
                        "相同" if row[key] == baseline[key] else "不同")
                else:
                    value = f"{row[key]:.4f}" if isinstance(row[key], float) else str(row[key])
                    if key in ("total_length", "max_length") and not row["all_routed"]:
                        value += "（未布通）"
                cells.append(value)
            lines.append("| " + case + " | " + " | ".join(cells) + " |")
        lines.append("")
    lines += ["## 统计口径与日志", "",
              "一个SyncNet整体计为一个顶层Net；成员数与成员线长另外记录。"
              "SyncNet只有成员全部非零且等长、并且未抛出路由失败时才计为布通。",
              "总线长直接累计现有Net::length()，沿用当前压缩/计数规则；"
              "失败运行可能包含残留部分路径，不能与完整布通结果直接比较线长优劣。",
              "`routing_seconds`包含排序、资源检查、布线、路径导出与日志开销，"
              "不包含编译、读取配置、线网构建和布局。单次运行不提供统计显著性结论。",
              "`results.csv`保存完整汇总；每次运行的`order.tsv`为实际排序，`nets.tsv`为逐Net结果，"
              "`sync_members.tsv`为同步成员结果，`paths.tsv`为坐标与连接器路径，`placement.tsv`为固定布局。",
              "`debug.log`与`console.log`保留原生路由输出；超时/进程错误保留已有日志并继续其他组合。",
              "每个进程的完整参数在`command.json`中；环境、输入哈希和编译信息见`manifest.json`。", ""]
    return "\n".join(lines)
