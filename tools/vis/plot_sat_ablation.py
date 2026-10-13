#!/usr/bin/env python3
"""Plot the two ablation tables in Projects/v24-0930-reselection/data.md."""

from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "pr_tool-matplotlib"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import StrMethodFormatter


DEFAULT_DATA = Path(__file__).resolve().parents[3] / "Projects/v24-0930-reselection/data.md"
BLUE = "#0072B2"
ORANGE = "#D55E00"


def read_table(text: str, heading: str, columns: int) -> list[list[str]]:
    if text.count(heading) != 1:
        raise ValueError(f"expected one heading: {heading}")
    table = []
    for line in text.split(heading, 1)[1].splitlines():
        if line.startswith("|"):
            table.append([cell.strip() for cell in line.strip().strip("|").split("|")])
        elif table:
            break
    if len(table) < 3 or any(len(row) != columns for row in table):
        raise ValueError(f"missing or malformed table under {heading}")
    return [row for row in table[2:] if row[0] != "Average"]


def read_data(data: Path) -> tuple[list, list]:
    text = data.read_text(encoding="utf-8")
    stages = read_table(text, "1. 实验1", 9)
    comparisons = read_table(text, "2. 实验2", 5)
    cases = [row[1] for row in stages]
    if not cases or len(set(cases)) != len(cases) or cases != [row[0] for row in comparisons]:
        raise ValueError("the ablation tables must contain the same unique cases in the same order")
    stage_values = [(row[1], *(float(value) for value in row[2:5])) for row in stages]
    comparison_values = [(row[0], *(None if value == "/" else float(value) for value in row[1:]))
                         for row in comparisons]
    for case, sat, ilp, final in stage_values:
        if not 0 < final <= ilp <= sat:
            raise ValueError(f"{case}: invalid SAT/ILP/final wirelengths")
    for case, wl, without_wl, time, without_time in comparison_values:
        if wl is None or time is None or wl <= 0 or time <= 0:
            raise ValueError(f"{case}: missing or invalid Ours result")
        if (without_wl is None) != (without_time is None):
            raise ValueError(f"{case}: inconsistent missing without-SAT result")
        if without_wl is not None and (without_wl <= 0 or without_time <= 0):
            raise ValueError(f"{case}: invalid without-SAT result")
    if [row[3] for row in stage_values] != [row[1] for row in comparison_values]:
        raise ValueError("final wirelengths differ between the two ablation tables")
    return stage_values, comparison_values


def draw(stages: list, comparisons: list, output: Path) -> None:
    plt.rcParams.update({"font.family": "serif", "font.serif": ["Times New Roman", "DejaVu Serif"],
                         "font.size": 16.5, "pdf.fonttype": 42, "ps.fonttype": 42})
    fig, axes = plt.subplots(1, 3, figsize=(6.8, 3.4),
                             gridspec_kw={"width_ratios": [1.0, 1.15, 1.55]})
    fig.subplots_adjust(left=.17, right=.94, bottom=.159, top=.829, wspace=.85)
    wl_ax, time_ax, gain_ax = axes
    y = list(range(len(comparisons)))
    labels = [row[0][:-1] + r"$^{*}$" if row[0].endswith("*") else row[0]
              for row in comparisons]

    for ax, reference in [(wl_ax, 1), (time_ax, 1)]:
        ax.set_ylim(len(y) - .4, -.6)
        ax.set_yticks(y)
        ax.spines[["top", "right", "left"]].set_visible(False)
        ax.tick_params(axis="y", length=0, pad=3, labelsize=13.5)
        ax.tick_params(axis="x", length=2, pad=2, labelsize=15)
        ax.grid(axis="y", color="#ededed", linewidth=.5, zorder=0)
        ax.axvline(reference, color="#c3c7cd", linewidth=.7, zorder=1)
        ax.scatter([reference] * len(y), y, color=BLUE, s=14, zorder=3)
    wl_ax.set_yticklabels(labels)
    time_ax.set_yticklabels([])
    wl_ax.set(xlim=(.996, 1.056), xticks=[1, 1.05])
    time_ax.set(xlim=(-.5, 30), xticks=[1, 15, 30])
    for ax in (wl_ax, time_ax):
        ax.xaxis.set_major_formatter(StrMethodFormatter("{x:g}×"))
    wl_ax.set_xlabel("WL", labelpad=5)
    time_ax.set_xlabel("Runtime", labelpad=5)

    for index, (case, wl, without_wl, time, without_time) in enumerate(comparisons):
        if without_wl is None:
            missing_label = ">24h" if case.rstrip("*") in {"real_4", "Ho_NPU_3"} else "NR"
            for ax, x in [(wl_ax, 1.045), (time_ax, 24)]:
                ax.text(x, index, missing_label, ha="center", va="center", color=ORANGE, fontsize=15)
            continue
        values = [(wl_ax, 1, without_wl / wl),
                  (time_ax, 1, without_time / time)]
        for ax, reference, value in values:
            if not ax.get_xlim()[0] <= value <= ax.get_xlim()[1]:
                raise ValueError("comparison lies outside the figure's axes")
            ax.plot([reference, value], [index, index], color="#a8adb3", linewidth=.7, zorder=2)
            ax.scatter([value], [index], marker="D", s=24, facecolors="none",
                       edgecolors=ORANGE, linewidths=1, zorder=4)

    # Preserve the original Stage gains data, colors and bar height.
    ilp_gain = [100 * (1 - ilp / sat) for _, sat, ilp, _ in stages]
    rrr_gain = [100 * (ilp - final) / sat for _, sat, ilp, final in stages]
    gain_ax.barh(y, ilp_gain, height=.68, color=BLUE, label="ILP")
    gain_ax.barh(y, rrr_gain, left=ilp_gain, height=.68, color=ORANGE, label="RRR")
    gain_ax.set_yticks(y, [])
    gain_ax.set_ylim(len(y) - .4, -.6)
    gain_ax.set(xlim=(0, 20), xticks=[0, 10, 20], xlabel="WL Reduction(%)")
    gain_ax.tick_params(axis="both", length=2, labelsize=13.5, pad=1)
    gain_ax.tick_params(axis="x", labelsize=15)
    gain_ax.spines[["top", "right"]].set_visible(False)

    left = wl_ax.get_position().x0
    right = time_ax.get_position().x1
    fig.text((left + right) / 2, .988, "(a) Effect of SAT initialization", ha="center", va="top", fontsize=15)
    gain_pos = gain_ax.get_position()
    fig.text((gain_pos.x0 + gain_pos.x1) / 2, .988, "(b) ILP/RRR stage gains", ha="center", va="top", fontsize=15)
    gain_handles, gain_labels = gain_ax.get_legend_handles_labels()
    fig.legend(gain_handles, gain_labels, loc="upper center",
               bbox_to_anchor=((gain_pos.x0 + gain_pos.x1) / 2, .951),
               frameon=False, ncol=2, fontsize=15, handlelength=1, columnspacing=1)
    handles = [Line2D([], [], marker="o", linestyle="none", color=BLUE, markersize=4, label="SAT+ILP+RRR"),
               Line2D([], [], marker="D", linestyle="none", color=ORANGE, markerfacecolor="none",
                      markersize=5, label="ILP+RRR")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=((left + right) / 2, .951),
               frameon=False, ncol=2, fontsize=15, handlelength=1, columnspacing=1)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=300, facecolor="white")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    stages, comparisons = read_data(args.data)
    output = args.output or args.data.parent / "sat_ablation.png"
    draw(stages, comparisons, output)
    missing = [row[0] for row in comparisons if row[2] is None]
    print(f"Rendered {len(comparisons)} cases to {output}; NR: {', '.join(missing)}")


if __name__ == "__main__":
    main()
