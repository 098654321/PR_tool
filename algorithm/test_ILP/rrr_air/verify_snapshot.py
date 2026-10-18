"""Verify namespace-only core files against the requested Git source."""
from pathlib import Path
import subprocess

SOURCE = "316e47799aa73cbc123134fe34d530504f235894"
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ADAPTED = {"rrr_router.cc", "rrr_router.hh", "rrr_types.hh", "stop_policy.hh", "import_bbox.hh",
           "air_search.hh", "hardware_graph.hh", "maze_search.cc",
           "resource_model.cc", "resource_model.hh", "resource_cost.cc"}
count = 0
for path in sorted(HERE.iterdir()):
    if path.suffix not in {".cc", ".hh"} or path.name in ADAPTED:
        continue
    original = subprocess.check_output(
        ["git", "show", f"{SOURCE}:algorithm/FPIA-RRR/{path.name}"],
        cwd=ROOT, text=True,
    )
    expected = original.replace("PR_tool::rrr_detail", "PR_tool::air316::rrr_detail")
    expected = expected.replace("namespace PR_tool {", "namespace PR_tool::air316 {")
    expected = expected.replace("} // namespace PR_tool\n", "} // namespace PR_tool::air316\n")
    if path.read_text() != expected:
        raise SystemExit(f"Snapshot differs beyond namespace: {path.name}")
    count += 1
print(f"PASS: {count} core files match {SOURCE} (namespace-only changes)")
