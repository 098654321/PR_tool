# Diff interpretation

After `compare_controlbits.py` reports mismatches, assume steps 2–4 and path
replay are correct unless evidence shows otherwise.

## Allowed vs unexpected

| Kind | Guidance |
|------|----------|
| Register values describing **used** interconnect resources | Must match golden. Fix PR Writer (or harness path application) if not. |
| TOB unused mux fill (`bump_to_hori_muxs` / `hori_to_vert_muxs` / `vert_to_track_muxs`) via PR `randomly_map_remain_indexes()` | May differ from kiwi **only if** proven to be unused mux-fill. Document evidence. |
| COB / `dly` / `drv` / other non mux-fill diffs | Unexpected — investigate. |

## Writer bug path

1. Locate differing register names and hex values.
2. Map names to Writer / golden print paths (`source/parse/writer/`, kiwi `printControlBit*`).
3. Fix PR_tool output logic.
4. Re-run from writer step (or `run_case.sh`) without `-s`.

## Harness issues (not Writer)

Symptoms seen historically:

- Stale `net_path_info` vs regenerated golden → regenerate kiwi + convert.
- Golden path blocks **outnumber** leaf nets (ExtIO extras) → harness must
  `occupy_all` + `connect_all` unused blocks (see `scripts/test_writer.cc`).
- Power rails (`endpoint: nege|pose`, multi `Printing path...`) →
  `TracksToBumpsNet` + per-segment history merge.

If the failure is harness/path-shape support, fix `scripts/test_writer.cc`, not
Writer.

## Next case

When all four files match: mark case done, pick the next numbered testcase.
When all cases pass: skill run complete.
