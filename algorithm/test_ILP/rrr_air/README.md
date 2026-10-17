# AIR/OpenROAD RRR fixed implementation

Source: `316e47799aa73cbc123134fe34d530504f235894:algorithm/FPIA-RRR/`.
This directory belongs to test_ILP. It does not depend on the current standalone
FPIA-RRR, which subsequently changed to VPR cost.

`PR_tool` and `PR_tool::rrr_detail` are renamed to `PR_tool::air316` and
`PR_tool::air316::rrr_detail` to keep SAT/ILP types and algorithms separate.
`route_ilp/route_rrr.cc` maps the same physical graph, endpoints and ILP paths;
PN virtual nodes/arcs are omitted. Production ordering still calls the original
Sort/Resources on `basedie->nets().at(0)`, after ILP has finished. The bridge
permutes only AIR nets and imported path slots by raw object identity, preserving
the ILP IDs and mapping the result back; SAT/ILP retain nets_to_vector() order.

The only algorithm adaptations to the snapshot are:

- `rrr_router.*`: import ILP paths as the completed first round, rebuild live
  claims/tree/bins, initialize the legal incumbent; use the new stopping policy;
  on timeout without a legal incumbent restore the original legal partial input,
  preserving empty demand slots (the baseline collector compacts missing lanes).
- `rrr_types.hh`: absolute RRR deadline.
- `import_bbox.hh`: contain all imported ILP path nodes in the initially padded
  terminal box before the original complete-net d==0 round-end expansion. Bins
  and independent reference routes retain their original definitions.
- `stop_policy.hh`: stop at ten consecutive nonimproving **legal solutions** or
  at the absolute deadline, including when no legal solution exists.
- Reference work checks that deadline between nets. Search units remain atomic
  as in the source implementation; one maze/sync search can overrun the budget.

All other copied files are namespace-only copies. Preserve their ordering,
resource rules, search limits, dynamic boxes, sync equalization and node cost.
There is no VPR present-factor scheduling and no extra arc/resource surcharge.
Unused standalone CLI/main/net adapter files are not copied.

Verification (from PR_tool):

```bash
python3 algorithm/test_ILP/rrr_air/verify_snapshot.py
xmake build route_ilp_unit
./output/route_ilp_unit
xmake build test_ILP
```
