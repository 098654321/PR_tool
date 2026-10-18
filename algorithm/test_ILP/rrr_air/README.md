# AIR RRR with 0d6e86f test_ILP cost

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
- `rrr_types.hh`: absolute RRR deadline; remove obsolete OpenROAD cost parameters.
- `import_bbox.hh`: contain all imported ILP path nodes in the initially padded
  terminal box before the original complete-net d==0 round-end expansion. Bins
  and independent reference routes retain their original definitions.
- `stop_policy.hh`: stop at ten consecutive nonimproving **legal solutions** or
  at the absolute deadline, including when no legal solution exists.
- Reference work checks that deadline between nets. Search units remain atomic
  as in the source implementation; one maze/sync search can overrun the budget.

- Cost only: `resource_cost.cc` ports
  `0d6e86f47ebe4e796f3d267336510b8febfef92e:algorithm/test_ILP/route_ilp/rrr_cost.cc`.
  `resource_model.*` supplies live owner counts and additive decaying history;
  `hardware_graph.hh` caches the separate cost projection. `air_search.hh` and
  `maze_search.cc` use this arc cost for ordinary, reference and sync-tail search
  and branch comparison. Capacity keys and legality checks are unchanged.

Cost: arc base 1 plus `weight*P(predicted_usage)+history` for non-owned resources,
where `P(u)=1+4/(exp(1-u)+1)+0.2*max(0,u-1)`. Weights are node 1,
switch/matching/mux 2, opposite-mode conflict 8. Mux usage counts distinct peers;
ordinary resources use owner count +1. History is `0.9*h+overflow`, outside P.
Reuse and mode ownership follow the old test_ILP switch-based rules. No extra
free-node exception is applied to the search start set. History remains updated
at the existing AIR round-end point before pruning.

All remaining copied files are namespace-only copies. Preserve their ordering,
resource legality rules, search limits, dynamic boxes, sync equalization and
stopping policy. There is no VPR present-factor scheduling.
Unused standalone CLI/main/net adapter files are not copied.

Verification (from PR_tool):

```bash
python3 algorithm/test_ILP/rrr_air/verify_snapshot.py
xmake build route_ilp_unit
./output/route_ilp_unit
xmake build test_ILP
```
