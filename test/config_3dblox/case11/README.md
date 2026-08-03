# case11: PR_tool to 3DBlox package

Load this package from its directory with:

```tcl
source load_case11.tcl
```

`case11.3dbx` and its included `.3dbv` are standard 3DBlox.  The top
connectivity is standard Verilog.  It uses one Verilog module per physical
topdie instance so every module port preserves that instance's PR_tool
connection direction: sources are `output`, sinks are `input`, and only
unconnected ports remain `inout`.  A net called `prsync__m<mode>__g<group>[i]`
is lane `i` of a non-overlapping synchronous group.  A scalar net containing
several `__g<group>_l<lane>` fragments is one physical multi-source/multi-sink
component that belongs to several groups; this occurs in case16 and prevents
incorrectly splitting its shared port.

The converted interposer geometry uses the provided hardware dimensions:
COB 100x100, channel gap 500 (short edge 50), TOB 400x200, and a substrate
edge margin of 300 microns.  This case uses a COB array of 9 x 13,
yielding an interposer size of 7900.0 x 5500.0 microns.
Topdie placement is derived from TOB array coordinates.  External/0/1 port
positions use the COB-edge port pitch (0.3 micron) and are written into the
interposer bump map referenced by `.3dbv`.  `read_3dbx` loads tech/bump LEF
plus bump maps only; `case11_interposer.def` and
`case11_interposer_macros.lef` are still emitted for later drawing but
are not referenced from `.3dbv`/`.3dbx`.  `register_adder.json` is copied from
the source configuration and must be loaded by `prt` before route.
