# case12: PR_tool to 3DBlox package

Load this package from its directory with:

```tcl
source load_case12.tcl
```

`case12.3dbx` and its included `.3dbv` are standard 3DBlox.  Its external
Verilog is intentionally a minimal stub so 3DBlox does not build generic chip
nets for very large PR_tool designs; `prtool_route` automatically loads the
adjacent `case12_connectivity.v` instead.  That connectivity is standard
Verilog with PR_tool direction comments.  A net called `prsync__m<mode>__g<group>[i]`
is lane `i` of a non-overlapping synchronous group.  A scalar net containing
several `__g<group>_l<lane>` fragments is one physical multi-source/multi-sink
component that belongs to several groups; this occurs in case16 and prevents
incorrectly splitting its shared port.

The converted interposer geometry uses the provided hardware dimensions:
COB 100x100, channel gap 500 (short edge 50), TOB 400x200, and a substrate
edge margin of 300 microns.  This yields an interposer size of
7900.0 x 5500.0 microns.  Topdie placement is
derived from TOB array coordinates, and external/0/1 port physical positions
are computed from the COB-edge port pitch (0.3 micron) and written into the
generated standard physical files.  Bump maps are emitted for topdies and the
interposer's external I/O ports.  The original PR_tool configuration is copied
to `prtool_reference/` for `prtool_verify_import`.  No metadata Tcl sidecar is
emitted.
