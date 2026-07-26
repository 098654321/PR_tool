#!/usr/bin/env python3
"""Convert PR_tool JSON cases into a 3DBlox/OpenROAD-loadable package.

The generated package uses standard 3DBlox, Verilog, LEF, DEF and bump-map
files. Positive connection groups become named synchronous Verilog nets; no
metadata Tcl sidecar is emitted.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
from collections import defaultdict
from pathlib import Path
from typing import Any


DEFAULT_INPUT = Path("/Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool/test/config")
DEFAULT_OUTPUT = Path("/Users/jiaheng/FDU_files/Tao_group/PR_tool/PR_tool/test/config_3dblox")

# Interposer geometry (microns), per the user-provided hardware dimensions.
COB_ROWS = 9
COB_COLS = 12
COB_SIZE = 100.0
CHANNEL_LONG = 500.0
CHANNEL_SHORT = 50.0
TOB_PARALLEL = 200.0
TOB_PERP = 400.0
EDGE_MARGIN = 300.0
PORT_PITCH = 0.3

# Topdie package assumptions not specified in the PR_tool inputs.  They only
# affect the synthetic 3D placement of topdie instances above TOBs.
TOPDIE_WIDTH = TOB_PERP
TOPDIE_HEIGHT = TOB_PARALLEL
TOPDIE_THICKNESS = 20.0
INTERPOSER_THICKNESS = 50.0
MICROBUMP_GAP = 1.0

INTERPOSER_WIDTH = 2 * EDGE_MARGIN + COB_COLS * COB_SIZE + (COB_COLS - 1) * CHANNEL_LONG
INTERPOSER_HEIGHT = 2 * EDGE_MARGIN + COB_ROWS * COB_SIZE + (COB_ROWS - 1) * CHANNEL_LONG
PORT_SPAN = (128 - 1) * PORT_PITCH
PORT_OFFSET = (COB_SIZE - PORT_SPAN) / 2.0


def read_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def write_text(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def verilog_id(value: str, prefix: str) -> str:
    """Return a portable simple Verilog identifier (and keep it deterministic)."""
    result = re.sub(r"[^A-Za-z0-9_$]", "_", value)
    if not result or not re.match(r"[A-Za-z_]", result):
        result = prefix + result
    return result


def net_name_token(value: str) -> str:
    """Keep a mode/group token readable after its fixed m/g prefix."""
    result = re.sub(r"[^A-Za-z0-9_$]", "_", value)
    return result or "0"


def tcl_brace(value: Any) -> str:
    """Quote the simple scalar names used by these configs for a Tcl word."""
    text = str(value)
    if "\n" in text or "\r" in text:
        raise ValueError("newlines are not supported in PR_tool metadata keys")
    return "{" + text.replace("\\", "\\\\").replace("}", "\\}").replace("{", "\\{") + "}"


class UnionFind:
    def __init__(self) -> None:
        self.parent: dict[str, str] = {}

    def find(self, item: str) -> str:
        self.parent.setdefault(item, item)
        if self.parent[item] != item:
            self.parent[item] = self.find(self.parent[item])
        return self.parent[item]

    def union(self, left: str, right: str) -> None:
        left_root, right_root = self.find(left), self.find(right)
        if left_root != right_root:
            self.parent[right_root] = left_root


def normalize_connections(data: Any) -> list[tuple[str, str, list[list[str]]]]:
    """Accept current flat files and the documented optional mode hierarchy."""
    result: list[tuple[str, str, list[list[str]]]] = []
    if all(isinstance(value, list) for value in data.values()):
        for group, pairs in data.items():
            result.append(("0", str(group), pairs))
        return result
    for mode, groups in data.items():
        if not isinstance(groups, dict):
            raise ValueError(f"connections mode {mode!r} is not an object")
        for group, pairs in groups.items():
            result.append((str(mode), str(group), pairs))
    return result


def scalar(value: Any) -> str:
    if value is None:
        return "null"
    if value is True:
        return "true"
    if value is False:
        return "false"
    return str(value)


def emit_property_tree(lines: list[str], parent: str, data: Any, serial: list[int]) -> None:
    """Emit a lossless JSON value using nested standard ODB string properties."""
    if isinstance(data, dict):
        iterator = data.items()
        tag = "object"
    elif isinstance(data, list):
        iterator = ((str(index), value) for index, value in enumerate(data))
        tag = "array"
    else:
        raise TypeError("emit_property_tree needs a container")
    for key, value in iterator:
        serial[0] += 1
        child = f"p{serial[0]}"
        if isinstance(value, (dict, list)):
            lines.append(f"set {child} [prtool_prop ${parent} {tcl_brace(key)} {tcl_brace('object' if isinstance(value, dict) else 'array')}]")
            emit_property_tree(lines, child, value, serial)
        else:
            lines.append(f"prtool_prop ${parent} {tcl_brace(key)} {tcl_brace(scalar(value))}")


def connected_components(groups: list[tuple[str, str, list[list[str]]]]) -> tuple[list[set[str]], dict[str, set[tuple[str, str]]]]:
    union_find = UnionFind()
    membership: dict[str, set[tuple[str, str]]] = defaultdict(set)
    for mode, group, pairs in groups:
        for pair in pairs:
            if not isinstance(pair, list) or len(pair) != 2:
                raise ValueError(f"connection {mode}/{group} contains a non-pair: {pair!r}")
            left, right = map(str, pair)
            union_find.union(left, right)
    components: dict[str, set[str]] = defaultdict(set)
    for endpoint in union_find.parent:
        components[union_find.find(endpoint)].add(endpoint)
    for mode, group, pairs in groups:
        for left, right in pairs:
            root = union_find.find(str(left))
            if group != "-1":
                membership[root].add((mode, group))
    ordered = sorted(components.values(), key=lambda component: tuple(sorted(component)))
    return ordered, membership


def bmap_lines(port_map: dict[str, str]) -> str:
    lines: list[str] = []
    for index, (original, port) in enumerate(sorted(port_map.items())):
        # Instance names and positions are unique even if the original pin_map
        # deliberately maps several logical signals to the same bump number.
        x, y = index % 32, index // 32
        lines.append(f"bump_{index} PRTOOL_BUMP {x}.0 {y}.0 {port} {port}")
    return "\n".join(lines) + ("\n" if lines else "")


def cob_origin(row: int, col: int) -> tuple[float, float]:
    return (
        EDGE_MARGIN + col * (COB_SIZE + CHANNEL_LONG),
        EDGE_MARGIN + row * (COB_SIZE + CHANNEL_LONG),
    )


def tob_origin(tob_row: int, tob_col: int) -> tuple[float, float]:
    # TOB(row,col) is anchored on the lower COB of the pair ((2r,3c),(2r+1,3c)).
    # It crosses the vertical channel between those two COBs, is symmetric with
    # respect to the channel in the left/right direction, and matches the user
    # calibration point TOB(0,0)=(150,550) when EDGE_MARGIN=300.
    cob_x, cob_y = cob_origin(2 * tob_row, 3 * tob_col)
    return (
        cob_x - (TOB_PERP - COB_SIZE) / 2.0,
        cob_y + COB_SIZE + (CHANNEL_LONG - TOB_PARALLEL) / 2.0,
    )


def topdie_origin_from_tob(tob_row: int, tob_col: int) -> tuple[float, float]:
    return tob_origin(tob_row, tob_col)


def clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, value))


def boundary_port_xy(coord: dict[str, Any]) -> tuple[float, float]:
    direction = coord["dir"]
    port_index = int(coord["index"])
    if direction == "vert":
        col = clamp(int(coord["col"]), 0, COB_COLS - 1)
        top_side = int(coord["row"]) >= COB_ROWS
        row = COB_ROWS - 1 if top_side else 0
        cob_x, cob_y = cob_origin(row, col)
        x = cob_x + COB_SIZE - PORT_OFFSET - port_index * PORT_PITCH
        y = cob_y + COB_SIZE if top_side else cob_y
        return x, y
    row = clamp(int(coord["row"]), 0, COB_ROWS - 1)
    right_side = int(coord["col"]) >= COB_COLS
    col = COB_COLS - 1 if right_side else 0
    cob_x, cob_y = cob_origin(row, col)
    x = cob_x + COB_SIZE if right_side else cob_x
    y = cob_y + COB_SIZE - PORT_OFFSET - port_index * PORT_PITCH
    return x, y


TECH_LEF = """VERSION 5.8 ;
BUSBITCHARS \"[]\" ;
DIVIDERCHAR \"/\" ;
UNITS
  DATABASE MICRONS 1000 ;
END UNITS
LAYER metal1
  TYPE ROUTING ;
  DIRECTION HORIZONTAL ;
  PITCH 1.0 ;
  WIDTH 0.2 ;
  SPACING 0.2 ;
END metal1
END LIBRARY
"""

BUMP_LEF = """VERSION 5.8 ;
MACRO PRTOOL_BUMP
  CLASS COVER BUMP ;
  ORIGIN 0 0 ;
  SIZE 1 BY 1 ;
  PIN PAD
    DIRECTION INOUT ;
    USE SIGNAL ;
    PORT
      LAYER metal1 ;
        RECT 0 0 1 1 ;
    END
  END PAD
END PRTOOL_BUMP
END LIBRARY
"""


def make_interposer_macros_lef() -> str:
    return f"""VERSION 5.8 ;
MACRO PRTOOL_COB
  CLASS BLOCK ;
  ORIGIN 0 0 ;
  SIZE {COB_SIZE:.1f} BY {COB_SIZE:.1f} ;
END PRTOOL_COB

MACRO PRTOOL_TOB
  CLASS BLOCK ;
  ORIGIN 0 0 ;
  SIZE {TOB_PERP:.1f} BY {TOB_PARALLEL:.1f} ;
END PRTOOL_TOB

MACRO PRTOOL_HCHAN
  CLASS BLOCK ;
  ORIGIN 0 0 ;
  SIZE {CHANNEL_LONG:.1f} BY {CHANNEL_SHORT:.1f} ;
END PRTOOL_HCHAN

MACRO PRTOOL_VCHAN
  CLASS BLOCK ;
  ORIGIN 0 0 ;
  SIZE {CHANNEL_SHORT:.1f} BY {CHANNEL_LONG:.1f} ;
END PRTOOL_VCHAN

END LIBRARY
"""


def make_interposer_def(
    case_name: str,
    external_ports: dict[str, Any],
    ports_01: dict[str, Any],
    net_names: dict[str, str],
) -> str:
    dbu = 1000

    def db(value: float) -> int:
        return round(value * dbu)

    components: list[str] = []
    for row in range(COB_ROWS):
        for col in range(COB_COLS):
            x, y = cob_origin(row, col)
            components.append(f"- COB_{row}_{col} PRTOOL_COB + PLACED ( {db(x)} {db(y)} ) N ;")

    for row in range(COB_ROWS):
        for col in range(COB_COLS - 1):
            x, y = cob_origin(row, col)
            hx = x + COB_SIZE
            hy = y + (COB_SIZE - CHANNEL_SHORT) / 2.0
            components.append(f"- HCHAN_{row}_{col + 1} PRTOOL_HCHAN + PLACED ( {db(hx)} {db(hy)} ) N ;")

    for row in range(COB_ROWS - 1):
        for col in range(COB_COLS):
            x, y = cob_origin(row, col)
            vx = x + (COB_SIZE - CHANNEL_SHORT) / 2.0
            vy = y + COB_SIZE
            components.append(f"- VCHAN_{row + 1}_{col} PRTOOL_VCHAN + PLACED ( {db(vx)} {db(vy)} ) N ;")

    for tob_row in range(4):
        for tob_col in range(4):
            x, y = tob_origin(tob_row, tob_col)
            components.append(f"- TOB_{tob_row}_{tob_col} PRTOOL_TOB + PLACED ( {db(x)} {db(y)} ) N ;")

    pins: list[str] = []
    for name, spec in sorted(external_ports.items()):
        x, y = boundary_port_xy(spec["coord"])
        pins.append(
            f"- {name} + NET {net_names[name]} + DIRECTION INOUT + USE SIGNAL + FIXED ( {db(x)} {db(y)} ) N + LAYER metal1 ( 0 0 ) ( 0 0 ) ;"
        )
    for polarity in ("pose", "nege"):
        for key, spec in sorted(ports_01.get(polarity, {}).items(), key=lambda item: int(item[0])):
            x, y = boundary_port_xy(spec)
            pins.append(
                f"- {polarity}_{key} + NET {polarity}_{key} + DIRECTION INOUT + USE SIGNAL + FIXED ( {db(x)} {db(y)} ) N + LAYER metal1 ( 0 0 ) ( 0 0 ) ;"
            )

    return "\n".join([
        "VERSION 5.8 ;",
        'DIVIDERCHAR "/" ;',
        'BUSBITCHARS "[]" ;',
        f"DESIGN {case_name}_interposer ;",
        f"UNITS DISTANCE MICRONS {dbu} ;",
        f"DIEAREA ( 0 0 ) ( {db(INTERPOSER_WIDTH)} {db(INTERPOSER_HEIGHT)} ) ;",
        "",
        f"COMPONENTS {len(components)} ;",
        *components,
        "END COMPONENTS",
        "",
        f"PINS {len(pins)} ;",
        *pins,
        "END PINS",
        "",
        "END DESIGN",
        "",
    ])


def make_3dbv(case_name: str, chiplets: dict[str, dict[str, Any]]) -> str:
    lines = ["Header:", '  version: "3.0"', "  unit: micron", "  precision: 1000", "", "ChipletDef:"]
    for chiplet, info in sorted(chiplets.items()):
        lef_files = ", ".join(info.get("lef_files", [f"{case_name}_bump.lef"]))
        lines.extend([
            f"  {chiplet}:",
            f"    type: {info.get('type', 'die')}",
            f"    design_area: [{info['width']:.1f}, {info['height']:.1f}]",
            "    shrink: 1.0",
            "    tsv: false",
            f"    thickness: {info['thickness']:.1f}",
            "    offset: [0.0, 0.0]",
            "    regions:",
            "      front:",
            "        side: front",
            f"        coords: [[0.0, 0.0], [{info['width']:.1f}, 0.0], [{info['width']:.1f}, {info['height']:.1f}], [0.0, {info['height']:.1f}]]",
            "        layer: metal1",
            "    external:",
            f"      APR_tech_file: [{case_name}_tech.lef]",
            f"      LEF_file: [{lef_files}]",
        ])
        if info.get("def_file"):
            lines.append(f"      DEF_file: {info['def_file']}")
        if info.get("bmap"):
            # bmap belongs under the front region, before layer; recreate the
            # two lines in their standard order for parsers that preserve YAML.
            layer_line = lines.pop()
            lines.insert(len(lines) - 3, f"        bmap: {info['bmap']}")
            lines.append(layer_line)
    return "\n".join(lines) + "\n"


def make_3dbx(
    case_name: str,
    instance_types: dict[str, str],
    instances: dict[str, Any],
    inst_ids: dict[str, str],
) -> str:
    lines = [
        "Header:",
        '  version: "3.0"',
        "  unit: micron",
        "  precision: 1000",
        "  include:",
        f"    - {case_name}.3dbv",
        "",
        "Design:",
        f"  name: {case_name}_3dblox",
        "  external:",
        # Current OpenROAD parser reads this standard's list field as a scalar.
        f"    verilog_file: {case_name}_connectivity.v",
        "",
        "ChipletInst:",
    ]
    for instance, chiplet in sorted(instance_types.items()):
        lines.extend([f"  {instance}:", f"    reference: {chiplet}"])
    lines.extend(["", "Stack:"])
    original_by_id = {value: key for key, value in inst_ids.items()}
    for instance in sorted(instance_types):
        if instance == "prtool_interposer":
            loc, z, orient = (0.0, 0.0), 0.0, "R0"
        elif instance == "prtool_boundary":
            loc, z, orient = (0.0, 0.0), 0.0, "R0"
        else:
            original = original_by_id[instance]
            coord = instances[original]["coord"]
            loc = topdie_origin_from_tob(int(coord["row"]), int(coord["col"]))
            # With MZ, the local front surface at z=TOPDIE_THICKNESS maps to
            # loc.z - TOPDIE_THICKNESS, i.e. one microbump gap over the
            # interposer's unflipped front surface at z=INTERPOSER_THICKNESS.
            z = INTERPOSER_THICKNESS + MICROBUMP_GAP + TOPDIE_THICKNESS
            orient = "MZ"
        lines.extend([
            f"  {instance}:",
            f"    loc: [{loc[0]:.1f}, {loc[1]:.1f}]",
            f"    z: {z:.1f}",
            f"    orient: {orient}",
        ])
    lines.extend(["", "Connection:"])
    for original, instance in sorted(inst_ids.items()):
        lines.extend([
            f"  {instance}_to_interposer:",
            f"    top: {instance}.regions.front",
            "    bot: prtool_interposer.regions.front",
            f"    thickness: {MICROBUMP_GAP:.1f}",
        ])
    return "\n".join(lines) + "\n"


def make_metadata(case_name: str, source: dict[str, Any], nets: list[dict[str, Any]]) -> str:
    lines = [
        "# Generated OpenROAD Tcl.  It uses ODB's built-in generic property API.",
        "set _prtool_db [ord::get_db]",
        "set _prtool_chip [$_prtool_db getChip]",
        "proc prtool_prop {parent name value} {",
        "  set old [odb::dbStringProperty_find $parent $name]",
        "  if {$old ne \"NULL\" && $old ne \"\"} { odb::dbProperty_destroy $old }",
        "  return [odb::dbStringProperty_create $parent $name $value]",
        "}",
        "set p0 [prtool_prop $_prtool_chip {X_PRTool} {schema=prtool-3dblox-v1}]",
        "set p1 [prtool_prop $p0 {input_json} {object}]",
    ]
    serial = [1]
    emit_property_tree(lines, "p1", source, serial)
    lines.append(f"set p{serial[0] + 1} [prtool_prop $p0 {{net_encoding}} {{array}}]")
    net_parent = f"p{serial[0] + 1}"
    serial[0] += 1
    emit_property_tree(lines, net_parent, nets, serial)
    lines.extend(["rename prtool_prop {}", "unset _prtool_db _prtool_chip", ""])
    return "\n".join(lines)


def make_readme(case_name: str) -> str:
    return f"""# {case_name}: PR_tool to 3DBlox package

Load this package from its directory with:

```tcl
source load_{case_name}.tcl
```

`{case_name}.3dbx` and its included `.3dbv` are standard 3DBlox.  The top
connectivity is standard Verilog.  A net called `prsync__m<mode>__g<group>[i]`
is lane `i` of a non-overlapping synchronous group.  A scalar net containing
several `__g<group>_l<lane>` fragments is one physical multi-source/multi-sink
component that belongs to several groups; this occurs in case16 and prevents
incorrectly splitting its shared port.

The converted interposer geometry uses the provided hardware dimensions:
COB 100x100, channel gap 500 (short edge 50), TOB 400x200, and a substrate
edge margin of 300 microns.  This yields an interposer size of
{INTERPOSER_WIDTH:.1f} x {INTERPOSER_HEIGHT:.1f} microns.  Topdie placement is
derived from TOB array coordinates, and external/0/1 port physical positions
are computed from the COB-edge port pitch (0.3 micron) and written into the
generated standard physical files.  Bump maps are emitted only for topdies;
the interposer's external I/O ports are DEF PINS.  No metadata Tcl sidecar is
emitted.
"""


def convert_case(source_dir: Path, output_root: Path, force: bool) -> None:
    case_name = source_dir.name
    config = read_json(source_dir / "config.json")
    topdies = read_json(source_dir / "topdies.json")
    instances = read_json(source_dir / "topdie_insts.json")
    external_ports = read_json(source_dir / "external_ports.json")
    ports_01 = read_json(source_dir / "01_ports.json")
    connections = read_json(source_dir / "connections.json")
    interposer_path = source_dir / "interposer.json"
    interposer = read_json(interposer_path) if interposer_path.exists() else {}
    register_path = source_dir / "reigster_adder.json"
    if not register_path.exists():
        register_path = source_dir / "register_adder.json"
    registers = read_json(register_path) if register_path.exists() else {}

    groups = normalize_connections(connections)
    components, group_membership = connected_components(groups)

    inst_ids = {name: verilog_id(name, "inst_") for name in instances}
    type_ids = {name: verilog_id(name, "chiplet_") for name in topdies}
    port_ids: dict[str, dict[str, str]] = {
        topdie: {name: verilog_id(name, "pin_") for name in spec["pin_map"]}
        for topdie, spec in topdies.items()
    }
    ext_ids = {name: verilog_id(name, "ext_") for name in external_ports}
    bare_endpoints = {endpoint for component in components for endpoint in component if "." not in endpoint}
    fixed_endpoints = sorted(bare_endpoints - set(external_ports))
    fixed_ids = {name: verilog_id(name, "fixed_") for name in fixed_endpoints}

    def endpoint_expr(endpoint: str) -> tuple[str, str]:
        if "." in endpoint:
            instance, port = endpoint.split(".", 1)
            if instance not in instances:
                raise ValueError(f"{case_name}: connection references unknown instance {instance!r}")
            topdie = instances[instance]["topdie"]
            if port not in port_ids[topdie]:
                raise ValueError(f"{case_name}: {endpoint!r} is absent from {topdie}.pin_map")
            return inst_ids[instance], port_ids[topdie][port]
        if endpoint in ext_ids:
            return "prtool_interposer", ext_ids[endpoint]
        return "prtool_interposer", fixed_ids[endpoint]

    # Each component receives one legal physical Verilog net.  Membership is
    # collected from all groups so fanout/multisink endpoints cannot be split.
    component_records: list[dict[str, Any]] = []
    lane_by_group: dict[tuple[str, str], list[int]] = defaultdict(list)
    for index, component in enumerate(components):
        # A component can span several groups through fanout/multisink ports.
        # Collect membership from its original directed pair edges, rather than
        # assuming the group ID is a one-to-one net identifier.
        member_groups = sorted({
            (mode, group)
            for mode, group, pairs in groups
            for left, right in pairs
            if group != "-1" and (left in component or right in component)
        })
        record = {"index": index, "endpoints": sorted(component), "groups": member_groups}
        component_records.append(record)
        for group in member_groups:
            lane_by_group[group].append(index)

    # A vector is safe only when every component of this group belongs to it
    # alone.  Otherwise its scalar name explicitly carries all memberships.
    vector_groups = {
        group for group, indices in lane_by_group.items()
        if all(component_records[index]["groups"] == [group] for index in indices)
    }
    lane_number = {group: {component: lane for lane, component in enumerate(indices)} for group, indices in lane_by_group.items()}
    for record in component_records:
        memberships = record["groups"]
        if not memberships:
            record["net"] = f"prasync__m0__n{record['index']}"
        elif len(memberships) == 1 and memberships[0] in vector_groups:
            mode, group = memberships[0]
            record["net"] = f"prsync__m{net_name_token(mode)}__g{net_name_token(group)}[{lane_number[memberships[0]][record['index']]}]"
        else:
            fragments = "__".join(
                f"m{net_name_token(mode)}__g{net_name_token(group)}_l{lane_number[(mode, group)][record['index']]}"
                for mode, group in memberships
            )
            record["net"] = f"prsync__{fragments}"

    endpoint_nets = {
        endpoint: record["net"]
        for record in component_records
        for endpoint in record["endpoints"]
    }

    output_dir = output_root / case_name
    if output_dir.exists():
        if not force:
            raise FileExistsError(f"output already exists: {output_dir} (use --force to replace this case)")
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)
    (output_dir / "bmaps").mkdir()

    source = {
        "config": config,
        "topdies": topdies,
        "topdie_insts": instances,
        "external_ports": external_ports,
        "ports_01": ports_01,
        "connections": connections,
        "interposer": interposer,
        "register_adder_source_file": register_path.name,
        "register_adder": registers,
    }

    # 3DBlox definitions plus physical interposer geometry derived from the
    # provided COB/TOB/channel dimensions.
    chiplets: dict[str, dict[str, Any]] = {}
    for topdie, ports in port_ids.items():
        chiplet = type_ids[topdie]
        bmap = f"bmaps/{chiplet}.bmap"
        chiplets[chiplet] = {
            "bmap": bmap,
            "width": TOPDIE_WIDTH,
            "height": TOPDIE_HEIGHT,
            "thickness": TOPDIE_THICKNESS,
            "lef_files": [f"{case_name}_bump.lef"],
        }
        write_text(output_dir / bmap, bmap_lines(ports))
    boundary_ports = dict(ext_ids)
    boundary_ports.update(fixed_ids)
    boundary_net_names = {
        original: endpoint_nets.get(original, f"prunconnected__p{index}")
        for index, original in enumerate(sorted(boundary_ports))
    }

    chiplets["PRTOOL_INTERPOSER"] = {
        "type": "rdl",
        "width": INTERPOSER_WIDTH,
        "height": INTERPOSER_HEIGHT,
        "thickness": INTERPOSER_THICKNESS,
        "lef_files": [f"{case_name}_interposer_macros.lef"],
        "def_file": f"{case_name}_interposer.def",
    }
    write_text(output_dir / f"{case_name}_tech.lef", TECH_LEF)
    write_text(output_dir / f"{case_name}_bump.lef", BUMP_LEF)
    write_text(output_dir / f"{case_name}_interposer_macros.lef", make_interposer_macros_lef())
    write_text(
        output_dir / f"{case_name}_interposer.def",
        make_interposer_def(case_name, external_ports, ports_01, boundary_net_names),
    )
    write_text(output_dir / f"{case_name}.3dbv", make_3dbv(case_name, chiplets))
    instance_types = {inst_ids[name]: type_ids[spec["topdie"]] for name, spec in instances.items()}
    instance_types["prtool_interposer"] = "PRTOOL_INTERPOSER"
    write_text(output_dir / f"{case_name}.3dbx", make_3dbx(case_name, instance_types, instances, inst_ids))

    # Verilog module declarations and all interposer ports.  Unconnected
    # boundary ports retain a named one-terminal net so DEF and connectivity
    # Verilog use the same name.
    verilog: list[str] = ["// Generated by convert_prtool_configs_to_3dblox.py", ""]
    for topdie, ports in sorted(port_ids.items()):
        verilog.append(f"module {type_ids[topdie]} ({', '.join(ports.values())});")
        for port in ports.values():
            verilog.append(f"  inout {port};")
        verilog.extend(["endmodule", ""])
    verilog.append(f"module PRTOOL_INTERPOSER ({', '.join(boundary_ports.values())});")
    for port in boundary_ports.values():
        verilog.append(f"  inout {port};")
    verilog.extend(["endmodule", "", f"module {case_name}_3dblox;"])
    for group in sorted(vector_groups):
        mode, group_id = group
        width = len(lane_by_group[group])
        verilog.append(f"  wire [{width - 1}:0] prsync__m{net_name_token(mode)}__g{net_name_token(group_id)};")
    for record in component_records:
        if not (len(record["groups"]) == 1 and record["groups"][0] in vector_groups):
            verilog.append(f"  wire {record['net']};")
    for net_name in sorted(set(boundary_net_names.values()) - set(endpoint_nets.values())):
        verilog.append(f"  wire {net_name};")
    for original, spec in sorted(instances.items()):
        topdie = spec["topdie"]
        conns = []
        for port, vport in sorted(port_ids[topdie].items()):
            endpoint = f"{original}.{port}"
            if endpoint in endpoint_nets:
                conns.append(f".{vport}({endpoint_nets[endpoint]})")
        verilog.append(f"  {type_ids[topdie]} {inst_ids[original]} ({', '.join(conns)});")
    boundary_conns = []
    for original, vport in sorted(boundary_ports.items()):
        boundary_conns.append(f".{vport}({boundary_net_names[original]})")
    verilog.append(f"  PRTOOL_INTERPOSER prtool_interposer ({', '.join(boundary_conns)});")
    verilog.extend(["endmodule", ""])
    write_text(output_dir / f"{case_name}_connectivity.v", "\n".join(verilog))

    write_text(output_dir / f"load_{case_name}.tcl", f"cd [file dirname [info script]]\nread_3dbx {case_name}.3dbx\n")
    write_text(output_dir / "README.md", make_readme(case_name))


def parse_cases(text: str) -> list[int]:
    cases: set[int] = set()
    for token in text.split(","):
        if "-" in token:
            first, last = map(int, token.split("-", 1))
            cases.update(range(first, last + 1))
        else:
            cases.add(int(token))
    return sorted(cases)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--cases", default="1-16", help="comma/range list, e.g. 1-4,7")
    parser.add_argument("--force", action="store_true", help="replace already-generated case directories")
    args = parser.parse_args()
    for number in parse_cases(args.cases):
        source_dir = args.input / f"case{number}"
        if not source_dir.is_dir():
            raise FileNotFoundError(source_dir)
        convert_case(source_dir, args.output, args.force)
        print(f"converted {source_dir.name}")


if __name__ == "__main__":
    main()
