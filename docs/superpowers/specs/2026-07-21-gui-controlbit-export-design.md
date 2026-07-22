# GUI Control Bit Export (Directory + Simplify)

## Problem

After PR_tool product output switched to `{output}/regnamecontrolbit_4part/` (four REG files), GUI `Window::generateControlBitAs` still uses `QFileDialog::getSaveFileName` with a default `output.ctb` path. That path is passed to `parse::write_control_bits` as an **output root**, so the UX implies a single file while the implementation writes a directory tree.

File menu **Save / Save as** (`saveConfig` / `saveConfigAs`) are empty stubs for project config and do **not** write control bits. The only GUI control-bit write path is the toolbar action that uses the save icon and calls `generateControlBitAs`.

## Goals

1. Align GUI export with CLI `-o`: user picks an **output root directory**; Writer creates `regnamecontrolbit_4part/` under it.
2. Optional simplify (`-s`): checkbox **智能简化寄存器输出**, default **unchecked** (full output).
3. On success, show a `QMessageBox` that includes the path `{output_root}/regnamecontrolbit_4part`.
4. On cancel, **silent** close (no “Cancelled” dialog).

## Non-goals

- Controlbits readback / incremental warm-start (`load_controlbits`).
- `-c/--compare` adaptation.
- Multi-mode `mode_<m>/regnamecontrolbit_4part/` layout.
- Implementing File → Save / Save as (config).
- New automated Qt UI tests.
- Changing Writer four-file format or omit rules.

## Approach

**Custom dialog** (not bare `getExistingDirectory` + second popup; not reinterpreting `getSaveFileName`).

New `ControlBitExportDialog` under `source/widget/frame/`:

| Control | Behavior |
|---------|----------|
| Path `QLineEdit` | Default `QDir::currentPath()`; user may edit |
| Browse… | `QFileDialog::getExistingDirectory`; fills path |
| Checkbox | Label **智能简化寄存器输出**; default unchecked |
| OK / Cancel | Cancel → `reject`; caller returns silently |

Accessors (after `Accepted`): `outputDir() -> QString`, `simplify() -> bool`.

Do **not** reuse `LineEditWithButton` (its button is “Yes” / confirm-text semantics).

## Call site

`Window::generateControlBitAs`:

1. Show `ControlBitExportDialog`.
2. If not accepted → return (silent).
3. If path empty → `QMessageBox::warning` (“请选择输出目录”) → return.
4. `parse::connect_registers(interposer, basedie, 0)`.
5. `parse::write_control_bits(interposer, output_root, 0, simplify, _register_map)`.
6. Success: `QMessageBox::information` with `{output_root}/regnamecontrolbit_4part`.
7. Exceptions: keep existing `QMESSAGEBOX_REPORT_EXCEPTION("Generate control bit file")`.

Remove the `TODO(split-output)` comment at this call site once fixed. Update `TODO.md` GUI row accordingly.

## Error / edge cases

| Case | Behavior |
|------|----------|
| Cancel | Silent return |
| Empty path | Warning; no write |
| Writer / map errors | Existing exception → critical box; map empty/missing same as CLI |
| Existing `regnamecontrolbit_4part/` | Overwrite without extra confirm (same as CLI; YAGNI) |

## Build

`xmake` already includes `source/widget/frame/**.cc` for GUI targets; new dialog sources are picked up automatically.

## Testing

Manual only:

1. After P&R, toolbar save → pick dir, simplify off → four full REG files under `regnamecontrolbit_4part/`.
2. Same with simplify on → fewer lines; omit rules unchanged vs CLI `-s`.
3. Success dialog shows correct `…/regnamecontrolbit_4part` path.
4. Cancel → no dialog, no write.

Writer / `-s` semantics remain covered by `test_writer` / check-controlbits-file; GUI is wiring only.

## Success criteria

- No `.ctb` / single-file save dialog for control bits.
- Artifacts always under `{chosen_root}/regnamecontrolbit_4part/`.
- Checkbox maps to `simplify_controlbits`; default off = full.
- Success message includes that directory path.
