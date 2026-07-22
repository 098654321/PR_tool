# GUI Control Bit Export Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix toolbar control-bit export to pick an output root directory (CLI `-o` semantics), optional simplify checkbox, and success path feedback for `regnamecontrolbit_4part/`.

**Architecture:** Add `ControlBitExportDialog` under `source/widget/frame/`; rewire `Window::generateControlBitAs` to use it and pass `simplify` into existing `parse::write_control_bits`. No Writer changes.

**Tech Stack:** Qt6 widgets (QDialog, QLineEdit, QCheckBox, QFileDialog, QMessageBox), existing `parse::write_control_bits`.

**Spec:** `docs/superpowers/specs/2026-07-21-gui-controlbit-export-design.md`

---

### Task 1: `ControlBitExportDialog`

**Files:**
- Create: `source/widget/frame/controlbitexportdialog.h`
- Create: `source/widget/frame/controlbitexportdialog.cc`

- [ ] **Step 1: Add header**

```cpp
#pragma once

#include <QCheckBox>
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QString>

namespace PR_tool::widget {

class ControlBitExportDialog : public QDialog {
    Q_OBJECT

public:
    explicit ControlBitExportDialog(QWidget* parent = nullptr);

    auto outputDir() const -> QString;
    auto simplify() const -> bool;

private slots:
    void onBrowse();

private:
    QLineEdit* _pathEdit {nullptr};
    QCheckBox* _simplifyCheck {nullptr};
};

}  // namespace PR_tool::widget
```

- [ ] **Step 2: Implement dialog**

Layout: title-friendly window title e.g. `"导出控制位"`; horizontal row (path edit + Browse…); checkbox text `"智能简化寄存器输出"` unchecked; `QDialogButtonBox` Ok|Cancel.

- Path default: `QDir::currentPath()`
- Browse: `QFileDialog::getExistingDirectory(this, "选择输出目录")`; if non-empty, set path edit
- Style: optional light button stylesheet consistent with `EntryDialog` (not required)

- [ ] **Step 3: Build GUI target**

```bash
xmake build PR_tool
```

Expected: compiles; new `.cc` picked up via `source/widget/frame/**.cc`.

- [ ] **Step 4: Commit**

```bash
git add source/widget/frame/controlbitexportdialog.h source/widget/frame/controlbitexportdialog.cc
git commit -m "feat(gui): add ControlBitExportDialog for directory + simplify"
```

---

### Task 2: Wire `generateControlBitAs`

**Files:**
- Modify: `source/widget/window.cc` (`generateControlBitAs`, includes)

- [ ] **Step 1: Replace body**

Include `widget/frame/controlbitexportdialog.h` (or `./frame/...` per local include style).

```cpp
void Window::generateControlBitAs() try {
    assert(this->_finishPR == true);

    ControlBitExportDialog dialog{this};
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    auto output_root = dialog.outputDir().trimmed();
    if (output_root.isEmpty()) {
        QMessageBox::warning(this, "导出控制位", "请选择输出目录");
        return;
    }

    const bool simplify = dialog.simplify();
    parse::connect_registers(this->_interposer.get(), this->_basedie.get(), 0);
    parse::write_control_bits(
        this->_interposer.get(),
        output_root.toStdString(),
        0,
        simplify,
        this->_register_map);

    const auto out_dir =
        QDir{output_root}.filePath(QStringLiteral("regnamecontrolbit_4part"));
    QMessageBox::information(
        this,
        "导出控制位",
        QStringLiteral("已写出控制位到：\n%1").arg(out_dir));
}
QMESSAGEBOX_REPORT_EXCEPTION("Generate control bit file")
```

Remove `TODO(split-output)` and old `getSaveFileName` / Cancel information box.

Add `#include <QDir>` if not already present.

- [ ] **Step 2: Build**

```bash
xmake build PR_tool
```

Expected: success.

- [ ] **Step 3: Commit**

```bash
git add source/widget/window.cc
git commit -m "feat(gui): export control bits to output root with optional simplify"
```

---

### Task 3: Docs cleanup

**Files:**
- Modify: `TODO.md` — remove or mark done the GUI `generateControlBitAs` deferred row

- [ ] **Step 1: Update TODO.md**

Under “仍待后续”, delete the GUI row (or note completed with link to this spec).

- [ ] **Step 2: Commit**

```bash
git add TODO.md
git commit -m "docs: mark GUI controlbit export TODO resolved"
```

---

### Task 4: Manual verification checklist

No automated test. Operator checklist:

- [ ] P&R finish → toolbar save → cancel → no message, no files
- [ ] Pick dir, simplify off → `{dir}/regnamecontrolbit_4part/` four full files; success box shows that path
- [ ] Simplify on → same dir tree, fewer lines (defaults omitted)
- [ ] Empty path + OK → warning only

---

## Spec coverage

| Spec item | Task |
|-----------|------|
| Directory root = CLI `-o` | 1–2 |
| Checkbox 智能简化… default off | 1–2 |
| Success path message | 2 |
| Silent cancel | 2 |
| Remove GUI TODO | 3 |
| Manual test | 4 |
| Non-goals (readback, File Save, etc.) | out of plan |
