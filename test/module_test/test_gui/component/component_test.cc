#include "component_test.h"

#include <widget/frame/controlbitexportdialog.h>
#include <widget/frame/entrydialog.h>
#include <widget/frame/graphicsview.h>

#include <QCheckBox>
#include <QDir>
#include <QLineEdit>
#include <QPushButton>
#include <QtTest/QTest>

namespace {
constexpr qreal kEpsilon = 0.0001;
}

void ComponentTest::entryDialogPresentsLoadConfigContract() {
    PR_tool::widget::EntryDialog dialog;
    dialog.show();

    QTRY_VERIFY(dialog.isVisible());
    QCOMPARE(dialog.size(), QSize(400, 240));
    QVERIFY(!dialog.getResult().has_value());

    auto* loadConfigButton = dialog.findChild<QPushButton*>(QStringLiteral("PrimaryCta"));
    QVERIFY(loadConfigButton != nullptr);
    QCOMPARE(loadConfigButton->text(), QStringLiteral("Load Config"));
    QCOMPARE(loadConfigButton->accessibleName(), QStringLiteral("Load Config"));
    QCOMPARE(loadConfigButton->accessibleDescription(),
             QStringLiteral("Load an existing config directory"));
}

void ComponentTest::controlBitExportDialogProvidesOutputOptions() {
    PR_tool::widget::ControlBitExportDialog dialog;

    QCOMPARE(dialog.windowTitle(), QStringLiteral("Export Controlbits"));
    QVERIFY(dialog.isModal());
    QCOMPARE(dialog.outputDir(), QDir::currentPath());
    QVERIFY(!dialog.simplify());

    auto* pathEdit = dialog.findChild<QLineEdit*>();
    auto* simplifyCheck = dialog.findChild<QCheckBox*>();
    QVERIFY(pathEdit != nullptr);
    QVERIFY(simplifyCheck != nullptr);

    pathEdit->setText(QStringLiteral("/tmp/pr-tool-controlbits"));
    simplifyCheck->setChecked(true);
    QCOMPARE(dialog.outputDir(), QStringLiteral("/tmp/pr-tool-controlbits"));
    QVERIFY(dialog.simplify());
}

void ComponentTest::graphicsViewProvidesApplicationZoomAndLockState() {
    PR_tool::widget::GraphicsView view;

    QVERIFY(qAbs(view.transform().m11() - PR_tool::widget::GraphicsView::kDefaultScale)
            < kEpsilon);
    QVERIFY(view.isInteractive());
    QVERIFY(!view.isLookbackLocked());

    view.scale(2.0, 2.0);
    view.resetZoom();
    QVERIFY(qAbs(view.transform().m11() - PR_tool::widget::GraphicsView::kDefaultScale)
            < kEpsilon);

    view.setLookbackLocked(true);
    QVERIFY(view.isLookbackLocked());
}
