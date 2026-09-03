#include "workflow_test.h"

#include <widget/layout/layoutwidget.h>
#include <widget/schematic/schematicwidget.h>
#include <widget/window.h>

#include <QAction>
#include <QLabel>
#include <QStackedWidget>
#include <QtTest/QTest>

namespace {
auto actionNamed(PR_tool::widget::Window& window, const QString& text) -> QAction* {
    for (auto* action : window.findChildren<QAction*>()) {
        if (action->text() == text) {
            return action;
        }
    }
    return nullptr;
}
}

void WorkflowTest::loadsCase5AndBrowsesDesignWithoutRouting() {
    const auto configDirectory = QFINDTESTDATA("../test/config/case5");
    QVERIFY2(!configDirectory.isEmpty(), "case5 test configuration must exist");

    PR_tool::widget::Window window;
    window.resize(1280, 800);
    window.show();
    QTRY_VERIFY(window.isVisible());

    auto* stackedWidget = window.findChild<QStackedWidget*>();
    auto* schematic = window.findChild<PR_tool::widget::SchematicWidget*>();
    auto* layout = window.findChild<PR_tool::widget::LayoutWidget*>();
    auto* stageLabel = window.findChild<QLabel*>(QStringLiteral("StatusStage"));
    auto* schematicAction = actionNamed(window, QStringLiteral("Schematic"));
    auto* layoutAction = actionNamed(window, QStringLiteral("Layout"));
    auto* view2DAction = actionNamed(window, QStringLiteral("2D"));
    auto* view3DAction = actionNamed(window, QStringLiteral("3D"));
    auto* navigatorAction = actionNamed(window, QStringLiteral("Show Navigator"));
    auto* inspectorAction = actionNamed(window, QStringLiteral("Show Inspector"));
    QVERIFY(stackedWidget != nullptr);
    QVERIFY(schematic != nullptr);
    QVERIFY(layout != nullptr);
    QVERIFY(stageLabel != nullptr);
    QVERIFY(schematicAction != nullptr);
    QVERIFY(layoutAction != nullptr);
    QVERIFY(view2DAction != nullptr);
    QVERIFY(view3DAction != nullptr);
    QVERIFY(navigatorAction != nullptr);
    QVERIFY(inspectorAction != nullptr);

    window.loadConfigFromPath(configDirectory);
    QCOMPARE(stageLabel->text(), QStringLiteral("Stage: Design"));
    QVERIFY(!view2DAction->isEnabled());
    QVERIFY(!view3DAction->isEnabled());

    layoutAction->trigger();
    QTRY_COMPARE(stackedWidget->currentWidget(), static_cast<QWidget*>(layout));
    QVERIFY(layout->isInspectorVisible());
    inspectorAction->trigger();
    QTRY_VERIFY(!layout->isInspectorVisible());
    inspectorAction->trigger();
    QTRY_VERIFY(layout->isInspectorVisible());

    schematicAction->trigger();
    QTRY_COMPARE(stackedWidget->currentWidget(), static_cast<QWidget*>(schematic));

    QVERIFY(schematic->isNavigatorVisible());
    navigatorAction->trigger();
    QTRY_VERIFY(!schematic->isNavigatorVisible());
    navigatorAction->trigger();
    QTRY_VERIFY(schematic->isNavigatorVisible());

    QVERIFY(schematic->isInspectorVisible());
    inspectorAction->trigger();
    QTRY_VERIFY(!schematic->isInspectorVisible());
    inspectorAction->trigger();
    QTRY_VERIFY(schematic->isInspectorVisible());
}
