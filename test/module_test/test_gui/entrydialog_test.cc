#if defined(__aarch64__)
#include <arm_acle.h>
#endif

#include <QtTest/QTest>

#include "entrydialog_test.h"
#include <widget/frame/entrydialog.h>
#include <widget/frame/placeprogresschart.h>

#include <QPushButton>
#include <QScrollBar>

void EntryDialogTest::showsLoadConfigAction() {
    PR_tool::widget::EntryDialog dialog;
    dialog.show();

    QTRY_VERIFY(dialog.isVisible());
    QCOMPARE(dialog.size(), QSize(400, 240));
    QVERIFY(!dialog.getResult().has_value());

    auto* loadConfigButton = dialog.findChild<QPushButton*>(QStringLiteral("PrimaryCta"));
    QVERIFY(loadConfigButton != nullptr);
    QCOMPARE(loadConfigButton->text(), QStringLiteral("Load Config"));
    QCOMPARE(loadConfigButton->accessibleName(), QStringLiteral("Load Config"));
    QCOMPARE(loadConfigButton->accessibleDescription(), QStringLiteral("Load an existing config directory"));
}

void EntryDialogTest::escapeCancelsWithoutSelectingAConfig() {
    PR_tool::widget::EntryDialog dialog;
    dialog.show();

    QTRY_VERIFY(dialog.isVisible());
    QTest::keyClick(&dialog, Qt::Key_Escape);
    QTRY_COMPARE(dialog.QDialog::result(), QDialog::Rejected);
    QVERIFY(!dialog.getResult().has_value());
}

void EntryDialogTest::placeProgressChartKeepsUserScrollPosition() {
    PR_tool::widget::PlaceProgressChart chart;
    chart.resize(200, 112);
    chart.show();

    auto* scrollBar = chart.horizontalScrollBar();
    for (int sample = 0; sample < 40; ++sample) {
        chart.append(sample + 1);
    }
    QTRY_VERIFY(scrollBar->maximum() > scrollBar->minimum());
    QTRY_COMPARE(scrollBar->value(), scrollBar->maximum());

    scrollBar->setValue(scrollBar->minimum());
    QTRY_COMPARE(scrollBar->value(), scrollBar->minimum());

    chart.append(41);
    QTRY_COMPARE(scrollBar->value(), scrollBar->minimum());
}

QTEST_MAIN(EntryDialogTest)
