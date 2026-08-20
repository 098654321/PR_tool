#if defined(__aarch64__)
#include <arm_acle.h>
#endif

#include <QtTest/QTest>

#include "entrydialog_test.h"
#include <widget/frame/entrydialog.h>
#include <widget/frame/placeprogresschart.h>

#include <QPushButton>
#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QScrollBar>
#include <QSignalSpy>
#include <QTimer>

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

void EntryDialogTest::loadConfigAcceptsSelectedCase5Directory() {
    QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    PR_tool::widget::EntryDialog dialog;
    const auto configDirectory = QFINDTESTDATA("../test/config/case5");
    QVERIFY2(!configDirectory.isEmpty(), "case5 test configuration must exist");

    auto* loadConfigButton = dialog.findChild<QPushButton*>(QStringLiteral("PrimaryCta"));
    QVERIFY(loadConfigButton != nullptr);

    QSignalSpy acceptedSpy(&dialog, &QDialog::accepted);
    bool fileDialogFound = false;
    QTimer fileDialogDriver;
    connect(&fileDialogDriver, &QTimer::timeout, this, [&]() {
        auto* fileDialog = qobject_cast<QFileDialog*>(QApplication::activeModalWidget());
        if (fileDialog == nullptr) {
            return;
        }

        fileDialogFound = true;
        fileDialog->setDirectory(configDirectory);
        fileDialog->selectFile(configDirectory);
        QVERIFY(QMetaObject::invokeMethod(fileDialog, "accept"));
        fileDialogDriver.stop();
    });
    QTimer watchdog;
    watchdog.setSingleShot(true);
    connect(&watchdog, &QTimer::timeout, this, []() {
        if (auto* modalDialog = qobject_cast<QDialog*>(QApplication::activeModalWidget())) {
            modalDialog->reject();
        }
    });
    fileDialogDriver.start(1);
    watchdog.start(2000);

    dialog.show();
    QTRY_VERIFY(dialog.isVisible());
    QTest::mouseClick(loadConfigButton, Qt::LeftButton);

    fileDialogDriver.stop();
    watchdog.stop();
    QVERIFY(fileDialogFound);
    QCOMPARE(acceptedSpy.count(), 1);
    QCOMPARE(dialog.QDialog::result(), QDialog::Accepted);
    QVERIFY(dialog.getResult().has_value());
    QCOMPARE(QDir::cleanPath(*dialog.getResult()), QDir::cleanPath(configDirectory));
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
