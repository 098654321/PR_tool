#include "interaction_test.h"

#include <widget/frame/entrydialog.h>
#include <widget/frame/graphicsview.h>
#include <widget/frame/placeprogresschart.h>

#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QGraphicsRectItem>
#include <QGraphicsScene>
#include <QPushButton>
#include <QScrollBar>
#include <QSignalSpy>
#include <QTimer>
#include <QtTest/QTest>

void InteractionTest::entryDialogAcceptsCase5FromFilePicker() {
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
        QMetaObject::invokeMethod(fileDialog, "accept");
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

void InteractionTest::escapeCancelsEntryDialogWithoutSelectingConfig() {
    PR_tool::widget::EntryDialog dialog;
    dialog.show();

    QTRY_VERIFY(dialog.isVisible());
    QTest::keyClick(&dialog, Qt::Key_Escape);
    QTRY_COMPARE(dialog.QDialog::result(), QDialog::Rejected);
    QVERIFY(!dialog.getResult().has_value());
}

void InteractionTest::placeProgressChartKeepsManualScrollPosition() {
    PR_tool::widget::PlaceProgressChart chart;
    chart.resize(200, 112);
    chart.show();

    auto* scrollBar = chart.horizontalScrollBar();
    for (int sample = 0; sample < 40; ++sample) {
        chart.append(sample + 1);
    }
    QTRY_VERIFY(scrollBar->maximum() > scrollBar->minimum());
    QTRY_COMPARE(scrollBar->value(), scrollBar->maximum());

    scrollBar->setFocus();
    QTest::keyClick(scrollBar, Qt::Key_Home);
    QTRY_COMPARE(scrollBar->value(), scrollBar->minimum());
    chart.append(41);
    QTRY_COMPARE(scrollBar->value(), scrollBar->minimum());
}

void InteractionTest::graphicsViewAllowsSelectionUntilLookbackLocksIt() {
    QGraphicsScene scene;
    PR_tool::widget::GraphicsView view;
    view.setScene(&scene);
    view.resize(300, 300);
    auto* item = scene.addRect(QRectF{-20.0, -20.0, 40.0, 40.0});
    item->setFlag(QGraphicsItem::ItemIsSelectable);
    view.centerOn(item);
    view.show();
    QTRY_VERIFY(view.isVisible());

    const auto itemCenter = view.mapFromScene(item->sceneBoundingRect().center());
    QTest::mouseClick(view.viewport(), Qt::LeftButton, Qt::NoModifier, itemCenter);
    QTRY_VERIFY(item->isSelected());

    item->setSelected(false);
    view.setLookbackLocked(true);
    QTest::mouseClick(view.viewport(), Qt::LeftButton, Qt::NoModifier, itemCenter);
    QTRY_VERIFY(!item->isSelected());
}
