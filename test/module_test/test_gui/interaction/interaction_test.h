#pragma once

#include <QObject>

class InteractionTest final : public QObject {
    Q_OBJECT

private slots:
    void entryDialogAcceptsCase5FromFilePicker();
    void escapeCancelsEntryDialogWithoutSelectingConfig();
    void placeProgressChartKeepsManualScrollPosition();
    void graphicsViewAllowsSelectionUntilLookbackLocksIt();
};
