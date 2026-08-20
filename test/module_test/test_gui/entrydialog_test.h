#pragma once

#include <QObject>

class EntryDialogTest final : public QObject {
    Q_OBJECT

private slots:
    void showsLoadConfigAction();
    void loadConfigAcceptsSelectedCase5Directory();
    void escapeCancelsWithoutSelectingAConfig();
    void placeProgressChartKeepsUserScrollPosition();
};
