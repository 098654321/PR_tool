#pragma once

#include <QObject>

class ComponentTest final : public QObject {
    Q_OBJECT

private slots:
    void entryDialogPresentsLoadConfigContract();
    void controlBitExportDialogProvidesOutputOptions();
    void graphicsViewProvidesApplicationZoomAndLockState();
};
