#pragma once

#include <QObject>

class WorkflowTest final : public QObject {
    Q_OBJECT

private slots:
    void loadsCase5AndBrowsesDesignWithoutRouting();
};
