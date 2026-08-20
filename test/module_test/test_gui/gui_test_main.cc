#include <QApplication>
#include <QSettings>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QtTest/QTest>

#include "component/component_test.h"
#include "interaction/interaction_test.h"
#include "workflow/workflow_test.h"

int main(int argc, char* argv[]) {
    QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    QStandardPaths::setTestModeEnabled(true);
    QApplication app{argc, argv};

    QTemporaryDir settingsDir;
    if (!settingsDir.isValid()) {
        return 2;
    }
    QSettings::setPath(QSettings::NativeFormat, QSettings::UserScope, settingsDir.path());

    ComponentTest componentTest;
    InteractionTest interactionTest;
    WorkflowTest workflowTest;
    int status = QTest::qExec(&componentTest, argc, argv);
    status |= QTest::qExec(&interactionTest, argc, argv);
    status |= QTest::qExec(&workflowTest, argc, argv);
    return status;
}
