#pragma once

#include <QCheckBox>
#include <QDialog>
#include <QLineEdit>
#include <QString>

namespace PR_tool::widget {

class ControlBitExportDialog : public QDialog {
    Q_OBJECT

public:
    explicit ControlBitExportDialog(QWidget* parent = nullptr);

    auto outputDir() const -> QString;
    auto simplify() const -> bool;

private slots:
    void onBrowse();

private:
    QLineEdit* _pathEdit {nullptr};
    QCheckBox* _simplifyCheck {nullptr};
};

}  // namespace PR_tool::widget
