#include "./controlbitexportdialog.h"

#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace PR_tool::widget {

ControlBitExportDialog::ControlBitExportDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("导出控制位"));
    setModal(true);

    auto* layout = new QVBoxLayout{this};

    auto* path_label = new QLabel{QStringLiteral("输出目录（将在其下创建 regnamecontrolbit_4part/）："), this};
    layout->addWidget(path_label);

    auto* path_row = new QHBoxLayout{};
    _pathEdit = new QLineEdit{this};
    _pathEdit->setText(QDir::currentPath());
    auto* browse = new QPushButton{QStringLiteral("浏览…"), this};
    browse->setMaximumWidth(80);
    path_row->addWidget(_pathEdit);
    path_row->addWidget(browse);
    layout->addLayout(path_row);

    _simplifyCheck = new QCheckBox{QStringLiteral("智能简化寄存器输出"), this};
    _simplifyCheck->setChecked(false);
    layout->addWidget(_simplifyCheck);

    auto* buttons = new QDialogButtonBox{QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this};
    layout->addWidget(buttons);

    connect(browse, &QPushButton::clicked, this, &ControlBitExportDialog::onBrowse);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    resize(520, 160);
}

auto ControlBitExportDialog::outputDir() const -> QString {
    return _pathEdit->text();
}

auto ControlBitExportDialog::simplify() const -> bool {
    return _simplifyCheck->isChecked();
}

void ControlBitExportDialog::onBrowse() {
    auto dir = QFileDialog::getExistingDirectory(
        this,
        QStringLiteral("选择输出目录"),
        _pathEdit->text().isEmpty() ? QDir::currentPath() : _pathEdit->text());
    if (!dir.isEmpty()) {
        _pathEdit->setText(dir);
    }
}

}  // namespace PR_tool::widget
