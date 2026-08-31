#include "./entrydialog.h"
#include "qlabel.h"

namespace PR_tool::widget {

    EntryDialog::EntryDialog(QWidget *parent)
        : QDialog(parent)
    {
        auto layout = new QVBoxLayout(this);
        layout->setContentsMargins(28, 28, 28, 28);
        layout->setSpacing(12);

        auto label = new QLabel{this};
        label->setText("Welcome to PR_tool");
        auto font = label->font();
        font.setPointSize(20);
        font.setBold(true);
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);
        layout->addWidget(label);

        auto flowHint = new QLabel{
            QStringLiteral(
                "Configure the design visually, complete place and route, and view the results."),
            this};
        flowHint->setWordWrap(true);
        flowHint->setAlignment(Qt::AlignCenter);
        flowHint->setStyleSheet(QStringLiteral("color: #86868b;"));
        layout->addWidget(flowHint);

        auto openButton = new QPushButton("Load Config", this);
        openButton->setObjectName(QStringLiteral("PrimaryCta"));
        openButton->setFixedHeight(40);
        openButton->setAccessibleName("Load Config");
        openButton->setAccessibleDescription("Load an existing config directory");
        layout->addWidget(openButton);

        connect(openButton, &QPushButton::clicked, this, &EntryDialog::onOpenExistingProject);

        this->setFixedSize(400, 240);
        this->setLayout(layout);
    }

    auto EntryDialog::getResult() const -> std::optional<QString> {
        return result;
    }

    void EntryDialog::onOpenExistingProject() {
        auto filePath = QFileDialog::getExistingDirectory(this, "Select Config path");
        if (!filePath.isEmpty()) {
            result.emplace(filePath);
            this->accept();
        }
    }

}
