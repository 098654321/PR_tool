#pragma once

#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <optional>

namespace PR_tool::widget {

    class EntryDialog : public QDialog {
        Q_OBJECT

    public:
        explicit EntryDialog(QWidget *parent = nullptr);
        auto getResult() const -> std::optional<QString>;

    private slots:
        void onOpenExistingProject();

    private:
        std::optional<QString> result {};
    };

}