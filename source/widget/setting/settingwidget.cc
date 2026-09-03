#include "./settingwidget.h"

#include <QLabel>
#include <QVBoxLayout>

namespace PR_tool::widget {

    SettingWidget::SettingWidget(QWidget* parent) :
        QWidget{parent}
    {
        auto* layout = new QVBoxLayout{this};
        layout->setContentsMargins(24, 24, 24, 24);
        layout->setSpacing(8);

        auto* title = new QLabel{QStringLiteral("Settings"), this};
        auto titleFont = title->font();
        titleFont.setPointSize(titleFont.pointSize() + 4);
        titleFont.setBold(true);
        title->setFont(titleFont);
        layout->addWidget(title);

        auto* note = new QLabel{
            QStringLiteral("Advanced settings are limited in this build."),
            this};
        note->setWordWrap(true);
        layout->addWidget(note);

        auto* styleNote = new QLabel{
            QStringLiteral("Appearance: Fusion + light chrome"),
            this};
        layout->addWidget(styleNote);

        auto* version = new QLabel{
            QStringLiteral("Version: —"),
            this};
        layout->addWidget(version);

        auto* tip = new QLabel{
            QStringLiteral("Tip: Ctrl+1–4 switch Schematic / Layout / View2D / View3D."),
            this};
        tip->setWordWrap(true);
        layout->addWidget(tip);

        auto* flow = new QLabel{
            QStringLiteral(
                "Typical flow: Schematic → Layout → Place & Route → View 2D/3D → Export"),
            this};
        flow->setWordWrap(true);
        layout->addWidget(flow);

        layout->addStretch(1);
    }

}
