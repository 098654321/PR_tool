#include "./netinfowidget.h"
#include "../item/netitem.h"
#include "../item/netpointitem.h"
#include "../item/pinitem.h"
#include <widget/frame/colorpickbutton.h>
#include <circuit/connection/connection.hh>
#include <debug/debug.hh>

#include <cassert>

#include <QLabel>
#include <QSpinBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QMessageBox>

#include "../schematictypography.h"

namespace PR_tool::widget::schematic {

    NetInfoWidget::NetInfoWidget(QWidget* parent) :
        QWidget{parent}
    {
        auto* thisLayout = new QVBoxLayout{this};
        thisLayout->setContentsMargins(4, 4, 4, 4);
        thisLayout->setSpacing(6);

        auto* title = new QLabel{QStringLiteral("NET"), this};
        SchematicTypography::applyInspectorTitle(title);
        thisLayout->addWidget(title);

        auto* line = new QFrame{this};
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        thisLayout->addWidget(line);

        auto* layout = new QGridLayout{};
        layout->setContentsMargins(4, 2, 4, 2);
        layout->setHorizontalSpacing(10);
        layout->setVerticalSpacing(6);
        layout->setColumnStretch(1, 1);
        thisLayout->addLayout(layout);
        thisLayout->addStretch();

        auto addPropRow = [&](int row, const QString& text, QWidget* value) {
            auto* label = new QLabel{text, this};
            SchematicTypography::applyPropertyLabel(label);
            SchematicTypography::applyPropertyValue(value);
            layout->addWidget(label, row, 0);
            layout->addWidget(value, row, 1);
        };

        this->_beginPinLabel = new QLabel{this};
        this->_beginPinLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
        addPropRow(0, QStringLiteral("Begin"), this->_beginPinLabel);

        this->_endPinLabel = new QLabel{this};
        this->_endPinLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
        addPropRow(1, QStringLiteral("End"), this->_endPinLabel);

        this->_syncSpinBox = new QSpinBox{this};
        this->_syncSpinBox->setMinimum(-1);
        this->_syncSpinBox->setMaximum(32);
        this->_syncSpinBox->setMinimumHeight(30);
        addPropRow(2, QStringLiteral("Sync"), this->_syncSpinBox);

        this->_colorButton = new ColorPickerButton{this};
        this->_colorButton->setMinimumHeight(30);
        addPropRow(3, QStringLiteral("Color"), this->_colorButton);

        this->_widthSpinBox = new QSpinBox{this};
        this->_widthSpinBox->setMinimum(1);
        this->_widthSpinBox->setMaximum(20);
        this->_widthSpinBox->setMinimumHeight(30);
        addPropRow(4, QStringLiteral("Width"), this->_widthSpinBox);

        connect(this->_colorButton, &ColorPickerButton::colorChanged, this, [this](const QColor& color) {
            assert(this->_net != nullptr);
            emit this->netColorChanged(this->_net, color);
        });

        connect(this->_syncSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int sync) {
            assert(this->_net != nullptr);
            emit this->netSyncChanged(this->_net, sync);
        });

        connect(this->_widthSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int width) {
            assert(this->_net != nullptr);
            emit this->netWidthChanged(this->_net, width);
        });

        auto* removeButton = new QPushButton{QStringLiteral("Remove"), this};
        removeButton->setMinimumHeight(30);
        layout->addWidget(removeButton, 5, 0, 1, 2);

        connect(removeButton, &QPushButton::clicked, this, [this]() {
            auto response = QMessageBox::question(
                this,
                QStringLiteral("Confirm"),
                QStringLiteral("Do you want to delete this net?"),
                QMessageBox::Yes | QMessageBox::No);

            if (response == QMessageBox::Yes) {
                assert(this->_net != nullptr);
                emit this->removeNet(this->_net);
            }
        });
    }

    void NetInfoWidget::loadNet(NetItem* net) {
        this->_net = net;
        if (this->_net == nullptr) {
            debug::exception("Load a empty net into NetInfoWidget");
        }

        auto beginPoint = this->_net->beginPoint();
        auto beginPin = beginPoint->connectedPin();
        this->_beginPinLabel->setText(beginPin->toString());

        auto endPoint = this->_net->endPoint();
        auto endPin = endPoint->connectedPin();
        this->_endPinLabel->setText(endPin->toString());

        this->_syncSpinBox->setValue(this->_net->unwrap()->sync());

        this->_colorButton->setColor(this->_net->color());
        this->_widthSpinBox->setValue(static_cast<int>(this->_net->width()));
    }

    auto NetInfoWidget::currentNet() -> NetItem* {
        return this->_net;
    }

}
