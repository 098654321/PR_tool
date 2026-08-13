#include "./exportwidget.h"

#include "../item/exportitem.h"
#include "hardware/cob/cob.hh"
#include "hardware/interposer.hh"
#include <circuit/export/export.hh>
#include "hardware/track/trackcoord.hh"

#include <cassert>
#include <debug/debug.hh>

#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QSpinBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QMessageBox>

#include "../schematictypography.h"

namespace PR_tool::widget::schematic {

    constexpr int MIN_HEIGHT = 30;

    ExternalPortInfoWidget::ExternalPortInfoWidget(QWidget* parent) :
        QWidget{parent}
    {
        auto* thisLayout = new QVBoxLayout{this};
        thisLayout->setContentsMargins(8, 8, 8, 8);
        thisLayout->setSpacing(8);

        auto* title = new QLabel{QStringLiteral("EXTERNAL PORT"), this};
        SchematicTypography::applyInspectorTitle(title);
        thisLayout->addWidget(title);

        auto* line = new QFrame{this};
        line->setObjectName(QStringLiteral("SideHairline"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Plain);
        thisLayout->addWidget(line);

        auto* layout = new QGridLayout{};
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setHorizontalSpacing(8);
        layout->setVerticalSpacing(8);
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

        this->_nameEdit = new QLineEdit{this};
        this->_nameEdit->setMinimumHeight(MIN_HEIGHT);
        addPropRow(0, QStringLiteral("Name"), this->_nameEdit);

        connect(this->_nameEdit, &QLineEdit::editingFinished, this, [this]() {
            if (this->_externalPort == nullptr) {
                return;
            }
            emit this->externalPortRename(this->_externalPort, this->_nameEdit->text());
        });

        this->_rowSpinBox = new QSpinBox{this};
        this->_rowSpinBox->setMinimumHeight(MIN_HEIGHT);
        this->_rowSpinBox->setMinimum(0);
        this->_rowSpinBox->setMaximum(hardware::Interposer::COB_ARRAY_HEIGHT - 1);
        addPropRow(1, QStringLiteral("Row"), this->_rowSpinBox);

        this->_colSpinBox = new QSpinBox{this};
        this->_colSpinBox->setMinimumHeight(MIN_HEIGHT);
        this->_colSpinBox->setMinimum(0);
        this->_colSpinBox->setMaximum(hardware::Interposer::COB_ARRAY_WIDTH - 1);
        addPropRow(2, QStringLiteral("Column"), this->_colSpinBox);

        this->_dirComboBox = new QComboBox{this};
        this->_dirComboBox->setMinimumHeight(MIN_HEIGHT);
        this->_dirComboBox->addItem(QStringLiteral("Hori"));
        this->_dirComboBox->addItem(QStringLiteral("Vert"));
        addPropRow(3, QStringLiteral("Dir"), this->_dirComboBox);

        this->_indexSpinBox = new QSpinBox{this};
        this->_indexSpinBox->setMinimumHeight(MIN_HEIGHT);
        this->_indexSpinBox->setMinimum(0);
        this->_indexSpinBox->setMaximum(hardware::COB::INDEX_SIZE);
        addPropRow(4, QStringLiteral("Index"), this->_indexSpinBox);

        this->_setCoordButton = new QPushButton{QStringLiteral("Set Coord"), this};
        this->_setCoordButton->setMinimumHeight(MIN_HEIGHT);
        layout->addWidget(this->_setCoordButton, 5, 0, 1, 2);

        connect(this->_setCoordButton, &QPushButton::clicked, this, [this]() {
            auto coord = hardware::TrackCoord{
                this->_rowSpinBox->value(),
                this->_colSpinBox->value(),
                this->_dirComboBox->currentIndex() == 0
                    ? hardware::TrackDirection::Horizontal
                    : hardware::TrackDirection::Vertical,
                static_cast<std::usize>(this->_indexSpinBox->value())
            };
            emit this->externalPortSetCoord(this->_externalPort, coord);
        });

        auto* deleteButton = new QPushButton{QStringLiteral("Remove"), this};
        deleteButton->setMinimumHeight(MIN_HEIGHT);
        layout->addWidget(deleteButton, 6, 0, 1, 2);
        connect(deleteButton, &QPushButton::clicked, this, [this]() {
            auto response = QMessageBox::question(
                this,
                QStringLiteral("Confirm"),
                QStringLiteral("Do you want to delete this external port?"),
                QMessageBox::Yes | QMessageBox::No);

            if (response == QMessageBox::Yes) {
                assert(this->_externalPort != nullptr);
                emit this->removeExternalPort(this->_externalPort);
            }
        });
    }

    void ExternalPortInfoWidget::loadExternalPort(ExternalPortItem* eport) {
        this->_externalPort = eport;
        if (this->_externalPort == nullptr) {
            debug::exception("Load a empty externl port into ExPortInfoWidget");
        }

        this->_nameEdit->setText(eport->pin()->name());

        this->_rowSpinBox->setValue(eport->unwrap()->coord().row);
        this->_colSpinBox->setValue(eport->unwrap()->coord().col);
        this->_dirComboBox->setCurrentIndex(
            eport->unwrap()->coord().dir == hardware::TrackDirection::Horizontal ? 0 : 1
        );
        this->_indexSpinBox->setValue(eport->unwrap()->coord().index);
    }

    auto ExternalPortInfoWidget::currentExternalPort() -> ExternalPortItem* {
        return this->_externalPort;
    }
}
