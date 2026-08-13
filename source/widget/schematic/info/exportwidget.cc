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

namespace PR_tool::widget::schematic {

    constexpr int MIN_HEIGHT = 30;

    ExternalPortInfoWidget::ExternalPortInfoWidget(QWidget* parent) :
        QWidget{parent}
    {
        auto* thisLayout = new QVBoxLayout{this};
        thisLayout->setContentsMargins(4, 4, 4, 4);
        thisLayout->setSpacing(6);

        auto* title = new QLabel{QStringLiteral("EXTERNAL PORT"), this};
        auto titleFont = title->font();
        titleFont.setBold(true);
        titleFont.setPointSize(titleFont.pointSize() + 1);
        title->setFont(titleFont);
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

        layout->addWidget(new QLabel{QStringLiteral("Name"), this}, 0, 0);
        this->_nameEdit = new QLineEdit{this};
        this->_nameEdit->setMinimumHeight(MIN_HEIGHT);
        layout->addWidget(this->_nameEdit, 0, 1);

        connect(this->_nameEdit, &QLineEdit::editingFinished, this, [this]() {
            if (this->_externalPort == nullptr) {
                return;
            }
            emit this->externalPortRename(this->_externalPort, this->_nameEdit->text());
        });

        layout->addWidget(new QLabel{QStringLiteral("Row"), this}, 1, 0);
        this->_rowSpinBox = new QSpinBox{this};
        this->_rowSpinBox->setMinimumHeight(MIN_HEIGHT);
        this->_rowSpinBox->setMinimum(0);
        this->_rowSpinBox->setMaximum(hardware::Interposer::COB_ARRAY_HEIGHT - 1);
        layout->addWidget(this->_rowSpinBox, 1, 1);

        layout->addWidget(new QLabel{QStringLiteral("Column"), this}, 2, 0);
        this->_colSpinBox = new QSpinBox{this};
        this->_colSpinBox->setMinimumHeight(MIN_HEIGHT);
        this->_colSpinBox->setMinimum(0);
        this->_colSpinBox->setMaximum(hardware::Interposer::COB_ARRAY_WIDTH - 1);
        layout->addWidget(this->_colSpinBox, 2, 1);

        layout->addWidget(new QLabel{QStringLiteral("Dir"), this}, 3, 0);
        this->_dirComboBox = new QComboBox{this};
        this->_dirComboBox->setMinimumHeight(MIN_HEIGHT);
        this->_dirComboBox->addItem(QStringLiteral("Hori"));
        this->_dirComboBox->addItem(QStringLiteral("Vert"));
        layout->addWidget(this->_dirComboBox, 3, 1);

        layout->addWidget(new QLabel{QStringLiteral("Index"), this}, 4, 0);
        this->_indexSpinBox = new QSpinBox{this};
        this->_indexSpinBox->setMinimumHeight(MIN_HEIGHT);
        this->_indexSpinBox->setMinimum(0);
        this->_indexSpinBox->setMaximum(hardware::COB::INDEX_SIZE);
        layout->addWidget(this->_indexSpinBox, 4, 1);

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
