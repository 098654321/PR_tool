#include "./layoutinfowidget.h"
#include "./layoutscene.h"
#include "qlineedit.h"
#include <cassert>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTableView>
#include <QGridLayout>
#include <QStandardItemModel>
#include <QHeaderView>
#include <QGroupBox>
#include <QDebug>
#include <QMessageBox>
#include <QLineEdit>

namespace PR_tool::widget {

    constexpr int MIN_HEIGHT = 30;

    LayoutInfoWidget::LayoutInfoWidget(hardware::Interposer* interposer, circuit::BaseDie* basedie, LayoutScene* scene, QWidget* parent):
        QWidget{parent},
        _interposer{interposer},
        _basedie{basedie},
        _scene{scene}
    {
        auto thisLayout = new QVBoxLayout {this};
        auto widget = new QGroupBox {"Layout Information", this};
        thisLayout->addWidget(widget);
        thisLayout->addStretch();

        auto layout = new QGridLayout{widget};
        layout->setSpacing(10);

        // L3: clarify Schematic vs Layout editing roles
        auto roleHint = new QLabel {
            QStringLiteral("Edit connectivity in Schematic; adjust TopDie placement here."),
            widget
        };
        roleHint->setWordWrap(true);
        roleHint->setStyleSheet(QStringLiteral("color: gray;"));
        layout->addWidget(roleHint, 0, 0, 1, 2);

        // Size (display-only)
        layout->addWidget(new QLabel {"TopDie Instance Size ", widget}, 1, 0);
        this->_topdieInstSizeSpinBox = new QSpinBox {this};
        this->_topdieInstSizeSpinBox->setMinimum(0);
        this->_topdieInstSizeSpinBox->setMaximum(hardware::Interposer::TOB_ARRAY_HEIGHT * hardware::Interposer::TOB_ARRAY_WIDTH);
        this->_topdieInstSizeSpinBox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        this->_topdieInstSizeSpinBox->setEnabled(false);
        layout->addWidget(this->_topdieInstSizeSpinBox, 1, 1);

        // Layout Map (read-only; no jump-to-TOB)
        auto label = new QLabel {"Layout Place Map ", widget};
        label->setMinimumHeight(MIN_HEIGHT);
        layout->addWidget(label, 2, 0, 1, 2);
        this->_instPlaceView = new QTableView {widget};
        this->_instPlaceView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        this->_instPlaceView->setEditTriggers(QAbstractItemView::NoEditTriggers);
        this->_instPlaceView->setFocusPolicy(Qt::NoFocus);
        layout->addWidget(this->_instPlaceView, 3, 0, 1, 2);

        // Path length (display-only)
        layout->addWidget(new QLabel {"Path Length", widget}, 4, 0);
        this->_pathLengthEdit = new QLineEdit {this};
        this->_pathLengthEdit->setMinimumHeight(MIN_HEIGHT);
        this->_pathLengthEdit->setEnabled(false);
        layout->addWidget(this->_pathLengthEdit, 5, 0, 1, 2);

        layout->setColumnMinimumWidth(0, 50);
        layout->setColumnStretch(0, 0);

        this->updateInfo();
    }

    void LayoutInfoWidget::updateInfo() {
        // Instance size
        auto instSize = this->_basedie->topdie_insts().size();
        this->_topdieInstSizeSpinBox->setValue(instSize);

        // Coords
        auto model = new QStandardItemModel {static_cast<int>(instSize), 2};
        model->setHorizontalHeaderLabels(QStringList{"TopDie Instance", "TOB Coord"}); 
        // Add items
        auto itemRoot = model->invisibleRootItem();
        int row = 0;
        for (auto& [name, inst] : this->_basedie->topdie_insts()) {
            auto nameItem = new QStandardItem {QString::fromStdString(name.data())};
            nameItem->setEditable(false);
            model->setItem(row, 0, nameItem);
            assert(inst->tob() != nullptr);
            auto coordItem = new QStandardItem {
                QString::fromStdString(std::format("{}", inst->tob()->coord()))
            };
            coordItem->setEditable(false);
            model->setItem(row, 1, coordItem);
            row += 1;
        }

        auto originModel = this->_instPlaceView->model();
        if (originModel != nullptr) {
            delete originModel;
        }

        this->_instPlaceView->setModel(model);

        // Length
        this->_pathLengthEdit->setText(QString{"%1"}.arg(this->_scene->totalNetLenght()));
    }

}