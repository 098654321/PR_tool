#include "./layoutinfowidget.h"
#include "./layoutscene.h"
#include <cassert>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

#include <QLabel>
#include <QVBoxLayout>
#include <QTableView>
#include <QGridLayout>
#include <QStandardItemModel>
#include <QHeaderView>
#include <QGroupBox>
#include <QLocale>
#include <QPushButton>
#include <QSizePolicy>

namespace PR_tool::widget {

    constexpr int MIN_HEIGHT = 30;

    LayoutInfoWidget::LayoutInfoWidget(hardware::Interposer* interposer, circuit::BaseDie* basedie, LayoutScene* scene, QWidget* parent):
        QWidget{parent},
        _interposer{interposer},
        _basedie{basedie},
        _scene{scene}
    {
        auto thisLayout = new QVBoxLayout {this};
        auto widget = new QGroupBox {"Layout", this};
        thisLayout->addWidget(widget);
        thisLayout->addStretch();

        auto* resetButton = new QPushButton {QStringLiteral("Restore Default Layout"), this};
        resetButton->setObjectName(QStringLiteral("SecondaryCta"));
        resetButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        resetButton->setCursor(Qt::PointingHandCursor);
        thisLayout->addWidget(resetButton);
        connect(resetButton, &QPushButton::clicked, this, [this]() {
            if (this->_scene != nullptr) {
                this->_scene->restoreDefaultPlacement();
            }
        });

        auto layout = new QGridLayout{widget};
        layout->setSpacing(10);

        // Estimated HPWL (same bbox Manhattan formula as SA placer)
        auto* metric = new QWidget {widget};
        metric->setObjectName(QStringLiteral("LayoutHpwlMetric"));
        auto* metricLayout = new QVBoxLayout {metric};
        metricLayout->setContentsMargins(12, 10, 12, 12);
        metricLayout->setSpacing(2);
        this->_estimatedLengthLabel = new QLabel {"Estimated Wire Length (HPWL)", metric};
        this->_estimatedLengthLabel->setObjectName(QStringLiteral("LayoutHpwlLabel"));
        this->_estimatedLengthValue = new QLabel {metric};
        this->_estimatedLengthValue->setObjectName(QStringLiteral("LayoutHpwlValue"));
        this->_estimatedLengthValue->setTextInteractionFlags(Qt::TextSelectableByMouse);
        metricLayout->addWidget(this->_estimatedLengthLabel);
        metricLayout->addWidget(this->_estimatedLengthValue);
        layout->addWidget(metric, 0, 0, 1, 2);

        // Layout Map (read-only display; row click jumps to TOB)
        auto label = new QLabel {"Layout Place Map", widget};
        label->setMinimumHeight(MIN_HEIGHT);
        layout->addWidget(label, 1, 0, 1, 2);
        this->_instPlaceView = new QTableView {widget};
        this->_instPlaceView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        this->_instPlaceView->setEditTriggers(QAbstractItemView::NoEditTriggers);
        this->_instPlaceView->setSelectionBehavior(QAbstractItemView::SelectRows);
        this->_instPlaceView->setSelectionMode(QAbstractItemView::SingleSelection);
        this->_instPlaceView->setFocusPolicy(Qt::StrongFocus);
        layout->addWidget(this->_instPlaceView, 2, 0, 1, 2);

        connect(this->_instPlaceView, &QTableView::clicked, this, [this](const QModelIndex& index) {
            if (!index.isValid() || this->_scene == nullptr) {
                return;
            }
            const auto nameIndex = index.sibling(index.row(), 0);
            const auto name = nameIndex.data(Qt::DisplayRole).toString();
            this->_scene->focusTopDieInstance(name);
        });

        layout->setColumnMinimumWidth(0, 50);
        layout->setColumnStretch(0, 0);

        this->updateInfo();
    }

    void LayoutInfoWidget::updateInfo() {
        const auto instSize = this->_basedie->topdie_insts().size();

        auto model = new QStandardItemModel {static_cast<int>(instSize), 2};
        model->setHorizontalHeaderLabels(QStringList{"TopDie Instance", "TOB Coord"});
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

        this->_estimatedLengthValue->setText(
            QLocale::system().toString(this->_scene->estimatedTotalWireLength())
        );
    }

}
