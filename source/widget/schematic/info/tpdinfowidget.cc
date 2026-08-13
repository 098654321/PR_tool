#include "./tpdinfowidget.h"
#include "../item/topdieinstitem.h"
#include "../item/pinitem.h"
#include "../item/netitem.h"
#include "../item/netpointitem.h"
#include "../schematicscene.h"

#include <circuit/topdieinst/topdieinst.hh>
#include <circuit/connection/connection.hh>
#include <hardware/tob/tob.hh>

#include <cassert>
#include <algorithm>
#include <debug/debug.hh>

#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QTableView>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QStandardItemModel>
#include <QSortFilterProxyModel>
#include <QToolButton>
#include <QScrollArea>
#include <QMessageBox>
#include <QMenu>
#include <QAction>
#include <QClipboard>
#include <QApplication>
#include <QFrame>
#include <QSignalBlocker>
#include <QAbstractItemView>
#include <QSizePolicy>
#include <QSettings>

#include "../schematictypography.h"

namespace PR_tool::widget::schematic {

    namespace {

        constexpr int kMinHeight = 28;
        constexpr auto kSettingsOrg = "PR_tool";
        constexpr auto kSettingsApp = "PR_tool";
        constexpr auto kGeneralFoldKey = "inspector/topdie/generalExpanded";
        constexpr auto kConnectivityFoldKey = "inspector/topdie/connectivityExpanded";
        constexpr auto kPinMapFoldKey = "inspector/topdie/pinMapExpanded";

        auto isPowerPinName(const QString& name) -> bool {
            const auto upper = name.toUpper();
            return upper.contains(QStringLiteral("VDD"))
                || upper.contains(QStringLiteral("VSS"))
                || upper.contains(QStringLiteral("GND"))
                || upper.contains(QStringLiteral("VCC"));
        }

        auto pinHasPowerNet(PinItem* pin) -> bool {
            if (!pin) {
                return false;
            }
            for (auto* point : pin->connectedPoints()) {
                if (!point || !point->netItem() || !point->netItem()->unwrap()) {
                    continue;
                }
                const auto* conn = point->netItem()->unwrap();
                if (conn->input_pin().is_fixed() || conn->output_pin().is_fixed()) {
                    return true;
                }
            }
            return false;
        }

    } // namespace

    TopDieInstanceInfoWidget::TopDieInstanceInfoWidget(SchematicScene* scene, QWidget* parent) :
        QWidget{parent},
        _scene{scene}
    {
        this->loadFoldState();
        this->buildUi();
    }

    void TopDieInstanceInfoWidget::buildUi() {
        auto* outer = new QVBoxLayout{this};
        outer->setContentsMargins(8, 8, 8, 8);
        outer->setSpacing(8);

        this->_titleLabel = new QLabel{QStringLiteral("TOPDIE INSTANCE"), this};
        SchematicTypography::applyInspectorTitle(this->_titleLabel);
        outer->addWidget(this->_titleLabel);

        auto* line = new QFrame{this};
        line->setObjectName(QStringLiteral("SideHairline"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Plain);
        outer->addWidget(line);

        auto* scroll = new QScrollArea{this};
        scroll->setWidgetResizable(true);
        scroll->setFrameShape(QFrame::NoFrame);
        outer->addWidget(scroll, 1);

        auto* content = new QWidget{scroll};
        scroll->setWidget(content);
        auto* contentLayout = new QVBoxLayout{content};
        contentLayout->setContentsMargins(0, 0, 0, 0);
        contentLayout->setSpacing(8);

        // —— General ——
        auto* generalBody = new QWidget{content};
        auto* generalGrid = new QGridLayout{generalBody};
        generalGrid->setContentsMargins(0, 0, 0, 0);
        generalGrid->setHorizontalSpacing(8);
        generalGrid->setVerticalSpacing(8);
        generalGrid->setColumnStretch(1, 1);

        this->_nameEdit = new QLineEdit{generalBody};
        this->_nameEdit->setMinimumHeight(kMinHeight);
        SchematicTypography::applyPropertyValue(this->_nameEdit);
        this->_typeLabel = new QLabel{generalBody};
        SchematicTypography::applyPropertyValue(this->_typeLabel);
        this->_positionLabel = new QLabel{generalBody};
        SchematicTypography::applyPropertyValue(this->_positionLabel);
        this->_orientationLabel = new QLabel{QStringLiteral("R0"), generalBody};
        SchematicTypography::applyPropertyValue(this->_orientationLabel);
        this->_visibleToggle = new QCheckBox{generalBody};
        SchematicTypography::applyPropertyValue(this->_visibleToggle);
        this->_visibleToggle->setCursor(Qt::PointingHandCursor);
        this->_visibleToggle->setFocusPolicy(Qt::TabFocus);
        this->_statusLabel = new QLabel{QStringLiteral("● Valid"), generalBody};
        SchematicTypography::applyPropertyValue(this->_statusLabel);
        SchematicTypography::applyForeground(this->_statusLabel, ChromeTokens::success);

        auto addPropRow = [&](QGridLayout* grid, QWidget* parent, int row, const QString& text, QWidget* value) {
            auto* label = new QLabel{text, parent};
            SchematicTypography::applyPropertyLabel(label);
            grid->addWidget(label, row, 0);
            grid->addWidget(value, row, 1);
        };

        int row = 0;
        addPropRow(generalGrid, generalBody, row++, QStringLiteral("Name"), this->_nameEdit);
        addPropRow(generalGrid, generalBody, row++, QStringLiteral("Type"), this->_typeLabel);
        addPropRow(generalGrid, generalBody, row++, QStringLiteral("Position"), this->_positionLabel);
        addPropRow(generalGrid, generalBody, row++, QStringLiteral("Orientation"), this->_orientationLabel);
        addPropRow(generalGrid, generalBody, row++, QStringLiteral("Visible"), this->_visibleToggle);
        addPropRow(generalGrid, generalBody, row++, QStringLiteral("Status"), this->_statusLabel);

        contentLayout->addWidget(
            this->makeCollapsibleGroup(QStringLiteral("General"), generalBody, &this->_generalExpanded));

        // —— Connectivity ——
        auto* connBody = new QWidget{content};
        auto* connGrid = new QGridLayout{connBody};
        connGrid->setContentsMargins(0, 0, 0, 0);
        connGrid->setHorizontalSpacing(8);
        connGrid->setVerticalSpacing(8);
        connGrid->setColumnStretch(1, 1);

        this->_connectionsLabel = new QLabel{connBody};
        SchematicTypography::applyPropertyValue(this->_connectionsLabel);
        this->_signalPinsLabel = new QLabel{connBody};
        SchematicTypography::applyPropertyValue(this->_signalPinsLabel);
        this->_powerPinsLabel = new QLabel{connBody};
        SchematicTypography::applyPropertyValue(this->_powerPinsLabel);

        row = 0;
        addPropRow(connGrid, connBody, row++, QStringLiteral("Connections"), this->_connectionsLabel);
        addPropRow(connGrid, connBody, row++, QStringLiteral("Signal Pins"), this->_signalPinsLabel);
        addPropRow(connGrid, connBody, row++, QStringLiteral("Power Pins"), this->_powerPinsLabel);

        contentLayout->addWidget(
            this->makeCollapsibleGroup(QStringLiteral("Connectivity"), connBody, &this->_connectivityExpanded));

        // —— Pin Map ——
        auto* pinBody = new QWidget{content};
        auto* pinLayout = new QVBoxLayout{pinBody};
        pinLayout->setContentsMargins(0, 0, 0, 0);
        pinLayout->setSpacing(8);

        this->_pinSearchEdit = new QLineEdit{pinBody};
        this->_pinSearchEdit->setPlaceholderText(QStringLiteral("Search pins..."));
        this->_pinSearchEdit->setClearButtonEnabled(true);
        SchematicTypography::applyPropertyValue(this->_pinSearchEdit);
        pinLayout->addWidget(this->_pinSearchEdit);

        this->_pinMapModel = new QStandardItemModel{0, 2, this};
        this->_pinMapModel->setHorizontalHeaderLabels({QStringLiteral("Pin"), QStringLiteral("Bump")});

        this->_pinProxy = new QSortFilterProxyModel{this};
        this->_pinProxy->setSourceModel(this->_pinMapModel);
        this->_pinProxy->setFilterKeyColumn(0);
        this->_pinProxy->setFilterCaseSensitivity(Qt::CaseInsensitive);
        this->_pinProxy->setSortCaseSensitivity(Qt::CaseInsensitive);

        this->_pinMapView = new QTableView{pinBody};
        this->_pinMapView->setModel(this->_pinProxy);
        this->_pinMapView->setSelectionBehavior(QAbstractItemView::SelectRows);
        this->_pinMapView->setSelectionMode(QAbstractItemView::SingleSelection);
        this->_pinMapView->setEditTriggers(QAbstractItemView::NoEditTriggers);
        this->_pinMapView->setSortingEnabled(true);
        this->_pinMapView->sortByColumn(0, Qt::AscendingOrder);
        this->_pinMapView->setContextMenuPolicy(Qt::CustomContextMenu);
        this->_pinMapView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        this->_pinMapView->verticalHeader()->setVisible(false);
        this->_pinMapView->verticalHeader()->setDefaultSectionSize(22);
        this->_pinMapView->verticalHeader()->setMinimumSectionSize(20);
        this->_pinMapView->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
        this->_pinMapView->setAlternatingRowColors(true);
        SchematicTypography::applyPropertyValue(this->_pinMapView);
        pinLayout->addWidget(this->_pinMapView, 1);

        contentLayout->addWidget(
            this->makeCollapsibleGroup(QStringLiteral("Pin Map"), pinBody, &this->_pinMapExpanded), 1);

        auto* removeButton = new QPushButton{QStringLiteral("Remove"), content};
        removeButton->setMinimumHeight(kMinHeight);
        contentLayout->addWidget(removeButton);

        // Signals
        connect(this->_nameEdit, &QLineEdit::editingFinished, this, [this]() {
            if (this->_topdieInstance == nullptr) {
                return;
            }
            emit this->topdieInstanceRename(this->_topdieInstance, this->_nameEdit->text());
        });

        connect(this->_visibleToggle, &QCheckBox::toggled, this, [this](bool on) {
            if (this->_topdieInstance) {
                this->_topdieInstance->setVisible(on);
            }
        });

        connect(this->_pinSearchEdit, &QLineEdit::textChanged, this, &TopDieInstanceInfoWidget::applyPinFilter);
        connect(this->_pinMapView, &QTableView::clicked, this, &TopDieInstanceInfoWidget::onPinClicked);
        connect(this->_pinMapView, &QTableView::doubleClicked, this, &TopDieInstanceInfoWidget::onPinDoubleClicked);
        connect(this->_pinMapView, &QWidget::customContextMenuRequested,
                this, &TopDieInstanceInfoWidget::onPinContextMenu);

        connect(removeButton, &QPushButton::clicked, this, [this]() {
            auto response = QMessageBox::question(
                this,
                QStringLiteral("Confirm"),
                QStringLiteral("Do you want to delete this topdie instance?"),
                QMessageBox::Yes | QMessageBox::No);
            if (response == QMessageBox::Yes && this->_topdieInstance != nullptr) {
                emit this->removeTopDieInstance(this->_topdieInstance);
            }
        });
    }

    auto TopDieInstanceInfoWidget::makeCollapsibleGroup(
        const QString& title,
        QWidget* body,
        bool* expandedState
    ) -> QWidget* {
        auto* wrap = new QWidget{this};
        auto* layout = new QVBoxLayout{wrap};
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(8);

        auto* header = new QToolButton{wrap};
        header->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        header->setArrowType((*expandedState) ? Qt::DownArrow : Qt::RightArrow);
        header->setText(title);
        header->setCheckable(true);
        header->setChecked(*expandedState);
        header->setAutoRaise(true);
        header->setCursor(Qt::PointingHandCursor);
        header->setFocusPolicy(Qt::TabFocus);
        header->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        header->setStyleSheet(ChromeTokens::applyToQss(QStringLiteral(
            "QToolButton { background: transparent; border: none; color: @textMuted; padding: 4px 0px; text-align: left; }"
            "QToolButton:hover { background: @bg; border-radius: @radiusSmpx; }"
            "QToolButton:focus { border: 1px solid @accent; border-radius: @radiusSmpx; }")));
        SchematicTypography::applyPanelSectionTitle(header);
        layout->addWidget(header);

        body->setVisible(*expandedState);
        layout->addWidget(body);

        connect(header, &QToolButton::toggled, this, [this, header, body, expandedState](bool on) {
            *expandedState = on;
            body->setVisible(on);
            header->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
            this->saveFoldState();
        });

        return wrap;
    }

    void TopDieInstanceInfoWidget::loadFoldState() {
        QSettings settings{kSettingsOrg, kSettingsApp};
        this->_generalExpanded = settings.value(kGeneralFoldKey, true).toBool();
        this->_connectivityExpanded = settings.value(kConnectivityFoldKey, true).toBool();
        this->_pinMapExpanded = settings.value(kPinMapFoldKey, true).toBool();
    }

    void TopDieInstanceInfoWidget::saveFoldState() {
        QSettings settings{kSettingsOrg, kSettingsApp};
        settings.setValue(kGeneralFoldKey, this->_generalExpanded);
        settings.setValue(kConnectivityFoldKey, this->_connectivityExpanded);
        settings.setValue(kPinMapFoldKey, this->_pinMapExpanded);
    }

    void TopDieInstanceInfoWidget::loadTopDieInstance(TopDieInstanceItem* inst) {
        this->_topdieInstance = inst;
        if (this->_topdieInstance == nullptr) {
            debug::exception("Load a empty net into TopDieInstanceInfoWidget");
        }

        this->_titleLabel->setText(QStringLiteral("TOPDIE INSTANCE"));
        this->_nameEdit->setText(this->_topdieInstance->name());
        this->_typeLabel->setText(this->_topdieInstance->typeName());

        QString posText = QStringLiteral("—");
        if (auto* circ = this->_topdieInstance->unwrap()) {
            if (auto* tob = circ->tob()) {
                const auto& c = tob->coord();
                posText = QStringLiteral("(%1, %2)").arg(c.row).arg(c.col);
            }
        }
        this->_positionLabel->setText(posText);
        this->_orientationLabel->setText(QStringLiteral("R0"));

        {
            QSignalBlocker block{this->_visibleToggle};
            this->_visibleToggle->setChecked(this->_topdieInstance->isVisible());
        }
        this->_statusLabel->setText(QStringLiteral("● Valid"));
        SchematicTypography::applyForeground(this->_statusLabel, ChromeTokens::success);

        this->updateConnectivityStats();
        this->rebuildPinMapModel();
        this->applyPinFilter();
    }

    void TopDieInstanceInfoWidget::updateConnectivityStats() {
        int connections = 0;
        int signalPins = 0;
        int powerPins = 0;

        if (this->_topdieInstance) {
            for (auto* pin : this->_topdieInstance->pins()) {
                if (!pin) {
                    continue;
                }
                const bool power = isPowerPinName(pin->name()) || pinHasPowerNet(pin);
                if (power) {
                    ++powerPins;
                } else {
                    ++signalPins;
                }
                for (auto* point : pin->connectedPoints()) {
                    if (point && point->netItem() && !point->netItem()->isFloating()) {
                        ++connections;
                        break;
                    }
                }
            }
        }

        this->_connectionsLabel->setText(QString::number(connections));
        this->_signalPinsLabel->setText(QString::number(signalPins));
        this->_powerPinsLabel->setText(QString::number(powerPins));
    }

    void TopDieInstanceInfoWidget::rebuildPinMapModel() {
        this->_pinMapModel->removeRows(0, this->_pinMapModel->rowCount());
        if (!this->_topdieInstance || !this->_topdieInstance->unwrap()) {
            return;
        }

        const auto& pinmap = this->_topdieInstance->unwrap()->topdie()->pins_map();
        QList<QPair<QString, std::usize>> rows;
        rows.reserve(static_cast<int>(pinmap.size()));
        for (const auto& [pinName, bumpIndex] : pinmap) {
            rows.append({QString::fromStdString(pinName), bumpIndex});
        }
        std::sort(rows.begin(), rows.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        for (const auto& [pinName, bumpIndex] : rows) {
            auto* nameItem = new QStandardItem{pinName};
            nameItem->setEditable(false);
            nameItem->setData(pinName, Qt::UserRole);
            auto* bumpItem = new QStandardItem{QString::number(static_cast<qulonglong>(bumpIndex))};
            bumpItem->setEditable(false);
            bumpItem->setData(static_cast<qulonglong>(bumpIndex), Qt::UserRole);
            this->_pinMapModel->appendRow({nameItem, bumpItem});
        }

        this->_pinMapView->sortByColumn(0, Qt::AscendingOrder);
    }

    void TopDieInstanceInfoWidget::applyPinFilter() {
        this->_pinProxy->setFilterFixedString(this->_pinSearchEdit->text().trimmed());
    }

    auto TopDieInstanceInfoWidget::pinItemForRow(int proxyRow) -> PinItem* {
        if (!this->_topdieInstance || proxyRow < 0) {
            return nullptr;
        }
        const auto index = this->_pinProxy->index(proxyRow, 0);
        const auto pinName = this->_pinProxy->data(index, Qt::UserRole).toString();
        return this->_topdieInstance->pins().value(pinName, nullptr);
    }

    void TopDieInstanceInfoWidget::onPinClicked(const QModelIndex& index) {
        if (!index.isValid() || !this->_scene) {
            return;
        }
        if (auto* pin = this->pinItemForRow(index.row())) {
            this->_scene->focusPin(pin, /*locate=*/false);
        }
    }

    void TopDieInstanceInfoWidget::onPinDoubleClicked(const QModelIndex& index) {
        if (!index.isValid()) {
            return;
        }
        if (auto* pin = this->pinItemForRow(index.row())) {
            this->locatePin(pin);
        }
    }

    void TopDieInstanceInfoWidget::onPinContextMenu(const QPoint& pos) {
        const auto index = this->_pinMapView->indexAt(pos);
        if (!index.isValid()) {
            return;
        }
        auto* pin = this->pinItemForRow(index.row());
        const auto pinName = this->_pinProxy->data(this->_pinProxy->index(index.row(), 0), Qt::UserRole).toString();
        const auto bumpText = this->_pinProxy->data(this->_pinProxy->index(index.row(), 1)).toString();

        QMenu menu{this};
        auto* locateAct = menu.addAction(QStringLiteral("Locate Pin"));
        auto* showConnAct = menu.addAction(QStringLiteral("Show Connections"));
        menu.addSeparator();
        auto* copyNameAct = menu.addAction(QStringLiteral("Copy Pin Name"));
        auto* copyBumpAct = menu.addAction(QStringLiteral("Copy Bump Index"));

        auto* chosen = menu.exec(this->_pinMapView->viewport()->mapToGlobal(pos));
        if (chosen == locateAct) {
            this->locatePin(pin);
        } else if (chosen == showConnAct) {
            this->showPinConnections(pin);
        } else if (chosen == copyNameAct) {
            this->copyText(pinName);
        } else if (chosen == copyBumpAct) {
            this->copyText(bumpText);
        }
    }

    void TopDieInstanceInfoWidget::locatePin(PinItem* pin) {
        if (pin && this->_scene) {
            this->_scene->focusPin(pin, /*locate=*/true);
        }
    }

    void TopDieInstanceInfoWidget::showPinConnections(PinItem* pin) {
        if (pin && this->_scene) {
            this->_scene->focusPin(pin, /*locate=*/false);
        }
    }

    void TopDieInstanceInfoWidget::copyText(const QString& text) {
        if (auto* clipboard = QApplication::clipboard()) {
            clipboard->setText(text);
        }
    }

    auto TopDieInstanceInfoWidget::currentTopDieInstance() -> TopDieInstanceItem* {
        return this->_topdieInstance;
    }

}
