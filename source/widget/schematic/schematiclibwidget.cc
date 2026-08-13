#include "./schematiclibwidget.h"
#include "../chrometokens.h"
#include "./item/topdieinstitem.h"
#include "./item/exportitem.h"
#include "./item/netitem.h"
#include "./item/netpointitem.h"
#include "./item/pinitem.h"
#include "./item/sourceportitem.h"
#include "./schematicscene.h"
#include "./schematictypography.h"

#include <circuit/basedie.hh>
#include <circuit/topdie/topdie.hh>
#include <parse/reader/config/topdie.hh>
#include <serde/de.hh>
#include <serde/json/json.hh>
#include <std/exception.hh>
#include <std/file.hh>
#include <std/string.hh>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QFileDialog>
#include <QMessageBox>
#include <QLabel>
#include <QFrame>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVariant>
#include <QGraphicsItem>
#include <QAbstractItemView>
#include <QDir>
#include <QToolButton>
#include <QMenu>
#include <QSizePolicy>
#include <QGridLayout>
#include <algorithm>

namespace PR_tool::widget {

    namespace {

        auto netDisplayName(schematic::NetItem* net) -> QString {
            if (!net || net->isFloating()) {
                return QStringLiteral("(floating)");
            }
            QString begin = QStringLiteral("?");
            QString end = QStringLiteral("?");
            if (net->beginPoint() && net->beginPoint()->connectedPin()) {
                begin = net->beginPoint()->connectedPin()->toString();
            }
            if (net->endPoint() && net->endPoint()->connectedPin()) {
                end = net->endPoint()->connectedPin()->toString();
            }
            return QStringLiteral("%1 → %2").arg(begin, end);
        }

        void clearLayout(QLayout* layout) {
            if (!layout) {
                return;
            }
            while (auto* item = layout->takeAt(0)) {
                if (auto* w = item->widget()) {
                    w->deleteLater();
                }
                delete item;
            }
        }

    } // namespace

    SchematicLibWidget::SchematicLibWidget(
        circuit::BaseDie* basedie,
        SchematicScene* scene,
        QWidget* parent
    ) :
        QWidget{parent},
        _basedie{basedie},
        _scene{scene}
    {
        this->buildUi();
        this->rebuildPalette();
        this->rebuildTree();
    }

    void SchematicLibWidget::buildUi() {
        auto* thisLayout = new QVBoxLayout{this};
        thisLayout->setContentsMargins(8, 8, 8, 8);
        thisLayout->setSpacing(8);

        auto* designLabel = new QLabel{QStringLiteral("DESIGN"), this};
        schematic::SchematicTypography::applyPanelSectionTitle(designLabel);
        thisLayout->addWidget(designLabel);

        auto* line = new QFrame{this};
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        thisLayout->addWidget(line);

        // Palette strip (placement) — Ch.十五: Navi top, not a global toolbar
        auto* paletteScroll = new QScrollArea{this};
        paletteScroll->setWidgetResizable(true);
        paletteScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        paletteScroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        paletteScroll->setFixedHeight(40);
        paletteScroll->setFrameShape(QFrame::NoFrame);

        this->_paletteStrip = new QWidget{paletteScroll};
        this->_paletteLayout = new QHBoxLayout{this->_paletteStrip};
        this->_paletteLayout->setContentsMargins(0, 0, 0, 0);
        this->_paletteLayout->setSpacing(4);
        paletteScroll->setWidget(this->_paletteStrip);
        thisLayout->addWidget(paletteScroll);

        this->_searchEdit = new QLineEdit{this};
        this->_searchEdit->setPlaceholderText(QStringLiteral("Search..."));
        this->_searchEdit->setClearButtonEnabled(true);
        thisLayout->addWidget(this->_searchEdit);

        auto* filterBox = new QWidget{this};
        auto* filterLayout = new QVBoxLayout{filterBox};
        filterLayout->setContentsMargins(0, 0, 0, 0);
        filterLayout->setSpacing(4);

        auto* showLabel = new QLabel{QStringLiteral("Show:"), filterBox};
        schematic::SchematicTypography::applyPropertyLabel(showLabel);
        filterLayout->addWidget(showLabel);

        const auto chipQss = ChromeTokens::applyToQss(QStringLiteral(
            "QPushButton {"
            "  background-color: @surface;"
            "  color: @text;"
            "  border: 1px solid @borderStrong;"
            "  border-radius: @radiusSmpx;"
            "  padding: 2px 8px;"
            "  min-height: 24px;"
            "}"
            "QPushButton:checked {"
            "  background-color: @selectionFill;"
            "  color: @text;"
            "}"
            "QPushButton:hover:!checked:!disabled {"
            "  background-color: @bg;"
            "}"
            "QPushButton:checked:hover:!disabled {"
            "  background-color: @selectionFill;"
            "}"
            "QPushButton:pressed:!disabled {"
            "  background-color: @panel;"
            "}"
            "QPushButton:focus {"
            "  border: 1px solid @accent;"
            "}"
            "QPushButton:disabled {"
            "  background-color: @bg;"
            "  color: @disabledText;"
            "  border: 1px solid @border;"
            "}"
        ));
        auto makeFilterChip = [filterBox, &chipQss](const QString& text) {
            auto* chip = new QPushButton{text, filterBox};
            chip->setCheckable(true);
            chip->setChecked(true);
            chip->setCursor(Qt::PointingHandCursor);
            chip->setFocusPolicy(Qt::TabFocus);
            chip->setAttribute(Qt::WA_StyledBackground, true);
            chip->setMinimumHeight(24);
            chip->setMaximumHeight(26);
            chip->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
            chip->setStyleSheet(chipQss);
            schematic::SchematicTypography::applyPaletteButton(chip);
            return chip;
        };
        this->_filterSignal = makeFilterChip(QStringLiteral("Signal"));
        this->_filterBus = makeFilterChip(QStringLiteral("Bus"));
        this->_filterPower = makeFilterChip(QStringLiteral("Power"));
        this->_filterGround = makeFilterChip(QStringLiteral("Ground"));
        this->_filterExternal = makeFilterChip(QStringLiteral("External"));

        auto* chipGrid = new QGridLayout{};
        chipGrid->setContentsMargins(0, 0, 0, 0);
        chipGrid->setHorizontalSpacing(4);
        chipGrid->setVerticalSpacing(4);
        chipGrid->addWidget(this->_filterSignal, 0, 0, Qt::AlignLeft);
        chipGrid->addWidget(this->_filterBus, 0, 1, Qt::AlignLeft);
        chipGrid->addWidget(this->_filterPower, 0, 2, Qt::AlignLeft);
        chipGrid->addWidget(this->_filterGround, 1, 0, Qt::AlignLeft);
        chipGrid->addWidget(this->_filterExternal, 1, 1, Qt::AlignLeft);
        chipGrid->setColumnStretch(3, 1);
        filterLayout->addLayout(chipGrid);
        thisLayout->addWidget(filterBox);

        this->_tree = new QTreeWidget{this};
        schematic::SchematicTypography::applyTree(this->_tree);
        this->_tree->setHeaderHidden(true);
        this->_tree->setRootIsDecorated(true);
        this->_tree->setUniformRowHeights(true);
        this->_tree->setSelectionMode(QAbstractItemView::SingleSelection);
        this->_tree->setExpandsOnDoubleClick(false);
        thisLayout->addWidget(this->_tree, 1);

        this->_topDiesRoot = new QTreeWidgetItem{this->_tree, {QStringLiteral("TopDie Instances")}};
        this->_portsRoot = new QTreeWidgetItem{this->_tree, {QStringLiteral("External Ports")}};
        this->_netsRoot = new QTreeWidgetItem{this->_tree, {QStringLiteral("Nets")}};
        this->_powerRoot = new QTreeWidgetItem{this->_tree, {QStringLiteral("Power")}};
        this->_topDiesRoot->setExpanded(true);
        this->_portsRoot->setExpanded(true);
        this->_netsRoot->setExpanded(false); // Ch.十五: Nets default collapsed
        this->_powerRoot->setExpanded(true);

        connect(this->_searchEdit, &QLineEdit::textChanged, this, &SchematicLibWidget::applySearchFilter);
        connect(this->_tree, &QTreeWidget::itemClicked, this, &SchematicLibWidget::onTreeItemClicked);
        connect(this->_filterSignal, &QPushButton::toggled, this, &SchematicLibWidget::pushConnectionFilter);
        connect(this->_filterBus, &QPushButton::toggled, this, &SchematicLibWidget::pushConnectionFilter);
        connect(this->_filterPower, &QPushButton::toggled, this, &SchematicLibWidget::pushConnectionFilter);
        connect(this->_filterGround, &QPushButton::toggled, this, &SchematicLibWidget::pushConnectionFilter);
        connect(this->_filterExternal, &QPushButton::toggled, this, &SchematicLibWidget::pushConnectionFilter);
    }

    auto SchematicLibWidget::makePaletteButton(const QString& text, const QColor& fill) -> QPushButton* {
        auto* button = new QPushButton{QStringLiteral("+ %1").arg(text), this->_paletteStrip};
        button->setMinimumHeight(28);
        button->setMaximumHeight(32);
        button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        button->setFocusPolicy(Qt::TabFocus);
        button->setCursor(Qt::PointingHandCursor);
        button->setAttribute(Qt::WA_StyledBackground, true);
        const auto bar = QStringLiteral("rgb(%1, %2, %3)")
            .arg(fill.red()).arg(fill.green()).arg(fill.blue());
        auto qss = ChromeTokens::applyToQss(QStringLiteral(
            "QPushButton {"
            "  background-color: @surface;"
            "  color: @text;"
            "  border: 1px solid @borderStrong;"
            "  border-left: 3px solid %1;"
            "  border-radius: @radiusSmpx;"
            "  padding: 4px 10px 4px 8px;"
            "  min-height: 28px;"
            "}"
            "QPushButton:hover:!disabled {"
            "  background-color: @bg;"
            "}"
            "QPushButton:pressed:!disabled {"
            "  background-color: @panel;"
            "}"
            "QPushButton:focus {"
            "  border: 1px solid @accent;"
            "  border-left: 3px solid %1;"
            "}"
            "QPushButton:disabled {"
            "  background-color: @bg;"
            "  color: @disabledText;"
            "  border: 1px solid @border;"
            "  border-left: 3px solid %1;"
            "}"
        )).arg(bar);
        button->setStyleSheet(qss);
        schematic::SchematicTypography::applyPaletteButton(button);
        return button;
    }

    void SchematicLibWidget::rebuildPalette() {
        clearLayout(this->_paletteLayout);

        if (this->_basedie) {
            for (auto& [_, topdie] : this->_basedie->topdies()) {
                auto* td = topdie.get();
                if (!td) {
                    continue;
                }
                const auto name = QString::fromStdString(td->name().data());
                const auto fill = schematic::TopDieInstanceItem::colorForTopDieType(td->name());
                auto* button = this->makePaletteButton(name, fill);
                this->_paletteLayout->addWidget(button);
                connect(button, &QPushButton::clicked, this, [this, td]() {
                    emit this->initialTopDieInst(td);
                });
            }
        }

        auto* exportBtn = this->makePaletteButton(
            QStringLiteral("Export"), QColor(180, 180, 200, 160));
        this->_paletteLayout->addWidget(exportBtn);
        connect(exportBtn, &QPushButton::clicked, this, &SchematicLibWidget::addExport);

        auto* vddBtn = this->makePaletteButton(
            QStringLiteral("VDD"), QColor(220, 80, 80, 160));
        this->_paletteLayout->addWidget(vddBtn);
        connect(vddBtn, &QPushButton::clicked, this, &SchematicLibWidget::addVdd);

        auto* gndBtn = this->makePaletteButton(
            QStringLiteral("GND"), QColor(80, 80, 80, 160));
        this->_paletteLayout->addWidget(gndBtn);
        connect(gndBtn, &QPushButton::clicked, this, &SchematicLibWidget::addGnd);

        auto* moreBtn = new QToolButton{this->_paletteStrip};
        moreBtn->setText(QStringLiteral("⋯"));
        moreBtn->setToolTip(QStringLiteral("More actions"));
        moreBtn->setPopupMode(QToolButton::InstantPopup);
        moreBtn->setFixedHeight(32);
        moreBtn->setAutoRaise(true);
        moreBtn->setFocusPolicy(Qt::TabFocus);
        moreBtn->setCursor(Qt::PointingHandCursor);
        moreBtn->setStyleSheet(ChromeTokens::applyToQss(QStringLiteral(
            "QToolButton:focus { border: 1px solid @accent; }"
            "QToolButton:disabled { color: @disabledText; }")));
        auto* moreMenu = new QMenu{moreBtn};
        moreMenu->addAction(
            QStringLiteral("Load TopDie…"),
            this,
            &SchematicLibWidget::onLoadTopDieClicked);
        moreMenu->addAction(
            QStringLiteral("Load TopDies…"),
            this,
            &SchematicLibWidget::onLoadTopDiesClicked);
        moreBtn->setMenu(moreMenu);
        this->_paletteLayout->addWidget(moreBtn);

        this->_paletteLayout->addStretch();
    }

    void SchematicLibWidget::rebuildTree() {
        const bool netsExpanded = this->_netsRoot && this->_netsRoot->isExpanded();

        auto clearChildren = [](QTreeWidgetItem* root) {
            if (!root) {
                return;
            }
            while (root->childCount() > 0) {
                delete root->takeChild(0);
            }
        };
        clearChildren(this->_topDiesRoot);
        clearChildren(this->_portsRoot);
        clearChildren(this->_netsRoot);
        clearChildren(this->_powerRoot);

        if (!this->_scene) {
            return;
        }

        // TopDie instances
        QList<schematic::TopDieInstanceItem*> tops = this->_scene->topdieinstMap().values();
        std::sort(tops.begin(), tops.end(), [](auto* a, auto* b) {
            return a->name() < b->name();
        });
        for (auto* top : tops) {
            if (!top) {
                continue;
            }
            auto* item = new QTreeWidgetItem{
                this->_topDiesRoot,
                {QStringLiteral("%1   %2").arg(top->name(), top->typeName())}
            };
            item->setData(0, kNavRoleType, static_cast<int>(NavKind::TopDieInst));
            item->setData(0, kNavRolePtr, QVariant::fromValue(static_cast<void*>(top)));
        }
        this->_topDiesRoot->setText(
            0, QStringLiteral("TopDie Instances (%1)").arg(tops.size()));

        // External ports
        QList<schematic::ExternalPortItem*> ports = this->_scene->exportMap().values();
        std::sort(ports.begin(), ports.end(), [](auto* a, auto* b) {
            return a->name() < b->name();
        });
        for (auto* eport : ports) {
            if (!eport) {
                continue;
            }
            auto* item = new QTreeWidgetItem{this->_portsRoot, {eport->name()}};
            item->setData(0, kNavRoleType, static_cast<int>(NavKind::ExternalPort));
            item->setData(0, kNavRolePtr, QVariant::fromValue(static_cast<void*>(eport)));
        }
        this->_portsRoot->setText(
            0, QStringLiteral("External Ports (%1)").arg(ports.size()));

        // Nets (signal only in main list)
        QList<schematic::NetItem*> nets;
        for (auto* net : this->_scene->nets()) {
            if (!net || net->isFloating() || !net->unwrap()) {
                continue;
            }
            const auto& in = net->unwrap()->input_pin();
            const auto& out = net->unwrap()->output_pin();
            if (in.is_fixed() || out.is_fixed()) {
                continue;
            }
            nets.push_back(net);
        }
        std::sort(nets.begin(), nets.end(), [](auto* a, auto* b) {
            return netDisplayName(a) < netDisplayName(b);
        });
        for (auto* net : nets) {
            auto* item = new QTreeWidgetItem{this->_netsRoot, {netDisplayName(net)}};
            item->setData(0, kNavRoleType, static_cast<int>(NavKind::Net));
            item->setData(0, kNavRolePtr, QVariant::fromValue(static_cast<void*>(net)));
        }
        this->_netsRoot->setText(0, QStringLiteral("Nets (%1)").arg(nets.size()));
        this->_netsRoot->setExpanded(netsExpanded); // preserve; default false on first build

        // Power: VDD / GND source ports
        int powerCount = 0;
        for (auto* port : this->_scene->vddPorts()) {
            if (!port) {
                continue;
            }
            auto* item = new QTreeWidgetItem{this->_powerRoot, {port->name()}};
            item->setData(0, kNavRoleType, static_cast<int>(NavKind::SourcePort));
            item->setData(0, kNavRolePtr, QVariant::fromValue(static_cast<void*>(port)));
            ++powerCount;
        }
        for (auto* port : this->_scene->gndPorts()) {
            if (!port) {
                continue;
            }
            auto* item = new QTreeWidgetItem{this->_powerRoot, {port->name()}};
            item->setData(0, kNavRoleType, static_cast<int>(NavKind::SourcePort));
            item->setData(0, kNavRolePtr, QVariant::fromValue(static_cast<void*>(port)));
            ++powerCount;
        }
        this->_powerRoot->setText(0, QStringLiteral("Power (%1)").arg(powerCount));

        this->applySearchFilter();
    }

    void SchematicLibWidget::reload() {
        this->rebuildPalette();
        this->rebuildTree();
    }

    void SchematicLibWidget::filterTreeItem(
        QTreeWidgetItem* item,
        const QString& filter,
        bool forceVisible
    ) {
        if (!item) {
            return;
        }
        const bool selfMatch = filter.isEmpty()
            || forceVisible
            || item->text(0).contains(filter, Qt::CaseInsensitive);

        bool anyChildVisible = false;
        for (int i = 0; i < item->childCount(); ++i) {
            auto* child = item->child(i);
            const bool childMatch = filter.isEmpty()
                || child->text(0).contains(filter, Qt::CaseInsensitive);
            // Recurse one level (roots only have leaves in our tree).
            child->setHidden(!childMatch && !filter.isEmpty());
            if (!child->isHidden()) {
                anyChildVisible = true;
            }
        }

        // Category roots stay visible if any child matches, or name matches.
        if (item->parent() == nullptr) {
            item->setHidden(!filter.isEmpty() && !anyChildVisible && !selfMatch);
            if (!filter.isEmpty() && anyChildVisible) {
                item->setExpanded(true);
            }
        } else {
            item->setHidden(!selfMatch && !filter.isEmpty());
        }
    }

    void SchematicLibWidget::applySearchFilter() {
        const auto filter = this->_searchEdit ? this->_searchEdit->text().trimmed() : QString{};
        for (int i = 0; i < this->_tree->topLevelItemCount(); ++i) {
            this->filterTreeItem(this->_tree->topLevelItem(i), filter, false);
        }
    }

    void SchematicLibWidget::pushConnectionFilter() {
        if (!this->_scene) {
            return;
        }
        ConnectionFilter filter;
        filter.signal = this->_filterSignal && this->_filterSignal->isChecked();
        filter.bus = this->_filterBus && this->_filterBus->isChecked();
        filter.power = this->_filterPower && this->_filterPower->isChecked();
        filter.ground = this->_filterGround && this->_filterGround->isChecked();
        filter.external = this->_filterExternal && this->_filterExternal->isChecked();
        this->_scene->setConnectionFilter(filter);
    }

    void SchematicLibWidget::onTreeItemClicked(QTreeWidgetItem* item, int) {
        if (!item || !this->_scene || this->_syncingSelection) {
            return;
        }
        if (item->parent() == nullptr) {
            return; // category headers
        }

        const auto kind = static_cast<NavKind>(item->data(0, kNavRoleType).toInt());
        auto* ptr = item->data(0, kNavRolePtr).value<void*>();
        if (!ptr) {
            return;
        }

        QGraphicsItem* canvasItem = nullptr;
        switch (kind) {
            case NavKind::TopDieInst:
                canvasItem = static_cast<schematic::TopDieInstanceItem*>(ptr);
                break;
            case NavKind::ExternalPort:
                canvasItem = static_cast<schematic::ExternalPortItem*>(ptr);
                break;
            case NavKind::Net:
                canvasItem = static_cast<schematic::NetItem*>(ptr);
                break;
            case NavKind::SourcePort:
                canvasItem = static_cast<schematic::SourcePortItem*>(ptr);
                break;
            default:
                break;
        }

        if (canvasItem) {
            this->_scene->selectFromNavigator(canvasItem);
        }
    }

    void SchematicLibWidget::syncSelectionFromCanvas(QGraphicsItem* item) {
        if (!this->_tree) {
            return;
        }
        if (!item) {
            QSignalBlocker block{this->_tree};
            this->_tree->clearSelection();
            return;
        }

        this->_syncingSelection = true;
        QSignalBlocker block{this->_tree};

        auto matchPtr = [&](QTreeWidgetItem* root, void* ptr) -> QTreeWidgetItem* {
            if (!root || !ptr) {
                return nullptr;
            }
            for (int i = 0; i < root->childCount(); ++i) {
                auto* child = root->child(i);
                if (child->data(0, kNavRolePtr).value<void*>() == ptr) {
                    return child;
                }
            }
            return nullptr;
        };

        QTreeWidgetItem* found = nullptr;
        if (item->type() == schematic::TopDieInstanceItem::Type) {
            found = matchPtr(this->_topDiesRoot, item);
        } else if (item->type() == schematic::ExternalPortItem::Type) {
            found = matchPtr(this->_portsRoot, item);
        } else if (item->type() == schematic::NetItem::Type) {
            found = matchPtr(this->_netsRoot, item);
        } else if (item->type() == schematic::SourcePortItem::Type) {
            found = matchPtr(this->_powerRoot, item);
        }

        this->_tree->clearSelection();
        if (found) {
            found->setHidden(false);
            if (auto* parent = found->parent()) {
                parent->setHidden(false);
                parent->setExpanded(true);
            }
            found->setSelected(true);
            this->_tree->scrollToItem(found);
        }

        this->_syncingSelection = false;
    }

    void SchematicLibWidget::onLoadTopDieClicked() try {
        auto filePath = QFileDialog::getOpenFileName(
            this,
            tr("Select TopDie Config"),
            QDir::currentPath(),
            tr("JSON File (*.json);;All File (*)")
        );
        if (!filePath.isEmpty()) {
            this->loadTopDie(filePath);
        }
    }
    catch (const std::Exception& err) {
        QMessageBox::critical(
            this,
            QStringLiteral("Load TopDie Error"),
            QString::fromStdString(err.what())
        );
    }

    void SchematicLibWidget::onLoadTopDiesClicked() try {
        auto filePath = QFileDialog::getOpenFileName(
            this,
            tr("Select TopDies Config"),
            QDir::currentPath(),
            tr("JSON File (*.json);;All File (*)")
        );
        if (!filePath.isEmpty()) {
            this->loadTopDies(filePath);
        }
    }
    catch (const std::Exception& err) {
        QMessageBox::critical(
            this,
            QStringLiteral("Load TopDies Error"),
            QString::fromStdString(err.what())
        );
    }

    void SchematicLibWidget::loadTopDie(const QString& path) {
        auto filepath = std::FilePath{path.toStdString()};
        auto topdieConfig = serde::deserialize_from<serde::Json, parse::TopDieConfig>(filepath);
        this->addTopDie(filepath.stem().string(), std::move(topdieConfig.pin_map));
    }

    void SchematicLibWidget::loadTopDies(const QString& path) {
        auto filepath = std::FilePath{path.toStdString()};
        auto topdieConfigs =
            serde::deserialize_from<serde::Json, std::HashMap<std::String, parse::TopDieConfig>>(filepath);
        for (auto& [name, config] : topdieConfigs) {
            this->addTopDie(name, std::move(config.pin_map));
        }
    }

    void SchematicLibWidget::addTopDie(std::String name, std::HashMap<std::String, std::usize> pinmap) {
        auto topdie = this->_basedie->add_topdie(std::move(name), std::move(pinmap));
        this->addTopDie(topdie);
    }

    void SchematicLibWidget::addTopDie(circuit::TopDie* topdie) {
        Q_UNUSED(topdie);
        this->rebuildPalette();
    }

}
