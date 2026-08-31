#include "./schematicwidget.h"
#include "./schematicscene.h"
#include "./schematicview.h"
#include "./schematicinfowidget.h"
#include "./schematiclibwidget.h"
#include "../chrometokens.h"

#include "./item/topdieinstitem.h"
#include "./item/netitem.h"
#include "./item/exportitem.h"
#include "./item/sourceportitem.h"

#include <QSplitter>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPalette>
#include <QShortcut>
#include <QKeySequence>
#include <QLabel>

namespace PR_tool::widget {

    namespace {
        void fillChrome(QWidget* w, const char* hex) {
            if (w == nullptr) {
                return;
            }
            w->setAttribute(Qt::WA_StyledBackground, true);
            w->setAutoFillBackground(true);
            QPalette pal = w->palette();
            pal.setColor(QPalette::Window, ChromeTokens::color(hex));
            pal.setColor(QPalette::Base, ChromeTokens::color(hex));
            pal.setColor(QPalette::AlternateBase, ChromeTokens::color(hex));
            w->setPalette(pal);
        }
    }

    SchematicWidget::SchematicWidget(hardware::Interposer* interposer, circuit::BaseDie* basedie, QWidget *parent) :
        QWidget{parent},
        _interposer{interposer},
        _basedie{basedie}
    {
        this->_scene = new SchematicScene{this->_basedie, interposer};

        this->setObjectName(QStringLiteral("ChromePage"));
        fillChrome(this, ChromeTokens::bg);
        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->setContentsMargins(8, 8, 8, 8);

        this->_lockBanner = new QLabel{
            QStringLiteral("This view is locked after routing. Use Edit Design to edit."),
            this};
        this->_lockBanner->setObjectName(QStringLiteral("LookbackLockBanner"));
        this->_lockBanner->setWordWrap(true);
        this->_lockBanner->hide();
        layout->addWidget(this->_lockBanner);

        this->_splitter = new QSplitter{Qt::Horizontal, this};
        this->_splitter->setHandleWidth(1);
        layout->addWidget(this->_splitter);

        this->initTopdieLibWidget();
        this->initSchematicView(interposer, basedie);
        this->initInfoWidget();

        this->setWindowTitle("Schematic Editor");
        this->resize(1000, 800);
    }

    void SchematicWidget::reload() {
        this->_scene->reloadItems();
        this->_view->adjustSceneRect();
        this->_view->bindMiniMap();
        this->_view->applyInitialView();
        this->_libWidget->reload();
        this->_infoWidget->reload();
    }

    void SchematicWidget::arrangeFromPlacement() {
        this->_scene->arrangeTopDiesFromPlacement();
        this->_view->adjustSceneRect();
        this->_view->bindMiniMap();
        this->_infoWidget->reload();
    }

    auto SchematicWidget::graphicsView() const -> GraphicsView* {
        return this->_view;
    }

    void SchematicWidget::setNavigatorVisible(bool visible) {
        if (this->_libWidget != nullptr) {
            this->_libWidget->setVisible(visible);
        }
    }

    void SchematicWidget::setInspectorVisible(bool visible) {
        if (this->_infoWidget != nullptr) {
            this->_infoWidget->setVisible(visible);
        }
    }

    auto SchematicWidget::isNavigatorVisible() const -> bool {
        return this->_libWidget != nullptr && this->_libWidget->isVisible();
    }

    auto SchematicWidget::isInspectorVisible() const -> bool {
        return this->_infoWidget != nullptr && this->_infoWidget->isVisible();
    }

    void SchematicWidget::setLookbackLocked(bool locked) {
        if (this->_lockBanner != nullptr) {
            this->_lockBanner->setVisible(locked);
        }
        if (this->_libWidget != nullptr) {
            this->_libWidget->setEnabled(!locked);
        }
        if (this->_infoWidget != nullptr) {
            this->_infoWidget->setEnabled(!locked);
        }
        if (this->_view != nullptr) {
            this->_view->setLookbackLocked(locked);
            for (auto* shortcut : this->_view->findChildren<QShortcut*>()) {
                shortcut->setEnabled(!locked);
            }
        }
    }

    void SchematicWidget::initTopdieLibWidget() {
        this->_libWidget = new SchematicLibWidget {this->_basedie, this->_scene, this->_splitter};
        this->_libWidget->setObjectName(QStringLiteral("SidePanel"));
        fillChrome(this->_libWidget, ChromeTokens::panel);
        this->_libWidget->setMinimumWidth(200);
        this->_libWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

        this->_splitter->addWidget(this->_libWidget);

        QObject::connect(
            this->_libWidget, &SchematicLibWidget::initialTopDieInst, 
            this->_scene, &SchematicScene::handleInitialTopDie);

        QObject::connect(
            this->_libWidget, &SchematicLibWidget::addExport, 
            this->_scene, &SchematicScene::handleAddExport);

        QObject::connect(
            this->_libWidget, &SchematicLibWidget::addVdd,
            this->_scene, &SchematicScene::handleAddVdd);

        QObject::connect(
            this->_libWidget, &SchematicLibWidget::addGnd,
            this->_scene, &SchematicScene::handleAddGnd);
    }

    void SchematicWidget::initSchematicView(hardware::Interposer* interposer, circuit::BaseDie* basedie) {
        this->_view = new SchematicView {interposer, basedie};
        this->_view->setScene(this->_scene);
        this->_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        this->_view->adjustSceneRect();
        this->_view->bindMiniMap();

        this->_splitter->addWidget(this->_view);
    }

    void SchematicWidget::initInfoWidget() {
        this->_infoWidget = new SchematicInfoWidget{this->_basedie, this->_scene, this->_view, this->_splitter};
        this->_infoWidget->setObjectName(QStringLiteral("SidePanel"));
        fillChrome(this->_infoWidget, ChromeTokens::panel);
        this->_infoWidget->setContentsMargins(8, 8, 8, 8);
        this->_infoWidget->setMinimumWidth(250);
        this->_infoWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);

        this->_splitter->addWidget(this->_infoWidget);

        QObject::connect(
            this->_scene, &SchematicScene::viewSelected, 
            this->_infoWidget, &SchematicInfoWidget::showViewInfo);

        QObject::connect(
            this->_scene, &SchematicScene::netSelected, 
            this->_infoWidget, &SchematicInfoWidget::showNetInfoWidget);

        QObject::connect(
            this->_scene, &SchematicScene::exportSelected, 
            this->_infoWidget, &SchematicInfoWidget::showExPortInfoWidget);

        QObject::connect(
            this->_scene, &SchematicScene::topdieInstSelected, 
            this->_infoWidget, &SchematicInfoWidget::showTopDieInstanceInfoWidget);

        QObject::connect(
            this->_scene, &SchematicScene::layoutChanged, 
            this, &SchematicWidget::layoutChanged);
    
        QObject::connect(
            this->_infoWidget, &SchematicInfoWidget::layoutChanged,
            this, &SchematicWidget::layoutChanged);

        // Ch.十五: canvas selection ↔ navigator tree cross-probe
        QObject::connect(
            this->_scene, &SchematicScene::viewSelected,
            this->_libWidget, [this]() {
                this->_libWidget->syncSelectionFromCanvas(nullptr);
            });
        QObject::connect(
            this->_scene, &SchematicScene::netSelected,
            this->_libWidget, [this](schematic::NetItem* net) {
                this->_libWidget->syncSelectionFromCanvas(net);
            });
        QObject::connect(
            this->_scene, &SchematicScene::exportSelected,
            this->_libWidget, [this](schematic::ExternalPortItem* eport) {
                this->_libWidget->syncSelectionFromCanvas(eport);
            });
        QObject::connect(
            this->_scene, &SchematicScene::topdieInstSelected,
            this->_libWidget, [this](schematic::TopDieInstanceItem* inst) {
                this->_libWidget->syncSelectionFromCanvas(inst);
            });
        QObject::connect(
            this->_scene, &SchematicScene::sourcePortSelected,
            this->_libWidget, [this](schematic::SourcePortItem* port) {
                this->_libWidget->syncSelectionFromCanvas(port);
            });
        QObject::connect(
            this, &SchematicWidget::layoutChanged,
            this->_libWidget, &SchematicLibWidget::reload);

        // Delete/Backspace on the canvas removes the item shown in the property panel (S9).
        auto* deleteShortcut = new QShortcut{QKeySequence::Delete, this->_view};
        deleteShortcut->setContext(Qt::WidgetShortcut);
        QObject::connect(
            deleteShortcut, &QShortcut::activated,
            this->_infoWidget, &SchematicInfoWidget::deleteCurrentItem);

        auto* backspaceShortcut = new QShortcut{QKeySequence{Qt::Key_Backspace}, this->_view};
        backspaceShortcut->setContext(Qt::WidgetShortcut);
        QObject::connect(
            backspaceShortcut, &QShortcut::activated,
            this->_infoWidget, &SchematicInfoWidget::deleteCurrentItem);

        // Esc cancels floating topdie / export / net (U8/S5; Right-click also cancels).
        auto* escapeShortcut = new QShortcut{QKeySequence{Qt::Key_Escape}, this->_view};
        escapeShortcut->setContext(Qt::WidgetShortcut);
        QObject::connect(
            escapeShortcut, &QShortcut::activated,
            this->_scene, &SchematicScene::cancelFloatingPlacement);
    }

}