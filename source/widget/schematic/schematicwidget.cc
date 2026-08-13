#include "./schematicwidget.h"
#include "./schematicscene.h"
#include "./schematicview.h"
#include "./schematicinfowidget.h"
#include "./schematiclibwidget.h"

#include "./item/topdieinstitem.h"
#include "./item/netitem.h"
#include "./item/exportitem.h"
#include "./item/sourceportitem.h"

#include <QSplitter>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QShortcut>
#include <QKeySequence>

namespace PR_tool::widget {

    SchematicWidget::SchematicWidget(hardware::Interposer* interposer, circuit::BaseDie* basedie, QWidget *parent) :
        QWidget{parent},
        _interposer{interposer},
        _basedie{basedie}
    {
        this->_scene = new SchematicScene{this->_basedie, interposer};

        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->setContentsMargins(10, 10, 10, 10);

        this->_splitter = new QSplitter{Qt::Horizontal, this};
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
        this->_libWidget->reload();
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

    void SchematicWidget::initTopdieLibWidget() {
        this->_libWidget = new SchematicLibWidget {this->_basedie, this->_scene, this->_splitter};
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

        this->_splitter->addWidget(this->_view);
    }

    void SchematicWidget::initInfoWidget() {
        this->_infoWidget = new SchematicInfoWidget{this->_basedie, this->_scene, this->_view, this->_splitter};
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