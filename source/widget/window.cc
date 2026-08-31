#include "./window.h"
#include "./chrometokens.h"
#include "./prthread.h"
#include "./frame/routeprogresstrack.h"
#include "./frame/placeprogresschart.h"
#include "./view2d/view2dwidget.h"
#include "./view3d/view3dwidget.h"
#include "./schematic/schematicwidget.h"
#include "./schematic/schematicview.h"
#include "./schematic/schematictypography.h"
#include "./layout/layoutwidget.h"

#include "algo/netbuilder/netbuilder.hh"
#include "algo/router/common/maze/mazererouter.hh"
#include "parse/writer/module.hh"
#include "qaction.h"
#include "qdir.h"
#include "qfiledialog.h"
#include "qmessagebox.h"
#include "widget/setting/settingwidget.h"
#include <algo/router/route_nets.hh>
#include <algo/placer/sa/saplacestrategy.hh>

#include <cassert>
#include <parse/reader/module.hh>
#include <widget/frame/msgexception.h>
#include <widget/frame/controlbitexportdialog.h>
#include <widget/frame/graphicsview.h>

#include <hardware/interposer.hh>
#include <hardware/track/trackcoord.hh>
#include <hardware/tob/tob.hh>
#include <circuit/basedie.hh>
#include <circuit/connection/pin.hh>
#include <circuit/topdieinst/topdieinst.hh>

#include <serde/json/json.hh>
#include <std/exception.hh>
#include <std/file.hh>
#include <std/utility.hh>

#include <QApplication>
#include <QActionGroup>
#include <QByteArray>
#include <QCloseEvent>
#include <QDebug>
#include <QResizeEvent>
#include <QSettings>
#include <QSplitter>
#include <QVBoxLayout>
#include <QToolBar>
#include <QStackedWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QFrame>
#include <QSignalBlocker>
#include <QKeySequence>
#include <QShortcut>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QDialog>
#include <QLabel>
#include <QThread>
#include <QStatusBar>
#include <QSizePolicy>

#include <fstream>
#include <filesystem>

namespace PR_tool::widget {

    Window::Window(QWidget *parent)
        : QMainWindow{parent}
    {
        this->createSystem();
        this->setUnifiedTitleAndToolBarOnMac(false);

        this->createMenuBar();
        this->createToolBar();
        this->createCentralWidget();
        this->createStatusBar();

        this->restoreWindowSettings();
    }

    namespace {
        constexpr auto kSettingsOrg = "PR_tool";
        constexpr auto kSettingsApp = "PR_tool";
        constexpr auto kGeometryKey = "geometry";
        constexpr auto kWindowStateKey = "windowState";
        constexpr auto kSchematicSplitterKey = "schematicSplitter";
        constexpr auto kShowNavigatorKey = "showNavigator";
        constexpr auto kShowInspectorKey = "showInspector";

        constexpr auto kSwitcherStyle = R"(
QFrame#ViewSwitcher {
    background-color: @surface;
    border: 1px solid @borderStrong;
    border-radius: @radiuspx;
}
QFrame#ViewSwitcher QPushButton {
    background-color: @surface;
    color: @text;
    border: 1px solid @surface;
    border-right: 1px solid @borderStrong;
    border-top-left-radius: 0px;
    border-top-right-radius: 0px;
    border-bottom-right-radius: 0px;
    border-bottom-left-radius: 0px;
    padding: 6px 13px;
    min-height: 26px;
}
QFrame#ViewSwitcher QPushButton[viewSwitch="lead"] {
    border-top-left-radius: @radiuspx;
    border-bottom-left-radius: @radiuspx;
}
QFrame#ViewSwitcher QPushButton[viewSwitch="trail"] {
    border-right: 1px solid @surface;
    border-top-right-radius: @radiuspx;
    border-bottom-right-radius: @radiuspx;
}
QFrame#ViewSwitcher QPushButton:checked {
    background-color: @accent;
    color: @onAccent;
    border-color: @accent;
}
QFrame#ViewSwitcher QPushButton:hover:!checked:!disabled {
    background-color: @bg;
    border-color: @bg;
    border-right-color: @borderStrong;
}
QFrame#ViewSwitcher QPushButton[viewSwitch="trail"]:hover:!checked:!disabled {
    border-right-color: @bg;
}
QFrame#ViewSwitcher QPushButton:pressed:!checked:!disabled {
    background-color: @panel;
    border-color: @panel;
    border-right-color: @borderStrong;
}
QFrame#ViewSwitcher QPushButton[viewSwitch="trail"]:pressed:!checked:!disabled {
    border-right-color: @panel;
}
QFrame#ViewSwitcher QPushButton:checked:hover:!disabled {
    background-color: @accentHover;
    border-color: @accentHover;
}
QFrame#ViewSwitcher QPushButton:checked:pressed {
    background-color: @accentPressed;
    border-color: @accentPressed;
}
QFrame#ViewSwitcher QPushButton:disabled {
    color: @disabledText;
    background-color: @bg;
    border-color: @bg;
    border-right-color: @borderStrong;
}
QFrame#ViewSwitcher QPushButton[viewSwitch="trail"]:disabled {
    border-right-color: @bg;
}
QFrame#ViewSwitcher QPushButton:focus {
    border: 1px solid @accent;
}
QFrame#ViewSwitcher QPushButton:checked:focus {
    border: 1px solid @onAccent;
}
)";

        constexpr auto kPrimaryCtaStyle = R"(
QPushButton {
    background-color: @accent;
    color: @onAccent;
    border: 2px solid @accent;
    border-radius: @radiusSmpx;
    padding: 5px 14px;
    min-height: 28px;
    font-weight: 600;
}
QPushButton:hover:!disabled {
    background-color: @accentHover;
    border-color: @accentHover;
}
QPushButton:pressed {
    background-color: @accentPressed;
    border-color: @accentPressed;
}
QPushButton:disabled {
    background-color: @disabledBg;
    border-color: @disabledBg;
    color: @onAccent;
}
QPushButton:focus {
    border: 2px solid @onAccent;
}
)";

        constexpr auto kSecondaryCtaStyle = R"(
QPushButton {
    background-color: @surface;
    color: @text;
    border: 1px solid @borderStrong;
    border-radius: @radiusSmpx;
    padding: 7px 16px;
    min-height: 28px;
}
QPushButton:hover:!disabled {
    background-color: @bg;
}
QPushButton:pressed:!disabled {
    background-color: @panel;
}
QPushButton:disabled {
    background-color: @bg;
    color: @disabledText;
    border: 1px solid @border;
}
QPushButton:focus {
    border: 1px solid @accent;
}
)";

        void bindButtonToAction(QPushButton* button, QAction* action) {
            button->setCheckable(action->isCheckable());
            button->setFocusPolicy(Qt::TabFocus);
            auto sync = [button, action]() {
                const QSignalBlocker blocker{button};
                button->setText(action->text());
                button->setToolTip(action->toolTip());
                button->setEnabled(action->isEnabled());
                button->setChecked(action->isChecked());
                button->setAccessibleName(action->text());
                button->setAccessibleDescription(action->statusTip());
            };
            sync();
            QObject::connect(action, &QAction::changed, button, sync);
            QObject::connect(button, &QPushButton::clicked, action, [action]() {
                action->trigger();
            });
        }
    }

    void Window::restoreWindowSettings() {
        QSettings settings{kSettingsOrg, kSettingsApp};

        const auto geometry = settings.value(kGeometryKey).toByteArray();
        if (!geometry.isEmpty()) {
            this->restoreGeometry(geometry);
        } else {
            this->resize(1500, 900);
        }

        const auto windowState = settings.value(kWindowStateKey).toByteArray();
        if (!windowState.isEmpty()) {
            this->restoreState(windowState);
        }

        if (this->_schematicWidget != nullptr) {
            if (auto* splitter = this->_schematicWidget->splitter()) {
                const auto splitterState =
                    settings.value(kSchematicSplitterKey).toByteArray();
                if (!splitterState.isEmpty()) {
                    splitter->restoreState(splitterState);
                }
            }
        }

        const bool showNavigator = settings.value(kShowNavigatorKey, true).toBool();
        const bool showInspector = settings.value(kShowInspectorKey, true).toBool();
        if (this->_showNavigatorAction != nullptr) {
            this->_showNavigatorAction->setChecked(showNavigator);
        }
        if (this->_showInspectorAction != nullptr) {
            this->_showInspectorAction->setChecked(showInspector);
        }
        if (this->_schematicWidget != nullptr) {
            this->_schematicWidget->setNavigatorVisible(showNavigator);
            this->_schematicWidget->setInspectorVisible(showInspector);
        }
        if (this->_layoutWidget != nullptr) {
            this->_layoutWidget->setInspectorVisible(showInspector);
        }
    }

    void Window::saveWindowSettings() const {
        QSettings settings{kSettingsOrg, kSettingsApp};
        settings.setValue(kGeometryKey, this->saveGeometry());
        settings.setValue(kWindowStateKey, this->saveState());

        if (this->_schematicWidget != nullptr) {
            if (auto* splitter = this->_schematicWidget->splitter()) {
                settings.setValue(kSchematicSplitterKey, splitter->saveState());
            }
        }

        if (this->_showNavigatorAction != nullptr) {
            settings.setValue(kShowNavigatorKey, this->_showNavigatorAction->isChecked());
        }
        if (this->_showInspectorAction != nullptr) {
            settings.setValue(kShowInspectorKey, this->_showInspectorAction->isChecked());
        }
    }

    void Window::closeEvent(QCloseEvent* event) {
        this->saveWindowSettings();
        QMainWindow::closeEvent(event);
    }

    void Window::createSystem() {
        this->_interposer = std::make_unique<hardware::Interposer>();
        this->_basedie = std::make_unique<circuit::BaseDie>();
    }

    void Window::createMenuBar() {
        this->_menuBar = this->menuBar();

        // ====================== File ======================
        auto fileMenu= new QMenu("File", this->_menuBar);

        // Load
        this->_loadAction = new QAction("Load", fileMenu);
        fileMenu->addAction(this->_loadAction);
        
        fileMenu->addSeparator(); 

        // Save
        auto saveAction= new QAction("Save", fileMenu);
        fileMenu->addAction(saveAction);

        // Save as
        auto saveAsAction= new QAction("Save as", fileMenu);
        fileMenu->addAction(saveAsAction);

        fileMenu->addSeparator();

        this->_placeAction = new QAction(QStringLiteral("Place"), fileMenu);
        this->_placeAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_P));
        this->_placeAction->setStatusTip(QStringLiteral("Run automatic placement"));
        fileMenu->addAction(this->_placeAction);

        // Route (same command as Design CTA; keep shortcut)
        this->_placeRouteAction = new QAction(QStringLiteral("Route"), fileMenu);
        this->_placeRouteAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
        this->_placeRouteAction->setStatusTip(QStringLiteral("Run routing with the current Layout placement"));
        fileMenu->addAction(this->_placeRouteAction);

        // Export Controlbits (Results only)
        this->_generateControlBitAction = new QAction(QStringLiteral("Export Controlbits"), fileMenu);
        this->_generateControlBitAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_E));
        this->_generateControlBitAction->setStatusTip(QStringLiteral("Export controlbits to output directory"));
        this->_generateControlBitAction->setEnabled(false);
        fileMenu->addAction(this->_generateControlBitAction);

        fileMenu->addSeparator(); 

        // Exit
        auto exitAction = new QAction("Exit", fileMenu);
        fileMenu->addAction(exitAction);

        this->_menuBar->addMenu(fileMenu);

        connect(this->_loadAction, &QAction::triggered, this, &Window::loadConfig);
        connect(saveAction, &QAction::triggered, this, &Window::saveConfig);
        connect(saveAsAction, &QAction::triggered, this, &Window::saveConfigAs);
        connect(this->_placeAction, &QAction::triggered, this, &Window::executePlace);
        connect(this->_placeRouteAction, &QAction::triggered, this, &Window::executePlaceRoute);
        connect(this->_generateControlBitAction, &QAction::triggered, this, &Window::generateControlBitAs);
        connect(exitAction, &QAction::triggered, this, &Window::close);

        // ====================== View ======================
        this->_viewMenu = new QMenu("View", this->_menuBar);

        this->_pageActionGroup = new QActionGroup{this};
        this->_pageActionGroup->setExclusive(true);

        this->_schematicAction = new QAction(QStringLiteral("Schematic"), this->_viewMenu);
        this->_schematicAction->setCheckable(true);
        this->_schematicAction->setStatusTip(QStringLiteral("Switch to Schematic view"));
        this->_pageActionGroup->addAction(this->_schematicAction);
        this->_viewMenu->addAction(this->_schematicAction);

        this->_layoutAction = new QAction(QStringLiteral("Layout"), this->_viewMenu);
        this->_layoutAction->setCheckable(true);
        this->_layoutAction->setStatusTip(QStringLiteral("Switch to Layout view"));
        this->_pageActionGroup->addAction(this->_layoutAction);
        this->_viewMenu->addAction(this->_layoutAction);

        this->_view2DAction = new QAction(QStringLiteral("2D"), this->_viewMenu);
        this->_view2DAction->setCheckable(true);
        this->_view2DAction->setStatusTip(QStringLiteral("Switch to 2D view"));
        this->_pageActionGroup->addAction(this->_view2DAction);
        this->_viewMenu->addAction(this->_view2DAction);

        this->_view3DAction = new QAction(QStringLiteral("3D"), this->_viewMenu);
        this->_view3DAction->setCheckable(true);
        this->_view3DAction->setStatusTip(QStringLiteral("Switch to 3D view"));
        this->_pageActionGroup->addAction(this->_view3DAction);
        this->_viewMenu->addAction(this->_view3DAction);

        this->_schematicAction->setChecked(true);

        this->_viewMenu->addSeparator();

        this->_showNavigatorAction = new QAction(QStringLiteral("Show Navigator"), this->_viewMenu);
        this->_showNavigatorAction->setCheckable(true);
        this->_showNavigatorAction->setChecked(true);
        this->_showNavigatorAction->setStatusTip(QStringLiteral("Show or hide the left navigator panel"));
        this->_viewMenu->addAction(this->_showNavigatorAction);

        this->_showInspectorAction = new QAction(QStringLiteral("Show Inspector"), this->_viewMenu);
        this->_showInspectorAction->setCheckable(true);
        this->_showInspectorAction->setChecked(true);
        this->_showInspectorAction->setStatusTip(QStringLiteral("Show or hide the right inspector panel"));
        this->_viewMenu->addAction(this->_showInspectorAction);

        this->_viewMenu->addSeparator();

        auto fitInViewAction = new QAction{QStringLiteral("Fit in View"), this->_viewMenu};
        fitInViewAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_0));
        fitInViewAction->setStatusTip(QStringLiteral("Fit current canvas content into the viewport"));
        this->_viewMenu->addAction(fitInViewAction);

        auto resetZoomAction = new QAction{QStringLiteral("Reset Zoom"), this->_viewMenu};
        resetZoomAction->setStatusTip(QStringLiteral("Restore default zoom for the current canvas"));
        this->_viewMenu->addAction(resetZoomAction);

        this->_viewMenu->addSeparator();

        auto themesAction = new QAction{QStringLiteral("Themes"), this->_viewMenu};
        themesAction->setEnabled(false);
        themesAction->setToolTip(QStringLiteral("Not available yet"));
        this->_viewMenu->addAction(themesAction);

        this->_settingsAction = new QAction(QStringLiteral("Settings"), this->_viewMenu);
        this->_settingsAction->setStatusTip(QStringLiteral("Open Settings"));
        this->_viewMenu->addAction(this->_settingsAction);

        this->_menuBar->addMenu(this->_viewMenu);

        connect(fitInViewAction, &QAction::triggered, this, [this]() {
            if (auto* view = this->currentGraphicsView()) {
                view->fitContent();
            }
        });
        connect(resetZoomAction, &QAction::triggered, this, [this]() {
            if (auto* view = this->currentGraphicsView()) {
                view->resetZoom();
            }
        });

        // ====================== Help ======================
        auto helpMenu= new QMenu("Help", this->_menuBar);
        auto aboutAction= new QAction("About", helpMenu);
        helpMenu->addAction(aboutAction);

        auto aboutQTAction= new QAction("About Qt", helpMenu);
        helpMenu->addAction(aboutQTAction);

        this->_menuBar->addMenu(helpMenu);

        connect(aboutAction, &QAction::triggered, this, [this]() {
            QMessageBox::about(
                this,
                "About PR_tool",
                "PR_tool is a chiplet interposer place-and-route tool."
            );
        });
        connect(aboutQTAction, &QAction::triggered, qApp, &QApplication::aboutQt);
    }

    void Window::createToolBar() {
        this->_toolBar = new QToolBar(QStringLiteral("Main"), this);
        this->_toolBar->setObjectName(QStringLiteral("MainChromeToolBar"));
        this->_toolBar->setMovable(false);
        this->_toolBar->setFloatable(false);
        this->_toolBar->setToolButtonStyle(Qt::ToolButtonTextOnly);
        this->_toolBar->setAttribute(Qt::WA_StyledBackground, true);
        this->_toolBar->setStyleSheet(ChromeTokens::applyToQss(QStringLiteral(
            "QToolBar {"
            "  background-color: @surface;"
            "  border: none;"
            "  border-bottom: 1px solid @border;"
            "  spacing: 10px;"
            "  padding: 8px 12px;"
            "}")));
        this->addToolBar(Qt::TopToolBarArea, this->_toolBar);

        auto* switcher = new QFrame{this->_toolBar};
        switcher->setObjectName(QStringLiteral("ViewSwitcher"));
        switcher->setFrameShape(QFrame::NoFrame);
        switcher->setAttribute(Qt::WA_StyledBackground, true);
        switcher->setStyleSheet(ChromeTokens::applyToQss(QString::fromUtf8(kSwitcherStyle)));
        auto* switcherRow = new QHBoxLayout{switcher};
        switcherRow->setContentsMargins(0, 0, 0, 0);
        switcherRow->setSpacing(0);

        auto addViewButton = [switcher, switcherRow](QAction* action, const char* slot) {
            auto* button = new QPushButton{switcher};
            button->setProperty("viewSwitch", QLatin1String(slot));
            button->setFlat(false);
            button->setAttribute(Qt::WA_StyledBackground, true);
            bindButtonToAction(button, action);
            auto syncCursor = [button, action]() {
                button->setCursor(action->isEnabled() ? Qt::PointingHandCursor
                                                      : Qt::ForbiddenCursor);
            };
            syncCursor();
            QObject::connect(action, &QAction::changed, button, syncCursor);
            switcherRow->addWidget(button);
            return button;
        };
        addViewButton(this->_schematicAction, "lead");
        addViewButton(this->_layoutAction, "mid");
        addViewButton(this->_view2DAction, "mid");
        addViewButton(this->_view3DAction, "trail");
        this->_toolBar->addWidget(switcher);

        QWidget* stretch = new QWidget(this->_toolBar);
        stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        this->_toolBar->addWidget(stretch);

        // Place slot (becomes Edit Design after place/route) then Route.
        this->_placeCtaAction = new QAction(QStringLiteral("Place"), this);
        this->_placeCtaAction->setToolTip(QStringLiteral("Run automatic placement"));
        this->_placeCtaAction->setStatusTip(QStringLiteral("Run placement"));
        connect(this->_placeCtaAction, &QAction::triggered, this, &Window::onPlaceCta);

        auto* placeButton = new QPushButton{this->_toolBar};
        placeButton->setObjectName(QStringLiteral("PrimaryCta"));
        placeButton->setStyleSheet(ChromeTokens::applyToQss(QString::fromUtf8(kPrimaryCtaStyle)));
        bindButtonToAction(placeButton, this->_placeCtaAction);
        auto syncPlaceCursor = [placeButton, action = this->_placeCtaAction]() {
            placeButton->setCursor(action->isEnabled() ? Qt::PointingHandCursor
                                                       : Qt::ForbiddenCursor);
        };
        syncPlaceCursor();
        connect(this->_placeCtaAction, &QAction::changed, placeButton, syncPlaceCursor);
        this->_toolBar->addWidget(placeButton);

        this->_primaryCtaAction = new QAction(QStringLiteral("Route"), this);
        this->_primaryCtaAction->setToolTip(QStringLiteral("Run routing with the current Layout placement"));
        this->_primaryCtaAction->setStatusTip(QStringLiteral("Run routing"));
        connect(this->_primaryCtaAction, &QAction::triggered, this, &Window::executePlaceRoute);

        auto* primaryButton = new QPushButton{this->_toolBar};
        primaryButton->setObjectName(QStringLiteral("PrimaryCta"));
        primaryButton->setStyleSheet(ChromeTokens::applyToQss(QString::fromUtf8(kPrimaryCtaStyle)));
        bindButtonToAction(primaryButton, this->_primaryCtaAction);
        auto syncPrimaryCursor = [primaryButton, action = this->_primaryCtaAction]() {
            primaryButton->setCursor(action->isEnabled() ? Qt::PointingHandCursor
                                                         : Qt::ForbiddenCursor);
        };
        syncPrimaryCursor();
        connect(this->_primaryCtaAction, &QAction::changed, primaryButton, syncPrimaryCursor);
        this->_toolBar->addWidget(primaryButton);

        auto* exportButton = new QPushButton{this->_toolBar};
        exportButton->setObjectName(QStringLiteral("SecondaryCta"));
        exportButton->setAttribute(Qt::WA_StyledBackground, true);
        exportButton->setStyleSheet(ChromeTokens::applyToQss(QString::fromUtf8(kSecondaryCtaStyle)));
        bindButtonToAction(exportButton, this->_generateControlBitAction);
        auto syncExportCursor = [exportButton, action = this->_generateControlBitAction]() {
            exportButton->setCursor(action->isEnabled() ? Qt::PointingHandCursor
                                                        : Qt::ForbiddenCursor);
        };
        syncExportCursor();
        connect(this->_generateControlBitAction, &QAction::changed, exportButton, syncExportCursor);
        this->_toolBar->addWidget(exportButton);

        connect(this->_schematicAction, &QAction::triggered, this, [this]() {
            this->switchToView(this->_schematicWidget, this->_schematicAction, QString{});
        });
        connect(this->_layoutAction, &QAction::triggered, this, [this]() {
            this->switchToView(
                this->_layoutWidget,
                this->_layoutAction,
                QStringLiteral("Ctrl+Wheel zoom · Middle-drag pan"));
        });
        connect(this->_view2DAction, &QAction::triggered, this, [this]() {
            this->switchToView(
                this->_view2DWidget,
                this->_view2DAction,
                QStringLiteral("Ctrl+Wheel zoom · Middle-drag pan"));
        });
        connect(this->_view3DAction, &QAction::triggered, this, [this]() {
            this->switchToView(this->_view3DWidget, this->_view3DAction, QString{});
        });

        connect(this->_settingsAction, &QAction::triggered, this, [this]() {
            this->_stackedWidget->setCurrentWidget(this->_settingWidget);
            this->updateStatusLabel();
            this->statusBar()->clearMessage();
        });

        connect(this->_showNavigatorAction, &QAction::toggled, this, [this](bool visible) {
            if (this->_schematicWidget != nullptr) {
                this->_schematicWidget->setNavigatorVisible(visible);
            }
        });
        connect(this->_showInspectorAction, &QAction::toggled, this, [this](bool visible) {
            if (this->_schematicWidget != nullptr) {
                this->_schematicWidget->setInspectorVisible(visible);
            }
            if (this->_layoutWidget != nullptr) {
                this->_layoutWidget->setInspectorVisible(visible);
            }
        });

        // Ctrl+1..4 always active: locked views toast instead of switching.
        auto* shortcut1 = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_1), this);
        connect(shortcut1, &QShortcut::activated, this, [this]() {
            this->tryShortcutView(
                this->_schematicWidget,
                this->_schematicAction,
                QString{},
                false);
        });
        auto* shortcut2 = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_2), this);
        connect(shortcut2, &QShortcut::activated, this, [this]() {
            this->tryShortcutView(
                this->_layoutWidget,
                this->_layoutAction,
                QStringLiteral("Ctrl+Wheel zoom · Middle-drag pan"),
                false);
        });
        auto* shortcut3 = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_3), this);
        connect(shortcut3, &QShortcut::activated, this, [this]() {
            this->tryShortcutView(
                this->_view2DWidget,
                this->_view2DAction,
                QStringLiteral("Ctrl+Wheel zoom · Middle-drag pan"),
                true);
        });
        auto* shortcut4 = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_4), this);
        connect(shortcut4, &QShortcut::activated, this, [this]() {
            this->tryShortcutView(this->_view3DWidget, this->_view3DAction, QString{}, true);
        });

        this->updateStageUi();
    }

    void Window::createCentralWidget() {
        this->_stackedWidget = new QStackedWidget(this);

        this->_schematicWidget = new SchematicWidget{this->_interposer.get(), this->_basedie.get(), this};
        this->_stackedWidget->addWidget(this->_schematicWidget);

        this->_layoutWidget = new LayoutWidget{this->_interposer.get(), this->_basedie.get(), this};
        this->_stackedWidget->addWidget(this->_layoutWidget);

        this->_view2DWidget = new View2DWidget {this->_interposer.get(), this->_basedie.get(), this};
        this->_stackedWidget->addWidget(this->_view2DWidget);

        this->_view3DWidget = new View3DWidget {this->_interposer.get(), this->_basedie.get(), this};
        this->_stackedWidget->addWidget(this->_view3DWidget);

        this->_settingWidget = new SettingWidget {this};
        this->_stackedWidget->addWidget(this->_settingWidget); 

        this->setCentralWidget(this->_stackedWidget);

        connect(this->_schematicWidget, &SchematicWidget::layoutChanged, this->_layoutWidget, &LayoutWidget::reload);
        // S14: Layout TOB place/swap → refresh Schematic from basedie (positions are independent of TOB).
        connect(this->_layoutWidget, &LayoutWidget::layoutChanged, this->_schematicWidget, &SchematicWidget::reload);

        if (auto* view = this->_schematicWidget->schematicView()) {
            connect(view, &SchematicView::statusContextChanged, this, &Window::updateStatusLabel);
        }
    }

    void Window::createStatusBar() {
        auto statusBar = this->statusBar();

        this->_stageLabel = new QLabel{this};
        this->_stageLabel->setObjectName(QStringLiteral("StatusStage"));
        this->_stageLabel->setMinimumWidth(120);
        schematic::SchematicTypography::applyStatusEmphasis(this->_stageLabel);
        statusBar->addWidget(this->_stageLabel);

        this->_detailLabel = new QLabel{this};
        this->_detailLabel->setObjectName(QStringLiteral("StatusDetail"));
        this->_detailLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        schematic::SchematicTypography::applyStatus(this->_detailLabel);
        statusBar->addWidget(this->_detailLabel, 1);

        this->_statusLabel = new QLabel{this};
        this->_statusLabel->setObjectName(QStringLiteral("StatusPage"));
        this->_statusLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        this->_statusLabel->setMinimumWidth(160);
        schematic::SchematicTypography::applyStatus(this->_statusLabel);
        statusBar->addPermanentWidget(this->_statusLabel);

        this->_routeLabel = new QLabel{this};
        this->_routeLabel->setObjectName(QStringLiteral("StatusRoute"));
        this->_routeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        this->_routeLabel->setMinimumWidth(88);
        schematic::SchematicTypography::applyStatusEmphasis(this->_routeLabel);
        statusBar->addPermanentWidget(this->_routeLabel);

        this->updateStatusLabel();
    }

    void Window::loadConfig() {
        if (this->_finishPR || this->_placed) {
            QMessageBox::information(
                this,
                QStringLiteral("Load Config"),
                QStringLiteral(
                    "A placed or routed result is active.\n"
                    "Use Edit Design before loading a new config.")
            );
            return;
        }

        if (this->hasConfigPath()) {
            auto reply = QMessageBox::question(this, 
                        "Load Config", 
                        "A configuration already exists.\nDo you want to delete the original configuration and import it?", 
                        QMessageBox::Yes | QMessageBox::No);
            if (reply == QMessageBox::No) {
                return;
            }
        }

        auto filePath = QFileDialog::getExistingDirectory(this, "Select Config path");
        if (!filePath.isEmpty()) {
            this->loadConfigFromPath(filePath);
        }
    }

    void Window::loadConfigFromPath(const QString& path) try {
        if (path.isEmpty()) {
            return;
        }

        auto configPath = std::FilePath{path.toStdString()};

        // Clear before in-place read_config: if parse throws, prior design data is already wiped.
        if (this->hasConfigPath()) {
            this->_interposer->clear();
            this->_basedie->clear();
        }

        this->_register_map = parse::read_config(configPath, this->_interposer.get(), this->_basedie.get(), 0, false);

        this->_schematicWidget->reload();
        this->_layoutWidget->reload();
        this->_view2DWidget->reload();
        this->_view3DWidget->reload();

        this->_configPath.emplace(std::move(configPath));
        this->_finishPR = false;
        this->_placed = false;
        this->_placementSnapshot.clear();
        this->applyDesignEditability();
        this->updateStageUi();
        this->updateStatusLabel();
    } catch (const std::Exception& err) {
        QMessageBox::critical(
            this,
            QStringLiteral("Load Config"),
            QStringLiteral(
                "Failed to load configuration.\n\n"
                "%1\n\n"
                "If a previous design was loaded, it may already have been cleared "
                "before this failure. Reload a valid config or restart the application.")
                .arg(QString::fromLatin1(err.what()))
        );
    }

    namespace {

        auto pin_to_config_string(const circuit::Pin& pin) -> std::String {
            return std::match(pin.connected_point(),
                [](const circuit::ConnectVDD& vdd) -> std::String {
                    return vdd.name;
                },
                [](const circuit::ConnectGND& gnd) -> std::String {
                    return gnd.name;
                },
                [](const circuit::ConnectExPort& eport) -> std::String {
                    return eport.port->name();
                },
                [](const circuit::ConnectBump& bump) -> std::String {
                    return std::format("{}.{}", bump.inst->name(), bump.name);
                }
            );
        }

        auto dir_to_json_string(hardware::TrackDirection dir) -> std::String {
            switch (dir) {
                case hardware::TrackDirection::Horizontal:
                    return "hori";
                case hardware::TrackDirection::Vertical:
                    return "vert";
            }
            return "vert";
        }

        auto write_json_file(const std::FilePath& path, const serde::Json& json) -> void {
            std::ofstream out{path};
            if (!out.is_open()) {
                throw std::runtime_error(std::format("Cannot open '{}' for writing", path.string()));
            }
            out << json.to_string();
            if (!out.good()) {
                throw std::runtime_error(std::format("Failed writing '{}'", path.string()));
            }
        }

        auto default_config_json() -> serde::Json {
            auto root = serde::Json::object();
            root.insert("interposer", serde::Json::string("interposer.json"));
            root.insert("topdies", serde::Json::string("topdies.json"));
            root.insert("topdie_insts", serde::Json::string("topdie_insts.json"));
            root.insert("external_ports", serde::Json::string("external_ports.json"));
            root.insert("connections", serde::Json::string("connections.json"));
            root.insert("reigster_adder", serde::Json::string("register_adder.json"));
            root.insert("ports_01", serde::Json::string("01_ports.json"));
            return root;
        }

        auto load_or_default_config_paths(const std::Option<std::FilePath>& sourceFolder)
            -> serde::Json
        {
            if (sourceFolder.has_value()) {
                const auto configPath = *sourceFolder / "config.json";
                if (std::filesystem::exists(configPath)) {
                    return serde::Json::load_from(configPath);
                }
            }
            return default_config_json();
        }

        auto json_path_field(const serde::Json& config, const std::String& key, const char* fallback)
            -> std::String
        {
            if (auto field = config.get(key); field.has_value() && (*field)->is_string()) {
                return std::String{(*field)->as_string()};
            }
            return fallback;
        }

        auto build_topdie_insts_json(const circuit::BaseDie& basedie) -> serde::Json {
            auto root = serde::Json::object();
            for (const auto& [name, inst] : basedie.topdie_insts()) {
                auto entry = serde::Json::object();
                entry.insert("topdie", serde::Json::string(inst->topdie()->name()));

                const auto& tob_coord = inst->tob()->coord();
                auto coord = serde::Json::object();
                coord.insert("row", serde::Json::integer(static_cast<int>(tob_coord.row)));
                coord.insert("col", serde::Json::integer(static_cast<int>(tob_coord.col)));
                entry.insert("coord", std::move(coord));

                root.insert(std::String{name}, std::move(entry));
            }
            return root;
        }

        auto build_external_ports_json(const circuit::BaseDie& basedie) -> serde::Json {
            auto root = serde::Json::object();
            for (const auto& [name, eport] : basedie.external_ports()) {
                const auto& tc = eport->coord();
                auto coord = serde::Json::object();
                coord.insert("row", serde::Json::integer(static_cast<int>(tc.row)));
                coord.insert("col", serde::Json::integer(static_cast<int>(tc.col)));
                coord.insert("index", serde::Json::integer(static_cast<int>(tc.index)));
                coord.insert("dir", serde::Json::string(dir_to_json_string(tc.dir)));

                auto entry = serde::Json::object();
                entry.insert("coord", std::move(coord));
                root.insert(std::String{name}, std::move(entry));
            }
            return root;
        }

        auto build_sync_group_json(
            const std::HashMap<int, std::Vector<std::Box<circuit::Connection>>>& sync_map
        ) -> serde::Json {
            auto root = serde::Json::object();
            for (const auto& [sync, connections] : sync_map) {
                auto sync_arr = serde::Json::array();
                for (const auto& conn : connections) {
                    auto pair = serde::Json::array();
                    pair.push(serde::Json::string(pin_to_config_string(conn->input_pin())));
                    pair.push(serde::Json::string(pin_to_config_string(conn->output_pin())));
                    sync_arr.push(std::move(pair));
                }
                root.insert(std::to_string(sync), std::move(sync_arr));
            }
            return root;
        }

        auto build_connections_json(const circuit::BaseDie& basedie) -> serde::Json {
            const auto& all = basedie.connections();
            // GUI loads with mode==0 (flat sync → pairs). Prefer that shape when
            // only mode 0 is present; otherwise emit mode → sync → pairs.
            if (all.size() <= 1) {
                if (all.empty()) {
                    return serde::Json::object();
                }
                return build_sync_group_json(all.begin()->second);
            }

            auto root = serde::Json::object();
            for (const auto& [mode, sync_map] : all) {
                root.insert(std::to_string(mode), build_sync_group_json(sync_map));
            }
            return root;
        }

        auto copy_if_exists(const std::FilePath& from, const std::FilePath& to) -> void {
            if (!std::filesystem::exists(from)) {
                return;
            }
            std::filesystem::create_directories(to.parent_path());
            std::filesystem::copy_file(
                from,
                to,
                std::filesystem::copy_options::overwrite_existing
            );
        }

        auto copy_static_config_files(
            const std::FilePath& srcFolder,
            const std::FilePath& destFolder,
            const serde::Json& configPaths
        ) -> void {
            copy_if_exists(srcFolder / "config.json", destFolder / "config.json");

            const auto copy_named = [&](const std::String& key, const char* fallback) {
                const auto rel = json_path_field(configPaths, key, fallback);
                copy_if_exists(srcFolder / rel, destFolder / rel);
            };

            copy_named("interposer", "interposer.json");
            copy_named("topdies", "topdies.json");
            copy_named("ports_01", "01_ports.json");
            copy_named("reigster_adder", "register_adder.json");
        }

    } // namespace

    auto Window::writeConfigFolder(
        const std::FilePath& destFolder,
        const std::Option<std::FilePath>& copyStaticFrom
    ) -> bool {
        if (this->_basedie == nullptr) {
            QMessageBox::critical(
                this,
                QStringLiteral("Save Config"),
                QStringLiteral("No project loaded (basedie is null).")
            );
            return false;
        }

        std::filesystem::create_directories(destFolder);

        const auto configPaths = load_or_default_config_paths(copyStaticFrom);

        const bool same_folder =
            copyStaticFrom.has_value()
            && (std::filesystem::absolute(*copyStaticFrom).lexically_normal()
                == std::filesystem::absolute(destFolder).lexically_normal());

        if (copyStaticFrom.has_value() && !same_folder) {
            copy_static_config_files(*copyStaticFrom, destFolder, configPaths);
        }

        // Ensure destination has a config.json (Save As with no prior path, or
        // copy skipped because source had none).
        const auto destConfigPath = destFolder / "config.json";
        if (!std::filesystem::exists(destConfigPath)) {
            write_json_file(destConfigPath, configPaths);
        }

        const auto topdieInstsRel =
            json_path_field(configPaths, "topdie_insts", "topdie_insts.json");
        const auto externalPortsRel =
            json_path_field(configPaths, "external_ports", "external_ports.json");
        const auto connectionsRel =
            json_path_field(configPaths, "connections", "connections.json");

        write_json_file(
            destFolder / topdieInstsRel,
            build_topdie_insts_json(*this->_basedie)
        );
        write_json_file(
            destFolder / externalPortsRel,
            build_external_ports_json(*this->_basedie)
        );
        write_json_file(
            destFolder / connectionsRel,
            build_connections_json(*this->_basedie)
        );

        return true;
    }

    void Window::saveConfig() try {
        if (this->_basedie == nullptr) {
            QMessageBox::critical(
                this,
                QStringLiteral("Save Config"),
                QStringLiteral("No project loaded (basedie is null).")
            );
            return;
        }

        if (!this->hasConfigPath()) {
            this->saveConfigAs();
            return;
        }

        if (!this->writeConfigFolder(*this->_configPath, this->_configPath)) {
            return;
        }

        this->updateStatusLabel();

        QMessageBox::information(
            this,
            QStringLiteral("Save Config"),
            QStringLiteral("Config saved to:\n%1")
                .arg(QString::fromStdString(this->_configPath->string()))
        );
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Save Config")

    void Window::saveConfigAs() try {
        if (this->_basedie == nullptr) {
            QMessageBox::critical(
                this,
                QStringLiteral("Save Config As"),
                QStringLiteral("No project loaded (basedie is null).")
            );
            return;
        }

        QString startDir;
        if (this->hasConfigPath()) {
            startDir = QString::fromStdString(this->_configPath->string());
        }

        const auto selected = QFileDialog::getExistingDirectory(
            this,
            QStringLiteral("Save Config As — choose destination folder"),
            startDir,
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
        );
        if (selected.isEmpty()) {
            return;
        }

        auto destPath = std::FilePath{selected.toStdString()};
        if (!this->writeConfigFolder(destPath, this->_configPath)) {
            return;
        }

        this->_configPath.emplace(std::move(destPath));
        this->updateStatusLabel();

        QMessageBox::information(
            this,
            QStringLiteral("Save Config As"),
            QStringLiteral("Config saved to:\n%1")
                .arg(QString::fromStdString(this->_configPath->string()))
        );
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Save Config As")

    void Window::executePlace() try {
        if (this->_placed || this->_finishPR) {
            QMessageBox::information(
                this,
                QStringLiteral("Execute Place"),
                QStringLiteral("Placement is already applied. Use Edit Design to undo it.")
            );
            return;
        }
        if (this->_placing || this->_routing) {
            return;
        }

        auto topdies = this->collectTopdies();
        if (topdies.empty()) {
            QMessageBox::warning(
                this,
                QStringLiteral("Execute Place"),
                QStringLiteral("No chip instances to place.")
            );
            return;
        }

        const auto answer = QMessageBox::question(
            this,
            QStringLiteral("Execute Place"),
            QStringLiteral(
                "Start automatic placement?\n\n"
                "Chips will be assigned to TOBs. Layout will lock afterwards.\n"
                "Use Edit Design to restore the current placement."),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        if (answer != QMessageBox::Yes) {
            return;
        }

        this->capturePlacementSnapshot();

        auto dialog = QDialog(this);
        dialog.setWindowTitle(QStringLiteral("Place — busy"));
        dialog.setModal(true);
        dialog.setWindowFlags(
            (dialog.windowFlags() | Qt::CustomizeWindowHint | Qt::WindowTitleHint)
            & ~Qt::WindowCloseButtonHint
        );

        QVBoxLayout layout(&dialog);
        layout.setContentsMargins(20, 16, 20, 18);
        layout.setSpacing(14);

        auto* header = new QHBoxLayout();
        header->setContentsMargins(0, 0, 0, 0);

        auto title = QLabel(QStringLiteral("Placement Process …"));
        auto titleFont = title.font();
        titleFont.setPointSize(15);
        titleFont.setBold(true);
        title.setFont(titleFont);

        auto countLabel = QLabel();
        auto countFont = countLabel.font();
        countFont.setPointSize(22);
        countFont.setBold(true);
        countLabel.setFont(countFont);
        countLabel.setTextFormat(Qt::RichText);
        countLabel.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        auto setCount = [&countLabel](int iteration) {
            countLabel.setText(
                QStringLiteral(
                    "<span style='color:%1'>%2</span>"
                    "<span style='color:%3;font-size:16pt;font-weight:500'> iteration</span>")
                    .arg(QLatin1String(ChromeTokens::accent))
                    .arg(iteration)
                    .arg(QLatin1String(ChromeTokens::textMuted)));
        };
        setCount(0);

        header->addWidget(&title, 1);
        header->addWidget(&countLabel, 0);
        layout.addLayout(header);

        auto chart = PlaceProgressChart();
        layout.addWidget(&chart);

        dialog.setFixedSize(448, 228);

        this->_placing = true;
        this->updateStatusLabel();

        bool success = false;
        QString message;

        auto* worker = new PlaceThread{this->_interposer.get(), this->_basedie.get()};
        connect(worker, &PlaceThread::placeFinished, &dialog,
            [&dialog, &success, &message](bool ok, const QString& msg) {
                success = ok;
                message = msg;
                dialog.accept();
            });
        connect(
            worker,
            &PlaceThread::placeProgress,
            &dialog,
            [&chart, setCount](int iteration, qint64 cost) {
                setCount(iteration);
                chart.append(static_cast<qreal>(cost));
            },
            Qt::QueuedConnection);
        connect(worker, &PlaceThread::finished, worker, &QObject::deleteLater);
        worker->start();

        dialog.exec();
        worker->wait();

        this->_placing = false;
        this->updateStatusLabel();

        if (!success) {
            this->restorePlacementSnapshot();
            this->_placementSnapshot.clear();
            if (this->_layoutWidget != nullptr) {
                this->_layoutWidget->reload();
            }
            QMessageBox::critical(
                this,
                QStringLiteral("Execute Place"),
                message.isEmpty() ? QStringLiteral("Placement failed") : message
            );
            return;
        }

        if (this->_layoutWidget != nullptr) {
            this->_layoutWidget->reload();
        }
        if (this->_schematicWidget != nullptr) {
            this->_schematicWidget->arrangeFromPlacement();
        }

        this->enterPlacedStage();
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Execute Placement")

    void Window::executePlaceRoute() try {
        if (this->_finishPR) {
            QMessageBox::critical(
                this,
                "Execute Route",
                "Routing already finished!"
            );

            return;
        }
        if (this->_placing || this->_routing) {
            return;
        }

        // U26: preflight summary (nets, default-coord exports, idle TOBs)
        std::size_t connectionCount = 0;
        for (const auto& [mode, inner] : this->_basedie->connections()) {
            (void)mode;
            for (const auto& [sync, vec] : inner) {
                (void)sync;
                connectionCount += vec.size();
            }
        }

        std::size_t idleTobCount = 0;
        for (const auto& [coord, tob] : this->_interposer->tobs()) {
            (void)coord;
            if (tob->is_idle()) {
                ++idleTobCount;
            }
        }

        // GUI "Add Export" seeds TrackCoord{}; treat still-default as possibly unset.
        const hardware::TrackCoord defaultExportCoord {};
        std::size_t defaultCoordExportCount = 0;
        for (const auto& [name, eport] : this->_basedie->external_ports()) {
            (void)name;
            if (eport->coord() == defaultExportCoord) {
                ++defaultCoordExportCount;
            }
        }

        auto summary = QStringLiteral(
            "Start routing?\n\n"
            "Connections (nets): %1\n"
            "Idle TOBs: %2\n")
            .arg(connectionCount)
            .arg(idleTobCount);
        if (defaultCoordExportCount > 0) {
            summary += QStringLiteral(
                "Exports still at default coord (0,0,vert,0): %1\n"
                "(Likely unset after Add Export — set coords in Schematic.)\n")
                .arg(defaultCoordExportCount);
        }

        const auto answer = QMessageBox::question(
            this,
            QStringLiteral("Execute Route"),
            summary,
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        if (answer != QMessageBox::Yes) {
            return;
        }

        // U12: keep modal for wait()/thread correctness; clarify busy (no cancel).
        auto dialog = QDialog(this);
        dialog.setWindowTitle(QStringLiteral("Route — busy"));
        dialog.setModal(true);
        dialog.setWindowFlags(
            (dialog.windowFlags() | Qt::CustomizeWindowHint | Qt::WindowTitleHint)
            & ~Qt::WindowCloseButtonHint
        );

        QVBoxLayout layout(&dialog);
        layout.setContentsMargins(20, 16, 20, 18);
        layout.setSpacing(14);

        auto* header = new QHBoxLayout();
        header->setContentsMargins(0, 0, 0, 0);

        auto title = QLabel(QStringLiteral("Routing in progress…"));
        auto titleFont = title.font();
        titleFont.setPointSize(15);
        titleFont.setBold(true);
        title.setFont(titleFont);

        auto countLabel = QLabel();
        auto countFont = countLabel.font();
        countFont.setPointSize(22);
        countFont.setBold(true);
        countLabel.setFont(countFont);
        countLabel.setTextFormat(Qt::RichText);
        countLabel.setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        auto setCount = [&countLabel](int done, int total) {
            countLabel.setText(
                QStringLiteral(
                    "<span style='color:%1'>%2</span>"
                    "<span style='color:%3;font-size:16pt;font-weight:500'>/%4</span>")
                    .arg(QLatin1String(ChromeTokens::accent))
                    .arg(done)
                    .arg(QLatin1String(ChromeTokens::textMuted))
                    .arg(total));
        };
        setCount(0, 0);

        header->addWidget(&title, 1);
        header->addWidget(&countLabel, 0);
        layout.addLayout(header);

        auto track = RouteProgressTrack();
        track.setProgress(0, 0);
        layout.addWidget(&track);

        dialog.setFixedSize(448, 220);

        this->_routing = true;
        this->updateStatusLabel();

        bool success = false;
        QString message;

        auto *worker = new PRThread{this->_interposer.get(), this->_basedie.get()};
        connect(worker, &PRThread::prFinished, &dialog,
            [&dialog, &success, &message](bool ok, const QString& msg) {
                success = ok;
                message = msg;
                dialog.accept();
            });
        connect(
            worker,
            &PRThread::routeProgress,
            &dialog,
            [&track, setCount](int done, int total) {
                setCount(done, total);
                track.setProgress(done, total);
            },
            Qt::QueuedConnection);
        connect(worker, &PRThread::finished, worker, &QObject::deleteLater);
        worker->start();

        dialog.exec();
        worker->wait();

        this->_routing = false;
        this->updateStatusLabel();

        if (!success) {
            QMessageBox::critical(
                this,
                "Execute Route",
                message.isEmpty() ? QStringLiteral("Routing failed") : message
            );
            return;
        }

        this->_view2DWidget->reload();
        this->_view3DWidget->reload();
        this->_view3DWidget->displayRoutingResult();

        this->enterResultsStage();
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Execute Routing")

    void Window::generateControlBitAs() try {
        assert(this->_finishPR == true);

        ControlBitExportDialog dialog{this};
        if (dialog.exec() != QDialog::Accepted) {
            return;
        }

        auto output_root = dialog.outputDir().trimmed();
        if (output_root.isEmpty()) {
            QMessageBox::warning(this, QStringLiteral("Export Controlbits"), QStringLiteral("Please select an output directory"));
            return;
        }

        const bool simplify = dialog.simplify();
        parse::connect_registers(this->_interposer.get(), this->_basedie.get(), 0);
        parse::write_control_bits(
            this->_interposer.get(),
            output_root.toStdString(),
            0,
            simplify,
            this->_register_map);

        const auto out_dir =
            QDir{output_root}.filePath(QStringLiteral("regnamecontrolbit_4part"));
        QMessageBox::information(
            this,
            QStringLiteral("Export Controlbits"),
            QStringLiteral("Controlbits written to:\n%1").arg(out_dir));
        // U17: leave export path visible after the MessageBox is dismissed.
        this->statusBar()->showMessage(out_dir, 15000);
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Generate control bit file")

    auto Window::hasConfigPath() -> bool {
        return this->_configPath.has_value();
    }

    void Window::onPlaceCta() {
        if (this->_placed || this->_finishPR) {
            this->editDesign();
        } else {
            this->executePlace();
        }
    }

    void Window::editDesign() {
        const bool hadRouting = this->_finishPR;
        const bool hadPlacement = !this->_placementSnapshot.empty();

        QString body;
        if (hadRouting && hadPlacement) {
            body = QStringLiteral(
                "Return to Design stage?\n\n"
                "Routing results will be discarded.\n"
                "Placement will be restored to the positions from before Place.\n"
                "2D / 3D will be locked again until routing succeeds.");
        } else if (hadRouting) {
            body = QStringLiteral(
                "Return to Design stage?\n\n"
                "Current routing results will be discarded for editing purposes.\n"
                "2D / 3D will be locked again until routing succeeds.");
        } else {
            body = QStringLiteral(
                "Undo placement?\n\n"
                "Chip positions will be restored to those from before Place.\n"
                "Layout will be editable again.");
        }

        const auto reply = QMessageBox::question(
            this,
            QStringLiteral("Edit Design"),
            body,
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        if (reply != QMessageBox::Yes) {
            return;
        }

        for (auto& [_, nets] : this->_basedie->nets()) {
            for (auto& net : nets) {
                net->clear_path();
            }
        }
        for (auto& [_, instance] : this->_basedie->topdie_insts()) {
            instance->clear_nets();
        }
        this->_basedie->nets().clear();
        this->_interposer->reset_regs();

        this->_finishPR = false;
        this->_placed = false;

        if (hadPlacement) {
            this->restorePlacementSnapshot();
            if (this->_layoutWidget != nullptr) {
                this->_layoutWidget->reload();
            }
            if (this->_schematicWidget != nullptr) {
                this->_schematicWidget->arrangeFromPlacement();
            }
        }
        this->_placementSnapshot.clear();

        if (this->_stackedWidget != nullptr) {
            const auto* current = this->_stackedWidget->currentWidget();
            if (current == this->_view2DWidget || current == this->_view3DWidget) {
                this->switchToView(
                    this->_schematicWidget,
                    this->_schematicAction,
                    QString{});
            }
        }

        this->applyDesignEditability();
        this->updateStageUi();
        this->updateStatusLabel();

        this->statusBar()->showMessage(
            hadRouting
                ? QStringLiteral("Returned to Design — routing results discarded for editing")
                : QStringLiteral("Returned to Design — placement undone"),
            8000);
    }

    void Window::enterPlacedStage() {
        this->_placed = true;
        this->applyDesignEditability();
        this->updateStageUi();
        this->updateStatusLabel();
    }

    void Window::enterResultsStage() {
        this->_finishPR = true;
        this->applyDesignEditability();
        this->updateStageUi();
        this->updateStatusLabel();
    }

    void Window::applyDesignEditability() {
        const bool results = this->_finishPR;
        const bool layoutLocked = this->_placed || results;
        const bool loadBlocked = this->_placed || results;

        if (this->_schematicWidget != nullptr) {
            this->_schematicWidget->setLookbackLocked(results);
        }
        if (this->_layoutWidget != nullptr) {
            this->_layoutWidget->setLookbackLocked(layoutLocked);
            this->_layoutWidget->setLockBannerText(
                results
                    ? QStringLiteral("This view is locked after routing. Use Edit Design to edit.")
                    : QStringLiteral("This view is locked after placement. Use Edit Design to edit."));
        }

        if (this->_loadAction != nullptr) {
            this->_loadAction->setEnabled(!loadBlocked);
            this->_loadAction->setToolTip(
                loadBlocked
                    ? QStringLiteral("Disabled — use Edit Design first")
                    : QString{});
        }

        if (this->_view3DWidget != nullptr) {
            this->_view3DWidget->setCobRegisterEditEnabled(!results);
        }

        constexpr auto kReadOnlySuffix = " — Results (read-only lookback)";
        auto title = this->windowTitle();
        if (title.isEmpty()) {
            title = QStringLiteral("PR_tool");
        }
        if (title.endsWith(QLatin1String(kReadOnlySuffix))) {
            title.chop(static_cast<int>(sizeof(kReadOnlySuffix) - 1));
        }
        constexpr auto kLegacySuffix = " — read-only after P&R";
        if (title.endsWith(QLatin1String(kLegacySuffix))) {
            title.chop(static_cast<int>(sizeof(kLegacySuffix) - 1));
        }
        if (results) {
            this->setWindowTitle(title + QLatin1String(kReadOnlySuffix));
        } else {
            this->setWindowTitle(title);
        }
    }

    void Window::updateStageUi() {
        const bool results = this->_finishPR;
        const bool editMode = this->_placed || results;

        if (this->_stageLabel != nullptr) {
            if (results) {
                this->_stageLabel->setText(QStringLiteral("Stage: Results"));
            } else if (this->_placed) {
                this->_stageLabel->setText(QStringLiteral("Stage: Placed"));
            } else {
                this->_stageLabel->setText(QStringLiteral("Stage: Design"));
            }
        }

        if (this->_view2DAction != nullptr) {
            this->_view2DAction->setEnabled(results);
            this->_view2DAction->setText(QStringLiteral("2D"));
            this->_view2DAction->setToolTip(
                results ? QStringLiteral("2D")
                        : QStringLiteral("Locked until routing succeeds"));
        }
        if (this->_view3DAction != nullptr) {
            this->_view3DAction->setEnabled(results);
            this->_view3DAction->setText(QStringLiteral("3D"));
            this->_view3DAction->setToolTip(
                results ? QStringLiteral("3D")
                        : QStringLiteral("Locked until routing succeeds"));
        }

        if (this->_placeAction != nullptr) {
            this->_placeAction->setEnabled(!editMode);
        }
        if (this->_placeRouteAction != nullptr) {
            this->_placeRouteAction->setEnabled(!results);
        }
        if (this->_generateControlBitAction != nullptr) {
            this->_generateControlBitAction->setEnabled(results);
        }

        if (this->_placeCtaAction != nullptr) {
            if (editMode) {
                this->_placeCtaAction->setText(QStringLiteral("Edit Design"));
                this->_placeCtaAction->setToolTip(
                    results
                        ? QStringLiteral("Discard routing results and resume design editing")
                        : QStringLiteral("Undo placement and resume layout editing"));
                this->_placeCtaAction->setStatusTip(
                    results
                        ? QStringLiteral("Return to Design stage (discards routing and placement)")
                        : QStringLiteral("Undo placement"));
            } else {
                this->_placeCtaAction->setText(QStringLiteral("Place"));
                this->_placeCtaAction->setToolTip(QStringLiteral("Run automatic placement"));
                this->_placeCtaAction->setStatusTip(QStringLiteral("Run placement"));
            }
            this->_placeCtaAction->setEnabled(true);
        }

        if (this->_primaryCtaAction != nullptr) {
            this->_primaryCtaAction->setText(QStringLiteral("Route"));
            this->_primaryCtaAction->setEnabled(!results);
            this->_primaryCtaAction->setToolTip(
                results
                    ? QStringLiteral("Locked in Results — use Edit Design first")
                    : QStringLiteral("Run routing with the current Layout placement"));
            this->_primaryCtaAction->setStatusTip(
                results ? QStringLiteral("Locked until Edit Design")
                        : QStringLiteral("Run routing"));
        }
    }

    void Window::switchToView(QWidget* page, QAction* action, const QString& tip) {
        if (this->_stackedWidget == nullptr || page == nullptr) {
            return;
        }
        this->_stackedWidget->setCurrentWidget(page);
        if (action != nullptr) {
            action->setChecked(true);
        }
        this->updateStatusLabel();
        if (tip.isEmpty()) {
            this->statusBar()->clearMessage();
        } else {
            this->statusBar()->showMessage(tip);
        }
    }

    void Window::tryShortcutView(
        QWidget* page,
        QAction* action,
        const QString& tip,
        bool requiresResults
    ) {
        if (requiresResults && !this->_finishPR) {
            this->statusBar()->showMessage(
                QStringLiteral("View locked until routing succeeds"),
                5000);
            return;
        }
        this->switchToView(page, action, tip);
    }

    auto Window::currentPageName() const -> QString {
        if (this->_stackedWidget == nullptr) {
            return QStringLiteral("Schematic");
        }
        const auto* current = this->_stackedWidget->currentWidget();
        if (current == this->_layoutWidget) {
            return QStringLiteral("Layout");
        }
        if (current == this->_view2DWidget) {
            return QStringLiteral("2D");
        }
        if (current == this->_view3DWidget) {
            return QStringLiteral("3D");
        }
        if (current == this->_settingWidget) {
            return QStringLiteral("Settings");
        }
        return QStringLiteral("Schematic");
    }

    auto Window::isSchematicPage() const -> bool {
        return this->_stackedWidget != nullptr
            && this->_stackedWidget->currentWidget() == this->_schematicWidget;
    }

    auto Window::routeStatusText() const -> QString {
        if (this->_placing) {
            return QStringLiteral("Placing…");
        }
        if (this->_routing) {
            return QStringLiteral("Routing…");
        }
        if (this->_finishPR) {
            return QStringLiteral("Routed");
        }
        if (this->_placed) {
            return QStringLiteral("Placed");
        }
        return QStringLiteral("Ready");
    }

    auto Window::currentGraphicsView() const -> GraphicsView* {
        if (this->_stackedWidget == nullptr) {
            return nullptr;
        }
        const auto* current = this->_stackedWidget->currentWidget();
        if (current == this->_schematicWidget) {
            return this->_schematicWidget->graphicsView();
        }
        if (current == this->_layoutWidget) {
            return this->_layoutWidget->graphicsView();
        }
        if (current == this->_view2DWidget) {
            return this->_view2DWidget->graphicsView();
        }
        return nullptr;
    }

    void Window::updateStatusLabel() {
        if (this->_stageLabel != nullptr) {
            if (this->_finishPR) {
                this->_stageLabel->setText(QStringLiteral("Stage: Results"));
            } else if (this->_placed) {
                this->_stageLabel->setText(QStringLiteral("Stage: Placed"));
            } else {
                this->_stageLabel->setText(QStringLiteral("Stage: Design"));
            }
        }
        if (this->isSchematicPage()
            && this->_schematicWidget != nullptr
            && this->_schematicWidget->schematicView() != nullptr) {
            if (this->statusBar() != nullptr) {
                this->statusBar()->clearMessage();
            }
            if (this->_detailLabel != nullptr) {
                const auto line = this->_schematicWidget->schematicView()->statusLine();
                schematic::SchematicTypography::applyStatus(this->_detailLabel);
                if (line.startsWith(QLatin1String("Ctrl+Wheel"))) {
                    const int sep = line.indexOf(QLatin1String(" | "));
                    const auto hint = sep < 0 ? line : line.left(sep);
                    const auto rest = sep < 0 ? QString{} : line.mid(sep);
                    this->_detailLabel->setTextFormat(Qt::RichText);
                    this->_detailLabel->setText(
                        QStringLiteral("<span style='color:%1'>%2</span>"
                                       "<span style='color:%3'>%4</span>")
                            .arg(
                                QLatin1String(ChromeTokens::disabledText),
                                hint.toHtmlEscaped(),
                                QLatin1String(ChromeTokens::textMuted),
                                rest.toHtmlEscaped()));
                } else {
                    this->_detailLabel->setTextFormat(Qt::PlainText);
                    this->_detailLabel->setText(line);
                }
                this->_detailLabel->show();
            }
            if (this->_statusLabel != nullptr) {
                this->_statusLabel->setText(QStringLiteral("Schematic"));
            }
            if (this->_routeLabel != nullptr) {
                this->_routeLabel->setText(this->routeStatusText());
                this->_routeLabel->show();
            }
            return;
        }

        if (this->_detailLabel != nullptr) {
            this->_detailLabel->clear();
            this->_detailLabel->setTextFormat(Qt::PlainText);
            schematic::SchematicTypography::applyStatus(this->_detailLabel);
            this->_detailLabel->hide();
        }
        if (this->_routeLabel != nullptr) {
            this->_routeLabel->clear();
            this->_routeLabel->hide();
        }

        if (this->_statusLabel == nullptr) {
            return;
        }

        const auto path = this->hasConfigPath()
            ? QString::fromStdString(this->_configPath->string())
            : QStringLiteral("unsaved");
        const auto page = this->currentPageName();

        this->_statusLabel->setText(QStringLiteral("%1 | %2").arg(page, path));
    }

    auto Window::collectTopdies() -> std::Vector<circuit::TopDieInstance*> {
        std::Vector<circuit::TopDieInstance*> topdies;
        if (this->_basedie == nullptr) {
            return topdies;
        }
        for (auto& [name, inst] : this->_basedie->topdie_insts()) {
            (void)name;
            topdies.push_back(inst.get());
        }
        return topdies;
    }

    void Window::capturePlacementSnapshot() {
        this->_placementSnapshot.clear();
        auto topdies = this->collectTopdies();
        if (topdies.empty()) {
            return;
        }
        this->_placementSnapshot = algo::SAPlaceStrategy{}.save_current_placement(topdies);
    }

    void Window::restorePlacementSnapshot() {
        if (this->_placementSnapshot.empty()) {
            return;
        }
        try {
            auto topdies = this->collectTopdies();
            algo::SAPlaceStrategy{}.restore_placement(topdies, this->_placementSnapshot);
        } catch (const std::exception& e) {
            QMessageBox::critical(
                this,
                QStringLiteral("Restore placement"),
                QStringLiteral("Failed to restore placement:\n%1")
                    .arg(QString::fromUtf8(e.what()))
            );
        }
    }

    Window::~Window() {}

}
