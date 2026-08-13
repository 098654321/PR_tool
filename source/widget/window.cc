#include "./window.h"
#include "./prthread.h"
#include "./view2d/view2dwidget.h"
#include "./view3d/view3dwidget.h"
#include "./schematic/schematicwidget.h"
#include "./schematic/schematicview.h"
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

#include <cassert>
#include <parse/reader/module.hh>
#include <widget/frame/msgexception.h>
#include <widget/frame/controlbitexportdialog.h>
#include <widget/frame/graphicsview.h>

#include <hardware/interposer.hh>
#include <hardware/track/trackcoord.hh>
#include <circuit/basedie.hh>
#include <circuit/connection/pin.hh>

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
#include <QToolButton>
#include <QKeySequence>
#include <QShortcut>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QDialog>
#include <QLabel>
#include <QProgressBar>
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

        // Place & Route (same command as Design CTA; keep shortcut)
        this->_placeRouteAction = new QAction(QStringLiteral("Place & Route"), fileMenu);
        this->_placeRouteAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
        this->_placeRouteAction->setStatusTip(QStringLiteral("Run place and route"));
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
        this->addToolBar(Qt::TopToolBarArea, this->_toolBar);

        // Segmented view switcher (same actions as View menu)
        this->_toolBar->addAction(this->_schematicAction);
        this->_toolBar->addAction(this->_layoutAction);
        this->_toolBar->addAction(this->_view2DAction);
        this->_toolBar->addAction(this->_view3DAction);

        this->_toolBar->addSeparator();

        QWidget* stretch = new QWidget(this->_toolBar);
        stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        this->_toolBar->addWidget(stretch);

        // Single primary CTA slot: Run P&R ↔ Edit Design
        this->_primaryCtaAction = this->_toolBar->addAction(QStringLiteral("Run P&R"));
        this->_primaryCtaAction->setToolTip(QStringLiteral("Run Place & Route"));
        this->_primaryCtaAction->setStatusTip(QStringLiteral("Run place and route"));
        connect(this->_primaryCtaAction, &QAction::triggered, this, &Window::onPrimaryCta);

        this->_toolBar->addAction(this->_generateControlBitAction);

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

        for (QAction* action : {
                 this->_schematicAction, this->_layoutAction, this->_view2DAction,
                 this->_view3DAction, this->_primaryCtaAction, this->_generateControlBitAction}) {
            if (auto* button = qobject_cast<QToolButton*>(this->_toolBar->widgetForAction(action))) {
                button->setAccessibleName(action->text());
                button->setAccessibleDescription(action->statusTip());
            }
        }

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
        this->_stageLabel->setMinimumWidth(120);
        statusBar->addWidget(this->_stageLabel);

        this->_detailLabel = new QLabel{this};
        this->_detailLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        statusBar->addWidget(this->_detailLabel, 1);

        this->_statusLabel = new QLabel{this};
        this->_statusLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        this->_statusLabel->setMinimumWidth(160);

        statusBar->addPermanentWidget(this->_statusLabel);
        this->updateStatusLabel();
    }

    void Window::loadConfig() {
        if (this->_finishPR) {
            QMessageBox::information(
                this,
                QStringLiteral("Load Config"),
                QStringLiteral(
                    "Results stage is active.\n"
                    "Use Edit Design to discard routing results before loading a new config.")
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

    void Window::executePlaceRoute() try {
        if (this->_finishPR) {
            QMessageBox::critical(
                this,
                "Execute Place & Route",
                "P&R already finished!"
            );

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
            "Start Place & Route?\n\n"
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
        summary += QStringLiteral(
            "\nNote: GUI routes with the current Layout placement only "
            "(no automatic placer).\n"
            "Router: maze (mode 0). SAT/router picker not in GUI this phase.\n"
            "On success the app enters Results stage (Edit Design to return).");

        const auto answer = QMessageBox::question(
            this,
            QStringLiteral("Execute Place & Route"),
            summary,
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        if (answer != QMessageBox::Yes) {
            return;
        }

        // U12: keep modal for wait()/thread correctness; clarify busy (no cancel).
        auto dialog = QDialog(this);
        dialog.setWindowTitle(QStringLiteral("Place & Route — busy"));
        dialog.setModal(true);
        dialog.setWindowFlags(
            (dialog.windowFlags() | Qt::CustomizeWindowHint | Qt::WindowTitleHint)
            & ~Qt::WindowCloseButtonHint
        );

        QVBoxLayout layout(&dialog);
        auto title = QLabel(QStringLiteral("Place & Route in progress…"));
        auto font = title.font();
        font.setPointSize(16);
        font.setBold(true);
        title.setFont(font);
        title.setAlignment(Qt::AlignCenter);
        layout.addWidget(&title);

        auto hint = QLabel(QStringLiteral(
            "Main window is blocked until P&R finishes. "
            "This dialog closes when finished (no cancel)."));
        hint.setAlignment(Qt::AlignCenter);
        hint.setWordWrap(true);
        layout.addWidget(&hint);

        auto progress = QProgressBar();
        progress.setRange(0, 0); // indeterminate — no real % without algo cooperation
        layout.addWidget(&progress);

        dialog.setFixedSize(400, 200);

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
        connect(worker, &PRThread::finished, worker, &QObject::deleteLater);
        worker->start();

        dialog.exec();
        worker->wait();

        this->_routing = false;
        this->updateStatusLabel();

        if (!success) {
            QMessageBox::critical(
                this,
                "Execute Place & Route",
                message.isEmpty() ? QStringLiteral("P&R failed") : message
            );
            return;
        }

        this->_view2DWidget->reload();
        this->_view3DWidget->reload();
        this->_view3DWidget->displayRoutingResult();

        this->enterResultsStage();

        QMessageBox::information(
            this,
            QStringLiteral("Place & Route Complete"),
            QStringLiteral(
                "Entered Results stage.\n"
                "Schematic and Layout are read-only lookback.\n"
                "2D / 3D are unlocked for viewing results.\n"
                "Export Controlbits is enabled.\n\n"
                "Use Edit Design to discard results and resume editing."));
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Execute Place & Routing")

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

    void Window::onPrimaryCta() {
        if (this->_finishPR) {
            this->editDesign();
        } else {
            this->executePlaceRoute();
        }
    }

    void Window::editDesign() {
        const auto reply = QMessageBox::question(
            this,
            QStringLiteral("Edit Design"),
            QStringLiteral(
                "Return to Design stage?\n\n"
                "Current routing results will be discarded for editing purposes.\n"
                "2D / 3D will be locked again until Place & Route succeeds."),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No
        );
        if (reply != QMessageBox::Yes) {
            return;
        }

        this->_finishPR = false;

        // If looking at result views, return to Schematic for editing.
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
            QStringLiteral("Returned to Design — routing results discarded for editing"),
            8000);
    }

    void Window::enterResultsStage() {
        this->_finishPR = true;
        this->applyDesignEditability();
        this->updateStageUi();
        this->updateStatusLabel();
    }

    void Window::applyDesignEditability() {
        const bool results = this->_finishPR;

        // Results: Sch/Layout read-only lookback; Design: editable again.
        if (this->_schematicWidget != nullptr) {
            this->_schematicWidget->setEnabled(!results);
        }
        if (this->_layoutWidget != nullptr) {
            this->_layoutWidget->setEnabled(!results);
        }

        if (this->_loadAction != nullptr) {
            this->_loadAction->setEnabled(!results);
            this->_loadAction->setToolTip(
                results
                    ? QStringLiteral("Disabled in Results — use Edit Design first")
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
        // Also strip the older permanent-lock suffix if present.
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

        if (this->_stageLabel != nullptr) {
            this->_stageLabel->setText(
                results ? QStringLiteral("Stage: Results")
                        : QStringLiteral("Stage: Design"));
        }

        if (this->_view2DAction != nullptr) {
            this->_view2DAction->setEnabled(results);
            this->_view2DAction->setText(
                results ? QStringLiteral("2D") : QStringLiteral("2D 🔒"));
            this->_view2DAction->setToolTip(
                results ? QStringLiteral("2D")
                        : QStringLiteral("Locked until Place & Route succeeds"));
        }
        if (this->_view3DAction != nullptr) {
            this->_view3DAction->setEnabled(results);
            this->_view3DAction->setText(
                results ? QStringLiteral("3D") : QStringLiteral("3D 🔒"));
            this->_view3DAction->setToolTip(
                results ? QStringLiteral("3D")
                        : QStringLiteral("Locked until Place & Route succeeds"));
        }

        if (this->_placeRouteAction != nullptr) {
            this->_placeRouteAction->setEnabled(!results);
        }
        if (this->_generateControlBitAction != nullptr) {
            this->_generateControlBitAction->setEnabled(results);
        }

        if (this->_primaryCtaAction != nullptr) {
            if (results) {
                this->_primaryCtaAction->setText(QStringLiteral("Edit Design"));
                this->_primaryCtaAction->setToolTip(
                    QStringLiteral("Discard routing results and resume design editing"));
                this->_primaryCtaAction->setStatusTip(
                    QStringLiteral("Return to Design stage (discards routing results)"));
            } else {
                this->_primaryCtaAction->setText(QStringLiteral("Run P&R"));
                this->_primaryCtaAction->setToolTip(QStringLiteral("Run Place & Route"));
                this->_primaryCtaAction->setStatusTip(QStringLiteral("Run place and route"));
            }
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
                QStringLiteral("View locked until Place & Route succeeds"),
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
        if (this->_routing) {
            return QStringLiteral("Routing…");
        }
        if (this->_finishPR) {
            return QStringLiteral("Routed");
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
        if (this->_statusLabel == nullptr) {
            return;
        }

        if (this->_stageLabel != nullptr) {
            this->_stageLabel->setText(
                this->_finishPR ? QStringLiteral("Stage: Results")
                                : QStringLiteral("Stage: Design"));
        }

        if (this->isSchematicPage()
            && this->_schematicWidget != nullptr
            && this->_schematicWidget->schematicView() != nullptr) {
            if (this->statusBar() != nullptr) {
                this->statusBar()->clearMessage();
            }
            if (this->_detailLabel != nullptr) {
                this->_detailLabel->setText(this->_schematicWidget->schematicView()->statusLine());
                this->_detailLabel->show();
            }
            this->_statusLabel->setText(
                QStringLiteral("Schematic | %1").arg(this->routeStatusText()));
            return;
        }

        if (this->_detailLabel != nullptr) {
            this->_detailLabel->clear();
            this->_detailLabel->hide();
        }

        const auto path = this->hasConfigPath()
            ? QString::fromStdString(this->_configPath->string())
            : QStringLiteral("unsaved");
        const auto page = this->currentPageName();

        this->_statusLabel->setText(QStringLiteral("%1 | %2").arg(page, path));
    }

    Window::~Window() {}

}
