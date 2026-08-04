#include "./window.h"
#include "./prthread.h"
#include "./view2d/view2dwidget.h"
#include "./view3d/view3dwidget.h"
#include "./schematic/schematicwidget.h"
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

#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

#include <std/exception.hh>

#include <QDebug>
#include <QResizeEvent>
#include <QVBoxLayout>
#include <QToolBar>
#include <QStackedWidget>
#include <QToolButton>
#include <QKeySequence>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QDialog>
#include <QLabel>
#include <QProgressBar>
#include <QThread>
#include <QStatusBar>

namespace PR_tool::widget {

    Window::Window(QWidget *parent)
        : QMainWindow{parent}
    {
        this->createSystem();
        
        this->createMenuBar();
        this->createToolBar();
        this->createCentralWidget();
        this->createStatusBar();

        this->resize(1500, 900);
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
        auto loadAction= new QAction("Load", fileMenu);
        fileMenu->addAction(loadAction);
        
        fileMenu->addSeparator(); 

        // Save
        auto saveAction= new QAction("Save", fileMenu);
        fileMenu->addAction(saveAction);

        // Save as
        auto saveAsAction= new QAction("Save as", fileMenu);
        fileMenu->addAction(saveAsAction);

        fileMenu->addSeparator(); 

        // Exit
        auto exitAction = new QAction("Exit", fileMenu);
        fileMenu->addAction(exitAction);

        this->_menuBar->addMenu(fileMenu);

        connect(loadAction, &QAction::triggered, this, &Window::loadConfig);
        connect(saveAction, &QAction::triggered, this, &Window::saveConfig);
        connect(saveAsAction, &QAction::triggered, this, &Window::saveConfigAs);
        connect(exitAction, &QAction::triggered, this, &Window::close);

        // ====================== View ======================
        auto viewMenu= new QMenu("View", this->_menuBar);
        auto themesAction = new QAction{"Themes", viewMenu};
        viewMenu->addAction(themesAction);

        this->_menuBar->addMenu(viewMenu);

        // ====================== Help ======================
        auto helpMenu= new QMenu("Help", this->_menuBar);
        auto aboutAction= new QAction("About", helpMenu);
        helpMenu->addAction(aboutAction);

        auto aboutQTAction= new QAction("About Qt", helpMenu);
        helpMenu->addAction(aboutQTAction);

        this->_menuBar->addMenu(helpMenu);
    }

    void Window::createToolBar() {
        this->_toolBar = new QToolBar(this);
        this->_toolBar->setOrientation(Qt::Vertical);
        this->_toolBar->setMovable(false);
        this->_toolBar->setMinimumWidth(50);
        this->_toolBar->setIconSize(QSize(35, 35));
        this->_toolBar->setToolButtonStyle(Qt::ToolButtonIconOnly);
        this->addToolBar(Qt::LeftToolBarArea, this->_toolBar);

        // Page buttons (icon-only; text/tooltip for recognition)
        auto schematicButton = this->_toolBar->addAction(QIcon(":/image/image/icon/chip.png"), "Schematic");
        schematicButton->setToolTip("Schematic");
        schematicButton->setStatusTip("Switch to Schematic view");
        schematicButton->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_1));

        auto layoutButton = this->_toolBar->addAction(QIcon(":/image/image/icon/layout.png"), "Layout");
        layoutButton->setToolTip("Layout");
        layoutButton->setStatusTip("Switch to Layout view");
        layoutButton->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_2));

        auto view2DButton = this->_toolBar->addAction(QIcon(":/image/image/icon/view2d.png"), "View 2D");
        view2DButton->setToolTip("View 2D");
        view2DButton->setStatusTip("Switch to 2D view");
        view2DButton->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_3));

        auto view3DButton = this->_toolBar->addAction(QIcon(":/image/image/icon/view3d.png"), "View 3D");
        view3DButton->setToolTip("View 3D");
        view3DButton->setStatusTip("Switch to 3D view");
        view3DButton->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_4));

        connect(schematicButton, &QAction::triggered, [this]() {
            this->_stackedWidget->setCurrentWidget(this->_schematicWidget);
        });
        connect(layoutButton, &QAction::triggered, [this]() {
            this->_stackedWidget->setCurrentWidget(this->_layoutWidget);
        });
        connect(view2DButton, &QAction::triggered, [this] () {
            this->_stackedWidget->setCurrentWidget(this->_view2DWidget);
        });
        connect(view3DButton, &QAction::triggered, [this] () {
            this->_stackedWidget->setCurrentWidget(this->_view3DWidget);
        });

        this->_toolBar->addSeparator();

        QWidget *stretch = new QWidget(this->_toolBar);
        stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        this->_toolBar->addWidget(stretch);

        this->_placeRouteAction = this->_toolBar->addAction(QIcon{":/image/image/icon/execute.png"}, "Place & Route");
        this->_placeRouteAction->setToolTip("Place & Route");
        this->_placeRouteAction->setStatusTip("Run place and route");
        connect(this->_placeRouteAction, &QAction::triggered, this, &Window::executePlaceRoute);

        this->_generateControlBitAction = this->_toolBar->addAction(QIcon{":/image/image/icon/save.png"}, "Export Controlbits");
        this->_generateControlBitAction->setToolTip("Export Controlbits");
        this->_generateControlBitAction->setStatusTip("Export controlbits to output directory");
        this->_generateControlBitAction->setEnabled(false);
        connect(this->_generateControlBitAction, &QAction::triggered, this, &Window::generateControlBitAs);

        this->_toolBar->addSeparator();

        auto settingButton = this->_toolBar->addAction(QIcon{":/image/image/icon/setting.png"}, "Settings");
        settingButton->setToolTip("Settings");
        settingButton->setStatusTip("Open Settings");
        connect(settingButton, &QAction::triggered, [this] () {
            this->_stackedWidget->setCurrentWidget(this->_settingWidget);
        });
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
    }

    void Window::createStatusBar() {
        auto statusBar = this->statusBar();

        this->_statusLabel = new QLabel{QStringLiteral("PR_tool"), this};
        this->_statusLabel->setAlignment(Qt::AlignCenter);
        this->_statusLabel->setMinimumWidth(200);

        statusBar->addPermanentWidget(this->_statusLabel);
    }

    void Window::loadConfig() try {
        if (this->_finishPR) {
            QMessageBox::critical(\
                this,
                "Load Config",
                "Can't load new config after finishing P&R"
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

        // Load a new one!
        auto filePath = QFileDialog::getExistingDirectory(this, "Select Config path");
        if (!filePath.isEmpty()) {
            // MARK: For loss origin data, it should be these, buf box<net> in basedie...
            // Once faild, loss all 

            // auto configPath = std::FilePath{filePath.toStdString()};
            // auto [i, b] = parse::read_config(configPath);

            // this->_interposer->clear();
            // this->_basedie->clear();

            // *(this->_interposer) = std::move(*i);
            // *(this->_basedie) = std::move(*b);

            // this->_schematicWidget->reload();
            // this->_layoutWidget->reload();

            // this->_configPath.emplace(std::move(configPath));

            auto configPath = std::FilePath{filePath.toStdString()};

            if (this->hasConfigPath()) {
                this->_interposer->clear();
                this->_basedie->clear();
            }

            this->_register_map = parse::read_config(configPath, this->_interposer.get(), this->_basedie.get(), 0, false);

            this->_schematicWidget->reload();
            this->_layoutWidget->reload();

            this->_configPath.emplace(std::move(configPath));
        }
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Load Config")

    void Window::saveConfig() {

    }

    void Window::saveConfigAs() {
        
    }

    void Window::executePlaceRoute() try {
        if (this->_finishPR) {
            QMessageBox::critical(
                this,
                "Execute Place & Route",
                "P&R already finished!"
            );

            return;
        }

        auto dialog = QDialog(this);
        dialog.setWindowTitle("Execute Place & Route");
        dialog.setModal(true);

        QVBoxLayout layout(&dialog);
        auto title = QLabel(QStringLiteral("Place & Route in progress…"));
        auto font = title.font();
        font.setPointSize(16);
        font.setBold(true);
        title.setFont(font);
        title.setAlignment(Qt::AlignCenter);
        layout.addWidget(&title);

        auto hint = QLabel(QStringLiteral("Please wait. This window closes when finished."));
        hint.setAlignment(Qt::AlignCenter);
        hint.setWordWrap(true);
        layout.addWidget(&hint);

        auto progress = QProgressBar();
        progress.setRange(0, 0); // indeterminate — no real % without algo cooperation
        layout.addWidget(&progress);

        dialog.setFixedSize(400, 200);

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

        if (!success) {
            QMessageBox::critical(
                this,
                "Execute Place & Route",
                message.isEmpty() ? QStringLiteral("P&R failed") : message
            );
            return;
        }

        this->_view2DWidget->reload();
        this->_view3DWidget->displayRoutingResult();

        this->disableEdit();

        assert(this->_generateControlBitAction != nullptr);
        this->_generateControlBitAction->setEnabled(true);
        this->_placeRouteAction->setEnabled(false);
        this->_finishPR = true;

        QMessageBox::information(
            this,
            QStringLiteral("Place & Route Complete"),
            QStringLiteral(
                "Schematic and layout editing are now locked for this session.\n\n"
                "Export Controlbits is enabled. To edit again, reload the application."));
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
            QMessageBox::warning(this, QStringLiteral("导出控制位"), QStringLiteral("请选择输出目录"));
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
            QStringLiteral("导出控制位"),
            QStringLiteral("已写出控制位到：\n%1").arg(out_dir));
    }
    QMESSAGEBOX_REPORT_EXCEPTION("Generate control bit file")

    auto Window::hasConfigPath() -> bool {
        return this->_configPath.has_value();
    }

    void Window::disableEdit() {
        this->_schematicWidget->setEnabled(false);
        this->_layoutWidget->setEnabled(false);

        if (this->_statusLabel != nullptr) {
            this->_statusLabel->setText(QStringLiteral(
                "Editing locked after P&R — use Export Controlbits; reload app to edit again"));
        }

        const auto suffix = QStringLiteral(" — read-only after P&R");
        auto title = this->windowTitle();
        if (title.isEmpty()) {
            title = QStringLiteral("PR_tool");
        }
        if (!title.endsWith(suffix)) {
            this->setWindowTitle(title + suffix);
        }
    }

    Window::~Window() {}

}
