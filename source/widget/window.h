#pragma once

#include "std/file.hh"
#include "std/memory.hh"
#include "std/utility.hh"
#include <parse/reader/config/config.hh>
#include <QMainWindow>

namespace PR_tool::hardware {
    class Interposer;
}

namespace PR_tool::circuit {
    class BaseDie;
}

class QToolBar;
class QStackedWidget;
class QMenuBar;
class QPushButton;
class QLabel;

namespace PR_tool::widget {

    class SchematicWidget;
    class LayoutWidget;
    class View3DWidget;
    class View2DWidget;
    class SettingWidget;

    class Window : public QMainWindow {
        Q_OBJECT

    public:
        Window(QWidget *parent = nullptr);
        ~Window();

    private:
        void createSystem();
        void createMenuBar();
        void createToolBar();
        void createCentralWidget();
        void createStatusBar();

    private:
        void loadConfig();
        void saveConfig();
        void saveConfigAs();
        
        void executePlaceRoute();
        void generateControlBitAs();

        /// Write in-memory project into `destFolder` (JSON config set).
        /// When `copyStaticFrom` is set and differs from dest, copy static
        /// files (config.json, interposer, topdies, ports_01, register map).
        auto writeConfigFolder(
            const std::FilePath& destFolder,
            const std::Option<std::FilePath>& copyStaticFrom
        ) -> bool;

    private:
        auto hasConfigPath() -> bool;
        void disableEdit();

    private:
        QMenuBar* _menuBar {nullptr};
        QToolBar* _toolBar {nullptr};
        QStackedWidget* _stackedWidget {nullptr};

        SchematicWidget* _schematicWidget {nullptr};
        LayoutWidget* _layoutWidget {nullptr};
        View2DWidget* _view2DWidget {nullptr};
        View3DWidget* _view3DWidget {nullptr};
        SettingWidget* _settingWidget {nullptr};

        QAction* _placeRouteAction {nullptr};
        QAction* _generateControlBitAction {nullptr};

        QLabel* _statusLabel {nullptr};

    private:
        std::Box<hardware::Interposer> _interposer {nullptr};
        std::Box<circuit::BaseDie> _basedie {nullptr};
        parse::RegisterMapConfig _register_map {};

        std::Option<std::FilePath> _configPath {}; 
        bool _finishPR {false};
    };

}

