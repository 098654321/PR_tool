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

    private:
        std::Box<hardware::Interposer> _interposer {nullptr};
        std::Box<circuit::BaseDie> _basedie {nullptr};
        parse::RegisterMapConfig _register_map {};

        std::Option<std::FilePath> _configPath {}; 
        bool _finishPR {false};
    };

}

