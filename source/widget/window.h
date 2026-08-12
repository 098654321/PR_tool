#pragma once

#include "std/file.hh"
#include "std/memory.hh"
#include "std/utility.hh"
#include <parse/reader/config/config.hh>
#include <QMainWindow>
#include <QString>

class QCloseEvent;

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
class QAction;
class QActionGroup;
class QMenu;

namespace PR_tool::widget {

    class SchematicWidget;
    class LayoutWidget;
    class View3DWidget;
    class View2DWidget;
    class SettingWidget;
    class GraphicsView;

    class Window : public QMainWindow {
        Q_OBJECT

    public:
        Window(QWidget *parent = nullptr);
        ~Window();

        /// Load config from an existing directory path (no file dialog).
        void loadConfigFromPath(const QString& path);

    protected:
        void closeEvent(QCloseEvent* event) override;

    private:
        void createSystem();
        void createMenuBar();
        void createToolBar();
        void createCentralWidget();
        void createStatusBar();
        void restoreWindowSettings();
        void saveWindowSettings() const;

    private:
        void loadConfig();
        void saveConfig();
        void saveConfigAs();
        
        void executePlaceRoute();
        void generateControlBitAs();
        void onPrimaryCta();
        void editDesign();
        void updateStageUi();
        void switchToView(QWidget* page, QAction* action, const QString& tip);
        void tryShortcutView(QWidget* page, QAction* action, const QString& tip, bool requiresResults);

        /// Write in-memory project into `destFolder` (JSON config set).
        /// When `copyStaticFrom` is set and differs from dest, copy static
        /// files (config.json, interposer, topdies, ports_01, register map).
        auto writeConfigFolder(
            const std::FilePath& destFolder,
            const std::Option<std::FilePath>& copyStaticFrom
        ) -> bool;

    private:
        auto hasConfigPath() -> bool;
        void enterResultsStage();
        void applyDesignEditability();
        void updateStatusLabel();
        auto currentPageName() const -> QString;
        auto currentGraphicsView() const -> GraphicsView*;

    private:
        QMenuBar* _menuBar {nullptr};
        QMenu* _viewMenu {nullptr};
        QToolBar* _toolBar {nullptr};
        QStackedWidget* _stackedWidget {nullptr};

        SchematicWidget* _schematicWidget {nullptr};
        LayoutWidget* _layoutWidget {nullptr};
        View2DWidget* _view2DWidget {nullptr};
        View3DWidget* _view3DWidget {nullptr};
        SettingWidget* _settingWidget {nullptr};

        QAction* _schematicAction {nullptr};
        QAction* _layoutAction {nullptr};
        QAction* _view2DAction {nullptr};
        QAction* _view3DAction {nullptr};
        QAction* _settingsAction {nullptr};
        QAction* _showNavigatorAction {nullptr};
        QAction* _showInspectorAction {nullptr};
        QActionGroup* _pageActionGroup {nullptr};

        QAction* _loadAction {nullptr};
        QAction* _placeRouteAction {nullptr};
        QAction* _primaryCtaAction {nullptr};
        QAction* _generateControlBitAction {nullptr};

        QLabel* _stageLabel {nullptr};
        QLabel* _statusLabel {nullptr};

    private:
        std::Box<hardware::Interposer> _interposer {nullptr};
        std::Box<circuit::BaseDie> _basedie {nullptr};
        parse::RegisterMapConfig _register_map {};

        std::Option<std::FilePath> _configPath {}; 
        bool _finishPR {false};
    };

}
