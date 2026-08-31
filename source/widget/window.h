#pragma once

#include "std/file.hh"
#include "std/memory.hh"
#include "std/utility.hh"
#include "std/collection.hh"
#include <parse/reader/config/config.hh>
#include <QMainWindow>
#include <QString>

class QCloseEvent;

namespace PR_tool::hardware {
    class Interposer;
    class TOB;
}

namespace PR_tool::circuit {
    class BaseDie;
    class TopDieInstance;
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
        
        void executePlace();
        void executePlaceRoute();
        void generateControlBitAs();
        void onPlaceCta();
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
        void enterPlacedStage();
        void enterResultsStage();
        void applyDesignEditability();
        void updateStatusLabel();
        auto collectTopdies() -> std::Vector<circuit::TopDieInstance*>;
        void capturePlacementSnapshot();
        void restorePlacementSnapshot();
        auto currentPageName() const -> QString;
        auto currentGraphicsView() const -> GraphicsView*;
        auto isSchematicPage() const -> bool;
        auto routeStatusText() const -> QString;

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
        QAction* _placeAction {nullptr};
        QAction* _placeCtaAction {nullptr};
        QAction* _placeRouteAction {nullptr};
        QAction* _primaryCtaAction {nullptr};
        QAction* _generateControlBitAction {nullptr};

        QLabel* _stageLabel {nullptr};
        QLabel* _detailLabel {nullptr};
        QLabel* _statusLabel {nullptr};
        QLabel* _routeLabel {nullptr};

    private:
        std::Box<hardware::Interposer> _interposer {nullptr};
        std::Box<circuit::BaseDie> _basedie {nullptr};
        parse::RegisterMapConfig _register_map {};

        std::Option<std::FilePath> _configPath {};
        bool _finishPR {false};
        bool _placed {false};
        bool _routing {false};
        bool _placing {false};
        std::HashMap<circuit::TopDieInstance*, hardware::TOB*> _placementSnapshot {};
    };

}
