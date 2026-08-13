#pragma once

#include <std/memory.hh>
#include <QWidget>
#include <circuit/topdie/topdie.hh>

class QLineEdit;
class QTreeWidget;
class QTreeWidgetItem;
class QHBoxLayout;
class QPushButton;
class QColor;
class QGraphicsItem;

namespace PR_tool::circuit {
    class BaseDie;
};

namespace PR_tool::widget {

    class SchematicScene;

    class SchematicLibWidget : public QWidget {
        Q_OBJECT

    public:
        SchematicLibWidget(
            circuit::BaseDie* basedie,
            SchematicScene* scene,
            QWidget* parent = nullptr);

    public:
        void reload();
        /// Ch.十五: highlight tree row matching canvas selection (no zoom).
        void syncSelectionFromCanvas(QGraphicsItem* item);

    signals:
        void initialTopDieInst(circuit::TopDie* topdie);
        void addExport();
        void addVdd();
        void addGnd();

    public:
        void onLoadTopDieClicked();
        void onLoadTopDiesClicked();

        void loadTopDie(const QString& path);
        void loadTopDies(const QString& path);

        void addTopDie(std::String name, std::HashMap<std::String, std::usize> pinmap);
        void addTopDie(circuit::TopDie* topdie);

    private:
        void buildUi();
        void rebuildPalette();
        void rebuildTree();
        void applySearchFilter();
        void filterTreeItem(QTreeWidgetItem* item, const QString& filter, bool forceVisible);
        void onTreeItemClicked(QTreeWidgetItem* item, int column);
        void pushConnectionFilter();
        auto makePaletteButton(const QString& text, const QColor& fill) -> QPushButton*;

        static constexpr int kNavRoleType = Qt::UserRole;
        static constexpr int kNavRolePtr = Qt::UserRole + 1;

        enum class NavKind : int {
            None = 0,
            TopDieInst = 1,
            ExternalPort = 2,
            Net = 3,
            SourcePort = 4,
        };

    private:
        QWidget* _paletteStrip {nullptr};
        QHBoxLayout* _paletteLayout {nullptr};
        QLineEdit* _searchEdit {nullptr};
        QPushButton* _filterSignal {nullptr};
        QPushButton* _filterBus {nullptr};
        QPushButton* _filterPower {nullptr};
        QPushButton* _filterGround {nullptr};
        QPushButton* _filterExternal {nullptr};
        QTreeWidget* _tree {nullptr};
        QTreeWidgetItem* _topDiesRoot {nullptr};
        QTreeWidgetItem* _portsRoot {nullptr};
        QTreeWidgetItem* _netsRoot {nullptr};
        QTreeWidgetItem* _powerRoot {nullptr};

        circuit::BaseDie* _basedie {nullptr};
        SchematicScene* _scene {nullptr};
        bool _syncingSelection {false};
    };

}
