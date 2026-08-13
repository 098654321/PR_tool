#pragma once

#include "qobjectdefs.h"
#include <QWidget>

class QLabel;
class QLineEdit;
class QCheckBox;
class QTableView;
class QStandardItemModel;
class QSortFilterProxyModel;
class QToolButton;

namespace PR_tool::widget {
    class SchematicScene;
}

namespace PR_tool::widget::schematic {

    class TopDieInstanceItem;
    class PinItem;

    class TopDieInstanceInfoWidget : public QWidget {
        Q_OBJECT

    public:
        TopDieInstanceInfoWidget(SchematicScene* scene, QWidget* parent = nullptr);

    signals:
        void topdieInstanceRename(TopDieInstanceItem* inst, const QString& name);
        void removeTopDieInstance(TopDieInstanceItem* inst);

    public:
        void loadTopDieInstance(TopDieInstanceItem* inst);
        auto currentTopDieInstance() -> TopDieInstanceItem*;

    private:
        void buildUi();
        auto makeCollapsibleGroup(const QString& title, QWidget* body, bool* expandedState) -> QWidget*;
        void loadFoldState();
        void saveFoldState();
        void rebuildPinMapModel();
        void applyPinFilter();
        void updateConnectivityStats();
        auto pinItemForRow(int proxyRow) -> PinItem*;
        void onPinClicked(const QModelIndex& index);
        void onPinDoubleClicked(const QModelIndex& index);
        void onPinContextMenu(const QPoint& pos);
        void locatePin(PinItem* pin);
        void showPinConnections(PinItem* pin);
        void copyText(const QString& text);

    private:
        SchematicScene* _scene {nullptr};
        TopDieInstanceItem* _topdieInstance {nullptr};

        QLabel* _titleLabel {nullptr};
        QLineEdit* _nameEdit {nullptr};
        QLabel* _typeLabel {nullptr};
        QLabel* _positionLabel {nullptr};
        QLabel* _orientationLabel {nullptr};
        QCheckBox* _visibleToggle {nullptr};
        QLabel* _statusLabel {nullptr};

        QLabel* _connectionsLabel {nullptr};
        QLabel* _signalPinsLabel {nullptr};
        QLabel* _powerPinsLabel {nullptr};

        QLineEdit* _pinSearchEdit {nullptr};
        QTableView* _pinMapView {nullptr};
        QStandardItemModel* _pinMapModel {nullptr};
        QSortFilterProxyModel* _pinProxy {nullptr};

        bool _generalExpanded {true};
        bool _connectivityExpanded {true};
        bool _pinMapExpanded {true};
    };

}
