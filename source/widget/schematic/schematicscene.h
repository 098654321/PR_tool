#pragma once

#include "qchar.h"
#include "qhash.h"
#include "qset.h"
#include "qvector.h"
#include "widget/schematic/item/sourceportitem.h"
#include <QGraphicsScene>
#include <QPair>
#include <QSet>
#include <optional>

namespace PR_tool::circuit {
    class TopDieInstance;
    class TopDie;
    class ExternalPort;
    class BaseDie;
    class Connection;
    class Pin;
}

namespace PR_tool::hardware {
    class Interposer;
}

namespace PR_tool::widget {

    namespace schematic {
        class NetPointItem;
        class NetItem;
        class PinItem;
        class TopDieInstanceItem;
        class ExternalPortItem;
        class PortGroupItem;
        class ExportPortGroupHost;
        class SourcePortItem;
        class PowerRailItem;
    }

    class SchematicScene : public QGraphicsScene {
        Q_OBJECT
        
    public:
        SchematicScene(circuit::BaseDie* basedie, hardware::Interposer* interposer);

    signals:
        void netSelected(schematic::NetItem* net);
        void topdieInstSelected(schematic::TopDieInstanceItem* topdieinst);
        void exportSelected(schematic::ExternalPortItem* eport);
        void viewSelected();
        void layoutChanged();

    public slots:
        void flushPortGroupSync();

    public:
        void reloadItems();
        /// Deferred Port Group rebuild (safe vs paint / item lifetime).
        void requestPortGroupSync(schematic::TopDieInstanceItem* item);
        void requestExportPortGroupSync();

        /// Ch.七 connection focus (hover / select share visuals).
        void setHoverTopDie(schematic::TopDieInstanceItem* die);
        void setHoverPin(schematic::PinItem* pin);
        void setHoverNet(schematic::NetItem* net);
        void setHoverPortGroup(schematic::PortGroupItem* group);
        void onTopDieSelectionChanged(schematic::TopDieInstanceItem* die, bool selected);
        void refreshConnectionFocus();
        /// Select pin + highlight connected nets; locate zooms to Near.
        void focusPin(schematic::PinItem* pin, bool locate);
        /// Ch.八: rebuild VDD/GND rail + stubs after die move / power net change.
        void refreshPowerRails();
        /// Ch.九: recompute bus bundles (endpoint-pair collapse / Near expand).
        void refreshBusBundling();

    protected:
        void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
        void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
        void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) override;

    private:
        void emitSelectionForItem(QGraphicsItem* item);
        void enforceSingleTopDieSelection(schematic::TopDieInstanceItem* keep);
        auto expandToBundleNets(schematic::NetItem* seed) const -> QSet<schematic::NetItem*>;
        auto netsForPortGroup(schematic::PortGroupItem* group) const -> QSet<schematic::NetItem*>;
        auto netsTouchingDie(schematic::TopDieInstanceItem* die) const -> QSet<schematic::NetItem*>;
        auto endpointTopDies(schematic::NetItem* net) const -> QSet<schematic::TopDieInstanceItem*>;
        auto endpointPins(schematic::NetItem* net) const -> QSet<schematic::PinItem*>;
        /// Stable endpoint-container pair key (TopDieInst / ExternalPort); nullopt if ineligible.
        auto endpointPairKey(schematic::NetItem* net) const -> std::optional<QPair<quintptr, quintptr>>;
        auto viewScale() const -> qreal;

    public:
        auto addExPort(circuit::ExternalPort*) -> schematic::ExternalPortItem*;
        auto addTopDieInst(circuit::TopDieInstance* inst) -> schematic::TopDieInstanceItem*;
        auto addNet(circuit::Connection* connection) -> schematic::NetItem*;
        auto addVDDSourcePort(const QString& name) -> schematic::SourcePortItem*;
        auto addGNDSourcePort(const QString& name) -> schematic::SourcePortItem*;

    public:
        void removeExternalPort(schematic::ExternalPortItem* eport);
        void removeTopDieInstance(schematic::TopDieInstanceItem* inst);
        void removeNet(schematic::NetItem* net);
        
    private:
        auto addNetPoint(schematic::PinItem* pin) -> schematic::NetPointItem*;

    public:
        /*
            Load all item from the basedie!
        */
        void addSceneItems();

    public:
        void headleCreateNet(schematic::PinItem* pin, QGraphicsSceneMouseEvent* event);
        void handleInitialTopDie(circuit::TopDie* topdie);
        void handleAddExport();
        void handleAddVdd();
        void handleAddGnd();

        /// Cancel floating topdie / export / net placement (Right-click or Esc).
        void cancelFloatingPlacement();

    public:
        auto topdieinstMap() -> QHash<circuit::TopDieInstance*, schematic::TopDieInstanceItem*>& 
        { return this->_topdieinstMap; }

    private:
        void addTopDieInstItems();
        void addExternalPortItems();
        void placeExternalPortsByConnections();
        void syncExportPortGroups();
        void addNetItems();
        /// Ch.八: hide physical VDD/GND nets; draw rail + stubs instead.
        void applyPowerNetPresentation(schematic::NetItem* net);
        /// Ch.九: auto-bundle parallel nets between the same endpoint pair (zoom expand).
        void markBundleNets();
        static auto isPowerConnection(const circuit::Connection* connection) -> bool;

    private:
        auto circuitPinToPinItem(const circuit::Pin& pin) -> schematic::PinItem*;

    private:
        void placeFloatingTopdDieInst();
        void cleanFloatingTopdDieInst();

        void placeFloatingExPort();
        void cleanFloatingExPort();

        void cleanFloatingNet();

        /// Grow/fit view scene rect so newly placed items stay reachable (S3).
        void adjustSceneRect();

    protected:
        circuit::BaseDie* _basedie;
        hardware::Interposer* _interposer;

        QHash<circuit::TopDieInstance*, schematic::TopDieInstanceItem*> _topdieinstMap;
        QHash<circuit::ExternalPort*, schematic::ExternalPortItem*> _exportMap;
        QSet<schematic::NetItem*> _nets;
        QVector<schematic::SourcePortItem*> _vddPorts;
        QVector<schematic::SourcePortItem*> _gndPorts;
        schematic::PowerRailItem* _vddRail {nullptr};
        schematic::PowerRailItem* _gndRail {nullptr};

        // Temp var 
        schematic::NetItem* _floatingNet {nullptr};
        schematic::TopDieInstanceItem* _floatingTopdDieInst {nullptr};
        schematic::ExternalPortItem* _floatingExPort {nullptr};
        schematic::ExportPortGroupHost* _exportGroupHost {nullptr};

        QSet<schematic::TopDieInstanceItem*> _pendingTopDieGroupSync {};
        bool _pendingExportGroupSync {false};
        bool _portGroupFlushScheduled {false};

        // Ch.七 focus state (hover temporarily overrides selected when both set).
        schematic::TopDieInstanceItem* _hoverTopDie {nullptr};
        schematic::TopDieInstanceItem* _selectedTopDie {nullptr};
        QSet<schematic::NetItem*> _hoverFocusNets {};
        QSet<schematic::NetItem*> _selectedFocusNets {};
        bool _refreshingFocus {false};
    };

}