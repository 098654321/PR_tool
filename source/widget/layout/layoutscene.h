#pragma once

#include "qglobal.h"
#include "qobject.h"
#include "qobjectdefs.h"
#include "qpoint.h"
#include "qvector.h"
#include <circuit/basedie.hh>
#include <circuit/export/export.hh>
#include <QGraphicsScene>
#include <QHash>
#include <QString>

namespace PR_tool::hardware {
    class TOB;
    class Interposer;
}

namespace PR_tool::circuit {
    class TopDieInstance;
    class BaseDie;
}

namespace PR_tool::widget {

    namespace layout {
        class NetItem;
        class PinItem;
        class TOBItem;
        class TopDieInstanceItem;
        class ExternalPortItem;
        class SourcePortItem;
    }

    class LayoutScene : public QGraphicsScene {  
        Q_OBJECT

        static constexpr qreal TOB_INTERVAL = 30.0;
        static constexpr qreal INTERPOSER_SIDE_GAP = 30.0;
        static constexpr qreal EXPORT_SIDE_GAP = 150.0;

        static_assert(EXPORT_SIDE_GAP > INTERPOSER_SIDE_GAP);

        static const   QPointF INTERPOSER_LEFT_DOWN_POSITION;
        static const   QPointF INTERPOSER_RIGHT_UP_POSITION;

        static const   QPointF EXPORT_LEFT_DOWN_POSITION;
        static const   QPointF EXPORT_RIGHT_UP_POSITION;

    public:
        LayoutScene(hardware::Interposer* interposer, circuit::BaseDie* basedie, QObject* parent = nullptr);

    signals:
        void layoutChanged();

    public:
        void reloadItems();

    public:
        auto addNet(layout::PinItem* beginPoint, layout::PinItem* endPoint) -> layout::NetItem*;
        auto addTOB(hardware::TOB* tob) -> layout::TOBItem*;
        auto addTopDieInstance(circuit::TopDieInstance* topdieInst, layout::TOBItem* tob) -> layout::TopDieInstanceItem*;
        auto addExternalPort(circuit::ExternalPort* eport) -> layout::ExternalPortItem*;
        auto addVDDSourcePort() -> layout::SourcePortItem*;
        auto addGNDSourcePort() -> layout::SourcePortItem*;

    private:
        void addSceneItems();

        void addTOBItems();
        void addTopDieInstanceItems();
        void addExternalPortItems();
        void addNetItems();
        void syncDefaultPlacement();

    public:
        /// Estimated total wire length (HPWL), same formula as SA placer net_cost.
        auto estimatedTotalWireLength() -> qint64;
        void choiseSourcePort();

        /// Highlight the TOB hosting the named TopDieInstance and center views on it.
        void focusTopDieInstance(const QString& name);
        void clearTopDieHighlight();

        /// Restore each TopDieInstance to the TOB captured before manual Layout edits.
        void restoreDefaultPlacement();

    private:
        auto circuitPinToPinItem(const circuit::Pin& pin) -> layout::PinItem*;
        auto estimatedHpwlFromNets() -> qint64;
        auto estimatedHpwlFromConnections() -> qint64;
        
    private:
        static auto getMinDistancePort(const QVector<layout::SourcePortItem*> ports, const QPointF& targetPos) -> layout::SourcePortItem*;
        static auto pinDistance(layout::PinItem* pin1, layout::PinItem* pin2) -> qreal;
        static auto pointDistance(const QPointF& p1, const QPointF& p2) -> qreal;
        static auto tobPosition(const hardware::TOBCoord& coord) -> QPointF;

    protected:
        circuit::BaseDie* _basedie;
        hardware::Interposer* _interposer;

        QHash<circuit::TopDieInstance*, layout::TopDieInstanceItem*> _topdieinstMap {};
        QHash<circuit::ExternalPort*, layout::ExternalPortItem*> _externalPortsMap {};
        QHash<hardware::TOB*, layout::TOBItem*> _tobsMaps {};
        QVector<layout::NetItem*> _nets {};
        QVector<layout::SourcePortItem*> _vddPorts {};
        QVector<layout::SourcePortItem*> _gndPorts {};
        QVector<layout::NetItem*> _netsWithSourcePorts {};
        layout::TOBItem* _highlightedTOB {nullptr};
        layout::TopDieInstanceItem* _highlightedTopDie {nullptr};
        QHash<circuit::TopDieInstance*, hardware::TOB*> _defaultPlacement {};
    };


}