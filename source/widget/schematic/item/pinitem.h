#pragma once

#include <QColor>
#include <QGraphicsItem>
#include <QPainter>
#include <QPainterPath>
#include <QDebug>
#include <circuit/connection/pin.hh>

namespace PR_tool::widget {
   class SchematicScene;
}

namespace PR_tool::widget::schematic {

    enum class PinSide {
        Top,
        Left,
        Right,
        Bottom,
    };

    class NetItem;
    class NetPointItem;
    class TopDieInstanceItem;
    class ExternalPortItem;
    class SourcePortItem;

    class PinItem : public QGraphicsItem {
    public:
    
        static constexpr qreal PIN_RADIUS = 5.;
        static constexpr qreal PIN_DIAMETER = 2 * PIN_RADIUS;
        // Extra invisible margin so pins stay clickable after default view scale(1/2.5).
        static constexpr qreal HIT_PADDING = 5.;
        static constexpr qreal NAME_INTERVAL = 10.;
        static constexpr qreal CHAR_WIDTH_ = 10.;
        static constexpr qreal CHAR_HEIGHT = 20.;
        // Pin LOD (Ch.五): s = QGraphicsView::transform().m11(). Default open ≈ 0.40 → Far.
        static constexpr qreal LOD_FAR_MAX = 0.45;   // s < Far: hide pin / group graphics
        static constexpr qreal LOD_NEAR_MIN = 0.90;  // s >= Near: draw pin mark + name
        // Medium: LOD_FAR_MAX <= s < LOD_NEAR_MIN → Port Group bars (Ch.六), not per-pin ticks/names
        static const    QColor COLOR;
        static const    QColor HOVERED_COLOR;

        enum { Type = UserType + 5 };
        int type() const override { return Type; }
    
    public:
        PinItem(
            const QString &name, 
            QPointF position, 
            PinSide side, 
            QGraphicsItem *parent = nullptr
        );

    public:        
        auto boundingRect() const -> QRectF override;
        auto shape() const -> QPainterPath override;
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) override;
        auto itemChange(GraphicsItemChange change, const QVariant& value) -> QVariant override;

    protected:
        void mousePressEvent(QGraphicsSceneMouseEvent*) override;
        void hoverEnterEvent(QGraphicsSceneHoverEvent *) override;
        void hoverLeaveEvent(QGraphicsSceneHoverEvent *) override;

    public:
        auto isExternalPortPin() const -> bool;
        auto isTopDieInstancePin() const -> bool;
        auto isSourcePortPin() const -> bool;

    public:
        auto toString() const -> QString;
        auto toCircuitPin() const -> circuit::Pin;

    public: 
        auto name() const -> const QString& { return this->_name; }
        void setName(const QString& name) { this->_name = name; }

        auto side() const -> PinSide { return this->_side; }
        void setSide(PinSide side) { this->_side = side; this->update(); }

        auto connectedPoints() const -> const QVector<NetPointItem*>& 
        { return this->_connectedNetPoints; }

        void addConnectedPoint(NetPointItem* point) 
        { this->_connectedNetPoints.push_back(point); }

        void removeConnectedPoint(NetPointItem* point)
        { this->_connectedNetPoints.removeOne(point); }

        void setRaduis(qreal radius) { this->_raduis = radius; }
        void resetRaduis() { this->_raduis = PIN_RADIUS; }

        /// Display-only hover style (also used when a connected net is hovered).
        void setHovered(bool hovered);
        /// Ch.七: force-show when pin is on a focused / related net.
        void setFocusRelated(bool related);

    public:
        auto parentExternalPort() const -> ExternalPortItem*;
        auto parentTopDieInstance() const -> TopDieInstanceItem*;
        auto parentSourcePort() const -> SourcePortItem*;

    private:
        /// View scale s = transform().m11(); 1.0 if no view yet.
        auto viewScale() const -> qreal;
        /// Selected / hover / related-net focus force-show (Ch.五 + Ch.七).
        auto shouldForceShowPin() const -> bool;

    private:
        QString _name;
        PinSide _side;

        qreal _raduis {PIN_RADIUS};
        bool _hovered {false};
        bool _focusRelated {false};

        QVector<NetPointItem*> _connectedNetPoints {};
    };


}