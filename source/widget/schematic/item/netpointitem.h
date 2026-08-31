#pragma once

#include <QGraphicsLineItem>
#include <QPainterPath>
#include <QPen>
#include <QDebug>

namespace PR_tool::widget::schematic {

    class PinItem;
    class NetItem;
    
    class NetPointItem : public QGraphicsEllipseItem {
    public:
        // Ch.十二: junction ● (not a mid-wire crossing marker).
        static constexpr qreal JUNCTION_RADIUS = 2.75;
        static constexpr qreal JUNCTION_FOCUS_SCALE = 1.3;
        static constexpr qreal RADIUS = JUNCTION_RADIUS;
        static constexpr qreal DIAMETER = 2. * RADIUS;
        // Extra invisible margin so net points stay clickable after default view scale(1/2.5).
        static constexpr qreal HIT_PADDING = 5.;

        static constexpr qreal MOVING_RADIUS = 8.;
        static constexpr qreal MOVING_DIAMETER = 2. * MOVING_RADIUS;
        static const     QColor COLOR;
        static const     QColor HOVER_COLOR;
    
        enum { Type = UserType + 4 };
        int type() const override { return Type; }

    public:
        NetPointItem(PinItem* connectedPin);

    public:
        void linkToPin(PinItem* pin);
        auto unlinkPin() -> PinItem*;

        auto netItem() const -> NetItem* { return this->_netitem; }
        void setNetItem(NetItem* netitem) 
        { this->_netitem = netitem; }

        void updatePos();

        /// Ch.十二 + Ch.七: junction ● follows net focus width/opacity; dim with weak nets.
        void applyNetVisual(qreal opacity, bool focused);

        auto boundingRect() const -> QRectF override;
        auto shape() const -> QPainterPath override;
        void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr) override;

    protected:
        QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

    protected:
        void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
        void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
        void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
        void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
        void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
        void mouseDoubleClickEvent(QGraphicsSceneMouseEvent*) override;

    public:
        auto connectedPin() const -> PinItem* { return this->_connectedPin; }

    private:
        /// True electrical join (multi-net on pin / edit handle) — not a mere wire cross.
        auto shouldDrawJunction() const -> bool;
        void refreshRect();

    private:
        bool _dragging {false};
        bool _hovered {false};
        bool _focused {false};
        qreal _opacity {1.0};
        PinItem* _connectedPin {nullptr};
        NetItem* _netitem {nullptr};
    };

}
