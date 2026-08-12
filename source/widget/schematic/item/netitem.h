#pragma once

#include "qcolor.h"
#include "qglobal.h"
#include "qgraphicssceneevent.h"
#include <cassert>
#include <std/utility.hh>
#include <QGraphicsLineItem>
#include <QPen>
#include <QVector>
#include <QDebug>

namespace PR_tool::circuit {
    class Connection;
}

namespace PR_tool::widget::schematic {

    class PinItem;
    class NetPointItem;

    /// Tunable Ch.七 “弱背景 + 强交互” visual params (schematic connections).
    struct ConnectionFocusStyle {
        static constexpr qreal DEFAULT_WIDTH = 1.0;
        static constexpr qreal DEFAULT_OPACITY = 0.35;

        static constexpr qreal DIE_RELATED_WIDTH = 2.0;
        static constexpr qreal DIE_RELATED_OPACITY = 1.0;
        static constexpr qreal DIE_UNRELATED_OPACITY = 0.15;

        static constexpr qreal NET_FOCUS_WIDTH = 2.5;
        static constexpr qreal NET_FOCUS_OPACITY = 1.0;
        static constexpr qreal NET_UNRELATED_OPACITY = 0.08;

        static constexpr qreal DIE_BORDER_DEFAULT = 1.5;
        static constexpr qreal DIE_BORDER_FOCUS = 3.0;
    };

    enum class NetFocusRole {
        Default,
        DieRelated,
        DieUnrelated,
        NetFocused,
        NetUnrelated,
    };

    class NetItem : public QGraphicsItem {
    public:
        static const     QColor DEFAULT_COLOR;
        static constexpr qreal  DEFAULT_WIDTH = ConnectionFocusStyle::DEFAULT_WIDTH;

        static const     QColor HOVER_COLOR;

        enum { Type = UserType + 3 };
        int type() const override { return Type; }
        
    public:
        NetItem(circuit::Connection* connection, NetPointItem* beginPoint, NetPointItem* endPoint);
        NetItem(NetPointItem* beginPoint);

    public:
        void updateLine();
        void updatePositionFrom(NetPointItem* pointItem, const QPointF& pos);
        void updateEndPoint(const QPointF& point);
        void addPoint(const QPointF& point);

        void updateConnectPin(NetPointItem* point);

    private:
        void updateBeginPosition(const QPointF& pos);
        void updateEndPosition(const QPointF& pos);
        void updateBeginPin(PinItem* pin);
        void updateEndPin(PinItem* pin);
        void updatePath();

    public:
        void setLine(const QPointF& begin, const QPointF& end);
        /// Replace geometry with an orthogonal polyline (schematic display only).
        void setRoutePoints(const QVector<QPointF>& points);
        /// Apply Ch.七 focus role (width / opacity / contrast). Floating nets ignored.
        void applyFocusRole(NetFocusRole role);

    protected:
        auto boundingRect() const -> QRectF override;
        auto shape() const -> QPainterPath override;
        void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget = nullptr) override;

        void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
        void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

    public:
        auto beginPoint() const -> NetPointItem* 
        { return this->_beginPoint; }
        
        auto endPoint() const -> NetPointItem* 
        { return this->_endPoint; }
        
        void setEndPoint(NetPointItem* point) 
        { this->_endPoint = point; }
        
        auto color() const -> const QColor& 
        { return this->_color; }
        
        void setColor(const QColor& color)
        { this->_color = color; this->_paintColor = color; }

        auto width() const -> qreal 
        { return this->_width; }

        auto setWidth(qreal width) 
        { this->_width = width; this->_paintWidth = width; }

        void resetPaint();
        auto isFloating() const -> bool;

    public:
        void wrap(circuit::Connection* connection) 
        { assert(this->_connection == nullptr); this->_connection = connection; }

        auto unwrap() const -> circuit::Connection* {
            assert(this->_connection != nullptr);
            return this->_connection;
        }

    protected:
        circuit::Connection* _connection {nullptr};

        NetPointItem* _beginPoint {nullptr};
        NetPointItem* _endPoint {nullptr};

        QColor _paintColor {DEFAULT_COLOR};
        qreal  _paintWidth {DEFAULT_WIDTH};
        qreal  _paintOpacity {ConnectionFocusStyle::DEFAULT_OPACITY};
        Qt::PenStyle _paintStyle {Qt::SolidLine};
        
        QColor _color {DEFAULT_COLOR};
        qreal  _width {DEFAULT_WIDTH};

        QVector<QPointF> _points;
        QPainterPath _path;
        std::Option<QPointF> _tempPoint;
        QPointF _end;
    };

}