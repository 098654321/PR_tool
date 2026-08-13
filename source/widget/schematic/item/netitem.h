#pragma once

#include "qcolor.h"
#include "qglobal.h"
#include "qgraphicssceneevent.h"
#include <cassert>
#include <std/utility.hh>
#include <QGraphicsLineItem>
#include <QPen>
#include <QString>
#include <QVector>
#include <QDebug>

namespace PR_tool::circuit {
    class Connection;
}

namespace PR_tool::widget::schematic {

    class PinItem;
    class NetPointItem;

    /// Tunable Ch.七 “弱背景 + 强交互” visual params (schematic connections).
    /// Ch.八: General / External / other non-power share one encoding; bus may be
    /// slightly thicker (BUNDLE_WIDTH). Selected/hover is focus, not a third linestyle.
    struct ConnectionFocusStyle {
        static constexpr qreal DEFAULT_WIDTH = 1.0;
        static constexpr qreal DEFAULT_OPACITY = 0.35;
        /// Mild bus/bundle thickening (same color family). Full bundling is Ch.九.
        static constexpr qreal BUNDLE_WIDTH = 1.8;

        static constexpr qreal DIE_RELATED_WIDTH = 2.0;
        static constexpr qreal DIE_RELATED_OPACITY = 1.0;
        static constexpr qreal DIE_UNRELATED_OPACITY = 0.15;

        static constexpr qreal NET_FOCUS_WIDTH = 2.5;
        static constexpr qreal NET_FOCUS_OPACITY = 1.0;
        static constexpr qreal NET_UNRELATED_OPACITY = 0.08;

        static constexpr qreal DIE_BORDER_DEFAULT = 1.5;
        /// Ch.七 strong die border; Ch.21 selected uses DIE_RELATED_WIDTH (2px), not this.
        static constexpr qreal DIE_BORDER_FOCUS = 3.0;
    };

    enum class NetFocusRole {
        Default,
        DieRelated,
        DieUnrelated,
        NetFocused,
        NetUnrelated,
    };

    /// Ch.21 item chrome (TopDie / pin / port group). Dim % comes from Ch.七 only.
    enum class ItemChromeState {
        Normal,
        Hover,
        Selected,
        Related,
        Dimmed,
    };

    inline auto itemChromeWidth(ItemChromeState state) -> qreal {
        switch (state) {
            case ItemChromeState::Hover:
            case ItemChromeState::Selected:
                return ConnectionFocusStyle::DIE_RELATED_WIDTH;
            default:
                return ConnectionFocusStyle::DIE_BORDER_DEFAULT;
        }
    }

    inline auto itemChromeBorder(ItemChromeState state) -> QColor {
        switch (state) {
            case ItemChromeState::Hover:
                return QColor(50, 50, 50);
            case ItemChromeState::Selected:
                return QColor(24, 24, 24);
            case ItemChromeState::Related:
                return QColor(70, 100, 140);
            case ItemChromeState::Dimmed:
                return QColor(130, 130, 130);
            case ItemChromeState::Normal:
            default:
                return QColor(80, 80, 80);
        }
    }

    inline auto itemChromeDimOpacity(bool netLevelFocus) -> qreal {
        return netLevelFocus
            ? ConnectionFocusStyle::NET_UNRELATED_OPACITY
            : ConnectionFocusStyle::DIE_UNRELATED_OPACITY;
    }

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

        /// Ch.八 / Ch.九: mark as bus-bundle member for mild default thickening.
        void setBundleMember(bool on) { this->_bundleMember = on; }
        auto isBundleMember() const -> bool { return this->_bundleMember; }

        /// Ch.九: count/range label on the visible representative when collapsed.
        void setBundleLabel(const QString& label);
        auto bundleLabel() const -> const QString& { return this->_bundleLabel; }

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
        bool _bundleMember {false};
        QString _bundleLabel {};
    };

}