#pragma once

#include <QGraphicsItem>
#include <QVector>
#include <QPointF>
#include <QColor>

namespace PR_tool::widget::schematic {

    /// Schematic-only VDD/GND expression (Ch.八): rail + short stubs to dies.
    /// Not a physical net item — power NetItems stay hidden by default.
    class PowerRailItem : public QGraphicsItem {
    public:
        enum class Kind { Vdd, Gnd };

        enum { Type = UserType + 10 };
        int type() const override { return Type; }

        static const QColor VDD_COLOR;
        static const QColor GND_COLOR;

        static constexpr qreal RAIL_WIDTH = 2.5;
        static constexpr qreal STUB_WIDTH = 1.5;
        static constexpr qreal SYMBOL_SIZE = 10.;

        explicit PowerRailItem(Kind kind, QGraphicsItem* parent = nullptr);

        /// Scene-space layout: horizontal rail and stub endpoints on die bodies.
        void setLayout(qreal railY, qreal x0, qreal x1, const QVector<QPointF>& stubEnds);

        auto kind() const -> Kind { return this->_kind; }

    protected:
        auto boundingRect() const -> QRectF override;
        void paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) override;

    private:
        Kind _kind;
        qreal _railY {0};
        qreal _x0 {0};
        qreal _x1 {0};
        QVector<QPointF> _stubEnds {};
        QRectF _bounds {};
    };

}
