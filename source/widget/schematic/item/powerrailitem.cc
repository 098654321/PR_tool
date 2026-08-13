#include "./powerrailitem.h"

#include "../schematictypography.h"

#include <QPainter>
#include <QPen>
#include <algorithm>

namespace PR_tool::widget::schematic {

    const QColor PowerRailItem::VDD_COLOR = QColor(187, 51, 51);
    const QColor PowerRailItem::GND_COLOR = QColor(51, 51, 51);

    PowerRailItem::PowerRailItem(Kind kind, QGraphicsItem* parent) :
        QGraphicsItem{parent},
        _kind{kind}
    {
        this->setZValue(-0.2);
        this->setAcceptedMouseButtons(Qt::NoButton);
    }

    void PowerRailItem::setLayout(qreal railY, qreal x0, qreal x1, const QVector<QPointF>& stubEnds) {
        this->prepareGeometryChange();
        this->_railY = railY;
        this->_x0 = std::min(x0, x1);
        this->_x1 = std::max(x0, x1);
        this->_stubEnds = stubEnds;

        qreal top = railY;
        qreal bottom = railY;
        qreal left = this->_x0;
        qreal right = this->_x1;
        for (const auto& p : stubEnds) {
            top = std::min(top, std::min(railY, p.y()));
            bottom = std::max(bottom, std::max(railY, p.y()));
            left = std::min(left, p.x());
            right = std::max(right, p.x());
        }
        // Room for label + GND earth bars / VDD arrow tips.
        constexpr qreal pad = 18.;
        this->_bounds = QRectF{left - pad, top - pad, (right - left) + 2 * pad, (bottom - top) + 2 * pad};
        this->update();
    }

    auto PowerRailItem::boundingRect() const -> QRectF {
        return this->_bounds;
    }

    void PowerRailItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) {
        if (this->_x1 <= this->_x0 && this->_stubEnds.isEmpty()) {
            return;
        }

        const QColor color = this->_kind == Kind::Vdd ? VDD_COLOR : GND_COLOR;
        painter->setRenderHint(QPainter::Antialiasing, true);

        QPen railPen(color, RAIL_WIDTH, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter->setPen(railPen);
        painter->drawLine(QPointF{this->_x0, this->_railY}, QPointF{this->_x1, this->_railY});

        QPen stubPen(color, STUB_WIDTH, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter->setPen(stubPen);
        painter->setBrush(color);

        for (const auto& end : this->_stubEnds) {
            painter->drawLine(QPointF{end.x(), this->_railY}, end);

            if (this->_kind == Kind::Vdd) {
                // Arrow tip into the die (triangle pointing toward die).
                const qreal dir = (end.y() >= this->_railY) ? 1. : -1.;
                QPolygonF tip;
                tip << end
                    << QPointF{end.x() - SYMBOL_SIZE * 0.45, end.y() - dir * SYMBOL_SIZE}
                    << QPointF{end.x() + SYMBOL_SIZE * 0.45, end.y() - dir * SYMBOL_SIZE};
                painter->drawPolygon(tip);
            } else {
                // Earth bars at rail junction (classic GND).
                const qreal y = this->_railY;
                painter->drawLine(QPointF{end.x() - 8., y + 4.}, QPointF{end.x() + 8., y + 4.});
                painter->drawLine(QPointF{end.x() - 5., y + 8.}, QPointF{end.x() + 5., y + 8.});
                painter->drawLine(QPointF{end.x() - 2.5, y + 12.}, QPointF{end.x() + 2.5, y + 12.});
            }
        }

        painter->setFont(SchematicTypography::topDieTypeFont());
        painter->setPen(color);
        const QString label = this->_kind == Kind::Vdd ? QStringLiteral("VDD") : QStringLiteral("GND");
        const qreal labelX = this->_x1 - 28.;
        const qreal labelY = this->_kind == Kind::Vdd ? this->_railY - 6. : this->_railY - 4.;
        painter->drawText(QPointF{labelX, labelY}, label);
    }

}
