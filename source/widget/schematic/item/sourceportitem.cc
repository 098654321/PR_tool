#include "./sourceportitem.h"
#include <circuit/export/export.hh>
#include "qobject.h"
#include "widget/schematic/item/griditem.h"
#include "widget/schematic/item/pinitem.h"

#include <QPen>
#include <QPolygonF>

namespace PR_tool::widget::schematic {
    
    const QColor SourcePortItem::VDD_COLOR = QColor::fromRgb(255, 100, 100, 100);
    const QColor SourcePortItem::GND_COLOR = QColor::fromRgb(100, 255, 100, 100);

    SourcePortItem::SourcePortItem(const QString& name, SourcePortType type) : 
        GridItem{},
        _pin{nullptr},
        _type{type},
        _width{0}
    {
        this->_pin = new PinItem{name, QPointF{0, 0}, PinSide::Left, this};
        this->_width = GridItem::snapToGrid(PIN_SIDE_INTERVAL + PinItem::NAME_INTERVAL + name.size() * PinItem::CHAR_WIDTH_ + PIN_SIDE_INTERVAL);

        this->setFlags(this->flags() | QGraphicsItem::ItemIsMovable);
        this->setZValue(0);
    }

    auto SourcePortItem::boundingRect() const -> QRectF {
        return QRectF {-PIN_SIDE_INTERVAL, -HEIGHT / 2., this->_width, HEIGHT};
    }

    void SourcePortItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) {
        // Ch.八: schematic power symbols (rail expression hides connected ports).
        painter->setRenderHint(QPainter::Antialiasing, true);
        const QRectF box = this->boundingRect();
        const QPointF c = box.center();

        if (this->_type == SourcePortType::VDD) {
            painter->setPen(QPen(QColor(187, 51, 51), 1.5));
            painter->setBrush(VDD_COLOR);
            QPolygonF tri;
            tri << QPointF{c.x(), box.top() + 4.}
                << QPointF{c.x() - 10., box.bottom() - 6.}
                << QPointF{c.x() + 10., box.bottom() - 6.};
            painter->drawPolygon(tri);
            painter->drawLine(QPointF{c.x(), box.bottom() - 6.}, QPointF{c.x(), box.bottom() - 2.});
        } else {
            painter->setPen(QPen(QColor(51, 51, 51), 2.));
            painter->setBrush(Qt::NoBrush);
            const qreal y0 = c.y() - 4.;
            painter->drawLine(QPointF{c.x(), box.top() + 4.}, QPointF{c.x(), y0});
            painter->drawLine(QPointF{c.x() - 10., y0}, QPointF{c.x() + 10., y0});
            painter->drawLine(QPointF{c.x() - 6., y0 + 5.}, QPointF{c.x() + 6., y0 + 5.});
            painter->drawLine(QPointF{c.x() - 3., y0 + 10.}, QPointF{c.x() + 3., y0 + 10.});
        }
    }

}
