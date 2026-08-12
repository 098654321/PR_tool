#include "./exportitem.h"
#include <circuit/export/export.hh>
#include "qobject.h"
#include "widget/schematic/item/griditem.h"
#include "widget/schematic/item/pinitem.h"

namespace PR_tool::widget::schematic {
    
    const QColor ExternalPortItem::COLOR = QColor::fromRgb(200, 200, 200, 100);

    ExternalPortItem::ExternalPortItem(circuit::ExternalPort* eport) : 
        GridItem{},
        _externalPort{eport},
        _pin{nullptr},
        _width{0}
    {
        auto name = QString::fromStdString(eport->name());
        this->_pin = new PinItem{name, QPointF{0, 0}, PinSide::Left, this};
        this->_anchorSide = PinSide::Left;
        this->_width = GridItem::snapToGrid(PIN_SIDE_INTERVAL + PinItem::NAME_INTERVAL + name.size() * PinItem::CHAR_WIDTH_ + PIN_SIDE_INTERVAL);

        // Physical Track coord is set in the property panel only (canvas drag would desync Layout).
        this->setToolTip(QStringLiteral("Set physical coord in the property panel"));
        this->setFlags(this->flags() | QGraphicsItem::ItemIsSelectable);
        this->setZValue(0);
    }

    void ExternalPortItem::setAnchorSide(PinSide side) {
        if (side != PinSide::Left && side != PinSide::Right) {
            side = PinSide::Left;
        }
        this->prepareGeometryChange();
        this->_anchorSide = side;
        this->_pin->setSide(side);
        this->update();
    }

    void ExternalPortItem::setAggregateBodyHidden(bool hidden) {
        if (this->_aggregateBodyHidden == hidden) {
            return;
        }
        this->_aggregateBodyHidden = hidden;
        this->update();
        if (this->_pin) {
            this->_pin->update();
        }
    }

    auto ExternalPortItem::boundingRect() const -> QRectF {
        if (this->_anchorSide == PinSide::Right) {
            // Body and name sit to the left of the pin origin.
            return QRectF {-this->_width + PIN_SIDE_INTERVAL, -HEIGHT / 2., this->_width, HEIGHT};
        }
        // Body and name sit to the right of the pin origin.
        return QRectF {-PIN_SIDE_INTERVAL, -HEIGHT / 2., this->_width, HEIGHT};
    }

    void ExternalPortItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) {
        // Ch.六: while export Port Group bars are active, skip individual bodies.
        if (this->_aggregateBodyHidden) {
            return;
        }
        painter->setBrush(COLOR);
        painter->drawRect(this->boundingRect());
    }

}
