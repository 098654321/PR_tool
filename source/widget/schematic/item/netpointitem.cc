#include "./netpointitem.h"
#include "./netitem.h"
#include "./pinitem.h"
#include "widget/schematic/schematicscene.h"
#include "qnamespace.h"
#include "qobject.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>

#include <QDebug>

namespace PR_tool::widget::schematic {

    const QColor NetPointItem::COLOR = Qt::blue;
    const QColor NetPointItem::HOVER_COLOR = Qt::red;

    NetPointItem::NetPointItem(PinItem* connectedPin): 
        QGraphicsEllipseItem{nullptr}, 
        _connectedPin{connectedPin}
    {
        this->setRect(-RADIUS, -RADIUS, DIAMETER, DIAMETER);
        this->setBrush(COLOR);
        this->setFlags(this->flags() | QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemSendsScenePositionChanges);
        this->setAcceptHoverEvents(true);

        this->linkToPin(connectedPin);
        this->setZValue(1);
    }

    auto NetPointItem::boundingRect() const -> QRectF {
        const auto r = qMax(this->rect().width(), this->rect().height()) / 2. + HIT_PADDING;
        return QRectF(-r, -r, 2. * r, 2. * r);
    }

    auto NetPointItem::shape() const -> QPainterPath {
        QPainterPath path;
        const auto r = qMax(this->rect().width(), this->rect().height()) / 2. + HIT_PADDING;
        path.addEllipse(QPointF{0., 0.}, r, r);
        return path;
    }
 
    void NetPointItem::linkToPin(PinItem* pin) {
        if (pin != nullptr) {
            this->_connectedPin = pin;
            this->setPos(pin->scenePos());
            pin->addConnectedPoint(this);        
        }
    }

    auto NetPointItem::unlinkPin() -> PinItem* {
        auto pin = this->_connectedPin;
        this->_connectedPin = nullptr;
        pin->removeConnectedPoint(this);
        return pin;
    }

    void NetPointItem::updatePos() {
        this->setPos(this->_connectedPin->scenePos());
    }

    QVariant NetPointItem::itemChange(GraphicsItemChange change, const QVariant& value) {
        auto v = QGraphicsEllipseItem::itemChange(change, value);
        if (change == GraphicsItemChange::ItemPositionChange && this->_netitem != nullptr) {
            if (this->_netitem->isFloating()) {
                return v;
            }
            this->_netitem->updatePositionFrom(this, v.toPointF());
        }
        return v;
    }

    void NetPointItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
        setPen(QPen(HOVER_COLOR, 3));
        if (this->_netitem) {
            if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
                sc->setHoverNet(this->_netitem);
            }
        }
        QGraphicsEllipseItem::hoverEnterEvent(event);
    }

    void NetPointItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
        setPen(QPen(COLOR, 2));
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            sc->setHoverNet(nullptr);
        }
        QGraphicsEllipseItem::hoverLeaveEvent(event);
    }

    void NetPointItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
        if (event->button() == Qt::LeftButton) {
            this->_dragging = true;
            this->setRect(-MOVING_RADIUS, -MOVING_RADIUS, MOVING_DIAMETER, MOVING_DIAMETER);
        }
        QGraphicsEllipseItem::mousePressEvent(event);
    }

    void NetPointItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
        QGraphicsEllipseItem::mouseMoveEvent(event);
    }

    void NetPointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
        if (this->_dragging) {
            auto pin = this->_connectedPin;
            auto items = this->scene()->items(event->scenePos());
            for (auto* item : items) {
                if (item->type() == PinItem::Type) {
                    pin = dynamic_cast<PinItem*>(item);
                    break;
                }
            }

            if (this->_connectedPin != nullptr) {
                this->unlinkPin();
            }
            this->linkToPin(pin);
            this->setRect(-RADIUS, -RADIUS, DIAMETER, DIAMETER);

            // Explicit write-back: setPos→itemChange may no-op when position is unchanged,
            // leaving Connection still pointing at the old pin after a re-link.
            if (this->_netitem != nullptr && !this->_netitem->isFloating()
                && this->_connectedPin != nullptr) {
                this->_netitem->updatePositionFrom(this, this->pos());
            }

            this->_dragging = false;
        }

        QGraphicsEllipseItem::mouseReleaseEvent(event);
    }

    void NetPointItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
        event->ignore(); 
    }

}