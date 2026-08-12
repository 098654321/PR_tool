#include "./netpointitem.h"
#include "./netitem.h"
#include "./pinitem.h"
#include "widget/schematic/schematicscene.h"
#include "qnamespace.h"
#include "qobject.h"

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

#include <QDebug>

namespace PR_tool::widget::schematic {

    const QColor NetPointItem::COLOR = Qt::black;
    const QColor NetPointItem::HOVER_COLOR = Qt::red;

    NetPointItem::NetPointItem(PinItem* connectedPin): 
        QGraphicsEllipseItem{nullptr}, 
        _connectedPin{connectedPin}
    {
        this->refreshRect();
        this->setBrush(COLOR);
        this->setPen(Qt::NoPen);
        this->setFlags(this->flags() | QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemSendsScenePositionChanges);
        this->setAcceptHoverEvents(true);
        // Above nets; still below topdie chrome when not focused.
        this->setZValue(0.6);

        this->linkToPin(connectedPin);
    }

    void NetPointItem::refreshRect() {
        qreal r = JUNCTION_RADIUS;
        if (this->_dragging) {
            r = MOVING_RADIUS;
        } else if (this->_focused || this->_hovered) {
            r = JUNCTION_RADIUS * JUNCTION_FOCUS_SCALE;
        }
        this->setRect(-r, -r, 2. * r, 2. * r);
    }

    auto NetPointItem::shouldDrawJunction() const -> bool {
        // Edit affordance always visible while interacting.
        if (this->_dragging || this->_hovered) {
            return true;
        }
        // Floating wire endpoint while drawing.
        if (this->_connectedPin == nullptr) {
            return true;
        }
        // True junction: ≥2 nets electrically share this pin. Mere crossings get no ●.
        return this->_connectedPin->connectedPoints().size() >= 2;
    }

    void NetPointItem::applyNetVisual(qreal opacity, bool focused) {
        this->_opacity = opacity;
        this->_focused = focused;
        this->refreshRect();
        this->update();
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

    void NetPointItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
        Q_UNUSED(option);
        Q_UNUSED(widget);
        if (!this->shouldDrawJunction()) {
            return;
        }

        QColor color = this->_hovered ? HOVER_COLOR : COLOR;
        if (this->_netitem != nullptr) {
            color = this->_hovered ? HOVER_COLOR : this->_netitem->color();
        }
        color.setAlphaF(qBound(0., this->_opacity, 1.));
        painter->setPen(Qt::NoPen);
        painter->setBrush(color);
        const qreal r = this->rect().width() / 2.;
        painter->drawEllipse(QPointF{0., 0.}, r, r);
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
        this->_hovered = true;
        this->refreshRect();
        this->update();
        if (this->_netitem) {
            if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
                sc->setHoverNet(this->_netitem);
            }
        }
        QGraphicsEllipseItem::hoverEnterEvent(event);
    }

    void NetPointItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
        this->_hovered = false;
        this->refreshRect();
        this->update();
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            sc->setHoverNet(nullptr);
        }
        QGraphicsEllipseItem::hoverLeaveEvent(event);
    }

    void NetPointItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
        if (event->button() == Qt::LeftButton) {
            this->_dragging = true;
            this->refreshRect();
            this->update();
        }
        QGraphicsEllipseItem::mousePressEvent(event);
    }

    void NetPointItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
        QGraphicsEllipseItem::mouseMoveEvent(event);
    }

    void NetPointItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
        if (this->_dragging) {
            auto* oldPin = this->_connectedPin;
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
            this->_dragging = false;
            this->refreshRect();
            this->update();
            // Sibling endpoints on the same pin must re-evaluate junction visibility.
            if (oldPin != nullptr) {
                for (auto* sibling : oldPin->connectedPoints()) {
                    sibling->update();
                }
            }
            if (pin != nullptr && pin != oldPin) {
                for (auto* sibling : pin->connectedPoints()) {
                    sibling->update();
                }
            }

            // Explicit write-back: setPos→itemChange may no-op when position is unchanged,
            // leaving Connection still pointing at the old pin after a re-link.
            if (this->_netitem != nullptr && !this->_netitem->isFloating()
                && this->_connectedPin != nullptr) {
                this->_netitem->updatePositionFrom(this, this->pos());
            }
        }

        QGraphicsEllipseItem::mouseReleaseEvent(event);
    }

    void NetPointItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event) {
        event->ignore(); 
    }

}
