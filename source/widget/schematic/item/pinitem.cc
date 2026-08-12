#include "./pinitem.h"
#include "debug/debug.hh"
#include "qcolor.h"
#include "qglobal.h"
#include "qnamespace.h"
#include "qobject.h"
#include "qpoint.h"
#include "./netitem.h"
#include "./netpointitem.h"
#include "./topdieinstitem.h"
#include "./sourceportitem.h"
#include "../schematicscene.h"
#include "./exportitem.h"
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <cassert>

namespace PR_tool::widget::schematic {

    extern SchematicScene* scene;

    const QColor PinItem::COLOR = Qt::black;
    const QColor PinItem::HOVERED_COLOR = Qt::red;

    PinItem::PinItem(
        const QString &name, 
        QPointF position, 
        PinSide side, 
        QGraphicsItem *parent
    )
        : QGraphicsItem(parent), 
        _name{name},
        _side{side}
    {
        this->setPos(position);
        this->setAcceptHoverEvents(true);
        this->setFlags(QGraphicsItem::ItemSendsScenePositionChanges | QGraphicsItem::ItemIsSelectable);
        this->setZValue(0);
    }

    auto PinItem::boundingRect() const -> QRectF {
        const auto r = this->_raduis + HIT_PADDING;
        return QRectF(-r, -r, 2 * r, 2 * r);
    }

    auto PinItem::shape() const -> QPainterPath {
        QPainterPath path;
        const auto r = this->_raduis + HIT_PADDING;
        path.addEllipse(QPointF{0., 0.}, r, r);
        return path;
    }

    auto PinItem::viewScale() const -> qreal {
        if (auto* sc = this->scene()) {
            const auto views = sc->views();
            if (!views.isEmpty()) {
                return views.first()->transform().m11();
            }
        }
        return 1.0;
    }

    auto PinItem::shouldForceShowPin() const -> bool {
        // Selected / hover / related-net focus always force-show (Ch.五 + Ch.七).
        if (this->isSelected() || this->_hovered || this->_focusRelated) {
            return true;
        }
        // Ch.六: click-expand to group size 1 at Medium zoom reveals pins.
        if (this->isTopDieInstancePin()) {
            if (auto* top = this->parentTopDieInstance()) {
                if (top->shouldRevealPins()) {
                    return true;
                }
            }
        }
        return false;
    }

    void PinItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *) {
        // LOD (Ch.五): hide per-pin mark+name unless Near or force-show.
        // boundingRect/shape keep HIT_PADDING so invisible pins remain clickable; anchors stay.
        const qreal s = this->viewScale();
        if (!this->shouldForceShowPin()) {
            if (s < LOD_FAR_MAX) {
                // Far: do not draw pin / group.
                return;
            }
            if (s < LOD_NEAR_MIN) {
                // Medium: Ch.六 draws Port Group bars here — no per-pin ticks/names in Ch.五.
                return;
            }
        }

        if (this->_hovered) {
            painter->setPen(QPen(HOVERED_COLOR, 2, Qt::DashLine));
            painter->setBrush(HOVERED_COLOR);
        } else {
            painter->setPen(Qt::NoPen);
            painter->setBrush(COLOR);
        }
        painter->drawEllipse(QPointF{0., 0.}, this->_raduis, this->_raduis);

        auto length = this->_name.size() * CHAR_WIDTH_;
        painter->setPen(Qt::blue);
        switch (this->_side) {
            case PinSide::Top: {
                auto textRect = QRectF {
                    -CHAR_HEIGHT / 2., 
                    NAME_INTERVAL, 
                    CHAR_HEIGHT,
                    length
                };

                painter->save();
                painter->translate(textRect.center());
                painter->rotate(90);
                painter->drawText(QRectF(-textRect.height() / 2, -textRect.width() / 2, textRect.height(), textRect.width()), 
                                  Qt::AlignLeft, this->_name);
                painter->restore();
                break;
            }
            case PinSide::Bottom: {
                auto textRect = QRectF {
                    - CHAR_HEIGHT / 2, 
                    - length - NAME_INTERVAL, 
                    CHAR_HEIGHT,
                    length
                };
                
                painter->save();
                painter->translate(textRect.center());
                painter->rotate(-90);
                painter->drawText(QRectF(-textRect.height() / 2, -textRect.width() / 2, textRect.height(), textRect.width()), 
                                  Qt::AlignLeft, this->_name);
                painter->restore();
                break;
            }
            case PinSide::Left: {
                auto textRect = QRectF {
                    NAME_INTERVAL, 
                    - CHAR_HEIGHT / 2., 
                    length, 
                    CHAR_HEIGHT
                };
                painter->drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter, this->_name);
                break;
            }
            case PinSide::Right: {
                auto textRect = QRectF {
                    - length - NAME_INTERVAL, 
                    - CHAR_HEIGHT / 2., 
                    length, 
                    CHAR_HEIGHT
                };
                painter->drawText(textRect, Qt::AlignRight | Qt::AlignVCenter, this->_name);
                break;
            }
        }
    }

    auto PinItem::isExternalPortPin() const -> bool {
        return this->parentItem()->type() == ExternalPortItem::Type;
    }

    auto PinItem::isTopDieInstancePin() const -> bool {
        return this->parentItem()->type() == TopDieInstanceItem::Type;
    }

    auto PinItem::isSourcePortPin() const -> bool {
        return this->parentItem()->type() == SourcePortItem::Type;
    }

    auto PinItem::toString() const -> QString {
        if (this->isTopDieInstancePin()) {
            return QString{"%1.%2"}.arg(this->parentTopDieInstance()->name()).arg(this->_name);
        }
        return this->_name;
    }

    auto PinItem::toCircuitPin() const -> circuit::Pin {
        assert(this->parentItem() != nullptr);

        if (this->isExternalPortPin()) {
            return circuit::Pin::connect_export(
                this->parentExternalPort()->unwrap()
            );
        }
        else if (this->isTopDieInstancePin()) {
            return circuit::Pin::connect_bump(
                this->parentTopDieInstance()->unwrap(),
                this->name().toStdString()
            );
        }
        else if (this->isSourcePortPin()) {
            auto port = this->parentSourcePort();
            if (port->isVDD()) {
                return circuit::Pin::connect_vdd(this->name().toStdString());
            } else {
                return circuit::Pin::connect_gnd(this->name().toStdString());
            }
        }

        debug::unreachable("PinItem::toCircuitPin()");
    }

    auto PinItem::itemChange(GraphicsItemChange change, const QVariant& value) -> QVariant {
        if (change == QGraphicsItem::ItemScenePositionHasChanged) {
            for (auto point : this->_connectedNetPoints) {
                point->updatePos();
            }
        }
        return QGraphicsItem::itemChange(change, value);
    }

    void PinItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
        dynamic_cast<SchematicScene*>(this->scene())->headleCreateNet(this, event);
        QGraphicsItem::mousePressEvent(event);
    }

    void PinItem::setHovered(bool hovered) {
        if (this->_hovered == hovered) {
            return;
        }
        this->_hovered = hovered;
        this->update();
    }

    void PinItem::setFocusRelated(bool related) {
        if (this->_focusRelated == related) {
            return;
        }
        this->_focusRelated = related;
        this->update();
    }

    void PinItem::hoverEnterEvent(QGraphicsSceneHoverEvent * event) {
        this->setHovered(true);
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            if (this->isTopDieInstancePin()) {
                sc->setHoverTopDie(this->parentTopDieInstance());
            }
            sc->setHoverPin(this);
        }
        QGraphicsItem::hoverEnterEvent(event);
    }

    void PinItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
        this->setHovered(false);
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            sc->setHoverPin(nullptr);
            if (this->isTopDieInstancePin()) {
                sc->setHoverTopDie(nullptr);
            }
        }
        QGraphicsItem::hoverLeaveEvent(event);
    }

    auto PinItem::parentExternalPort() const -> ExternalPortItem* {
        assert(this->parentItem() != nullptr);
        return dynamic_cast<ExternalPortItem*>(this->parentItem());
    }

    auto PinItem::parentTopDieInstance() const -> TopDieInstanceItem* {
        assert(this->parentItem() != nullptr);
        return dynamic_cast<TopDieInstanceItem*>(this->parentItem());   
    }

    auto PinItem::parentSourcePort() const -> SourcePortItem* {
        assert(this->parentItem() != nullptr);
        return dynamic_cast<SourcePortItem*>(this->parentItem());     
    }

}