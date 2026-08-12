#include "./netitem.h"
#include "./netpointitem.h"
#include "./pinitem.h"
#include "debug/debug.hh"
#include "qglobal.h"
#include "qnamespace.h"
#include "qobject.h"
#include "qpoint.h"
#include "widget/schematic/info/netinfowidget.h"
#include "widget/schematic/schematicscene.h"
#include <circuit/connection/connection.hh>

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsScene>

#include <QDebug>

namespace PR_tool::widget::schematic {

    const QColor NetItem::DEFAULT_COLOR = Qt::black;
    const QColor NetItem::HOVER_COLOR = Qt::red;

    NetItem::NetItem(circuit::Connection* connection, NetPointItem* beginPoint, NetPointItem* endPoint):
        QGraphicsItem{},
        _connection{connection},
        _beginPoint{beginPoint},
        _endPoint{endPoint}
    {
        this->_beginPoint->setNetItem(this);
        this->_endPoint->setNetItem(this);
        this->setAcceptHoverEvents(true);
        // Above topdie bodies so hover works on segments that cross instances.
        this->setZValue(0.5);

        auto begin = beginPoint->scenePos();
        auto end = endPoint->scenePos();

        this->_path.moveTo(begin);
        this->_points.push_back(begin);

        if (begin.x() == end.x() || begin.y() == end.y()) {
            this->_path.lineTo(end);

            this->_points.push_back(end);
        } else {
            QPointF mid(end.x(), begin.y());
            this->_path.lineTo(mid);
            this->_path.lineTo(end);

            this->_points.push_back(mid);
            this->_points.push_back(end);
        }
    }

    NetItem::NetItem(NetPointItem* beginPoint) :
        QGraphicsItem{},
        _connection{nullptr},
        _beginPoint{beginPoint},
        _endPoint{nullptr}
    {
        this->_paintColor = HOVER_COLOR;
        this->_paintWidth = DEFAULT_WIDTH + 1;
        this->_paintOpacity = 1.0;
        this->_paintStyle = Qt::DashLine;

        this->_color = DEFAULT_COLOR;
        this->_width = DEFAULT_WIDTH;
        
        this->_beginPoint->setNetItem(this);
        this->setAcceptHoverEvents(true);
        this->setZValue(0.5);
        this->setLine(this->_beginPoint->scenePos(), this->_beginPoint->scenePos());
    }

    void NetItem::updateEndPoint(const QPointF& point) {
        if (this->_tempPoint.has_value()) {
            auto tempPoint = *this->_tempPoint;
            auto lastPoint = this->_points.back();
            auto isHoverLine = (lastPoint.y() == tempPoint.y());

            if (tempPoint == point) {
                this->_tempPoint.reset();
            }
            else if (tempPoint.x() != point.x() && tempPoint.y() != point.y()) {
                if (isHoverLine) {
                    this->_tempPoint = QPointF{point.x(), tempPoint.y()};
                } else {
                    this->_tempPoint = QPointF{tempPoint.x(), point.y()};
                }
            }
        }
        else {
            auto lastPoint = this->_points.back();
            if (lastPoint == point) {
                
            }
            else if (lastPoint.x() == point.x() || lastPoint.y() == point.y()) {

            } else {
                if (lastPoint != this->_end) {
                    this->_tempPoint = this->_end;
                }
            }
        }

        this->_end = point;
        this->updatePath();
    }

    void NetItem::addPoint(const QPointF& point) {
        if (this->_tempPoint.has_value()) {
            this->_points.push_back(*this->_tempPoint);
            this->_tempPoint.reset();
        }

        if (this->_points.size() >= 2) {
            auto end = this->_points.rbegin();
            auto lastEnd = end + 1;

            if (end->y() == lastEnd->y() && point.y() == end->y() ||
                end->x() == lastEnd->x() && point.x() == end->x()) {
                this->_points.pop_back();
            }
        }
        this->_points.push_back(point);
    }

    void NetItem::updateLine() {
        if (this->_beginPoint == nullptr || this->_endPoint == nullptr) {
            return;
        }
        this->setLine(this->_beginPoint->scenePos(), this->_endPoint->scenePos());
    }

    void NetItem::updatePositionFrom(NetPointItem* pointItem, const QPointF& pos) {
        if (this->_beginPoint == nullptr || this->_endPoint == nullptr) {
            return;
        }

        if (pointItem == this->_beginPoint) {
            this->updateBeginPosition(pos);
            this->updateBeginPin(pointItem->connectedPin());
        } else if (pointItem == this->_endPoint) {
            this->updateEndPosition(pos);
            this->updateEndPin(pointItem->connectedPin());
        } else {
            debug::unreachable("NetItem::updatePositionFrom");
        }
    }

    void NetItem::updateBeginPosition(const QPointF& pos) {
        this->prepareGeometryChange();

        if (this->_points.size() == 2) {
            auto beginIter = this->_points.rbegin();
            auto endIter = beginIter + 1;

            auto beginPos = *beginIter;
            auto endPos = *endIter;
            
            if (beginPos.x() == endPos.x()) {
                if (pos == endPos) {

                }
                else if (pos.x() == endPos.x()) {
                    *endIter = pos;
                }
                else if (pos.y() == endPos.y()) {
                    if (endPos.y() == beginPos.y()) {
                        this->_points.front() = pos;
                    } else {
                        this->_points.push_front(pos);
                    }
                }
            }
            else if (beginPos.y() == endPos.y()) {
                if (pos == endPos) {

                }
                else if (pos.x() == endPos.x()) {
                    if (endPos.x() == beginPos.x()) {
                        this->_points.front() = pos;
                    } else {
                        this->_points.push_front(pos);
                    }
                }
                else if (pos.y() == endPos.y()) {
                    *endIter = pos;
                }
            } 
            else {
                debug::exception("The netitem has unparall points");
            }
        }
        else {
            auto endIter = this->_points.begin();
            auto lastIter = endIter + 1;

            auto endPos = *endIter;
            auto lastPos = *lastIter;
            auto isHoverLine = endPos.y() == lastPos.y();

            if (endPos == pos) {

            } 
            else if (isHoverLine) {
                // if (pos == lastPos) {
                //     this->_points.pop_back();
                // }
                // else 
                if (pos.y() == endPos.y()) {
                    *endIter = pos;
                } 
                else {
                    *endIter = pos;
                    lastIter->setY(pos.y());
                }
            }
            else {
                // if (pos == lastPos) {
                //     this->_points.pop_back();
                // }
                // else 
                if (pos.x() == endPos.x()) {
                    *endIter = pos;
                } 
                else {
                    *endIter = pos;
                    lastIter->setX(pos.x());
                }
            }
        }

        this->_path = QPainterPath{};
        if (!this->_points.isEmpty()) {
            this->_path.moveTo(this->_points[0]);
            for (int i = 1; i < this->_points.size(); ++i) {
                this->_path.lineTo(this->_points[i]);
            }
        }

        this->update();
    }

    void NetItem::updateEndPosition(const QPointF& pos) {
        this->prepareGeometryChange();

        if (this->_points.size() == 2) {
            auto beginPos = this->_points[0];
            auto endPos = this->_points[1];
            
            if (beginPos.x() == endPos.x()) {
                if (pos == endPos) {

                }
                else if (pos.x() == endPos.x()) {
                    this->_points[1] = pos;
                }
                else if (pos.y() == endPos.y()) {
                    if (endPos.y() == beginPos.y()) {
                        this->_points.back() = pos;
                    } else {
                        this->_points.push_back(pos);
                    }
                }
            }
            else if (beginPos.y() == endPos.y()) {
                if (pos == endPos) {

                }
                else if (pos.x() == endPos.x()) {
                    if (endPos.x() == beginPos.x()) {
                        this->_points.back() = pos;
                    } else {
                        this->_points.push_back(pos);
                    }
                }
                else if (pos.y() == endPos.y()) {
                    this->_points[1] = pos;
                }
            } 
            else {
                debug::exception("The netitem has unparall points");
            }
        }
        else {
            auto endIter = this->_points.rbegin();
            auto lastIter = endIter + 1;

            auto endPos = *endIter;
            auto lastPos = *lastIter;
            auto isHoverLine = endPos.y() == lastPos.y();

            if (endPos == pos) {

            } 
            else if (isHoverLine) {
                // if (pos == lastPos) {
                //     this->_points.pop_back();
                // }
                // else 
                if (pos.y() == endPos.y()) {
                    *endIter = pos;
                } 
                else {
                    *endIter = pos;
                    lastIter->setY(pos.y());
                }
            }
            else {
                // if (pos == lastPos) {
                //     this->_points.pop_back();
                // }
                // else 
                if (pos.x() == endPos.x()) {
                    *endIter = pos;
                } 
                else {
                    *endIter = pos;
                    lastIter->setX(pos.x());
                }
            }
        }

        this->_path = QPainterPath{};
        if (!this->_points.isEmpty()) {
            this->_path.moveTo(this->_points[0]);
            for (int i = 1; i < this->_points.size(); ++i) {
                this->_path.lineTo(this->_points[i]);
            }
        }

        this->update();
    }

    void NetItem::updateBeginPin(PinItem* pin) {
        this->_connection->set_input(pin->toCircuitPin());
    }

    void NetItem::updateEndPin(PinItem* pin) {
        this->_connection->set_output(pin->toCircuitPin());
    }

    void NetItem::setLine(const QPointF& begin, const QPointF& end) {
        this->prepareGeometryChange();

        this->_path.clear();
        this->_path.moveTo(begin);
        if (begin.x() == end.x() || begin.y() == end.y()) {
            this->_path.lineTo(end); // 单一方向
        } else {
            QPointF mid(end.x(), begin.y());
            this->_path.lineTo(mid);  // 水平转折点
            this->_path.lineTo(end);
        }

        this->_points.push_back(begin);
        this->_end = end;

        this->update();
    }

    void NetItem::setRoutePoints(const QVector<QPointF>& points) {
        if (points.size() < 2) {
            return;
        }

        this->prepareGeometryChange();
        this->_points = points;
        this->_end = points.back();
        this->_tempPoint.reset();

        this->_path = QPainterPath{};
        this->_path.moveTo(points.front());
        for (int i = 1; i < points.size(); ++i) {
            this->_path.lineTo(points[i]);
        }
        this->update();
    }
    
    void NetItem::updatePath() {
        this->prepareGeometryChange();

        this->_path = QPainterPath{};
        if (!this->_points.isEmpty()) {
            this->_path.moveTo(this->_points[0]);
            for (int i = 1; i < this->_points.size(); ++i) {
                this->_path.lineTo(this->_points[i]);
            }
        }
        if (this->_tempPoint.has_value()) {
            this->_path.lineTo(*this->_tempPoint);
        }
        this->_path.lineTo(this->_end);

        this->update();
    }

    auto NetItem::boundingRect() const -> QRectF {
        return this->_path.boundingRect().adjusted(-4, -4, 4, 4);
    }

    auto NetItem::shape() const -> QPainterPath {
        QPainterPathStroker stroker;
        stroker.setWidth(4);
        return stroker.createStroke(_path);
    }
    
    void NetItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
        Q_UNUSED(option);
        Q_UNUSED(widget);
        QColor color = this->_paintColor;
        color.setAlphaF(this->_paintOpacity);
        QPen pen(color, this->_paintWidth, this->_paintStyle);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        painter->setPen(pen);
        painter->drawPath(this->_path);
    }

    void NetItem::applyFocusRole(NetFocusRole role) {
        if (this->isFloating()) {
            return;
        }

        this->_paintColor = this->_color;
        this->_paintStyle = Qt::SolidLine;

        switch (role) {
            case NetFocusRole::Default:
                this->_paintWidth = this->_bundleMember
                    ? ConnectionFocusStyle::BUNDLE_WIDTH
                    : ConnectionFocusStyle::DEFAULT_WIDTH;
                this->_paintOpacity = ConnectionFocusStyle::DEFAULT_OPACITY;
                this->setZValue(0.5);
                break;
            case NetFocusRole::DieRelated:
                this->_paintWidth = ConnectionFocusStyle::DIE_RELATED_WIDTH;
                this->_paintOpacity = ConnectionFocusStyle::DIE_RELATED_OPACITY;
                this->setZValue(0.8);
                break;
            case NetFocusRole::DieUnrelated:
                this->_paintWidth = ConnectionFocusStyle::DEFAULT_WIDTH;
                this->_paintOpacity = ConnectionFocusStyle::DIE_UNRELATED_OPACITY;
                this->setZValue(0.4);
                break;
            case NetFocusRole::NetFocused:
                this->_paintWidth = ConnectionFocusStyle::NET_FOCUS_WIDTH;
                this->_paintOpacity = ConnectionFocusStyle::NET_FOCUS_OPACITY;
                this->setZValue(1.0);
                break;
            case NetFocusRole::NetUnrelated:
                this->_paintWidth = ConnectionFocusStyle::DEFAULT_WIDTH;
                this->_paintOpacity = ConnectionFocusStyle::NET_UNRELATED_OPACITY;
                this->setZValue(0.3);
                break;
        }
        this->_width = this->_paintWidth;
        this->update();
    }

    void NetItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
        if (!this->isFloating()) {
            if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
                sc->setHoverNet(this);
            }
        }
        QGraphicsItem::hoverEnterEvent(event);
    }

    void NetItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
        if (!this->isFloating()) {
            if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
                sc->setHoverNet(nullptr);
            }
        }
        QGraphicsItem::hoverLeaveEvent(event);
    }

    void NetItem::resetPaint() {
        this->prepareGeometryChange();
        this->_paintColor = this->_color;
        this->_paintWidth = this->_width;
        this->_paintOpacity = ConnectionFocusStyle::DEFAULT_OPACITY;
        this->_paintStyle = Qt::SolidLine;
        this->update();
    }

    auto NetItem::isFloating() const -> bool {
        return this->_beginPoint == nullptr || this->_endPoint == nullptr;
    }

}