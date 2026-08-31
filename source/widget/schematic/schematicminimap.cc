#include "./schematicminimap.h"
#include "./schematicview.h"
#include "./schematicscene.h"
#include "./item/topdieinstitem.h"

#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include <QScrollBar>
#include <QPen>
#include <QColor>
#include <cmath>

namespace PR_tool::widget {

    namespace {

        constexpr int kMiniMapWidth = 168;
        constexpr int kMiniMapHeight = 120;
        constexpr int kInnerPad = 6;
        constexpr qreal kWorldPadFrac = 0.08;

        const QColor kPanelBg {255, 255, 255, 235};
        const QColor kPanelBorder {0x88, 0x88, 0x88};
        const QColor kViewportFill {30, 100, 220, 45};
        const QColor kViewportPen {30, 90, 200};

    } // namespace

    auto SchematicMiniMap::MapXform::toMap(const QPointF& scenePt) const -> QPointF {
        return QPointF {
            this->origin.x() + (scenePt.x() - this->world.left()) * this->scale,
            this->origin.y() + (scenePt.y() - this->world.top()) * this->scale
        };
    }

    auto SchematicMiniMap::MapXform::toScene(const QPointF& mapPt) const -> QPointF {
        if (this->scale <= 0.) {
            return this->world.center();
        }
        return QPointF {
            this->world.left() + (mapPt.x() - this->origin.x()) / this->scale,
            this->world.top() + (mapPt.y() - this->origin.y()) / this->scale
        };
    }

    auto SchematicMiniMap::MapXform::toMapRect(const QRectF& sceneRect) const -> QRectF {
        return QRectF {this->toMap(sceneRect.topLeft()), this->toMap(sceneRect.bottomRight())}
            .normalized();
    }

    SchematicMiniMap::SchematicMiniMap(SchematicView* view) :
        QWidget{view},
        _view{view}
    {
        this->setFixedSize(kMiniMapWidth, kMiniMapHeight);
        this->setFocusPolicy(Qt::NoFocus);
        this->setCursor(Qt::PointingHandCursor);
        this->setAttribute(Qt::WA_NoMouseReplay);

        if (this->_view == nullptr) {
            return;
        }

        auto* hBar = this->_view->horizontalScrollBar();
        auto* vBar = this->_view->verticalScrollBar();
        if (hBar != nullptr) {
            QObject::connect(hBar, &QScrollBar::valueChanged, this, [this](int) { this->update(); });
            QObject::connect(hBar, &QScrollBar::rangeChanged, this, [this](int, int) { this->update(); });
        }
        if (vBar != nullptr) {
            QObject::connect(vBar, &QScrollBar::valueChanged, this, [this](int) { this->update(); });
            QObject::connect(vBar, &QScrollBar::rangeChanged, this, [this](int, int) { this->update(); });
        }
    }

    void SchematicMiniMap::bindScene() {
        if (this->_view == nullptr) {
            return;
        }
        auto* sc = this->_view->scene();
        if (sc == this->_boundScene) {
            return;
        }
        if (this->_boundScene != nullptr) {
            QObject::disconnect(this->_boundScene, nullptr, this, nullptr);
        }
        this->_boundScene = sc;
        if (sc != nullptr) {
            QObject::connect(sc, &QGraphicsScene::changed, this, [this](const QList<QRectF>&) {
                this->update();
            });
        }
        this->update();
    }

    auto SchematicMiniMap::currentXform() const -> MapXform {
        MapXform xf;
        if (this->_view == nullptr) {
            return xf;
        }

        auto* scene = dynamic_cast<SchematicScene*>(this->_view->scene());
        QRectF world;
        if (scene != nullptr) {
            for (auto* inst : scene->topdieinstMap().values()) {
                if (inst == nullptr) {
                    continue;
                }
                world |= inst->sceneBoundingRect();
            }
        }
        if (!world.isValid() || world.isEmpty()) {
            world = this->_view->sceneRect();
        }
        if (!world.isValid() || world.width() <= 0. || world.height() <= 0.) {
            world = QRectF {0., 0., 1., 1.};
        }

        const qreal pad = kWorldPadFrac * std::max(world.width(), world.height());
        world.adjust(-pad, -pad, pad, pad);
        xf.world = world;

        const QRectF inner = this->rect().adjusted(kInnerPad, kInnerPad, -kInnerPad, -kInnerPad);
        const qreal sx = inner.width() / world.width();
        const qreal sy = inner.height() / world.height();
        xf.scale = std::min(sx, sy);
        const QSizeF mapped {world.width() * xf.scale, world.height() * xf.scale};
        xf.origin = QPointF {
            inner.x() + (inner.width() - mapped.width()) / 2.,
            inner.y() + (inner.height() - mapped.height()) / 2.
        };
        return xf;
    }

    void SchematicMiniMap::panTo(const QPoint& widgetPos) {
        if (this->_view == nullptr) {
            return;
        }
        const auto xf = this->currentXform();
        this->_view->centerOn(xf.toScene(QPointF {widgetPos}));
        this->update();
    }

    void SchematicMiniMap::paintEvent(QPaintEvent*) {
        QPainter painter {this};
        painter.setRenderHint(QPainter::Antialiasing, false);

        painter.fillRect(this->rect(), kPanelBg);
        painter.setPen(QPen {kPanelBorder, 1});
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(this->rect().adjusted(0, 0, -1, -1));

        if (this->_view == nullptr) {
            return;
        }

        const auto xf = this->currentXform();
        auto* scene = dynamic_cast<SchematicScene*>(this->_view->scene());
        if (scene != nullptr) {
            painter.setPen(QPen {QColor {0x50, 0x50, 0x50}, 1});
            for (auto* inst : scene->topdieinstMap().values()) {
                if (inst == nullptr) {
                    continue;
                }
                const QColor fill = schematic::TopDieInstanceItem::colorForTopDieType(
                    inst->typeName().toStdString()
                );
                painter.setBrush(fill);
                painter.drawRect(xf.toMapRect(inst->sceneBoundingRect()));
            }
        }

        QRectF viewScene = this->_view->mapToScene(this->_view->viewport()->rect()).boundingRect();
        QRectF viewMap = xf.toMapRect(viewScene);
        if (viewMap.width() < 4.) {
            viewMap.setWidth(4.);
        }
        if (viewMap.height() < 4.) {
            viewMap.setHeight(4.);
        }
        painter.setPen(QPen {kViewportPen, 2});
        painter.setBrush(kViewportFill);
        painter.drawRect(viewMap);
    }

    void SchematicMiniMap::mousePressEvent(QMouseEvent* event) {
        if (event->button() == Qt::LeftButton) {
            this->_dragging = true;
            this->grabMouse();
            this->panTo(event->pos());
            event->accept();
            return;
        }
        QWidget::mousePressEvent(event);
    }

    void SchematicMiniMap::mouseMoveEvent(QMouseEvent* event) {
        if (this->_dragging) {
            this->panTo(event->pos());
            event->accept();
            return;
        }
        QWidget::mouseMoveEvent(event);
    }

    void SchematicMiniMap::mouseReleaseEvent(QMouseEvent* event) {
        if (event->button() == Qt::LeftButton && this->_dragging) {
            this->_dragging = false;
            this->releaseMouse();
            event->accept();
            return;
        }
        QWidget::mouseReleaseEvent(event);
    }

}
