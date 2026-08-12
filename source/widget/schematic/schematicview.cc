#include "./schematicview.h"

#include "qglobal.h"
#include "qnamespace.h"
#include "widget/schematic/schematicscene.h"
#include <QPainter>
#include <QBrush>
#include <cassert>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>
#include <QDebug>
#include <QWheelEvent>
#include <QScrollBar>
#include <cmath>

namespace PR_tool::widget {

    using namespace schematic;

    SchematicView::SchematicView(
        hardware::Interposer* interposer, 
        circuit::BaseDie* basedie,
        QWidget *parent
    ) :
        GraphicsView{parent},
        _interposer{interposer},
        _basedie{basedie}
    {
        this->setBackColor(Qt::white);

        this->setDragMode(QGraphicsView::RubberBandDrag);
        this->setInteractive(true);
        this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    }

    SchematicView::~SchematicView() noexcept {}

    void SchematicView::wheelEvent(QWheelEvent* event) {
        GraphicsView::wheelEvent(event);
        // Ch.九: bundle collapse/expand follows zoom immediately.
        if (event->modifiers() & Qt::ControlModifier) {
            if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
                sc->refreshBusBundling();
            }
        }
    }

    void SchematicView::drawBackground(QPainter* painter, const QRectF& rect) {
        QGraphicsView::drawBackground(painter, rect);

        if (this->gridVisible()) {
            auto gridPen = QPen{this->gridColor()};
            gridPen.setWidth(1);
            painter->setPen(gridPen);

            qreal left = std::floor(rect.left() / this->gridSize()) * this->gridSize();
            qreal top = std::floor(rect.top() / this->gridSize()) * this->gridSize();

            for (qreal x = left; x < rect.right(); x += this->gridSize()) {
                painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
            }
            for (qreal y = top; y < rect.bottom(); y += this->gridSize()) {
                painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
            }
        }
    }

    void SchematicView::updateBack() {
        // QGraphicsView background/grid is painted on the viewport;
        // QWidget::update() on the view itself is unreliable here.
        this->viewport()->update();
    }
}
