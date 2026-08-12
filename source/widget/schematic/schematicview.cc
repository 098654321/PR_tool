#include "./schematicview.h"

#include "qglobal.h"
#include "qnamespace.h"
#include "widget/schematic/item/griditem.h"
#include "widget/schematic/item/pinitem.h"
#include "widget/schematic/schematicscene.h"
#include <QPainter>
#include <QBrush>
#include <QPen>
#include <cassert>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>
#include <QDebug>
#include <QWheelEvent>
#include <QScrollBar>
#include <cmath>

namespace PR_tool::widget {

    using namespace schematic;

    namespace {

        // Ch.十: major = 5 × minor (minor aligns with GridItem::GRID_SIZE / snap).
        constexpr qreal kGridMajorFactor = 5.;
        const QColor kGridMajorColor {0xE6, 0xE6, 0xE6};
        const QColor kGridMinorColor {0xF0, 0xF0, 0xF0};

        void drawGridLines(
            QPainter* painter,
            const QRectF& rect,
            qreal step,
            const QColor& color
        ) {
            QPen pen{color, 1.};
            pen.setCosmetic(true);
            painter->setPen(pen);

            const qreal left = std::floor(rect.left() / step) * step;
            const qreal top = std::floor(rect.top() / step) * step;

            for (qreal x = left; x < rect.right(); x += step) {
                painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
            }
            for (qreal y = top; y < rect.bottom(); y += step) {
                painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
            }
        }

    } // namespace

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
        this->_gridSize = schematic::GridItem::GRID_SIZE;
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
        // Ch.十: grid LOD depends on s = transform().m11().
        this->viewport()->update();
    }

    void SchematicView::drawBackground(QPainter* painter, const QRectF& rect) {
        QGraphicsView::drawBackground(painter, rect);

        if (!this->gridVisible()) {
            return;
        }

        // Ch.十 LOD (same metric as Pin LOD Ch.五): s = transform().m11().
        const qreal s = this->transform().m11();
        if (s < schematic::PinItem::LOD_FAR_MAX) {
            return; // Far: hide grid
        }

        const qreal minor = this->gridSize() > 0. ? this->gridSize() : schematic::GridItem::GRID_SIZE;
        const qreal major = kGridMajorFactor * minor;

        if (s >= 1.0) {
            // Near+: minor first (lighter), then major on top — still below scene items.
            drawGridLines(painter, rect, minor, kGridMinorColor);
        }
        // Medium + Near+: major
        drawGridLines(painter, rect, major, kGridMajorColor);
    }

    void SchematicView::updateBack() {
        // QGraphicsView background/grid is painted on the viewport;
        // QWidget::update() on the view itself is unreliable here.
        this->viewport()->update();
    }
}
