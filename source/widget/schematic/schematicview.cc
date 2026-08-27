#include "./schematicview.h"
#include "./schematicminimap.h"
#include "./schematictypography.h"
#include "../chrometokens.h"

#include "qglobal.h"
#include "qnamespace.h"
#include "widget/schematic/item/exportitem.h"
#include "widget/schematic/item/griditem.h"
#include "widget/schematic/item/netitem.h"
#include "widget/schematic/item/netpointitem.h"
#include "widget/schematic/item/pinitem.h"
#include "widget/schematic/item/portgroupitem.h"
#include "widget/schematic/item/sourceportitem.h"
#include "widget/schematic/item/topdieinstitem.h"
#include "widget/schematic/schematicscene.h"
#include <QPainter>
#include <QBrush>
#include <QPen>
#include <QPalette>
#include <QLabel>
#include <cassert>
#include <hardware/interposer.hh>
#include <circuit/basedie.hh>
#include <QDebug>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QScrollBar>
#include <QResizeEvent>
#include <QEvent>
#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QObject>
#include <cmath>

namespace PR_tool::widget {

    using namespace schematic;

    namespace {

        // Ch.十: major = 5 × minor (minor aligns with GridItem::GRID_SIZE / snap).
        constexpr qreal kGridMajorFactor = 5.;
        const QColor kGridMajorColor {0xE6, 0xE6, 0xE6};
        const QColor kGridMinorColor {0xF0, 0xF0, 0xF0};

        auto netDisplayName(schematic::NetItem* net) -> QString {
            if (!net || net->isFloating()) {
                return QStringLiteral("(floating)");
            }
            QString begin = QStringLiteral("?");
            QString end = QStringLiteral("?");
            if (net->beginPoint() && net->beginPoint()->connectedPin()) {
                begin = net->beginPoint()->connectedPin()->toString();
            }
            if (net->endPoint() && net->endPoint()->connectedPin()) {
                end = net->endPoint()->connectedPin()->toString();
            }
            return QStringLiteral("%1 → %2").arg(begin, end);
        }

        auto nameForItem(QGraphicsItem* item) -> QString {
            if (!item) {
                return {};
            }
            switch (item->type()) {
            case schematic::TopDieInstanceItem::Type: {
                auto* die = static_cast<schematic::TopDieInstanceItem*>(item);
                if (!die->typeName().isEmpty()) {
                    return QStringLiteral("%1 (%2)").arg(die->name(), die->typeName());
                }
                return die->name();
            }
            case schematic::ExternalPortItem::Type:
                return static_cast<schematic::ExternalPortItem*>(item)->name();
            case schematic::SourcePortItem::Type:
                return static_cast<schematic::SourcePortItem*>(item)->name();
            case schematic::NetItem::Type:
                return netDisplayName(static_cast<schematic::NetItem*>(item));
            case schematic::NetPointItem::Type:
                return netDisplayName(static_cast<schematic::NetPointItem*>(item)->netItem());
            case schematic::PinItem::Type:
                return static_cast<schematic::PinItem*>(item)->toString();
            case schematic::PortGroupItem::Type: {
                auto* group = static_cast<schematic::PortGroupItem*>(item);
                const QString range = QStringLiteral("%1[%2:%3]")
                    .arg(group->isExportGroup() ? QStringLiteral("exports") : QStringLiteral("bumps"))
                    .arg(group->startIndex())
                    .arg(group->endIndex());
                if (!group->isExportGroup() && group->ownerTopDie()) {
                    return QStringLiteral("%1 %2").arg(group->ownerTopDie()->name(), range);
                }
                return range;
            }
            default:
                return {};
            }
        }

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
        this->setObjectName(QStringLiteral("CanvasWorkSurface"));
        this->setAttribute(Qt::WA_StyledBackground, true);
        this->setAutoFillBackground(true);
        this->setFrameShape(QFrame::NoFrame);
        this->setBackColor(ChromeTokens::color(ChromeTokens::surface));
        {
            QPalette pal = this->palette();
            pal.setColor(QPalette::Window, ChromeTokens::color(ChromeTokens::surface));
            pal.setColor(QPalette::Base, ChromeTokens::color(ChromeTokens::surface));
            this->setPalette(pal);
        }
        this->setDragMode(QGraphicsView::RubberBandDrag);
        this->setInteractive(true);
        this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        this->setMouseTracking(true);
        if (QWidget* vp = this->viewport()) {
            vp->setAutoFillBackground(true);
            QPalette pal = vp->palette();
            pal.setColor(QPalette::Window, ChromeTokens::color(ChromeTokens::surface));
            pal.setColor(QPalette::Base, ChromeTokens::color(ChromeTokens::surface));
            vp->setPalette(pal);
            vp->setMouseTracking(true);
        }
        this->_gridSize = schematic::GridItem::GRID_SIZE;

        this->_minimap = new SchematicMiniMap {this};
        this->repositionMiniMap();

        this->_emptyHint = new QLabel{
            QStringLiteral("Load a config or place from the Palette"),
            this->viewport()};
        this->_emptyHint->setAttribute(Qt::WA_TransparentForMouseEvents);
        this->_emptyHint->setFocusPolicy(Qt::NoFocus);
        this->_emptyHint->setAlignment(Qt::AlignCenter);
        this->_emptyHint->setWordWrap(false);
        this->_emptyHint->setAutoFillBackground(false);
        SchematicTypography::applyStatus(this->_emptyHint);
        SchematicTypography::applyForeground(this->_emptyHint, ChromeTokens::textMuted);
        this->updateEmptyHint();
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
        if (this->_minimap != nullptr) {
            this->_minimap->update();
        }
        this->emitStatusContext();
    }

    void SchematicView::bindMiniMap() {
        if (this->_minimap != nullptr) {
            this->_minimap->bindScene();
        }
        this->repositionMiniMap();
        if (auto* sc = this->scene()) {
            QObject::connect(
                sc, &QGraphicsScene::selectionChanged,
                this, &SchematicView::emitStatusContext,
                Qt::UniqueConnection);
            QObject::connect(
                sc, &QGraphicsScene::changed,
                this, &SchematicView::updateEmptyHint,
                Qt::UniqueConnection);
        }
        this->updateEmptyHint();
    }

    void SchematicView::applyInitialView() {
        this->resetTransform();
        this->scale(kInitialScale, kInitialScale);
        if (this->scene() != nullptr && !this->items().isEmpty()) {
            const QRectF bounds = this->scene()->itemsBoundingRect();
            if (bounds.isValid() && !bounds.isEmpty()) {
                this->centerOn(bounds.center());
            }
        }
        if (this->_minimap != nullptr) {
            this->_minimap->update();
        }
        this->emitStatusContext();
    }

    void SchematicView::repositionMiniMap() {
        if (this->_minimap == nullptr || this->viewport() == nullptr) {
            return;
        }
        constexpr int kMargin = 8;
        const QSize sz = this->_minimap->size();
        const QRect vr = this->viewport()->geometry();
        const int x = qMax(vr.left(), vr.right() - sz.width() - kMargin);
        const int y = qMax(vr.top(), vr.bottom() - sz.height() - kMargin);
        this->_minimap->move(x, y);
        this->_minimap->raise();
        this->_minimap->show();
    }

    void SchematicView::scrollContentsBy(int dx, int dy) {
        GraphicsView::scrollContentsBy(dx, dy);
        this->repositionMiniMap();
        if (this->_minimap != nullptr) {
            this->_minimap->update();
        }
    }

    void SchematicView::repositionEmptyHint() {
        if (this->_emptyHint == nullptr || this->viewport() == nullptr) {
            return;
        }
        this->_emptyHint->adjustSize();
        const QRect vr = this->viewport()->rect();
        const QSize sz = this->_emptyHint->sizeHint();
        const int x = qMax(0, (vr.width() - sz.width()) / 2);
        const int y = qMax(0, (vr.height() - sz.height()) / 2);
        this->_emptyHint->setGeometry(x, y, sz.width(), sz.height());
    }

    auto SchematicView::schematicCanvasIsEmpty() const -> bool {
        auto* sc = this->scene();
        if (sc == nullptr) {
            return true;
        }
        for (QGraphicsItem* item : sc->items()) {
            if (item == nullptr) {
                continue;
            }
            switch (item->type()) {
            case schematic::TopDieInstanceItem::Type:
            case schematic::ExternalPortItem::Type:
            case schematic::SourcePortItem::Type:
            case schematic::NetItem::Type:
                return false;
            default:
                break;
            }
        }
        return true;
    }

    void SchematicView::updateEmptyHint() {
        if (this->_emptyHint == nullptr) {
            return;
        }
        const bool empty = this->schematicCanvasIsEmpty();
        this->_emptyHint->setVisible(empty);
        if (empty) {
            this->repositionEmptyHint();
            this->_emptyHint->raise();
            if (this->_minimap != nullptr) {
                this->_minimap->raise();
            }
        }
    }

    void SchematicView::resizeEvent(QResizeEvent* event) {
        GraphicsView::resizeEvent(event);
        this->repositionMiniMap();
        this->repositionEmptyHint();
    }

    void SchematicView::mouseMoveEvent(QMouseEvent* event) {
        GraphicsView::mouseMoveEvent(event);
        if (this->_isPanning && this->_minimap != nullptr) {
            this->_minimap->update();
        }

        const QPointF sp = this->mapToScene(event->pos());
        const int x = qRound(sp.x());
        const int y = qRound(sp.y());
        if (!this->_hasStatusPos
            || qRound(this->_statusScenePos.x()) != x
            || qRound(this->_statusScenePos.y()) != y) {
            this->_statusScenePos = sp;
            this->_hasStatusPos = true;
            this->emitStatusContext();
        }
    }

    auto SchematicView::viewportEvent(QEvent* event) -> bool {
        const bool handled = GraphicsView::viewportEvent(event);
        if (event->type() == QEvent::Resize) {
            this->repositionMiniMap();
            this->repositionEmptyHint();
        }
        return handled;
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

    void SchematicView::setGridVisible(bool visible) {
        this->_gridVisible = visible;
        this->emitStatusContext();
    }

    void SchematicView::setGridSize(qreal size) {
        this->_gridSize = size;
        this->emitStatusContext();
    }

    void SchematicView::fitContent() {
        GraphicsView::fitContent();
        this->emitStatusContext();
    }

    void SchematicView::resetZoom() {
        this->applyInitialView();
    }

    void SchematicView::ensureVisibleAtMinScale(QGraphicsItem* item, qreal minScale) {
        GraphicsView::ensureVisibleAtMinScale(item, minScale);
        this->emitStatusContext();
    }

    void SchematicView::emitStatusContext() {
        emit this->statusContextChanged();
    }

    auto SchematicView::statusScenePos() const -> QPointF {
        if (this->_hasStatusPos) {
            return this->_statusScenePos;
        }
        if (this->viewport() == nullptr) {
            return {};
        }
        return this->mapToScene(this->viewport()->rect().center());
    }

    auto SchematicView::selectionField() const -> QString {
        if (this->scene() == nullptr) {
            return QStringLiteral("Ctrl+Wheel: Zoom · Middle Drag: Pan · Esc: Cancel");
        }
        const auto items = this->scene()->selectedItems();
        for (auto* item : items) {
            const QString name = nameForItem(item);
            if (!name.isEmpty()) {
                return QStringLiteral("Selected: %1").arg(name);
            }
        }
        return QStringLiteral("Ctrl+Wheel: Zoom · Middle Drag: Pan · Esc: Cancel");
    }

    auto SchematicView::gridField() const -> QString {
        if (!this->gridVisible()) {
            return QStringLiteral("Grid off");
        }
        const qreal s = this->transform().m11();
        if (s < schematic::PinItem::LOD_FAR_MAX) {
            return QStringLiteral("Grid off");
        }
        const qreal minor = this->gridSize() > 0. ? this->gridSize() : schematic::GridItem::GRID_SIZE;
        const qreal major = kGridMajorFactor * minor;
        if (s >= 1.0) {
            return QStringLiteral("Grid %1/%2").arg(qRound(major)).arg(qRound(minor));
        }
        return QStringLiteral("Grid %1").arg(qRound(major));
    }

    auto SchematicView::zoomField() const -> QString {
        return QStringLiteral("Zoom ") + QString::number(qRound(this->transform().m11() * 100.0))
            + QLatin1Char('%');
    }

    auto SchematicView::statusLine() const -> QString {
        const QPointF p = this->statusScenePos();
        return QStringLiteral("%1 | X: %2 Y: %3 | %4 | %5")
            .arg(
                this->selectionField(),
                QString::number(qRound(p.x())),
                QString::number(qRound(p.y())),
                this->gridField(),
                this->zoomField());
    }
}
