#include "./portgroupitem.h"
#include "./topdieinstitem.h"
#include "./exportitem.h"
#include "./netitem.h"
#include "../schematicscene.h"
#include "../schematictypography.h"

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

#include <algorithm>
#include <cmath>

namespace PR_tool::widget::schematic {

    const QColor PortGroupItem::COLOR = QColor(70, 100, 140, 200);
    const QColor PortGroupItem::SELECTED_COLOR = QColor(200, 80, 60, 220);

    auto PortGroupLod::groupSizeForScale(qreal s) -> int {
        if (s >= SCALE_SIZE_1) {
            return 1;
        }
        if (s >= SCALE_SIZE_2) {
            return 2;
        }
        if (s >= SCALE_SIZE_4) {
            return 4;
        }
        if (s >= SCALE_SIZE_8) {
            return 8;
        }
        if (s >= SCALE_SIZE_16) {
            return 16;
        }
        if (s >= SCALE_SIZE_32) {
            return 32;
        }
        if (s >= SCALE_SIZE_64) {
            return 64;
        }
        if (s >= SCALE_SIZE_128) {
            return 128;
        }
        return 0;
    }

    auto PortGroupLod::exportGroupSizeForScale(qreal s, int count) -> int {
        if (count <= 0) {
            return 0;
        }
        if (s < SCALE_SIZE_128) {
            return 0;
        }
        // Approximate binary split: start at full count, halve at each finer threshold.
        int size = count;
        const qreal steps[] = {
            SCALE_SIZE_64, SCALE_SIZE_32, SCALE_SIZE_16, SCALE_SIZE_8,
            SCALE_SIZE_4, SCALE_SIZE_2, SCALE_SIZE_1
        };
        for (qreal t : steps) {
            if (s >= t) {
                size = std::max(1, (size + 1) / 2);
            } else {
                break;
            }
        }
        return size;
    }

    PortGroupItem::PortGroupItem(
        TopDieInstanceItem* owner,
        PinSide side,
        int startIndex,
        int endIndex,
        const QVector<PinItem*>& pins,
        QGraphicsItem* parent
    )
        : QGraphicsItem(parent)
        , _owner{owner}
        , _side{side}
        , _startIndex{startIndex}
        , _endIndex{endIndex}
        , _pins{pins}
        , _exportMode{false}
    {
        this->setAcceptHoverEvents(true);
        this->setFlag(QGraphicsItem::ItemIsSelectable, true);
        this->setZValue(1);
    }

    PortGroupItem::PortGroupItem(
        ExportPortGroupHost* host,
        const QVector<ExternalPortItem*>& exports,
        int startIndex,
        int endIndex,
        QGraphicsItem* parent
    )
        : QGraphicsItem(parent)
        , _exportHost{host}
        , _side{PinSide::Left}
        , _startIndex{startIndex}
        , _endIndex{endIndex}
        , _exports{exports}
        , _exportMode{true}
    {
        this->setAcceptHoverEvents(true);
        this->setFlag(QGraphicsItem::ItemIsSelectable, true);
        this->setZValue(1);
    }

    void PortGroupItem::setBarRect(const QRectF& rect) {
        this->prepareGeometryChange();
        this->_barRect = rect;
    }

    auto PortGroupItem::boundingRect() const -> QRectF {
        return this->_barRect.adjusted(-2., -2., 2., 2.);
    }

    auto PortGroupItem::shape() const -> QPainterPath {
        QPainterPath path;
        path.addRect(this->_barRect);
        return path;
    }

    auto PortGroupItem::labelText() const -> QString {
        if (this->_exportMode) {
            return QStringLiteral("exports[%1:%2]")
                .arg(this->_startIndex)
                .arg(this->_endIndex);
        }
        return QStringLiteral("bumps[%1:%2]")
            .arg(this->_startIndex)
            .arg(this->_endIndex);
    }

    void PortGroupItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) {
        if (this->_barRect.isEmpty()) {
            return;
        }

        const bool chromeOk = this->interactionChromeAllowed();
        ItemChromeState chrome = ItemChromeState::Normal;
        if (chromeOk) {
            if (this->isSelected()) {
                chrome = ItemChromeState::Selected;
            } else if (this->_hovered) {
                chrome = ItemChromeState::Hover;
            }
        }
        const bool emphasis = chrome == ItemChromeState::Selected
            || chrome == ItemChromeState::Hover;
        painter->setPen(QPen(
            emphasis ? itemChromeBorder(chrome) : COLOR.darker(130),
            itemChromeWidth(chrome)));
        painter->setBrush(emphasis ? SELECTED_COLOR : COLOR);
        painter->drawRect(this->_barRect);

        painter->setFont(SchematicTypography::pinNameFont());
        painter->setPen(Qt::white);

        const QString text = this->labelText();
        const QRectF textRect = this->_barRect.adjusted(LABEL_PAD, 0., -LABEL_PAD, 0.);

        const bool vertical = !this->_exportMode
            && (this->_side == PinSide::Left || this->_side == PinSide::Right);
        if (vertical) {
            painter->save();
            painter->translate(textRect.center());
            painter->rotate(this->_side == PinSide::Left ? -90. : 90.);
            const QRectF rotated {
                -textRect.height() / 2.,
                -textRect.width() / 2.,
                textRect.height(),
                textRect.width()
            };
            painter->drawText(rotated, Qt::AlignCenter, text);
            painter->restore();
        } else {
            painter->drawText(textRect, Qt::AlignCenter, text);
        }
    }

    auto PortGroupItem::parentExportContextActive() const -> bool {
        if (!this->scene()) {
            return false;
        }
        for (auto* item : this->scene()->selectedItems()) {
            if (!item) {
                continue;
            }
            if (item->type() == ExternalPortItem::Type) {
                return true;
            }
            if (item->type() == PortGroupItem::Type) {
                auto* group = static_cast<PortGroupItem*>(item);
                if (group->_exportMode) {
                    return true;
                }
            }
            if (item->type() == ExportPortGroupHost::Type) {
                return true;
            }
        }
        return false;
    }

    auto PortGroupItem::interactionChromeAllowed() const -> bool {
        if (this->_exportMode) {
            return this->parentExportContextActive();
        }
        return this->_owner && this->_owner->isSelected();
    }

    void PortGroupItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
        const bool canSelect = this->interactionChromeAllowed();
        this->_hovered = canSelect;
        this->update();

        if (canSelect) {
            this->setSelected(true);
        }

        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            if (this->_owner) {
                // Keep die focus while over its port groups (nested hover leave on die).
                sc->setHoverTopDie(this->_owner);
            }
            // Bundle net-focus when parent context allows (Ch.六 gate) or die already focused.
            if (canSelect || (this->_owner && this->_owner->isSelected())) {
                sc->setHoverPortGroup(this);
            }
        }

        QGraphicsItem::hoverEnterEvent(event);
    }

    void PortGroupItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
        this->_hovered = false;
        this->update();
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            sc->setHoverPortGroup(nullptr);
            if (this->_owner && !sc->hoverMovesWithin(this->_owner, event->scenePos())) {
                sc->setHoverTopDie(nullptr);
            }
        }
        QGraphicsItem::hoverLeaveEvent(event);
    }

    void PortGroupItem::mousePressEvent(QGraphicsSceneMouseEvent* event) {
        if (event->button() & Qt::LeftButton) {
            this->setSelected(true);
            if (this->_owner) {
                this->_owner->setSelected(true);
                this->_owner->expandPortGroups();
            } else if (this->_exportHost) {
                this->_exportHost->setSelected(true);
                this->_exportHost->expandGroups();
            }
            event->accept();
            return;
        }
        QGraphicsItem::mousePressEvent(event);
    }

    // ---------------------------------------------------------------------------
    // ExportPortGroupHost
    // ---------------------------------------------------------------------------

    ExportPortGroupHost::ExportPortGroupHost(QGraphicsItem* parent)
        : QGraphicsItem(parent)
    {
        this->setFlag(QGraphicsItem::ItemIsSelectable, true);
        this->setZValue(0.5);
    }

    void ExportPortGroupHost::setExports(const QVector<ExternalPortItem*>& exports) {
        this->_exports = exports;
        this->_expandGroupSize.reset();
        this->_syncedGroupSize = -1;
        this->syncGroups();
    }

    auto ExportPortGroupHost::viewScale() const -> qreal {
        if (auto* sc = this->scene()) {
            const auto views = sc->views();
            if (!views.isEmpty()) {
                return views.first()->transform().m11();
            }
        }
        return 1.0;
    }

    auto ExportPortGroupHost::effectiveGroupSize() const -> int {
        // Export aggregate bars are unused: keep individual gray export boxes at all zooms.
        return 1;
    }

    void ExportPortGroupHost::expandGroups() {
        const int cur = std::max(1, this->effectiveGroupSize());
        if (cur <= 1) {
            this->_expandGroupSize = 1;
        } else {
            this->_expandGroupSize = std::max(1, cur / 2);
        }
        // Defer rebuild so PortGroupItem::mousePressEvent does not delete `this`.
        this->_syncedGroupSize = -1;
        this->scheduleSync();
    }

    void ExportPortGroupHost::clearGroups() {
        for (auto* g : this->_groups) {
            if (g && g->scene()) {
                g->scene()->removeItem(g);
            }
            delete g;
        }
        this->_groups.clear();
        this->_syncedGroupSize = -1;
    }

    void ExportPortGroupHost::applyExportBodyVisibility(bool hideBodies) {
        for (auto* e : this->_exports) {
            if (e) {
                e->setAggregateBodyHidden(hideBodies);
            }
        }
    }

    void ExportPortGroupHost::rebuildGroups(int groupSize) {
        this->clearGroups();
        this->applyExportBodyVisibility(false);

        if (groupSize <= 1 || this->_exports.isEmpty()) {
            this->_syncedGroupSize = groupSize;
            this->_bounds = QRectF{};
            return;
        }

        // Sort exports by scene Y for stable ranges.
        QVector<ExternalPortItem*> ordered = this->_exports;
        std::sort(ordered.begin(), ordered.end(), [](ExternalPortItem* a, ExternalPortItem* b) {
            return a->scenePos().y() < b->scenePos().y();
        });

        const int n = ordered.size();
        QRectF bounds;
        for (int start = 0; start < n; start += groupSize) {
            const int end = std::min(n - 1, start + groupSize - 1);
            QVector<ExternalPortItem*> slice;
            slice.reserve(end - start + 1);
            QRectF unionRect;
            for (int i = start; i <= end; ++i) {
                slice.push_back(ordered[i]);
                const QRectF r = ordered[i]->sceneBoundingRect();
                unionRect = unionRect.isNull() ? r : unionRect.united(r);
            }

            auto* group = new PortGroupItem{this, slice, start, end, this};
            // Bar along the left of the union (scene coords; host at 0,0).
            const QRectF bar {
                unionRect.left() - PortGroupItem::BAR_GAP - PortGroupItem::BAR_THICKNESS,
                unionRect.top(),
                PortGroupItem::BAR_THICKNESS,
                std::max(PortGroupItem::BAR_THICKNESS, unionRect.height())
            };
            group->setBarRect(bar);
            this->_groups.push_back(group);
            bounds = bounds.isNull() ? group->boundingRect() : bounds.united(group->boundingRect());
        }

        this->prepareGeometryChange();
        this->_bounds = bounds;
        this->_syncedGroupSize = groupSize;
    }

    void ExportPortGroupHost::syncGroups() {
        // Clear expand when zoom has caught up to a finer size.
        if (this->_expandGroupSize.has_value()) {
            const qreal s = this->viewScale();
            const int zoomSize = PortGroupLod::exportGroupSizeForScale(s, this->_exports.size());
            if (zoomSize > 0 && zoomSize <= *this->_expandGroupSize) {
                this->_expandGroupSize.reset();
            }
        }

        const int want = this->effectiveGroupSize();
        if (want == this->_syncedGroupSize && (want <= 1 || !this->_groups.isEmpty())) {
            if (want <= 1 && !this->_groups.isEmpty()) {
                this->clearGroups();
                this->_syncedGroupSize = want;
                this->applyExportBodyVisibility(false);
            }
            return;
        }
        this->rebuildGroups(want);
    }

    void ExportPortGroupHost::scheduleSync() {
        if (this->_syncPending) {
            return;
        }
        this->_syncPending = true;
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            sc->requestExportPortGroupSync();
        }
        this->_syncPending = false;
    }

    auto ExportPortGroupHost::boundingRect() const -> QRectF {
        QRectF r = this->_bounds;
        for (auto* e : this->_exports) {
            if (!e) {
                continue;
            }
            const QRectF sr = e->sceneBoundingRect().adjusted(
                -PortGroupItem::BAR_THICKNESS - PortGroupItem::BAR_GAP - 4.,
                -4.,
                PortGroupItem::BAR_THICKNESS + PortGroupItem::BAR_GAP + 4.,
                4.
            );
            r = r.isNull() ? sr : r.united(sr);
        }
        if (r.isNull()) {
            return QRectF{0., 0., 1., 1.};
        }
        return r;
    }

    void ExportPortGroupHost::paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) {
        // Children draw bars; defer LOD rebuild out of paint.
        const int want = this->effectiveGroupSize();
        if (want != this->_syncedGroupSize
            || (want > 1 && this->_groups.isEmpty())
            || (want <= 1 && !this->_groups.isEmpty())) {
            const_cast<ExportPortGroupHost*>(this)->scheduleSync();
        }
    }

}
