#include "./topdieinstitem.h"
#include "./pinitem.h"
#include "./portgroupitem.h"
#include "./netitem.h"
#include "../schematicscene.h"
#include "qchar.h"
#include "qcolor.h"
#include "qnamespace.h"
#include "qpoint.h"
#include "qvector.h"

#include <circuit/topdieinst/topdieinst.hh>
#include <circuit/topdie/topdie.hh>

#include <QFont>
#include <QFontMetricsF>
#include <QGraphicsScene>
#include <QGraphicsSceneHoverEvent>
#include <QGraphicsView>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

#include <algorithm>
#include <string_view>


namespace PR_tool::widget::schematic {

    namespace {

        // Spec 4.0: deeper type-colored header bar (more visible than body),
        // still semi-transparent so nets under/through remain readable.
        auto headerFillFrom(const QColor& base) -> QColor {
            const qreal h = base.hslHueF();
            const qreal s = base.hslSaturationF();
            const qreal l = base.lightnessF();
            if (h < 0.) {
                auto c = base.darker(130);
                c.setAlpha(185);
                return c;
            }
            return QColor::fromHslF(
                h,
                std::clamp(s * 0.90, 0.0, 1.0),
                std::clamp(l * 0.62, 0.18, 0.55),
                0.72);
        }

        // Spec 4.0: same-hue low-sat wash (not near-white, not high-sat slab),
        // transparent enough for connections crossing the body.
        auto bodyFillFrom(const QColor& base) -> QColor {
            const qreal h = base.hslHueF();
            const qreal s = base.hslSaturationF();
            const qreal l = base.lightnessF();
            if (h < 0.) {
                auto c = base.lighter(160);
                c.setAlpha(70);
                return c;
            }
            return QColor::fromHslF(
                h,
                std::clamp(s * 0.38, 0.0, 1.0),
                std::clamp(std::min(0.90, l + 0.28), 0.72, 0.92),
                0.28);
        }

    } // namespace

    auto TopDieInstanceItem::bodySizeForPinCount(std::size_t pinCount) -> QSizeF {
        auto pinsPeSide = pinCount / 4;
        auto remainder = pinCount % 4;

        auto top_pins = pinsPeSide + (remainder > 0 ? 1 : 0);
        auto right_pins = pinsPeSide + (remainder > 1 ? 1 : 0);
        auto bottom_pins = pinsPeSide + (remainder > 2 ? 1 : 0);
        auto left_pins = pinsPeSide;

        auto pinAreaWidth  = (std::max(top_pins, bottom_pins) + 1) * PIN_INTERVAL;
        auto pinAreaHeight = (std::max(left_pins, right_pins) + 1) * PIN_INTERVAL;

        return QSizeF {
            pinAreaWidth + 2 * SPACE_LENGTH,
            pinAreaHeight + 2 * SPACE_LENGTH
        };
    }

    auto TopDieInstanceItem::colorForTopDieType(std::string_view typeName) -> QColor {
        // Expandable restrained base hues (opaque). Header/body derive alpha +
        // saturation/lightness; unknown types hash into this table.
        static const QColor kPalette[] = {
            QColor(186, 132, 142), // dusty rose
            QColor(126, 162, 138), // sage green
            QColor(120, 148, 186), // soft blue
            QColor(196, 168, 110), // muted ochre
            QColor(152, 132, 178), // lavender
            QColor(110, 160, 158), // teal
            QColor(176, 120, 102), // terracotta
            QColor(148, 152, 128), // olive stone
        };

        std::size_t hash = 1469598103934665603ull;
        for (unsigned char c : typeName) {
            hash ^= c;
            hash *= 1099511628211ull;
        }
        return kPalette[hash % (sizeof(kPalette) / sizeof(kPalette[0]))];
    }

    TopDieInstanceItem::TopDieInstanceItem(circuit::TopDieInstance* topdieinst, std::size_t maxPinCount):
        GridItem{},
        _topdieinstance{topdieinst}
    {
        this->setFlags(this->flags()
            | QGraphicsItem::ItemIsMovable
            | QGraphicsItem::ItemIsSelectable);
        this->setZValue(0);

        auto pinmap = this->_topdieinstance->topdie()->pins_map();
        auto pinnames = QVector<QString>{};
        for (const auto& [name, _] : pinmap) {
            pinnames.push_back(QString::fromStdString(name));
        }

        this->_name = QString::fromStdString(topdieinst->name().data());
        this->_typeName = QString::fromStdString(topdieinst->topdie()->name().data());
        this->_fillColor = colorForTopDieType(topdieinst->topdie()->name());

        /*
            Header (type icon + type + instance name) sits above the pin body.
            Pins live on the body edges; nets may enter/cross the body.
        
            +----+-------------------------+----+
            | ICON TYPE      instance name      |  header
            +----+-------------------------+----+
            |    |        up pin           |    |
            +----+-------------------------+----+
            |    |                         |    |
            | l  |                         |    |
            | e  |                         |    |
            | f  |          BODY           |    |
            | t  |                         |    |
            |    |                         |    |
            +----+-------------------------+----+
            |    |       bottom pin        |    |
            +----+-------------------------+----+
        
        */
        const auto bodyPinCount = std::max(maxPinCount, pinmap.size());
        const auto body = bodySizeForPinCount(bodyPinCount);
        this->_width = body.width();
        this->_height = HEADER_HEIGHT + body.height();

        auto pinAreaWidth  = this->_width - 2 * SPACE_LENGTH;
        auto pinAreaHeight = body.height() - 2 * SPACE_LENGTH;
        const qreal bodyTop = HEADER_HEIGHT;

        auto pinsCount = pinmap.size();
        auto pinsPeSide = pinsCount / 4;
        auto remainder = pinsCount % 4;

        auto top_pins = pinsPeSide + (remainder > 0 ? 1 : 0);
        auto right_pins = pinsPeSide + (remainder > 1 ? 1 : 0);
        auto bottom_pins = pinsPeSide + (remainder > 2 ? 1 : 0);
        auto left_pins = pinsPeSide;

        auto iter = pinnames.begin();
        this->createPins(top_pins, pinAreaWidth, SPACE_LENGTH, bodyTop, iter, PinSide::Top);
        this->createPins(right_pins, pinAreaHeight, this->_width, bodyTop + SPACE_LENGTH, iter, PinSide::Right);
        this->createPins(bottom_pins, pinAreaWidth, SPACE_LENGTH, this->_height, iter, PinSide::Bottom);
        this->createPins(left_pins, pinAreaHeight, 0, bodyTop + SPACE_LENGTH, iter, PinSide::Left);
    }

    QRectF TopDieInstanceItem::boundingRect() const {
        return QRectF{0., 0., this->_width,  this->_height};
    }

    void TopDieInstanceItem::paintTypeIcon(QPainter* painter, const QRectF& iconRect) const {
        const auto kind = this->_typeName.toUpper();
        painter->setRenderHint(QPainter::Antialiasing, false);

        if (kind == QLatin1String("CPU")) {
            // ▣ filled square with inner frame
            painter->setBrush(Qt::black);
            painter->setPen(QPen(Qt::black, 1.5));
            painter->drawRect(iconRect);
            painter->setBrush(Qt::NoBrush);
            painter->drawRect(iconRect.adjusted(6., 6., -6., -6.));
        } else if (kind == QLatin1String("MEM")) {
            // ▤ rectangle with horizontal bars
            painter->setBrush(Qt::NoBrush);
            painter->setPen(QPen(Qt::black, 1.5));
            painter->drawRect(iconRect);
            const qreal y1 = iconRect.top() + iconRect.height() / 3.;
            const qreal y2 = iconRect.top() + 2. * iconRect.height() / 3.;
            painter->drawLine(QPointF(iconRect.left(), y1), QPointF(iconRect.right(), y1));
            painter->drawLine(QPointF(iconRect.left(), y2), QPointF(iconRect.right(), y2));
        } else if (kind == QLatin1String("AI")) {
            // ◈ diamond with inner diamond
            const QPointF c = iconRect.center();
            const qreal hx = iconRect.width() / 2.;
            const qreal hy = iconRect.height() / 2.;
            QPolygonF outer;
            outer << QPointF(c.x(), c.y() - hy)
                  << QPointF(c.x() + hx, c.y())
                  << QPointF(c.x(), c.y() + hy)
                  << QPointF(c.x() - hx, c.y());
            painter->setBrush(Qt::NoBrush);
            painter->setPen(QPen(Qt::black, 1.5));
            painter->drawPolygon(outer);
            const qreal ix = hx * 0.45;
            const qreal iy = hy * 0.45;
            QPolygonF inner;
            inner << QPointF(c.x(), c.y() - iy)
                  << QPointF(c.x() + ix, c.y())
                  << QPointF(c.x(), c.y() + iy)
                  << QPointF(c.x() - ix, c.y());
            painter->drawPolygon(inner);
        } else {
            // Default: simple square outline
            painter->setBrush(Qt::NoBrush);
            painter->setPen(QPen(Qt::black, 1.5));
            painter->drawRect(iconRect);
        }
    }

    auto TopDieInstanceItem::viewScale() const -> qreal {
        if (auto* sc = this->scene()) {
            const auto views = sc->views();
            if (!views.isEmpty()) {
                return views.first()->transform().m11();
            }
        }
        return 1.0;
    }

    auto TopDieInstanceItem::effectiveGroupSize() const -> int {
        const qreal s = this->viewScale();
        if (s < PinItem::LOD_FAR_MAX) {
            return 0;
        }
        // Near LOD: pins take over unless click-expand is still coarser... at Near always pins.
        if (s >= PinItem::LOD_NEAR_MIN) {
            return 1;
        }

        int zoomSize = PortGroupLod::groupSizeForScale(s);
        if (zoomSize <= 0) {
            return 0;
        }
        // Fixed-128 ladder: clamp to actual side length happens at rebuild time.
        if (this->_expandGroupSize.has_value() && *this->_expandGroupSize < zoomSize) {
            return std::max(1, *this->_expandGroupSize);
        }
        return zoomSize;
    }

    auto TopDieInstanceItem::shouldRevealPins() const -> bool {
        // Expand-to-single-pin at Medium zoom, or Near LOD.
        return this->effectiveGroupSize() <= 1 && this->viewScale() >= PinItem::LOD_FAR_MAX;
    }

    void TopDieInstanceItem::schedulePortGroupSync() {
        if (this->_portGroupSyncPending) {
            return;
        }
        this->_portGroupSyncPending = true;
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            sc->requestPortGroupSync(this);
        }
        this->_portGroupSyncPending = false;
    }

    void TopDieInstanceItem::expandPortGroups() {
        const int cur = std::max(1, this->effectiveGroupSize());
        if (cur <= 1) {
            this->_expandGroupSize = 1;
        } else {
            this->_expandGroupSize = std::max(1, cur / 2);
        }
        // Defer rebuild so PortGroupItem::mousePressEvent does not delete `this`.
        this->_syncedGroupSize = -1;
        this->schedulePortGroupSync();
    }

    auto TopDieInstanceItem::barRectForPins(PinSide side, const QVector<PinItem*>& pins) const -> QRectF {
        if (pins.isEmpty()) {
            return {};
        }
        qreal minX = pins.first()->pos().x();
        qreal maxX = minX;
        qreal minY = pins.first()->pos().y();
        qreal maxY = minY;
        for (auto* pin : pins) {
            const QPointF p = pin->pos();
            minX = std::min(minX, p.x());
            maxX = std::max(maxX, p.x());
            minY = std::min(minY, p.y());
            maxY = std::max(maxY, p.y());
        }

        const qreal t = PortGroupItem::BAR_THICKNESS;
        const qreal gap = PortGroupItem::BAR_GAP;
        constexpr qreal endPad = 6.;

        switch (side) {
            case PinSide::Top:
                return QRectF{
                    minX - endPad,
                    -gap - t,
                    std::max(t, maxX - minX) + 2. * endPad,
                    t
                };
            case PinSide::Bottom:
                return QRectF{
                    minX - endPad,
                    this->_height + gap,
                    std::max(t, maxX - minX) + 2. * endPad,
                    t
                };
            case PinSide::Left:
                return QRectF{
                    -gap - t,
                    minY - endPad,
                    t,
                    std::max(t, maxY - minY) + 2. * endPad
                };
            case PinSide::Right:
                return QRectF{
                    this->_width + gap,
                    minY - endPad,
                    t,
                    std::max(t, maxY - minY) + 2. * endPad
                };
        }
        return {};
    }

    void TopDieInstanceItem::clearPortGroups() {
        for (auto* g : this->_portGroups) {
            delete g;
        }
        this->_portGroups.clear();
        this->_syncedGroupSize = -1;
    }

    void TopDieInstanceItem::rebuildPortGroups(int groupSize) {
        this->clearPortGroups();
        if (groupSize <= 1) {
            this->_syncedGroupSize = groupSize;
            return;
        }

        struct SidePins {
            PinSide side;
            const QVector<PinItem*>* pins;
        };
        const SidePins sides[] = {
            {PinSide::Top, &this->_pinsTop},
            {PinSide::Right, &this->_pinsRight},
            {PinSide::Bottom, &this->_pinsBottom},
            {PinSide::Left, &this->_pinsLeft},
        };

        // Global bump index across sides in creation order (top→right→bottom→left).
        int globalIndex = 0;
        for (const auto& sp : sides) {
            const auto& list = *sp.pins;
            const int n = list.size();
            if (n == 0) {
                continue;
            }
            // One group covering the whole side when groupSize >= n.
            const int step = groupSize;
            for (int start = 0; start < n; start += step) {
                const int endLocal = std::min(n - 1, start + step - 1);
                QVector<PinItem*> slice;
                slice.reserve(endLocal - start + 1);
                for (int i = start; i <= endLocal; ++i) {
                    slice.push_back(list[i]);
                }
                const int gStart = globalIndex + start;
                const int gEnd = globalIndex + endLocal;
                auto* group = new PortGroupItem{
                    this, sp.side, gStart, gEnd, slice, this
                };
                group->setBarRect(this->barRectForPins(sp.side, slice));
                this->_portGroups.push_back(group);
            }
            globalIndex += n;
        }

        this->_syncedGroupSize = groupSize;
    }

    void TopDieInstanceItem::syncPortGroups() {
        if (this->_expandGroupSize.has_value()) {
            const qreal s = this->viewScale();
            const int zoomSize = PortGroupLod::groupSizeForScale(s);
            if (zoomSize > 0 && zoomSize <= *this->_expandGroupSize) {
                this->_expandGroupSize.reset();
            }
            // Near clears expand — pins own the display.
            if (s >= PinItem::LOD_NEAR_MIN) {
                this->_expandGroupSize.reset();
            }
        }

        const int want = this->effectiveGroupSize();
        if (want == this->_syncedGroupSize) {
            if (want <= 1 && !this->_portGroups.isEmpty()) {
                this->clearPortGroups();
                this->_syncedGroupSize = want;
            }
            return;
        }
        this->rebuildPortGroups(want);
    }

    void TopDieInstanceItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
        Q_UNUSED(option);
        Q_UNUSED(widget);

        // Keep Port Group LOD in sync with view scale (Ch.六) — defer mutations out of paint.
        const int want = this->effectiveGroupSize();
        if (want != this->_syncedGroupSize
            || (want > 1 && this->_portGroups.isEmpty())
            || (want <= 1 && !this->_portGroups.isEmpty())) {
            this->schedulePortGroupSync();
        }

        const QRectF bounds = this->boundingRect();
        const QRectF headerRect {0., 0., this->_width, HEADER_HEIGHT};
        const QRectF bodyRect {0., HEADER_HEIGHT, this->_width, this->_height - HEADER_HEIGHT};

        const QColor borderColor = this->_focusBorder
            ? QColor::fromRgb(20, 20, 20)
            : QColor::fromRgb(80, 80, 80);
        const qreal borderWidth = this->_focusBorder
            ? ConnectionFocusStyle::DIE_BORDER_FOCUS
            : ConnectionFocusStyle::DIE_BORDER_DEFAULT;

        // Body (lighter) then header (more visible) — sharp corners only.
        painter->setPen(Qt::NoPen);
        painter->setBrush(bodyFillFrom(this->_fillColor));
        painter->drawRect(bodyRect);

        painter->setBrush(headerFillFrom(this->_fillColor));
        painter->drawRect(headerRect);

        painter->setBrush(Qt::NoBrush);
        painter->setPen(QPen(borderColor, borderWidth));
        painter->drawRect(bounds);
        painter->drawLine(QPointF(0., HEADER_HEIGHT), QPointF(this->_width, HEADER_HEIGHT));

        // Header: icon + type + instance name. Always drawn (never hidden on zoom);
        // elide to fit — Pin LOD (Ch.五) may hide pins later, not this header.
        const qreal pad = 8.;
        const qreal iconSize = std::min(HEADER_ICON_SIZE, HEADER_HEIGHT - 2. * pad);
        const QRectF iconRect {
            pad,
            (HEADER_HEIGHT - iconSize) / 2.,
            iconSize,
            iconSize
        };
        this->paintTypeIcon(painter, iconRect);

        auto font = painter->font();
        font.setPixelSize(HEADER_FONT_PIXEL_SIZE);
        font.setBold(true);
        painter->setFont(font);
        painter->setPen(Qt::black);

        const QFontMetricsF fm {font};
        const qreal textLeft = iconRect.right() + pad;
        const qreal textMaxWidth = std::max(0., this->_width - textLeft - pad);

        const QString typeLabel = fm.elidedText(this->_typeName, Qt::ElideRight, textMaxWidth * 0.35);
        const qreal typeWidth = fm.horizontalAdvance(typeLabel);
        const QRectF typeRect {
            textLeft,
            0.,
            typeWidth,
            HEADER_HEIGHT
        };
        painter->drawText(typeRect, Qt::AlignVCenter | Qt::AlignLeft, typeLabel);

        const qreal nameLeft = typeRect.right() + pad * 1.5;
        const qreal nameMaxWidth = std::max(0., this->_width - nameLeft - pad);
        const QString nameLabel = fm.elidedText(this->_name, Qt::ElideRight, nameMaxWidth);
        const QRectF nameRect {
            nameLeft,
            0.,
            nameMaxWidth,
            HEADER_HEIGHT
        };
        painter->drawText(nameRect, Qt::AlignVCenter | Qt::AlignLeft, nameLabel);
    }

    void TopDieInstanceItem::setFocusBorder(bool on) {
        if (this->_focusBorder == on) {
            return;
        }
        this->_focusBorder = on;
        this->update();
    }

    void TopDieInstanceItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event) {
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            sc->setHoverTopDie(this);
        }
        QGraphicsItem::hoverEnterEvent(event);
    }

    void TopDieInstanceItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event) {
        if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
            sc->setHoverTopDie(nullptr);
        }
        QGraphicsItem::hoverLeaveEvent(event);
    }

    auto TopDieInstanceItem::itemChange(GraphicsItemChange change, const QVariant& value) -> QVariant {
        if (change == QGraphicsItem::ItemSelectedHasChanged) {
            if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
                sc->onTopDieSelectionChanged(this, value.toBool());
            }
        }
        return GridItem::itemChange(change, value);
    }

    void TopDieInstanceItem::createPins(int n, qreal side_length, qreal x_offset, qreal y_offset, QVector<QString>::iterator& iter, PinSide side) {
        if (n == 0) return;

        auto horizontal = side == PinSide::Bottom || side == PinSide::Top;

        double spacing = static_cast<double>(side_length) / (n + 1);

        QVector<PinItem*>* sideList = nullptr;
        switch (side) {
            case PinSide::Top: sideList = &this->_pinsTop; break;
            case PinSide::Right: sideList = &this->_pinsRight; break;
            case PinSide::Bottom: sideList = &this->_pinsBottom; break;
            case PinSide::Left: sideList = &this->_pinsLeft; break;
        }

        for (int i = 0; i < n; ++i, ++iter) {
            double pos = (i + 1) * spacing;
            int x = horizontal ? static_cast<int>(pos + x_offset) : x_offset;
            int y = horizontal ? y_offset : static_cast<int>(pos + y_offset);

            auto *pin = new PinItem {*iter, QPointF(x, y), side, this};
            this->_pins.insert(*iter, pin);
            if (sideList) {
                sideList->push_back(pin);
            }
        }
    }

}
