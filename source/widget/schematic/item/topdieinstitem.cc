#include "./topdieinstitem.h"
#include "./pinitem.h"
#include "./portgroupitem.h"
#include "./netitem.h"
#include "../schematicscene.h"
#include "../schematictypography.h"
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

        // Spec 4.0: deeper type-colored cap (more visible than body),
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
            Type-color cap on the top edge. Name + type sit in the body center.
            Pins live on the body edges; nets may enter/cross the body.
        */
        const auto bodyPinCount = std::max(maxPinCount, pinmap.size());
        const auto body = bodySizeForPinCount(bodyPinCount);
        this->_width = body.width();
        this->_height = body.height();

        auto pinAreaWidth  = this->_width - 2 * SPACE_LENGTH;
        auto pinAreaHeight = body.height() - 2 * SPACE_LENGTH;
        const qreal bodyTop = 0.;

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
        // Port-group bars on TopDie edges are not used; pins own the four sides.
        return 1;
    }

    auto TopDieInstanceItem::shouldRevealPins() const -> bool {
        return this->viewScale() >= PinItem::LOD_FAR_MAX;
    }

    auto TopDieInstanceItem::emphasizePins() const -> bool {
        return this->isSelected()
            || this->_chromeState == ItemChromeState::Hover
            || this->_chromeState == ItemChromeState::Selected;
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
        const QRectF capRect {0., 0., this->_width, TYPE_CAP_HEIGHT};

        const QColor borderColor = itemChromeBorder(this->_chromeState);
        const qreal borderWidth = itemChromeWidth(this->_chromeState);
        const qreal dim = qBound(0., this->_chromeOpacity, 1.);
        const qreal textDim = qMax(dim, ConnectionFocusStyle::DEFAULT_OPACITY);

        QColor bodyFill = bodyFillFrom(this->_fillColor);
        QColor capFill = headerFillFrom(this->_fillColor);
        bodyFill.setAlphaF(bodyFill.alphaF() * dim);
        capFill.setAlphaF(capFill.alphaF() * dim);
        QColor border = borderColor;
        border.setAlphaF(dim);

        painter->setRenderHint(QPainter::Antialiasing, false);
        painter->setPen(Qt::NoPen);
        painter->setBrush(bodyFill);
        painter->drawRect(bounds);
        painter->setBrush(capFill);
        painter->drawRect(capRect);

        painter->setBrush(Qt::NoBrush);
        painter->setPen(QPen(border, borderWidth));
        painter->drawRect(bounds);

        // Name + type centered in the body. Always drawn (never hidden on zoom).
        const qreal pad = 8.;
        const QRectF labelArea = bounds.adjusted(pad, TYPE_CAP_HEIGHT + pad, -pad, -pad);
        const auto nameFont = SchematicTypography::topDieNameFont();
        const auto typeFont = SchematicTypography::topDieTypeFont();
        const QFontMetricsF nameFm {nameFont};
        const QFontMetricsF typeFm {typeFont};
        const QString nameLabel = nameFm.elidedText(this->_name, Qt::ElideRight, labelArea.width());
        const QString typeLabel = typeFm.elidedText(this->_typeName, Qt::ElideRight, labelArea.width());
        const qreal gap = 4.;
        const qreal blockH = nameFm.height() + gap + typeFm.height();
        const qreal blockTop = labelArea.center().y() - blockH / 2.;

        painter->setRenderHint(QPainter::TextAntialiasing, true);
        QColor nameColor = ChromeTokens::color(ChromeTokens::text);
        nameColor.setAlphaF(textDim);
        painter->setPen(nameColor);
        painter->setFont(nameFont);
        painter->drawText(
            QRectF{labelArea.left(), blockTop, labelArea.width(), nameFm.height()},
            Qt::AlignHCenter | Qt::AlignVCenter,
            nameLabel);

        QColor typeColor = ChromeTokens::color(ChromeTokens::textMuted);
        typeColor.setAlphaF(textDim);
        painter->setPen(typeColor);
        painter->setFont(typeFont);
        painter->drawText(
            QRectF{labelArea.left(), blockTop + nameFm.height() + gap, labelArea.width(), typeFm.height()},
            Qt::AlignHCenter | Qt::AlignVCenter,
            typeLabel);
    }

    void TopDieInstanceItem::setChromeState(ItemChromeState state, qreal opacity) {
        opacity = qBound(0., opacity, 1.);
        if (this->_chromeState == state && qAbs(this->_chromeOpacity - opacity) < 0.0001) {
            return;
        }
        this->_chromeState = state;
        this->_chromeOpacity = opacity;
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
            if (!sc->hoverMovesWithin(this, event->scenePos())) {
                sc->setHoverTopDie(nullptr);
            }
        }
        QGraphicsItem::hoverLeaveEvent(event);
    }

    auto TopDieInstanceItem::itemChange(GraphicsItemChange change, const QVariant& value) -> QVariant {
        if (change == QGraphicsItem::ItemSelectedHasChanged) {
            if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
                sc->onTopDieSelectionChanged(this, value.toBool());
            }
        }
        if (change == QGraphicsItem::ItemScenePositionHasChanged) {
            if (auto* sc = dynamic_cast<SchematicScene*>(this->scene())) {
                sc->refreshPowerRails();
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
