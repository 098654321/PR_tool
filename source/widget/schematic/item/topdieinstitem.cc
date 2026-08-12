#include "./topdieinstitem.h"
#include "./pinitem.h"
#include "qchar.h"
#include "qcolor.h"
#include "qnamespace.h"
#include "qpoint.h"
#include "qvector.h"

#include <circuit/topdieinst/topdieinst.hh>
#include <circuit/topdie/topdie.hh>

#include <QFont>
#include <QFontMetricsF>
#include <QPainter>
#include <QStyleOptionGraphicsItem>

#include <algorithm>
#include <string_view>


namespace PR_tool::widget::schematic {

    namespace {

        auto headerFillFrom(const QColor& base) -> QColor {
            auto c = base;
            c = c.darker(115);
            c.setAlpha(210);
            return c;
        }

        auto bodyFillFrom(const QColor& base) -> QColor {
            auto c = base;
            c = c.lighter(135);
            c.setAlpha(90);
            return c;
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
        // Morandi fills with wider hue spacing so common types stay distinct.
        static const QColor kPalette[] = {
            QColor::fromRgb(186, 132, 142, 160), // dusty rose
            QColor::fromRgb(126, 162, 138, 160), // sage green
            QColor::fromRgb(120, 148, 186, 160), // soft blue
            QColor::fromRgb(196, 168, 110, 160), // muted ochre
            QColor::fromRgb(152, 132, 178, 160), // lavender
            QColor::fromRgb(110, 160, 158, 160), // teal
            QColor::fromRgb(176, 120, 102, 160), // terracotta
            QColor::fromRgb(148, 152, 128, 160), // olive stone
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
        this->setFlags(this->flags() | QGraphicsItem::ItemIsMovable);
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

    void TopDieInstanceItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
        Q_UNUSED(option);
        Q_UNUSED(widget);

        const QRectF bounds = this->boundingRect();
        const QRectF headerRect {0., 0., this->_width, HEADER_HEIGHT};
        const QRectF bodyRect {0., HEADER_HEIGHT, this->_width, this->_height - HEADER_HEIGHT};

        const QColor borderColor = QColor::fromRgb(80, 80, 80);

        // Body (lighter) then header (more visible) — sharp corners only.
        painter->setPen(Qt::NoPen);
        painter->setBrush(bodyFillFrom(this->_fillColor));
        painter->drawRect(bodyRect);

        painter->setBrush(headerFillFrom(this->_fillColor));
        painter->drawRect(headerRect);

        painter->setBrush(Qt::NoBrush);
        painter->setPen(QPen(borderColor, 1.5));
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

    void TopDieInstanceItem::createPins(int n, qreal side_length, qreal x_offset, qreal y_offset, QVector<QString>::iterator& iter, PinSide side) {
        if (n == 0) return;

        auto horizontal = side == PinSide::Bottom || side == PinSide::Top;

        double spacing = static_cast<double>(side_length) / (n + 1);

        for (int i = 0; i < n; ++i, ++iter) {
            double pos = (i + 1) * spacing;
            int x = horizontal ? static_cast<int>(pos + x_offset) : x_offset;
            int y = horizontal ? y_offset : static_cast<int>(pos + y_offset);

            auto *pin = new PinItem {*iter, QPointF(x, y), side, this};
            this->_pins.insert(*iter, pin);
        }
    }

}
