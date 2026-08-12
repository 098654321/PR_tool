#pragma once

#include "./griditem.h"
#include "./pinitem.h"
#include "qchar.h"
#include "qcolor.h"
#include "qglobal.h"
#include "qmap.h"
#include <QGraphicsItem>
#include <cstddef>
#include <string_view>

namespace PR_tool::circuit {
    class TopDieInstance;
}

namespace PR_tool::widget {
    class SchematicScene;
}

namespace PR_tool::widget::schematic {

    class TopDieInstanceItem : public GridItem {
    public:
        static constexpr int PIN_INTERVAL_SIZE = 1;
        static constexpr int SPACE_LENGTH_SIZE = 4;
        static constexpr int HEADER_HEIGHT_SIZE = 3;

        static constexpr qreal PIN_INTERVAL = GridItem::gridLength(PIN_INTERVAL_SIZE);
        static constexpr qreal SPACE_LENGTH = GridItem::gridLength(SPACE_LENGTH_SIZE);
        static constexpr qreal HEADER_HEIGHT = GridItem::gridLength(HEADER_HEIGHT_SIZE);
        static constexpr int HEADER_FONT_PIXEL_SIZE = 28;
        static constexpr qreal HEADER_ICON_SIZE = 32.;

        enum { Type = UserType + 6 };
        int type() const override { return Type; }

        /// Body size is driven by the largest topdie pin-map in the design (option B).
        static auto bodySizeForPinCount(std::size_t pinCount) -> QSizeF;
        static auto colorForTopDieType(std::string_view typeName) -> QColor;

    public:
        TopDieInstanceItem(circuit::TopDieInstance* topdieinst, std::size_t maxPinCount);

        
    protected:
        auto boundingRect() const -> QRectF override;
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *item, QWidget *widget) override;

    protected:
        void createPins(int n, qreal side_length, qreal x_offset, qreal y_offset, QVector<QString>::iterator& iter, PinSide side);
        void paintTypeIcon(QPainter* painter, const QRectF& iconRect) const;
    
    public: 
        auto name() const -> const QString 
        { return this->_name; }

        auto typeName() const -> const QString
        { return this->_typeName; }
        
        auto width() const -> qreal 
        { return this->_width; }
        
        auto height() const -> qreal 
        { return this->_height; }

        auto pins() const -> const QMap<QString, PinItem*>& 
        { return this->_pins; }

        auto unwrap() const -> circuit::TopDieInstance* { return this->_topdieinstance; }

        void setName(const QString& name) { this->_name = name; }

    protected:
        QString _name {};
        QString _typeName {};
        qreal _width {};
        qreal _height {};
        QColor _fillColor {};

        circuit::TopDieInstance* const _topdieinstance {nullptr};
        QMap<QString, PinItem*> _pins {};
    };


}
