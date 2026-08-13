#pragma once

#include "./griditem.h"
#include "./netitem.h"
#include "./pinitem.h"
#include "qchar.h"
#include "qcolor.h"
#include "qglobal.h"
#include "qmap.h"
#include <QGraphicsItem>
#include <cstddef>
#include <optional>
#include <string_view>

namespace PR_tool::circuit {
    class TopDieInstance;
}

namespace PR_tool::widget {
    class SchematicScene;
}

namespace PR_tool::widget::schematic {

    class PortGroupItem;

    class TopDieInstanceItem : public GridItem {
    public:
        static constexpr int PIN_INTERVAL_SIZE = 1;
        static constexpr int SPACE_LENGTH_SIZE = 4;
        static constexpr int HEADER_HEIGHT_SIZE = 3;

        static constexpr qreal PIN_INTERVAL = GridItem::gridLength(PIN_INTERVAL_SIZE);
        static constexpr qreal SPACE_LENGTH = GridItem::gridLength(SPACE_LENGTH_SIZE);
        static constexpr qreal HEADER_HEIGHT = GridItem::gridLength(HEADER_HEIGHT_SIZE);
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

        /// View scale s = transform().m11().
        auto viewScale() const -> qreal;
        /// Effective Port Group size after zoom LOD + click expand (0 = Far / hide).
        auto effectiveGroupSize() const -> int;
        /// Click-expand one LOD step (halve group size).
        void expandPortGroups();
        /// Rebuild child PortGroupItems for current effective size.
        void syncPortGroups();
        /// When expand reaches size 1 at Medium zoom, pins should paint.
        auto shouldRevealPins() const -> bool;

        auto portGroups() const -> const QVector<PortGroupItem*>& { return this->_portGroups; }

        /// Ch.21 chrome: Normal / Hover / Selected / Related / Dimmed.
        void setChromeState(ItemChromeState state, qreal opacity = 1.0);

    protected:
        void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
        void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
        auto itemChange(GraphicsItemChange change, const QVariant& value) -> QVariant override;

    private:
        void clearPortGroups();
        void rebuildPortGroups(int groupSize);
        void schedulePortGroupSync();
        auto barRectForPins(PinSide side, const QVector<PinItem*>& pins) const -> QRectF;

    protected:
        QString _name {};
        QString _typeName {};
        qreal _width {};
        qreal _height {};
        QColor _fillColor {};
        ItemChromeState _chromeState {ItemChromeState::Normal};
        qreal _chromeOpacity {1.0};

        circuit::TopDieInstance* const _topdieinstance {nullptr};
        QMap<QString, PinItem*> _pins {};

        // Pins in creation order per side (Top, Right, Bottom, Left).
        QVector<PinItem*> _pinsTop {};
        QVector<PinItem*> _pinsRight {};
        QVector<PinItem*> _pinsBottom {};
        QVector<PinItem*> _pinsLeft {};

        QVector<PortGroupItem*> _portGroups {};
        std::optional<int> _expandGroupSize {};
        int _syncedGroupSize {-1};
        bool _portGroupSyncPending {false};
    };


}
