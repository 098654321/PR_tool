#pragma once

#include "./pinitem.h"

#include <QGraphicsItem>
#include <QVector>
#include <optional>

namespace PR_tool::widget::schematic {

    class TopDieInstanceItem;
    class ExternalPortItem;
    class ExportPortGroupHost;

    /// Named zoom→group-size thresholds (Ch.六 6.0).
    struct PortGroupLod {
        static constexpr qreal SCALE_SIZE_128 = 0.45;
        static constexpr qreal SCALE_SIZE_64  = 0.55;
        static constexpr qreal SCALE_SIZE_32  = 0.65;
        static constexpr qreal SCALE_SIZE_16  = 0.75;
        static constexpr qreal SCALE_SIZE_8   = 0.85;
        static constexpr qreal SCALE_SIZE_4   = 0.95;
        static constexpr qreal SCALE_SIZE_2   = 1.05;
        static constexpr qreal SCALE_SIZE_1   = 1.15;

        /// Group size from view scale for fixed-128 TopDie bumps; 0 = Far (hide).
        static auto groupSizeForScale(qreal s) -> int;

        /// Export aggregate: start at `count`, halve at each size threshold.
        static auto exportGroupSizeForScale(qreal s, int count) -> int;
    };

    /// Contiguous pin-range bar on a TopDieInst side, or an export aggregate range.
    class PortGroupItem : public QGraphicsItem {
    public:
        static constexpr qreal BAR_THICKNESS = 10.;
        static constexpr qreal BAR_GAP = 4.;
        static constexpr qreal LABEL_PAD = 2.;
        static const QColor COLOR;
        static const QColor SELECTED_COLOR;

        enum { Type = UserType + 8 };
        int type() const override { return Type; }

        PortGroupItem(
            TopDieInstanceItem* owner,
            PinSide side,
            int startIndex,
            int endIndex,
            const QVector<PinItem*>& pins,
            QGraphicsItem* parent
        );

        PortGroupItem(
            ExportPortGroupHost* host,
            const QVector<ExternalPortItem*>& exports,
            int startIndex,
            int endIndex,
            QGraphicsItem* parent
        );

        auto boundingRect() const -> QRectF override;
        auto shape() const -> QPainterPath override;
        void paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) override;

        auto startIndex() const -> int { return this->_startIndex; }
        auto endIndex() const -> int { return this->_endIndex; }
        auto side() const -> PinSide { return this->_side; }
        auto isExportGroup() const -> bool { return this->_exportMode; }

        auto ownerTopDie() const -> TopDieInstanceItem* { return this->_owner; }
        auto exportHost() const -> ExportPortGroupHost* { return this->_exportHost; }
        auto exportMembers() const -> const QVector<ExternalPortItem*>& { return this->_exports; }
        auto pinMembers() const -> const QVector<PinItem*>& { return this->_pins; }

        void setBarRect(const QRectF& rect);

    protected:
        void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override;
        void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;
        void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

    private:
        auto labelText() const -> QString;
        auto parentExportContextActive() const -> bool;

    private:
        TopDieInstanceItem* _owner {nullptr};
        ExportPortGroupHost* _exportHost {nullptr};
        PinSide _side {PinSide::Bottom};
        int _startIndex {0};
        int _endIndex {0};
        QVector<PinItem*> _pins {};
        QVector<ExternalPortItem*> _exports {};
        bool _exportMode {false};
        QRectF _barRect {};
        bool _hovered {false};
    };

    /// Scene-level host that aggregates ExternalPortItems into Port Group bars.
    class ExportPortGroupHost : public QGraphicsItem {
    public:
        enum { Type = UserType + 9 };
        int type() const override { return Type; }

        explicit ExportPortGroupHost(QGraphicsItem* parent = nullptr);

        void setExports(const QVector<ExternalPortItem*>& exports);
        void syncGroups();
        void expandGroups();

        auto effectiveGroupSize() const -> int;
        auto viewScale() const -> qreal;

        auto boundingRect() const -> QRectF override;
        void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget*) override;

    private:
        void clearGroups();
        void rebuildGroups(int groupSize);
        void scheduleSync();
        void applyExportBodyVisibility(bool hideBodies);

        QVector<ExternalPortItem*> _exports {};
        QVector<PortGroupItem*> _groups {};
        std::optional<int> _expandGroupSize {};
        int _syncedGroupSize {-1};
        bool _syncPending {false};
        QRectF _bounds {};
    };

}
