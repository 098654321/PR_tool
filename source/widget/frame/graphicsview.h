#pragma once

#include <QGraphicsView>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsItem>

namespace PR_tool::widget {

    class GraphicsView : public QGraphicsView {
        Q_OBJECT

    public:
        explicit GraphicsView(QWidget* parent = nullptr);

    public:
        void adjustSceneRect();
        /// Fit all scene items into the viewport (KeepAspectRatio).
        virtual void fitContent();
        /// Restore the default zoom (constructor scale 1/2.5).
        virtual void resetZoom();
        /// Ch.十四 Locate: raise scale to at least minScale, then center on item.
        virtual void ensureVisibleAtMinScale(QGraphicsItem* item, qreal minScale);

        static constexpr qreal kDefaultScale = 1.0 / 2.5;

        /// Results lookback: pan/zoom only; left-click editing is blocked.
        void setLookbackLocked(bool locked);
        auto isLookbackLocked() const -> bool { return this->_lookbackLocked; }

    protected:
        void wheelEvent(QWheelEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;
        void mouseDoubleClickEvent(QMouseEvent* event) override;

    protected:
        bool _isPanning; 
        QPoint _lastMousePos;
        bool _lookbackLocked {false};
    };

}