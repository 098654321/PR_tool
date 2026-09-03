#pragma once

#include <QWidget>
#include <QRectF>
#include <QPointF>

class QGraphicsScene;

namespace PR_tool::widget {

    class SchematicView;

    /// Ch.十九: canvas overlay — TopDie blocks + live viewport rect.
    class SchematicMiniMap : public QWidget {
    public:
        explicit SchematicMiniMap(SchematicView* view);

        void bindScene();

    protected:
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
        void mouseMoveEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event) override;

    private:
        struct MapXform {
            QRectF world;
            QPointF origin;
            qreal scale {1.};

            auto toMap(const QPointF& scenePt) const -> QPointF;
            auto toScene(const QPointF& mapPt) const -> QPointF;
            auto toMapRect(const QRectF& sceneRect) const -> QRectF;
        };

        auto currentXform() const -> MapXform;
        void panTo(const QPoint& widgetPos);

        SchematicView* _view {nullptr};
        QGraphicsScene* _boundScene {nullptr};
        bool _dragging {false};
    };

}
