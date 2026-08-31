#include <QApplication>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPathItem>
#include <QPen>

namespace PR_tool::widget::view2d {

    class HighLightPathItem : public QGraphicsPathItem {
    public:
        explicit HighLightPathItem(QGraphicsItem* parent = nullptr)
            : QGraphicsPathItem(parent) {
            setPen(QPen(Qt::black, 2));
            setAcceptHoverEvents(true);
        }

        void setRouted(bool routed) {
            this->_routed = routed;
            setPen(QPen(routed ? Qt::red : Qt::black, routed ? 3 : 2));
        }

    protected:
        void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override {
            setPen(QPen(Qt::red, 5));
            QGraphicsPathItem::hoverEnterEvent(event);
        }

        void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override {
            setPen(QPen(this->_routed ? Qt::red : Qt::black, this->_routed ? 3 : 2));
            QGraphicsPathItem::hoverLeaveEvent(event);
        }

    private:
        bool _routed {false};
    };

}
