#pragma once

#include <QScrollArea>
#include <QVector>
#include <QWidget>

namespace PR_tool::widget {

    class PlaceProgressPlot : public QWidget {
    public:
        explicit PlaceProgressPlot(QWidget* parent = nullptr);

        void setStepX(qreal step);
        void append(qreal cost);
        void lockStep();
        auto isStepLocked() const -> bool { return this->_stepLocked; }
        auto sizeHint() const -> QSize override;
        auto minimumSizeHint() const -> QSize override;

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        auto contentWidth() const -> int;
        auto yOf(qreal cost) const -> qreal;

        QVector<qreal> _costs;
        qreal _stepX {10};
        qreal _yMin {0};
        qreal _yMax {1};
        bool _stepLocked {false};
    };

    class PlaceProgressChart : public QScrollArea {
    public:
        explicit PlaceProgressChart(QWidget* parent = nullptr);

        void append(qreal cost);
        auto sizeHint() const -> QSize override;

    protected:
        void resizeEvent(QResizeEvent* event) override;

    private:
        void refreshStep();
        void followIfNeeded();

        PlaceProgressPlot* _plot {nullptr};
        bool _follow {true};
    };

}
