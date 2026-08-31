#include "./placeprogresschart.h"
#include "../chrometokens.h"

#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>
#include <QScrollBar>
#include <QFrame>

namespace PR_tool::widget {

    namespace {
        constexpr qreal kPadX = 16;
        constexpr qreal kPadY = 16;
        constexpr int kPlotH = 104;
        constexpr qreal kLineW = 2.6;
        constexpr qreal kDotR = 2.05;
        constexpr qreal kStepsInView = 28;
    }

    PlaceProgressPlot::PlaceProgressPlot(QWidget* parent)
        : QWidget{parent}
    {
        this->setAttribute(Qt::WA_OpaquePaintEvent, true);
        this->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    }

    void PlaceProgressPlot::setStepX(qreal step) {
        if (this->_stepLocked) {
            return;
        }
        this->_stepX = qMax(qreal(1), step);
        this->updateGeometry();
        this->update();
    }

    void PlaceProgressPlot::append(qreal cost) {
        if (this->_costs.isEmpty()) {
            this->_yMax = cost * 1.12;
            this->_yMin = 0;
            if (this->_yMax <= this->_yMin) {
                this->_yMax = 1;
            }
        }
        this->_costs.push_back(cost);
        this->updateGeometry();
        this->update();
    }

    void PlaceProgressPlot::lockStep() {
        this->_stepLocked = true;
    }

    auto PlaceProgressPlot::contentWidth() const -> int {
        const int n = this->_costs.size();
        if (n <= 0) {
            return 1;
        }
        return qRound(kPadX * 2 + qMax(0, n - 1) * this->_stepX);
    }

    auto PlaceProgressPlot::sizeHint() const -> QSize {
        return QSize{this->contentWidth(), kPlotH};
    }

    auto PlaceProgressPlot::minimumSizeHint() const -> QSize {
        return this->sizeHint();
    }

    auto PlaceProgressPlot::yOf(qreal cost) const -> qreal {
        if (this->_yMax <= this->_yMin) {
            return kPlotH / 2.0;
        }
        const qreal t = (cost - this->_yMin) / (this->_yMax - this->_yMin);
        const qreal y = kPadY + (1 - t) * (kPlotH - kPadY * 2);
        return qBound(kPadY, y, kPlotH - kPadY);
    }

    void PlaceProgressPlot::paintEvent(QPaintEvent*) {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        const QColor wash = ChromeTokens::color(ChromeTokens::selectionFill);
        p.fillRect(this->rect(), wash);

        if (this->_costs.isEmpty()) {
            return;
        }

        const QColor accent = ChromeTokens::color(ChromeTokens::accent);
        QPainterPath path;
        for (int i = 0; i < this->_costs.size(); ++i) {
            const QPointF pt(kPadX + i * this->_stepX, this->yOf(this->_costs[i]));
            if (i == 0) {
                path.moveTo(pt);
            } else {
                path.lineTo(pt);
            }
        }

        p.setBrush(Qt::NoBrush);
        QPen pen(accent, kLineW, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        p.setPen(pen);
        p.drawPath(path);

        p.setPen(Qt::NoPen);
        p.setBrush(accent);
        for (int i = 0; i < this->_costs.size(); ++i) {
            const QPointF pt(kPadX + i * this->_stepX, this->yOf(this->_costs[i]));
            p.drawEllipse(pt, kDotR, kDotR);
        }
    }

    PlaceProgressChart::PlaceProgressChart(QWidget* parent)
        : QScrollArea{parent}
    {
        this->setWidgetResizable(false);
        this->setFrameShape(QFrame::NoFrame);
        this->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        this->setFocusPolicy(Qt::NoFocus);
        this->setMinimumHeight(112);
        this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

        this->_plot = new PlaceProgressPlot{this};
        this->setWidget(this->_plot);

        const QColor wash = ChromeTokens::color(ChromeTokens::selectionFill);
        QPalette pal = this->palette();
        pal.setColor(QPalette::Window, wash);
        pal.setColor(QPalette::Base, wash);
        this->setPalette(pal);
        this->viewport()->setPalette(pal);
        this->viewport()->setAutoFillBackground(true);

        this->setStyleSheet(QStringLiteral(
            "QScrollArea {"
            "  background-color: %1;"
            "  border: 1px solid #d7e6f5;"
            "  border-radius: 10px;"
            "}"
            "QScrollBar:horizontal {"
            "  height: 8px;"
            "  background: rgba(0, 113, 227, 0.06);"
            "  margin: 0;"
            "  border: none;"
            "}"
            "QScrollBar::handle:horizontal {"
            "  background: #b7cde0;"
            "  border-radius: 4px;"
            "  min-width: 24px;"
            "}"
            "QScrollBar::handle:horizontal:hover {"
            "  background: %2;"
            "}"
            "QScrollBar::add-line:horizontal,"
            "QScrollBar::sub-line:horizontal {"
            "  width: 0;"
            "  height: 0;"
            "}"
            "QScrollBar::add-page:horizontal,"
            "QScrollBar::sub-page:horizontal {"
            "  background: transparent;"
            "}")
            .arg(QLatin1String(ChromeTokens::selectionFill),
                 QLatin1String(ChromeTokens::accent)));

        auto* bar = this->horizontalScrollBar();
        QObject::connect(bar, &QScrollBar::sliderPressed, this, [this]() {
            this->_follow = false;
        });
        QObject::connect(bar, &QScrollBar::valueChanged, this, [this](int value) {
            this->_follow = value >= this->horizontalScrollBar()->maximum() - 2;
        });
    }

    void PlaceProgressChart::append(qreal cost) {
        this->refreshStep();
        this->_plot->append(cost);
        if (this->viewport()->width() > 1 || this->width() > 1) {
            this->_plot->lockStep();
        }
        this->_plot->adjustSize();
        this->followIfNeeded();
    }

    auto PlaceProgressChart::sizeHint() const -> QSize {
        return QSize{400, 112};
    }

    void PlaceProgressChart::resizeEvent(QResizeEvent* event) {
        QScrollArea::resizeEvent(event);
        this->refreshStep();
        this->_plot->adjustSize();
        this->followIfNeeded();
    }

    void PlaceProgressChart::refreshStep() {
        if (this->_plot->isStepLocked()) {
            return;
        }
        const int w = qMax(qMax(this->viewport()->width(), this->width()), 408);
        const qreal inner = qMax(qreal(1), qreal(w) - kPadX * 2);
        this->_plot->setStepX(inner / kStepsInView);
    }

    void PlaceProgressChart::followIfNeeded() {
        if (!this->_follow) {
            return;
        }
        auto* bar = this->horizontalScrollBar();
        bar->setValue(bar->maximum());
    }

}
