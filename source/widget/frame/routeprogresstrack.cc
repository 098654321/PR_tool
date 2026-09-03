#include "./routeprogresstrack.h"
#include "../chrometokens.h"

#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>

namespace PR_tool::widget {

    RouteProgressTrack::RouteProgressTrack(QWidget* parent)
        : QWidget{parent}
    {
        this->setMinimumHeight(104);
        this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    void RouteProgressTrack::setProgress(int done, int total) {
        this->_done = qMax(0, done);
        this->_total = qMax(0, total);
        this->update();
    }

    auto RouteProgressTrack::sizeHint() const -> QSize {
        return QSize{400, 104};
    }

    void RouteProgressTrack::paintEvent(QPaintEvent*) {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        const QRectF r = this->rect().adjusted(1, 1, -1, -1);
        const QColor wash = ChromeTokens::color(ChromeTokens::selectionFill);
        const QColor accent = ChromeTokens::color(ChromeTokens::accent);
        const QColor ghost(183, 205, 224);
        const QColor pinMute(138, 163, 184);
        const QColor grid(0, 113, 227, 22);

        p.setPen(QPen(QColor(215, 230, 245), 1));
        p.setBrush(wash);
        p.drawRoundedRect(r, 10, 10);

        const qreal pad = 16;
        const qreal xL = r.left() + pad;
        const qreal xR = r.right() - pad;
        const qreal span = xR - xL;

        const qreal y0 = r.top() + 18;
        const qreal y2 = r.bottom() - 18;
        const qreal y1 = (y0 + y2) * 0.5;
        const qreal ys = (y1 + y2) * 0.5;

        // Channel routing: permute L0→R2, L1→R0, L2→R1.
        // Verticals at distinct X so no two nets share a collinear segment.
        const qreal xa = xL + span * 0.20;
        const qreal xb = xL + span * 0.36;
        const qreal xd = xL + span * 0.52;
        const qreal xc = xL + span * 0.68;

        QPainterPath nets[3];
        nets[0].moveTo(xL, y0);
        nets[0].lineTo(xa, y0);
        nets[0].lineTo(xa, ys);
        nets[0].lineTo(xc, ys);
        nets[0].lineTo(xc, y2);
        nets[0].lineTo(xR, y2);

        nets[1].moveTo(xL, y1);
        nets[1].lineTo(xb, y1);
        nets[1].lineTo(xb, y0);
        nets[1].lineTo(xR, y0);

        nets[2].moveTo(xL, y2);
        nets[2].lineTo(xd, y2);
        nets[2].lineTo(xd, y1);
        nets[2].lineTo(xR, y1);

        const QPointF src[3] = {
            QPointF(xL, y0),
            QPointF(xL, y1),
            QPointF(xL, y2),
        };
        const QPointF dst[3] = {
            QPointF(xR, y2),
            QPointF(xR, y0),
            QPointF(xR, y1),
        };

        p.setPen(QPen(grid, 1));
        const qreal hy[] = {y0, y1, ys, y2};
        for (qreal y : hy) {
            p.drawLine(QPointF(xL, y), QPointF(xR, y));
        }
        const qreal vx[] = {xa, xb, xd, xc};
        for (qreal x : vx) {
            p.drawLine(QPointF(x, r.top() + 8), QPointF(x, r.bottom() - 8));
        }

        auto strokeNets = [&](const QColor& color, qreal width) {
            p.setBrush(Qt::NoBrush);
            QPen pen(color, width, Qt::SolidLine, Qt::FlatCap, Qt::MiterJoin);
            pen.setMiterLimit(2.0);
            p.setPen(pen);
            for (const auto& path : nets) {
                p.drawPath(path);
            }
        };

        strokeNets(ghost, 2.2);

        const qreal ratio = (this->_total <= 0)
            ? 0.0
            : qBound(0.0, static_cast<qreal>(this->_done) / static_cast<qreal>(this->_total), 1.0);
        if (ratio > 0.0) {
            p.save();
            p.setClipRect(QRectF(r.left(), r.top(), r.width() * ratio, r.height()));
            strokeNets(accent, 2.6);
            p.restore();
        }

        const qreal clipX = r.left() + r.width() * ratio;
        auto drawPin = [&](const QPointF& pt, bool lit) {
            p.setPen(QPen(lit ? accent : pinMute, 1.4));
            p.setBrush(lit ? accent : QColor(255, 255, 255));
            p.drawEllipse(pt, 3.6, 3.6);
        };
        for (int i = 0; i < 3; ++i) {
            drawPin(src[i], src[i].x() <= clipX);
            drawPin(dst[i], dst[i].x() <= clipX);
        }
    }

}
