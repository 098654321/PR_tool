#include "./schematicautoroute.h"

#include <cmath>

namespace PR_tool::widget::schematic {

    namespace {
        constexpr qreal kEps = 0.5;

        void appendUnique(QVector<QPointF>& points, const QPointF& p) {
            if (points.isEmpty() || points.back() != p) {
                points.push_back(p);
            }
        }

        auto outwardFromPin(const QPointF& pin, PinSide side, qreal clearance) -> QPointF {
            switch (side) {
                case PinSide::Left:
                    return QPointF{pin.x() - clearance, pin.y()};
                case PinSide::Right:
                    return QPointF{pin.x() + clearance, pin.y()};
                case PinSide::Top:
                    return QPointF{pin.x(), pin.y() - clearance};
                case PinSide::Bottom:
                    return QPointF{pin.x(), pin.y() + clearance};
            }
            return pin;
        }

        auto compactPoints(QVector<QPointF> points, const QPointF& start, const QPointF& end)
            -> QVector<QPointF>
        {
            QVector<QPointF> compact;
            compact.reserve(points.size());
            for (const auto& p : points) {
                appendUnique(compact, p);
            }
            if (compact.size() == 1 && start != end) {
                compact.push_back(end);
            }
            return compact;
        }
    }

    auto offsetManhattanRoute(
        const QPointF& start,
        const QPointF& end,
        qreal dx,
        qreal dy,
        PinSide startSide,
        PinSide endSide,
        qreal edgeClearance
    ) -> QVector<QPointF> {
        const QPointF startExit = outwardFromPin(start, startSide, edgeClearance);
        const QPointF endApproach = outwardFromPin(end, endSide, edgeClearance);

        // Separate parallel horizontal trunks (dy) and vertical trunks (dx).
        const qreal trunkY = startExit.y() + dy;
        const qreal trunkX = endApproach.x() + dx;

        QVector<QPointF> points;
        points.reserve(10);
        appendUnique(points, start);
        appendUnique(points, startExit);

        if (std::abs(trunkY - startExit.y()) > kEps) {
            appendUnique(points, QPointF{startExit.x(), trunkY});
        }
        appendUnique(points, QPointF{trunkX, trunkY});
        if (std::abs(trunkX - endApproach.x()) > kEps
            || std::abs(trunkY - endApproach.y()) > kEps) {
            appendUnique(points, QPointF{trunkX, endApproach.y()});
        }
        appendUnique(points, endApproach);
        appendUnique(points, end);

        return compactPoints(std::move(points), start, end);
    }

    void nudgePathOffTopdieEdges(
        QVector<QPointF>& points,
        const QVector<QRectF>& topdieRects,
        qreal edgeClearance
    ) {
        if (points.size() < 3 || topdieRects.isEmpty()) {
            return;
        }

        const QPointF start = points.front();
        const QPointF end = points.back();

        for (int i = 1; i + 1 < points.size(); ++i) {
            QPointF& p = points[i];
            for (const QRectF& rect : topdieRects) {
                const bool yOverlaps = p.y() >= rect.top() - kEps && p.y() <= rect.bottom() + kEps;
                const bool xOverlaps = p.x() >= rect.left() - kEps && p.x() <= rect.right() + kEps;

                if (yOverlaps && std::abs(p.x() - rect.left()) <= kEps) {
                    p.setX(rect.left() - edgeClearance);
                } else if (yOverlaps && std::abs(p.x() - rect.right()) <= kEps) {
                    p.setX(rect.right() + edgeClearance);
                }

                if (xOverlaps && std::abs(p.y() - rect.top()) <= kEps) {
                    p.setY(rect.top() - edgeClearance);
                } else if (xOverlaps && std::abs(p.y() - rect.bottom()) <= kEps) {
                    p.setY(rect.bottom() + edgeClearance);
                }
            }
        }

        for (int i = 1; i + 1 < points.size(); ++i) {
            QPointF& a = points[i];
            QPointF& b = points[i + 1];
            if (std::abs(a.y() - b.y()) > kEps && std::abs(a.x() - b.x()) <= edgeClearance) {
                const qreal sharedX = (std::abs(a.x() - start.x()) >= std::abs(b.x() - start.x()))
                    ? a.x()
                    : b.x();
                a.setX(sharedX);
                b.setX(sharedX);
            }
            if (std::abs(a.x() - b.x()) > kEps && std::abs(a.y() - b.y()) <= edgeClearance) {
                const qreal sharedY = (std::abs(a.y() - start.y()) >= std::abs(b.y() - start.y()))
                    ? a.y()
                    : b.y();
                a.setY(sharedY);
                b.setY(sharedY);
            }
        }

        points.front() = start;
        points.back() = end;
        points = compactPoints(points, start, end);
    }

}
