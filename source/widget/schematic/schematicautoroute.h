#pragma once

#include "./item/pinitem.h"

#include <QPointF>
#include <QRectF>
#include <QVector>

namespace PR_tool::widget::schematic {

    /// Orthogonal display path with dx/dy trunk separation. May cross topdies.
    /// Pin sides add a short outward stub so trunks do not hug instance borders.
    auto offsetManhattanRoute(
        const QPointF& start,
        const QPointF& end,
        qreal dx,
        qreal dy,
        PinSide startSide,
        PinSide endSide,
        qreal edgeClearance
    ) -> QVector<QPointF>;

    /// Push intermediate vertical/horizontal trunks off topdie borders.
    void nudgePathOffTopdieEdges(
        QVector<QPointF>& points,
        const QVector<QRectF>& topdieRects,
        qreal edgeClearance
    );

}
