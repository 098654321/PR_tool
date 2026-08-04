#pragma once

#include "qwidget.h"
#include <QWidget>

namespace PR_tool::hardware {
    class Interposer;
};

namespace PR_tool::circuit {
    class BaseDie;
};

namespace PR_tool::widget {

    class View2DView;
    class View2DScene;
    class GraphicsView;

    class View2DWidget : public QWidget {
    public:
        View2DWidget(
            hardware::Interposer* interposer, 
            circuit::BaseDie* basedie,
            QWidget* parent = nullptr);

    public:
        void reload();
        auto graphicsView() const -> GraphicsView*;

    protected:
        View2DScene* _scene {nullptr};
        View2DView* _view {nullptr};

        hardware::Interposer* _interposer {nullptr};
        circuit::BaseDie* _basedie {nullptr};
    };

}