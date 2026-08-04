#pragma once

#include <QWidget>
#include <QSplitter>

namespace PR_tool::hardware {
    class Interposer;
};

namespace PR_tool::circuit {
    class BaseDie;
};

namespace PR_tool::widget {

    class LayoutScene;
    class LayoutView;
    class LayoutInfoWidget;
    class GraphicsView;

    class LayoutWidget : public QWidget {
        Q_OBJECT

    public:
        explicit LayoutWidget(
            hardware::Interposer* interposer, 
            circuit::BaseDie* basedie,
            QWidget *parent = nullptr
        );

    public:
        void reload();
        auto graphicsView() const -> GraphicsView*;

    private:
        QSplitter* _splitter {nullptr};

        LayoutScene* _scene {nullptr};
        LayoutView*  _view {nullptr};  
        LayoutInfoWidget* _infoWidget {nullptr};

        hardware::Interposer* _interposer {nullptr};
        circuit::BaseDie*     _basedie {nullptr};
    };

}