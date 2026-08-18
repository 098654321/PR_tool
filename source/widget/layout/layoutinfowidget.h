#pragma once

#include <QWidget>

class QTableView;
class QLabel;

namespace PR_tool::hardware {
    class Interposer;
};

namespace PR_tool::circuit {
    class BaseDie;
};

namespace PR_tool::widget {

    class LayoutScene;

    class LayoutInfoWidget : public QWidget {
    public:
        LayoutInfoWidget(
            hardware::Interposer* interposer, circuit::BaseDie* basedie, 
            LayoutScene* scene,
            QWidget* parent
        );

        void updateInfo();

    private:
        hardware::Interposer* _interposer {nullptr};
        circuit::BaseDie*     _basedie {nullptr};
        LayoutScene* _scene {nullptr};
    
        QTableView* _instPlaceView {nullptr};
        QLabel* _estimatedLengthLabel {nullptr};
        QLabel* _estimatedLengthValue {nullptr};
    };

}
