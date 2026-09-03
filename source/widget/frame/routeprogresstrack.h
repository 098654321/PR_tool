#pragma once

#include <QWidget>

namespace PR_tool::widget {

    /// Manhattan progress strip: interleaved channel-routed nets, round pins.
    class RouteProgressTrack : public QWidget {
    public:
        explicit RouteProgressTrack(QWidget* parent = nullptr);

        void setProgress(int done, int total);
        auto sizeHint() const -> QSize override;

    protected:
        void paintEvent(QPaintEvent* event) override;

    private:
        int _done {0};
        int _total {0};
    };

}
