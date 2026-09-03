#include "./layoutview.h"
#include "../chrometokens.h"

#include <QPalette>

namespace PR_tool::widget {

    LayoutView::LayoutView(
        QWidget *parent
    ) :
        GraphicsView{parent}
    {
        this->setObjectName(QStringLiteral("CanvasWorkSurface"));
        this->setAttribute(Qt::WA_StyledBackground, true);
        this->setAutoFillBackground(true);
        this->setFrameShape(QFrame::NoFrame);
        this->setBackgroundBrush(ChromeTokens::color(ChromeTokens::surface));
        {
            QPalette pal = this->palette();
            pal.setColor(QPalette::Window, ChromeTokens::color(ChromeTokens::surface));
            pal.setColor(QPalette::Base, ChromeTokens::color(ChromeTokens::surface));
            this->setPalette(pal);
        }
        if (QWidget* vp = this->viewport()) {
            vp->setAutoFillBackground(true);
            QPalette pal = vp->palette();
            pal.setColor(QPalette::Window, ChromeTokens::color(ChromeTokens::surface));
            pal.setColor(QPalette::Base, ChromeTokens::color(ChromeTokens::surface));
            vp->setPalette(pal);
        }
        this->setDragMode(QGraphicsView::RubberBandDrag);
        this->setInteractive(true);
        this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    }

    LayoutView::~LayoutView() noexcept {}

}