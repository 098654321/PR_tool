#include "./view2dwidget.h"
#include "./view2dview.h"
#include "./view2dscene.hh"

#include <hardware/interposer.hh>

#include <QGridLayout>
#include <QLabel>

namespace PR_tool::widget {

    View2DWidget::View2DWidget(
        hardware::Interposer* interposer, 
        circuit::BaseDie* basedie,
        QWidget* parent
    ) :
        QWidget{parent},
        _interposer{interposer},
        _basedie{basedie}
    {
        this->_scene = new View2DScene {this->_interposer, this->_basedie};
        this->_view = new View2DView {this};
        this->_view->setScene(this->_scene);
        this->_view->adjustSceneRect();

        // U22: sparse corner legend (Fusion-plain QLabel, not a card)
        auto legend = new QLabel{this};
        legend->setTextFormat(Qt::RichText);
        legend->setText(QStringLiteral(
            "<span style='color:#CDAD00'>■</span> COB<br>"
            "<span style='color:#548B54'>■</span> TOB<br>"
            "<span style='color:#000000'>■</span> Track/Net"));
        legend->setStyleSheet(QStringLiteral(
            "QLabel { color: #333333; background: transparent; padding: 9px;"
            " font-size: 150%; line-height: 1.45; }"));
        legend->setAttribute(Qt::WA_TransparentForMouseEvents);
        legend->setAccessibleName(QStringLiteral("View 2D color legend"));
        legend->setWordWrap(false);

        // V2-1: thin bottom-right hint (not a full inspector)
        const auto cobCount = this->_interposer != nullptr
            ? static_cast<int>(this->_interposer->cobs().size())
            : 0;
        auto hint = new QLabel{this};
        hint->setText(QStringLiteral("Double-click a COB for details  ·  %1 COBs")
                          .arg(cobCount));
        hint->setStyleSheet(QStringLiteral(
            "QLabel { color: #555555; background: transparent; padding: 6px; }"));
        hint->setAttribute(Qt::WA_TransparentForMouseEvents);
        hint->setAccessibleName(QStringLiteral("View 2D interaction hint"));

        auto layout = new QGridLayout{this};
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(this->_view, 0, 0);
        layout->addWidget(legend, 0, 0, Qt::AlignTop | Qt::AlignLeft);
        layout->addWidget(hint, 0, 0, Qt::AlignBottom | Qt::AlignRight);
    }

    void View2DWidget::reload() {
        this->_scene->reloadItems();
        this->_view->adjustSceneRect();
    }

    auto View2DWidget::graphicsView() const -> GraphicsView* {
        return this->_view;
    }

}