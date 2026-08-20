#include "./layoutwidget.h"
#include "./layoutscene.h"
#include "./layoutview.h"
#include "./layoutinfowidget.h"
#include "../chrometokens.h"

#include <QSplitter>
#include <QVBoxLayout>
#include <QPalette>
#include <QDebug>
#include <QLabel>
#include <QShortcut>

namespace PR_tool::widget {

    namespace {
        void fillChrome(QWidget* w, const char* hex) {
            if (w == nullptr) {
                return;
            }
            w->setAttribute(Qt::WA_StyledBackground, true);
            w->setAutoFillBackground(true);
            QPalette pal = w->palette();
            pal.setColor(QPalette::Window, ChromeTokens::color(hex));
            pal.setColor(QPalette::Base, ChromeTokens::color(hex));
            pal.setColor(QPalette::AlternateBase, ChromeTokens::color(hex));
            w->setPalette(pal);
        }
    }

    using namespace layout;

    LayoutWidget::LayoutWidget(hardware::Interposer* interposer, circuit::BaseDie* basedie, QWidget* parent):
        QWidget{parent},
        _interposer{interposer},
        _basedie{basedie}
    {
        this->_scene = new LayoutScene {this->_interposer, this->_basedie, this};
        // MARK
        this->_scene->setItemIndexMethod(QGraphicsScene::NoIndex);

        this->setObjectName(QStringLiteral("ChromePage"));
        fillChrome(this, ChromeTokens::bg);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(8, 8, 8, 8);

        this->_lockBanner = new QLabel{
            QStringLiteral("This view is locked after routing. Use Edit Design to edit."),
            this};
        this->_lockBanner->setObjectName(QStringLiteral("LookbackLockBanner"));
        this->_lockBanner->setWordWrap(true);
        this->_lockBanner->hide();
        layout->addWidget(this->_lockBanner);

        this->_splitter = new QSplitter{Qt::Horizontal, this};
        this->_splitter->setHandleWidth(1);
        layout->addWidget(this->_splitter);

        // View
        this->_view = new LayoutView {this->_splitter};
        this->_view->setScene(this->_scene);
        this->_view->adjustSceneRect();
        this->_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        this->_splitter->addWidget(this->_view);

        // Info
        this->_infoWidget = new LayoutInfoWidget {this->_interposer, this->_basedie, this->_scene, this->_splitter};
        this->_infoWidget->setObjectName(QStringLiteral("SidePanel"));
        fillChrome(this->_infoWidget, ChromeTokens::panel);
        this->_infoWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        this->_infoWidget->setMinimumWidth(300);
        if (auto* infoLayout = this->_infoWidget->layout()) {
            infoLayout->setContentsMargins(8, 8, 8, 8);
            infoLayout->setSpacing(8);
        }
        this->_splitter->addWidget(this->_infoWidget);

        connect(this->_scene, &LayoutScene::layoutChanged, [this]() {
                this->_scene->clearTopDieHighlight();
                this->_scene->choiseSourcePort();
                this->_infoWidget->updateInfo();
                emit this->layoutChanged();
            }
        );

        this->setWindowTitle("Layout Editor");
        this->resize(1000, 800);
    }

    void LayoutWidget::reload() {
        this->_scene->reloadItems();
        this->_view->adjustSceneRect();
        this->_infoWidget->updateInfo();
    }

    auto LayoutWidget::graphicsView() const -> GraphicsView* {
        return this->_view;
    }

    void LayoutWidget::setInspectorVisible(bool visible) {
        if (this->_infoWidget != nullptr) {
            this->_infoWidget->setVisible(visible);
        }
    }

    auto LayoutWidget::isInspectorVisible() const -> bool {
        return this->_infoWidget != nullptr && this->_infoWidget->isVisible();
    }

    void LayoutWidget::setLookbackLocked(bool locked) {
        if (this->_lockBanner != nullptr) {
            this->_lockBanner->setVisible(locked);
        }
        if (this->_infoWidget != nullptr) {
            this->_infoWidget->setEnabled(!locked);
        }
        if (this->_view != nullptr) {
            this->_view->setLookbackLocked(locked);
            for (auto* shortcut : this->_view->findChildren<QShortcut*>()) {
                shortcut->setEnabled(!locked);
            }
        }
    }

    void LayoutWidget::setLockBannerText(const QString& text) {
        if (this->_lockBanner != nullptr) {
            this->_lockBanner->setText(text);
        }
    }

}