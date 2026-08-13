#include "./viewinfowidget.h"
#include "../schematicview.h"
#include <widget/frame/colorpickbutton.h>
#include "widget/schematic/item/griditem.h"
#include <debug/debug.hh>

#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QIntValidator>
#include <QFrame>

namespace PR_tool::widget::schematic {

    constexpr int MIN_HEIGHT = 30;

    ViewInfoWidget::ViewInfoWidget(SchematicView* view, QWidget* parent) :
        QWidget{parent},
        _view{view}
    {
        if (this->_view == nullptr) {
            debug::exception("Nullptr view");
        }

        auto* thisLayout = new QVBoxLayout{this};
        thisLayout->setContentsMargins(8, 12, 8, 8);

        auto* placeholder = new QLabel{QStringLiteral("Select an object to inspect"), this};
        placeholder->setWordWrap(true);
        placeholder->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        placeholder->setStyleSheet(QStringLiteral("color: #666666; font-size: 13px;"));
        thisLayout->addWidget(placeholder);

        auto* line = new QFrame{this};
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        thisLayout->addWidget(line);

        auto* widget = new QGroupBox{QStringLiteral("Canvas"), this};
        thisLayout->addWidget(widget);
        thisLayout->addStretch();

        auto* layout = new QGridLayout{widget};
        layout->setSpacing(10);

        layout->addWidget(new QLabel{QStringLiteral("Grid Visible"), widget}, 0, 0);
        auto* gridVisibleCheckBox = new QCheckBox{widget};
        gridVisibleCheckBox->setMinimumHeight(MIN_HEIGHT);
        gridVisibleCheckBox->setChecked(this->_view->gridVisible());
        layout->addWidget(gridVisibleCheckBox, 0, 1);

        layout->addWidget(new QLabel{QStringLiteral("Grid Color"), widget}, 1, 0);
        auto* gridColorButton = new ColorPickerButton{widget};
        gridColorButton->setMinimumHeight(MIN_HEIGHT);
        gridColorButton->setColor(this->_view->gridColor());
        layout->addWidget(gridColorButton, 1, 1);

        layout->addWidget(new QLabel{QStringLiteral("Grid Size"), widget}, 2, 0);
        auto* gridSizeEdit = new QLineEdit{widget};
        auto* validator = new QIntValidator(GridItem::GRID_SIZE, 100, gridSizeEdit);
        gridSizeEdit->setValidator(validator);
        gridSizeEdit->setMinimumHeight(MIN_HEIGHT);
        gridSizeEdit->setText(QStringLiteral("%1").arg(this->_view->gridSize()));
        layout->addWidget(gridSizeEdit, 2, 1);

        layout->setColumnMinimumWidth(0, 50);
        layout->setColumnStretch(0, 0);

        connect(gridVisibleCheckBox, &QCheckBox::stateChanged, [this](int state) {
            if (state == Qt::Checked) {
                this->_view->setGridVisible(true);
                this->_view->updateBack();
            } else if (state == Qt::Unchecked) {
                this->_view->setGridVisible(false);
                this->_view->updateBack();
            } else {
                debug::unreachable();
            }
        });

        connect(gridColorButton, &ColorPickerButton::colorChanged, [this](const QColor& color) {
            this->_view->setGridColor(color);
            this->_view->updateBack();
        });

        connect(gridSizeEdit, &QLineEdit::textChanged, [this](const QString& text) {
            bool ok = false;
            const auto size = text.toInt(&ok);
            if (ok) {
                this->_view->setGridSize(size);
                this->_view->updateBack();
            }
        });
    }

}
