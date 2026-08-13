#pragma once

#include <QWidget>
#include <hardware/track/trackcoord.hh>

class QSpinBox;
class QLineEdit;
class QComboBox;
class QPushButton;

namespace PR_tool::widget::schematic {

    class ExternalPortItem;

    class ExternalPortInfoWidget : public QWidget {
        Q_OBJECT

    public:
        ExternalPortInfoWidget(QWidget* parent = nullptr);

    signals:
        void externalPortRename(ExternalPortItem* eport, const QString& name);
        void externalPortSetCoord(ExternalPortItem* eport, const hardware::TrackCoord& coord);
        void removeExternalPort(ExternalPortItem* eport);

    public:
        void loadExternalPort(ExternalPortItem* eport);
        auto currentExternalPort() -> ExternalPortItem*;

    protected:
        ExternalPortItem* _externalPort {nullptr};

        QLineEdit* _nameEdit {nullptr};
        QSpinBox* _rowSpinBox {nullptr};
        QSpinBox* _colSpinBox {nullptr};
        QComboBox* _dirComboBox {nullptr};
        QSpinBox* _indexSpinBox {nullptr};
        QPushButton* _setCoordButton {nullptr};
    };

}
