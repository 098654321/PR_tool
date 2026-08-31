#pragma once

#include "hardware/cob/cobregister.hh"
#include "std/utility.hh"
#include <QDialog>

namespace PR_tool::hardware {
    class COB;
}

class QPushButton;
class QComboBox;
class QSpinBox;

namespace PR_tool::widget {

    class COBInfoDialog : public QDialog {
    public:
        /// When `allowEdit` is false (e.g. after P&R), Enable Edit / Set Value stay disabled.
        COBInfoDialog(hardware::COB* cob, bool allowEdit = true);
        ~COBInfoDialog();

    private:
        auto currentRegister() -> std::Tuple<hardware::COBSwRegister*, hardware::COBSelRegister*>;
        auto setRegister() -> void;
        auto updateRegister() -> void;
        
    private:
        hardware::COB* _cob;

        QComboBox* _fromDirection {nullptr};
        QSpinBox*  _fromTrackIndex {nullptr};
        QComboBox* _toDirection {nullptr};

        QSpinBox*  _toIndex {nullptr};
        QComboBox* _swRegister {nullptr};
        QComboBox* _selRegister {nullptr};

        QPushButton* _editorButton {nullptr};
        QPushButton* _setButton {nullptr};

    };

}