#include "./gui.hh"
#include <widget/window.h>
#include <widget/frame/entrydialog.h>

#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

#include <algo/router/route_nets.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>

#include <parse/reader/module.hh>
#include <parse/writer/module.hh>

#include <std/utility.hh>
#include <std/range.hh>
#include <std/string.hh>
#include <debug/debug.hh>
#include <std/algorithm.hh>

#include <QApplication>
#include <QDialog>

namespace PR_tool {

    auto gui_main(int argc, char** argv) -> int {
        auto app = QApplication{argc, argv};
        app.setStyle("Fusion");

        auto entry = widget::EntryDialog{};
        if (entry.exec() != QDialog::Accepted) {
            return 0;
        }

        auto w = widget::Window{};
        if (auto path = entry.getResult(); path.has_value()) {
            w.loadConfigFromPath(*path);
        }
        w.show();
        return app.exec();
    }

}
