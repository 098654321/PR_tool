#include "./gui.hh"
#include <widget/chrometokens.h>
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
#include <QDebug>
#include <QDialog>
#include <QFile>
#include <QPalette>

namespace PR_tool {

    namespace {

        void applyChromeTheme(QApplication& app) {
            using widget::ChromeTokens;
            auto pal = app.palette();
            pal.setColor(QPalette::Window, ChromeTokens::color(ChromeTokens::bg));
            pal.setColor(QPalette::Base, ChromeTokens::color(ChromeTokens::surface));
            pal.setColor(QPalette::AlternateBase, ChromeTokens::color(ChromeTokens::panel));
            pal.setColor(QPalette::Button, ChromeTokens::color(ChromeTokens::surface));
            pal.setColor(QPalette::ButtonText, ChromeTokens::color(ChromeTokens::text));
            pal.setColor(QPalette::Text, ChromeTokens::color(ChromeTokens::text));
            pal.setColor(QPalette::WindowText, ChromeTokens::color(ChromeTokens::text));
            pal.setColor(QPalette::Highlight, ChromeTokens::color(ChromeTokens::selectionFill));
            pal.setColor(QPalette::HighlightedText, ChromeTokens::color(ChromeTokens::text));
            pal.setColor(QPalette::PlaceholderText, ChromeTokens::color(ChromeTokens::textMuted));
            pal.setColor(QPalette::ToolTipBase, ChromeTokens::color(ChromeTokens::surface));
            pal.setColor(QPalette::ToolTipText, ChromeTokens::color(ChromeTokens::text));
            app.setPalette(pal);

            QFile qss{QStringLiteral(":/qss/qss/app.qss")};
            if (!qss.open(QIODevice::ReadOnly | QIODevice::Text)) {
                qWarning("Failed to load chrome QSS :/qss/qss/app.qss");
            } else {
                app.setStyleSheet(
                    ChromeTokens::applyToQss(QString::fromUtf8(qss.readAll())));
            }
        }

    } // namespace

    auto gui_main(int argc, char** argv) -> int {
        auto app = QApplication{argc, argv};
        app.setStyle("Fusion");
        applyChromeTheme(app);

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
