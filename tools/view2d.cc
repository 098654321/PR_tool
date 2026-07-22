#include <hardware/interposer.hh>
#include <circuit/basedie.hh>

#include <parse/reader/module.hh>

#include <algo/netbuilder/netbuilder.hh>
#include <algo/router/route_nets.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>

#include <widget/view2d/view2dview.h>

#include <std/utility.hh>
#include <std/range.hh>
#include <std/string.hh>
#include <std/algorithm.hh>
#include <debug/debug.hh>

#include <QApplication>

auto main(int argc, char** argv) -> int {
    if (argc < 2) {
        PR_tool::debug::error_fmt("No <config_path> given");
        PR_tool::debug::info_fmt("Usage: view2d <config_path>");
        return 0;
    }

    PR_tool::debug::set_debug_level(PR_tool::debug::DebugLevel::Debug);

    auto config_path = std::StringView{argv[1]};
    auto [interposer, basedie, register_map] = PR_tool::parse::read_config(config_path, 0, false);
    (void)register_map;

    PR_tool::algo::NetBuilder{basedie.get(), interposer.get()}.build();
    auto result = PR_tool::algo::route_nets(
        interposer.get(), basedie.get(), PR_tool::algo::MazeRouteStrategy{},
        PR_tool::algo::HK{}, 0, false, false
    );
    PR_tool::debug::info_fmt("Length: '{}'", result.data._total_length);

    interposer->randomly_map_remain_indexes();

    auto app = QApplication{argc, argv};
    app.setStyle("Fusion");
    auto w = PR_tool::widget::View2DView{interposer.get(), basedie.get()};
    w.show();
    w.displayRoutingResult();
    return app.exec();
}