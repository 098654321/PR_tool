#include "./route.hh"
#include <algo/router/route_nets.hh>
#include <algo/router/routeerror.hh>
#include <global/debug/debug.hh>
#include "./clear.hh"


namespace PR_tool::algo {

Route::Route() {
    this->_remediation.emplace_back(std::make_shared<Clear>());
}

auto Route::execute(hardware::Interposer* interposer, RouteEngine& engine) const -> void {
    debug::info("routing ...");

    auto nets = engine.nets();
    auto posi = engine.position();
    for (std::usize i = posi; i < nets.size(); ++i) {
        auto net = nets[i];
        net->set_reuse_type(false);

        // check existing path
        auto routed_nets = engine.routed_nets();
        net->search_related_nets(routed_nets);

        try {
            net->route(interposer, engine.routestrategy());
        }
        catch (const RetryExpt& err) {
            debug::info(err.what());
            show_retry_expt(net, engine, interposer);
            net->clear_path();
            engine.record_failed_net(net->name());
        }
        engine.move_on();
    }
}

auto Route::to_string() const -> const std::String {
    return "Route";
}

}

