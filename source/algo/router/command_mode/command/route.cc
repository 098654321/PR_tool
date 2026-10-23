#include "./route.hh"
#include <algo/router/route_nets.hh>
#include <algo/router/routeerror.hh>
#include <global/debug/debug.hh>
#include "./clear.hh"
#include <algo/router/common/routing_budget.hh>


namespace PR_tool::algo {

Route::Route() {
    this->_remediation.emplace_back(std::make_shared<Clear>());
}

auto Route::execute(hardware::Interposer* interposer, RouteEngine& engine) const -> void {
    debug::info("routing ...");

    auto nets = engine.nets();
    auto posi = engine.position();
    const auto total = nets.size();
    engine.report_progress(posi, total);
    const auto restore_completed = [&]() {
        // A multi-sink maze may have claimed a local, not-yet-published package.
        // Rebuild from completed nets to release these otherwise invisible claims.
        interposer->reset_regs();
        for (auto* completed : engine.routed_nets()) completed->pathpackage().occupy_all();
    };
    for (std::usize i = posi; i < nets.size(); ++i) {
        auto net = nets[i];
        net->set_reuse_type(false);

        // check existing path
        auto routed_nets = engine.routed_nets();
        net->search_related_nets(routed_nets);

        try {
            RoutingBudget::check();
            net->route(interposer, engine.routestrategy());
        }
        catch (const RoutingTimeout&) {
            net->clear_path();
            engine.record_failed_net(net->name());
            restore_completed();
            throw;
        }
        catch (const RetryExpt& err) {
            debug::info(err.what());
            show_retry_expt(net, engine, interposer);
            net->clear_path();
            engine.record_failed_net(net->name());
            if (engine.stop_on_first_failure) { restore_completed(); return; }
        }
        engine.move_on();
        engine.report_progress(i + 1, total);
    }
}

auto Route::to_string() const -> const std::String {
    return "Route";
}

}

