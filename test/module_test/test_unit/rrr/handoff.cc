#include <algo/router/command_mode/command/route.hh>
#include <algo/router/common/maze/mazeroutestrategy.hh>
#include <algo/router/common/allocate/hopcroft_karp.hh>
#include <algo/router/common/routing_budget.hh>
#include <algo/router/routeerror.hh>
#include <algo/router/rrr/cob_direction.hh>
#include <hardware/interposer.hh>
#include <stdexcept>

namespace {
using namespace PR_tool;
auto require(bool ok,const char* why)->void {if(!ok)throw std::runtime_error(why);}
// Inject failure after unpublished local resource claims, as multi-sink maze
// routing does. No wall-clock timing is needed to cover cancellation cleanup.
struct FailingMaze : algo::MazeRouteStrategy {
    bool timeout{false};
    mutable int calls{0};
    mutable std::optional<hardware::COBConnector> orphan;
    auto route_track_to_bump_net(hardware::Interposer* i,circuit::TrackToBumpNet* n) const -> void override {
        if(++calls!=2) {MazeRouteStrategy::route_track_to_bump_net(i,n);return;}
        auto* t=i->get_track(5,5,hardware::TrackDirection::Horizontal,0).value();
        orphan=std::get<1>(i->adjacent_tracks(t).front());
        orphan->suspend();
        if(timeout)throw algo::RoutingTimeout{};
        throw algo::RetryExpt("injected local-package failure");
    }
};
}

void test_rrr_handoff() {
    using namespace PR_tool;
    for(bool timeout:{false,true}) {
        hardware::Interposer i;
        FailingMaze strategy;strategy.timeout=timeout;
        algo::HK allocator;
        std::vector<std::Rc<circuit::Net>> nets;
        for(int n=0;n<3;++n) {
            std::string name="handoff"+std::to_string(n),uid=name;
            nets.push_back(std::make_shared<circuit::TrackToBumpNet>(
                i.get_track(0,0,hardware::TrackDirection::Vertical,n).value(),
                i.get_bump(0,n,n).value(),std::HashSet<int>{0},name,uid));
        }
        algo::RouteEngine engine({{0,nets}},strategy,allocator,0,false,false,false,&i);
        engine.stop_on_first_failure=true;
        bool cancelled=false;
        try {algo::Route{}.execute(&i,engine);} catch(const algo::RoutingTimeout&){cancelled=true;}
        require(cancelled==timeout,"only timeout propagates cancellation");
        require(strategy.calls==2 && engine.position()==1,"stop immediately at first failed net");
        require(nets[0]->length()>0 && nets[1]->length()==0 && nets[2]->length()==0,"preserve completed net only");
        require(engine.failed_net_names().size()==1,"record exactly the failed net");
        require(strategy.orphan && !strategy.orphan->is_occupied(),"unpublished claims must be released");
        for(auto [t,c]:nets[0]->pathpackage()._regular_path)
            if(c)require(c->is_occupied(),"completed net resources must be restored");
    }
    hardware::COBUnit unit;
    for(auto from:{hardware::COBDirection::Left,hardware::COBDirection::Right,
                   hardware::COBDirection::Up,hardware::COBDirection::Down})
        for(int j=0;j<8;++j)for(auto c:unit.adjacent_connectors(from,j))
            c.sw_reg->set(hardware::COBSwState::Suspended);
    unit.reset_regs();
    for(auto from:{hardware::COBDirection::Left,hardware::COBDirection::Right,
                   hardware::COBDirection::Up,hardware::COBDirection::Down})
        for(int j=0;j<8;++j)for(auto c:unit.adjacent_connectors(from,j))
            require(!c.sw_reg->is_occupied(),"reset must clear all six COB switch families");
    rrr::UnifiedGraph g;g.rows=2;g.cols=2;
    for(auto [dir,row,col]:{std::tuple{1,0,0},std::tuple{0,0,1},std::tuple{1,1,0},std::tuple{0,0,2}}) {
        rrr::UnifiedNode node;node.track_dir=dir;node.track_row=row;node.track_col=col;g.nodes.push_back(node);
    }
    require(!rrr::cob_directions_valid(g,{{0,1,2}}),"same COB endpoint cannot receive and drive");
    require(!rrr::cob_directions_valid(g,{{0,1},{1,2}}),"direction conflicts span branches");
    require(rrr::cob_directions_valid(g,{{0,1,3}}),"opposite ends of a Track have independent directions");
}
