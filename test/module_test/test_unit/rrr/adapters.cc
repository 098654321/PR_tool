#include "test_support.hh"
#include <algo/router/rrr/net_adapter.hh>
#include <algo/router/rrr/route_validate.hh>
#include <algo/router/rrr/rrr_routing.hh>
#include <circuit/net/nets.hh>
#include <stdexcept>

namespace {
auto require(bool ok,const char* why)->void{if(!ok)throw std::runtime_error(why);}
}
void test_rrr_adapters() {
    using namespace PR_tool;
    using namespace PR_tool::rrr;
    hardware::Interposer i;
    const std::HashSet<int> modes{0};
    std::string name="mixed",uid="mixed",bbname="bb",bbuid="bb",btname="bt",btuid="bt";
    auto bb=std::make_shared<circuit::BumpToBumpNet>(i.get_bump(0,0,0).value(),i.get_bump(0,1,0).value(),modes,bbname,bbuid);
    auto bt=std::make_shared<circuit::BumpToTrackNet>(i.get_bump(0,2,1).value(),i.get_track(0,6,hardware::TrackDirection::Vertical,1).value(),modes,btname,btuid);
    auto sync=std::make_shared<circuit::SyncNet>(std::vector{bb},std::vector{bt},std::vector<std::Rc<circuit::TrackToBumpNet>>{},modes,name,uid);
    std::vector<std::Rc<circuit::Net>> shared{sync};
    const auto nets=build_routing_nets(shared);
    const auto owners=rrr_detail::build_owners(nets);
    require(owners.size()==2 && owners[0].is_bnet && !owners[1].is_bnet,"mixed sync lane type must survive adaptation");
    const auto graph=build_hardware_graph(&i,nets);
    RrrParams params;params.time_budget_seconds=0.5;
    const auto result=run_empty(graph,nets,params,&i);
    require(result.status=="success","mixed sync must route");
    commit_paths(&i,graph,nets,result.paths,{sync.get()});
    require(!bt->pathpackage()._tob_to_track.empty() && bt->pathpackage()._track_to_tob.empty(),"BumpToTrack direction must survive commit");
    require(bb->length()==bt->length(),"mixed SyncNet equal length");
    require(import_paths(&i,graph,nets,{sync.get()})==result.paths,"mixed sync warm-start roundtrip");
    sync->clear_path();i.reset_regs();
    for(int kind=0;kind<2;++kind) {
        std::vector<std::Rc<circuit::Net>> fanout;
        std::string n="fanout",u="fanout";
        if(kind==0) fanout.push_back(std::make_shared<circuit::BumpToBumpsNet>(i.get_bump(0,0,4).value(),
            std::vector{i.get_bump(0,1,4).value(),i.get_bump(0,2,4).value()},modes,n,u));
        else {
            auto* root_track=i.get_track(1,0,hardware::TrackDirection::Vertical,4).value();
            auto* next_track=std::get<0>(i.adjacent_tracks(root_track).front());
            fanout.push_back(std::make_shared<circuit::BumpToTracksNet>(i.get_bump(0,0,4).value(),
                std::vector{root_track,next_track},modes,n,u));
        }
        const auto fn=build_routing_nets(fanout);const auto fg=build_hardware_graph(&i,fn);
        const auto fr=run_empty(fg,fn,params,&i);
        require(fr.status=="success","multi-bump-source net must route");
        commit_paths(&i,fg,fn,fr.paths,{fanout[0].get()});
        require(import_paths(&i,fg,fn,{fanout[0].get()})==fr.paths,"fanout roundtrip");
        fanout[0]->clear_path();i.reset_regs();
    }
}
