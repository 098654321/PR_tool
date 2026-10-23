#include <algo/router/rrr/signal_owners.hh>
#include "controlbits_check.hh"
#include <algo/router/rrr/route_validate.hh>
#include <circuit/net/types/btnet.hh>
#include <circuit/net/types/syncnet.hh>
#include <fstream>
#include <sstream>
#include <queue>
#include <format>

namespace PR_tool::test {
namespace {
using namespace rrr;
using Words = std::map<std::string,std::uint32_t>;
using Edges = std::set<std::pair<int,int>>;
auto require(bool ok, const std::string& why) -> void {
    if (!ok) throw std::runtime_error("controlbits: "+why);
}
auto read_words(const parse::RegisterMapConfig& map, const std::filesystem::path& dir, bool sparse) -> Words {
    Words words;
    for (const auto& [file,addresses] : map) {
        std::ifstream input(dir/file);
        require(input.is_open(),"missing file "+file);
        std::string line;
        while (std::getline(input,line)) {
            std::istringstream row(line);
            std::string hex,address,name,extra;
            require(bool(row>>hex>>address>>name) && !(row>>extra),"malformed row");
            require(addresses.contains(name) && addresses.at(name)==address,"address/name/partition mismatch: "+name);
            require(hex.size()==8 && hex.find_first_not_of("0123456789abcdefABCDEF")==std::string::npos,"invalid word");
            require(words.emplace(name,std::stoul(hex,nullptr,16)).second,"duplicate register "+name);
        }
        for (const auto& [name,address] : addresses) if (!words.contains(name)) {
            require(sparse,"missing full register "+name);
            // Hardware reset values; do not call Writer's default-value helper.
            const bool ff = name.starts_with("tob_") &&
                (name.find("_track2tob_")!=std::string::npos || name.find("_tob2bump_bank")!=std::string::npos);
            words[name]=ff ? 0xffffffffu : 0u;
        }
    }
    return words;
}
auto bit(const Words& words, const std::string& prefix, std::size_t index) -> bool {
    return (words.at(prefix+"_"+std::to_string(index/32))>>(index%32))&1u;
}
auto bump_bit(const Words& words, const std::string& prefix, std::size_t index) -> bool {
    return (words.at(std::format("{}_bank{}_en_{}",prefix,index/64,(index%64)/32))>>(index%32))&1u;
}
auto mux(const Words& words, const std::string& prefix, std::size_t index) -> unsigned {
    const auto word=words.at(std::format("{}_bank{}_{}",prefix,index/64,(index%64)/8));
    require((word>>24)==0,"nonzero mux padding");
    return (word>>(3*(index%8)))&7u;
}
auto track_id(const UnifiedGraph& graph, hardware::TrackCoord c) -> int {
    GraphNodeRef ref; ref.kind=GraphNodeRef::Kind::Track;ref.track_coord=c;ref.track_index=c.index;
    const int n=resolve_graph_node(graph,ref);require(n>=0,"unresolved physical track");return n;
}
auto decode(const Words& words, hardware::Interposer* interposer, const UnifiedGraph& graph) -> Edges {
    Edges edges;
    using D=hardware::COBDirection;
    const std::vector<std::tuple<D,D,std::string,std::string,std::string>> pairs{
        {D::Right,D::Up,"sw_ru","right_sel","up_sel"},{D::Left,D::Up,"sw_lu","left_sel","up_sel"},
        {D::Right,D::Down,"sw_rd","right_sel","down_sel"},{D::Left,D::Down,"sw_ld","left_sel","down_sel"},
        {D::Right,D::Left,"sw_h","right_sel","left_sel"},{D::Up,D::Down,"sw_v","up_sel","down_sel"}};
    for (std::size_t r=0;r<hardware::Interposer::COB_ARRAY_HEIGHT;++r)
        for (std::size_t c=0;c<hardware::Interposer::COB_ARRAY_WIDTH;++c) {
            auto* cob=interposer->get_cob({static_cast<std::i64>(r),static_cast<std::i64>(c)}).value();
            const auto prefix=std::format("cob_{}_{}_",r,c);
            if (!words.contains(prefix+"sw_ru_0")) continue;
            for (const auto& [a,b,sw,asel,bsel]:pairs) for (std::size_t i=0;i<128;++i) {
                if (!bit(words,prefix+sw,i)) continue;
                const auto j=hardware::COB::cob_index_map(a,i,b);
                const bool ao=bit(words,prefix+asel,i),bo=bit(words,prefix+bsel,j);
                require(ao!=bo,std::format("connected COB switch with incompatible directions {}{} i={} j={} a={} b={}",prefix,sw,i,j,ao,bo));
                const int u=track_id(graph,cob->to_dir_track_coord(a,hardware::COB::cob_index_to_track_index(i)));
                const int v=track_id(graph,cob->to_dir_track_coord(b,hardware::COB::cob_index_to_track_index(j)));
                edges.emplace(ao?v:u,ao?u:v);
            }
        }
    for (const auto& arc:graph.arcs) {
        const auto& u=graph.nodes[arc.u]; const auto& v=graph.nodes[arc.v];
        if (u.kind==UnifiedNodeKind::Track && v.kind==UnifiedNodeKind::Track) continue;
        const auto& n=u.kind==UnifiedNodeKind::Track?v:u;
        const auto prefix=std::format("tob_{}_{}_",n.tob/hardware::Interposer::TOB_ARRAY_WIDTH,
                                     n.tob%hardware::Interposer::TOB_ARRAY_WIDTH);
        const auto is=[&](UnifiedNodeKind a,UnifiedNodeKind b) {return u.kind==a && v.kind==b;};
        bool enabled=false;
        if (is(UnifiedNodeKind::Bump,UnifiedNodeKind::HLine) || is(UnifiedNodeKind::HLine,UnifiedNodeKind::Bump)) {
            const bool out=u.kind==UnifiedNodeKind::Bump;
            const auto& b=out?u:v;const auto& h=out?v:u;
            const auto index=b.bump.Bank*64+b.bump.Group*8+b.bump.Index;
            const bool drive=bump_bit(words,prefix+"bump2tob",index);
            const bool receive=bump_bit(words,prefix+"tob2bump",index);
            require(!(drive&&receive),"bidirectional bump enable");
            enabled=mux(words,prefix+"hctrl",index)==h.line_index && (out?drive:receive);
        } else if (is(UnifiedNodeKind::HLine,UnifiedNodeKind::VLine) || is(UnifiedNodeKind::VLine,UnifiedNodeKind::HLine)) {
            const auto& h=u.kind==UnifiedNodeKind::HLine?u:v;
            const auto& vl=u.kind==UnifiedNodeKind::VLine?u:v;
            enabled=mux(words,prefix+"vctrl",h.bank*64+h.line_index*8+h.group)==vl.line_index%8;
        } else {
            const bool out=v.kind==UnifiedNodeKind::Track;
            const auto& vl=out?u:v; const auto& t=out?v:u;
            const bool drive=bit(words,prefix+"tob2track",t.track_index);
            const bool receive=bit(words,prefix+"track2tob",t.track_index);
            require(!(drive&&receive),"bidirectional TOB track enable");
            const bool swap=bit(words,prefix+"bank_sel",vl.line_index%64);
            enabled=(t.track_index/64==((vl.line_index/64)^swap)) && (out?drive:receive);
        }
        if (enabled) edges.emplace(arc.u,arc.v);
    }
    return edges;
}
auto reversed(circuit::Net* raw,std::size_t demand) -> bool {
    if (dynamic_cast<circuit::BumpToTrackNet*>(raw)) return true;
    if (auto* sync=dynamic_cast<circuit::SyncNet*>(raw))
        return demand>=sync->btbnets().size() && demand<sync->btbnets().size()+sync->bttnets().size();
    return false;
}
}

auto check_routed_controlbits(hardware::Interposer* interposer,
    const rrr::UnifiedGraph& graph, const std::vector<rrr::RoutingNet>& nets,
    const std::vector<circuit::Net*>& raw, const parse::RegisterMapConfig& map,
    const std::filesystem::path& directory, bool sparse, const rrr::NetPaths* expected) -> void {
    const auto words=read_words(map,directory,sparse);
    const auto edges=decode(words,interposer,graph);
    std::vector<std::vector<int>> adjacent(graph.nodes.size()),undirected(graph.nodes.size());
    for (auto [u,v]:edges) {adjacent[u].push_back(v);undirected[u].push_back(v);undirected[v].push_back(u);}
    RrrResult decoded;decoded.paths.resize(nets.size());
    std::map<int,OwnerId> terminals;
    const auto aliases=signal_owners(graph,nets);
    Edges expected_cob,actual_cob;
    for (const auto& e:edges) if (graph.nodes[e.first].kind==UnifiedNodeKind::Track && graph.nodes[e.second].kind==UnifiedNodeKind::Track)
        actual_cob.insert(e);
    for (std::size_t ni=0;ni<nets.size();++ni) {
        const auto& net=nets[ni];
        for (std::size_t di=0;di<net.demands.size();++di) {
            const auto& d=net.demands[di];
            const auto raw_owner=demand_owner(net,d);
            const auto owner=aliases.contains(raw_owner)?aliases.at(raw_owner):raw_owner;
            std::set<int> sources;
            for (auto si:d.candidate_source_indices) sources.insert(resolve_graph_node(graph,net.sources.at(si)));
            const int sink=resolve_graph_node(graph,d.sink);
            // Multi-source fixed rails may have unused source alternatives; assign only actual path endpoints below.
            const bool reverse=reversed(raw.at(ni),di);
            std::vector<int> previous(graph.nodes.size(),-2);std::queue<int> queue;
            if (reverse) {previous[sink]=-1;queue.push(sink);}
            else for (int s:sources) {previous[s]=-1;queue.push(s);}
            int target=-1;
            while (!queue.empty()) {
                const int n=queue.front();queue.pop();
                if (reverse?sources.contains(n):n==sink) {target=n;break;}
                if (graph.nodes[n].kind==UnifiedNodeKind::Bump && previous[n]!=-1) continue;
                for (int v:adjacent[n]) if (previous[v]==-2) {previous[v]=n;queue.push(v);}
            }
            require(target>=0,"unreachable demand "+net.origin_uid+"/"+std::to_string(di));
            std::vector<int> path;
            for (int n=target;n>=0;n=previous[n]) path.push_back(n);
            if (!reverse) std::reverse(path.begin(),path.end());
            for (int terminal:{path.front(),path.back()}) {
                auto [it,inserted]=terminals.emplace(terminal,owner);
                require(inserted || it->second==owner,"shared terminal across independent nets");
            }
            decoded.paths[ni].push_back(path);
            if (expected) {
                require(path==expected->at(ni).at(di),"decoded path differs from saved router path");
                auto physical=path;if(reverse)std::reverse(physical.begin(),physical.end());
                for (std::size_t i=1;i<physical.size();++i)
                    if(graph.nodes[physical[i-1]].kind==UnifiedNodeKind::Track && graph.nodes[physical[i]].kind==UnifiedNodeKind::Track)
                        expected_cob.emplace(physical[i-1],physical[i]);
            }
        }
    }
    require(validate_rrr_solution(graph,nets,decoded,interposer),"decoded routing violates physical constraints");
    if(expected) require(expected_cob==actual_cob,"extra or missing active COB switches");
    // Electrical shorts are checked independently of chosen BFS paths.
    std::set<int> visited;
    for (const auto& [start,owner]:terminals) {
        if(visited.contains(start))continue;
        std::queue<int> q;q.push(start);visited.insert(start);
        while(!q.empty()) {
            const int n=q.front();q.pop();
            auto it=terminals.find(n);
            require(it==terminals.end() || it->second==owner,"short between independent nets/lanes");
            for(int v:undirected[n])if(visited.insert(v).second)q.push(v);
        }
    }
}
}
