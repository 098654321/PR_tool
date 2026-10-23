#pragma once
#include "hardware_graph.hh"

namespace PR_tool::rrr {
// A Track touches two COBs. Direction selects belong to a COB endpoint, not
// the whole Track. Passing through two switches of the same COB would assign
// both directions to one endpoint and cannot be encoded in controlbits.
inline auto edge_cob(const UnifiedGraph& graph,int a,int b) -> std::optional<std::pair<int,int>> {
    if (graph.rows<=0 || graph.cols<=0) return {}; // Synthetic graphs have no physical COB topology.
    const auto& u=graph.nodes.at(a);const auto& v=graph.nodes.at(b);
    if(u.kind!=UnifiedNodeKind::Track || v.kind!=UnifiedNodeKind::Track)return {};
    const std::array<std::pair<int,int>,2> uc{{{u.track_row,u.track_col},
        {u.track_row-(u.track_dir==1),u.track_col-(u.track_dir==0)}}};
    const std::array<std::pair<int,int>,2> vc{{{v.track_row,v.track_col},
        {v.track_row-(v.track_dir==1),v.track_col-(v.track_dir==0)}}};
    for(auto c:uc)for(auto d:vc)if(c==d && c.first>=0 && c.first<graph.rows && c.second>=0 && c.second<graph.cols)return c;
    return {};
}
// 0: no COB arrival (source/TOB), 1/2: the two physical Track ends.
inline auto cob_arrival(const UnifiedGraph& graph,int previous,int node) -> int {
    if(previous<0)return 0;
    const auto cob=edge_cob(graph,previous,node);if(!cob)return 0;
    const auto& n=graph.nodes[node];
    return *cob==std::pair{n.track_row,n.track_col}?1:2;
}
inline auto same_cob_turn(const UnifiedGraph& graph,int previous,int node,int next) -> bool {
    if(previous<0)return false;
    const auto a=edge_cob(graph,previous,node),b=edge_cob(graph,node,next);
    return a && b && a==b;
}
inline auto cob_directions_valid(const UnifiedGraph& graph,const std::Vector<std::Vector<int>>& paths) -> bool {
    std::map<std::tuple<int,int,int>,bool> roles;
    for(const auto& path:paths)for(std::size_t i=1;i<path.size();++i) {
        const auto cob=edge_cob(graph,path[i-1],path[i]);if(!cob)continue;
        for(const auto& [node,out]:{std::pair{path[i-1],false},std::pair{path[i],true}}) {
            auto [it,inserted]=roles.emplace(std::tuple{cob->first,cob->second,node},out);
            if(!inserted && it->second!=out)return false;
        }
    }
    return true;
}
}
