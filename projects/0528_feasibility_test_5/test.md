## 简记

在修改的过程中发现，代码没有正确处理一条net的原始整体内容和ILP拆分出来子net。

修复采用的思路是给每一个net添加一个uid，在构建net的时候就加上这个全局唯一的标记

修复结束之后发现case5多多扇出net有时候可以布线成功。同时如果把bus线去掉，直接跑SimpleMCF也可以稳定成功，说明不是mcf建模本身的问题，是没有处理好ILP拆分的子net与原始net的对应关系

ILP可以通过扩大搜索区域获得更多可能可行的track，但是如何选择？如何考虑routability&wirelength？如何在ILP阶段就能考虑到MCF的congestion？

## todo

1. 查一下MCF求解的失败原因具体是什么？“HiGHS MIP 支持 IIS（不可行子系统），可定位冲突约束行；实现成本较高，适合专门 debug 开关。”

