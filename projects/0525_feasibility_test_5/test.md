## 0524

### test 1

测试数据：case5

测试目的：检测第三版方法同步线长约束的数学形式修改之后，能否正常发挥作用；检测同步线和非同步线的等长约束是否调整正确

测试方法：使用case5跑一遍，加入参数 --enable-mcf-routing, 但是不加参数 --enable-mcf-obj。检测同步线长度（看路径组成信息） & 同步线等长约束的数量应该是64个

### 测试结果

1. BusMCF分析

    [2026-05-25 11:23:48]  INFO   > BusMCF model graph: nodes=30338 arcs=165892 (physical_arcs=165888) commodities=80
    [2026-05-25 11:23:49]  INFO   > BusMCF constraint "bus_equal_length": 64 HiGHS row(s)       // 这个数量对了
    [2026-05-25 11:23:49]  INFO   > BusMCF constraint "edge_capacity": 82944 HiGHS row(s)       // 这个数量对
    [2026-05-25 11:23:49]  INFO   > BusMCF constraint "f_le_o_link": 151680 HiGHS row(s)
    [2026-05-25 11:23:49]  INFO   > BusMCF constraint "flow_conservation": 151680 HiGHS row(s)  
    [2026-05-25 11:23:49]  INFO   > BusMCF constraint "node_capacity": 30336 HiGHS row(s)       // 这个数量对（比整个的去除掉virtual_P/Nnode）
    [2026-05-25 11:23:49]  INFO   > BusMCF constraint rows total: 416704
    [2026-05-25 11:23:49]  INFO   > BusMCF variables: f=829440 o=151680 cols=981120 rows=416704

    图的规模太大的，约束条件和变量太多。如果换成无向图，节点使用约束(f_le_o_link)可以减少一点，因为不需要再份f和o的不等式关系以及o求和的不等式关系，直接使用节点领域的所有net度数总和就可以。

2. tracktoBumpsNet路径分析

    目前的路径很长（见debug.log），有可能是没有加目标函数，也有可能是一开始求解的范围不合理

3. SimpleMCF分析

    边容量约束条件的个数不对，和BusMCF当中所有unit一起求解的个数一样，说明没有单独考虑一个unit

    
