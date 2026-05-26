## 0524

测试数据：case5

测试目的：检测第三版方法能否利用MCF正确求解多扇出的net。同时加入了MCF建模的规模统计数据，检测建模是否过大

测试方法：使用case5跑一遍，加入参数 --enable-mcf-routing, 但是不加参数 --enable-mcf-obj

## 测试结果

1. 图规模分析

    [2026-05-24 15:02:34]  INFO   > BusMCF model graph: nodes=30338 arcs=165892 (physical_arcs=165888) commodities=112  // 数量符合预期，但是还是太大，能否换成无向图？
    [2026-05-24 15:02:35]  INFO   > BusMCF constraint "bus_equal_length": 88 HiGHS row(s)           // 这个不对，原因是当前代码在识别同步线的时候把bumptobumpnet(group -1)也算进去了
    [2026-05-24 15:02:35]  INFO   > BusMCF constraint "edge_capacity": 82944 HiGHS row(s)           // 这个数量是对的
    [2026-05-24 15:02:35]  INFO   > BusMCF constraint "f_le_o_link": 212352 HiGHS row(s)
    [2026-05-24 15:02:35]  INFO   > BusMCF constraint "flow_conservation": 212352 HiGHS row(s)
    [2026-05-24 15:02:35]  INFO   > BusMCF constraint "node_capacity": 30336 HiGHS row(s)
    [2026-05-24 15:02:35]  INFO   > BusMCF constraint rows total: 538072
    [2026-05-24 15:02:35]  INFO   > BusMCF variables: f=1161216 o=212352 cols=1373568 rows=538072

    
