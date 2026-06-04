## 0602

测试目的：检查有时候会失败的case5，和一直失败的case8，case13具体失败的原因是什么。并检查如果MCF没有走预分配的最短路径，会对线长产生怎样的影响

测试数据：case5，8，13

测试方法：

xmake run test_ILP ../test/config/case5 --enable-ilp-parallel --enable-mcf-routing 

修改hardware.hh的阵列规模

xmake run test_ILP ../test/config/case8 --enable-ilp-parallel --enable-mcf-routing --cob-rows 9 --cob-cols 13

xmake run test_ILP ../test/config/case13 --enable-ilp-parallel --enable-mcf-routing --cob-rows 9 --cob-cols 13

**（输出的日志察出来有一些问题，所以本次实验生成的日志数据作废）**


## 预期结果

1. case5

- 渲染资源使用量图

- 查看到底是什么约束失效了？Gurobi是如何实现这种检查的？

2. case8

和case5相同

3. case13

展示出到底哪些TOB上的哪些net分配失败了

4. 路径变化

见switch_study.log。如果没有走ILP分配TOB资源时预定的最短路径，会导致绕路，但是绕的范围不是很长，COB结构提供的路径是比较灵活的（前提是COB资源充足）

