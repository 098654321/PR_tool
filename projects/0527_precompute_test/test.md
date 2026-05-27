# 0527

## 实验目的：

以往的实验出现四个问题：case13有第一步TOB资源分配过程中ILP求解失败的问题，case5有TrackToBumpsNet路径不合理和求解失败的问题，module_test/test_function/case1含obj的mcf求解过程停不下来，需要检查一下这四个问题是什么原因

## 测试数据

case13，case5

## 测试方法

-  case5 TrackToBumpsNet 路径不合理

可以把路径画出来，看一下是否有TOB位置的track阻挡、其它路径阻挡、track变换关系不合理等原因

- case5 TrackToBumpsNet 路径求解失败

需要单个unit下全局的布线资源使用情况，看看是不是因为track变换关系不合理，或者是先算了bus导致剩下的net无法连通，或者是建模有问题

可以在失败的unit下单独跑一个maze，看maze能否跑通。

- case13第一步ILP求解失败

这个例子本身是可解的，需要检查失败的net，然后看针对这个net的建模本身是否有问题，或者是track变换关系不合理

- test/module_test/test_function/test_tracktobumpsnet/case1_feasible 的mcf求解过程停不下来

这个例子是有解的，停不下来说明建模不合理/求解器没用对，可以先换一个求解器检查一下


## 测试结果

- 第一次测试：case5 TrackToBumpsNet 路径求解失败

debug-ilp-mcf在ILP+MCF之后调用了完整的mazerouting功能，发现可以跑通。但是debug-mcf根据ILP分配好的track在COB阵列上用maze跑的时候发现还是会失败。说明不是MCF建模本身的问题，而是一开始ILP分配的track不合理，或者是先跑了bus导致剩下的net无法连通
但是bus确实应该先布线，所以优先考虑调整一开始的track分配策略

