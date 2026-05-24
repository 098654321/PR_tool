## 0522 实验计划

testcases：使用case8 和case13这两个有解，但是以前不一定跑得出来的例子测试

实验目的：检测当前的算法是否正确

实验步骤：在服务器上使用case8和case13分别跑一次，查看结果与日志

## 测试结果

1. case13
    
    "HiGHS: expected exactly one active Y for PNnet: records_index=514 record_id=514 bit_id=0 2pin_record="Nege nets__split_514" logical_net(origin_key)="Nege nets" power_kind=Nege bump(s)=[T12,B1,G0,I0], got 0"，PNnet至少需要能分配到一个端口，但是分配失败，说明在ILP求解阶段就出bug。

    推测原因是precompute阶段计算可用track的时候范围压的太小，导致可用track太少，导致分配失败。

2. case8

    在MCF求解阶段失败。推测是MCF的建模仍然使用ILP阶段拆分的net，导致原本属于一个多扇出net的多个bump不能共享路径
