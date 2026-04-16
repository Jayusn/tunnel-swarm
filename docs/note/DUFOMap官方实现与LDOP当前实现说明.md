# DUFOMap 官方实现与 LDOP 当前实现说明

这份文档可以简化成下面这版核心结论。

## 简化总结

`LDOP` 现在和官方 `DUFOMap` 的核心一致点在于：都用 `UFOMap` 做逐帧增量建图，用 `seenFree(point)` 作为动态残影清理依据。区别不在“有没有用 DUFOMap 思路”，而在处理模式不同：官方参考实现更偏离线，`LDOP` 当前实现更偏在线。

可以把当前逻辑只记成两条流：

### 1. 地图更新流

当前帧点云每帧都送进 `UFOMap`，持续更新占据、free、seen-free 等状态。  
这部分是主链路，也是 `seenFree()` 判断的基础。

### 2. 历史点缓存流

历史点只是在旁路里缓存，供低频导出或调试使用，不会再次参与建图。  
因此，累计点云不是“重复积分地图”，而是“以后拿来筛选的候选点集合”。

## 和官方实现的主要区别

官方 `dufomap.cpp` 更像离线清图工具：先把整段序列全部积分进地图，再统一遍历累计点云，用 `!seenFree(p)` 保留静态点。  
`LDOP` 当前更像在线分割系统：每帧更新地图后，直接对当前帧做分类；累计点云重分类只保留为低频旁路，不放进热路径。

## 几个最容易混淆的点

- `seenFree(point)` 的含义：这个位置后来已经被观测为空闲，所以旧点更像动态残影。
- `propagate` 的作用：控制本帧地图修改后，状态是否立刻向八叉树上层传播。在线流程通常更适合 `true`。
- `min_range`：当前项目里主要用于和上游 SLAM 的近点过滤口径保持一致，继续传给 `UFOMap` 即可。
- `max_range`：适合同时约束 `UFOMap` 的工作半径和 `LDOP` 当前帧输出的判别半径。
- 官方 `cluster()`：只是对动态候选体素做辅助连通域聚类，不等于完整的检测、跟踪或预测模块，也不是点级动静态判别的核心。

## 一句话版本

官方 `DUFOMap` 参考程序是“离线累计后统一清图”，当前 `LDOP` 是“在线逐帧分割，历史缓存只做旁路导出”；两者共享的真正核心，都是 `UFOMap` 增量更新加 `seenFree` 判别。

## 参考位置

本地代码：

- `src/ufomap_mapper.cpp`
- `include/ldop/ufomap_mapper.h`
- `config/ldop.yaml`
- `docs/note/LDOP_design.md`

外部参考：

- DUFOMap 官方仓库：<https://github.com/KTH-RPL/dufomap>
- DUFOMap 官方参考实现主流程：<https://github.com/KTH-RPL/dufomap/blob/main/src/dufomap.cpp>
- DUFOMap 论文：<https://mit-spark.github.io/Longterm-Perception-WS/2024_IROS/assets/proceedings/DUFOMap/paper.pdf>
