# DUFOMap 官方实现与 LDOP 当前实现说明

本文整理当前 `LDOP` 中 `UfomapMapper` 的实现语义，以及它和 `DUFOMap` 官方论文、官方 C++ 参考实现之间的关系，重点澄清下面几个容易混淆的问题：

- `UFOMap` 的增量更新和 `seenFree(point)` 的关系
- 累计点云在官方实现里为什么存在
- 论文中的离线模式、在线模式和耗时口径分别是什么
- 当前 `LDOP` 为什么把累计点云重分类放在旁路
- 距离范围参数 `min_range / max_range` 在 `DUFOMap` 和 `LDOP` 里分别应该怎么使用
- `propagate` 参数到底在控制什么
- 官方 `cluster()` 的作用是什么，它和目标检测聚类是不是一回事

补充说明：

- 本文汇总当前已经确认过的实现结论，包括离线/在线、耗时口径、旁路累计、距离范围参数、`propagate`、`cluster` 等内容。
- 本文讨论时，以当前 `UfomapMapper` 主流程和官方 `dufomap.cpp` 为准。

## 1. 先分清两条数据流

当前实现里，有两条不同的数据流。

### 1.1 地图更新流

作用：更新 `UFOMap` 地图状态。

对应代码：

- `src/ufomap_mapper.cpp` 中 `updateUfomap()`
- 关键调用：`ufo::insertPointCloud(runtime_->map, *ufomap_cloud, sensor_origin, runtime_->integration_params, params_.propagate);`

这条流的输入只有“当前帧点云”。

每来一帧，都会把当前帧的点积分到 `UFOMap` 里，用于更新：

- 射线终点附近的 `hit / occupied` 证据
- 传感器到终点之间路径上的 `free / seen-free` 证据
- 地图内部维护的占据、可见空闲、`label` 等状态

因此，`LDOP` 和 `DUFOMap` 对 `UFOMap` 的处理都属于“每帧增量更新地图”，而不是每帧重置地图。

### 1.2 累计点云流

作用：保存历史原始观测点，供后续过滤、导出和可视化使用。

这条流不会再次调用 `insertPointCloud()`，因此不会直接更新 `UFOMap`。

也就是说，累计点云不是“重复积分地图”的输入，而是“后处理筛选”的候选点集合。

在当前 `LDOP` 代码里，这条流的行为是：

1. 每帧在 `UFOMap` 更新之后，把当前帧点追加进 `accumulated_points_`
2. 按 `ufomap_accumulation_max_points` 维护历史点缓存窗口
3. 默认不自动执行累计点云的动静态判别
4. 只有显式调用 `classifyAccumulatedCloud()` 时，才会对累计历史点执行 `seenFree` 过滤

## 2. `seenFree(point)` 到底是什么意思

当前阶段可以把 `seenFree(point)` 粗略理解成：

“这个位置后来已经被观测为可穿过的空闲空间。”

这正是 `DUFOMap` / 当前实现用来清理动态残影的核心依据。

举例：

1. 第 10 帧，一个行人站在 `x=5m`，累计点云里留下旧点 `p=(5, 0, 0)`。
2. 第 30 帧，这个人走开了，激光可以穿过 `x=5m` 并打到更远的墙。
3. 这时地图会积累出“`x=5m` 现在已经是空的”这类证据。
4. 当再次检查旧点 `p` 时，如果 `map.seenFree(p) == true`，就说明它更像是动态残影，应当从静态点云里删除。

所以：

- 地图更新是在描述“现在世界是什么状态”
- `seenFree` 过滤是在判断“过去存下来的哪些旧点已经不该再保留”

一句话总结：

- `UFOMap` 负责“判定依据”
- 累计点云负责“待筛选对象”

## 3. 论文里的离线模式和在线模式分别是什么

论文明确区分了两种使用方式：

- `offline map cleaning`
- `online detection of dynamic points`

其中：

- 离线模式：先把整段数据都积进地图，再基于最终地图统一分类历史点
- 在线模式：每来一帧，就用当前已经积累到此刻的地图去分类当前帧

论文原文里有两个很关键的结论：

1. `DUFOMap` 可以同时支持离线清图和在线动态点检测
2. 离线模式下，动态点分类是在最后、基于全部 void region 一次性完成的

这意味着：

- 官方 C++ 参考程序 `src/dufomap.cpp` 走的是典型离线路径
- 你现在 `LDOP` 的主流程更接近论文里的在线版本

## 4. 官方 C++ 参考实现里，累计点云是怎么用的

官方 `dufomap.cpp` 是一个离线程序。

它会：

1. 从数据目录读取一整段点云序列
2. 按顺序逐帧调用 `insertPointCloud()` 更新 `UFOMap`
3. 同时把每帧原始点追加到 `cloud_acc`
4. 整段序列处理完之后，再统一遍历 `cloud_acc`
5. 用 `!map.seenFree(p)` 保留静态点，生成最终 `cloud_static`

这里的“整段序列”不是算法自己决定的，而是由外部输入范围决定的，例如：

- `pcd` 目录下有哪些文件
- `dataset.first`
- `dataset.last`
- `dataset.num`

因此，官方 `cloud_acc` 更像：

- 最终 `clean static map` 的离线导出素材

而不是：

- 每帧反复参与地图积分的输入

## 5. 当前 LDOP 为什么把累计点云重分类放在旁路

当前 `LDOP` 是在线 ROS 流式处理，不存在天然的“整段离线序列结束”信号。

为了避免累计点云拖慢热路径，当前代码已经改成：

1. 当前帧点云更新 `UFOMap`
2. 顺手维护一份仅供低频导出的历史点缓存
3. 只对当前帧点执行 `seenFree` 分类
4. 当前帧静/动态点云直接用于主流程输出
5. 静态地图本体由 `UFOMap` 地图状态单独查询和可视化

也就是说，当前主流程里已经没有“累计点云重新过滤再同步输出”的这一步，但仍然保留了：

- 历史点缓存
- `classifyAccumulatedCloud()` 低频导出接口

这和论文思路并不冲突。更准确地说：

- 官方离线清图：允许最后统一回看全部历史点
- 当前 `LDOP` 在线分割：更适合把累计重分类放到低频旁路

因此，“累计点云很耗时，所以放在旁路”是合理选择，不代表没有使用 `DUFOMap` 的核心判别逻辑。

## 6. 论文里的耗时口径应该怎么理解

论文里关于耗时和运行频率，需要区分两个口径。

### 6.1 方法级计算瓶颈

论文明确指出：

- 主要计算量在“void region classification / 地图更新”这一部分
- 点级“静态还是动态”的分类，本质上只是一次快速地图查询

换句话说：

- `UFOMap` 增量更新和 void region 维护是重活
- `point -> seenFree lookup` 只是轻量查询

### 6.2 实验表格的 runtime 口径

论文 `Table II` 的说明是：

- reported time = total processing time divided by the number of point clouds

这表示它统计的是“总处理时间 / 帧数”的均摊值，不是严格意义上的在线单帧端到端发布时延。

此外，论文主 map cleaning 实验默认是离线清图口径；它另外再给在线变体 `DUFOMap*` 和 `Dynablox` 做在线对比。

论文还额外提到：

- 在 Intel NUC、`semi-indoor` 数据、`range=20m` 的设置下，`DUFOMap` 可以维持 `20 Hz`

因此，当前 `LDOP` 里如果把 `classifyAccumulatedCloud()` 放回每帧热路径，其耗时并不能直接拿去和论文主文里的 runtime 逐项对比，因为那部分更接近离线导出成本，而不是论文强调的在线主路径成本。

## 7. 当前代码里和累计点云相关的三个层次

为了避免再次混淆，可以把当前代码分成下面三层理解。

### 7.1 每帧必跑

- 点云提取
- `updateUfomap()`
- `classifyPoints(frame_points)`

这些共同构成当前帧主流程。

### 7.2 每帧维护但不参与主输出

- `accumulated_points_` 的追加与裁剪
- `accumulateCache` 耗时统计

这部分现在仍会每帧执行，但它只是在维护低频导出所需的历史缓存，不会自动改变当前帧静/动态点云的输出语义。

### 7.3 手动低频触发

- `classifyAccumulatedCloud()`

这部分已经实现，但不会自动运行；只有测试或后续调试/导出接口显式调用时，才会对累计点云执行 `seenFree` 判别。

## 8. `propagate` 参数到底在控制什么

`propagate` 不是“离线模式”和“在线模式”的别名，但它和两者的使用方式强相关。

从当前安装版 `ufomap` 的实现看，`insertPointCloud()` 在完成本帧 hit/miss 更新后，会根据 `propagate` 决定是否立刻调用：

- `map.propagateModified()`

`octree.hpp` 对 `propagateModified()` 的注释是：

- `Propagate modified information up the octree.`

也就是说，`propagate` 控制的是：

- 本帧修改过的叶子节点，是否要立刻把派生信息、汇总状态、修改标记继续向八叉树父节点方向传播

可以把它直观理解成：

- `propagate=true`
  - 每帧插入后，地图立刻进入更完整的“查询就绪”状态
  - 更适合在线流程，因为插入后马上就要做 `seenFree()`、候选查询、`label` 查询等操作
- `propagate=false`
  - 先只积累局部修改，暂时不把整棵树的派生状态都更新完
  - 更适合批处理很多帧后，最后再统一 `propagateModified()` 一次

因此：

- 它和离线/在线有关，但不是一回事
- 在线主路径通常更倾向 `true`
- 离线批量导出更容易采用 `false + 最后手动 propagateModified()`

在官方 `dufomap.cpp` 里，如果配置了 `propagate=false`，它会在整段数据处理完后显式调用一次：

- `map.propagateModified()`

然后再去做后续查询、聚类和导出。

## 9. 距离范围参数在 DUFOMap 和 LDOP 里应该怎么使用

这里重点讨论两个参数：

- `min_range`
- `max_range`

以及它们和 `only_valid` 的关系。

### 9.1 DUFOMap 官方如何使用 `min_range`

官方 `DUFOMap` 对 `min_range` 采用的是“两层用法”：

1. 在 `ufo::insertPointCloud(...)` 之前，先手动调用 `filterminDistance(...)`
2. 再把同一个 `min_range` 放进 `config.integration`，继续传给 `UFOMap`

也就是说，官方的 `min_range` 既用于：

- 外层近点预过滤

也用于：

- `UFOMap` 内部射线积分和 miss / void-region 相关逻辑

官方配置里，`min_range` 的注释也写得很明确：

- 用于滤掉 ego 点，例如某些 SLAM 结果会把传感器自身附近点一并带进来

### 9.2 当前 LDOP 对 `min_range` 的推荐用法

当前 `LDOP` 使用的是：

- 经过 `SLAM` 后、已经对齐到世界坐标系的点云
- 同样在世界坐标系下的里程计

在这个前提下，只要点云和里程计位姿在同一个坐标系里，`min_range` 和 `max_range` 本质上看的都是：

- 点到传感器原点的欧氏距离

因此，世界系使用这两个参数在语义上是成立的。

结合当前项目的实际前提：

- 上游 `SLAM` 已经严格过滤了近点
- 这个近点阈值就是当前真正想过滤掉的最小距离

那么当前 `LDOP` 的推荐做法是：

- 不再额外重复做一层外部 `min_range` 近点过滤
- 继续把同样的 `ufomap_min_range` 传给 `UFOMap`

这样可以把 `ufomap_min_range` 理解成：

- 和上游 `SLAM` 保持一致的保护参数
- 用于让 `UFOMap` 内部积分/射线逻辑和输入点云预处理口径保持一致

因此，在“上游点云已经严格去掉近点”的前提下，可以认为：

- 当前 `LDOP` 对 `min_range` 只继续传给 `UFOMap` 就够了

### 9.3 DUFOMap 官方如何使用 `max_range`

和 `min_range` 不同，官方 `DUFOMap` 没有额外手写一层“外部最大距离过滤”。

官方对 `max_range` 的使用方式主要是：

- 作为 `config.integration.max_range` 传给 `ufo::insertPointCloud(...)`

默认配置甚至直接给的是：

- `max_range = -1`

也就是：

- 默认不限制最大距离

所以更准确地说，官方 `DUFOMap` 对 `max_range` 的主要使用方式是：

- 给 `UFOMap` 积分逻辑使用

### 9.4 当前 LDOP 对 `max_range` 的推荐用法

当前 `LDOP` 里，`ufomap_max_range` 适合承担两个一致的语义：

1. `UFOMap` 地图更新的最大工作半径
2. `LDOP` 输出静/动态点云的最大判别半径

推荐做法是：

- 地图更新分支：仍然把完整当前帧点云传给 `ufo::insertPointCloud(...)`
- 由 `UFOMap` 内部根据 `ufomap_max_range` 和 `ufomap_insert_only_valid` 决定实际有效积分范围
- 输出分支：在做 `classifyPoints()` 之前，用同一个 `ufomap_max_range` 先筛出需要参与当前帧输出判别的局部点集

这样做有三个好处：

1. `UFOMap` 的工作半径和 `LDOP` 的输出半径可以共用同一个参数
2. 调用 `UFOMap` 接口时，输入仍然保持完整当前帧，不需要在进入地图之前额外做一层外部 `max_range` 裁剪
3. 输出分支只对局部点做 `seenFree()` 查询，能避免无意义地分类远处点后再丢弃，性能上更稳

这里要特别区分两件事：

- “传给 `ufo::insertPointCloud(...)` 的原始当前帧点云是否完整”
- “真正被 `UFOMap` 作为有效积分输入的点是否完整”

在当前这套推荐设计下：

- 调用接口时传入的仍然可以是完整当前帧点云
- 但实际会不会进入 `UFOMap` 有效更新，还要由 `max_range` 和 `only_valid` 决定

### 9.5 `max_range + only_valid` 该怎么理解

当前项目配置示例是：

- `ufomap_max_range: 12.0`
- `ufomap_insert_only_valid: true`

这组参数可以理解成：

- `UFOMap` 只维护传感器周围 `12m` 内的有效积分范围
- 超过 `12m` 的点不会作为有效输入参与地图更新

如果同时在输出分支也使用同一个 `12m` 先过滤当前帧点集，再做 `classifyPoints()`，那么可以认为：

- 最终 `LDOP` 输出的最大判别范围，和 `UFOMap` 的最大工作范围保持一致
- 不会在“最大判别范围内”混入那些超出 `UFOMap` 工作半径、却仍被拿来输出分类的点

这正是当前项目里对 `max_range` 最稳妥、也最容易理解的用法。

### 9.6 当前项目对距离参数的一句话建议

综合当前项目前提，推荐记住下面三点：

- `min_range`：由于上游 `SLAM` 已经严格过滤近点，当前 `LDOP` 里只需要继续把同样的最小距离传给 `UFOMap`
- `max_range`：继续作为 `UFOMap` 的最大工作半径参数使用
- 输出范围过滤：最好放在“分类前的输出分支”而不是“消息生成的最后一步”，这样既能保证输出语义清晰，也能减少不必要的 `seenFree()` 查询开销

## 10. 官方 `cluster()` 的作用是什么

官方 `dufomap.cpp` 里还有一个 `cluster()`，它会对满足下面条件的体素做 BFS/区域生长：

- `SeenFree`
- `HitsMin(1)`
- `Label(0)`

它的作用主要是：

1. 把邻近的动态候选体素归成一团
2. 用 `min_points` 去掉很小的噪声团
3. 给每一团写上 `label`

更准确地说，它是：

- 动态候选体素的连通域聚类

而不是：

- 完整意义上的目标检测 / 目标跟踪聚类

它不直接提供：

- 稳定目标 ID
- 跨帧关联
- 包围盒输出
- 速度估计

还要特别注意一点：

- 官方 `cloud_static` 的生成并不依赖 `cluster()` 的结果
- 最终静态点云仍然只是按 `!map.seenFree(p)` 过滤得到

所以 `cluster()` 更像：

- 一个辅助的动态候选区域后处理步骤

而不是：

- 点级动静态判别主链条的必要组成部分

论文也明确提到：

- 他们早期版本用过 clustering 来处理静/动态边界问题
- 后来发现更好地建模定位误差 `dp` 和传感器噪声 `ds` 更有效、更通用

因此，当前 `LDOP` 没把这套 clustering 主流程接回主链条，并不意味着缺失了点级 `DUFOMap` 判别本体。

## 11. 当前阶段一句话结论

最应该记住的结论是：

- `UFOMap` 的每帧增量更新负责维护地图状态
- `seenFree` 负责提供点级动静态判别依据
- 官方 `DUFOMap` C++ 程序更偏“离线累计 + 序列末尾统一导出”
- 当前 `LDOP` 更偏“在线当前帧分割 + 历史缓存旁路保留”
- `min_range` 在当前项目里主要用于和上游 `SLAM` 近点过滤口径保持一致
- `max_range` 适合同时约束 `UFOMap` 的工作半径和 `LDOP` 的输出判别半径
- `propagate` 控制的是地图修改后是否立刻把状态向八叉树上层传播
- 官方 `cluster()` 是动态候选体素的辅助聚类，不等同于完整目标检测

## 12. 参考位置

本地代码：

- `src/ufomap_mapper.cpp`
- `include/ldop/ufomap_mapper.h`
- `config/ldop.yaml`
- `docs/note/LDOP_design.md`

外部参考：

- DUFOMap 官方仓库：<https://github.com/KTH-RPL/dufomap>
- DUFOMap 官方参考实现主流程：<https://github.com/KTH-RPL/dufomap/blob/main/src/dufomap.cpp>
- DUFOMap 论文：<https://mit-spark.github.io/Longterm-Perception-WS/2024_IROS/assets/proceedings/DUFOMap/paper.pdf>
