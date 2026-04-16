# DUFOMap 参数工作流与 LDOP 参数说明

这份文档的目标不是逐个背参数名，而是按 `DUFOMap` / `UFOMap` 的实际工作流程来理解：

1. 点云怎样进入地图
2. 射线怎样把空间写成 hit / miss / seenFree
3. `LDOP` 怎样基于 `seenFree(point)` 把当前帧分成静态和动态

这样理解后，后面调参时可以先判断自己是在调“输入预处理”“积分方式”“free 空间生成方式”，还是在调“当前帧输出口径”。

## 1. 先看官方 DUFOMap 的主流程

官方 `DUFOMap` 当前主程序在 `src/dufomap.cpp`，配置文件是 `assets/config.toml`。

可以把它简化成下面 6 步：

### 1.1 读取配置并创建地图

- 读取 `important.resolution`
- 读取 `map.levels`
- 用这两个参数构造 `ufo::Map`

对应含义：

- `resolution` 决定叶子体素尺寸
- `levels` 决定八叉树层级规模

### 1.2 逐帧读入点云和视点

每帧会读出：

- 当前帧点云 `cloud`
- 传感器视点 `viewpoint.translation`

这里的 `viewpoint.translation` 就是后面射线积分的起点。

### 1.3 先做近点过滤

官方会先用 `min_range` 把传感器附近的点滤掉，避免自车点、原点附近脏点进入地图。

这一步发生在积分之前，所以会影响后面所有 hit / miss / seenFree 结果。

### 1.4 把当前帧点云积分进 UFOMap

核心调用就是：

```cpp
ufo::insertPointCloud(map, cloud, viewpoint.translation, config.integration, config.propagate);
```

这一句决定了：

- 哪些点被当作命中 `hit`
- 哪些射线区段被写成 free / miss
- 哪些位置后续会变成 `seenFree`

`important` 组和 `integration` 组的大多数参数都在这一步生效。

### 1.5 可选传播与聚类

- 如果 `propagate=false`，官方会在全部帧处理完后再统一 `map.propagateModified()`
- 如果启用 clustering，会对 `seenFree && hits>=1 && label==0` 的叶子节点做聚类

注意：

- 聚类不是点级动静态判别的核心
- 判别核心仍然是 `seenFree(point)`

### 1.6 用 `seenFree(point)` 过滤累计点云

官方最后遍历累计点云：

```cpp
if (!map.seenFree(p)) {
  cloud_static.push_back(p);
}
```

含义很直接：

- `seenFree(p) == true`：这个位置后来被观测为空闲，更像动态残影
- `seenFree(p) == false`：这个点没有被后续 free 观测“推翻”，更像静态

## 2. LDOP 当前实现和官方 DUFOMap 的差异

当前 `LDOP` 的 `UfomapMapper` 沿用了同一条核心思路，但工作模式已经从“离线累计后统一过滤”变成了“在线逐帧更新后立即对当前帧分类”。

主流程可以简化成下面 5 步：

### 2.1 读取 ROS 点云并提取有限点

- 从 `PointCloud2` 里提取有限 `xyz`
- 非有限点直接丢弃

### 2.2 用当前 odom 位置作为 `sensor_origin`

- `odom.pose.pose.position` 被直接当作射线起点
- 当前实现默认输入点云和 odom 已经在同一个世界系

### 2.3 当前帧立即积分进 UFOMap

同样调用 `ufo::insertPointCloud(...)`，因此大多数 `ufomap_*` 参数本质上仍然是 UFOMap 积分参数。

### 2.4 当前帧按 `seenFree(point)` 立即分类

和官方“最后统一过滤累计点云”不同，当前 `LDOP` 在每帧更新后就对当前帧做：

- `seenFree(point) == true` -> `/ldop/dynamic_cloud`
- `seenFree(point) == false` -> `/ldop/static_cloud`

### 2.5 历史点缓存只保留给低频导出

`ufomap_accumulation_max_points` 维护的是历史点缓存窗口，只给低频 `classifyAccumulatedCloud()` 使用，不参与当前帧实时分类。

所以当前 `LDOP` 里有三类参数：

1. 从 DUFOMap / UFOMap 直接继承的积分参数
2. 在线流程新增的“当前帧输出口径”参数
3. 工程层面的性能 / 内存参数

## 3. 官方 DUFOMap 参数和 LDOP 参数的对应关系

| 官方 DUFOMap | 当前 LDOP | 说明 |
| --- | --- | --- |
| `important.resolution` | `ufomap_resolution` | 叶子体素尺寸 |
| `map.levels` | `ufomap_depth_levels` | 八叉树层级规模 |
| `integration.min_range` | `ufomap_min_range` | 近点过滤 / 最小积分距离 |
| `integration.max_range` | `ufomap_max_range` | 最大积分距离；在 LDOP 里还影响当前帧分类输出半径 |
| `integration.hit_depth` | `ufomap_insert_hit_depth` | 命中更新深度 |
| `integration.miss_depth` | `ufomap_insert_miss_depth` | free / miss 更新深度 |
| `integration.ray_casting_depth` | `ufomap_ray_casting_depth` | 射线投射深度 |
| `important.inflate_unknown` | `ufomap_inflate_unknown` | 论文里的 `d_p` |
| `integration.inflate_unknown_compensation` | `ufomap_inflate_unknown_compensation` | 是否补偿未知区域膨胀 |
| `integration.ray_passthrough_hits` | `ufomap_ray_passthrough_hits` | 射线是否穿过命中点 |
| `important.inflate_hits_dist` | `ufomap_inflate_hits_dist` | 论文里的 `d_s` |
| `integration.down_sampling_method` | `ufomap_down_sampling_method` | 积分前降采样策略 |
| `integration.simple_ray_casting` | `ufomap_simple_ray_casting` | 简化射线投射开关 |
| `integration.simple_ray_casting_factor` | `ufomap_simple_ray_casting_factor` | 简化射线步长系数 |
| `integration.parallel` | `ufomap_parallel` | 是否并行积分 |
| `integration.num_threads` | `ufomap_num_threads` | 线程数 |
| `integration.only_valid` | `ufomap_insert_only_valid` | 是否只对有效射线段积分 |
| `integration.propagate` | `ufomap_propagate` | 每帧后是否立即传播修改 |
| 无 | `ufomap_map_reserve_nodes` | LDOP 工程参数，预分配节点容量 |
| 无 | `ufomap_accumulation_max_points` | LDOP 工程参数，历史点缓存窗口 |

## 4. 按工作流程理解每个参数

下面按“数据流经过哪里，这个参数就在哪里讲”的顺序说明。

### 4.1 地图初始化阶段

这一阶段决定地图的基本空间表达能力。

#### `ufomap_resolution`

作用：

- 叶子体素边长，单位米
- 这是最核心的参数之一

调大时通常会：

- 地图更粗
- 小目标和细边缘更容易被吞并
- 对位姿抖动、测距噪声更宽容
- 内存和积分开销通常下降

调小时通常会：

- 地图更细
- 更容易保留结构细节
- 也更容易把轻微配准误差放大成 `seenFree` 误检

经验理解：

- 它本质上决定了“你允许系统把多大的几何差异视为同一个体素”

#### `ufomap_depth_levels`

作用：

- 八叉树层级规模
- 也约束 `hit_depth`、`miss_depth`、`ray_casting_depth` 的合法范围

对当前项目的直接意义：

- 它更像地图空间规模和层级能力的上限设置
- 一般不是第一优先级调参项

什么时候需要关心：

- 你想扩更大空间范围
- 或者希望更细地控制不同深度上的更新行为

#### `ufomap_map_reserve_nodes`

作用：

- 初始化地图时预留节点容量

影响：

- 只影响内存分配和性能
- 不改变动静态判别语义

什么时候调：

- 地图长期增长、插入阶段频繁扩容时

### 4.2 输入预处理阶段

这一阶段决定“哪些点有资格进入积分”。

#### `ufomap_min_range`

作用：

- 过滤离 `sensor_origin` 太近的点
- 常用于剔除自车点、原点附近噪声、某些 SLAM 算法夹带的 pose 点

调大时通常会：

- 更激进地去掉近点
- 近距离障碍物和机体附近结构也更容易丢失

调小时通常会：

- 保留更多近点
- 也更容易把自车残影、近场脏点写进地图

#### `ufomap_down_sampling_method`

作用：

- 点云进入积分前的降采样策略

可选值：

- `none`
- `center`
- `centroid`
- `uniform`

调参理解：

- `center` 更偏稳定、可解释，常用于调试
- `uniform` 更偏速度和分布均匀
- `none` 保留全部点，但积分负担最大

如果你在调试误检：

- 优先用 `center`
- 这样更容易把误差归因到射线和位姿，而不是采样随机性

### 4.3 射线积分阶段

这是最关键的阶段，因为 `seenFree` 的来源就在这里。

#### `ufomap_insert_hit_depth`

作用：

- 命中点 `hit` 写入的深度

直观理解：

- 控制“障碍物命中”被记录在哪一层体素上

常见情况：

- 保持 `0`，沿用默认叶层语义，通常够用

#### `ufomap_insert_miss_depth`

作用：

- free / miss 写入深度

直观理解：

- 控制“射线经过但没命中”的空间以哪一层体素被写成 free

调参影响：

- 这个参数和 `ray_casting_depth` 一起，决定 free 空间写入的尺度

#### `ufomap_ray_casting_depth`

作用：

- 射线投射时使用的深度

调参影响：

- 会影响 free carving 的空间尺度
- 进而影响后面哪些点会被 `seenFree(point)` 判成动态

什么时候调：

- 只有当你明确在做不同深度更新策略实验时再碰
- 否则一般保持 `0`

#### `ufomap_max_range`

作用：

- 限制积分时的最大射线长度

在当前 `LDOP` 中还有一个额外作用：

- 当前帧分类前，还会用它再过滤一次输出点，只分类工作半径内的点

所以在 LDOP 里它同时决定两件事：

1. 多远的点会参与地图积分
2. 多远的点会参与当前帧动静态输出

调大时通常会：

- 纳入更远点
- 远距离噪声、稀疏点、配准误差更容易影响 `seenFree`

调小时通常会：

- 当前帧输出更保守
- 远处误检常常下降
- 但远处动态也更可能直接不参与判别

#### `ufomap_insert_only_valid`

作用：

- 是否只对有效射线区段做积分

直观理解：

- 它控制“超过有效范围或不满足条件的射线段”是否参与 free carving

调参影响：

- 对远距离噪声和范围边界附近的误判比较敏感

### 4.4 DUFOMap 论文相关的两组关键容错参数

这组参数最接近论文里的 `d_p` 和 `d_s`，本质是在做“对误差的容忍建模”。

#### `ufomap_inflate_unknown`

作用：

- 论文里的 `d_p`
- 在未知区域邻域上做膨胀，容忍位姿误差或观测稀疏带来的边界不稳定

调大时通常会：

- 系统更不容易把轻微错位直接判成 free
- 误检动态可能下降
- 但真实动态和细结构也可能被“保守化”

调小时通常会：

- 系统更敏感
- 更容易出现边缘闪烁式动态误检

#### `ufomap_inflate_unknown_compensation`

作用：

- 是否对 `inflate_unknown` 带来的末端影响做补偿

直观理解：

- 是 `d_p` 的配套细化开关

建议：

- 如果你在按 DUFOMap 默认思路靠拢，通常和 `inflate_unknown` 配套开启

#### `ufomap_inflate_hits_dist`

作用：

- 论文里的 `d_s`
- 沿射线方向扩展命中区域，容忍测距噪声

调大时通常会：

- 更不容易因为一点点距离波动就被后续 free 观测推翻
- 静态结构更稳
- 但动态目标残影也可能更容易留在静态里

调小时通常会：

- 对真实动态更敏感
- 也更容易让静态边缘被误判成动态

### 4.5 射线是否“穿透命中点”

#### `ufomap_ray_passthrough_hits`

作用：

- 控制射线遇到命中点后，是否继续向后穿过命中位置

常见推荐：

- DUFOMap 默认更偏向 `false`

为什么：

- 不让射线轻易穿过命中点，通常更符合“命中点后面不可随便写 free”的保守原则
- 对减少静态结构背后的错误 free carving 往往更有利

### 4.6 射线投射算法选择

#### `ufomap_simple_ray_casting`

作用：

- `true` 使用简化射线投射
- `false` 使用更完整的 proper ray casting

调参理解：

- `true` 往往更快
- `false` 往往更稳、更适合调试和对齐官方默认行为

如果你正在追查误检：

- 这是非常值得优先做 A/B 的参数

#### `ufomap_simple_ray_casting_factor`

作用：

- 简化射线投射的步长系数

影响：

- 只在 `ufomap_simple_ray_casting=true` 时有意义
- 系数越大，步长越激进，速度更高，但近似误差风险也更大

调试建议：

- 如果要对齐官方默认思路，先回到 `simple_ray_casting=false`
- 如果必须用简化射线，再从 `1.0` 起步

### 4.7 并行与传播阶段

这组参数主要影响性能、时序和地图状态何时可见。

#### `ufomap_parallel`

作用：

- 是否并行执行积分

影响：

- 主要是性能
- 理论上不应改变最终语义，但调试时为了减少不必要变量，常先关掉做基线实验

#### `ufomap_num_threads`

作用：

- 并行积分的线程数

影响：

- 主要是性能调度
- 不直接改变判别逻辑

#### `ufomap_propagate`

作用：

- 每帧插入后是否立即传播修改

在官方 DUFOMap 中：

- 更偏离线；可以处理完整段数据后再统一传播

在当前 LDOP 中：

- 因为每帧插入后立刻就要用 `seenFree(point)` 做在线分类，所以通常更适合保持 `true`

这是一个非常关键的语义差异：

- 在 LDOP 里，它不是单纯性能开关
- 它还决定“当前帧分类时，地图状态是否已经更新到可查询状态”

### 4.8 历史缓存与低频导出阶段

这部分不是官方 DUFOMap 的积分参数，而是 LDOP 在线实现为了保留低频 clean map 导出能力额外加的。

#### `ufomap_accumulation_max_points`

作用：

- 限制历史点缓存窗口大小

重要说明：

- 它不参与当前帧 `/ldop/static_cloud` 和 `/ldop/dynamic_cloud` 的实时分类
- 只影响后续低频 `classifyAccumulatedCloud()` 的候选点集合

所以：

- 如果你在调当前帧动静态判别效果，不要优先改它

## 5. 哪些参数最值得优先调

如果目标是“让当前帧动静态分割先稳定下来”，优先级建议如下。

### 第一优先级：最影响 `seenFree` 误检的参数

- `ufomap_resolution`
- `ufomap_max_range`
- `ufomap_inflate_unknown`
- `ufomap_inflate_hits_dist`
- `ufomap_ray_passthrough_hits`
- `ufomap_simple_ray_casting`
- `ufomap_down_sampling_method`

这几项最直接影响：

- free 空间怎么被 carving
- 静态结构边缘会不会被后续 free 观测推翻
- 远距离噪声会不会进入判别链

### 第二优先级：输入口径与近场稳定性

- `ufomap_min_range`
- `ufomap_insert_only_valid`

这两项更像“哪些点和哪些射线有资格参与积分”。

### 第三优先级：一般不先动的参数

- `ufomap_depth_levels`
- `ufomap_insert_hit_depth`
- `ufomap_insert_miss_depth`
- `ufomap_ray_casting_depth`
- `ufomap_parallel`
- `ufomap_num_threads`
- `ufomap_map_reserve_nodes`
- `ufomap_accumulation_max_points`

它们不是没用，而是通常不是第一批造成“效果很差”的主因。

## 6. 如果想先靠近官方 DUFOMap 的默认思路

可以把下面这组当作“官方思路基线”来理解：

```yaml
ufomap_resolution: 0.1
ufomap_inflate_unknown: 1
ufomap_inflate_unknown_compensation: true
ufomap_ray_passthrough_hits: false
ufomap_inflate_hits_dist: 0.2
ufomap_down_sampling_method: center
ufomap_min_range: 0.2
ufomap_max_range: <=0  # 表示不限制
ufomap_insert_only_valid: false
ufomap_insert_hit_depth: 0
ufomap_insert_miss_depth: 0
ufomap_ray_casting_depth: 0
ufomap_simple_ray_casting: false
ufomap_simple_ray_casting_factor: 1.0
ufomap_parallel: true
ufomap_num_threads: 0
```

但要注意，当前 `LDOP` 和官方仍然有三个本质差异：

1. `LDOP` 是在线逐帧分类，官方更偏离线累计后统一过滤
2. `LDOP` 当前帧输出还会额外受 `ufomap_max_range` 影响
3. `LDOP` 的 odom 和点云必须已经在同一个世界系里，否则参数再像官方也会误检

所以“照抄官方默认参数”不一定直接得到最好结果，但这组参数很适合作为调试起点。

## 7. 对当前仓库最实用的调参建议

如果你现在的目标是先把 `/ldop/static_cloud` 和 `/ldop/dynamic_cloud` 稳住，建议按这个顺序做：

### 7.1 先确认不是输入问题

先确认：

- `/cloud_registered` 在 `world_frame` 下是否稳定
- `/Odometry` 的位置是否就是点云射线起点
- 点云和 odom 是否已经对齐到同一世界系

如果这里有抖动，后面任何 `seenFree` 相关参数都会被动放大误差。

### 7.2 先从“官方思路基线”开始

优先回到更保守、更可解释的一组：

- `ufomap_down_sampling_method: center`
- `ufomap_simple_ray_casting: false`
- `ufomap_simple_ray_casting_factor: 1.0`
- `ufomap_inflate_unknown: 1`
- `ufomap_inflate_unknown_compensation: true`
- `ufomap_inflate_hits_dist: 0.2`
- `ufomap_ray_passthrough_hits: false`

### 7.3 再单变量调 `resolution` 和 `max_range`

这两个参数最容易让结果发生肉眼可见变化：

- `resolution` 主要影响“对抖动的容忍度”和“结构细节”
- `max_range` 主要影响“远处点是否进入积分和当前帧输出”

### 7.4 最后才碰深度和并行参数

等前面稳定后，再决定要不要调整：

- `ufomap_insert_hit_depth`
- `ufomap_insert_miss_depth`
- `ufomap_ray_casting_depth`
- `ufomap_parallel`
- `ufomap_num_threads`

## 8. 相关代码位置

当前仓库里和本文最相关的位置：

- `config/ldop.yaml`
- `include/ldop/ufomap_mapper.h`
- `src/ufomap_mapper.cpp`
- `docs/note/DUFOMap官方实现与LDOP当前实现说明.md`

外部参考：

- 官方配置：`https://github.com/KTH-RPL/dufomap/blob/main/assets/config.toml`
- 官方主流程：`https://github.com/KTH-RPL/dufomap/blob/main/src/dufomap.cpp`

