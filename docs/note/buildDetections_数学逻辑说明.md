# buildDetections 数学逻辑说明

本文把 `DynamicObjectClusterer::buildDetections` 的实现过程转写为数学步骤，便于从“代码细节”切换到“算法视角”。

## 1. 输入与符号

设当前帧动态点集合为：

$$
\mathcal{P}=\{p_i\}_{i=1}^{N}
$$

每个点包含两类信息：

1. 空间坐标：$p_i=(x_i,y_i,z_i)$
2. 体素键：$k_i=(k_i^x,k_i^y,k_i^z,d_i)$

其中 $d_i$ 是八叉树 depth（层级）。在本实现中，键相等定义为四元组完全相等：

$$
k_a = k_b \iff (k_a^x,k_a^y,k_a^z,d_a)=(k_b^x,k_b^y,k_b^z,d_b)
$$

## 2. 按键分桶

先把点按体素键聚合，构造映射：

$$
\phi: k \mapsto B_k
$$

其中 $B_k$ 是属于键 $k$ 的点索引集合：

$$
B_k = \{i\mid k_i = k\}
$$

这一步等价于把原始点集压缩成“非空体素桶集合”。

## 3. 稳定排序

对桶索引按键做字典序排序：

$$
(d, k^x, k^y, k^z)
$$

即先比 depth，再比 $x/y/z$。数学上不改变聚类结果，只改变遍历顺序。工程目的：

1. 提高结果可复现性
2. 减少检测 ID 的抖动

## 4. 构建 key 空间图与邻接规则

把每个非空桶看作图节点 $v_k$。若两个键在连通规则下是邻居，则在它们之间连边。

给定当前键 $k=(k^x,k^y,k^z,d)$，同层步长为：

$$
s = 2^d
$$

邻居候选通过偏移 $\Delta=(\Delta_x,\Delta_y,\Delta_z)$ 生成：

$$
k'=(k^x+\Delta_x s,\;k^y+\Delta_y s,\;k^z+\Delta_z s,\;d)
$$

其中偏移集合由 connectivity 决定：

1. 6 连通：只允许单轴变化
2. 18 连通：允许最多两轴同时变化
3. 26 连通：允许三轴变化

注意：负向平移会做下溢保护（无符号 key 不能减到负数）。

## 5. BFS 连通域聚类

在上述图上执行 BFS，得到若干连通分量：

$$
\mathcal{C}_1,\mathcal{C}_2,\dots,\mathcal{C}_m
$$

每个连通分量对应一个候选目标，其点索引集合为桶索引并集：

$$
I(\mathcal{C}) = \bigcup_{k\in\mathcal{C}} B_k
$$

## 6. 点数阈值过滤

若某连通分量点数不足最小阈值 $n_{\min}$，则视为噪声并丢弃：

$$
|I(\mathcal{C})| < n_{\min} \Rightarrow \text{discard}
$$

## 7. AABB 与质心估计

对每个保留分量，同时计算坐标边界和点簇质心。AABB 用于尺寸、过滤和检测框可视化；点簇质心作为目标位置观测，后续 tracker 会用它更新轨迹中心。

$$
x_{\min}=\min_{i\in I(\mathcal{C})} x_i,\quad x_{\max}=\max_{i\in I(\mathcal{C})} x_i
$$
$$
y_{\min}=\min_{i\in I(\mathcal{C})} y_i,\quad y_{\max}=\max_{i\in I(\mathcal{C})} y_i
$$
$$
z_{\min}=\min_{i\in I(\mathcal{C})} z_i,\quad z_{\max}=\max_{i\in I(\mathcal{C})} z_i
$$

AABB 尺寸：

$$
\text{size}_x=x_{\max}-x_{\min},\;\text{size}_y=y_{\max}-y_{\min},\;\text{size}_z=z_{\max}-z_{\min}
$$

点簇质心：

$$
\text{center}=\frac{1}{|I(\mathcal{C})|}\sum_{i\in I(\mathcal{C})}(x_i,y_i,z_i)^T
$$

## 8. 尺寸上限过滤

用 AABB 对角线长度做尺度约束：

$$
L = \sqrt{\text{size}_x^2 + \text{size}_y^2 + \text{size}_z^2}
$$

若启用最大尺度阈值 $E_{\max}$，则：

$$
L > E_{\max} \Rightarrow \text{discard}
$$

## 9. 输出检测对象

对通过全部过滤的连通分量，生成一个检测：

1. `detection_id`：顺序分配
2. `bbox.center`：由点簇质心给出，作为跟踪阶段的位置观测
3. `bbox.size`：由 AABB 尺寸给出，用于尺度表达和可视化
4. `bbox.yaw = 0`
5. `stamp`：当前帧时间

最终输出：

$$
\mathcal{D}=\{d_j\}_{j=1}^{M}
$$

其中 $M$ 为本帧有效目标数。

## 10. 一句话总结

该函数可以概括为：

1. 点云在体素键空间离散聚合
2. 在键图上做连通域分割
3. 对每个连通域估计 AABB 尺寸和点簇质心
4. 通过点数和尺度阈值过滤后输出检测框
