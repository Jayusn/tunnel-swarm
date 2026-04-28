# DynamicObjectTracker 与多运动模型 Kalman 工具数学逻辑说明

本文把当前跟踪模块和内联的多运动模型 Kalman 工具，从代码流程改写成数学形式。它描述的是当前实现，而不是理想化设计：第一版虽然保留了 `MotionModelType` 和 factory，但运行时实际只使用 `CA3D`，也就是 3D constant-acceleration 模型。

对应代码主要位于：

1. `include/ldop/dynamic_object_tracker.h`
2. `src/dynamic_object_tracker.cpp`
3. `include/ldop/multi_model_kalman_filter.h`
4. `src/multi_model_kalman_filter.cpp`

## 1. 模块边界

当前第二阶段跟踪链路可以写成：

$$
\mathcal{D}_k
\xrightarrow{\text{DynamicObjectTracker}}
\mathcal{T}_k
\xrightarrow{\text{buildOutput}}
\mathcal{O}_k
$$

其中：

1. $\mathcal{D}_k=\{d_j\}_{j=1}^{M_k}$ 是第 $k$ 帧的动态目标检测集合，由聚类模块输出。
2. $\mathcal{T}_k=\{T_i\}_{i=1}^{N_k}$ 是 tracker 内部维护的轨迹集合。
3. $\mathcal{O}_k$ 是对外发布的 `ldop/DynamicObjectArray`。

职责边界是：

1. `DynamicObjectTracker` 维护轨迹列表、数据关联、建轨、漏检 coast、删除、输出消息和 marker。
2. `TrackState` 是单条轨迹的运行状态，内部独占一个 `MultiModelKalmanFilter`。
3. `MultiModelKalmanFilter` 只负责单条轨迹的状态估计，不接触 ROS I/O，不知道其他轨迹，也不做数据关联。
4. `MotionModel` 是滤波器文件内部的运动模型接口，当前唯一实现是 `CA3DMotionModel`。

换成数学对象：

$$
T_i =
\left(
\text{id}_i,\;
x_i,\;
P_i,\;
B_i,\;
\text{age}_i,\;
\text{hits}_i,\;
\text{miss}_i,\;
\tau_i,\;
\mathcal{H}_i
\right)
$$

其中：

1. $\text{id}_i$ 是稳定轨迹 ID。
2. $x_i$ 是 Kalman 状态向量。
3. $P_i$ 是状态协方差。
4. $B_i$ 是轨迹当前 bbox，中心由滤波器位置覆盖，尺寸来自检测框平滑。
5. $\text{age}_i$ 是轨迹经历的总帧数。
6. $\text{hits}_i$ 是成功关联 detection 的次数。
7. $\text{miss}_i$ 是连续漏检帧数。
8. $\tau_i$ 是该轨迹上次处理时间戳。
9. $\mathcal{H}_i$ 是用于 marker 和调试的历史样本序列。

## 2. 输入检测量

第 $k$ 帧每个 detection 记为：

$$
d_j = (\text{detection\_id}_j,\; B_j,\; t_k)
$$

其中 bbox 为：

$$
B_j = (c_j,\; s_j,\; \psi_j)
$$

当前实现中：

1. $c_j=(x_j,y_j,z_j)^T\in\mathbb{R}^3$ 是聚类阶段计算的点簇质心。
2. $s_j=(l_j,w_j,h_j)^T\in\mathbb{R}^3$ 是同一连通块的 AABB 尺寸。
3. $\psi_j=0$，当前没有引入有向框。

跟踪滤波只把中心 $c_j$ 当作 3D 位置观测：

$$
z_j = c_j
$$

尺寸 $s_j$ 不进入 Kalman 状态，只在轨迹 bbox 中做指数平滑。

## 3. 时间步长处理

对已有轨迹 $T_i$，当前帧时间为 $t_k$，轨迹上次时间为 $\tau_i$。代码先计算预测步长：

$$
\Delta t_i =
\begin{cases}
\Delta t_{\text{default}}, & t_k=0\ \text{or}\ \tau_i=0 \\
t_k-\tau_i, & t_k-\tau_i>0 \\
\Delta t_{\text{default}}, & t_k-\tau_i\le 0
\end{cases}
$$

然后做上限裁剪：

$$
\Delta t_i \leftarrow \min(\Delta t_i,\; \Delta t_{\max})
$$

对应参数：

1. `tracking_default_dt`：$\Delta t_{\text{default}}$
2. `tracking_max_dt`：$\Delta t_{\max}$

这个裁剪很重要：如果点云时间戳跳变或长时间暂停，滤波器不会一次性外推一个很大的距离，而是最多按 $\Delta t_{\max}$ 做一步预测。

## 4. CA3D 状态定义

当前 `MotionModelType::CA3D` 的状态是 9 维：

$$
x =
\begin{bmatrix}
p \\
v \\
a
\end{bmatrix}
=
\begin{bmatrix}
p_x & p_y & p_z & v_x & v_y & v_z & a_x & a_y & a_z
\end{bmatrix}^T
$$

其中：

1. $p\in\mathbb{R}^3$ 是目标中心位置。
2. $v\in\mathbb{R}^3$ 是目标中心速度。
3. $a\in\mathbb{R}^3$ 是目标中心加速度。

观测只测位置：

$$
z =
\begin{bmatrix}
z_x & z_y & z_z
\end{bmatrix}^T
$$

观测矩阵为：

$$
H =
\begin{bmatrix}
I_3 & 0_3 & 0_3
\end{bmatrix}
$$

所以预测状态到观测空间的映射是：

$$
\hat{z}=Hx=p
$$

## 5. CA3D 状态转移矩阵

constant-acceleration 离散模型假设在一个时间步 $\Delta t$ 内加速度近似保持常量：

$$
p_k = p_{k-1} + v_{k-1}\Delta t + \frac{1}{2}a_{k-1}\Delta t^2
$$

$$
v_k = v_{k-1} + a_{k-1}\Delta t
$$

$$
a_k = a_{k-1}
$$

写成矩阵：

$$
x_k^- = F(\Delta t)x_{k-1}^+
$$

其中：

$$
F(\Delta t)=
\begin{bmatrix}
I_3 & \Delta t I_3 & \frac{1}{2}\Delta t^2 I_3 \\
0_3 & I_3 & \Delta t I_3 \\
0_3 & 0_3 & I_3
\end{bmatrix}
$$

代码中 `transition_` 的非零块就是：

$$
F_{p,v}=\Delta t I_3,\quad
F_{p,a}=\frac{1}{2}\Delta t^2 I_3,\quad
F_{v,a}=\Delta t I_3
$$

## 6. 过程噪声 Q

当前 CA3D 模型使用白 jerk 驱动的离散过程噪声。设 jerk 噪声方差为：

$$
q = \sigma_j^2
$$

对应参数：

$$
\sigma_j = \texttt{tracking\_ca\_jerk\_sigma}
$$

单轴状态为：

$$
x^{(1D)}=[p,\ v,\ a]^T
$$

单轴过程噪声模板是：

$$
Q_{\text{axis}}(\Delta t)=q
\begin{bmatrix}
\frac{\Delta t^5}{20} & \frac{\Delta t^4}{8} & \frac{\Delta t^3}{6} \\
\frac{\Delta t^4}{8} & \frac{\Delta t^3}{3} & \frac{\Delta t^2}{2} \\
\frac{\Delta t^3}{6} & \frac{\Delta t^2}{2} & \Delta t
\end{bmatrix}
$$

因为完整状态按 $[p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z]$ 分块排列，所以完整基础过程噪声可以写成：

$$
Q_0(\Delta t)=q
\begin{bmatrix}
\frac{\Delta t^5}{20}I_3 & \frac{\Delta t^4}{8}I_3 & \frac{\Delta t^3}{6}I_3 \\
\frac{\Delta t^4}{8}I_3 & \frac{\Delta t^3}{3}I_3 & \frac{\Delta t^2}{2}I_3 \\
\frac{\Delta t^3}{6}I_3 & \frac{\Delta t^2}{2}I_3 & \Delta t I_3
\end{bmatrix}
$$

这个矩阵表达的是：

1. jerk 噪声会积分到加速度、速度和位置。
2. 时间步长越大，位置方差会按 $\Delta t^5$ 量级增长。
3. 三个坐标轴之间暂不建模相关性，因此非同轴的交叉项为 0。

## 7. 测量噪声 R

位置观测噪声方差为：

$$
r=\texttt{tracking\_ca\_position\_noise}
$$

基础测量噪声矩阵是：

$$
R_0=rI_3
$$

含义是：

1. detection center 的 $x/y/z$ 三轴测量方差相同。
2. 三轴测量误差暂不建模相关性。
3. bbox 尺寸误差不在 $R$ 中，因为尺寸不作为 Kalman 观测。

## 8. 新轨迹初始化

创建新轨迹时，滤波器先用零状态初始化：

$$
x_0 =
\begin{bmatrix}
0\\0\\0\\0\\0\\0\\0\\0\\0
\end{bmatrix}
$$

初始协方差是分块对角矩阵：

$$
P_0 =
\begin{bmatrix}
c_p I_3 & 0 & 0 \\
0 & c_v I_3 & 0 \\
0 & 0 & c_a I_3
\end{bmatrix}
$$

对应参数：

1. $c_p=\texttt{tracking\_ca\_initial\_position\_cov}$
2. $c_v=\texttt{tracking\_ca\_initial\_velocity\_cov}$
3. $c_a=\texttt{tracking\_ca\_initial\_acceleration\_cov}$

随后 `createTrack()` 会立刻用首帧 detection 中心做一次 Kalman update：

$$
z_0=c_j
$$

注意：这不是直接执行 $p_0=z_0$ 的硬赋值，而是一次标准 Kalman 校正。由于初始状态为 0，第一次 innovation 为：

$$
y_0=z_0-Hx_0=z_0
$$

初始位置更新近似为：

$$
p_0^+ =
\frac{c_p}{c_p+r}z_0
$$

如果 $c_p \gg r$，新轨迹位置会非常接近首帧检测质心；如果 $c_p$ 与 $r$ 接近，则位置会更保守。

当前默认配置中：

$$
c_p=0.1,\quad r=0.1^2=0.01
$$

所以第一次位置权重约为：

$$
\frac{0.1}{0.1+0.01}\approx 0.909
$$

也就是说，新轨迹位置会被拉到很接近观测中心，但数学上仍然是滤波更新。

## 9. Kalman 预测

对每条已有轨迹，在数据关联前先预测到当前帧：

$$
x_i^- = F_i x_i^+
$$

$$
P_i^- = F_i P_i^+ F_i^T + \lambda_Q Q_i
$$

其中：

1. $F_i=F(\Delta t_i)$。
2. $\lambda_Q=\texttt{tracking\_prediction\_cov\_multiplier}$。
3. $Q_i$ 是当前使用的过程噪声矩阵。

在当前 tracker 调用路径中，每帧 `predictTracks()` 都会先调用：

$$
\texttt{setDt}(\Delta t_i)
$$

而 `setDt()` 只负责刷新当前步长下的运动模型参数。真正进入预测公式的过程噪声矩阵由滤波器当前持有的 `adaptive_process_noise_` 决定；当关闭自适应噪声时，它退回当前 $\Delta t$ 下的基础 $Q_0(\Delta t)$。因此当前预测阶段的有效形式可以理解为：

$$
P_i^- = F_i P_i^+ F_i^T + Q_i^{adaptive}
$$

其中 $Q_i^{adaptive}$ 由后续观测更新阶段基于 innovation 统计估计，并保留到下一次 predict 使用。

## 10. 数据关联代价

预测完成后，对每个 detection-track 组合计算 3D 位置马氏距离平方。

设 detection 中心为：

$$
z_j=c_j
$$

轨迹预测位置为：

$$
\hat{p}_i=H x_i^-
$$

位置 innovation 为：

$$
\nu_{j,i}=z_j-\hat{p}_i
$$

取预测协方差的位置块：

$$
P_{p,i}=P_i^-(0:2,0:2)
$$

代码会先强制位置协方差对称，并加入很小的正则项：

$$
\tilde{P}_{p,i}
=
\frac{1}{2}(P_{p,i}+P_{p,i}^T)
+\epsilon I_3
$$

其中：

$$
\epsilon=10^{-6}
$$

关联代价为：

$$
c_{j,i}=\nu_{j,i}^T \tilde{P}_{p,i}^{-1}\nu_{j,i}
$$

这个量是马氏距离平方。它不是普通欧氏距离，而是会被轨迹当前的位置不确定性调节：

1. 如果轨迹位置协方差小，同样的中心偏差会得到更大的代价。
2. 如果轨迹已经 coast，位置协方差通常会变大，同样的中心偏差会更容易通过 gate。

## 11. 卡方 gate

因为观测维度是 3，代码用 3 自由度卡方分布把置信度转换为 gate：

$$
\gamma =
F^{-1}_{\chi^2_3}(\alpha)
$$

其中：

1. $\alpha=\texttt{tracking\_association\_gate\_confidence}$。
2. $F^{-1}_{\chi^2_3}$ 是 3 自由度卡方分布分位数函数。

普通轨迹 gate：

$$
c_{j,i}\le\gamma
$$

处于漏检 coast 状态的轨迹，即 $\text{miss}_i>0$，使用放宽 gate：

$$
c_{j,i}\le \gamma\cdot \rho_{\text{coast}}
$$

其中：

$$
\rho_{\text{coast}}=\texttt{tracking\_coasting\_gate\_relax\_factor}
$$

如果组合不满足 gate，cost matrix 中写入 `invalid_cost`，后续 Hungarian 匹配不会把它作为真实合法匹配返回。

## 12. Hungarian 全局匹配

设当前帧 detection 数量为 $M$，已有轨迹数量为 $N$。先构造原始代价矩阵：

$$
C\in\mathbb{R}^{M\times N}
$$

其中：

$$
C_{j,i} =
\begin{cases}
c_{j,i}, & c_{j,i}\ \text{通过 gate} \\
C_{\text{invalid}}, & \text{否则}
\end{cases}
$$

代码再把它扩展为方阵，维度为：

$$
D=M+N
$$

扩展方阵的作用是让 Hungarian 算法可以同时表达：

1. detection 匹配真实 track。
2. detection 没有匹配任何 track。
3. track 没有匹配任何 detection。

dummy 匹配代价为：

$$
C_{\text{unmatched}}=\frac{1}{2}C_{\text{invalid}}
$$

Hungarian 算法求解：

$$
\pi^*=\arg\min_{\pi}\sum_{r=1}^{D}\tilde{C}_{r,\pi(r)}
$$

其中 $\pi$ 是一个一对一分配。最后代码只保留满足下面条件的真实匹配：

$$
0\le j<M,\quad 0\le i<N,\quad C_{j,i}<C_{\text{invalid}}
$$

所以 dummy 匹配、未通过 gate 的组合都会在输出 matches 时被过滤。

## 13. 匹配成功后的轨迹更新

如果 detection $d_j$ 匹配轨迹 $T_i$，先用 detection center 做 Kalman update。当前 detection center 是点簇质心，不是 AABB 几何中心。

预测观测：

$$
\hat{z}=Hx_i^-
$$

innovation：

$$
y=z_j-\hat{z}
$$

innovation covariance：

$$
S=H P_i^- H^T + R_i
$$

Kalman gain：

$$
K=P_i^-H^TS^{-1}
$$

状态更新：

$$
x_i^+=x_i^-+Ky
$$

协方差使用 Joseph 形式：

$$
P_i^+
=(I-KH)P_i^-(I-KH)^T + KR_iK^T
$$

Joseph 形式比简单的 $(I-KH)P$ 更稳，更容易保持协方差半正定和对称。

然后更新 bbox：

1. bbox 中心由滤波后位置覆盖：

$$
B_i.center \leftarrow p_i^+
$$

2. bbox 尺寸做指数平滑：

$$
s_i^+ =
\beta s_j + (1-\beta)s_i^-
$$

其中：

$$
\beta=\texttt{tracking\_box\_size\_smoothing\_alpha}
$$

尺寸不进入 Kalman 状态，原因是点云稀疏、遮挡和视角变化会让 AABB 尺寸比中心更抖；当前实现只平滑它，不用它反向影响运动状态。

最后更新生命周期计数：

$$
\text{age}_i\leftarrow \text{age}_i+1
$$

$$
\text{hits}_i\leftarrow \text{hits}_i+1
$$

$$
\text{miss}_i\leftarrow 0
$$

并把 `last_stamp` 改为当前帧时间。

## 14. 自适应测量噪声与过程噪声

每次 Kalman update 完成后，代码用本次 innovation 更新自适应噪声估计。设本次 innovation 为：

$$
y=[y_x,y_y,y_z]^T
$$

先计算三轴平均 innovation 能量：

$$
e_k=\frac{1}{3}\Vert y\Vert^2
$$

维护最近 $W$ 个 $e_k$ 的窗口：

$$
\mathcal{E}_k=\{e_{k-W+1},\dots,e_k\}
$$

其中：

$$
W=\texttt{tracking\_adaptive\_window\_size}
$$

样本协方差：

$$
C_{\gamma,k}=\frac{1}{|\mathcal{E}_k|}\sum_{e\in\mathcal{E}_k}ee^T
$$

先估计测量噪声：

$$
\hat{R}_k=C_{\gamma,k}-HP_k^-H^T
$$

然后只取对角项，并做非负截断、EMA 和下限约束：

$$
R_k(i,i)=
\max\left(
R_{\min}(i,i),
\alpha_R\max(0,\hat{R}_k(i,i)) + (1-\alpha_R)R_{k-1}(i,i)
\right)
$$

再用更新后的 $R_k$ 和临时增益 $K_k^{temp}$ 估计过程噪声：

$$
\hat{Q}_k=
K_k^{temp}
\left(
C_{\gamma,k}-H(P_k^- - Q_{k-1})H^T-R_k
\right)
\left(K_k^{temp}\right)^T
$$

同样只更新对角项：

$$
Q_k(i,i)=
\max\left(
Q_{\min}(i,i),
\alpha_Q\max(0,\hat{Q}_k(i,i)) + (1-\alpha_Q)Q_{k-1}(i,i)
\right)
$$

对应参数：

1. $\alpha_Q=\texttt{tracking\_adaptive\_alpha}$
2. $\alpha_R=\texttt{tracking\_adaptive\_r\_alpha}$
3. $Q_{\min}=Q_0\cdot\texttt{tracking\_adaptive\_min\_noise\_ratio}$
4. $R_{\min}=R_0\cdot\texttt{tracking\_adaptive\_min\_noise\_ratio}$

如果 `tracking_adaptive_window_size <= 1`，则自适应噪声关闭：

$$
Q_k^{adaptive}=Q_0,\quad R_k^{adaptive}=R_0
$$

实现细节再强调一次：`R_k^{adaptive}` 会立即作用在当前这次 update；`Q_k^{adaptive}` 会保留到下一次 predict 使用。当前实现和 LDOT 一样，`setDt()` 不会把已经估计出来的 adaptive Q 重新按基础 $Q_0(\Delta t)$ 映射成“统一缩放比”。

## 15. 协方差 clamp

每次 predict 和 update 后都会执行 `clampCovariance()`。

第一步，强制对称：

$$
P\leftarrow \frac{1}{2}(P+P^T)
$$

第二步，对每个状态维度的对角线做非负约束：

$$
P_{ll}\leftarrow \max(P_{ll},0)
$$

第三步，对位置、速度、加速度三类对角线分别应用上限：

$$
L_l =
\begin{cases}
L_p, & l\in p_x,p_y,p_z \\
L_v, & l\in v_x,v_y,v_z \\
L_a, & l\in a_x,a_y,a_z
\end{cases}
$$

其中：

1. $L_p=\texttt{tracking\_max\_pos\_cov}$
2. $L_v=\texttt{tracking\_max\_vel\_cov}$
3. $L_a=\texttt{tracking\_max\_acc\_cov}$

如果某个对角线超过上限：

$$
P_{ll}>L_l
$$

则记录缩放系数：

$$
s_l=\sqrt{\frac{L_l}{\max(P_{ll},10^{-9})}}
$$

否则：

$$
s_l=1
$$

最后统一左右乘：

$$
P\leftarrow DPD
$$

其中：

$$
D=\operatorname{diag}(s_1,\dots,s_9)
$$

这样做不是只截断对角线，而是同步缩放相关项，尽量保留协方差块内部的相对相关结构。

## 16. 未匹配 detection 的建轨抑制

Hungarian 匹配后，未匹配 detection 不一定立即创建新轨迹。代码会先检查它是否仍然靠近已有轨迹，以减少目标中心抖动、bbox 抖动或短时遮挡造成的裂轨。

对未匹配 detection $d_j$ 和已有轨迹 $T_i$，先计算三维中心距离：

$$
d_{3d}(j,i)
=
\sqrt{
(c_{j,x}-c_{i,x})^2+
(c_{j,y}-c_{i,y})^2+
(c_{j,z}-c_{i,z})^2
}
$$

如果：

$$
d_{3d}(j,i)\le d_{\text{sup}}
$$

则抑制建轨。其中：

$$
d_{\text{sup}}=\texttt{tracking\_spawn\_suppression\_distance}
$$

代码还计算水平 AABB IoU。设两个水平矩形面积分别为 $A_j,A_i$，交集面积为 $A_{\cap}$，并集面积为：

$$
A_{\cup}=A_j+A_i-A_{\cap}
$$

水平 IoU：

$$
\operatorname{IoU}_{xy}(j,i)=\frac{A_{\cap}}{A_{\cup}}
$$

如果：

$$
\operatorname{IoU}_{xy}(j,i)\ge \eta_{\text{sup}}
$$

也抑制建轨。其中：

$$
\eta_{\text{sup}}=\texttt{tracking\_spawn\_suppression\_iou\_threshold}
$$

两个判据任意一个成立，就不会为该 detection 创建新 ID：

$$
\text{suppressed}
=
\left(d_{3d}\le d_{\text{sup}}\right)
\lor
\left(\operatorname{IoU}_{xy}\ge \eta_{\text{sup}}\right)
$$

如果两个参数都配置为 0，则对应抑制判据关闭。

## 17. 未匹配 detection 创建新轨迹

如果 detection 未匹配，且没有被建轨抑制挡住，则创建新轨迹：

$$
\text{id}_{new}=\texttt{next\_track\_id}
$$

然后：

$$
\texttt{next\_track\_id}\leftarrow \texttt{next\_track\_id}+1
$$

新轨迹初始化步骤是：

1. 构造 `TrackState`。
2. 创建内部 `MultiModelKalmanFilter`。
3. 用 detection center 做一次 Kalman update。
4. bbox 取 detection bbox。
5. bbox center 再用滤波器位置覆盖。
6. 设置生命周期：

$$
\text{age}=1,\quad \text{hits}=1,\quad \text{miss}=0
$$

7. 写入一条 matched history sample。

## 18. 未匹配 track 的 coast

对没有匹配任何 detection 的轨迹，代码不会立即删除，而是进入 coast 状态。

注意：coast 之前，本帧开头已经对所有 track 执行过一次 predict。因此 coast 阶段不再额外预测，只更新生命周期和历史：

$$
\text{age}_i\leftarrow \text{age}_i+1
$$

$$
\text{miss}_i\leftarrow \text{miss}_i+1
$$

$$
\tau_i\leftarrow t_k
$$

$$
B_i.center\leftarrow Hx_i^-
$$

并追加一条：

$$
\text{matched}=false
$$

的历史样本。

coast 的直观含义是：

1. 轨迹短期没被观测到，但仍按运动模型保持存在。
2. 下一帧数据关联时，$\text{miss}_i>0$ 会触发更宽的 gate。
3. 如果后续重新匹配成功，`missed_frames` 会清零。

## 19. 轨迹删除

轨迹删除规则是：

$$
\text{delete}(T_i)
\iff
\text{miss}_i > M_{\max}
$$

其中：

$$
M_{\max}=\texttt{tracking\_max\_missed\_frames}
$$

注意这里是严格大于，不是大于等于。因此如果参数为 5，轨迹会在连续漏检计数变成 6 时被删除。

## 20. 对外发布条件

`buildOutput()` 会先按轨迹 ID 排序，然后只发布命中次数达到阈值的轨迹：

$$
\text{publish}(T_i)
\iff
\text{hits}_i\ge H_{\min}
$$

其中：

$$
H_{\min}=\texttt{tracking\_min\_hits\_to\_publish}
$$

输出消息中每个目标为：

$$
o_i =
\left(
\text{id}_i,\;
B_i.center,\;
B_i.size,\;
v_i
\right)
$$

其中速度来自滤波器状态：

$$
v_i=x_i(3:5)
$$

当前输出不包含：

1. acceleration
2. covariance
3. prediction horizon
4. class/type
5. track confidence

另外，`buildOutput()` 不检查 `missed_frames == 0`。所以只要轨迹还没被删除，且历史命中次数达到阈值，coasting 轨迹也可能继续出现在 `/ldop/dynamic_objects` 中。

## 21. 轨迹历史与 marker

每次 matched 或 coast 后都会追加历史样本：

$$
h_k=(t_k,\;p_k,\;v_k,\;\text{matched}_k)
$$

历史长度上限为：

$$
|\mathcal{H}_i|\le H_{\text{history}}
$$

其中：

$$
H_{\text{history}}=\texttt{tracking\_history\_size}
$$

如果超过上限，删除最旧样本：

$$
\mathcal{H}_i
\leftarrow
\text{last}_{H_{\text{history}}}(\mathcal{H}_i)
$$

marker 输出包括：

1. `DELETEALL`：每帧先清空旧 marker。
2. `dynamic_track_history`：历史轨迹线，至少 2 个点才发布。
3. `dynamic_track_head`：当前轨迹头部球体。

如果轨迹处于 coast 状态，即：

$$
\text{miss}_i>0
$$

marker 透明度会降低，用于可视化区分“当前帧未匹配到观测”的轨迹。

## 22. 一帧完整流程

把 `processDynamicTracks()` 写成数学化伪代码：

```text
输入：当前帧 header、detections D_k
状态：上一帧后保留的 tracks T_{k-1}

1. 对每条已有轨迹 T_i：
   1.1 根据 header.stamp 和 track.last_stamp 计算 dt_i
   1.2 setDt(dt_i)
   1.3 predict: x_i^- = F_i x_i^+, P_i^- = F_i P_i^+ F_i^T + lambda_Q Q_i
   1.4 bbox.center = H x_i^-

2. 对每个 detection-track 组合：
   2.1 计算 innovation nu_{j,i} = z_j - H x_i^-
   2.2 计算马氏距离平方 c_{j,i}
   2.3 用卡方 gate 过滤非法组合

3. 用 Hungarian 算法求全局最小匹配。

4. 对匹配成功的 detection-track：
   4.1 Kalman update
   4.2 平滑 bbox size
   4.3 hits += 1, age += 1, missed_frames = 0
   4.4 追加 matched history

5. 对未匹配 detection：
   5.1 若贴近已有轨迹或水平 IoU 过高，则抑制建轨
   5.2 否则创建新 TrackState 和新 filter

6. 对未匹配 track：
   6.1 age += 1
   6.2 missed_frames += 1
   6.3 追加 unmatched/coast history

7. 删除 missed_frames > max_missed_frames 的轨迹。

8. 构建输出：
   8.1 hits 不足阈值的轨迹不发布
   8.2 发布 id、center、size、velocity
   8.3 构建 history/head markers
```

## 23. 参数到数学量的对应关系

| 参数 | 数学符号 | 含义 |
| --- | --- | --- |
| `tracking_default_dt` | $\Delta t_{\text{default}}$ | 时间戳无效或非递增时的回退步长 |
| `tracking_max_dt` | $\Delta t_{\max}$ | 单帧预测步长上限 |
| `tracking_ca_jerk_sigma` | $\sigma_j$ | CA 模型 jerk 噪声标准差 |
| `tracking_ca_position_noise` | $r$ | 点簇质心位置观测噪声方差 |
| `tracking_ca_initial_position_cov` | $c_p$ | 新轨迹初始位置协方差 |
| `tracking_ca_initial_velocity_cov` | $c_v$ | 新轨迹初始速度协方差 |
| `tracking_ca_initial_acceleration_cov` | $c_a$ | 新轨迹初始加速度协方差 |
| `tracking_association_gate_confidence` | $\alpha$ | 3D 卡方 gate 置信度 |
| `tracking_coasting_gate_relax_factor` | $\rho_{\text{coast}}$ | coast 轨迹 gate 放宽倍数 |
| `tracking_box_size_smoothing_alpha` | $\beta$ | bbox 尺寸 EMA 平滑系数 |
| `tracking_spawn_suppression_distance` | $d_{\text{sup}}$ | 马氏距离 gate 失败后的第一层回退匹配阈值：distance 先过，才继续看 IoU；同时也是最终未匹配 detection 的建轨抑制阈值 |
| `tracking_spawn_suppression_iou_threshold` | $\eta_{\text{sup}}$ | 马氏距离 gate 失败后的第二层回退匹配阈值：只有 distance 已经过阈值时，才检查 IoU；同时也是最终未匹配 detection 的建轨抑制阈值 |
| `tracking_max_missed_frames` | $M_{\max}$ | 删除轨迹前允许的连续漏检帧数 |
| `tracking_min_hits_to_publish` | $H_{\min}$ | 对外发布前需要的命中次数 |
| `tracking_history_size` | $H_{\text{history}}$ | 每条轨迹保留的历史样本数 |
| `tracking_adaptive_window_size` | $W$ | innovation 能量统计窗口 |
| `tracking_adaptive_alpha` | $\alpha_Q$ | 自适应 Q 缩放 EMA 系数 |
| `tracking_adaptive_r_alpha` | $\alpha_R$ | 自适应 R 缩放 EMA 系数 |
| `tracking_adaptive_min_noise_ratio` | $\rho_{\min}$ | 自适应噪声缩放下限 |
| `tracking_max_pos_cov` | $L_p$ | 位置协方差对角线上限 |
| `tracking_max_vel_cov` | $L_v$ | 速度协方差对角线上限 |
| `tracking_max_acc_cov` | $L_a$ | 加速度协方差对角线上限 |

## 24. 当前实现的关键语义

1. 当前“多模型”是结构预留，不是 IMM。代码没有多个模型概率、模型混合、模型转移矩阵或多模型融合。
2. 当前唯一模型是 `CA3D`，状态维度固定为 9，观测维度固定为 3。
3. 跟踪关联默认使用位置马氏距离平方；只有马氏距离被 gate 拒绝时，才会退回到“先 distance、后 IoU”的两层阈值做次优匹配。
4. bbox IoU 当前不参与主匹配 cost，只作为 distance 已通过后的第二层回退条件，以及最终未匹配 detection 的建轨抑制条件。
5. bbox size 不进入 Kalman 状态，只做指数平滑。
6. 新轨迹会直接用首帧 detection center（点簇质心）初始化滤波器位置状态，速度与加速度仍从零起步。
7. coasting 轨迹会继续发布，前提是它已经满足 hits 阈值且尚未超过漏检删除阈值。
8. `max_missed_frames` 的删除条件是 `missed_frames > max_missed_frames`。
9. 自适应 R 会影响当前与后续观测更新；自适应 Q 会持续影响后续预测，并在当前 `update()` 内同步修正本次使用的 `P_{pred}`。

## 25. 一句话总结

当前跟踪模块可以概括为：

$$
\text{点簇质心观测}
\xrightarrow{\text{CA3D Kalman predict}}
\text{预测轨迹}
\xrightarrow{\chi^2\ \text{gate}+\text{Hungarian}}
\text{全局关联}
\xrightarrow{\text{update/create/coast/delete}}
\text{稳定 ID、平滑 bbox、估计速度}
$$

其中 `DynamicObjectTracker` 负责“哪条 detection 属于哪条轨迹，以及轨迹是否存在”，`MultiModelKalmanFilter` 负责“单条轨迹在数学上如何预测和校正位置、速度、加速度”。
