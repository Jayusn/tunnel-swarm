"""RRT 的 MINCO-S3 后端：把二维避障折线变成可以按时间跟踪的平滑轨迹。

建议阅读顺序：
1. 先看 MincoTrajectory.__init__ 中的第 1～10 步，了解完整数据流。
2. 再看 coefficient_map：途经点怎样决定五次多项式系数。
3. 然后看 corridor 和 certify_curve：曲线怎样保持避障。
4. 最后看 sample：飞行程序怎样取出某一时刻的位置和速度。

输入 path 是 RRT 输出的 [(x0,y0), ...]，单位为米；scene 提供障碍和安全余量。
输出对象的 sample(t) 返回二维位置、速度；飞行高度由外部 ROS 程序保持。
本文件只计算轨迹，不负责解锁、起飞或向 PX4 发送指令。

这里的 S3 表示最小化三阶导数 jerk 的平方积分：
    J = integral ||d³p/dt³||² dt
jerk 是加速度的变化率。减小 J 是改善平滑性的目标，不等于设置 jerk 硬上限。
每段轨迹为五次多项式，内部连接处满足 C4 连续：位置、速度、加速度、jerk、
四阶导数在左右两段一致；起点和终点的速度、加速度设为零。

计算使用归一化时间 u = t_local / T，每段 u 都从 0 走到 1：
    p(u) = c0 + c1*u + c2*u² + ... + c5*u⁵
因此系数按低次到高次排列。对真实时间求 d 阶导数，还需要除以 T**d。

实现范围：给定分段时间，用 MINCO 系数映射和安全走廊约束优化内部途经点；
外层逐段调整相对时间，每次重新优化位置；最终统一缩放并复验速度、加速度。
当前采用稠密矩阵求解，不是原版 MINCO 的带状线性复杂度实现。
"""
import bisect
import math
import time

import numpy as np
from scipy.linalg import solve, cholesky, solve_triangular
from scipy.optimize import linprog, minimize


def bernstein_norm_bound(control, depth=4):
    """细分导数曲线，求连续范数上界，而非采样最大值。

    de Casteljau 二分不改变曲线；每个子区间的曲线都在其控制点凸包内。
    默认细分 4 层（16 个子区间），再取所有控制点范数的最大值。
    输入已是真实时间导数，细分时不要再乘 2 或除以子区间时长。
    """
    control = np.asarray(control, dtype=float)
    if control.ndim != 2 or not len(control) or not np.isfinite(control).all():
        raise ValueError('Invalid Bernstein derivative controls')
    if not isinstance(depth, int) or not 0 <= depth <= 10:
        raise ValueError('Invalid subdivision depth')
    blocks = control[None, :, :]
    for _ in range(depth):
        level = blocks
        left, right = [level[:, 0]], [level[:, -1]]
        while level.shape[1] > 1:
            level = .5*level[:, :-1] + .5*level[:, 1:]
            left.append(level[:, 0]); right.append(level[:, -1])
        blocks = np.concatenate((np.stack(left, axis=1),
                                 np.stack(right[::-1], axis=1)), axis=0)
    # 浮点运算余量；最终限速缩放仍另保留原有的 0.1% 余量。
    padding = 64*np.finfo(float).eps*max(1., float(np.max(np.abs(control))))
    return float(np.max(np.linalg.norm(blocks, axis=2))) + padding


def basis(u, derivative=0):
    """生成多项式求值行向量；它乘系数矩阵即可得到位置或 u 的导数。

    derivative=0 时为 [1,u,u²,u³,u⁴,u⁵]；=1 时为 [0,1,2u,3u²,4u³,5u⁴]。
    一次矩阵乘法同时算出 x、y 两个方向，避免分别写两套公式。
    """
    return np.array([0. if k < derivative else
                     math.factorial(k)/math.factorial(k-derivative)*u**(k-derivative)
                     for k in range(6)])


def coefficient_map(durations):
    """建立“途经点位置 -> 最小 jerk 多项式系数”的线性映射 M。

    N 段轨迹对应 N+1 个途经点，每段每个方向有 6 个未知系数。
    若 knots.shape=(N+1,2)，则 M @ knots 得到 (6N,2) 的 x/y 系数。
    给定途经点和时长后，下列边界、连续性方程就确定最小 jerk 轨迹。
    """
    durations = np.asarray(durations, dtype=float)
    if not len(durations) or not np.isfinite(durations).all() or np.any(durations <= 0):
        raise ValueError('MINCO durations must be positive and finite')
    n = len(durations)
    matrix = np.zeros((6*n, 6*n))
    rhs = np.zeros((6*n, n+1))
    row = 0
    # 起点 3 个方程：位置等于第一个途经点，速度=0，加速度=0（PVA 边界）。
    for d in range(3):
        matrix[row, :6] = basis(0., d)/durations[0]**d
        if d == 0: rhs[row, 0] = 1.
        row += 1
    for i in range(n-1):
        # 每个内部接点 6 个方程：前段终点、后段起点都等于该途经点；
        # 再要求 1～4 阶真实时间导数相等，合起来使轨迹 C4 连续。
        matrix[row, 6*i:6*i+6] = basis(1.)
        rhs[row, i+1] = 1.; row += 1
        matrix[row, 6*(i+1):6*(i+2)] = basis(0.)
        rhs[row, i+1] = 1.; row += 1
        for d in range(1, 5):
            matrix[row, 6*i:6*i+6] = basis(1., d)/durations[i]**d
            matrix[row, 6*(i+1):6*(i+2)] = -basis(0., d)/durations[i+1]**d
            row += 1
    for d in range(3):
        matrix[row, -6:] = basis(1., d)/durations[-1]**d
        if d == 0: rhs[row, -1] = 1.
        row += 1
    # 上面最后 3 行是终点 PVA 约束：到达目标后速度、加速度都为零。
    # 方程为 matrix @ 系数 = rhs @ 途经点，所以映射 M = solve(matrix,rhs)。
    return solve(matrix, rhs)


def bernstein_map():
    """把五次幂基系数换成 6 个 Bernstein 控制点的转换矩阵。

    转换前后是同一条五次曲线，不改变轨迹形状。
    使用这种表示，是因为整条曲线一定在控制点的凸包内：
    只要控制点都在同一个凸安全走廊内，就能约束整段曲线避障。
    这里不是调用旧的二次 Bézier 圆滑后端。
    """
    return np.array([[math.comb(k,j)/math.comb(5,j) if j <= k else 0.
                      for j in range(6)] for k in range(6)])


def corridor(a, b, scene):
    """给 RRT 的一条边 a->b 建立凸安全走廊，返回不等式 A @ p <= h。

    二维中每条不等式代表一条直线的一侧；多条不等式的交集就是凸走廊。
    走廊包含原线段，并把每个障碍物挡在走廊外面。
    scene.boxes 已经膨胀过，包含机体与跟踪余量，这里不重复膨胀。
    """
    a, b = np.asarray(a), np.asarray(b)
    if not scene.free(a,b): raise ValueError('MINCO input edge collides')
    xmin,xmax,ymin,ymax = scene.bounds
    eps = 1e-9
    # 先用地图的左、右、上、下边界限制走廊；eps 避免恰好贴边。
    normals = [(1.,0.),(-1.,0.),(0.,1.),(0.,-1.)]
    bounds = [xmax-scene.margin-eps, -xmin-scene.margin-eps,
              ymax-scene.margin-eps, -ymin-scene.margin-eps]
    delta = b-a; squared = float(delta@delta)
    for left,right,bottom,top in scene.boxes:
        vertices = np.array([(left,bottom),(left,top),(right,bottom),(right,top)])
        candidates = []
        # 找线段与矩形的最近点对：检查端点到矩形、矩形角点到线段的投影。
        # 后面的 p 在路径边上，q 在障碍物上，p-q 指向可通行的一侧。
        for p in (a,b):
            q = np.clip(p, (left,bottom), (right,top))
            candidates.append((float(np.linalg.norm(p-q)), p, q))
        for q in vertices:
            u = np.clip(float((q-a)@delta)/squared, 0., 1.)
            p = a+u*delta
            candidates.append((float(np.linalg.norm(p-q)), p, q))
        distance,p,q = min(candidates, key=lambda item:item[0])
        if distance < 1e-10: raise ValueError('MINCO corridor has no obstacle separation')
        normal = (p-q)/distance
        # 沿 normal 投影后，support 是障碍物最靠近路径一侧的位置，
        # gap 是原线段与这条支撑线的间隙。
        support = float(np.max(vertices@normal))
        gap = min(float(a@normal),float(b@normal))-support
        if gap <= 0: raise ValueError('MINCO could not separate obstacle')
        normals.append(-normal)
        # 要求 normal·位置 >= support + 0.1*gap，留出间隙的 10%。
        # 乘以 -1 后统一成 A @ p <= h，便于交给优化器。
        bounds.append(-support-.1*gap)
    return np.asarray(normals), np.asarray(bounds)


def compact_halfspaces(normals, limits):
    """只合并完全相同的法向量，保留最紧的界；不靠近似平行判断删除约束。

    A 相同且 b1<=b2 时，A p<=b1 蕴含 A p<=b2，故可行域保持不变。
    其他一般冗余约束保留，避免逐条 LP 证明产生额外开销与数值误删。
    """
    rows={}
    for normal,limit in zip(normals,limits):
        key=tuple(float(v) for v in normal)
        rows[key]=min(rows.get(key,float('inf')),float(limit))
    return np.asarray(list(rows)),np.asarray(list(rows.values()))


def curve_length_value_gradient(knots, derivative_map, weights):
    """Gauss 积分估计整条多项式弧长及对位置点的梯度，不是点间折线长度。

    使用归一化参数 u：弧长为 integral ||dp/du|| du，时间因子相消。
    极小平滑项避免零速度处除零；此积分用于目标，不替代连续安全认证。
    """
    tangent = derivative_map@knots
    norms = np.sqrt(np.sum(tangent*tangent, axis=1)+1e-16)
    value = float(weights@norms)
    gradient = derivative_map.T@((weights/norms)[:,None]*tangent)
    return value, gradient


def speed_penalty_value_gradient(knots, velocity_map, weights, speed):
    """归一化时间积分 mean(max(||v||²/v_limit² - 1, 0)²) 及位置梯度。

    velocity_map 已含各段 1/T；weights 是 dt/总时间的 Gauss 权重。
    仅超速区间有代价，阈值处一阶连续；解析梯度避免数值差分。
    采样积分仅用于软目标，最终连续动力学认证和缩放仍独立执行。
    """
    velocity=velocity_map@knots
    excess=np.maximum(np.sum(velocity*velocity,axis=1)/speed**2-1.,0.)
    value=float(weights@(excess*excess))
    gradient=velocity_map.T@((4*weights*excess/speed**2)[:,None]*velocity)
    return value,gradient


class MincoTrajectory:
    """构造时完成整条轨迹的优化和认证，飞行时只调用 sample(t)。

    常用成员：
      coefficients[i,k,:]：第 i 段 u**k 的二维系数，形状 (N,6,2)。
      durations[i]：第 i 段实际执行时间，单位秒。
      times[i]：第 i 段的累计起始时间；total_time 为整条轨迹时间。
      points / s：显示用的离散点 / 其累计折线长度，用于预览和统计。
      velocity_bound / acceleration_bound：整条参考轨迹的保守认证上界。
    """
    def __init__(self, path, scene, speed=.6, acceleration=.8, time_optimization=True,
                 time_evaluations=8, pair_search=False, pair_evaluations=8, position_length_weight=1., terminal_braking=False, start_turn_braking=False, position_ftol=1e-8, compact_constraints=True, improve_initial=True, position_speed_weight=0.):
        if not all(math.isfinite(v) and v > 0 for v in (speed,acceleration)):
            raise ValueError('MINCO limits must be positive and finite')
        if not math.isfinite(position_length_weight) or position_length_weight < 0:
            raise ValueError('MINCO position length weight must be finite and nonnegative')
        if not math.isfinite(position_ftol) or position_ftol<=0:
            raise ValueError('position_ftol must be positive and finite')
        if not math.isfinite(position_speed_weight) or position_speed_weight<0:
            raise ValueError('MINCO speed penalty weight must be finite and nonnegative')
        self.position_speed_weight=float(position_speed_weight)
        self.position_ftol=position_ftol
        self.compact_constraints=bool(compact_constraints)
        self.improve_initial=bool(improve_initial)
        self.position_length_weight = float(position_length_weight)
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[1] != 2 or len(path)<2 or not np.isfinite(path).all():
            raise ValueError('MINCO needs a finite Nx2 path')
        # 第 1 步：整理 RRT 引导点。
        # 跳过重复点，并将每条边至少分成两段，长边按不超过约 0.7m 细分。
        # 新增内部点给优化器提供调整空间；起终点固定，其余点允许在走廊内移动。
        guides = [path[0]]
        for a,b in zip(path,path[1:]):
            distance = float(np.linalg.norm(b-a))
            if distance < 1e-8: continue
            count = max(2, int(math.ceil(distance/.7)))
            guides.extend(a+(b-a)*j/count for j in range(1,count+1))
        guides = np.asarray(guides)
        if len(guides)<2: raise ValueError('MINCO needs distinct endpoints')
        n = len(guides)-1
        if n > 120: raise ValueError('MINCO route exceeds 120-piece planning budget')
        # 第 2 步：初始化时长并建立两个线性映射。
        # 时长按“距离 / 速度上限”估计，至少 0.3s；这还不是最终的限速结果。
        durations = np.maximum(np.linalg.norm(np.diff(guides,axis=0),axis=1)/speed, .3)
        corridors = [corridor(a,b,scene) for a,b in zip(guides,guides[1:])]
        began = time.monotonic()
        self._fit(guides, corridors, durations, scene, speed, acceleration)
        baseline_fit = dict(self.fit_diagnostics)
        before = self._snapshot()
        baseline_time = self.total_time
        baseline_jerk = max(self.jerk_energy, 1e-12)
        # 嵌套联合优化：外层搜索各段时间，内层复用原位置 QP。
        # 每次都会重新求 MINCO 系数，不能直接改时间戳，否则连接处导数会断开。
        # 目标归一化为 总时间/原时间 + 0.02*jerk/原jerk；只接受时间不变长的候选。
        score = 1.02
        evaluations = accepted = rejected = 0
        history = []
        budget = max(0, int(time_evaluations)) if time_optimization else 0
        # time_evaluations 是整条轨迹的单段候选评估上限，默认 8 次，失败也计数。
        # 配对搜索默认关闭；显式开启时使用独立的追加预算，不占单段预算。
        coordinate_budget = budget
        pair_budget = max(0, int(pair_evaluations)) if pair_search and budget else 0
        budget += pair_budget
        raw = durations.copy()
        # 无导数坐标搜索，优先调整速度/加速度利用率较低的段。
        # 统一缩放会被最后的限速步骤抵消，所以这里重点改变各段的时间比例。
        for step in (.25, .12, .06):
            if evaluations >= coordinate_budget: break
            order = np.argsort(self._utilization(speed, acceleration))
            for index in order:
                for direction in (-1., 1.):
                    if evaluations >= coordinate_budget: break
                    proposed = raw.copy()
                    proposed[index] = max(.3, proposed[index]*math.exp(direction*step))
                    if np.array_equal(proposed, raw): continue
                    evaluations += 1
                    candidate = object.__new__(MincoTrajectory)
                    candidate.position_length_weight = self.position_length_weight
                    candidate.position_ftol = self.position_ftol
                    candidate.position_speed_weight = self.position_speed_weight
                    candidate.compact_constraints = self.compact_constraints
                    candidate.improve_initial = self.improve_initial
                    try:
                        candidate._fit(guides, corridors, proposed, scene, speed, acceleration,
                                       self.optimized_knots)
                    except (ValueError, np.linalg.LinAlgError) as error:
                        rejected += 1
                        history.append(dict(evaluation=evaluations, search='coordinate', accepted=False,
                                            reason=str(error),
                                            fit=getattr(candidate, 'fit_diagnostics', {})))
                        continue
                    objective = (candidate.total_time/baseline_time +
                                 .02*candidate.jerk_energy/baseline_jerk)
                    improved = (objective < score-1e-7 and
                                candidate.total_time <= self.total_time+1e-9)
                    history.append(dict(evaluation=evaluations, search='coordinate', accepted=bool(improved),
                                        duration_s=candidate.total_time, objective=objective,
                                        fit=candidate.fit_diagnostics))
                    if improved:
                        self.__dict__.update(candidate.__dict__)
                        raw, score = proposed, objective
                        accepted += 1
                if evaluations >= coordinate_budget: break
        pair_before = self._snapshot()
        pair_attempts = pair_accepted = 0
        # 每次基于最新最佳轨迹重算瓶颈。低利用率段缩短 delta，瓶颈段
        # 增加同样的 delta：原始总时间保持不变，争取降低最终统一缩放。
        for attempt in range(pair_budget):
            utilization = self._utilization(speed, acceleration)
            bottleneck = int(np.argmax(utilization))
            donors = [int(i) for i in np.argsort(utilization) if i != bottleneck and raw[i] > .3+1e-10]
            if not donors: break
            donor = donors[(attempt//2) % len(donors)]
            step = (.12, .06)[attempt % 2]
            proposed = self._paired_durations(raw, bottleneck, donor, step)
            evaluations += 1
            pair_attempts += 1
            entry = dict(evaluation=evaluations, search='pair', bottleneck_segment=bottleneck,
                         donor_segment=donor, step=step, transferred_s=float(proposed[bottleneck]-raw[bottleneck]))
            candidate = object.__new__(MincoTrajectory)
            candidate.position_length_weight = self.position_length_weight
            candidate.position_ftol = self.position_ftol
            candidate.position_speed_weight = self.position_speed_weight
            candidate.compact_constraints = self.compact_constraints
            candidate.improve_initial = self.improve_initial
            try:
                candidate._fit(guides, corridors, proposed, scene, speed, acceleration,
                               self.optimized_knots)
            except (ValueError, np.linalg.LinAlgError) as error:
                rejected += 1
                history.append(dict(entry, accepted=False, reason=str(error),
                                    fit=getattr(candidate, 'fit_diagnostics', {})))
                continue
            objective = candidate.total_time/baseline_time + .02*candidate.jerk_energy/baseline_jerk
            improved = objective < score-1e-7 and candidate.total_time <= self.total_time+1e-9
            history.append(dict(entry, accepted=bool(improved), duration_s=candidate.total_time,
                                length_m=float(candidate.s[-1]), objective=objective, fit=candidate.fit_diagnostics))
            if improved:
                self.__dict__.update(candidate.__dict__)
                raw, score = proposed, objective
                accepted += 1
                pair_accepted += 1
        pair_after = self._snapshot()
        # 末段减速是独立的跟踪改善步骤，允许有限参考时间增长，不能伪装为时间搜索收益。
        terminal_report = self._refine_terminal(guides, corridors, raw, scene, speed, acceleration) if terminal_braking else dict(enabled=False)
        # 末段调整后重建当前原始时间，再处理起步转弯；不重复使用旧时间分配。
        turn_report = self._refine_start_turn(guides,corridors,np.asarray(self.durations)/self.time_stretch,
                                              scene,speed,acceleration) if start_turn_braking else dict(enabled=False)
        self.backend = ('MINCO-S3 corridor waypoint and segment-time optimization' if budget
                        else 'MINCO-S3 corridor waypoint optimization')
        stage_totals = {}
        for fit in [baseline_fit] + [entry['fit'] for entry in history] + [entry.get('fit',{}) for entry in terminal_report.get('history',[])+turn_report.get('history',[])]:
            for name, seconds in fit.get('stage_wall_s', {}).items():
                stage_totals[name] = stage_totals.get(name, 0.) + seconds
        self.time_optimization = dict(
            enabled=bool(time_optimization), method=('coordinate and bottleneck pair search with waypoint QP' if pair_budget else
                                                   'nested coordinate search with waypoint QP'),
            evaluation_budget=budget, coordinate_evaluation_budget=coordinate_budget,
            evaluations=evaluations, accepted=accepted,
            infeasible_candidates=rejected, jerk_weight=.02,
            position_length_weight=self.position_length_weight,
            position_speed_weight=self.position_speed_weight,
            status=('improved' if accepted else 'baseline_retained'),
            wall_time_s=time.monotonic()-began, before=before, after=self._snapshot(),
            baseline_fit=baseline_fit, fit_stage_totals_s=stage_totals,
            pair_search=dict(enabled=bool(pair_budget), evaluation_budget=pair_budget,
                             evaluations=pair_attempts, accepted=pair_accepted,
                             before=pair_before, after=pair_after),
            terminal_braking=terminal_report, start_turn_braking=turn_report,
            history=history)

    def _piece_bounds(self, indices):
        v=a=0.
        for i in indices:
            c=self.control_points[i];T=self.durations[i]
            v=max(v,bernstein_norm_bound(5*np.diff(c,axis=0)/T))
            a=max(a,bernstein_norm_bound(20*np.diff(c,n=2,axis=0)/T**2))
        return v,a

    def _refine_start_turn(self,guides,corridors,raw,scene,speed,acceleration):
        before=self._snapshot()
        report=dict(enabled=True,accepted=False,before=before,history=[],max_extra_time_s=1.5,max_length_ratio=1.01)
        edges=np.diff(guides,axis=0);dist=np.linalg.norm(edges,axis=1);arc=np.cumsum(dist)
        corner=None
        accumulated_turn=0.
        for j in range(1,len(edges)):
            angle=math.acos(float(np.clip(np.dot(edges[j-1],edges[j])/(dist[j-1]*dist[j]),-1,1)))
            accumulated_turn+=angle
            # 连续小弯累积也会造成滞后：单点 >=25°，或累计 >=35°且当前有明显转向。
            if arc[j-1]<=4. and (angle>=math.radians(25) or
                                  (accumulated_turn>=math.radians(35) and angle>=math.radians(10))):
                corner=j;break
        if corner is None:
            report.update(reason='no_early_sharp_turn',after=before)
            return report
        # 覆盖转弯前两段和后两段，提前减速，不在弯心突然修改速度指令。
        indices=list(range(max(0,corner-2),min(len(raw),corner+2)))
        v0,a0=self._piece_bounds(indices);best=a0;tail0=self._terminal_metric()
        report.update(segments=indices,corner_xy=guides[corner].tolist(),speed_bound_before=v0,acceleration_bound_before=a0)
        for factor in (1.2,1.4,1.6):
            proposed=raw.copy();proposed[indices]*=factor
            candidate=object.__new__(MincoTrajectory);candidate.position_length_weight=self.position_length_weight
            candidate.position_ftol = self.position_ftol
            candidate.position_speed_weight = self.position_speed_weight
            candidate.compact_constraints = self.compact_constraints
            candidate.improve_initial = self.improve_initial
            try:
                candidate._fit(guides,corridors,proposed,scene,speed,acceleration,self.optimized_knots)
            except (ValueError,np.linalg.LinAlgError) as error:
                report['history'].append(dict(factor=factor,accepted=False,reason=str(error),fit=getattr(candidate,'fit_diagnostics',{})))
                continue
            v,a=candidate._piece_bounds(indices)
            feasible=(candidate.total_time<=before['duration_s']+1.5 and candidate.s[-1]<=before['length_m']*1.01
                      and candidate._terminal_metric()<=tail0*1.05)
            improved=feasible and a<best and a<=a0*.85 and v<=v0*.95
            report['history'].append(dict(factor=factor,accepted=bool(improved),duration_s=candidate.total_time,
                length_m=float(candidate.s[-1]),speed_bound=v,acceleration_bound=a,within_growth_limits=bool(feasible),fit=candidate.fit_diagnostics))
            if improved:
                self.__dict__.update(candidate.__dict__);best=a
                report.update(accepted=True,selected_factor=factor)
        v,a=self._piece_bounds(indices)
        report.update(after=self._snapshot(),speed_bound_after=v,acceleration_bound_after=a)
        return report

    def _terminal_metric(self, pieces=3):
        # 加速度范数的 Bernstein 连续上界，用来限制末段减速强度。
        # 这是参考曲线指标，不代表已证明实机不会超调。
        bounds=[]
        for c,T in zip(self.control_points[-pieces:], self.durations[-pieces:]):
            bounds.append(bernstein_norm_bound(20*np.diff(c,n=2,axis=0)/T**2))
        return max(bounds)

    def _refine_terminal(self, guides, corridors, raw, scene, speed, acceleration):
        before=self._snapshot(); initial_metric=self._terminal_metric()
        report=dict(enabled=True, accepted=False, before=before, history=[],
                    acceleration_bound_before=initial_metric, max_extra_time_s=.8,
                    max_length_ratio=1.005, pieces=min(3,len(raw)))
        if len(raw)<6:
            report.update(reason='route_too_short',after=before,acceleration_bound_after=initial_metric)
            return report
        best_metric=initial_metric
        # 原始时间最后三段增加 15%、30%、50%，每次重新解整条曲线，保持连接连续性。
        for factor in (1.15,1.30,1.50):
            proposed=raw.copy();proposed[-3:]*=factor
            candidate=object.__new__(MincoTrajectory)
            candidate.position_length_weight=self.position_length_weight
            candidate.position_ftol = self.position_ftol
            candidate.position_speed_weight = self.position_speed_weight
            candidate.compact_constraints = self.compact_constraints
            candidate.improve_initial = self.improve_initial
            try:
                candidate._fit(guides,corridors,proposed,scene,speed,acceleration,self.optimized_knots)
            except (ValueError,np.linalg.LinAlgError) as error:
                report['history'].append(dict(factor=factor,accepted=False,reason=str(error),fit=getattr(candidate,'fit_diagnostics',{})))
                continue
            metric=candidate._terminal_metric()
            feasible=(candidate.total_time<=before['duration_s']+.8 and
                      candidate.s[-1]<=before['length_m']*1.005)
            improved=feasible and metric<best_metric-1e-9 and metric<=initial_metric*.9
            report['history'].append(dict(factor=factor,accepted=bool(improved),
                duration_s=candidate.total_time,length_m=float(candidate.s[-1]),
                acceleration_bound=metric,within_growth_limits=bool(feasible),fit=candidate.fit_diagnostics))
            if improved:
                self.__dict__.update(candidate.__dict__)
                best_metric=metric
                report.update(accepted=True,selected_factor=factor)
        report.update(after=self._snapshot(),acceleration_bound_after=self._terminal_metric())
        return report

    @staticmethod
    def _paired_durations(raw, bottleneck, donor, step):
        """将 donor 的一部分原始时间移给瓶颈段；不改变时间和，不突破下限。"""
        proposed = raw.copy()
        delta = min(raw[donor]-.3, raw[donor]*(1.-math.exp(-step)))
        proposed[donor] -= delta
        proposed[bottleneck] += delta
        return proposed

    def _utilization(self, speed, acceleration):
        values = []
        for c, T in zip(self.control_points, self.durations):
            v = 5*np.diff(c, axis=0)/T
            a = 4*np.diff(v, axis=0)/T
            values.append(max(bernstein_norm_bound(v)/speed,
                              math.sqrt(bernstein_norm_bound(a)/acceleration)))
        return np.asarray(values)

    def _snapshot(self):
        """JSON 中记录的是参考轨迹认证，不是实际飞行余量认证。"""
        return dict(duration_s=float(self.total_time), length_m=float(self.s[-1]),
                    segment_durations_s=self.durations.tolist(),
                    jerk_energy=float(self.jerk_energy),
                    velocity_bound=float(self.velocity_bound),
                    acceleration_bound=float(self.acceleration_bound),
                    constraints=dict(self.constraint_checks),
                    dynamics_certificate=self.dynamics_certificate)

    def _mark_fit_stage(self, name):
        now = time.monotonic()
        if self._fit_stage is not None:
            self.fit_diagnostics['stage_wall_s'][self._fit_stage] = now-self._fit_stage_started
        self._fit_stage, self._fit_stage_started = name, now

    def _fit(self, guides, corridors, durations, scene, speed, acceleration, warm_knots=None):
        # 每个候选单独计时；失败候选也保留已执行步骤及失败步骤耗时。
        began = time.monotonic()
        self.fit_diagnostics = dict(stage_wall_s={}, initial_source=None,
                                    warm_start_used=False, lp_called=False, success=False)
        self._fit_stage = None
        self._mark_fit_stage('coefficient_map')
        try:
            self._fit_impl(guides, corridors, durations, scene, speed, acceleration, warm_knots)
            self.fit_diagnostics['success'] = True
        finally:
            self._mark_fit_stage(None)
            self.fit_diagnostics['wall_time_s'] = time.monotonic()-began

    def _fit_impl(self, guides, corridors, durations, scene, speed, acceleration, warm_knots):
        """给定时间分配，复用原来的位置优化、连续避障认证和动力学缩放。"""
        n = len(durations)
        # mapping[i]：全部途经点 -> 第 i 段 6 个幂基系数，形状 (6,N+1)。
        mapping = coefficient_map(durations).reshape(n,6,n+1)
        # controls[i]：全部途经点 -> 第 i 段 6 个 Bernstein 控制点。
        # einsum 相当于对每一段做“转换矩阵 @ mapping[i]”。
        self._mark_fit_stage('corridor_constraints')
        controls = np.einsum('ij,pjk->pik',bernstein_map(),mapping)
        # 起终点不参与优化：fixed 是它们对控制点位置的固定贡献，形状 (N,6,2)。
        # free_map 是内部 N-1 个可移动途经点对控制点的映射。
        fixed = controls[:,:,[0,-1]]@guides[[0,-1]]
        free_map = controls[:,:,1:-1]
        # 第 3 步：把“每段 6 个控制点都在该段走廊里”展开成线性不等式。
        # 未知数按 [x1,y1,x2,y2,...] 排列；起终点的贡献移到不等式右侧。
        rows=[]; upper=[]
        original_rows=sum(6*len(limits) for normals,limits in corridors)
        for i,(normals,limits) in enumerate(corridors):
            if getattr(self,'compact_constraints',True):
                normals,limits=compact_halfspaces(normals,limits)
            rows.append(np.einsum('kj,li->klji',free_map[i],normals).reshape(-1,2*(n-1)))
            upper.append((limits[None,:]-fixed[i]@normals.T).ravel())
        constraint = np.concatenate(rows)
        upper = np.concatenate(upper)
        self.fit_diagnostics.update(constraints_before=original_rows,constraints_after=len(upper),
            position_variables=2*(n-1),position_ftol=getattr(self,'position_ftol',1e-8))
        # 汇总后的避障条件：constraint @ 内部点坐标 <= upper。

        # 第 4 步：构造最小 jerk 二次目标。
        # gram 精确计算单段 integral_0^1 ||d³p/du³||² du。
        # 换回真实时间时，三阶导数平方带来 T^-6，dt 又带来 T，因此乘 T^-5。
        self._mark_fit_stage('jerk_objective')
        gram = np.zeros((6,6))
        for j in range(3,6):
            for k in range(3,6):
                gram[j,k] = (math.factorial(j)/math.factorial(j-3)*
                             math.factorial(k)/math.factorial(k-3)/(j+k-5))
        energy = np.einsum('pji,jk,pkl,p->il',mapping,gram,mapping,durations**-5)
        energy = .5*(energy+energy.T)
        # energy 是“所有途经点坐标 -> jerk 能量”的二次型矩阵。
        # 固定起终点后，剩余目标为 sum(q*(H@q)) + 2*sum(q*linear) + 常数。
        # q 是内部途经点的二维坐标，H 为下面的 hessian。
        hessian = energy[1:-1,1:-1]
        linear = energy[1:-1][:,[0,-1]]@guides[[0,-1]]
        # 第 5 步：改善数值条件，再求解二次规划（QP）。
        # 不同途经点的变化对 jerk 影响差别很大，直接求解容易迭代很慢。
        # 用 Cholesky 分解作坐标变换 q = center + transform @ z，
        # 将等价目标变成 0.5*||z||²；center 是不考虑障碍时的最优内部点。
        # 这只改变优化变量的表示方式，不改变原来的目标最优解和可行范围。
        self._mark_fit_stage('qp_conditioning')
        factor=cholesky(hessian,lower=True)
        transform=np.kron(solve_triangular(factor.T,np.eye(n-1)),np.eye(2))
        center=solve(hessian,-linear,assume_a='pos').ravel()
        transformed=constraint@transform
        shifted=upper-constraint@center
        row_scale=np.maximum(np.linalg.norm(transformed,axis=1),1e-12)
        # 各条约束除以自己的行范数，避免数值量级差异影响求解。
        transformed=transformed/row_scale[:,None]
        shifted=shifted/row_scale
        self._mark_fit_stage('initial_feasibility')
        # 优先复用当前最佳候选的位置解。时间变化后控制点约束也会变化，
        # 所以必须在新约束中重新验算，不能直接假定旧解仍然安全。
        initial = None
        if warm_knots is not None:
            warm = np.asarray(warm_knots, dtype=float)
            if warm.shape == guides.shape and np.isfinite(warm).all():
                proposed = warm[1:-1].ravel()
                if np.max(constraint@proposed-upper) <= 1e-9:
                    initial = proposed.copy()
                    self.fit_diagnostics.update(initial_source='previous_best', warm_start_used=True)
        if initial is None:
            proposed = guides[1:-1].ravel()
            if np.max(constraint@proposed-upper) <= 1e-9:
                initial = proposed.copy()
                self.fit_diagnostics['initial_source'] = 'guides'
        # 已有可行初值时跳过重复 LP；旧位置解和引导点均不可行时才求 LP。
        self._mark_fit_stage('feasibility_lp')
        if initial is None:
            self.fit_diagnostics['lp_called'] = True
            feasible = linprog(np.zeros(2*(n-1)), A_ub=constraint, b_ub=upper,
                               bounds=[(None,None)]*(2*(n-1)), method='highs')
            if not feasible.success:
                raise ValueError('MINCO safe-corridor problem infeasible; replan required')
            initial = feasible.x
            self.fit_diagnostics['initial_source'] = 'linear_program'
        self._mark_fit_stage('position_qp')
        initial=solve(transform,initial-center)
        # SLSQP 的不等式写成 g(z)>=0，所以这里用 shifted-transformed@z。
        length_weight = getattr(self, 'position_length_weight', 1.)
        speed_weight = getattr(self, 'position_speed_weight', 0.)
        objective_calls=0;objective_wall=0.
        def counted(fun):
            def wrapped(z):
                nonlocal objective_calls,objective_wall
                tick=time.monotonic();objective_calls+=1
                try:return fun(z)
                finally:objective_wall+=time.monotonic()-tick
            return wrapped
        if length_weight or speed_weight:
            # 固定本次位置求解的基准，不随迭代点变化；与坐标白化一致：
            # 实际 jerk = z.T@z + 常数，所以梯度为 2z/E0。
            nodes, quad_weights = np.polynomial.legendre.leggauss(24)
            derivative_map = np.einsum('qk,pkn->pqn',
                np.array([basis((u+1)/2,1) for u in nodes]), mapping).reshape(-1,n+1)
            weights = np.tile(quad_weights/2,n)
            velocity_map=derivative_map/np.repeat(durations,24)[:,None]
            speed_weights=np.repeat(durations,24)*weights/float(np.sum(durations))
            energy_reference = max(float(np.sum(guides*(energy@guides))),1e-12)
            length_reference = max(float(np.linalg.norm(np.diff(guides,axis=0),axis=1).sum()),1e-12)
            @counted
            def objective(z):
                positions = np.vstack((guides[0],(center+transform@z).reshape(-1,2),guides[-1]))
                length, gradient = curve_length_value_gradient(positions,derivative_map,weights)
                value = float(z@z)/energy_reference + length_weight*length/length_reference
                jac = 2*z/energy_reference + length_weight/length_reference*(transform.T@gradient[1:-1].ravel())
                if speed_weight:
                    penalty,penalty_grad=speed_penalty_value_gradient(positions,velocity_map,speed_weights,speed)
                    value+=speed_weight*penalty
                    jac+=speed_weight*(transform.T@penalty_grad[1:-1].ravel())
                return value,jac
            initial_positions=np.vstack((guides[0],(center+transform@initial).reshape(-1,2),guides[-1]))
            self.fit_diagnostics['speed_penalty_before']=speed_penalty_value_gradient(
                initial_positions,velocity_map,speed_weights,speed)[0]
            # 从可行 LP 解沿直线朝引导点/无约束最小 jerk 解移动。
            # 线性约束直接给出最大可行步长，且只采用目标更低的初值。
            initial_value=objective(initial)[0]
            self.fit_diagnostics['initial_objective_before']=initial_value
            if getattr(self,'improve_initial',True) and not self.fit_diagnostics['warm_start_used']:
                targets=[solve(transform,guides[1:-1].ravel()-center),np.zeros_like(initial)]
                for target in targets:
                    direction=target-initial;change=transformed@direction
                    mask=change>0
                    alpha=min(1.,float(np.min(np.maximum(shifted-transformed@initial,0.)[mask]/change[mask]))) if mask.any() else 1.
                    for fraction in (.5,1.):
                        trial=initial+direction*(alpha*.999*fraction)
                        value=objective(trial)[0]
                        if value<initial_value and np.max(transformed@trial-shifted)<=1e-9:
                            initial=trial;initial_value=value
                            self.fit_diagnostics['initial_improved']=True
                            break
            self.fit_diagnostics['initial_objective_after']=initial_value
            result = minimize(objective, initial, jac=True, method='SLSQP',
                              constraints={'type':'ineq','fun':lambda x:shifted-transformed@x,
                                           'jac':lambda x:-transformed},
                              options={'maxiter':200,'ftol':getattr(self,'position_ftol',1e-8)})
            self.fit_diagnostics.update(position_length_weight=length_weight,
                objective=('normalized jerk + curve length + speed excess' if speed_weight else 'normalized jerk + curve length'),
                speed_penalty_weight=speed_weight,speed_penalty_limit_mps=speed,
                speed_penalty_after=speed_penalty_value_gradient(
                    np.vstack((guides[0],(center+transform@result.x).reshape(-1,2),guides[-1])),
                    velocity_map,speed_weights,speed)[0],
                jerk_reference=energy_reference,
                length_reference_m=length_reference, length_quadrature_nodes_per_piece=24)
        else:
            # weight=0 完整保留旧版目标和数值缩放，可用于同路径消融对照。
            result = minimize(lambda x:.5*float(x@x), initial, jac=lambda x:x, method='SLSQP',
                              constraints={'type':'ineq','fun':lambda x:shifted-transformed@x,
                                           'jac':lambda x:-transformed},
                              options={'maxiter':200,'ftol':getattr(self,'position_ftol',1e-8)})
            self.fit_diagnostics.update(position_length_weight=0., objective='jerk only')
        self.fit_diagnostics.update(qp_iterations=int(result.nit),
            solver_objective_calls=int(result.nfev),solver_gradient_calls=int(result.njev),
            objective_gradient_evaluations=objective_calls,objective_gradient_wall_s=objective_wall,
            solver_message=str(result.message))
        if not result.success or not np.isfinite(result.x).all():
            raise ValueError('MINCO waypoint optimization failed: '+result.message)
        # 第 6 步：变回真实途经点坐标，得到最终的五次多项式和控制点。
        self._mark_fit_stage('corridor_verification')
        optimized=center+transform@result.x
        knots = np.vstack((guides[0],optimized.reshape(-1,2),guides[-1]))
        self.optimized_knots = knots.copy()
        self.coefficients = mapping@knots
        self.control_points = controls@knots
        residual = max(float(np.max(c@normals.T-limits))
                       for c,(normals,limits) in zip(self.control_points,corridors))
        if not np.isfinite(self.coefficients).all() or residual > 1e-7:
            raise ValueError('MINCO corridor constraint verification failed')
        # 第 7 步：独立复验整条曲线，而不是仅相信优化器的成功标志。
        # 递归凸包检查覆盖连续曲线；失败就拒绝执行，不自动改用旧后端。
        self._mark_fit_stage('collision_certificate')
        if not all(certify_curve(c,scene) for c in self.control_points):
            raise ValueError('MINCO continuous collision certificate failed')
        self._mark_fit_stage('dynamics_and_scaling')
        vmax=amax=0.
        raw_vmax=raw_amax=0.
        piece_bounds=[]
        # 第 8 步：计算真实时间下速度、加速度的保守上界。
        # 五次 Bernstein 曲线的一阶导数控制点为 5*(相邻控制点差)/T；
        # 再求导得到加速度控制点。导数曲线也在其控制点的凸包内，
        # 原控制点范数给出粗上界；细分 4 层后按各子区间凸包取得更紧上界。
        # 两者均覆盖连续曲线，不以离散采样最大值代替认证。
        for c,T in zip(self.control_points,durations):
            velocity=5*np.diff(c,axis=0)/T
            accel=4*np.diff(velocity,axis=0)/T
            old_v=float(np.max(np.linalg.norm(velocity,axis=1)))
            old_a=float(np.max(np.linalg.norm(accel,axis=1)))
            v=bernstein_norm_bound(velocity); a=bernstein_norm_bound(accel)
            raw_vmax=max(raw_vmax,old_v); raw_amax=max(raw_amax,old_a)
            vmax=max(vmax,v); amax=max(amax,a)
            piece_bounds.append(dict(segment=len(piece_bounds),
                raw_velocity_bound=old_v, velocity_bound=v,
                raw_acceleration_bound=old_a, acceleration_bound=a))
        # 第 9 步：统一拉长时间，不改变已经认证过的几何形状。
        # 时长乘 k 后，速度除以 k，加速度除以 k²，jerk 能量除以 k⁵。
        # k 至少为 1，并额外保留 0.1% 数值余量；这是限速缩放，不是最短时间优化。
        self.time_stretch=max(1.,vmax/speed,math.sqrt(amax/acceleration))*1.001
        self.dynamics_certificate=dict(method='Bernstein subdivision', depth=4,
            unscaled_segment_bounds=piece_bounds,
            coarse_time_stretch=max(1.,raw_vmax/speed,math.sqrt(raw_amax/acceleration))*1.001,
            tight_time_stretch=self.time_stretch)
        self.fit_diagnostics['dynamics_certificate']=self.dynamics_certificate
        self.durations=durations*self.time_stretch
        self.times=np.r_[0.,np.cumsum(self.durations)].tolist()
        self.total_time=self.times[-1]
        self.backend='MINCO-S3 corridor waypoint optimization'
        self.optimization_iterations=int(result.nit)
        self.velocity_bound=vmax/self.time_stretch
        self.acceleration_bound=amax/self.time_stretch**2
        self.jerk_energy=float(np.sum(knots*(energy@knots)))/self.time_stretch**5
        if (not math.isfinite(self.jerk_energy) or
                self.velocity_bound > speed+1e-9 or self.acceleration_bound > acceleration+1e-9):
            raise ValueError('MINCO dynamics verification failed')
        self.constraint_checks = dict(continuous_collision_free=True, corridor_valid=True,
                                      corridor_max_residual_m=residual,
                                      velocity_valid=True, acceleration_valid=True,
                                      speed_limit_mps=float(speed),
                                      acceleration_limit_mps2=float(acceleration))
        # 第 10 步：生成预览和长度统计用的离散点。
        # 采样间距由时长和速度上限估算，约不大于 2cm；不是沿弧长严格等距采样。
        # s[-1] 是采样折线长度近似值；真实飞行参考仍通过 sample(t) 求多项式得到。
        self._mark_fit_stage('preview_and_length')
        self.points=[]
        for i,T in enumerate(self.durations):
            count=max(2,int(math.ceil(T*speed/.02)))
            self.points.extend(tuple(basis(j/count)@self.coefficients[i]) for j in range(count))
        self.points.append(tuple(knots[-1]))
        self.s=[0.]
        for a,b in zip(self.points,self.points[1:]): self.s.append(self.s[-1]+math.dist(a,b))

    def derivative(self, elapsed, order=0):
        """查询整条轨迹在 elapsed 秒时的真实时间导数。

        order=0/1/2/3 分别对应位置/速度/加速度/jerk。
        先定位属于哪一段，再将段内时间换成 u，最后除以该段时长的 order 次方。
        elapsed 是轨迹时钟；外部跟踪程序可以因误差增大而放慢它。
        """
        if order not in range(6): raise ValueError('Derivative order must be 0..5')
        elapsed=max(0.,min(float(elapsed),self.total_time))
        i=min(len(self.durations)-1,bisect.bisect_right(self.times,elapsed)-1)
        u=(elapsed-self.times[i])/self.durations[i]
        return basis(u,order)@self.coefficients[i]/self.durations[i]**order

    def sample(self, elapsed):
        """飞行跟踪器的统一接口：返回 ((x,y), [vx,vy])。

        起点之前、终点之后直接返回准确的端点及零速度，方便等待和终点悬停。
        外部若放慢轨迹时钟，还需要同步缩放这里返回的速度前馈。
        """
        if elapsed <= 0: return self.points[0], [0.,0.]
        if elapsed >= self.total_time: return self.points[-1], [0.,0.]
        return tuple(self.derivative(elapsed)), self.derivative(elapsed,1).tolist()


def certify_curve(control,scene,depth=0):
    """用控制点包围盒递归检查整条曲线，返回是否能确认连续避障。

    包围盒与所有膨胀障碍都分离，则整条曲线肯定安全。
    包围盒与障碍重叠时，曲线本身未必碰撞，所以将曲线一分为二继续检查。
    达到递归上限仍无法确认时返回 False，表示拒绝这条轨迹。
    """
    lower=np.min(control,axis=0);upper=np.max(control,axis=0)
    xmin,xmax,ymin,ymax=scene.bounds;m=scene.margin
    if not (xmin+m<lower[0] and upper[0]<xmax-m and
            ymin+m<lower[1] and upper[1]<ymax-m): return False
    if not any(upper[0]>=l and lower[0]<=r and upper[1]>=b and lower[1]<=t
               for l,r,b,t in scene.boxes): return True
    if depth>=18: return False
    # de Casteljau 二分：反复取相邻控制点的中点，得到左右半条曲线的控制点。
    # 二分前后几何形状完全相同；right 倒序后才是从中点走向原终点的顺序。
    left=[control[0]];right=[control[-1]];level=np.asarray(control)
    while len(level)>1:
        level=(level[:-1]+level[1:])*.5
        left.append(level[0]);right.append(level[-1])
    return (certify_curve(np.asarray(left),scene,depth+1) and
            certify_curve(np.asarray(right[::-1]),scene,depth+1))
