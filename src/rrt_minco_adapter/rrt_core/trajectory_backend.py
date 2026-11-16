"""两个 Gazebo RRT 入口共用的后端选择接口。

阅读调用关系：local_rrt_sim.py / flight_compare.py
    -> build_trajectory -> MincoTrajectory -> sample(t) -> MAVROS/PX4。
前端只需提供 RRT 折线路径；后端统一提供位置、速度查询和预览数据。
"""
import time
from threading import RLock
from threadpoolctl import threadpool_limits, threadpool_info

from smooth_trajectory import SmoothTrajectory
from minco_trajectory import MincoTrajectory


# BLAS 线程数是进程级设置；串行进入该作用域，避免并发规划相互恢复错误。
_MINCO_BLAS_LOCK = RLock()

def _blas_pools():
    return [dict(library=p['internal_api'],prefix=p['prefix'],threads=p['num_threads'])
            for p in threadpool_info() if p['user_api']=='blas']

def build_trajectory(path, scene, speed=.6, acceleration=.8, backend='minco', **options):
    """默认选择 MINCO；传 backend='bezier' 可显式运行旧后端作对照。

    speed 单位 m/s，acceleration 单位 m/s²；path 使用米制二维坐标。
    options 中的转弯响应等参数只传给旧后端，MINCO 不使用这些参数。
    MINCO 求解失败时异常向上传递，让 /plan 返回失败，不自动换另一种后端。
    """
    if backend == 'minco':
        # 仅在 MINCO 构造/优化期间限制 BLAS，不修改全局环境变量。
        # with 在成功和异常两种情况下都会恢复进入前的线程配置。
        with _MINCO_BLAS_LOCK:
            before = _blas_pools()
            began = time.monotonic()
            with threadpool_limits(limits=1, user_api='blas'):
                active = _blas_pools()
                trajectory = MincoTrajectory(path, scene, speed, acceleration,
                    **{k:options[k] for k in ('time_optimization','time_evaluations','pair_search','pair_evaluations','position_length_weight','terminal_braking','start_turn_braking','position_ftol','compact_constraints','improve_initial','position_speed_weight')
                       if k in options})
            report = dict(requested_threads=1, before=before, during=active,
                          after=_blas_pools(), backend_wall_s=time.monotonic()-began)
            trajectory.blas_execution = report
            trajectory.time_optimization['blas_execution'] = report
            return trajectory
    if backend == 'bezier':
        return SmoothTrajectory(path, scene, speed, acceleration, **options)
    raise ValueError('trajectory_backend must be minco or bezier')


def backend_stats(trajectory):
    """把轨迹对象的统计量转成可写入 JSON 的普通 Python 数值。

    time_stretch 是时间放大倍数；velocity_bound/acceleration_bound 是参考上界；
    jerk_energy 是平方 jerk 的积分。它们不是 Gazebo 实际飞行测量值。
    """
    return dict(**({'time_optimization':trajectory.time_optimization}
                   if hasattr(trajectory,'time_optimization') else {}),
                backend=getattr(trajectory,'backend','quadratic Bezier'),
                reference_duration_s=trajectory.total_time,
                reference_length_m=trajectory.s[-1],
                **{key:float(getattr(trajectory,key)) for key in
                   ('time_stretch','velocity_bound','acceleration_bound','jerk_energy')
                   if hasattr(trajectory,key)})
