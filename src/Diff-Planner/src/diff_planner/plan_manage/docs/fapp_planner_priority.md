# FAPP 规划优先开关

默认关闭，保持原有动态绕行候选和动态碰撞硬检查。

单机七窗口启动时追加 `--planner-priority` 开启，追加 `--strict-dynamic` 关闭；也可设置环境变量 `UAV1_SINGLE_PLANNER_PRIORITY=true`。
直接启动 `run_swarm.launch` 时使用 `fapp_planner_priority:=true`。对应节点参数为 `optimization/fapp_planner_priority`。

开启后，Diff 使用正常轨迹优化，仍计算 FAPP 动态障碍代价，但不再强制要求生成与 FAPP 预测完全分离的下穿/侧绕候选；发布前和执行中的 FAPP 检查仅保留静态占据检查。原有动力学、机间碰撞、周期重规划质量检查继续生效。独立的 LDOP 摆动守卫不由此开关控制。

此模式允许轨迹与 FAPP 预测重叠，真实运动障碍也可能被放行；它不保证每次规划都能成功。`--detection-only` 仍会关闭整个 FAPP 动态避障，包括其优化代价。

验证命令（使用独立 localhost ROS master，不启动控制器）：

```bash
source devel/setup.bash
python3 src/Diff-Planner/src/diff_planner/plan_manage/test/fapp_dynamic_avoidance_smoke.py --scenario all
```

`planner_priority` 验证动态预测重叠时能发布并执行轨迹，`priority_static` 验证开启后实体墙仍阻止通行；其余用例验证默认行为。
