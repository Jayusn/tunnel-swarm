# 动态对象跟踪设计收敛计划

## 目标

完成 `docs/superpowers/specs/2026-04-24-dynamic-object-tracking-design.md` 的设计收口，并输出配套实施计划 `docs/superpowers/plans/2026-04-25-dynamic-object-tracking-implementation-plan.md`，为下一步实现提供可执行顺序、文件范围和验证闭环。全过程继续使用 `planning-with-files-zh` 记录。

## 当前状态

- 当前阶段：阶段 10 已完成，实施计划已在独立 worktree 中落地并完成临时构建/测试验证。
- 设计文档：`docs/superpowers/specs/2026-04-24-dynamic-object-tracking-design.md` 已完成第一版收口。
- 实施计划：`docs/superpowers/plans/2026-04-25-dynamic-object-tracking-implementation-plan.md` 已生成。
- 执行方式：用户已选择 `Subagent-Driven (recommended)`。
- 工作区：实现在 `/home/st/ldop_ws/src/LDOP/.worktrees/feature-dynamic-object-tracking` 上进行，主工作区保留为设计/计划记录。

## 阶段

| 阶段 | 状态 | 内容 |
|------|------|------|
| 1 | complete | 恢复上下文，建立 `task_plan.md`、`findings.md`、`progress.md` |
| 2 | complete | 阅读现有设计文档、相关代码和测试，整理事实 |
| 3 | complete | 按 brainstorming 流程澄清关键设计取舍 |
| 4 | complete | 汇总设计取舍并形成默认建议 |
| 5 | complete | 在用户确认和默认收口后更新设计文档 |
| 6 | complete | 对设计文档做自检：占位、矛盾、范围、歧义 |
| 7 | complete | 用户审阅设计文档并确认转入实施计划 |
| 8 | complete | 生成实施计划文档并同步过程记录 |
| 9 | complete | 按 `Superpowers` 的 writing-plans handoff，由用户选择执行方式 |
| 10 | complete | 按 `Superpowers:subagent-driven-development` 执行实施计划与阶段评审；受子代理额度限制，后半段由主代理完成 review、修复和验证 |

## 决策记录

| 时间 | 决策 | 原因 |
|------|------|------|
| 2026-04-24 | 本轮不使用视觉伴随工具 | 当前问题是 ROS/C++ 架构与接口设计，文本结构和代码事实比浏览器可视化更直接 |
| 2026-04-24 | 先对齐现有代码事实，再讨论 tracker 方案 | 当前 spec 已包含未落地的 tracker 设计，需区分“已实现聚类”和“待设计跟踪” |
| 2026-04-24 | 第一版 tracker 采用当前 spec 完整版 | 用户选择 B：第一版包含 adaptive Q/R、cov limit、history、coasting gate 放松等全部参数 |
| 2026-04-24 | 新增 track 内部的多模型 Kalman 组件作为设计约束 | 用户要求避免 `dynamic_object_tracker` 过于臃肿；进一步查看 LDOT 后确认应理解为 tracker 内部持有滤波器组件，而不是三套平级主流程模块 |
| 2026-04-24 | 确认 LDOT 风格的 tracker 内部滤波组件边界 | 用户确认“这就是我想要的” |
| 2026-04-24 | 第一版默认所有目标使用 3D CA | 用户选择默认 3D CA；类别切换不启用，但保留 `MotionModelType` 和 factory 结构 |
| 2026-04-24 | 已确认内容应及时落入 spec | 用户指出已确认内容不应只停留在过程记录或对话里 |
| 2026-04-24 | 合并 motion model 与 Kalman filter 文件组 | 为减少文件数量，第一版把 `MotionModel` 接口和 CA3D 实现放进 `multi_model_kalman_filter.{h,cpp}` |
| 2026-04-24 | `DynamicObjectArray` 归属 tracker，聚类保留 marker | 用户确认不采用把 tracker 塞进 clusterer 或新增 `TrackedObjectArray`；`DynamicObject.msg` 移除预测字段，预测未来走独立话题 |
| 2026-04-24 | 检测和跟踪可视化分开输出 | `/ldop/dynamic_object_markers` 保留为 clusterer 的当前帧检测框，新增 `/ldop/dynamic_track_markers` 显示 tracker 稳定轨迹 |
| 2026-04-24 | 数据关联求解器第一版使用 Hungarian | 用户要求直接保持与 `/home/st/catkin/LDOT/ldot_detector` 一致，不采用贪婪最小代价分配 |
| 2026-04-25 | 设计阶段结束，转入独立实施计划 | 用户要求“转成实施计划”，因此新增 `docs/superpowers/plans/2026-04-25-dynamic-object-tracking-implementation-plan.md` |
| 2026-04-25 | 实施计划 handoff 按 `Superpowers` 标准执行 | `writing-plans` 明确要求在计划生成后提供两种执行方式：Subagent-Driven 或 Inline Execution |
| 2026-04-25 | 执行方式选定为 `Subagent-Driven` | 用户明确选择 `Subagent-Driven (recommended)` |
| 2026-04-25 | 使用项目内 `.worktrees/` 创建隔离工作区 | 用户明确要求把 worktree 建在项目内 `.worktrees/` |
| 2026-04-25 | 阶段 1 以“先实现、再做 spec 复核”的方式通过 | 子代理实现消息契约与 clusterer 接口改造后，复核确认未越阶段塞入 tracker 主流程 |
| 2026-04-25 | 实施计划 5 个阶段已在 worktree 完成 | 子代理额度触顶后，主代理继续完成 phase 3 code review、phase 4 集成、phase 5 参数与验证 |

## 已收口与后续阶段设计项

| 优先级 | 项 | 状态/结论 | 备注 |
|--------|----|-----------|------|
| P0 | 重复轨迹清理 | 已收口：第一版不做 `removeDuplicateTracks()` | 先暴露问题，不在主流程静默合并 |
| P0 | Hungarian 实现细节 | 已收口：复刻 LDOT 当前简化语义 | 行/列归约后从零候选中按原始代价选不冲突匹配 |
| P1 | 参数组织方式 | 已收口：继续使用扁平 `tracking_*` | 保持与当前 `ldop.yaml` 风格一致 |
| P1 | adaptive Q/R 公式 | 已收口：语义对齐 LDOT 当前 innovation-window + EMA 实现 | 不另设计新公式 |
| P1 | 轨迹 history 内容 | 已收口：`TrackHistorySample = stamp + position + velocity + matched` | 第一版不保存协方差历史 |
| P2 | `/ldop/dynamic_track_markers` 细节 | 已收口：固定为 `LINE_STRIP` 历史线 + `SPHERE` 头部 | coasting 时降低 alpha |
| P2 | `/ldop/dynamic_object_markers` 细节 | 已收口：沿用现有 clusterer 样式 | `dynamic_object_bbox` + `LINE_LIST` + 橙色线框 |
| P2 | 未来预测消息 | 后续阶段设计项 | 第三阶段再定义独立预测消息字段 |
| P2 | 未来类别和多模型切换 | 后续阶段设计项 | 第一版只保留 `MotionModelType` 和 factory |

## 错误记录

| 时间 | 错误 | 处理 |
|------|------|------|
| 2026-04-24 | 一次 `rg` 检查命令的正则中包含未转义反引号，shell 误解释了片段 | 后续改用单引号或避免在 shell 正则中直接混用反引号；已重新用安全命令完成检查 |
| 2026-04-25 | 一次读取 brainstorming skill 路径时命中文件不存在的旧缓存路径 | 直接回退到现有规划文件和仓库上下文继续，不影响设计文档收口 |
