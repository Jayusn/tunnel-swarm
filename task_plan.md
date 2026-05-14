# 动态对象跟踪设计收敛计划

## 目标

完成 `docs/superpowers/specs/2026-04-24-dynamic-object-tracking-design.md` 的设计收口，并输出配套实施计划 `docs/superpowers/plans/2026-04-25-dynamic-object-tracking-implementation-plan.md`，为下一步实现提供可执行顺序、文件范围和验证闭环。全过程继续使用 `planning-with-files-zh` 记录。

## 当前状态

- 当前阶段：阶段 16 已完成；第二阶段“动态点云聚类、检测与跟踪”已标记完成，并已同步当前阶段文档、路线图和过程记录。
- 当前主线：进入第三阶段“动态目标短时轨迹预测”，后续设计应基于 tracker 已稳定输出的 `id/object_class/motion_model_type/model_state/model_covariance`。
- 上一阶段：阶段 10 已完成，第一版单 CA3D 跟踪实施计划已在独立 worktree 中落地并完成临时构建/测试验证。
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
| 11 | complete | 按 `superpowers:brainstorming` 调研当前 LDOP 与 `/home/st/catkin/LDOT/ldot_detector`，收敛目标分类和多运动模型 KF/EKF 的设计边界 |
| 12 | complete | 用户审阅 `docs/superpowers/specs/2026-04-30-dynamic-object-classification-multimodel-design.md`，确认后进入实施计划 |
| 13 | complete | 执行 `docs/superpowers/plans/2026-04-30-dynamic-object-classification-multimodel-implementation-plan.md`，实现目标分类、证据积分和模型切换 |
| 14 | complete | 对照第二阶段目标审查当前实现、配置、消息、文档、构建和测试，判断是否真正完成 |
| 15 | complete | 同步当前 `DynamicObject.msg` 对外接口文档，修复 CTRA EKF 预测前状态线性化问题；随后按用户要求删除新增测试内容 |
| 16 | complete | 按用户确认将第二阶段标记为已完成，更新 `AGENTS.md`、`README.md`、`docs/note/LDOP_design.md`、spec/plan 状态说明和过程记录 |

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
| 2026-04-30 | 新一轮目标分类和多运动模型实现先走设计门 | 用户显式要求使用 `superpowers:brainstorming`，因此在设计获批前只调研、记录和收敛方案，不修改实现代码 |
| 2026-04-30 | 本轮分类范围限定为对象类别分类 | LDOP 的动静态点云分割已经由 UFOMap 链路实现，本轮只补 human/vehicle/uav/else 类别、模型选择和模型切换 |
| 2026-04-30 | 新轨迹默认 `else + CV3D`，分类后再切模型 | 用户确认采用最贴近 LDOT 的时机：新轨迹先稳住，达到 `classification_start_frame` 后分类，若类别变化则在 Kalman update 前切到对应模型 |
| 2026-04-30 | `UNKNOWN` 与 `OTHER` 在对外消息中分开 | `UNKNOWN` 表示尚未可靠分类；`OTHER` 表示已分类但不属于 human/vehicle/uav，避免把新轨迹默认 CV3D 误解为已确认其他类别 |
| 2026-04-30 | 分类规则采用 LDOT 核心尺寸/质心规则，但适配 LDOP 数据边界 | 用户确认使用历史最大 bbox 尺寸和当前 detection 质心高度分类；不引入点云 PCA/std 或运动投票 |
| 2026-04-30 | 参数组织采用混合方案 | 生命周期/关联/分类参数继续扁平化，多模型 Kalman 参数按 LDOT 使用 `kalman_filter/...` 树状分组 |
| 2026-04-30 | `DynamicObject.msg` 本轮只新增 `uint8 object_class` | 加速度、yaw、motion model 保持 tracker 内部状态，不进入本轮对外 ROS 消息合同 |
| 2026-04-30 | 纳入 LDOT 临时合并/分离检测机制 | 用户确认本轮实现尺寸比例 + 点数比例保护，用于分类和 bbox 尺寸稳定；不扩展为轨迹去重或重新聚类 |
| 2026-04-30 | 设计文档落地但不创建提交 | 仓库规则要求任务过程中不创建中间提交；本轮只写入 spec 和规划记录，等待用户审阅 |
| 2026-04-30 | 设计文档改为中文 | 用户指出英文 spec 不符合 `AGENTS.md` 的中文默认要求；已按仓库语言规则修正 |
| 2026-04-30 | 分类稳定改为证据积分和滞回切换 | 用户认为固定间隔和连续次数还可优化；本轮删除 `classification_interval_sec`、`classification_stable_confirmations` 和 `classify_xy_distance_threshold`，可靠匹配帧更新类别分数，coasting/疑似遮挡帧冻结证据，尺寸变化确认继续使用 `classification_size_change_confirm_frames` |
| 2026-04-30 | 本轮在当前工作区内联执行 | 用户已明确说“继续”；本轮不创建中间提交，也不额外开子代理，直接在当前 LDOP 工作区按实施计划执行 |
| 2026-05-13 | 对外接口收口只更新文档描述 | 用户明确纠正：本轮不应修改 `DynamicObject.msg` 或 tracker 输出字段，只把文档与当前接口语义对齐 |
| 2026-05-13 | CTRA EKF 预测雅可比在预测前状态处计算 | 标准 EKF 协方差传播应围绕 $x_k^+$ 线性化，再推进到 $x_{k+1}^-$；否则 CTRA 的 yaw/yaw_rate 会影响错误的切线方向 |
| 2026-05-14 | 第二阶段标记为已完成，主线切到第三阶段 | 用户明确说明“第二阶段已完成”；文档状态以该结论为准，后续预测归入第三阶段 |

## 已收口与后续阶段设计项

| 优先级 | 项 | 状态/结论 | 备注 |
|--------|----|-----------|------|
| P0 | 重复轨迹清理 | 已收口：第一版不做 `removeDuplicateTracks()` | 先暴露问题，不在主流程静默合并 |
| P0 | Hungarian 实现细节 | 已收口：复刻 LDOT 当前简化语义 | 行/列归约后从零候选中按原始代价选不冲突匹配 |
| P1 | 参数组织方式 | 已收口：`tracking_*` / `classification_*` 保持扁平，多模型 Kalman 参数使用 `kalman_filter/...` 分组 | 生命周期与分类参数便于快速调试，模型噪声和协方差按模型分组避免含义混杂 |
| P1 | adaptive Q/R 公式 | 已收口：语义对齐 LDOT 当前 innovation-window + EMA 实现 | 不另设计新公式 |
| P1 | 轨迹 history 内容 | 已更新：`TrackHistorySample = stamp + object_class + motion_model_type + model_state + model_covariance + matched` | 历史样本与当前 `DynamicObject.msg` 一致，保存模型原生状态和协方差 |
| P2 | `/ldop/dynamic_track_markers` 细节 | 已更新：`LINE_STRIP` 历史线 + `SPHERE` 头部 + `TEXT_VIEW_FACING` 标签 | 标签显示 ID、类别和速度模长 |
| P2 | `/ldop/dynamic_object_markers` 细节 | 已收口：沿用现有 clusterer 样式 | `dynamic_object_bbox` + `LINE_LIST` + 橙色线框 |
| P2 | 未来预测消息 | 第三阶段设计项 | 第二阶段只输出当前跟踪状态，不把预测数组塞进 `DynamicObject.msg` |
| P2 | 目标类别和多模型切换 | 已完成 | 当前第二阶段已经输出 `object_class` 和 `motion_model_type`，并在 tracker 内按类别执行 CA2D / CTRA / CA3D / CV3D 切换 |

## 错误记录

| 时间 | 错误 | 处理 |
|------|------|------|
| 2026-04-24 | 一次 `rg` 检查命令的正则中包含未转义反引号，shell 误解释了片段 | 后续改用单引号或避免在 shell 正则中直接混用反引号；已重新用安全命令完成检查 |
| 2026-04-25 | 一次读取 brainstorming skill 路径时命中文件不存在的旧缓存路径 | 直接回退到现有规划文件和仓库上下文继续，不影响设计文档收口 |
| 2026-04-30 | 新增设计文档最初使用英文，违反 `AGENTS.md` 中“面向用户默认简体中文”的约定 | 已读取 `AGENTS.md` 并把 `docs/superpowers/specs/2026-04-30-dynamic-object-classification-multimodel-design.md` 全文改为中文，代码标识符和配置键保留英文 |
| 2026-04-30 | 一次 `rg` 检查命令把反引号放在双引号正则里，shell 误执行了 `CA3D` 片段 | 已用单引号重跑检查；后续包含反引号的 shell 正则继续用单引号 |
| 2026-04-30 | 第一次 `git diff --check` 在 `/home/st/ldop_ws` 执行，那里不是 git 仓库 | 已在 `/home/st/ldop_ws/src/LDOP` 重新执行，结果通过 |
