# LDOP Agent Notes

## 语言与输出
- 面向用户的自然语言默认使用简体中文；英文只保留代码标识符、命令、配置键、协议名等不可替换内容。

## 代理行为偏好
- 当代理推理程度为“低”时，不调用 `planning-with-files-zh` 和 `using-superpowers` 技能。

## 代码注释约定
- 以后新增或修改代码时，默认补充简要注释，优先解释非显而易见的逻辑与约束（如线程并发、坐标系假设、参数语义、接口边界），方便快速阅读。
- 注释保持简短、聚焦；避免逐行复述代码表面行为。
- 注释默认使用中文；必要术语、协议名或库 API 名可保留英文。

## 仓库与工作区边界
- 真实 git 仓库在 `src/LDOP`（`ldop_ws` 根目录不是 git 仓库）。
- 这是 ROS1 catkin 工作区；包名是 `ldop`。
- `external/install/ufomap` 是已安装依赖产物，按只读对待，不在此处改源码。

## 构建/测试（已验证命令）
- 工作目录建议：`/home/st/ldop_ws`。
- 构建单包：`catkin build ldop --no-status`
- 运行单包测试：`catkin run_tests ldop --no-status`
- 结果汇总：`catkin_test_results /home/st/ldop_ws/build/ldop`
- 仅改消息定义（`msg/*.msg`）后，必须重新 `catkin build ldop`（会触发消息重新生成）。

## 本地工具链约束（来自可执行配置）
- `.catkin_tools/profiles/default/config.yaml` 固定使用 `gcc-11/g++-11`。
- 同一配置显式注入 `-Dufomap_DIR=/home/st/ldop_ws/external/install/ufomap/lib/cmake/ufomap`。
- 同一配置将 `ufomap` 放在 blacklist；不要尝试在此工作区用 catkin 直接编它。
- `src/LDOP/.clangd` 指向 `build/ldop` 的 compilation database；若跳转/诊断异常，先确认该目录已由构建刷新。

## 运行与入口
- 节点入口：`src/LDOP/src/ldop_node.cpp` -> `ldop_node`。
- 主启动文件：`src/LDOP/launch/run_ldop.launch`（默认加载 `config/ldop.yaml` 并启动 RViz）。
- 常用运行：`roslaunch ldop run_ldop.launch`

## 架构事实（改代码前先对齐）
- `Ldop` 负责 ROS I/O 与线程调度；`UfomapMapper` 负责 UFOMap 插入、`seenFree` 判别、累计点云分类与候选聚类。
- 当前 `/ldop/static_cloud` 语义是“累计静态点云”；`/ldop/dynamic_cloud` 语义是“当前帧动态点云”。不要把二者语义改回“同一帧静/动态对”。
- `DynamicObjectArray` 已接线但仍是最小实现（基于动态点云包围盒生成观测对象）；后续扩展应在不破坏上述两路点云语义前提下进行。

## 测试注意事项
- 多个 gtest 用例依赖 ROS master；无 master 时会 `GTEST_SKIP`（非失败）。跑测试前先确保 roscore 可用。

## 当前阶段优先级（仓库既定）
- 第一优先级仍是“动静态分割 + 静态地图闭环可视化”：
  - 稳定 `/ldop/static_cloud`
  - 稳定 `/ldop/dynamic_cloud`
  - 补齐 UFOMap 静态体素地图 RViz 可视化输出（如 `/ldop/static_map_markers`）
- 在上面未稳定前，不优先做复杂跟踪/预测/规划融合。

## 提交约定
- 不为中间规划或过程产物创建提交；仅在任务完成且验证通过，或用户明确要求时提交。
