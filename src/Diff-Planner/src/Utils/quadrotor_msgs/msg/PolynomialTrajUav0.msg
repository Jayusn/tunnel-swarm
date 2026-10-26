Header header

# AutoTrans 使用的轨迹编号，从 1 开始。
uint32 trajectory_id

# 轨迹动作命令。
uint32 ACTION_ADD             = 1
uint32 ACTION_ABORT           = 2
uint32 ACTION_WARN_START      = 3
uint32 ACTION_WARN_FINAL      = 4
uint32 ACTION_WARN_IMPOSSIBLE = 5
uint32 action

# 每个元素是一段三维位置多项式，系数按 Eigen 列主序保存。
quadrotor_msgs/PolynomialMatrix[] trajectory

# true 时表示同时提供 FUEL 的 yaw 多项式；false 时控制器使用原有 yaw 回退逻辑。
bool has_yaw
quadrotor_msgs/PolynomialMatrix[] yaw_trajectory
