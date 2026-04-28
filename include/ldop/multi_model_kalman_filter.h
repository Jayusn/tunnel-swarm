#pragma once

#include <cstddef>
#include <deque>
#include <memory>

#include <Eigen/Dense>

namespace ldopcore {

enum class MotionModelType {
  // Constant Acceleration 3D：状态为 [pos, vel, acc]，每个分量各 3 维。
  CA3D,
};

// 单条轨迹滤波器的算法参数；由 tracker 统一读取 ROS 参数后传入，滤波器本身不直接访问参数服务器。
struct MultiModelKalmanFilterConfig {
  // 默认预测时间步长，真实帧间隔会由 tracker 在每帧 setDt() 覆盖。
  double dt{0.1};
  // 基础过程噪声和观测噪声方差，内部会转换为 Q/R 矩阵。
  double process_noise{1.0};
  double measurement_noise{0.5};
  // 新轨迹创建时 pos/vel/acc 三个状态块的初始协方差。
  double initial_pos_covariance{1.0};
  double initial_vel_covariance{1.0};
  double initial_acc_covariance{1.0};
  // Q/R 对角项的 EMA 平滑系数，越大越快跟随最近 innovation 协方差估计。
  double adaptive_q_alpha{0.2};
  double adaptive_r_alpha{0.2};
  // 自适应 Q/R 的最小阈值比例，按基础 Q/R 乘该比例得到下限矩阵。
  double min_noise_ratio{0.1};
  // 协方差上限用于防止长时间 coast 或异常 dt 后数值发散。
  double max_pos_covariance{1e3};
  double max_vel_covariance{1e3};
  double max_acc_covariance{1e3};
  // innovation 统计窗口长度；小于等于 1 时关闭自适应 Q/R。
  std::size_t innovation_buffer{10U};
};

// 运动模型抽象当前只作为实现细节存在，phase 2 不对外开放自定义注入。
class MotionModel;

class MultiModelKalmanFilter {
 public:
  // 构造只做算法对象初始化；ROS 参数加载归 DynamicObjectTracker 所有。
  explicit MultiModelKalmanFilter(const MultiModelKalmanFilterConfig& config = {});
  ~MultiModelKalmanFilter();

  MultiModelKalmanFilter(const MultiModelKalmanFilter&) = delete;
  MultiModelKalmanFilter& operator=(const MultiModelKalmanFilter&) = delete;
  MultiModelKalmanFilter(MultiModelKalmanFilter&&) noexcept;
  MultiModelKalmanFilter& operator=(MultiModelKalmanFilter&&) noexcept;

  // 为下一次 predict() 设置当前时间步长；这里只更新运动模型参数，不执行状态外推。
  // config.dt 只是构造期默认值，真实帧间隔通常由 tracker 在每帧预测前覆盖进来。
  // 当前生效的 adaptive Q/R 由 update() 内的自适应估计维护，不在这里重置。
  void setDt(double dt);
  // 新轨迹初始化时直接把位置状态放到首帧 detection 中心，避免零状态经一次 update 后
  // 仍然残留“朝原点收缩”的偏差；这一步语义对齐 LDOT 的 initialize(detection)。
  void initialize(const Eigen::Vector3d& measurement);
  // 按最近一次 setDt() 准备好的运动模型做一步预测，状态会从 pos/vel/acc 前向外推。
  void predict();
  // 使用 3D 位置观测更新状态；观测值必须已经在轨迹使用的同一坐标系中。
  void update(const Eigen::Vector3d& measurement);

  // 下游只读访问完整状态/协方差，实际公共输出通常只取 position() 和 velocity()。
  const Eigen::VectorXd& state() const;
  const Eigen::MatrixXd& covariance() const;
  Eigen::Vector3d position() const;
  Eigen::Vector3d velocity() const;
  MotionModelType modelType() const;

 private:
  // 参考 LDOT：用 innovation 协方差先估计 R，再用临时 K 和 P_pred-Q_old 估计 Q。
  // 返回值是已经按 Q_old -> Q_new 同步修正过的当前 P_pred，供本次 update() 继续使用。
  Eigen::MatrixXd updateAdaptiveNoise(const Eigen::Vector3d& innovation,
                                      const Eigen::MatrixXd& predicted_covariance);
  void clampCovariance();

  MultiModelKalmanFilterConfig config_;
  std::unique_ptr<MotionModel> motion_model_;
  // 当前 CA3D 状态布局为 [px, py, pz, vx, vy, vz, ax, ay, az]。
  Eigen::VectorXd state_;
  // 与 state_ 同维度的协方差矩阵，始终在 clampCovariance() 后保持对称。
  Eigen::MatrixXd covariance_;
  // 当前生效的自适应 Q/R；Q 只更新对角项，保留模型基础矩阵里的相关结构。
  Eigen::MatrixXd adaptive_process_noise_;
  Eigen::MatrixXd adaptive_measurement_noise_;
  // 参考 LDOT：最小 Q/R 阈值由初始化时的基础噪声矩阵给出，不随每帧 dt 重新缩小。
  Eigen::MatrixXd min_process_noise_;
  Eigen::MatrixXd min_measurement_noise_;
  // 保存最近完整 innovation 向量，用样本协方差 C_gamma 估计自适应 Q/R。
  std::deque<Eigen::Vector3d> innovation_history_;
};

std::unique_ptr<MultiModelKalmanFilter> createMultiModelKalmanFilter(
    MotionModelType model_type,
    const MultiModelKalmanFilterConfig& config = {});

}  // namespace ldopcore
