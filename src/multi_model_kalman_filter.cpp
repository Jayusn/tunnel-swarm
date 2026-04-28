#if !defined(EIGEN_DONT_PARALLELIZE)
#if defined(__has_include)
#if !__has_include(<omp.h>)
// 诊断环境缺少 omp.h 时禁用 Eigen 并行分支，避免编辑期级联误报。
#define EIGEN_DONT_PARALLELIZE
#endif
#endif
#endif

#if defined(__clang__) && !defined(EIGEN_DONT_VECTORIZE)
// clangd 10 + GCC11 头组合下可能出现 _mm_prefetch 冲突，诊断路径禁用 Eigen 向量化。
#define EIGEN_DONT_VECTORIZE
#endif

#include <ldop/multi_model_kalman_filter.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ldopcore {

// 运动模型只提供 F/H/Q/R 等矩阵；轨迹生命周期、ROS 参数和关联逻辑都由 tracker 负责。
class MotionModel {
 public:
  virtual ~MotionModel() = default;

  virtual MotionModelType type() const = 0;
  virtual void setDt(double dt) = 0;
  virtual double dt() const = 0;
  virtual int stateDim() const = 0;
  virtual int measurementDim() const = 0;
  virtual const Eigen::MatrixXd& transition() const = 0;
  virtual const Eigen::MatrixXd& measurement() const = 0;
  virtual const Eigen::MatrixXd& baseProcessNoise() const = 0;
  virtual const Eigen::MatrixXd& baseMeasurementNoise() const = 0;
};

namespace {

class CA3DMotionModel final : public ldopcore::MotionModel {
 public:
  explicit CA3DMotionModel(const MultiModelKalmanFilterConfig& config) {
    // 噪声强制为正，避免后续协方差更新出现奇异或非物理负噪声。
    process_noise_ = std::max(config.process_noise, 1e-9);
    measurement_noise_ = std::max(config.measurement_noise, 1e-9);
    setDt(config.dt);
  }

  MotionModelType type() const override {
    return MotionModelType::CA3D;
  }

  void setDt(double dt) override {
    dt_ = std::isfinite(dt) && dt > 0.0 ? dt : 1e-3;

    // CA3D 状态布局为 [pos, vel, acc]，观测只直接测量 position。
    // KF: F=transition_, H=measurement_。
    transition_ = Eigen::MatrixXd::Identity(9, 9);
    measurement_ = Eigen::MatrixXd::Zero(3, 9);
    measurement_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

    const double dt2 = dt_ * dt_;
    const double dt3 = dt2 * dt_;
    const double dt4 = dt2 * dt2;
    const double dt5 = dt2 * dt3;

    for (int axis = 0; axis < 3; ++axis) {
      // 每个轴独立使用 constant acceleration 离散转移：p += v*dt + 0.5*a*dt^2, v += a*dt。
      transition_(axis, axis + 3) = dt_;
      transition_(axis, axis + 6) = 0.5 * dt2;
      transition_(axis + 3, axis + 6) = dt_;
    }

    // 白 jerk 驱动的 CA 模型离散过程噪声；3 个轴共享同一 3x3 模板。
    // KF: Q=process_noise_matrix_。
    Eigen::Matrix3d axis_process_noise;
    axis_process_noise << dt5 / 20.0, dt4 / 8.0, dt3 / 6.0,
                          dt4 / 8.0, dt3 / 3.0, dt2 / 2.0,
                          dt3 / 6.0, dt2 / 2.0, dt_;

    process_noise_matrix_ = Eigen::MatrixXd::Zero(9, 9);
    for (int axis = 0; axis < 3; ++axis) {
      // 将单轴 pos/vel/acc 噪声块嵌入完整 9x9 矩阵，轴之间暂不建模相关性。
      process_noise_matrix_(axis, axis) = process_noise_ * axis_process_noise(0, 0);
      process_noise_matrix_(axis, axis + 3) = process_noise_ * axis_process_noise(0, 1);
      process_noise_matrix_(axis, axis + 6) = process_noise_ * axis_process_noise(0, 2);
      process_noise_matrix_(axis + 3, axis) = process_noise_ * axis_process_noise(1, 0);
      process_noise_matrix_(axis + 3, axis + 3) = process_noise_ * axis_process_noise(1, 1);
      process_noise_matrix_(axis + 3, axis + 6) = process_noise_ * axis_process_noise(1, 2);
      process_noise_matrix_(axis + 6, axis) = process_noise_ * axis_process_noise(2, 0);
      process_noise_matrix_(axis + 6, axis + 3) = process_noise_ * axis_process_noise(2, 1);
      process_noise_matrix_(axis + 6, axis + 6) = process_noise_ * axis_process_noise(2, 2);
    }

    // KF: R=measurement_noise_matrix_。
    measurement_noise_matrix_ =
        measurement_noise_ * Eigen::MatrixXd::Identity(measurementDim(), measurementDim());
  }

  double dt() const override {
    return dt_;
  }

  int stateDim() const override {
    return 9;
  }

  int measurementDim() const override {
    return 3;
  }

  const Eigen::MatrixXd& transition() const override {
    return transition_;
  }

  const Eigen::MatrixXd& measurement() const override {
    return measurement_;
  }

  const Eigen::MatrixXd& baseProcessNoise() const override {
    return process_noise_matrix_;
  }

  const Eigen::MatrixXd& baseMeasurementNoise() const override {
    return measurement_noise_matrix_;
  }

 private:
  double dt_{0.1};
  double process_noise_{1.0};
  double measurement_noise_{0.5};
  Eigen::MatrixXd transition_;
  Eigen::MatrixXd measurement_;
  Eigen::MatrixXd process_noise_matrix_;
  Eigen::MatrixXd measurement_noise_matrix_;
};

std::unique_ptr<MotionModel> createMotionModel(const MotionModelType model_type,
                                               const MultiModelKalmanFilterConfig& config) {
  switch (model_type) {
    case MotionModelType::CA3D:
      return std::make_unique<CA3DMotionModel>(config);
  }

  throw std::invalid_argument("unsupported motion model");
}

bool isFiniteVector(const Eigen::Vector3d& vector) {
  return vector.allFinite();
}

}  // namespace

MultiModelKalmanFilter::MultiModelKalmanFilter(const MultiModelKalmanFilterConfig& config)
    : config_(config) {
  // 滤波器作为内部算法对象，构造时完成配置合法化，之后不再接触 ROS 参数服务器。
  config_.dt = std::isfinite(config_.dt) && config_.dt > 0.0 ? config_.dt : 0.1;
  config_.process_noise = std::max(config_.process_noise, 1e-9);
  config_.measurement_noise = std::max(config_.measurement_noise, 1e-9);
  config_.initial_pos_covariance = std::max(config_.initial_pos_covariance, 1e-9);
  config_.initial_vel_covariance = std::max(config_.initial_vel_covariance, 1e-9);
  config_.initial_acc_covariance = std::max(config_.initial_acc_covariance, 1e-9);
  config_.adaptive_q_alpha = std::clamp(config_.adaptive_q_alpha, 0.0, 1.0);
  config_.adaptive_r_alpha = std::clamp(config_.adaptive_r_alpha, 0.0, 1.0);
  config_.min_noise_ratio = std::max(config_.min_noise_ratio, 1e-6);
  config_.max_pos_covariance = std::max(config_.max_pos_covariance, 1e-9);
  config_.max_vel_covariance = std::max(config_.max_vel_covariance, 1e-9);
  config_.max_acc_covariance = std::max(config_.max_acc_covariance, 1e-9);

  motion_model_ = createMotionModel(MotionModelType::CA3D, config_);
  motion_model_->setDt(config_.dt);
  adaptive_process_noise_ = motion_model_->baseProcessNoise();
  adaptive_measurement_noise_ = motion_model_->baseMeasurementNoise();
  min_process_noise_ = adaptive_process_noise_ * config_.min_noise_ratio;
  min_measurement_noise_ = adaptive_measurement_noise_ * config_.min_noise_ratio;
  initialize(Eigen::Vector3d::Zero());
}

MultiModelKalmanFilter::~MultiModelKalmanFilter() = default;

MultiModelKalmanFilter::MultiModelKalmanFilter(MultiModelKalmanFilter&&) noexcept = default;

MultiModelKalmanFilter& MultiModelKalmanFilter::operator=(MultiModelKalmanFilter&&) noexcept =
    default;

void MultiModelKalmanFilter::setDt(const double dt) {
  // 这里只为后续 predict() 准备当前帧模型参数，不推进状态本身。
  // 参考 LDOT：setDt() 只更新运动模型里的 dt/F，不在这里重建 adaptive Q/R。
  motion_model_->setDt(dt);
  if (config_.innovation_buffer <= 1U) {
    adaptive_process_noise_ = motion_model_->baseProcessNoise();
  }
}

void MultiModelKalmanFilter::initialize(const Eigen::Vector3d& measurement) {
  if (!isFiniteVector(measurement)) {
    throw std::invalid_argument("initial measurement must be finite");
  }

  // 参考 LDOT：新轨迹初始化时直接使用首帧 detection 作为位置状态，而不是从零状态
  // 先做一次 measurement update；否则在较大 R 下会把初值拉到原点与观测之间。
  state_ = Eigen::VectorXd::Zero(motion_model_->stateDim());
  state_.segment<3>(0) = measurement;

  covariance_ = Eigen::MatrixXd::Zero(motion_model_->stateDim(), motion_model_->stateDim());
  covariance_.block<3, 3>(0, 0) = config_.initial_pos_covariance * Eigen::Matrix3d::Identity();
  covariance_.block<3, 3>(3, 3) = config_.initial_vel_covariance * Eigen::Matrix3d::Identity();
  covariance_.block<3, 3>(6, 6) = config_.initial_acc_covariance * Eigen::Matrix3d::Identity();

  adaptive_process_noise_ = motion_model_->baseProcessNoise();
  adaptive_measurement_noise_ = motion_model_->baseMeasurementNoise();
  innovation_history_.clear();
  clampCovariance();
}

void MultiModelKalmanFilter::predict() {
  // 预测阶段只使用运动模型，不引入新观测。
  // KF: x = F x, P = F P F^T + Q。
  state_ = motion_model_->transition() * state_;
  covariance_ = motion_model_->transition() * covariance_ * motion_model_->transition().transpose() +
                adaptive_process_noise_;
  clampCovariance();
}

void MultiModelKalmanFilter::update(const Eigen::Vector3d& measurement) {
  if (!isFiniteVector(measurement)) {
    throw std::invalid_argument("measurement must be finite");
  }

  const Eigen::MatrixXd measurement_matrix = motion_model_->measurement();
  Eigen::MatrixXd predicted_covariance = covariance_;
  // innovation = 当前观测与预测观测的差值，是后续 Kalman 增益和自适应噪声的共同输入。
  // KF: y = z - H x。
  const Eigen::Vector3d innovation = measurement - measurement_matrix * state_;

  // 参考 LDOT：adaptive R/Q 基于当前 P_pred 和 innovation，在正式更新前完成估计。
  predicted_covariance = updateAdaptiveNoise(innovation, predicted_covariance);

  // KF: S = H P H^T + R。
  const Eigen::MatrixXd innovation_covariance =
      measurement_matrix * predicted_covariance * measurement_matrix.transpose() + adaptive_measurement_noise_;
  // 当前观测维度固定为 3，直接求逆足够简单，也能避开 GCC11 + Eigen 在 LDLT 上的模板兼容噪声。
  // KF: K = P H^T S^{-1}。
  const Eigen::MatrixXd kalman_gain =
      predicted_covariance * measurement_matrix.transpose() * innovation_covariance.inverse();

  state_ += kalman_gain * innovation;

  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(state_.size(), state_.size());
  const Eigen::MatrixXd measurement_projection = identity - kalman_gain * measurement_matrix;
  // Joseph 形式协方差更新比简单 (I-KH)P 更稳，能更好维持半正定性。
  // KF: P = (I-KH) P (I-KH)^T + K R K^T。
  covariance_ = measurement_projection * predicted_covariance * measurement_projection.transpose() +
                kalman_gain * adaptive_measurement_noise_ * kalman_gain.transpose();
  clampCovariance();
}

const Eigen::VectorXd& MultiModelKalmanFilter::state() const {
  return state_;
}

const Eigen::MatrixXd& MultiModelKalmanFilter::covariance() const {
  return covariance_;
}

Eigen::Vector3d MultiModelKalmanFilter::position() const {
  return state_.segment<3>(0);
}

Eigen::Vector3d MultiModelKalmanFilter::velocity() const {
  return state_.segment<3>(3);
}

MotionModelType MultiModelKalmanFilter::modelType() const {
  return motion_model_->type();
}

Eigen::MatrixXd MultiModelKalmanFilter::updateAdaptiveNoise(
    const Eigen::Vector3d& innovation,
    const Eigen::MatrixXd& predicted_covariance) {
  if (config_.innovation_buffer <= 1U) {
    // buffer 小于等于 1 时视为关闭自适应噪声，Q/R 回到当前 dt 下的基础矩阵。
    adaptive_measurement_noise_ = motion_model_->baseMeasurementNoise();
    adaptive_process_noise_ = motion_model_->baseProcessNoise();
    innovation_history_.clear();
    return predicted_covariance;
  }

  innovation_history_.push_back(innovation);
  if (innovation_history_.size() > config_.innovation_buffer) {
    innovation_history_.pop_front();
  }
  if (innovation_history_.size() < config_.innovation_buffer) {
    return predicted_covariance;
  }

  Eigen::MatrixXd innovation_sample_covariance = Eigen::MatrixXd::Zero(
      motion_model_->measurementDim(), motion_model_->measurementDim());
  for (const auto& sample : innovation_history_) {
    innovation_sample_covariance += sample * sample.transpose();
  }
  innovation_sample_covariance /=
      static_cast<double>(innovation_history_.size());

  const Eigen::MatrixXd measurement_matrix = motion_model_->measurement();
  const Eigen::MatrixXd estimated_measurement_noise =
      innovation_sample_covariance -
      measurement_matrix * predicted_covariance * measurement_matrix.transpose();
  for (int axis = 0; axis < motion_model_->measurementDim(); ++axis) {
    const double diagonal_estimate =
        std::max(estimated_measurement_noise(axis, axis), 0.0);
    const double smoothed_value =
        config_.adaptive_r_alpha * diagonal_estimate +
        (1.0 - config_.adaptive_r_alpha) * adaptive_measurement_noise_(axis, axis);
    adaptive_measurement_noise_(axis, axis) =
        std::max(smoothed_value, min_measurement_noise_(axis, axis));
  }

  const Eigen::MatrixXd temporary_innovation_covariance =
      measurement_matrix * predicted_covariance * measurement_matrix.transpose() +
      adaptive_measurement_noise_;
  const Eigen::MatrixXd temporary_kalman_gain =
      predicted_covariance * measurement_matrix.transpose() *
      temporary_innovation_covariance.inverse();
  const Eigen::MatrixXd previous_process_noise = adaptive_process_noise_;
  const Eigen::MatrixXd propagated_covariance =
      predicted_covariance - previous_process_noise;
  const Eigen::MatrixXd unexplained_innovation_covariance =
      innovation_sample_covariance -
      measurement_matrix * propagated_covariance * measurement_matrix.transpose() -
      adaptive_measurement_noise_;
  const Eigen::MatrixXd estimated_process_noise =
      temporary_kalman_gain * unexplained_innovation_covariance *
      temporary_kalman_gain.transpose();
  for (int axis = 0; axis < predicted_covariance.rows(); ++axis) {
    const double diagonal_estimate =
        std::max(estimated_process_noise(axis, axis), 0.0);
    const double smoothed_value =
        config_.adaptive_q_alpha * diagonal_estimate +
        (1.0 - config_.adaptive_q_alpha) * adaptive_process_noise_(axis, axis);
    adaptive_process_noise_(axis, axis) =
        std::max(smoothed_value, min_process_noise_(axis, axis));
  }

  // 参考 LDOT：Q 从 Q_old 更新到 Q_new 后，当前帧的 P_pred 也同步替换成与新 Q 一致的版本。
  // 否则本次 Kalman 增益仍在使用旧 Q 对应的预测协方差，自适应 Q 会晚一帧才真正生效。
  Eigen::MatrixXd adjusted_predicted_covariance =
      predicted_covariance - previous_process_noise + adaptive_process_noise_;
  adjusted_predicted_covariance =
      0.5 * (adjusted_predicted_covariance + adjusted_predicted_covariance.transpose());
  return adjusted_predicted_covariance;
}

void MultiModelKalmanFilter::clampCovariance() {
  // 数值计算后先强制对称，避免极小非对称误差在后续 inverse/乘法中逐步放大。
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());

  Eigen::VectorXd scale = Eigen::VectorXd::Ones(covariance_.rows());
  const auto clamp_block = [this, &scale](const int offset, const double limit) {
    for (int axis = 0; axis < 3; ++axis) {
      const int index = offset + axis;
      covariance_(index, index) = std::max(covariance_(index, index), 0.0);
      if (covariance_(index, index) > limit) {
        // 先记录缩放系数，最后统一左右乘，保留协方差的相关项比例关系。
        scale(index) = std::sqrt(limit / std::max(covariance_(index, index), 1e-9));
      }
    }
  };

  clamp_block(0, config_.max_pos_covariance);
  clamp_block(3, config_.max_vel_covariance);
  clamp_block(6, config_.max_acc_covariance);
  covariance_ = scale.asDiagonal() * covariance_ * scale.asDiagonal();
}

std::unique_ptr<MultiModelKalmanFilter> createMultiModelKalmanFilter(
    const MotionModelType model_type,
    const MultiModelKalmanFilterConfig& config) {
  switch (model_type) {
    case MotionModelType::CA3D:
      return std::make_unique<MultiModelKalmanFilter>(config);
  }

  throw std::invalid_argument("unsupported motion model");
}

}  // namespace ldopcore
