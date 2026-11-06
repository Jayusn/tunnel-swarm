#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "mpc_params.h"
#include "lowpassfilter2p.h"

namespace PayloadMPC {
// AeroThrow Eq. (12)-(13), arXiv:2507.13903v1. World force is positive
// in m*a = T*R*thrust_axis_body + m*g + f. The input is BODY SPECIFIC FORCE.
class AeroThrowNDOB {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static bool validParameters(double rate, double cutoff, double frequency) {
    return std::isfinite(frequency) && frequency > 0.0 &&
        std::isfinite(rate) && rate > 0.0 && rate / frequency < 2.0 &&
        std::isfinite(cutoff) && cutoff > 0.0 && cutoff < frequency / 2.0;
  }
  void init(const MpcParams& p) {
    const auto& f = p.force_estimator_param_;
    if (!validParameters(f.ndob_rate, f.ndob_cutoff_freq, p.ctrl_freq_max_))
      throw std::invalid_argument("invalid AeroThrow NDOB gain/filter frequency");
    mass_ = p.dyn_params_.mass_q;
    sqrt_kf_ = f.sqrt_kf;
    thrust_axis_body_ = f.thrust_axis_body.normalized();
    alpha_ = f.ndob_rate / p.ctrl_freq_max_;
    max_force_ = f.max_force;
    separate_z_limit_ = f.force_axis_gain_z == 0.0;
    enabled_ = f.enable_force_estimation;
    required_samples_ = f.max_queue;
    filter_.set_cutoff_frequency(p.ctrl_freq_max_, f.ndob_cutoff_freq);
    reset();
  }
  void reset() {
    count_ = 0;
    residual_.setZero();
    state_.setZero();
    filter_.reset(Eigen::Vector3d::Zero());
  }
  void setSystemState(const Eigen::Vector3d& specific_force,
                      const Eigen::Quaterniond& rotation,
                      const Eigen::Vector4d& rpm) {
    const Eigen::Quaterniond q = rotation.normalized();
    residual_ = mass_ * (q * specific_force) -
        (sqrt_kf_ * rpm).squaredNorm() * (q.toRotationMatrix() * thrust_axis_body_);
    count_ = std::min(count_ + 1, static_cast<size_t>(std::max(1, required_samples_)));
  }
  // One call per control tick, including ticks holding the last valid input.
  // Do not feed filtered/clipped output back into the Euler observer state.
  bool caculate_force(Eigen::Vector3d& fl, Eigen::Vector3d& fq) {
    fl.setZero();
    fq.setZero();
    if (!enabled_ || count_ == 0) return false;
    state_ += alpha_ * (residual_ - state_);
    Eigen::Vector3d bounded = state_;
    if (separate_z_limit_) {
      const double xy_norm = bounded.head<2>().norm();
      if (xy_norm > max_force_) bounded.head<2>() *= max_force_ / xy_norm;
      bounded.z() = std::max(-max_force_, std::min(max_force_, bounded.z()));
    } else if (bounded.norm() > max_force_) bounded *= max_force_ / bounded.norm();
    fq = filter_.apply(bounded);
    if (!fq.allFinite()) { reset(); fq.setZero(); return false; }
    if (separate_z_limit_) {
      const double xy_norm = fq.head<2>().norm();
      if (xy_norm > max_force_) fq.head<2>() *= max_force_ / xy_norm;
      fq.z() = std::max(-max_force_, std::min(max_force_, fq.z()));
    } else if (fq.norm() > max_force_) fq *= max_force_ / fq.norm();
    return true;
  }
  bool windowFull() const {
    return required_samples_ > 0 && count_ >= static_cast<size_t>(required_samples_);
  }
  size_t sampleCount() const { return count_; }
  const Eigen::Vector3d& observerState() const { return state_; }
 private:
  double mass_{1.0}, sqrt_kf_{0.0}, alpha_{0.0}, max_force_{5.0};
  bool enabled_{false}, separate_z_limit_{false};
  int required_samples_{20};
  size_t count_{0};
  Eigen::Vector3d thrust_axis_body_{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d residual_{Eigen::Vector3d::Zero()}, state_{Eigen::Vector3d::Zero()};
  LowPassFilter2p<Eigen::Vector3d> filter_;
};
}  // namespace PayloadMPC
