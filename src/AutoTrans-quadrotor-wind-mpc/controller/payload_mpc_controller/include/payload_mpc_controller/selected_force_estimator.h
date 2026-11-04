#pragma once
#include "multi_optimization_based_force_estimator.hpp"
#include "aerothrow_ndob.h"

namespace PayloadMPC {
// Startup-only selection; both online and offline use this exact dispatch.
// The window estimator is deliberately unchanged.
class SelectedForceEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void init(MpcParams& p) {
    ndob_selected_ = p.force_estimator_param_.method == "aerothrow_ndob";
    if (!ndob_selected_ && p.force_estimator_param_.method != "window_opt")
      throw std::invalid_argument("unknown force estimator method");
    if (ndob_selected_) ndob_.init(p); else window_.init(p);
  }
  void reset() { if (ndob_selected_) ndob_.reset(); else window_.reset(); }
  void setSystemState(const Eigen::Vector3d& a, const Eigen::Quaterniond& q,
                      const Eigen::Vector4d& rpm) {
    if (ndob_selected_) ndob_.setSystemState(a, q, rpm);
    else window_.setSystemState(a, q, rpm);
  }
  bool caculate_force(Eigen::Vector3d& fl, Eigen::Vector3d& fq) {
    return ndob_selected_ ? ndob_.caculate_force(fl, fq) : window_.caculate_force(fl, fq);
  }
  bool windowFull() const { return ndob_selected_ ? ndob_.windowFull() : window_.windowFull(); }
  size_t sampleCount() const { return ndob_selected_ ? ndob_.sampleCount() : window_.sampleCount(); }
 private:
  bool ndob_selected_{false};
  MultiOptForceEstimator window_;
  AeroThrowNDOB ndob_;
};
}  // namespace PayloadMPC
