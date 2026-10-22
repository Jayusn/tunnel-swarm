#pragma once
#include <Eigen/Core>
#include <Eigen/QR>
#include <algorithm>
#include <vector>

namespace mot_mapping {
struct MotionSample { double time; Eigen::Vector3d position; };
struct MotionContinuity {
  bool ready = false;
  bool continuous = false;
  double max_residual = 0.0;
};
// A short constant-acceleration fit admits a smooth swing reversal, while
// rejecting changes of sampled wall fragments. No object-shape assumption.
inline MotionContinuity motionContinuity(const std::vector<MotionSample>& samples,
                                        double duration, double tolerance) {
  MotionContinuity result;
  if (samples.size() < 4) return result;
  const double span = samples.back().time - samples.front().time;
  if (!std::isfinite(span) || span < duration - 1e-3) return result;
  result.ready = true;
  Eigen::MatrixXd design(samples.size(), 3), positions(samples.size(), 3);
  for (size_t i = 0; i < samples.size(); ++i) {
    if (!samples[i].position.allFinite() || !std::isfinite(samples[i].time) ||
        (i && (samples[i].time <= samples[i-1].time ||
               samples[i].time - samples[i-1].time > 0.25))) return result;
    const double t = (samples[i].time - samples.front().time) / span;
    design.row(i) << 1.0, t, t*t;
    positions.row(i) = (samples[i].position - samples.front().position).transpose();
  }
  const Eigen::MatrixXd residual = positions - design * design.colPivHouseholderQr().solve(positions);
  for (int i=0; i<residual.rows(); ++i)
    result.max_residual = std::max(result.max_residual, residual.row(i).norm());
  result.continuous = std::isfinite(result.max_residual) && result.max_residual <= tolerance;
  return result;
}
}
