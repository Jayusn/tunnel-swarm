#pragma once
#include <Eigen/Core>
#include <Eigen/QR>
#include <deque>
#include <cmath>
#include <algorithm>

namespace mot_mapping {
// Per-track measured world positions; independent of the detector's short window.
class SwingConfirmation {
 public:
  struct Sample { double time; Eigen::Vector3d position; };
  bool empty() const { return samples_.empty(); }
  void clear() { samples_.clear(); confirmed_ = false; }
  bool update(double time, const Eigen::Vector3d& position, bool eligible) {
    if (!eligible || !std::isfinite(time) || !position.allFinite()) {
      clear(); return false;
    }
    if (!samples_.empty() && (time <= samples_.back().time)) clear();
    samples_.push_back({time, position});
    while (samples_.size() > 160 || time - samples_.front().time > 8.0)
      samples_.pop_front();
    // Track loss and motion expiry are supplied by the detector via clear().
    // Do not invent a second timeout based on sparse input frame spacing.
    // A local quadratic fit admits smooth reversal but rejects fragment jumps.
    size_t first = samples_.size() > 6 ? samples_.size() - 6 : 0;
    while (first+1 < samples_.size() && time-samples_[first].time > 0.35) ++first;
    const size_t count = samples_.size()-first;
    if (count >= 4) {
      Eigen::MatrixXd a(count,3), p(count,3);
      const double span = time - samples_[first].time;
      for (size_t i=0; i<count; ++i) {
        const double t = (samples_[first+i].time - samples_[first].time) / span;
        a.row(i) << 1.0, t, t*t;
        p.row(i) = (samples_[first+i].position - samples_[first].position).transpose();
      }
      const Eigen::MatrixXd residual = p - a * a.colPivHouseholderQr().solve(p);
      if (residual.rowwise().norm().maxCoeff() > 0.03) {
        clear(); return false;
      }
    }
    if (confirmed_) return true;
    if (samples_.size() < 5) return false;
    // Require sustained travel on both sides, not individual velocity signs.
    for (size_t start=0; start+4<samples_.size(); ++start) {
    for (size_t pivot=start+2; pivot+2<samples_.size(); ++pivot) {
      const Eigen::Vector3d out = samples_[pivot].position - samples_[start].position;
      const Eigen::Vector3d back = position - samples_[pivot].position;
      if (out.norm() < 0.06 || back.norm() < 0.06 ||
          out.dot(back) > -0.5*out.norm()*back.norm()) continue;
      bool coherent = true;
      int outward_steps = 0, return_steps = 0;
      // Neither leg may be a single jump followed by small sampling jitter.
      double outward_distance=0, return_distance=0, outward_peak=0, return_peak=0;
      for (size_t i=start+1; i<=pivot; ++i) {
        const double step = std::max(0.0, (samples_[i].position-samples_[i-1].position).dot(out.normalized()));
        outward_distance += step; outward_peak = std::max(outward_peak,step);
        if ((samples_[i].position-samples_[i-1].position).dot(out.normalized()) > 0.005)
          ++outward_steps;
        if ((samples_[i].position-samples_[i-1].position).dot(out.normalized()) < -0.02)
          coherent = false;
      }
      for (size_t i=pivot+1; i<samples_.size(); ++i) {
        const double step = std::max(0.0, (samples_[i].position-samples_[i-1].position).dot(back.normalized()));
        return_distance += step; return_peak = std::max(return_peak,step);
        if ((samples_[i].position-samples_[i-1].position).dot(back.normalized()) > 0.005)
          ++return_steps;
        if ((samples_[i].position-samples_[i-1].position).dot(back.normalized()) < -0.02)
          coherent = false;
      }
      if (coherent && outward_steps >= 2 && return_steps >= 2 &&
          outward_peak <= 0.8*outward_distance && return_peak <= 0.8*return_distance) { confirmed_ = true; return true; }
    }
    }
    return false;
  }
 private:
  std::deque<Sample> samples_;
  bool confirmed_ = false;
};
}
