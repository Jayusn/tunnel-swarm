#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <deque>
#include <string>

namespace diff_planner {

// Match individual observations to the leader body, never a merged passage.
class FappLeaderIdentity {
 public:
  double radius{0.262}, top{0.05}, bottom{0.20}, margin{0.08};
  struct Sample {
    double stamp;
    Eigen::Vector3d position;
    std::string frame;
  };

  void observe(const Sample& sample) {
    if (!std::isfinite(sample.stamp) || sample.stamp <= 0.0 ||
        !sample.position.allFinite() || sample.frame.empty()) return;
    auto it = std::lower_bound(samples_.begin(), samples_.end(), sample.stamp,
        [](const Sample& s, double t) { return s.stamp < t; });
    if (it != samples_.end() && it->stamp == sample.stamp) *it = sample;
    else samples_.insert(it, sample);
    while (!samples_.empty() && samples_.back().stamp - samples_.front().stamp > 10.0)
      samples_.pop_front();
  }

  bool positionAt(double stamp, Eigen::Vector3d& position, std::string& frame) const {
    if (!std::isfinite(stamp) || stamp <= 0.0 || samples_.empty()) return false;
    const auto after = std::lower_bound(samples_.begin(), samples_.end(), stamp,
        [](const Sample& s, double t) { return s.stamp < t; });
    // At most 50 ms from a real odometry sample; never use latest pose for an old detection.
    if (after == samples_.begin() || after == samples_.end() || after->stamp == stamp) {
      const auto& s = after == samples_.end() ? samples_.back() : *after;
      if (std::abs(s.stamp - stamp) > 0.05) return false;
      position = s.position; frame = s.frame;
      return true;
    }
    const auto before = std::prev(after);
    if (stamp - before->stamp > 0.05 || after->stamp - stamp > 0.05 ||
        before->frame != after->frame) return false;
    const double ratio = (stamp - before->stamp) / (after->stamp - before->stamp);
    position = before->position + ratio * (after->position - before->position);
    frame = before->frame;
    return true;
  }

  bool contains(const Eigen::Vector3d& center, const Eigen::Vector3d& size,
                const Eigen::Vector3d& leader) const {
    if (!center.allFinite() || !size.allFinite() || !leader.allFinite() ||
        (size.array() < 0.0).any() || !(size.array() > 0.0).any() ||
        !std::isfinite(radius) || radius <= 0.0 ||
        !std::isfinite(top) || top <= 0.0 ||
        !std::isfinite(bottom) || bottom <= 0.0 ||
        !std::isfinite(margin) || margin < 0.0) return false;
    const Eigen::Vector3d delta = center - leader;
    // Require all AABB corners inside the body cylinder, including height.
    // Nearby balls and boxes mixing leader + environment remain passage observations.
    const Eigen::Vector2d farthest = delta.head<2>().cwiseAbs() + 0.5 * size.head<2>();
    return farthest.norm() <= radius + margin &&
        delta.z() + 0.5 * size.z() <= top + margin &&
        delta.z() - 0.5 * size.z() >= -bottom - margin;
  }

 private:
  std::deque<Sample> samples_;
};
}  // namespace diff_planner
