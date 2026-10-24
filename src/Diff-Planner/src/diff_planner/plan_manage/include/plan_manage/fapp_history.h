#pragma once
#include <optimizer/dynamic_obstacle.h>
#include <algorithm>
#include <deque>
#include <vector>

namespace diff_planner {
// 同位置按几何关联；ID变化和空帧都不能清除尚未穿过的摆球范围。
class FappHistory {
 public:
  double duration{10.0}, association{0.6};
  void update(const std::vector<DynamicObstacleState>& observations, double now) {
    if (now < last_time_) { samples_.clear(); active_ = false; }
    last_time_ = now;
    while (!samples_.empty() && now - samples_.front().stamp > duration)
      samples_.pop_front();
    for (const auto& sample : observations) {
      if (sample.leader_body) continue;
      if (!sample.position.allFinite() || !sample.size.allFinite() ||
          (sample.size.array() < 0.0).any() || !std::isfinite(sample.stamp) ||
          now - sample.stamp > duration) continue;
      std::vector<double> heights;
      for (const auto& prior : samples_)
        if ((prior.position - sample.position).norm() <= association && prior.size.z() > 0.0)
          heights.push_back(prior.size.z());
      if (heights.size() >= 3) {
        std::sort(heights.begin(), heights.end());
        if (sample.size.z() > 2.0 * heights[heights.size()/2] + 0.05) continue;
      }
      samples_.push_back(sample);
      if (active_ && near(sample.position, watched_)) include(watched_, sample);
    }
  }
  std::vector<DynamicObstacleState> regions(double now) const {
    std::vector<DynamicObstacleState> result;
    if (active_) result.push_back(watched_);
    for (const auto& sample : samples_) {
      if (now - sample.stamp > duration) continue;
      auto found = std::find_if(result.begin(), result.end(), [&](const DynamicObstacleState& r) {
        return near(sample.position, r);
      });
      if (found == result.end()) result.push_back(sample);
      else include(*found, sample);
    }
    for (auto& region : result) { region.velocity.setZero(); region.stamp = now; }
    return result;
  }
  void watch(const DynamicObstacleState& region, const Eigen::Vector3d& start,
             const Eigen::Vector3d& end, double padding) {
    if (active_ || region.leader_body) return;
    direction_ = end - start; direction_.z() = 0.0;
    if (direction_.norm() < 0.05) return;
    direction_.normalize(); watched_ = region; padding_ = padding; active_ = true;
  }
  void observePosition(const Eigen::Vector3d& actual) {
    if (!active_ || !actual.allFinite()) return;
    const double far = watched_.position.dot(direction_) +
        0.5 * watched_.size.dot(direction_.cwiseAbs()) + padding_;
    if (actual.dot(direction_) > far) active_ = false;
  }
  bool active() const { return active_; }
 private:
  bool near(const Eigen::Vector3d& p, const DynamicObstacleState& region) const {
    return ((p-region.position).cwiseAbs() - 0.5*region.size).cwiseMax(0.0).norm() <= association;
  }
  static void include(DynamicObstacleState& r, const DynamicObstacleState& sample) {
    const Eigen::Vector3d lower = (r.position-0.5*r.size).cwiseMin(sample.position-0.5*sample.size);
    const Eigen::Vector3d upper = (r.position+0.5*r.size).cwiseMax(sample.position+0.5*sample.size);
    r.position = 0.5*(lower+upper); r.size = upper-lower;
  }
  std::deque<DynamicObstacleState> samples_;
  DynamicObstacleState watched_;
  Eigen::Vector3d direction_{Eigen::Vector3d::UnitX()};
  double last_time_{0.0}, padding_{0.0};
  bool active_{false};
};
} // namespace diff_planner
