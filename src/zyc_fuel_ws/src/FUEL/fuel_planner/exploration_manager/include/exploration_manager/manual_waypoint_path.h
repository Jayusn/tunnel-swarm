#ifndef EXPLORATION_MANAGER_MANUAL_WAYPOINT_PATH_H
#define EXPLORATION_MANAGER_MANUAL_WAYPOINT_PATH_H

#include <Eigen/Core>
#include <vector>
#include <cstddef>
#include <limits>

namespace fast_planner {

// 截取已检查路径的弧长前缀；只改变本轮局部段，不消耗用户航点。
inline std::vector<Eigen::Vector3d> manualPathPrefix(
    const std::vector<Eigen::Vector3d>& path, double length) {
  if (path.empty()) return {};
  std::vector<Eigen::Vector3d> prefix{path.front()};
  for (std::size_t i = 1; i < path.size() && length > 1e-6; ++i) {
    const Eigen::Vector3d delta = path[i] - path[i - 1];
    const double distance = delta.norm();
    if (distance < 1e-6) continue;
    if (distance >= length) {
      prefix.push_back(path[i - 1] + delta * (length / distance));
      break;
    }
    prefix.push_back(path[i]);
    length -= distance;
  }
  return prefix;
}

// 只用实际里程计推进列表，规划起点和规划失败不能消耗途径点。
struct ManualWaypointPath {
  std::vector<Eigen::Vector3d> points;
  std::size_t next{0};

  void updateOdometry(const Eigen::Vector3d& position, double arrive_distance) {
    while (next < points.size() &&
           (position - points[next]).norm() <= arrive_distance)
      ++next;
  }

  bool finished() const { return next >= points.size(); }

  // 确认出口后，只保留到门内最近点为止的顺序；门外点全部跳过。
  // 最近点若已完成，不回飞。返回它在原列表中的下标，-1表示没有门内点。
  int truncateAtExit(const Eigen::Vector3d& portal, const Eigen::Vector2d& outward) {
    int nearest = -1;
    double distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < points.size(); ++i) {
      if ((points[i] - portal).head<2>().dot(outward) > 0.0) continue;
      const double candidate = (points[i] - portal).squaredNorm();
      if (candidate < distance) {
        nearest = static_cast<int>(i);
        distance = candidate;
      }
    }
    std::vector<Eigen::Vector3d> remaining;
    for (std::size_t i = next; nearest >= 0 && i <= static_cast<std::size_t>(nearest); ++i)
      if ((points[i] - portal).head<2>().dot(outward) <= 0.0)
        remaining.push_back(points[i]);
    points.resize(next);
    points.insert(points.end(), remaining.begin(), remaining.end());
    return nearest;
  }

  std::vector<Eigen::Vector3d> path(const Eigen::Vector3d& start) const {
    if (finished()) return {};
    const Eigen::Vector3d& target = points[next];
    std::vector<Eigen::Vector3d> result{start};
    if ((target - start).norm() >= 1e-3) result.push_back(target);
    return result;
  }
};

}  // namespace fast_planner
#endif
