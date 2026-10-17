#ifndef EXPLORATION_MANAGER_MANUAL_WAYPOINT_PATH_H
#define EXPLORATION_MANAGER_MANUAL_WAYPOINT_PATH_H

#include <Eigen/Core>
#include <vector>
#include <cstddef>

namespace fast_planner {

// 只用实际里程计推进列表，规划起点和规划失败不能消耗途径点。
struct ManualWaypointPath {
  std::vector<Eigen::Vector3d> points;
  std::size_t next{0};

  void updateOdometry(const Eigen::Vector3d& position, double arrive_distance) {
    while (next < points.size() &&
           (position - points[next]).norm() <= arrive_distance)
      ++next;
  }

  std::vector<Eigen::Vector3d> path(const Eigen::Vector3d& start,
                                   const Eigen::Vector3d& goal) const {
    const Eigen::Vector3d& target = next < points.size() ? points[next] : goal;
    std::vector<Eigen::Vector3d> result{start};
    if ((target - start).norm() >= 1e-3) result.push_back(target);
    return result;
  }
};

}  // namespace fast_planner
#endif
