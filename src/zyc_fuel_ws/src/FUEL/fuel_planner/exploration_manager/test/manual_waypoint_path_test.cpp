#include <exploration_manager/manual_waypoint_path.h>
#include <stdexcept>

namespace {
void require(bool condition) {
  if (!condition) throw std::runtime_error("manual waypoint path regression");
}
}

int main() {
  using Eigen::Vector3d;
  const Vector3d start(0, 0, 0.8), first(1, 0, 0.8), second(1, 1, 0.8);
  const Vector3d goal(2, 1, 0.8), changed_goal(3, 1, 0.8);
  fast_planner::ManualWaypointPath route;
  require(route.path(start, goal).size() == 2);
  route.points = {first, second};
  auto path = route.path(start, goal);
  require(path.size() == 2 && path.back() == first);
  // 预测起点已到达、重试或前沿重选，都不能提前消耗手填点。
  route.path(first, goal);
  path = route.path(start, changed_goal);
  require(route.next == 0 && path.back() == first);
  route.updateOdometry(second, 0.25);
  require(route.next == 0);  // 必须按顺序到达。
  route.updateOdometry(first + Vector3d(0, 0, 0.3), 0.25);
  require(route.next == 0);  // 高度也属于坐标。
  route.updateOdometry(first + Vector3d(0.1, 0, 0), 0.25);
  require(route.next == 1);
  path = route.path(first, goal);
  require(path.size() == 2 && path.back() == second);
  route.updateOdometry(second, 0.25);
  route.updateOdometry(start, 0.25);
  path = route.path(second, goal);
  require(route.next == 2 && path.size() == 2 && path.back() == goal);
  // 重复点与末点等于前沿目标时，不生成零长度路径段。
  route.points = {first, first, goal};
  route.next = 0;
  path = route.path(start, goal);
  require(path.size() == 2 && path.back() == first);
  route.updateOdometry(first, 0.25);
  require(route.next == 2);
  require(route.path(goal, goal).size() == 1);
  route.updateOdometry(goal, 0.25);
  require(route.next == 3 && route.path(goal, changed_goal).back() == changed_goal);
  return 0;
}
