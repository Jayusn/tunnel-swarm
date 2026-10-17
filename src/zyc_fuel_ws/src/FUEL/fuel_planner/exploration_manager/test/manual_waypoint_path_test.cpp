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
  fast_planner::ManualWaypointPath route;
  require(route.finished() && route.path(start).empty());
  route.points = {first, second};
  auto path = route.path(start);
  require(path.size() == 2 && path.back() == first);
  // 预测起点已到达或重试，都不能提前消耗手填点。
  route.path(first);
  path = route.path(start);
  require(route.next == 0 && path.back() == first);
  route.updateOdometry(second, 0.25);
  require(route.next == 0);  // 必须按顺序到达。
  route.updateOdometry(first + Vector3d(0, 0, 0.3), 0.25);
  require(route.next == 0);  // 高度也属于坐标。
  route.updateOdometry(first + Vector3d(0.1, 0, 0), 0.25);
  require(route.next == 1);
  path = route.path(first);
  require(path.size() == 2 && path.back() == second);
  route.updateOdometry(second, 0.25);
  route.updateOdometry(start, 0.25);
  path = route.path(second);
  require(route.next == 2 && route.finished() && path.empty());
  // 重复点不生成零长度路径段；全部到达后不能追加其他目标。
  route.points = {first, first, second};
  route.next = 0;
  path = route.path(start);
  require(path.size() == 2 && path.back() == first);
  route.updateOdometry(first, 0.25);
  require(route.next == 2);
  require(route.path(second).size() == 1);
  route.updateOdometry(second, 0.25);
  require(route.next == 3 && route.finished() && route.path(start).empty());
  return 0;
}
