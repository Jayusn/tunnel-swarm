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
  require(route.next == 1);  // Z仅作参考，XY到达就推进。
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
  // 最近门内点之后结束；即使门外点排在它前面，也不能继续执行门外点。
  const Vector3d portal(4, 0, 0.8), outside(4.1, 0, 0.8), nearest(3.8, 0, 0.8);
  route.points = {first, outside, second, nearest, Vector3d(2, 0, 0.8)};
  route.next = 1;
  require(route.truncateAtExit(portal, Eigen::Vector2d::UnitX()) == 3);
  require(route.points.size() == 3 && route.next == 1 && route.points[1] == second &&
          route.points.back() == nearest && !route.finished());
  route.updateOdometry(second, 0.25);
  require(route.path(second).back() == nearest);
  route.updateOdometry(nearest, 0.25);
  require(route.finished());
  // 最近点已经飞完时直接交接，不回头；没有门内点时也直接交接。
  route.points = {nearest, second, outside};
  route.next = 1;
  require(route.truncateAtExit(portal, Eigen::Vector2d::UnitX()) == 0 && route.finished());
  route.points = {outside}; route.next = 0;
  require(route.truncateAtExit(portal, Eigen::Vector2d::UnitX()) == -1 && route.finished());
  require(route.truncateAtExit(portal, Eigen::Vector2d::UnitX()) == -1);
  // 门内外按出口法向判断，不把世界X轴当出口朝向；最近距离包含XYZ。
  route.points = {{0, 3.9, 2.0}, {0, 3.7, 0.8}, {0, 4.1, 0.8}};
  route.next = 0;
  require(route.truncateAtExit(Vector3d(0, 4, 0.8), Eigen::Vector2d::UnitY()) == 1 &&
          route.points.size() == 2);
  return 0;
}
