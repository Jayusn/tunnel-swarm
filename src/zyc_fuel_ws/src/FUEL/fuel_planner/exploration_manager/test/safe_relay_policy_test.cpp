#include <exploration_manager/safe_relay_policy.h>
#include <iostream>
#include <stdexcept>
using namespace fast_planner::safe_relay;
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
std::vector<Sample> straight(int last = 60) {
  std::vector<Sample> route;
  for (int i = 0; i <= last; ++i) {
    Sample s;
    s.position = Eigen::Vector3d(i * 0.1, 0.0, 0.8);
    s.progress = i * 0.1;
    s.clearance = 1.0;
    s.hold_free = s.confirmed = true;
    route.push_back(s);
  }
  return route;
}
int main() {
  Config cfg;
  auto pass = [](std::size_t, std::size_t) { return true; };
  auto route = straight(100);
  auto pick = select(route, 0, route.back().position, cfg, pass);
  require(pick.kind == Kind::NORMAL && pick.index == 10, "normal route must release at 1m progress");
  auto next = select(route, pick.index, route.back().position, cfg, pass);
  require(next.kind == Kind::NORMAL && next.index == 20, "normal route must continue at 1m spacing");
  require(route[pick.index].position.z() == 0.8, "keep measured height");

  route = straight();
  for (int i = 20; i <= 30; ++i) route[i].hold_free = route[i].confirmed = false;
  pick = select(route, 0, route.back().position, cfg, pass);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index == 31, "first qualified post-obstacle sample is eligible");
  const auto rejected_b = pick.index;
  pick = select(route, 0, route.back().position, cfg,
                [&](std::size_t, std::size_t b) { return b != rejected_b; });
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index != rejected_b, "B fails: try qualified C");
  pick = select(route, 0, route.back().position, cfg,
                [](std::size_t, std::size_t b) { return b < 20; });
  require(pick.kind == Kind::BEFORE_OBSTACLE && pick.index == 19,
          "B/C fail: release forward pre-obstacle waiting point");
  pick = select(route, 0, route.back().position, cfg,
                [](std::size_t, std::size_t) { return false; });
  require(pick.kind == Kind::WAIT, "all paths fail: do not fabricate endpoint");
  pick = select(route, 17, route.back().position, cfg,
                [](std::size_t, std::size_t b) { return b < 20; });
  require(pick.kind == Kind::WAIT, "waiting fallback must not command rollback or duplicate anchor");

  route = straight(19);
  require(select(route, 0, route.back().position, cfg, pass).kind == Kind::WAIT,
          "do not publish a point the leader has not cleared by 1m");
  route = straight(20);
  pick = select(route, 0, route.back().position, cfg, pass);
  require(pick.kind == Kind::NORMAL && pick.index == 10,
          "1m normal progress plus 1m leader clearance is sufficient");

  // B itself can be a confirmed stopping point even when both neighbours
  // are transit-only. Removing the safe-span gate must not relax B or A -> B.
  route = straight();
  for (auto& s : route) s.hold_free = s.confirmed = false;
  route[30].hold_free = route[30].confirmed = true;
  pick = select(route, 0, route.back().position, cfg, pass);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index == 30,
          "isolated qualified B must not require neighbouring stopping space");
  require(select(route, 0, route.back().position, cfg,
                 [](std::size_t, std::size_t) { return false; }).kind == Kind::WAIT,
          "isolated B still requires a passable connection");
  route[30].confirmed = false;
  require(select(route, 0, route.back().position, cfg, pass).kind == Kind::WAIT,
          "isolated B still requires map confirmation");
  route[30].confirmed = true;
  route[30].hold_free = false;
  require(select(route, 0, route.back().position, cfg, pass).kind == Kind::WAIT,
          "isolated B itself must have safe stopping space");
  route = straight();
  for (auto& s : route) s.confirmed = false;
  require(select(route, 0, route.back().position, cfg, pass).kind == Kind::WAIT,
          "single map confirmation is insufficient; no dwell is needed");
  for (auto& s : route) s.hold_free = false;
  require(select(route, 0, route.back().position, cfg, pass).kind == Kind::WAIT,
          "no qualified stopping space must produce no target");
  route = straight();
  for (int i = 12; i <= 40; ++i) route[i].hold_free = route[i].confirmed = false;
  pick = select(route, 0, route.back().position, cfg, pass);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index > 40,
          "unsafe 1.2m planning endpoint must never become stopping goal");
  // A bend uses route order, not world X or vehicle yaw.
  for (auto& s : route) s.position = Eigen::Vector3d(0.0, -s.progress, 1.1);
  pick = select(route, 0, route.back().position, cfg, pass);
  require(pick.kind == Kind::AFTER_OBSTACLE && pick.index > 40,
          "post-obstacle means along actual route, not world X");
  auto leader_near = route[pick.index].position;
  auto released = select(route, 0, leader_near, cfg, pass);
  require(released.kind == Kind::WAIT ||
              (route[released.index].position.head<2>() - leader_near.head<2>()).norm() >= 1.0 - 1e-6,
          "arc clearance alone is insufficient on a looping route");
  require(select({}, 0, Eigen::Vector3d::Zero(), cfg, pass).kind == Kind::WAIT,
          "empty actual route cannot produce predicted waypoints");
  std::cout << "PASS safe relay: normal progress, B/C/W/no-update, passage, confirmation, curves, XYZ\n";
}
